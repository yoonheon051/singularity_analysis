import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, DurabilityPolicy
import roboticstoolbox as rtb
import numpy as np
import csv
import os
import tempfile
from datetime import datetime

# 결과 폴더: 실행 위치(현재 폴더)와 상관없이 항상 저장소의 results/ 에 저장
REPO_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_DIR = os.path.join(REPO_DIR, 'results')
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']   # M0609 URDF의 관절 이름
# Jacobian을 계산할 말단 링크. M0609 URDF에는 끝이 열린 링크가 두 개(높이 표시용 'base', 말단 'tool0') 있어서,
# 지정하지 않으면 Toolbox가 그중 첫 번째(=base)를 말단으로 골라 엉뚱한 Jacobian이 나올 수 있다.
# link_6 은 tool0 과 원점이 같아 jacob0 결과가 같고, 그리퍼가 달린 URDF에도 항상 있다.
END_LINK = 'link_6'

class M0609SingularityMonitor(Node):
    '''
    M0609의 특이점(Singularity) 및 가동성(Manipulability)을 실시간으로 모니터링하고, 
    데이터를 CSV로 기록하는 ROS2노드 클래스
    '''
    def __init__(self):
        # ROS2 노드 이름 초기화
        super().__init__('m0609_singularity_monitor')
        
        # 1. 데이터 저장 설정
        # 분석 결과를 저장할 CSV 파일 경로 설정 및 결과 폴더 자동 생성 
        self.results_path = os.path.join(RESULTS_DIR, 'analysis_log.csv')
        os.makedirs(RESULTS_DIR, exist_ok=True)
        self.init_csv()
        self._missing_warned = False   # 관절 이름이 안 맞을 때 경고를 한 번만 출력


        # 2. 로봇 모델 정보(URDF) 수신을 위한 설정
        # Transient Local: 노드가 늦게 켜져도 이전에 발행된 토픽 데이터를 받아올 수 있도록 설정(Latch 방식)
        self.get_logger().info('URDF 데이터를 /robot_description 토픽에서 기다리는 중...')
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        self.urdf_xml = None
        # /robot_description 토픽을 통해 로봇의 구조 정보(URDF)를 수신
        self.subscription = self.create_subscription(String, '/robot_description', self.urdf_callback, qos_profile)
        
        # URDF 데이터를 수신할 때까지 노드 실행을 잠시 대기(모델이 있어야 계산 가능)
        while self.urdf_xml is None:
            rclpy.spin_once(self)

        # 3. 수신된 URDF를 기반으로 수학적 로봇 모델 생성    
        try:
            # 수신된 XML 문자열을 파일 형태로 저장해야 Toolbox에서 로드 가능(임시 파일 활용)
            with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as tmp:
                tmp.write(self.urdf_xml)
                tmp_path = tmp.name

            # 기본 방식: Toolbox의 표준 URDF 로더 사용 시도
            try:
                self.robot = rtb.ERobot.URDF(tmp_path)
                self.get_logger().info('ERobot 방식으로 모델 로드 성공')
            except Exception as e1:
                # 라이브러리 버전이나 환경에 따른 대안 로드 방식(추출 로직 포함)
                self.get_logger().warn(f'ERobot 방식 실패, 대안 시도... ({str(e1)})')
                
                import roboticstoolbox.models.URDF as urdf_mod
                if hasattr(urdf_mod, 'URDF'):
                    full_robot = urdf_mod.URDF().parse_filepath(tmp_path).to_robot()
                else:
                    full_robot = rtb.models.URDF(tmp_path)
                
                # M0609에 그리퍼 등이 달린 경우, 본체까지(link_6까지)만 분리하여 모델 구축
                try:
                    self.robot = full_robot.extract_robot(full_robot.links[6])
                    self.get_logger().info('full_robot에서 6축 추출 성공')
                except:
                    self.robot = full_robot
                    self.get_logger().info('추출 실패로 전체 모델 사용')

            # 사용이 끝난 임시 URDF 파일은 삭제하여 시스템 자원 확보
            if os.path.exists(tmp_path):
                os.remove(tmp_path)
                
        except Exception as e:
            # 모델이 없으면 분석을 할 수 없으므로, 아무것도 안 하는 노드로 남지 않고 오류로 종료
            self.get_logger().error(f'최종 모델 로드 실패: {str(e)}')
            raise RuntimeError(f'로봇 모델을 만들지 못했습니다: {e}') from e

        # 모델이 인식하는 로봇의 자유도(DoF, Degree of Freedom) 저장(6축 vs. 12축 등 구분용)
        self.dof = self.robot.n

        # 말단 링크를 명시 (없으면 Toolbox 기본값 사용)
        self.end = END_LINK if END_LINK in self.robot.link_dict else None
        if self.end is None:
            self.get_logger().warn(f"'{END_LINK}' 링크가 없어 Toolbox 기본 말단 링크를 사용합니다: {[l.name for l in self.robot.ee_links]}")
        
        # 실시간 관절 각도 데이터를 수신하기 위해 /joint_states 토픽 구독
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.get_logger().info(f'모니터링 시작! (DOF: {self.dof})')

    def init_csv(self):
        # CSV 파일의 헤더(열 이름)를 정의하고 생성
        if not os.path.exists(self.results_path):
            with open(self.results_path, 'w', newline='') as f:
                writer = csv.writer(f)
                # 기본 정보: 시간, 관절각(6개), 가동성 지수, 조건수
                header = ['timestamp', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'manipulability', 'condition_number']
                # 상세 분석을 위해 6x6 Jacobian 행렬의 각 원소(J00~J55)를 열로 추가
                for i in range(6):
                    for j in range(6):
                        header.append(f'J_{i}{j}')
                writer.writerow(header)

    def urdf_callback(self, msg):
        # 로봇 구조 정보 수신 시 호출되는 콜백 함수
        self.urdf_xml = msg.data
        self.get_logger().info('URDF 데이터를 성공적으로 수신했습니다.')

    def joint_callback(self, msg):
        # 실시간 관절 상태(/joint_states) 수신 시 호출되어 분석을 수행하는 핵심 루프
        try:
            # 1. 수신된 조인트 이름과 위치값을 매핑하여 딕셔너리화(순서 혼선 방지)
            joint_map = {name: pos for name, pos in zip(msg.name, msg.position)}

            # 2. 모델 설정에 맞춰 분석할 관절 각도 추출
            if self.dof == 6:
                # 순수 6축 모델일 경우 정의된 6개 조인트만 사용
                q = [joint_map[name] for name in JOINT_NAMES if name in joint_map]
            else:
                # 그리퍼 등이 포함된 모델일 경우 메시지 앞쪽부터 모델 자유도만큼 사용 (본체 6축이 앞에 있다고 가정)
                q = list(msg.position[:self.dof])

            # 이름이 달라 관절을 못 찾으면 아무 기록도 안 남기고 조용히 넘어가므로, 한 번은 알려 준다
            if len(q) != self.dof and not self._missing_warned:
                self._missing_warned = True
                self.get_logger().warn(
                    f'/joint_states 에서 관절 {self.dof}개를 찾지 못해 분석을 건너뜁니다. '
                    f'받은 이름: {list(msg.name)} / 기대한 이름: {JOINT_NAMES}')

            # 3. 데이터 개수가 모델 DoF와 일치할 때만 수학적 연산 진행
            if len(q) == self.dof:
                q_np = np.array(q)
                # 기구학 지표 계산 함수 호출
                J, w, cond = self.calculate_metrics(q_np)
                
                # 계산 결과 실시간 로그 출력
                self.get_logger().info(f'Manipulability: {w:.4f} | Condition: {cond:.2f}')
                # 분석 결과 파일 저장
                self.save_to_csv(q_np, J, w, cond)
                
        except Exception as e:
            self.get_logger().error(f'데이터 처리 중 오류 발생: {str(e)}')

    def calculate_metrics(self, q):
        '''
        Jacobian 기반의 기구학적 성능 지표를 계산하는 함수
        - Jacobian(J): 관절 속도와 말단 속도 사이의 선형 매핑 행렬
        - Manipuability(w): 로봇이 특정 자세에서 모든 방향으로 얼마나 자유롭게 움직일 수 있는가(0에 가까우면 특이점)
        - Condition Number: Jacobian 행렬의 수치적 안정성(1에 가까울수록 이상적, 클수록 특이점 근처)
        '''

        # 1. 기하학적 Jacobian 계산 (Geometrical Jacobian)
        J_full = self.robot.jacob0(q, end=self.end)

        # 2. 본체 분석을 위해 6x6 정방 행렬(Square Matrix)로 슬라이싱
        # 모델이 12DoF더라도 본체 6축 제어 성능 분석을 위해 앞의 6열만 사용
        J = J_full[:, :6]

        # 3. 가동성 지수 계산(Yoshikawa's index): w = sqrt(det(J·Jᵀ))
        # 조건수와 같은 6x6 J로 계산해야 두 지표가 같은 대상을 본다 (robot.manipulability(q)는 말단·열을 따로 고름)
        w = float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))

        # 4. 행렬 수치 안정성(조건수) 계산
        # 특이값 분해(SVD)로 계산하므로 특이점 근처에서도 실제 크기(수천~수조)를 그대로 기록한다.
        # (예전처럼 det 기준으로 inf 를 넣으면 로그 그래프에서 그 점이 사라져, 가장 중요한 특이점 구간이 안 보였음)
        # 완전히 특이(최소 특이값 0)하면 numpy 가 inf 를 돌려준다.
        try:
            cond = float(np.linalg.cond(J))  # Jacobian의 최대 특이값 / 최소 특이값
        except Exception:
            cond = float('inf')
            
        return J, w, cond

    def save_to_csv(self, q, J, w, cond):
        # 계산된 모든 데이터를 CSV 파일의 한 줄(ROW)로 기록
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]

        # 본체 6개 관절 각도만 기록
        q_save = list(q[:6])
    
        # 6x6 Jacobian 행렬을 1차원 리스트로 직렬화(36개의 원소)
        j_flat = J.flatten().tolist()
        
        # 최종 데이터 행 구성: [시간, q1~q6, 가동성, 조건수, J00~J55]
        row = [timestamp] + q_save + [w, cond] + j_flat
        
        with open(self.results_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)

def main(args=None):
    # ROS2 노드 실행 및 종료를 관리하는 메인 엔트리 포인트
    rclpy.init(args=args) # ROS2 통신 라이브러리 초기화
    node = M0609SingularityMonitor() # 모니터링 노드 인스턴스 생성 
    try:
        rclpy.spin(node) # 노드가 종료될 때까지 반복 실행(이벤트 대기)
    except KeyboardInterrupt:
        pass # 사용자가 Ctrl+C 입력 시 안전하게 종료될 수 있도록 종료 절차 시행
    finally:
        node.destroy_node() # 노드 자원 해제
        rclpy.shutdown() # ROS2 통신 종료

if __name__ == '__main__':
    main()