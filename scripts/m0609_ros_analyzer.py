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
        self.results_path = 'results/analysis_log.csv'
        os.makedirs('results', exist_ok=True)
        self.init_csv()


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
                self.get_logger().info(f'ERobot 방식으로 모델 로드 성공')
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
            self.get_logger().error(f'최종 모델 로드 실패: {str(e)}')
            return

        # 모델이 인식하는 로봇의 자유도(DoF, Degree of Freedom) 저장(6축 vs. 12축 등 구분용)
        self.dof = self.robot.n
        
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
            target_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            
            # 2. 모델 설정에 맞춰 분석할 관절 각도 추출
            if self.dof == 6:
                # 순수 6축 모델일 경우 정의된 6개 조인트만 사용
                q = [joint_map[name] for name in target_names if name in joint_map]
            else:
                # 그리퍼 등이 포함된 모델일 경우 정의된 6개 조인트만 사용 
                q = list(msg.position[:self.dof])

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
        J_full = self.robot.jacob0(q) 

        # 2. 본체 분석을 위해 6x6 정방 행렬(Square Matrix)로 슬라이싱
        # 모델이 12DoF더라도 본체 6축 제어 성능 분석을 위해 앞의 6열만 사용
        J = J_full[:, :6] 

        # 3. 가동성 지수 계산(Yoshikawa's index)
        w = self.robot.manipulability(q)

        # 4. 행렬 수치 안정성(조건수) 계산
        try:
            # 행렬식(Determinant)이 0이면 특이점이므로 계산 전 확인
            det_J = np.linalg.det(J)
            if np.abs(det_J) > 1e-6:
                cond = np.linalg.cond(J) # Jacobian의 최대 특이값 / 최소 특이값
            else:
                cond = float('inf') # 특이점 발생 시 조건 수는 무한대
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