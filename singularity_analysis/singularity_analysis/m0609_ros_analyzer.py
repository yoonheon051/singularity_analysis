import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import csv
import os
from datetime import datetime

try:                                  # ros2 run / launch 로 실행할 때 (패키지)
    from . import singularity as sg   # 공통 계산 (타원체, 특이점 종류, URDF 정리)
    from .calculate_jacobian import load_robot_from_xml
except ImportError:                   # python3 파일경로로 직접 실행할 때
    import singularity as sg
    from calculate_jacobian import load_robot_from_xml

JOINT_NAMES = sg.JOINT_NAMES   # M0609 URDF의 관절 이름 (joint_1 ~ joint_6)
# Jacobian을 계산할 말단 링크. M0609 URDF에는 끝이 열린 링크가 두 개(높이 표시용 'base', 말단 'tool0') 있어서,
# 지정하지 않으면 Toolbox가 그중 첫 번째(=base)를 말단으로 골라 엉뚱한 Jacobian이 나올 수 있다.
# link_6 은 tool0 과 원점이 같아 jacob0 결과가 같고, 그리퍼가 달린 URDF에도 항상 있다.
END_LINK = sg.END_LINK

# CSV 열: 시간, 관절각 6개, 가동성, 조건수, 특이점 종류·지표, 6x6 Jacobian 36개
CSV_HEADER = (['timestamp', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'manipulability', 'condition_number',
               'singularity_type', 'sigma_min', 'wrist_idx', 'elbow_idx', 'shoulder_idx']
              + [f'J_{i}{j}' for i in range(6) for j in range(6)])

ELLIPSOID_TOPIC = '/manipulability_ellipsoid'   # RViz에서 MarkerArray 로 추가해서 봄
ELLIPSOID_SCALE = 0.3                           # 타원체 반지름(m/s 또는 rad/s per rad/s)을 RViz에서 몇 m 로 그릴지

# 지표 토픽: rqt_plot 등으로 실시간 그래프를 볼 수 있다 (예: rqt_plot /singularity/manipulability/data)
METRIC_TOPICS = {
    'manipulability': '/singularity/manipulability',
    'condition_number': '/singularity/condition_number',
    'sigma_min': '/singularity/sigma_min',
}
TYPE_TOPIC = '/singularity/type'   # 'normal' / 'wrist' / 'elbow' / 'shoulder' / 'wrist+elbow' ...
CSV_FLUSH_SEC = 1.0                # CSV 를 디스크에 내보내는 간격 (매 메시지마다 파일을 열고 닫지 않음)

class M0609SingularityMonitor(Node):
    '''
    M0609의 특이점(Singularity) 및 가동성(Manipulability)을 실시간으로 모니터링하고, 
    데이터를 CSV로 기록하는 ROS2노드 클래스
    '''
    def __init__(self):
        # ROS2 노드 이름 초기화
        super().__init__('m0609_singularity_monitor')

        # 1. 데이터 저장 설정
        # 실행할 때마다 새 파일(analysis_날짜_시각.csv)에 기록 → 실행끼리 섞이지 않고 비교하기 쉬움
        # 저장 폴더: 파라미터 results_dir (기본: 환경변수 SINGULARITY_RESULTS_DIR 또는 ~/singularity_results)
        results_dir = self.declare_parameter('results_dir', sg.results_dir()).value
        results_dir = os.path.expanduser(results_dir)
        os.makedirs(results_dir, exist_ok=True)
        self.results_path = os.path.join(results_dir, datetime.now().strftime('analysis_%Y%m%d_%H%M%S.csv'))
        self.csv_file = None   # 모델을 만든 뒤에 연다 (실패하면 빈 파일이 남지 않게)
        self._missing_warned = False   # 관절 이름이 안 맞을 때 경고를 한 번만 출력
        self._last_type = 'normal'     # 특이점 종류가 바뀔 때만 알리기 위함

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
        # 그리퍼(RG2)가 달린 URDF여도 base_link → link_6 팔 사슬만 남겨서 만든다 (calculate_jacobian.load_robot_from_xml)
        # → 모델은 항상 6축이고, 관절값은 이름(joint_1~6)으로 찾으므로 /joint_states 의 순서·그리퍼 관절과 무관
        try:
            self.robot = load_robot_from_xml(self.urdf_xml)
            self.get_logger().info('로봇 모델 생성 성공 (팔 6축: base_link → link_6)')
        except Exception as e:
            # 모델이 없으면 분석을 할 수 없으므로, 아무것도 안 하는 노드로 남지 않고 오류로 종료
            self.get_logger().error(f'모델 로드 실패: {str(e)}')
            raise RuntimeError(f'로봇 모델을 만들지 못했습니다: {e}') from e

        # 모델이 인식하는 로봇의 자유도(DoF). 팔만 남겼으므로 6이어야 한다
        self.dof = self.robot.n
        if self.dof != 6:
            self.get_logger().warn(f'팔 모델의 관절 수가 6이 아닙니다 (DOF: {self.dof}). 관절 이름을 확인하세요.')

        # 말단 링크·손목 중심 링크를 명시 (특이점 종류 분류와 분해 타원체에 필요)
        self.end = END_LINK
        self.wrist = sg.WRIST_LINK if sg.WRIST_LINK in self.robot.link_dict else None
        if self.wrist is None:
            self.get_logger().warn(f"'{sg.WRIST_LINK}' 링크가 없어 특이점 종류 분류는 하지 않습니다.")
        self.frame_id = sg.root_link(self.urdf_xml)   # RViz 마커 좌표계 (보통 base_link)

        # RViz 시각화: 가동성 타원체 (Add → By topic → /manipulability_ellipsoid → MarkerArray)
        self.marker_pub = self.create_publisher(MarkerArray, ELLIPSOID_TOPIC, 10)

        # 지표 토픽 (rqt_plot 실시간 그래프, 다른 노드에서 경고에 활용)
        self.metric_pubs = {k: self.create_publisher(Float64, topic, 10) for k, topic in METRIC_TOPICS.items()}
        self.type_pub = self.create_publisher(String, TYPE_TOPIC, 10)

        # CSV 를 열고, 주기적으로 디스크에 내보냄 (끌 때는 destroy_node 에서 마지막으로 내보내고 닫음)
        self.init_csv()
        self.get_logger().info(f'기록 파일: {self.results_path}')
        self.create_timer(CSV_FLUSH_SEC, self.flush_csv)

        # 실시간 관절 각도 데이터를 수신하기 위해 /joint_states 토픽 구독
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.get_logger().info(f'모니터링 시작! (DOF: {self.dof})')

    def init_csv(self):
        # 이번 실행용 CSV 파일을 열어 두고 헤더(열 이름)를 씀.
        # /joint_states 는 초당 수십 번 들어오므로 매번 파일을 열고 닫지 않고, flush_csv 가 주기적으로 디스크에 내보낸다.
        self.csv_file = open(self.results_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(CSV_HEADER)
        self.csv_file.flush()

    def flush_csv(self):
        if self.csv_file is not None and not self.csv_file.closed:
            self.csv_file.flush()

    def destroy_node(self):
        # 종료 시 남은 기록을 내보내고 파일을 닫음
        if getattr(self, 'csv_file', None) is not None and not self.csv_file.closed:
            self.csv_file.close()
        super().destroy_node()

    def urdf_callback(self, msg):
        # 로봇 구조 정보 수신 시 호출되는 콜백 함수
        self.urdf_xml = msg.data
        self.get_logger().info('URDF 데이터를 성공적으로 수신했습니다.')

    def joint_callback(self, msg):
        # 실시간 관절 상태(/joint_states) 수신 시 호출되어 분석을 수행하는 핵심 루프
        try:
            # 1. 수신된 조인트 이름과 위치값을 매핑하여 딕셔너리화(순서 혼선 방지)
            joint_map = {name: pos for name, pos in zip(msg.name, msg.position)}

            # 2. 팔 관절 6개를 이름으로 추출 (그리퍼 관절이 섞여 있거나 순서가 달라도 됨)
            q = [joint_map[name] for name in JOINT_NAMES if name in joint_map]

            # 이름이 달라 관절을 못 찾으면 아무 기록도 안 남기고 조용히 넘어가므로, 한 번은 알려 준다
            if len(q) != self.dof and not self._missing_warned:
                self._missing_warned = True
                self.get_logger().warn(
                    f'/joint_states 에서 관절 {self.dof}개를 찾지 못해 분석을 건너뜁니다. '
                    f'받은 이름: {list(msg.name)} / 기대한 이름: {JOINT_NAMES}')

            # 3. 데이터 개수가 모델 DoF와 일치할 때만 수학적 연산 진행
            if len(q) == self.dof:
                q_np = np.array(q)
                # 기구학 지표 계산 (singularity.py):
                # - Jacobian(J): 관절 속도와 말단 속도 사이의 선형 매핑 행렬 (말단 link_6, 6x6)
                # - Manipulability(w) = sqrt(det(J·Jᵀ)): 0에 가까우면 특이점
                # - Condition Number = σmax/σmin (SVD): 특이점 근처에서도 실제 크기를 그대로 기록 (완전 특이면 inf)
                # - 특이점 종류: 손목·팔꿈치·어깨 지표 (0에 가까울수록 그 특이점에 가까움)
                a = sg.analyze(self.robot, q_np, end=self.end, wrist=self.wrist)
                kind = sg.label(a['types'])

                # 계산 결과 로그: 매 메시지(수십 Hz)마다 찍으면 터미널이 넘치므로 1초에 한 번
                self.get_logger().info(
                    f"Manipulability: {a['manipulability']:.4f} | Condition: {a['condition_number']:.2f} | {kind}",
                    throttle_duration_sec=1.0)
                # 특이점 종류가 바뀌는 순간은 바로 알림
                if kind != self._last_type:
                    if kind == 'normal':
                        self.get_logger().info('특이점 영역을 벗어났습니다.')
                    else:
                        self.get_logger().warn(f'특이점 근처 진입: {kind} (지표: {self._fmt_indices(a)})')
                    self._last_type = kind

                # 분석 결과 파일 저장 + 지표 토픽 + RViz 타원체
                self.save_to_csv(q_np, a, kind)
                self.publish_metrics(a, kind)
                self.publish_ellipsoids(q_np, a)

        except Exception as e:
            self.get_logger().error(f'데이터 처리 중 오류 발생: {str(e)}')

    @staticmethod
    def _fmt_indices(a):
        return 'n/a' if a['indices'] is None else ', '.join(f'{k}={v:.3f}' for k, v in a['indices'].items())

    def save_to_csv(self, q, a, kind):
        # 계산된 모든 데이터를 CSV 파일의 한 줄(ROW)로 기록
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]

        # 본체 6개 관절 각도만 기록
        q_save = list(q[:6])

        idx = a['indices'] or {}
        extra = [kind, a['sigma_min'], idx.get('wrist', ''), idx.get('elbow', ''), idx.get('shoulder', '')]

        # 6x6 Jacobian 행렬을 1차원 리스트로 직렬화(36개의 원소)
        j_flat = a['J'].flatten().tolist()

        # 최종 데이터 행 구성: CSV_HEADER 순서
        row = [timestamp] + q_save + [a['manipulability'], a['condition_number']] + extra + j_flat

        self.csv_writer.writerow(row)   # 디스크에는 flush_csv 가 1초마다 내보냄

    def publish_metrics(self, a, kind):
        """지표를 토픽으로 발행 (rqt_plot /singularity/manipulability/data 등으로 실시간 그래프)"""
        for key, pub in self.metric_pubs.items():
            msg = Float64()
            msg.data = float(a[key])
            pub.publish(msg)
        msg = String()
        msg.data = kind
        self.type_pub.publish(msg)

    def publish_ellipsoids(self, q, a):
        """RViz용 가동성 타원체. 손목 중심에 두 개를 그린다 (plot_ellipsoid.py 와 같은 분해).
        - ns 'arm':   팔(1~3축) 선속도 타원체 → 팔꿈치·어깨 특이점에서 납작, 빨강
        - ns 'wrist': 손목(4~6축) 각속도 타원체 → 손목 특이점에서 납작, 빨강
        손목 중심 링크가 없으면 말단의 선속도 타원체 하나만 그린다."""
        def pos(link):
            T = self.robot.fkine(q, end=link)
            return np.asarray(T.A if hasattr(T, 'A') else T, dtype=float)[:3, 3]

        if self.wrist is not None:
            center = pos(self.wrist)
            items = [('arm', a['ellipsoid_arm'], bool({'elbow', 'shoulder'} & set(a['types']))),
                     ('wrist', a['ellipsoid_wrist'], 'wrist' in a['types'])]
        else:
            center = pos(self.end)
            items = [('linear', a['ellipsoid_linear'], False)]

        msg = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, (ns, (U, s), singular) in enumerate(items):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = stamp
            m.ns, m.id = ns, i
            m.type, m.action = Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in center)
            qx, qy, qz, qw = sg.rotation_to_quaternion(U)   # 타원체 축 방향 = U 의 열
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = qx, qy, qz, qw
            # SPHERE 의 scale 은 지름. 너무 얇으면 안 보이므로 최소 5 mm
            m.scale.x, m.scale.y, m.scale.z = (max(2 * float(r) * ELLIPSOID_SCALE, 0.005) for r in s)
            m.color.r, m.color.g, m.color.b = (0.9, 0.2, 0.2) if singular else (0.2, 0.8, 0.3)
            m.color.a = 0.45
            msg.markers.append(m)
        self.marker_pub.publish(msg)

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