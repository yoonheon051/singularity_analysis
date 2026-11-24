"""
M0609 실시간 특이점 분석 노드 (ros2 run singularity_analysis analyzer)

/joint_states 를 받을 때마다 Jacobian·지표·특이점 종류를 계산해서
- CSV 에 한 줄씩 기록 (실행마다 새 파일)
- 지표 토픽 /singularity/* 와 RViz 가동성 타원체 /manipulability_ellipsoid 발행
- 대시보드용 종합 상태 /singularity/status (JSON, status_rate_hz 로 빈도 제한) 발행
로봇에는 아무 명령도 보내지 않는다.
"""
import csv
import json
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from visualization_msgs.msg import Marker, MarkerArray

try:                                  # ros2 run / launch 로 실행할 때 (패키지)
    from . import robot_model as rm
    from . import singularity as sg
    from . import status as st
except ImportError:                   # python3 파일경로로 직접 실행할 때
    import robot_model as rm
    import singularity as sg
    import status as st

# CSV 열: 시간, 관절각 6개, 가동성, 조건수, 특이점 종류·지표, 6x6 Jacobian 36개
CSV_HEADER = (['timestamp', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'manipulability', 'condition_number',
               'singularity_type', 'sigma_min', 'wrist_idx', 'elbow_idx', 'shoulder_idx']
              + [f'J_{i}{j}' for i in range(6) for j in range(6)])
CSV_FLUSH_SEC = 1.0    # CSV 를 디스크에 내보내는 간격 (/joint_states 가 수십 Hz 라 매번 열고 닫지 않음)

ELLIPSOID_TOPIC = '/manipulability_ellipsoid'   # RViz 에서 MarkerArray 로 추가해서 봄
ELLIPSOID_SCALE = 0.3                           # 타원체 반지름(m/s 또는 rad/s per rad/s)을 RViz 에서 몇 m 로 그릴지

# 지표 토픽: rqt_plot 등으로 실시간 그래프 (예: rqt_plot /singularity/manipulability/data)
METRIC_TOPICS = {
    'manipulability': '/singularity/manipulability',
    'condition_number': '/singularity/condition_number',
    'sigma_min': '/singularity/sigma_min',
}
TYPE_TOPIC = '/singularity/type'       # 'normal' / 'wrist' / 'elbow' / 'shoulder' / 'wrist+elbow' ...
STATUS_TOPIC = '/singularity/status'   # 대시보드용 종합 상태 (std_msgs/String 에 JSON)

QDOT_SMOOTHING = 0.3   # 관절 속도를 차분으로 추정할 때의 지수 평활 계수 (/joint_states 에 velocity 가 없을 때)
LOG_EVERY_SEC = 1.0    # 지표 로그·처리 오류 로그를 이 간격보다 자주 찍지 않음


class M0609SingularityMonitor(Node):
    """M0609 의 특이점·가동성을 실시간으로 분석하고 CSV·토픽·RViz·대시보드로 내보내는 노드"""

    def __init__(self):
        super().__init__('m0609_singularity_monitor')

        # 파라미터 (launch 가 넣어 줌. 두산 드라이버는 보통 joint_states_topic=/dsr01/joint_states)
        results_dir = os.path.expanduser(self.declare_parameter('results_dir', rm.results_dir()).value)
        self.joint_states_topic = self.declare_parameter('joint_states_topic', '/joint_states').value
        description_topic = self.declare_parameter('robot_description_topic', '/robot_description').value
        urdf_file = self.declare_parameter('urdf_file', '').value
        rate_hz = float(self.declare_parameter('status_rate_hz', 15.0).value)
        self.status_period = 0.0 if rate_hz <= 0 else 1.0 / rate_hz
        self.warn_factor = float(self.declare_parameter('warn_factor', st.WARN_FACTOR).value)

        # 실행할 때마다 새 파일(analysis_날짜_시각.csv) → 실행끼리 섞이지 않음. 모델을 만든 뒤에 연다 (실패하면 빈 파일이 안 남게)
        os.makedirs(results_dir, exist_ok=True)
        self.results_path = os.path.join(results_dir, datetime.now().strftime('analysis_%Y%m%d_%H%M%S.csv'))
        self.csv_file = None

        self._missing_warned = False   # 관절 이름이 안 맞을 때 경고를 한 번만
        self._last_type = 'normal'     # 특이점 종류가 바뀔 때만 알리기 위함
        self._last_status = None       # 마지막으로 상태를 발행한 시각 (time.monotonic)
        self._prev_q = None            # 관절 속도 추정용 (직전 관절각, 시각)
        self._qdot = None

        self.urdf_xml = self._wait_for_urdf(urdf_file, description_topic)
        self._build_model()
        self._create_publishers()

        self.init_csv()
        self.get_logger().info(f'기록 파일: {self.results_path}')
        self.create_timer(CSV_FLUSH_SEC, self.flush_csv)

        self.create_subscription(JointState, self.joint_states_topic, self.joint_callback, 10)
        self.get_logger().info(f'모니터링 시작! (DOF: {self.dof}, 관절 토픽: {self.joint_states_topic})')

    # ------------------------------------------------------------------ 초기화

    def _wait_for_urdf(self, urdf_file, description_topic):
        """URDF 문자열을 얻는다. urdf_file 이 있으면 파일에서(실제 로봇), 없으면 토픽에서 받을 때까지 기다림"""
        if urdf_file:
            path = os.path.expanduser(urdf_file)
            if not os.path.isfile(path):
                self.get_logger().error(f'URDF 파일이 없습니다: {path}')
                raise RuntimeError(f'URDF 파일이 없습니다: {path}')
            self.get_logger().info(f'URDF 파일에서 모델을 읽습니다: {path}')
            return rm.read_urdf(path)

        self.urdf_xml = None
        self.get_logger().info(f'URDF 데이터를 {description_topic} 토픽에서 기다리는 중...')
        # Transient Local: 노드가 늦게 켜져도 이미 발행된 URDF 를 받음 (latch)
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.subscription = self.create_subscription(String, description_topic, self.urdf_callback, qos)
        while self.urdf_xml is None:
            rclpy.spin_once(self)
        return self.urdf_xml

    def urdf_callback(self, msg):
        self.urdf_xml = msg.data
        self.get_logger().info('URDF 데이터를 성공적으로 수신했습니다.')

    def _build_model(self):
        """URDF → Toolbox 모델 (그리퍼가 달려도 base_link → link_6 팔 6축만). 관절값은 이름으로 찾으므로 순서와 무관"""
        try:
            self.robot = rm.load_robot_from_xml(self.urdf_xml)
        except Exception as e:
            # 모델이 없으면 분석을 할 수 없으므로, 아무것도 안 하는 노드로 남지 않고 오류로 종료
            self.get_logger().error(f'모델 로드 실패: {e}')
            raise RuntimeError(f'로봇 모델을 만들지 못했습니다: {e}') from e
        self.get_logger().info('로봇 모델 생성 성공 (팔 6축: base_link → link_6)')

        self.dof = self.robot.n
        if self.dof != 6:
            self.get_logger().warn(f'팔 모델의 관절 수가 6이 아닙니다 (DOF: {self.dof}). 관절 이름을 확인하세요.')
        self.end = sg.END_LINK
        self.wrist = sg.WRIST_LINK if sg.WRIST_LINK in self.robot.link_dict else None
        if self.wrist is None:
            self.get_logger().warn(f"'{sg.WRIST_LINK}' 링크가 없어 특이점 종류 분류는 하지 않습니다.")
        self.frame_id = rm.root_link(self.urdf_xml)   # RViz 마커 좌표계 (보통 base_link)

    def _create_publishers(self):
        self.marker_pub = self.create_publisher(MarkerArray, ELLIPSOID_TOPIC, 10)
        self.metric_pubs = {k: self.create_publisher(Float64, topic, 10) for k, topic in METRIC_TOPICS.items()}
        self.type_pub = self.create_publisher(String, TYPE_TOPIC, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)

    # ------------------------------------------------------------------ CSV

    def init_csv(self):
        self.csv_file = open(self.results_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(CSV_HEADER)
        self.csv_file.flush()

    def flush_csv(self):
        if self.csv_file is not None and not self.csv_file.closed:
            self.csv_file.flush()

    def destroy_node(self):
        if getattr(self, 'csv_file', None) is not None and not self.csv_file.closed:
            self.csv_file.close()   # 남은 기록을 내보내고 닫음
        super().destroy_node()

    def save_to_csv(self, q, a, kind):
        idx = a['indices'] or {}
        row = ([datetime.now().strftime('%H:%M:%S.%f')[:-3]] + list(q[:6])
               + [a['manipulability'], a['condition_number'], kind, a['sigma_min'],
                  idx.get('wrist', ''), idx.get('elbow', ''), idx.get('shoulder', '')]
               + a['J'].flatten().tolist())
        self.csv_writer.writerow(row)   # 디스크에는 flush_csv 가 1초마다 내보냄

    # ------------------------------------------------------------------ 관절 상태 처리

    def joint_callback(self, msg):
        try:
            self._process(msg)
        except Exception as e:
            # 같은 오류가 메시지마다(수십 Hz) 반복되면 터미널이 넘치므로 1초에 한 번만
            self.get_logger().error(f'데이터 처리 중 오류 발생: {e}', throttle_duration_sec=LOG_EVERY_SEC)

    def _process(self, msg):
        # 팔 관절 6개를 이름으로 찾음 (그리퍼 관절이 섞여 있거나 순서가 달라도 됨)
        joint_map = dict(zip(msg.name, msg.position))
        q = [joint_map[name] for name in sg.JOINT_NAMES if name in joint_map]
        if len(q) != self.dof:
            if not self._missing_warned:   # 이름이 달라 조용히 아무것도 안 되는 일이 없게 한 번은 알림
                self._missing_warned = True
                self.get_logger().warn(
                    f'/joint_states 에서 관절 {self.dof}개를 찾지 못해 분석을 건너뜁니다. '
                    f'받은 이름: {list(msg.name)} / 기대한 이름: {sg.JOINT_NAMES}')
            return

        q = np.array(q, dtype=float)   # 잘못된 값이면 여기서 실패 → 속도 추정용 직전 값이 오염되지 않음
        qdot = self._joint_velocity(msg, q)
        a = sg.analyze(self.robot, q, end=self.end, wrist=self.wrist)
        kind = sg.label(a['types'])
        self._log(a, kind)

        self.save_to_csv(q, a, kind)
        self.publish_metrics(a, kind)
        self.publish_ellipsoids(q, a)
        self.publish_status(q, qdot, a)

    def _log(self, a, kind):
        """지표는 1초에 한 번, 특이점 종류가 바뀌는 순간은 바로"""
        self.get_logger().info(
            f"Manipulability: {a['manipulability']:.4f} | Condition: {a['condition_number']:.2f} | {kind}",
            throttle_duration_sec=LOG_EVERY_SEC)
        if kind == self._last_type:
            return
        if kind == 'normal':
            self.get_logger().info('특이점 영역을 벗어났습니다.')
        else:
            indices = 'n/a' if a['indices'] is None else ', '.join(f'{k}={v:.3f}' for k, v in a['indices'].items())
            self.get_logger().warn(f'특이점 근처 진입: {kind} (지표: {indices})')
        self._last_type = kind

    def _joint_velocity(self, msg, q):
        """관절 속도(rad/s). /joint_states 에 velocity 가 있으면(두산 드라이버·ros2_control) 그대로,
        없으면(관절 슬라이더) 직전 메시지와의 차분을 지수 평활해서 추정. 아직 모르면 None."""
        velocity = getattr(msg, 'velocity', None)
        if velocity is not None and len(velocity) == len(msg.name):
            vmap = dict(zip(msg.name, velocity))
            if all(name in vmap for name in sg.JOINT_NAMES):
                self._qdot = np.array([vmap[name] for name in sg.JOINT_NAMES], dtype=float)
                return self._qdot
        now = time.monotonic()
        if self._prev_q is not None:
            dt = now - self._prev_q[1]
            if dt > 0.5:                 # 오래 끊겼다가 다시 들어오면 속도를 처음부터 다시 추정
                self._qdot = None
            elif dt > 1e-4:
                raw = (q - self._prev_q[0]) / dt
                self._qdot = raw if self._qdot is None else (
                    QDOT_SMOOTHING * raw + (1 - QDOT_SMOOTHING) * self._qdot)
        self._prev_q = (q.copy(), now)
        return self._qdot

    # ------------------------------------------------------------------ 발행

    def publish_metrics(self, a, kind):
        """지표 토픽 (rqt_plot /singularity/manipulability/data 등으로 실시간 그래프)"""
        for key, pub in self.metric_pubs.items():
            msg = Float64()
            msg.data = float(a[key])
            pub.publish(msg)
        msg = String()
        msg.data = kind
        self.type_pub.publish(msg)

    def publish_ellipsoids(self, q, a):
        """RViz 가동성 타원체. 손목 중심에 두 개를 그린다 (plot_ellipsoid.py 와 같은 분해).
        - ns 'arm':   팔(1~3축) 선속도 타원체 → 팔꿈치·어깨 특이점에서 납작, 빨강
        - ns 'wrist': 손목(4~6축) 각속도 타원체 → 손목 특이점에서 납작, 빨강
        손목 중심 링크가 없으면 말단의 선속도 타원체 하나만 그린다."""
        if self.wrist is not None:
            center = rm.link_position(self.robot, q, self.wrist)
            items = [('arm', a['ellipsoid_arm'], bool({'elbow', 'shoulder'} & set(a['types']))),
                     ('wrist', a['ellipsoid_wrist'], 'wrist' in a['types'])]
        else:
            center = rm.link_position(self.robot, q, self.end)
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

    def publish_status(self, q, qdot, a):
        """대시보드용 종합 상태 JSON (status_rate_hz 로 빈도 제한).
        기울기 계산에 Jacobian 12번, 로봇팔 그림에 순기구학 6번이 더 들므로 발행할 때만 계산한다."""
        now = time.monotonic()
        if self._last_status is not None and now - self._last_status < self.status_period:
            return
        self._last_status = now
        grad = None
        if self.wrist is not None:
            grad = st.index_gradient(self.robot, q, end=self.end, wrist=self.wrist, base=a['indices'])
        status = st.build_status(a, q, qdot=qdot, grad=grad, stamp=time.time(), warn_factor=self.warn_factor,
                                 geometry=st.arm_geometry(self.robot, q, J=a['J']))
        msg = String()
        msg.data = json.dumps(status, allow_nan=False)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = M0609SingularityMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
