import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import roboticstoolbox as rtb
import numpy as np

class M0609SingularityMonitor(Node):
    def __init__(self):
        super().__init__('m0609_singularity_monitor')
        
        self.get_logger().info('URDF 데이터를 /robot_description 토픽에서 기다리는 중...')
        
        # 1. URDF 수신용 임시 구독자
        self.urdf_xml = None
        self.subscription = self.create_subscription(
            String,
            '/robot_description',
            self.urdf_callback,
            10)

        # URDF 데이터를 받을 때까지 노드를 회전하며 대기
        while self.urdf_xml is None:
            rclpy.spin_once(self)

        # 2. 받은 URDF 문자열로 로봇 모델 로드
        try:
            # m0609.urdf 파일이 따로 없어도 토픽의 내용으로 로드합니다.
            self.robot = rtb.models.URDF.Docce(self.urdf_xml)
            self.get_logger().info(f'로봇 모델 로드 성공: {self.robot.name}')
        except Exception as e:
            self.get_logger().error(f'모델 로드 실패: {str(e)}')
            return

        # 3. 모델 로드 후 실시간 관절 상태 구독 시작
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10)
        self.get_logger().info('실시간 모니터링 시작!')

    def urdf_callback(self, msg):
        self.urdf_xml = msg.data
        # 데이터를 한 번 받았으므로 구독 해제
        self.destroy_subscription(self.subscription)
        self.get_logger().info('URDF 데이터를 성공적으로 수신했습니다.')

    def joint_callback(self, msg):
        """RViz에서 조인트를 움직일 때마다 실행되는 콜백"""
        try:
            # m0609의 6개 관절 각도 추출
            # 실제 로봇의 관절 이름 순서에 따라 조정이 필요할 수 있습니다.
            q = msg.position[:6] 
            
            J, w, cond = self.calculate_metrics(q)
            
            # 터미널에 실시간 출력
            self.get_logger().info(f'Manipulability: {w:.4f} | Condition: {cond:.2f}')
        except Exception as e:
            self.get_logger().error(f'계산 오류: {str(e)}')

    def calculate_metrics(self, joint_states):
        """관절 각도를 받아 Jacobian과 특이점을 계산"""
        q = np.array(joint_states)
        J = self.robot.jacob0(q)
        w = self.robot.manipulability(q)
        cond = np.linalg.cond(J)
        return J, w, cond

def main(args=None):
    rclpy.init(args=args)
    try:
        node = M0609SingularityMonitor()
        rclpy.spin(node)
    except Exception as e:
        print(f"노드 실행 중 에러 발생: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()