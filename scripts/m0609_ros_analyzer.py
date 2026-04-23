import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, DurabilityPolicy
import roboticstoolbox as rtb
import numpy as np
import csv
import os
from datetime import datetime

class M0609SingularityMonitor(Node):
    def __init__(self):
        super().__init__('m0609_singularity_monitor')
        
        # 1. 파일 저장 경로 및 헤더 설정
        self.results_path = 'results/analysis_log.csv'
        self.init_csv()

        self.get_logger().info('URDF 데이터를 /robot_description 토픽에서 기다리는 중...')
        
        # [기존 URDF 수신 로직 생략] - 이전 버전과 동일하게 작성하시면 됩니다.
        self.urdf_xml = None
        self.subscription = self.create_subscription(String, '/robot_description', self.urdf_callback, 10)

        while self.urdf_xml is None:
            rclpy.spin_once(self)

        try:
            self.robot = rtb.models.URDF.Docce(self.urdf_xml)
            self.get_logger().info(f'로봇 모델 로드 성공: {self.robot.name}')
        except Exception as e:
            self.get_logger().error(f'모델 로드 실패: {str(e)}')
            return

        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.get_logger().info('실시간 모니터링 및 저장 시작!')

    def init_csv(self):
        """CSV 파일 초기화 및 헤더 작성"""
        if not os.path.exists('results'):
            os.makedirs('results')
        
        with open(self.results_path, 'w', newline='') as f:
            writer = csv.writer(f)
            # 헤더: 시간, 관절각(6개), 가동성, 조건수, 야코비안(36개 원소)
            header = ['timestamp', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'manipulability', 'condition_number']
            for i in range(6):
                for j in range(6):
                    header.append(f'J_{i}{j}')
            writer.writerow(header)

    def joint_callback(self, msg):
        try:
            q = msg.position[:6]
            J, w, cond = self.calculate_metrics(q)
            
            # 1. 터미널 출력
            self.get_logger().info(f'Manipulability: {w:.4f} | Condition: {cond:.2f}')
            
            # 2. 파일 저장 로직
            self.save_to_csv(q, J, w, cond)
            
        except Exception as e:
            self.get_logger().error(f'데이터 처리 오류: {str(e)}')

    def save_to_csv(self, q, J, w, cond):
        """데이터를 CSV 한 줄로 저장"""
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        
        # 데이터를 리스트로 구성
        row = [timestamp] + list(q) + [w, cond] + J.flatten().tolist()
        
        with open(self.results_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)

    def calculate_metrics(self, joint_states):
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