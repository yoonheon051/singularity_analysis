import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

# 1. 분석 대상 로봇 모델(m0609) 로드
# URDF 파일에는 로봇의 링크 길이, 질량, 관절 회전축 등 물리적/기구학적 정보가 포함됨 
try:
    # m0609.urdf 파일에서 기구학 트리 구조를 읽어와 수학적 객체로 생성
    robot = rtb.models.URDF('m0609.urdf')
except:
    # 파일이 없는 경우를 대비한 디버깅 출력
    print(f"---Model not found---")

def analyze_m0609_kinematics(joint_angles):
    """
    특정 관절 자세(Configuration)에서 로봇의 기구학적 특성을 종합 분석하는 함수
    관절 각도(Radian)-> 위치/자세(Pose), Jacobian, 특이점 지표 계산
    """

    # 1. 입력된 리스트 형태의 관절 각도를 수치 연산을 위해 Numpy 배열로 반환
    q = np.array(joint_angles)

    # 2. 순기구학(Forward Kinematics)계산
    # 관절 각도를 넣었을 때 로봇 말단(End-Effector)의 위치와 자세(4x4 Homogeneous Metrix)를 계산
    T = robot.fkine(q)

    # 3. Jacobian 행렬 계산
    # jacob0: 베이스 좌표계(world Frame) 기준의 기하학적 Jacobian
    # 선속도(v)와 각속도(w)를 관절 속도(q_dot)와 연결하는 6x6 행렬(v = J x q_dot) 
    J = robot.jacob0(q)

    # 4. 특이점(Singularity) 분석 지표 계산

    # (1) Determinant(행렬식)
    # det(J)가 0이면 역행렬이 존재하지 않으며, 특정 방향으로의 움직임이 불가능한 '특이점' 상태임을 의미
    det_j = np.linalg.det(J)

    # (2) Manipulability Index(가동성 지수)
    # Yoshikawa의 지표: sqrt(det(J x J^T))로 계산됨
    # 이 값이 클수록 로봇 말단이 전 방향으로 균일하고 빠르게 움직일 수 있는 좋은 자세임
    w = robot.manipulability(q)

    # (3) Condition Number(조건수)
    # Jacobian의 최대 특이값과 최소 특이값의 비율
    # 1에 가까울수록 말단의 제어 정밀도가 높고(Isotropic), 클수록 제어 능력이 상실되는 특이점에 근접함
    cond = np.linalg.cond(J)

    # 모든 분석 결과를 딕셔너리 형태로 반환
    return {
        "jacobian": J,
        "determinant": det_j,
        "manipulability": w,
        "condition_number": cond,
        "end_effector_pose": T
    }

# =========================================================================
# 분석 예시: 손목 특이점(Wrist Singularity) 상황 테스트
# 설명: 6축 로봇에서 4번 관절과 6번 관절의 회전축이 일직선이 될 때(보통 q5=0) 발생
# =========================================================================
test_q = [0, 0, np.pi/2, 0, 0.01, 0] # 5번 관절(q5)을 0에 매우 가깝게 설정하여 특이점 모사
results = analyze_m0609_kinematics(test_q)

# 결과 출력 (소수점 셋째자리까지 반올림)
print(f"Jacobian Matrix:\n{results['jacobian'].round(3)}")
print(f"Determinant: {results['determinant']:.6f}")
print(f"Manipulability Index: {results['manipulability']:.6f}")
print(f"Condition Number: {results['condition_number']:.2f}")