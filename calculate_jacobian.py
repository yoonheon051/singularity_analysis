import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

# 1. m0609 로봇 모델 로드 (URDF 경로 지정)
# Doosan Robotics 공식 URDF 파일이 있다면 rtb.models.URDF.load()를 사용합니다.
# 여기서는 예시를 위해 표준 6축 모델을 정의하거나 로드합니다.
robot = rtb.models.URDF.Docce('m0609.urdf') # URDF 파일이 있는 경우

print(f"--- Robot Model: {robot.name} ---")

def analyze_m0609_kinematics(joint_angles):
    """
    주어진 관절 각도에서 Jacobian 및 특이점 분석 수행
    joint_angles: list or ndarray (6 elements in radians)
    """
    # 현재 관절 각도 설정
    q = np.array(joint_angles)

    # 2. Forward Kinematics (말단 위치 확인)
    T = robot.fkine(q)
    
    # 3. Jacobian 행렬 계산 (6x6 matrix for 6-DOF)
    # 기하학적 자코비안 (Geometric Jacobian)
    J = robot.jacob0(q)

    # 4. 특이점 분석 지표 계산
    # (1) Determinant: 0에 가까울수록 특이점
    det_j = np.linalg.det(J)
    
    # (2) Manipulability Index (Yoshikawa's Index)
    # w = sqrt(det(J * J^T))
    w = robot.manipulability(q)
    
    # (3) Condition Number: 1에 가까울수록 등방성(Isotropic), 클수록 특이점에 근접
    cond = np.linalg.cond(J)

    return {
        "jacobian": J,
        "determinant": det_j,
        "manipulability": w,
        "condition_number": cond,
        "end_effector_pose": T
    }

# 예시: Wrist Singularity 상황 테스트 (q5가 0도에 가까울 때)
test_q = [0, 0, np.pi/2, 0, 0.01, 0] # 5번 관절이 0.01 rad일 때
results = analyze_m0609_kinematics(test_q)

print(f"Jacobian Matrix:\n{results['jacobian'].round(3)}")
print(f"Determinant: {results['determinant']:.6f}")
print(f"Manipulability Index: {results['manipulability']:.6f}")
print(f"Condition Number: {results['condition_number']:.2f}")