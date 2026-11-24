"""
한 자세의 Jacobian·특이점 지표 계산 (ros2 run singularity_analysis calculate_jacobian)

    ros2 run singularity_analysis calculate_jacobian                          # 기본: 손목 특이점 근처 (q5 ≈ 0)
    ros2 run singularity_analysis calculate_jacobian --q 0 -0.3 1.2 0 1.0 0   # 다른 자세 (rad)
    ros2 run singularity_analysis calculate_jacobian 다른로봇.urdf

출력: Jacobian(말단 link_6, base 좌표계), 행렬식, Manipulability √det(J·Jᵀ), 조건수 σmax/σmin,
      특이점 종류 지표(손목·팔꿈치·어깨)와 가장 움직이기 어려운 방향
"""
import argparse

import numpy as np

try:                                  # ros2 run / 패키지로 설치했을 때
    from . import robot_model as rm
    from . import singularity as sg
except ImportError:                   # python3 파일경로로 직접 실행할 때
    import robot_model as rm
    import singularity as sg


def main():
    ap = argparse.ArgumentParser(description="M0609 한 자세의 Jacobian·특이점 지표 계산")
    ap.add_argument("urdf", nargs="?", default=None,
                    help="M0609 URDF/xacro 파일 경로 (생략하면 my_robot_description 의 m0609.urdf.xacro)")
    ap.add_argument("--q", nargs=6, type=float, metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6"),
                    default=[0, 0, np.pi / 2, 0, 0.01, 0],
                    help="관절 각도(rad). 기본값은 q5≈0 인 손목 특이점 근처 자세")
    args = ap.parse_args()

    robot = rm.load_robot(args.urdf or rm.default_urdf())
    # 말단은 link_6 으로 명시 (지정하지 않으면 Toolbox 가 'base' 링크를 말단으로 고를 수 있음, docs/theory.md 참고)
    end = sg.END_LINK if sg.END_LINK in robot.link_dict else None
    wrist = sg.WRIST_LINK if sg.WRIST_LINK in robot.link_dict else None
    a = sg.analyze(robot, args.q, end=end, wrist=wrist)

    print(f"Joint angles (rad): {np.round(args.q, 3).tolist()}")
    print(f"Jacobian Matrix:\n{a['J'].round(3)}")
    print(f"Determinant: {np.linalg.det(a['J']):.6f}")   # 0 이면 역행렬이 없음 = 특이점
    print(f"Manipulability Index: {a['manipulability']:.6f}")
    print(f"Condition Number: {a['condition_number']:.2f}")
    if a['indices'] is not None:
        idx = a['indices']
        print(f"Singularity indices (0 = singular): wrist={idx['wrist']:.3f}  elbow={idx['elbow']:.3f}  "
              f"shoulder={idx['shoulder']:.3f}")
        print(f"Singularity type: {sg.label(a['types'])}")
    if a['lost_linear'] is not None:
        print(f"Hardest linear direction (base frame): {np.round(a['lost_linear'], 3).tolist()}")
    if a['lost_angular'] is not None:
        print(f"Hardest rotation axis (base frame): {np.round(a['lost_angular'], 3).tolist()}")


if __name__ == "__main__":
    main()
