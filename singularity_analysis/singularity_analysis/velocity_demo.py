"""
특이점 근처 '속도 명령 폭주'와 감쇠 최소제곱(DLS) 비교 시뮬레이션 (ROS 없이 URDF만 사용)

상황: 말단 자세(방향)는 그대로 두고 일정한 속도로 아래로 직선 이동.
      이 경로는 q5 를 0 쪽으로 끌고 가서 손목 특이점을 지나간다.
각 순간 원하는 말단 속도 ẋ (경로 속도 + 오차 보정)를 관절 속도 q̇ 로 바꾸는 방법 세 가지를 비교한다.

1. 역행렬 (Inverse):              q̇ = J⁻¹ ẋ
   → 특이점에서 J⁻¹ 이 무한대로 커져 관절 속도가 폭주
2. 역행렬 + 관절 속도 제한:        위 q̇ 를 최대 관절 속도(M0609 URDF 2.618 rad/s) 안으로 비율 축소
   → 실제 제어기처럼 속도는 막지만, 그만큼 경로를 못 따라가 오차가 커짐
3. 감쇠 최소제곱 (DLS):           q̇ = Jᵀ (J Jᵀ + λ² I)⁻¹ ẋ
   λ 는 최소 특이값 σmin 이 ε 보다 작을 때만 켬:  λ² = (1 − (σmin/ε)²) · λmax²   (Nakamura·Chiaverini 방식)
   → 특이점 근처에서 '못 움직이는 방향'의 속도를 조금 포기하는 대신 관절 속도를 작게 유지

사용:
    ros2 run singularity_analysis velocity_demo
    ros2 run singularity_analysis velocity_demo --eps 0.08 --lam 0.08     # 감쇠를 더 크게
결과: ~/singularity_results/velocity_dls_demo.png (SINGULARITY_RESULTS_DIR 로 변경 가능)
"""
import argparse
import os

import numpy as np
import matplotlib.pyplot as plt

try:                                  # ros2 run / 패키지로 설치했을 때
    from . import singularity as sg
    from .calculate_jacobian import load_robot
except ImportError:                   # python3 파일경로로 직접 실행할 때
    import singularity as sg
    from calculate_jacobian import load_robot


Q_START = [0, -0.3, 1.2, 0, 0.3, 0]   # 시작 자세 (q5 = 0.3 rad)
DISTANCE = 0.25                       # 아래로 이동할 거리 (m)
DURATION = 5.0                        # 이동 시간 (s) → 5 cm/s
DT = 0.002                            # 시뮬레이션 간격 (s)
GAIN = 5.0                            # 위치·자세 오차 보정 이득 (1/s)
JOINT_SPEED_LIMIT = 2.618             # M0609 URDF 관절 최대 속도 (rad/s)


def to_matrix(T):
    return np.asarray(T.A if hasattr(T, 'A') else T, dtype=float)


def rotation_error(R_des, R):
    """현재 자세 R 에서 목표 자세 R_des 로 가는 회전 오차 벡터 (축 × 각도, base 좌표계)"""
    Re = R_des @ R.T
    angle = np.arccos(np.clip((np.trace(Re) - 1) / 2, -1.0, 1.0))
    if angle < 1e-9:
        return np.zeros(3)
    return angle / (2 * np.sin(angle)) * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])


def joint_velocity(J, xdot, method, eps, lam_max):
    """방법별로 원하는 말단 속도 → 관절 속도. 반환: (q̇, 사용한 λ)"""
    if method == 'dls':
        sigma_min = np.linalg.svd(J, compute_uv=False)[-1]
        lam2 = 0.0 if sigma_min >= eps else (1 - (sigma_min / eps) ** 2) * lam_max ** 2
        return J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(6), xdot), np.sqrt(lam2)
    try:
        dq = np.linalg.solve(J, xdot)
    except np.linalg.LinAlgError:          # 정확히 특이하면 역행렬이 없음
        dq = np.linalg.lstsq(J, xdot, rcond=None)[0]
    if method == 'inverse_limited':
        peak = np.abs(dq).max()
        if peak > JOINT_SPEED_LIMIT:
            dq = dq * (JOINT_SPEED_LIMIT / peak)   # 방향은 유지하고 크기만 줄임 (실제 제어기의 속도 제한)
    return dq, 0.0


def simulate(robot, method, eps, lam_max):
    """직선 경로를 따라가는 속도 제어(resolved-rate control)를 적분. 시간별 기록을 dict 로 반환"""
    end = sg.END_LINK
    q = np.array(Q_START, dtype=float)
    T0 = to_matrix(robot.fkine(q, end=end))
    p0, R0 = T0[:3, 3].copy(), T0[:3, :3].copy()
    v = np.array([0.0, 0.0, -DISTANCE / DURATION])

    log = {k: [] for k in ('t', 'speed', 'pos_err', 'rot_err', 'sigma_min', 'lam', 'q4', 'q5', 'q6')}
    for k in range(int(DURATION / DT) + 1):
        t = k * DT
        T = to_matrix(robot.fkine(q, end=end))
        e = np.r_[p0 + v * t - T[:3, 3], rotation_error(R0, T[:3, :3])]
        xdot = np.r_[v, 0, 0, 0] + GAIN * e           # 경로 속도 + 오차 보정
        J = robot.jacob0(q, end=end)[:, :6]
        dq, lam = joint_velocity(J, xdot, method, eps, lam_max)

        log['t'].append(t)
        log['speed'].append(np.abs(dq).max())
        log['pos_err'].append(np.linalg.norm(e[:3]) * 1000)          # mm
        log['rot_err'].append(np.degrees(np.linalg.norm(e[3:])))      # deg
        log['sigma_min'].append(np.linalg.svd(J, compute_uv=False)[-1])
        log['lam'].append(lam)
        log['q4'].append(q[3]); log['q5'].append(q[4]); log['q6'].append(q[5])
        q = q + dq * DT
    return {k: np.array(v) for k, v in log.items()}


def main():
    ap = argparse.ArgumentParser(description="특이점 근처 관절 속도 폭주 vs 감쇠 최소제곱(DLS)")
    ap.add_argument("urdf", nargs="?", default=None, help="M0609 URDF/xacro (생략하면 my_robot_description 의 m0609.urdf.xacro)")
    ap.add_argument("--eps", type=float, default=0.05, help="DLS 를 켜는 최소 특이값 기준 ε (기본 0.05)")
    ap.add_argument("--lam", type=float, default=0.05, help="최대 감쇠 계수 λmax (기본 0.05)")
    ap.add_argument("--no-show", action="store_true", help="화면에 띄우지 않고 파일만 저장")
    args = ap.parse_args()

    robot = load_robot(args.urdf or sg.default_urdf())
    runs = {
        'Inverse  J^-1': ('inverse', 'tab:red'),
        f'Inverse + speed limit ({JOINT_SPEED_LIMIT} rad/s)': ('inverse_limited', 'tab:orange'),
        f'DLS (eps={args.eps}, lambda_max={args.lam})': ('dls', 'tab:green'),
    }
    results = {}
    print(f"{'method':48s} {'max joint speed':>16s} {'max pos err':>12s} {'max rot err':>12s}")
    for name, (method, _) in runs.items():
        r = simulate(robot, method, args.eps, args.lam)
        results[name] = r
        print(f"{name:48s} {r['speed'].max():12.2f} rad/s {r['pos_err'].max():9.2f} mm {r['rot_err'].max():9.2f} deg")

    fig, axes = plt.subplots(4, 1, figsize=(11, 13), sharex=True)
    ax_speed, ax_err, ax_sigma, ax_q = axes
    for name, (method, color) in runs.items():
        r = results[name]
        ax_speed.plot(r['t'], r['speed'], color=color, linewidth=2, label=name)
        ax_err.plot(r['t'], r['pos_err'], color=color, linewidth=2, label=name)
    ax_speed.axhline(JOINT_SPEED_LIMIT, color='k', linestyle=':', label='M0609 joint speed limit')
    ax_speed.set_yscale('log')
    ax_speed.set_ylabel('max |joint speed| (rad/s)')
    ax_speed.set_title('Joint speed command while moving straight down 25 cm at 5 cm/s (orientation fixed)')
    ax_speed.legend(fontsize=8)
    ax_err.set_yscale('symlog', linthresh=0.1)   # 0 근처는 선형, 큰 값은 로그
    ax_err.set_ylim(bottom=0)
    ax_err.set_ylabel('position error (mm)')
    ax_err.set_title('Path tracking error')
    ax_err.legend(fontsize=8)

    dls = results[list(runs)[2]]
    ax_sigma.plot(dls['t'], dls['sigma_min'], color='tab:blue', linewidth=2, label='sigma_min of J')
    ax_sigma.plot(dls['t'], dls['lam'], color='tab:green', linestyle='--', linewidth=2, label='lambda used by DLS')
    ax_sigma.axhline(args.eps, color='tab:blue', linestyle=':', label=f'eps = {args.eps} (DLS turns on below)')
    ax_sigma.set_ylabel('value')
    ax_sigma.set_title('Smallest singular value and DLS damping')
    ax_sigma.legend(fontsize=8)

    inv = results[list(runs)[0]]
    for j, c in (('q4', 'tab:purple'), ('q5', 'tab:orange'), ('q6', 'tab:brown')):
        ax_q.plot(inv['t'], inv[j], color=c, linewidth=1.5, linestyle=':', label=f'{j} (inverse)')
        ax_q.plot(dls['t'], dls[j], color=c, linewidth=2, label=f'{j} (DLS)')
    ax_q.set_ylabel('joint angle (rad)')
    ax_q.set_xlabel('time (s)')
    ax_q.set_title('Wrist joints: inverse flips q4/q6 violently as q5 crosses 0')
    ax_q.legend(fontsize=8, ncol=2)
    for ax in axes:
        ax.grid(True, linestyle='--', alpha=0.6)
    plt.tight_layout()

    out = os.path.join(sg.results_dir(), 'velocity_dls_demo.png')
    plt.savefig(out, dpi=130)
    print(f"그래프 저장: {out}")
    if not args.no_show:
        plt.show()


if __name__ == '__main__':
    main()
