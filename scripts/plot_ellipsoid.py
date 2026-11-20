"""
가동성 타원체(Manipulability Ellipsoid) 시각화

여러 자세에서 로봇 팔 모양과 가동성 타원체를 3D로 나란히 그린다.
M0609는 4·5·6축이 한 점(손목 중심)에서 만나므로 det(J) = det(J11)·det(J22) 로 나뉘고, 타원체도 둘로 나눠 그린다.
- 윗줄 ARM: 팔(1~3축)이 손목 중심을 움직이는 선속도 타원체 (J11) → 팔꿈치·어깨 특이점에서 납작해짐
- 아랫줄 WRIST: 손목(4~6축)이 손을 돌리는 각속도 타원체 (J22 = [z4 z5 z6]) → 손목 특이점에서 납작해짐
- 정상 자세: 구에 가까움 → 모든 방향으로 고르게 움직임
- 특이점 근처: 한 축이 납작해짐 → 그 방향(빨간 화살표)으로는 거의 못 움직임
(말단 전체의 3x6 선속도·각속도 타원체는 다른 관절이 조금씩 메워 줘서 특이점에서도 0까지 줄지 않는다.
 그래서 특이점 종류를 보여 주기에는 나눈 타원체가 더 정확하다)

사용:
    python3 scripts/plot_ellipsoid.py m0609.urdf                       # 기본 4자세(정상·손목·팔꿈치·어깨 특이점 근처)
    python3 scripts/plot_ellipsoid.py m0609.urdf --q 0 -0.3 1.2 0 1 0  # 원하는 자세 (여러 번 줄 수 있음)
결과: results/manipulability_ellipsoids.png
"""
import argparse
import os

import numpy as np
import matplotlib.pyplot as plt

import singularity as sg
from calculate_jacobian import load_robot

REPO_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LINKS = ['base_link', 'link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6']

# 기본 자세: (제목, 관절 각도 rad). 특이점 '근처'라 타원체가 납작하지만 완전히 0은 아님
DEFAULT_POSES = [
    ('Normal', [0, -0.3, 1.2, 0, 1.0, 0]),
    ('Near wrist singularity (q5 = 0.05)', [0, -0.3, 1.2, 0, 0.05, 0]),
    ('Near elbow singularity (q3 = 0.05)', [0, 0.5, 0.05, 0, 1.0, 0]),
    ('Near shoulder singularity', [0, -0.65, 1.5, 0, 1.0, 0]),
]
SCALE = 0.3   # 타원체 반지름(m/s per rad/s)을 그림에서 몇 m 로 그릴지


def to_matrix(T):
    """Toolbox 의 SE3 또는 4x4 배열 → 4x4 numpy 배열"""
    return np.asarray(T.A if hasattr(T, 'A') else T, dtype=float)


def link_points(robot, q):
    """팔 모양을 그리기 위한 각 링크 원점 위치들"""
    pts = [to_matrix(robot.fkine(q, end=name))[:3, 3] for name in LINKS if name in robot.link_dict]
    return np.array(pts)


def draw_ellipsoid(ax, pts, center, U, s, singular, title):
    """팔 모양 + center 위치에 타원체(축 방향 U, 반지름 s) + 가장 짧은 축 방향 화살표"""
    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], '-o', color='0.3', linewidth=3, markersize=4)

    # 단위 구를 축 방향 U, 반지름 s 로 늘림
    u, v = np.meshgrid(np.linspace(0, 2 * np.pi, 40), np.linspace(0, np.pi, 20))
    sphere = np.stack([np.cos(u) * np.sin(v), np.sin(u) * np.sin(v), np.cos(v)], axis=-1)
    ell = sphere @ np.diag(s * SCALE) @ U.T + center
    ax.plot_surface(ell[..., 0], ell[..., 1], ell[..., 2],
                    color='tab:red' if singular else 'tab:green', alpha=0.35, linewidth=0)

    # 가장 짧은 축 = 가장 움직이기 어려운 방향
    d = U[:, -1] * 0.25
    ax.quiver(*center, *d, color='red', linewidth=2)
    ax.quiver(*center, *-d, color='red', linewidth=2)

    ax.set_title(title, fontsize=9)
    ax.set_xlim(-0.7, 0.7); ax.set_ylim(-0.7, 0.7); ax.set_zlim(0, 1.4)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel('x'); ax.set_ylabel('y'); ax.set_zlabel('z')


def draw_pose(ax_lin, ax_ang, robot, title, q):
    a = sg.analyze(robot, q)
    pts = link_points(robot, q)
    wrist_center = to_matrix(robot.fkine(q, end=sg.WRIST_LINK))[:3, 3]
    U_a, s_a = a['ellipsoid_arm']
    U_w, s_w = a['ellipsoid_wrist']
    # 작은 반지름 / 큰 반지름: 1이면 구, 0에 가까우면 납작
    flat_a, flat_w = s_a[-1] / s_a[0], s_w[-1] / s_w[0]
    types = a['types']
    draw_ellipsoid(ax_lin, pts, wrist_center, U_a, s_a, bool({'elbow', 'shoulder'} & set(types)),
                   f"{title}\n{sg.label(types)} | w={a['manipulability']:.4f} | cond={a['condition_number']:.0f}\n"
                   f"ARM (j1-3, linear) radii={np.round(s_a, 3).tolist()}  min/max={flat_a:.3f}")
    draw_ellipsoid(ax_ang, pts, wrist_center, U_w, s_w, 'wrist' in types,
                   f"WRIST (j4-6, angular) radii={np.round(s_w, 3).tolist()}  min/max={flat_w:.3f}")
    return a


def main():
    ap = argparse.ArgumentParser(description="가동성 타원체 시각화")
    ap.add_argument("urdf", nargs="?", default="m0609.urdf")
    ap.add_argument("--q", nargs=6, type=float, action="append", metavar="Q",
                    help="관절 각도 6개(rad). 여러 번 주면 여러 자세를 그림")
    ap.add_argument("--no-show", action="store_true", help="화면에 띄우지 않고 파일만 저장")
    args = ap.parse_args()

    robot = load_robot(args.urdf)
    poses = [(f"q = {np.round(q, 2).tolist()}", q) for q in args.q] if args.q else DEFAULT_POSES

    # 자세마다 한 열: 윗줄 선속도, 아랫줄 각속도
    n = len(poses)
    fig = plt.figure(figsize=(5 * n, 10))
    for i, (title, q) in enumerate(poses):
        ax_lin = fig.add_subplot(2, n, i + 1, projection='3d')
        ax_ang = fig.add_subplot(2, n, n + i + 1, projection='3d')
        a = draw_pose(ax_lin, ax_ang, robot, title, q)
        print(f"[{title}] {sg.label(a['types'])}  w={a['manipulability']:.5f}  cond={a['condition_number']:.1f}  "
              f"indices={ {k: round(v, 3) for k, v in a['indices'].items()} }  "
              f"arm radii={np.round(a['ellipsoid_arm'][1], 3).tolist()}  "
              f"wrist radii={np.round(a['ellipsoid_wrist'][1], 3).tolist()}")
    fig.suptitle('Manipulability Ellipsoids (at wrist center) — top: arm joints 1-3 linear velocity, '
                 'bottom: wrist joints 4-6 angular velocity, red arrow: hardest direction', fontsize=12)
    plt.tight_layout()

    out = os.path.join(REPO_DIR, 'results', 'manipulability_ellipsoids.png')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    plt.savefig(out, dpi=150)
    print(f"그래프 저장: {out}")
    if not args.no_show:
        plt.show()


if __name__ == '__main__':
    main()
