"""
특이점 분석 공통 계산 (NumPy만 사용, ROS·Toolbox 없이도 테스트 가능)

1) 가동성 타원체 (Manipulability Ellipsoid)
   관절 속도를 크기 1인 구(||q̇|| = 1) 안에서 움직일 때 말단이 낼 수 있는 속도의 모양.
   J = U Σ Vᵀ (특이값 분해)라면 타원체의 축 방향은 U의 열, 반지름은 특이값 σ.
   - 구에 가까움: 모든 방향으로 고르게 움직일 수 있음
   - 납작함: σ가 작은 방향(U의 마지막 열)으로는 거의 못 움직임 → 특이점 접근

2) 특이점 종류 분류 (M0609처럼 4·5·6축이 한 점에서 만나는 '구형 손목' 로봇)
   손목 중심(wrist center) p_wc 기준으로 Jacobian을 쓰면
       J = [[J11, 0], [J21, J22]]   (J11: 팔 3축의 선속도 부분, J22 = [z4 z5 z6])
   이므로  det(J) = det(J11) · det(J22)  — 기준점을 말단으로 옮겨도 행렬식은 같다.
   그리고 J11 = [c1 c2 c3] (c_i = z_i × (p_wc − p_i)) 에서
       det(J11) = c1 · (c2 × c3) = (c1 · n) · |c2 × c3|,  n = (c2 × c3) / |c2 × c3|
   세 인수가 각각 0이 되는 경우가 세 가지 특이점이다.
   - 손목 특이점 (wrist):     det[z4 z5 z6] = 0      → 4번·6번 축이 일직선 (q5 ≈ 0)
   - 팔꿈치 특이점 (elbow):   |c2 × c3| = 0          → 팔이 완전히 펴지거나 접힘
   - 어깨 특이점 (shoulder):  c1 · n = 0             → 손목 중심이 1번 축 쪽으로 와서 1번 관절이 새 방향을 못 만듦
"""
import numpy as np

# 손목 중심 링크: M0609 URDF에서 joint_4·joint_5 원점이 같은 점(손목 중심)이고 link_5 원점이 그 점
WRIST_LINK = 'link_5'
END_LINK = 'link_6'
ARM_LENGTH = 0.411 + 0.368   # 상완 + 전완 길이(m). 어깨 지표를 0~1 범위로 맞추는 데 사용

# 지표가 이 값보다 작으면 그 종류의 특이점 '근처'로 본다 (M0609 모델로 각도를 바꿔 가며 정함, README 참고)
WRIST_LIMIT = 0.10      # |sin(4·6축 사이 각)| ≈ 0.1 → q5 약 ±5.7° 이내
ELBOW_LIMIT = 0.10      # 2·3축 열벡터 사이 각의 sin 값
SHOULDER_LIMIT = 0.05   # 1번 축 기여도(정규화). 약 4 cm 이내

TYPES = ('wrist', 'elbow', 'shoulder')


def ellipsoid(J_part):
    """3x6 (선속도 또는 각속도) Jacobian → (축 방향 3x3 회전행렬, 반지름 3개). 반지름은 큰 순서.
    회전행렬은 det = +1 로 맞춘다 (RViz 등에서 자세로 쓰려면 오른손 좌표계여야 함)."""
    U, s, _ = np.linalg.svd(np.asarray(J_part, dtype=float))
    if np.linalg.det(U) < 0:
        U[:, -1] *= -1
    return U, s


def lost_direction(J):
    """6x6 Jacobian 에서 가장 움직이기 어려운 말단 방향 (최소 특이값의 왼쪽 특이벡터).
    반환: (σ_min, 선속도 방향 단위벡터 또는 None, 각속도 방향 단위벡터 또는 None)"""
    U, s, _ = np.linalg.svd(np.asarray(J, dtype=float))
    u = U[:, -1]

    def unit(v):
        n = np.linalg.norm(v)
        return v / n if n > 0.3 else None   # 이 성분이 작으면 그쪽(선/각) 방향 문제는 아님
    return float(s[-1]), unit(u[:3]), unit(u[3:])


def singularity_indices(J_end, J_wrist):
    """
    J_end:   말단(link_6) 기준 6x6 Jacobian (base 좌표계)
    J_wrist: 손목 중심(link_5) 기준 Jacobian (6x5 이상, 앞 3열만 사용)
    반환: {'wrist': 0~1, 'elbow': 0~1, 'shoulder': 0~약1}  — 0에 가까울수록 그 특이점에 가까움
    """
    J_end = np.asarray(J_end, dtype=float)
    J_wrist = np.asarray(J_wrist, dtype=float)
    z4, z5, z6 = J_end[3:, 3], J_end[3:, 4], J_end[3:, 5]
    wrist = abs(np.linalg.det(np.column_stack([z4, z5, z6])))

    c1, c2, c3 = J_wrist[:3, 0], J_wrist[:3, 1], J_wrist[:3, 2]
    cross = np.cross(c2, c3)
    norm_23 = np.linalg.norm(c2) * np.linalg.norm(c3)
    elbow = np.linalg.norm(cross) / norm_23 if norm_23 > 1e-12 else 0.0
    if np.linalg.norm(cross) > 1e-12:
        shoulder = abs(np.dot(c1, cross / np.linalg.norm(cross))) / ARM_LENGTH
    else:
        shoulder = np.linalg.norm(c1) / ARM_LENGTH   # 팔꿈치가 완전히 특이하면 n 이 정해지지 않음
    return {'wrist': float(wrist), 'elbow': float(elbow), 'shoulder': float(shoulder)}


def classify(indices):
    """지표 → 근처에 있는 특이점 종류 목록 (예: ['wrist'] / ['elbow', 'shoulder'] / [])"""
    limits = {'wrist': WRIST_LIMIT, 'elbow': ELBOW_LIMIT, 'shoulder': SHOULDER_LIMIT}
    return [t for t in TYPES if indices[t] < limits[t]]


def label(types):
    """CSV·그래프용 문자열: 'normal' 또는 'wrist+elbow' 처럼"""
    return '+'.join(types) if types else 'normal'


def analyze(robot, q, end=END_LINK, wrist=WRIST_LINK):
    """Robotics Toolbox 로봇 모델(또는 같은 jacob0/fkine 을 가진 객체)로 한 자세를 종합 분석"""
    q = np.asarray(q, dtype=float)
    J = robot.jacob0(q, end=end)[:, :6]
    U_v, s_v = ellipsoid(J[:3])
    U_w, s_w = ellipsoid(J[3:])
    sigma_min, lost_v, lost_w = lost_direction(J)
    result = {
        'J': J,
        'manipulability': float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0))),
        'condition_number': float(np.linalg.cond(J)),
        'sigma_min': sigma_min,
        'lost_linear': lost_v, 'lost_angular': lost_w,
        'ellipsoid_linear': (U_v, s_v), 'ellipsoid_angular': (U_w, s_w),
        'indices': None, 'types': [],
    }
    if wrist is not None:
        J_wc = robot.jacob0(q, end=wrist)
        result['indices'] = singularity_indices(J, J_wc)
        result['types'] = classify(result['indices'])
        # 분해된 타원체: 특이점 종류마다 정확히 납작해지는 쪽
        #  - 팔(1~3축)이 손목 중심을 움직이는 선속도: J11 → 팔꿈치·어깨 특이점에서 한 축이 0
        #  - 손목(4~6축)이 손을 돌리는 각속도: J22 = [z4 z5 z6] → 손목 특이점에서 한 축이 0
        result['ellipsoid_arm'] = ellipsoid(J_wc[:3, :3])
        result['ellipsoid_wrist'] = ellipsoid(J[3:, 3:6])
    return result


def rotation_to_quaternion(R):
    """3x3 회전행렬 → (x, y, z, w) 쿼터니언 (RViz Marker 자세용)"""
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    if i == 1:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s)
