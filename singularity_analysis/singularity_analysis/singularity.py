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

3) URDF 정리·경로 도우미 (ROS·Toolbox 없이 동작)
   - arm_only_urdf: 그리퍼 등 곁가지를 떼고 base_link → link_6 팔 사슬만 남김
   - default_urdf / results_dir: 기본 URDF 위치, 결과 저장 폴더
"""
import os
import xml.etree.ElementTree as ET

import numpy as np

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']   # M0609 URDF의 관절 이름

# 손목 중심 링크: M0609 URDF에서 joint_4·joint_5 원점이 같은 점(손목 중심)이고 link_5 원점이 그 점
WRIST_LINK = 'link_5'
END_LINK = 'link_6'
ARM_LENGTH = 0.411 + 0.368   # 상완 + 전완 길이(m). 어깨 지표를 0~1 범위로 맞추는 데 사용

# 지표가 이 값보다 작으면 그 종류의 특이점 '근처'로 본다 (M0609 모델로 각도를 바꿔 가며 정함, README 참고)
WRIST_LIMIT = 0.10      # |sin(4·6축 사이 각)| ≈ 0.1 → q5 약 ±5.7° 이내
ELBOW_LIMIT = 0.10      # 2·3축 열벡터 사이 각의 sin 값
SHOULDER_LIMIT = 0.05   # 1번 축 기여도(정규화). 약 4 cm 이내

TYPES = ('wrist', 'elbow', 'shoulder')
LIMITS = {'wrist': WRIST_LIMIT, 'elbow': ELBOW_LIMIT, 'shoulder': SHOULDER_LIMIT}

# 근접도(0~100%)를 매기는 구간: 지표가 임계값의 WARN_FACTOR 배일 때 0%, 임계값일 때 100%
WARN_FACTOR = 3.0
ETA_HORIZON = 30.0     # 이보다 먼 도달 예상 시간(초)은 의미가 없으므로 None
UNITS = {'wrist': 'deg', 'elbow': 'deg', 'shoulder': 'cm'}

# 각 특이점에서 자유도를 잃는 관절 (대시보드에서 관절 색을 정할 때 사용, 1~6)
# - 손목:   det[z4 z5 z6] = 0 → 손목 세 축 J4·J5·J6
# - 팔꿈치: c2 ∥ c3        → 팔을 펴고 접는 J2·J3
# - 어깨:   c1 · n = 0      → 손목 중심이 1번 축 위에 와서 J1 이 말단을 새 방향으로 못 움직임
KIND_JOINTS = {'wrist': (4, 5, 6), 'elbow': (2, 3), 'shoulder': (1,)}
LINK_NAMES = ['link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6']   # joint_i 의 자식 링크 = 관절 i 의 원점


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
    return [t for t in TYPES if indices[t] < LIMITS[t]]


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


# ---------------------------------------------------------------------------------------------
# 대시보드용: 지표 → 물리량 · 근접도(%) · 도달 예상 시간 · 벗어나는 방향
# ---------------------------------------------------------------------------------------------

def physical_value(kind, index):
    """지표(0~1) → 사람이 읽기 쉬운 물리량
    - wrist:    4·6번 축 사이 각(°)   지표 = |det[z4 z5 z6]| = |sin(각)|
    - elbow:    c2·c3 사이 각(°)      지표 = sin(각) → 팔이 완전히 펴지거나 접히기까지 남은 각
    - shoulder: 손목 중심이 1번 축에서 떨어진 정도(cm)  지표 × 팔 길이"""
    if kind == 'shoulder':
        return float(index) * ARM_LENGTH * 100.0
    return float(np.degrees(np.arcsin(np.clip(index, 0.0, 1.0))))


def proximity(kind, index, warn_factor=WARN_FACTOR):
    """특이점 근접도 0~1. 지표가 임계값의 warn_factor 배 이상이면 0, 임계값 이하이면 1, 그 사이는 선형"""
    limit = LIMITS[kind]
    warn = limit * warn_factor
    return float(np.clip((warn - index) / (warn - limit), 0.0, 1.0))


def level_of(risk):
    """전체 근접도 → 'safe'(0%) / 'caution'(0~100%) / 'danger'(100% = 특이점 근처로 분류됨)"""
    if risk is None or risk <= 0.0:
        return 'safe'
    return 'danger' if risk >= 1.0 else 'caution'


def index_gradient(robot, q, end=END_LINK, wrist=WRIST_LINK, eps=1e-5, base=None):
    """특이점 지표의 관절 기울기 ∂지표/∂q (전진 차분). 반환: {'wrist': (6,), 'elbow': (6,), 'shoulder': (6,)}
    base 에 현재 자세의 지표를 주면 Jacobian 계산을 그만큼 아낀다."""
    q = np.asarray(q, dtype=float)

    def idx(qq):
        return singularity_indices(robot.jacob0(qq, end=end)[:, :6], robot.jacob0(qq, end=wrist))

    base = base or idx(q)
    grad = {t: np.zeros(len(q)) for t in TYPES}
    for i in range(len(q)):
        dq = q.copy()
        dq[i] += eps
        moved = idx(dq)
        for t in TYPES:
            grad[t][i] = (moved[t] - base[t]) / eps
    return grad


def time_to_limit(index, rate, limit, horizon=ETA_HORIZON):
    """지표가 지금 속도(rate, 1/s)로 줄어들 때 임계값에 닿기까지 남은 시간(초). 멀어지는 중이거나 이미 안쪽이면 None"""
    if rate is None or rate >= 0.0 or index <= limit:
        return None
    eta = (index - limit) / -rate
    return float(eta) if eta <= horizon else None


def escape_direction(grad):
    """지표를 가장 빨리 키우는(특이점에서 벗어나는) 관절과 방향. 반환: {'joint': 1~6, 'direction': +1/-1} 또는 None"""
    grad = np.asarray(grad, dtype=float)
    j = int(np.argmax(np.abs(grad)))
    if abs(grad[j]) < 1e-9:
        return None
    return {'joint': j + 1, 'direction': 1 if grad[j] > 0 else -1}


def joint_risk(kinds):
    """관절별 위험도 0~1: 그 관절이 자유도를 잃는 특이점(KIND_JOINTS)들의 근접도 중 최댓값.
    kinds: build_status 의 'kinds' (종류 → {'proximity': ...}). 종류 정보가 없으면 None."""
    if not kinds:
        return None
    risk = [0.0] * 6
    for kind, joints in KIND_JOINTS.items():
        if kind in kinds:
            for j in joints:
                risk[j - 1] = max(risk[j - 1], kinds[kind]['proximity'])
    return risk


def joint_eta(kinds):
    """관절별 도달 예상 시간(초): 그 관절이 관여하는 특이점들 중 가장 먼저 임계값에 닿는 시간. 없으면 None"""
    if not kinds:
        return None
    eta = [None] * 6
    for kind, joints in KIND_JOINTS.items():
        t = kinds.get(kind, {}).get('eta')
        if t is None:
            continue
        for j in joints:
            eta[j - 1] = t if eta[j - 1] is None else min(eta[j - 1], t)
    return eta


def arm_geometry(robot, q, J=None):
    """대시보드에 로봇팔을 그리기 위한 관절 위치·회전축 (base 좌표계, m)
    - joints: 관절 1~6 원점 위치 (link_1 ~ link_6 원점)
    - axes:   관절 1~6 회전축 단위벡터 = 말단 Jacobian 의 각속도 부분 열 z_i
    링크가 없으면 None"""
    if not all(name in robot.link_dict for name in LINK_NAMES):
        return None
    q = np.asarray(q, dtype=float)
    if J is None:
        J = robot.jacob0(q, end=END_LINK)
    joints = []
    for name in LINK_NAMES:
        T = robot.fkine(q, end=name)
        joints.append([float(v) for v in np.asarray(T.A if hasattr(T, 'A') else T, dtype=float)[:3, 3]])
    axes = [[float(v) for v in np.asarray(J)[3:, i]] for i in range(6)]
    return {'base': [0.0, 0.0, 0.0], 'joints': joints, 'axes': axes}


def _finite(v):
    """JSON 에 넣을 수 있는 값으로 (inf·nan → None). 브라우저 JSON.parse 는 Infinity 를 못 읽는다."""
    if v is None:
        return None
    v = float(v)
    return v if np.isfinite(v) else None


def build_status(a, q, qdot=None, grad=None, stamp=None, warn_factor=WARN_FACTOR, geometry=None):
    """analyze() 결과 → 대시보드로 보낼 상태 dict (JSON 직렬화 가능, inf 없음)
    a: analyze() 결과, q: 관절각(rad), qdot: 관절 속도(rad/s, 없으면 None), grad: index_gradient() 결과,
    geometry: arm_geometry() 결과 (로봇팔 그림용, 없으면 None)"""
    kinds = {}
    if a['indices'] is not None:
        for t in TYPES:
            index = a['indices'][t]
            limit = LIMITS[t]
            rate = float(np.dot(grad[t], qdot)) if grad is not None and qdot is not None else None
            kinds[t] = {
                'index': _finite(index),
                'limit': limit,
                'proximity': proximity(t, index, warn_factor),
                'value': _finite(physical_value(t, index)),
                'value_limit': physical_value(t, limit),
                'value_warn': physical_value(t, min(limit * warn_factor, 1.0)),
                'unit': UNITS[t],
                'rate': _finite(rate),
                'eta': time_to_limit(index, rate, limit),
                'escape': escape_direction(grad[t]) if grad is not None else None,
            }
    if kinds:
        dominant = max(kinds, key=lambda t: kinds[t]['proximity'])
        risk = kinds[dominant]['proximity']
    else:
        dominant, risk = None, None
    return {
        'stamp': stamp,
        'q': [float(v) for v in q],
        'qdot': None if qdot is None else [float(v) for v in qdot],
        'type': label(a['types']),
        'level': level_of(risk),
        'risk': risk,
        'dominant': dominant,
        'manipulability': _finite(a['manipulability']),
        'condition_number': _finite(a['condition_number']),
        'sigma_min': _finite(a['sigma_min']),
        'warn_factor': warn_factor,
        'kinds': kinds,
        'joint_risk': joint_risk(kinds),
        'joint_eta': joint_eta(kinds),
        'geometry': geometry,
    }


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


# ---------------------------------------------------------------------------------------------
# URDF 정리·경로 도우미
# ---------------------------------------------------------------------------------------------

def arm_only_urdf(urdf_xml, end_link=END_LINK):
    """
    URDF 문자열에서 루트 링크부터 end_link 까지의 사슬만 남긴 URDF 문자열을 돌려준다.
    - RG2 그리퍼처럼 말단 뒤에 붙은 관절(미믹 관절 포함)과, 'base' 같은 곁가지 링크를 뗀다.
      → Toolbox 모델이 항상 팔 6축만 갖게 되어, 관절 순서·개수가 URDF 종류(그리퍼 유무)와 상관없이 같다.
    - visual/collision 은 지운다: 기구학 계산에는 필요 없고, package:// 메시 경로를 찾다가 실패하는 일을 막는다.
    end_link 가 없으면 ValueError.
    """
    root = ET.fromstring(urdf_xml)
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    links = {l.get('name'): l for l in root.findall('link')}
    if end_link not in links:
        raise ValueError(f"URDF 에 '{end_link}' 링크가 없습니다")

    keep_links, keep_joints, link = {end_link}, [], end_link
    while link in joints:                      # 말단에서 부모 쪽으로 거슬러 올라감
        joint = joints[link]
        keep_joints.append(joint)
        link = joint.find('parent').get('link')
        keep_links.add(link)

    robot = ET.Element('robot', {'name': root.get('name', 'robot')})
    for name, el in links.items():
        if name in keep_links:
            for tag in ('visual', 'collision'):
                for child in el.findall(tag):
                    el.remove(child)
            robot.append(el)
    for joint in reversed(keep_joints):
        robot.append(joint)
    return ET.tostring(robot, encoding='unicode')


def root_link(urdf_xml):
    """URDF 의 루트(부모가 없는) 링크 이름. RViz 마커의 frame_id 로 쓴다."""
    root = ET.fromstring(urdf_xml)
    children = {j.find('child').get('link') for j in root.findall('joint')}
    for link in root.findall('link'):
        if link.get('name') not in children:
            return link.get('name')
    return 'base_link'


def read_urdf(path):
    """URDF 또는 xacro 파일 → URDF 문자열. xacro 는 ROS 의 xacro 모듈이 있으면 처리하고,
    없으면 그대로 읽는다 (m0609.urdf.xacro 는 매크로가 없는 일반 URDF라 그대로 써도 됨)."""
    if path.endswith('.xacro'):
        try:
            import xacro
            return xacro.process_file(path).toxml()
        except ImportError:
            pass
    with open(path, encoding='utf-8') as f:
        return f.read()


def default_urdf():
    """기본 M0609 URDF 위치: 설치된 my_robot_description 패키지 → 이 저장소의 my_robot_description 폴더"""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('my_robot_description'), 'urdf', 'm0609.urdf.xacro')
        if os.path.isfile(path):
            return path
    except Exception:   # ROS 가 없거나 패키지를 아직 빌드하지 않음
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, '..', '..', 'my_robot_description', 'urdf', 'm0609.urdf.xacro'))


def results_dir():
    """결과(CSV·그림) 저장 폴더: 환경변수 SINGULARITY_RESULTS_DIR, 없으면 ~/singularity_results.
    colcon 으로 설치하면 코드가 install/ 아래로 복사돼 저장소 위치를 알 수 없으므로 고정 위치를 쓴다."""
    path = os.path.expanduser(os.environ.get('SINGULARITY_RESULTS_DIR', '~/singularity_results'))
    os.makedirs(path, exist_ok=True)
    return path
