"""
대시보드용 상태 (/singularity/status)

특이점 지표(singularity.py)를 사람이 읽기 쉬운 값으로 바꾸고, 한 개의 JSON 으로 묶는다.
- 물리량·근접도(%): physical_value, proximity, level_of
- 도달 예상 시간·벗어나는 방향: index_gradient, time_to_limit, escape_direction
- 관절 J1~J6 기준 위험도: joint_risk, joint_eta (KIND_JOINTS)
- 로봇팔 그림용 관절 위치·축: arm_geometry
- 모두 묶기: build_status
"""
import numpy as np

try:                                  # ros2 run / 패키지로 설치했을 때
    from . import singularity as sg
    from .robot_model import link_position
except ImportError:                   # python3 파일경로로 직접 실행할 때
    import singularity as sg
    from robot_model import link_position

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


def physical_value(kind, index):
    """지표(0~1) → 사람이 읽기 쉬운 물리량
    - wrist:    4·6번 축 사이 각(°)   지표 = |det[z4 z5 z6]| = |sin(각)|
    - elbow:    c2·c3 사이 각(°)      지표 = sin(각) → 팔이 완전히 펴지거나 접히기까지 남은 각
    - shoulder: 손목 중심이 1번 축에서 떨어진 정도(cm)  지표 × 팔 길이"""
    if kind == 'shoulder':
        return float(index) * sg.ARM_LENGTH * 100.0
    return float(np.degrees(np.arcsin(np.clip(index, 0.0, 1.0))))


def proximity(kind, index, warn_factor=WARN_FACTOR):
    """특이점 근접도 0~1. 지표가 임계값의 warn_factor 배 이상이면 0, 임계값 이하이면 1, 그 사이는 선형"""
    limit = sg.LIMITS[kind]
    warn = limit * warn_factor
    return float(np.clip((warn - index) / (warn - limit), 0.0, 1.0))


def level_of(risk):
    """전체 근접도 → 'safe'(0%) / 'caution'(0~100%) / 'danger'(100% = 특이점 근처로 분류됨)"""
    if risk is None or risk <= 0.0:
        return 'safe'
    return 'danger' if risk >= 1.0 else 'caution'


def index_gradient(robot, q, end=sg.END_LINK, wrist=sg.WRIST_LINK, eps=1e-5, base=None):
    """특이점 지표의 관절 기울기 ∂지표/∂q (전진 차분). 반환: {'wrist': (6,), 'elbow': (6,), 'shoulder': (6,)}
    base 에 현재 자세의 지표를 주면 Jacobian 계산을 그만큼 아낀다."""
    q = np.asarray(q, dtype=float)

    def indices(qq):
        return sg.singularity_indices(robot.jacob0(qq, end=end)[:, :6], robot.jacob0(qq, end=wrist))

    base = base or indices(q)
    grad = {t: np.zeros(len(q)) for t in sg.TYPES}
    for i in range(len(q)):
        dq = q.copy()
        dq[i] += eps
        moved = indices(dq)
        for t in sg.TYPES:
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
        J = robot.jacob0(q, end=sg.END_LINK)
    joints = [[float(v) for v in link_position(robot, q, name)] for name in LINK_NAMES]
    axes = [[float(v) for v in np.asarray(J)[3:, i]] for i in range(6)]
    return {'base': [0.0, 0.0, 0.0], 'joints': joints, 'axes': axes}


def _finite(v):
    """JSON 에 넣을 수 있는 값으로 (inf·nan → None). 브라우저 JSON.parse 는 Infinity 를 못 읽는다."""
    if v is None:
        return None
    v = float(v)
    return v if np.isfinite(v) else None


def _kind_status(kind, index, rate, grad, warn_factor):
    limit = sg.LIMITS[kind]
    return {
        'index': _finite(index),
        'limit': limit,
        'proximity': proximity(kind, index, warn_factor),
        'value': _finite(physical_value(kind, index)),
        'value_limit': physical_value(kind, limit),
        'value_warn': physical_value(kind, min(limit * warn_factor, 1.0)),
        'unit': UNITS[kind],
        'rate': _finite(rate),
        'eta': time_to_limit(index, rate, limit),
        'escape': escape_direction(grad) if grad is not None else None,
    }


def build_status(a, q, qdot=None, grad=None, stamp=None, warn_factor=WARN_FACTOR, geometry=None):
    """analyze() 결과 → 대시보드로 보낼 상태 dict (JSON 직렬화 가능, inf 없음)
    a: analyze() 결과, q: 관절각(rad), qdot: 관절 속도(rad/s, 없으면 None), grad: index_gradient() 결과,
    geometry: arm_geometry() 결과 (로봇팔 그림용, 없으면 None)"""
    kinds = {}
    if a['indices'] is not None:
        for t in sg.TYPES:
            g = grad[t] if grad is not None else None
            rate = float(np.dot(g, qdot)) if g is not None and qdot is not None else None
            kinds[t] = _kind_status(t, a['indices'][t], rate, g, warn_factor)
    if kinds:
        dominant = max(kinds, key=lambda t: kinds[t]['proximity'])
        risk = kinds[dominant]['proximity']
    else:
        dominant, risk = None, None
    return {
        'stamp': stamp,
        'q': [float(v) for v in q],
        'qdot': None if qdot is None else [float(v) for v in qdot],
        'type': sg.label(a['types']),
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
