"""
실시간 대시보드 관련 검증 (ROS 없이 실행)

- status.py: 지표 → 물리량·근접도(%)·도달 예상 시간·벗어나는 방향, 상태 JSON
- 분석 노드: 실제 로봇용 파라미터(관절 토픽·URDF 파일), /singularity/status 발행·빈도 제한·관절 속도
- dashboard_server: index.html 을 HTTP 로 내보내는지
"""
import importlib
import json
import math
import threading
import types
import urllib.request

import numpy as np
import pytest

from conftest import ELBOW_NEAR, NORMAL, URDF, WRIST_NEAR, needs_urdf
from singularity_analysis import singularity as sg
from singularity_analysis import status as st

# ---------------------------------------------------------------- 물리량·근접도


def test_physical_values():
    assert st.physical_value('wrist', math.sin(math.radians(10))) == pytest.approx(10.0)
    assert st.physical_value('elbow', 1.0) == pytest.approx(90.0)
    assert st.physical_value('wrist', 1.2) == pytest.approx(90.0)          # 수치 오차로 1을 넘어도 안전
    assert st.physical_value('shoulder', 0.05) == pytest.approx(0.05 * sg.ARM_LENGTH * 100)   # 약 3.9 cm


@pytest.mark.parametrize('kind', sg.TYPES)
def test_proximity_scale(kind):
    limit = sg.LIMITS[kind]
    assert st.proximity(kind, limit * st.WARN_FACTOR) == 0.0      # 경고 시작
    assert st.proximity(kind, 1.0) == 0.0                         # 멀리
    assert st.proximity(kind, limit) == 1.0                       # 임계값 = 특이점 근처로 분류되는 경계
    assert st.proximity(kind, 0.0) == 1.0
    mid = limit * (1 + st.WARN_FACTOR) / 2
    assert st.proximity(kind, mid) == pytest.approx(0.5)


def test_level_matches_classification_boundary():
    assert st.level_of(None) == 'safe'
    assert st.level_of(0.0) == 'safe'
    assert st.level_of(0.3) == 'caution'
    assert st.level_of(1.0) == 'danger'


def test_time_to_limit():
    assert st.time_to_limit(0.3, -0.1, 0.1) == pytest.approx(2.0)
    assert st.time_to_limit(0.3, 0.1, 0.1) is None        # 멀어지는 중
    assert st.time_to_limit(0.05, -0.1, 0.1) is None      # 이미 안쪽
    assert st.time_to_limit(0.3, None, 0.1) is None       # 속도 모름
    assert st.time_to_limit(0.9, -0.01, 0.1) is None      # 30초보다 멂


def test_escape_direction():
    assert st.escape_direction([0, 0.1, 0, 0, -2.0, 0]) == {'joint': 5, 'direction': -1}
    assert st.escape_direction(np.zeros(6)) is None


# ---------------------------------------------------------------- 기울기·상태 (M0609 모델)

@needs_urdf
def test_gradient_matches_central_difference(robot):
    q = np.array([0.3, -0.5, 1.2, 0.4, 0.8, -0.2])
    grad = st.index_gradient(robot, q)
    h = 1e-4
    for i in range(6):
        e = np.eye(6)[i] * h
        up = sg.analyze(robot, q + e)['indices']
        dn = sg.analyze(robot, q - e)['indices']
        for t in sg.TYPES:
            assert grad[t][i] == pytest.approx((up[t] - dn[t]) / (2 * h), abs=1e-3)


@needs_urdf
def test_wrist_escape_is_joint5(robot):
    # q5 = +0.05 에서 손목 특이점(q5 = 0)을 벗어나려면 q5 를 + 방향으로
    grad = st.index_gradient(robot, WRIST_NEAR)
    assert st.escape_direction(grad['wrist']) == {'joint': 5, 'direction': 1}
    q = list(WRIST_NEAR)
    q[4] = -0.05
    assert st.escape_direction(st.index_gradient(robot, q)['wrist']) == {'joint': 5, 'direction': -1}


@needs_urdf
def test_status_eta_for_wrist_approach(robot):
    # q5 = 0.2 rad 에서 -0.1 rad/s 로 손목 특이점에 다가감 → 지표 ≈ |sin q5| 가 0.1 이 되는 시간
    q = np.array([0, -0.3, 1.2, 0, 0.2, 0])
    qdot = np.array([0, 0, 0, 0, -0.1, 0])
    a = sg.analyze(robot, q)
    s = st.build_status(a, q, qdot=qdot, grad=st.index_gradient(robot, q, base=a['indices']))
    w = s['kinds']['wrist']
    assert w['rate'] < 0
    expected = (math.sin(0.2) - sg.WRIST_LIMIT) / (0.1 * math.cos(0.2))   # 1차 근사
    assert w['eta'] == pytest.approx(expected, rel=0.05)
    assert s['kinds']['elbow']['eta'] is None                              # 팔은 안 움직임
    assert s['joint_eta'][3:] == [w['eta']] * 3 and s['joint_eta'][1] is None   # 손목 관절 J4~J6 에만 예상 시간


@needs_urdf
@pytest.mark.parametrize('q, kind, level', [(NORMAL, None, 'safe'), (WRIST_NEAR, 'wrist', 'danger'),
                                            (ELBOW_NEAR, 'elbow', 'danger')])
def test_status_summary(robot, q, kind, level):
    a = sg.analyze(robot, q)
    s = st.build_status(a, q, grad=st.index_gradient(robot, q))
    assert s['level'] == level
    assert s['type'] == sg.label(a['types'])
    if kind:
        assert s['dominant'] == kind and s['risk'] == 1.0
    assert set(s['kinds']) == set(sg.TYPES)
    for t, d in s['kinds'].items():
        assert d['value'] == pytest.approx(st.physical_value(t, a['indices'][t]))
        assert d['rate'] is None and d['eta'] is None     # 속도를 안 줬음


@needs_urdf
def test_status_json_has_no_infinity(robot):
    q = [0, -0.3, 1.2, 0, 0.0, 0]                       # 정확히 손목 특이점 → 조건수가 매우 크거나 inf
    a = sg.analyze(robot, q)
    a['condition_number'] = float('inf')                 # 완전 특이인 경우를 강제로
    text = json.dumps(st.build_status(a, q), allow_nan=False)
    assert json.loads(text)['condition_number'] is None


def test_status_without_wrist_link():
    a = {'indices': None, 'types': [], 'manipulability': 0.02, 'condition_number': 20.0, 'sigma_min': 0.1}
    s = st.build_status(a, [0.0] * 6)
    assert s['kinds'] == {} and s['risk'] is None and s['level'] == 'safe'
    assert s['joint_risk'] is None and s['geometry'] is None


def test_joint_risk_follows_involved_joints():
    kinds = {'wrist': {'proximity': 0.8}, 'elbow': {'proximity': 0.3}, 'shoulder': {'proximity': 0.0}}
    assert st.joint_risk(kinds) == [0.0, 0.3, 0.3, 0.8, 0.8, 0.8]
    assert st.joint_risk({}) is None


def test_joint_eta_takes_earliest_involved_condition():
    kinds = {'wrist': {'eta': 2.0}, 'elbow': {'eta': None}, 'shoulder': {'eta': 0.5}}
    assert st.joint_eta(kinds) == [0.5, None, None, 2.0, 2.0, 2.0]
    assert st.joint_eta({}) is None


@needs_urdf
@pytest.mark.parametrize('q, hot', [(NORMAL, []), (WRIST_NEAR, [4, 5, 6]), (ELBOW_NEAR, [2, 3])])
def test_joint_risk_in_status(robot, q, hot):
    a = sg.analyze(robot, q)
    risk = st.build_status(a, q)['joint_risk']
    assert [j for j in range(1, 7) if risk[j - 1] >= 1.0] == hot   # 특이점 근처면 관여하는 관절만 빨강(100%)


@needs_urdf
def test_arm_geometry_matches_kinematics(robot):
    q = np.array([0.3, -0.5, 1.2, 0.4, 0.8, -0.2])
    g = st.arm_geometry(robot, q)
    J = robot.jacob0(q)
    assert len(g['joints']) == 6 and len(g['axes']) == 6
    for i, name in enumerate(st.LINK_NAMES):
        assert g['joints'][i] == pytest.approx(robot.fkine(q, end=name)[:3, 3])
        assert g['axes'][i] == pytest.approx(J[3:, i])
        assert np.linalg.norm(g['axes'][i]) == pytest.approx(1.0)
    # M0609 치수: 1번 관절 높이 0.1345 m, 상완 0.411 m, 전완 0.368 m (J4·J5 는 손목 중심에서 겹침), 손목→J6 0.121 m
    j = np.array(g['joints'])
    assert j[0] == pytest.approx([0, 0, 0.1345], abs=1e-6)
    assert np.linalg.norm(j[2] - j[1]) == pytest.approx(0.411, abs=1e-3)
    assert np.linalg.norm(j[4] - j[2]) == pytest.approx(0.368, abs=1e-3)
    assert np.linalg.norm(j[4] - j[3]) == pytest.approx(0.0, abs=1e-6)
    assert np.linalg.norm(j[5] - j[4]) == pytest.approx(0.121, abs=1e-3)
    json.dumps(g, allow_nan=False)


# ---------------------------------------------------------------- 분석 노드

def make_node(ros_stubs, tmp_path, urdf=None, **params):
    if urdf is not None:
        ros_stubs.urdf_to_send = urdf
    ros_stubs.params.update(results_dir=str(tmp_path), **params)
    an = importlib.import_module('singularity_analysis.m0609_ros_analyzer')
    n = an.M0609SingularityMonitor()
    n.ros, n.module = ros_stubs, an
    return n


def joint_msg(q, velocity=None):
    return types.SimpleNamespace(name=list(sg.JOINT_NAMES), position=list(q),
                                 velocity=[] if velocity is None else list(velocity))


def statuses(ros):
    return [json.loads(m.data) for m in ros.pubs['/singularity/status']]


@needs_urdf
def test_status_topic_from_analyzer(ros_stubs, gripper_urdf, tmp_path):
    n = make_node(ros_stubs, tmp_path, urdf=gripper_urdf, status_rate_hz=0.0)
    try:
        for q in (NORMAL, WRIST_NEAR):
            ros_stubs.subs['/joint_states'](joint_msg(q))
        s = statuses(ros_stubs)
        assert [x['type'] for x in s] == ['normal', 'wrist']
        assert s[1]['level'] == 'danger' and s[1]['dominant'] == 'wrist'
        assert s[1]['kinds']['wrist']['escape'] == {'joint': 5, 'direction': 1}
        assert s[1]['q'] == pytest.approx(WRIST_NEAR)
        assert s[1]['joint_risk'][3:] == [1.0, 1.0, 1.0]                 # 손목 관절 J4~J6 빨강
        assert len(s[1]['geometry']['joints']) == 6                      # 그리퍼 URDF 여도 팔 6관절 위치
    finally:
        n.destroy_node()


@needs_urdf
def test_status_rate_limit(ros_stubs, gripper_urdf, tmp_path, monkeypatch):
    n = make_node(ros_stubs, tmp_path, urdf=gripper_urdf, status_rate_hz=10.0)
    clock = iter([0.0, 0.0, 0.05, 0.05, 0.11, 0.11])      # 메시지마다 (속도 추정, 상태 발행) 두 번 읽음
    monkeypatch.setattr(n.module.time, 'monotonic', lambda: next(clock))
    try:
        for _ in range(3):
            ros_stubs.subs['/joint_states'](joint_msg(NORMAL))
        assert len(ros_stubs.pubs['/singularity/status']) == 2   # 0.05초 뒤 메시지는 건너뜀
        assert len(ros_stubs.pubs['/singularity/type']) == 3     # 다른 지표 토픽은 매번
    finally:
        n.destroy_node()


@needs_urdf
def test_joint_velocity_from_message_and_estimate(ros_stubs, gripper_urdf, tmp_path, monkeypatch):
    n = make_node(ros_stubs, tmp_path, urdf=gripper_urdf, status_rate_hz=0.0)
    try:
        # 1) 드라이버처럼 velocity 가 들어오면 그대로 사용
        v = [0.1, 0, 0, 0, -0.2, 0]
        ros_stubs.subs['/joint_states'](joint_msg(NORMAL, velocity=v))
        assert statuses(ros_stubs)[-1]['qdot'] == pytest.approx(v)
        # 2) 슬라이더처럼 velocity 가 없으면 차분으로 추정 (첫 메시지는 모름)
        n._prev_q, n._qdot, n._last_status = None, None, None   # 아래 가짜 시계로 처음부터
        clock = iter([10.0, 10.0, 10.1, 10.1])
        monkeypatch.setattr(n.module.time, 'monotonic', lambda: next(clock))
        q2 = list(NORMAL)
        q2[4] += 0.01
        ros_stubs.subs['/joint_states'](joint_msg(NORMAL))
        assert statuses(ros_stubs)[-1]['qdot'] is None
        ros_stubs.subs['/joint_states'](joint_msg(q2))
        assert statuses(ros_stubs)[-1]['qdot'] == pytest.approx([0, 0, 0, 0, 0.1, 0])
    finally:
        n.destroy_node()


@needs_urdf
def test_real_robot_params(ros_stubs, tmp_path):
    # 실제 로봇: URDF 는 파일에서, 관절값은 드라이버 토픽에서 (robot_description 토픽을 기다리지 않음)
    n = make_node(ros_stubs, tmp_path, urdf_file=URDF, joint_states_topic='/dsr01/joint_states', status_rate_hz=0.0)
    try:
        assert '/robot_description' not in ros_stubs.subs
        assert '/joint_states' not in ros_stubs.subs
        ros_stubs.subs['/dsr01/joint_states'](joint_msg(WRIST_NEAR))
        assert statuses(ros_stubs)[-1]['type'] == 'wrist'
    finally:
        n.destroy_node()


def test_missing_urdf_file_stops(ros_stubs, tmp_path):
    with pytest.raises(RuntimeError):
        make_node(ros_stubs, tmp_path, urdf_file=str(tmp_path / 'nope.urdf'))


# ---------------------------------------------------------------- 대시보드 웹 서버

def test_dashboard_server_serves_index():
    from singularity_analysis import dashboard_server as ds
    directory = ds.dashboard_dir()
    server = ds.make_server('127.0.0.1', 0, directory)
    th = threading.Thread(target=server.serve_forever, daemon=True)
    th.start()
    try:
        port = server.server_address[1]
        html = urllib.request.urlopen(f'http://127.0.0.1:{port}/', timeout=5).read().decode('utf-8')
        assert '/singularity/status' in html and '<title>' in html
    finally:
        server.shutdown()
        server.server_close()
