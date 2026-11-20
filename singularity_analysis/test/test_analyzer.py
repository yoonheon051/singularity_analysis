"""
실시간 분석 노드(m0609_ros_analyzer) 검증 — ROS 없이 가짜 rclpy(conftest.ros_stubs)로 노드를 처음부터 실행한다.

확인하는 것
- 그리퍼 URDF 를 받아도 팔 6축 모델을 만들고, /joint_states 의 관절을 이름으로 찾는지
- CSV 기록(헤더, 값, 1초 flush, 종료 시 닫기), 지표 토픽, RViz 타원체 마커
- 특이점 종류가 바뀔 때만 경고, 관절 이름이 안 맞으면 경고 한 번
"""
import csv
import importlib
import types

import numpy as np
import pytest

from conftest import ELBOW_NEAR, NORMAL, WRIST_NEAR, needs_urdf
from singularity_analysis import singularity as sg

pytestmark = needs_urdf


@pytest.fixture
def node(ros_stubs, gripper_urdf, tmp_path):
    """그리퍼 URDF 를 /robot_description 으로 받아 시작한 분석 노드"""
    ros_stubs.urdf_to_send = gripper_urdf
    ros_stubs.params['results_dir'] = str(tmp_path)
    an = importlib.import_module('singularity_analysis.m0609_ros_analyzer')
    n = an.M0609SingularityMonitor()
    n.ros, n.module = ros_stubs, an
    yield n
    n.destroy_node()


def send(node, q, names=None, extra=None):
    """관절값을 /joint_states 로 보냄. names 를 주면 그 순서로, extra 는 그리퍼 관절 등 추가 값"""
    values = dict(zip(sg.JOINT_NAMES, q), **(extra or {}))
    names = names or list(values)
    node.ros.subs['/joint_states'](types.SimpleNamespace(name=names, position=[values[n] for n in names]))


def rows(node):
    node.flush_csv()
    with open(node.results_path, newline='') as f:
        return list(csv.DictReader(f))


def warnings(node):
    return [m for level, m, _ in node.ros.logs if level == 'warn']


def test_gripper_urdf_gives_six_joint_model(node):
    assert node.dof == 6
    assert node.frame_id == 'base_link'
    assert node.wrist == sg.WRIST_LINK
    assert node.results_path.endswith('.csv') and 'analysis_' in node.results_path
    assert not [m for level, m, _ in node.ros.logs if level == 'error']


def test_joints_are_matched_by_name_with_gripper_joints_mixed_in(node, robot):
    q = [0.1, -0.3, 1.2, 0.2, 0.8, -0.1]
    shuffled = ['right_finger_joint', 'joint_4', 'joint_2', 'left_finger_joint', 'joint_6', 'joint_1', 'joint_5', 'joint_3']
    send(node, q, names=shuffled, extra={'left_finger_joint': 0.4, 'right_finger_joint': 0.4})
    row = rows(node)[0]
    assert [float(row[f'q{i}']) for i in range(1, 7)] == pytest.approx(q)
    expected = sg.analyze(robot, q)
    assert float(row['manipulability']) == pytest.approx(expected['manipulability'])
    assert float(row['condition_number']) == pytest.approx(expected['condition_number'])
    assert row['singularity_type'] == 'normal'


def test_csv_header_flush_and_close(node):
    with open(node.results_path, newline='') as f:
        assert next(csv.reader(f)) == node.module.CSV_HEADER
    send(node, NORMAL)
    assert len(rows(node)) == 1
    assert any(sec == node.module.CSV_FLUSH_SEC for sec, _ in node.ros.timers)   # 1초 flush 타이머
    node.destroy_node()
    assert node.csv_file.closed


def test_type_change_warnings_and_csv_types(node):
    for q in (NORMAL, WRIST_NEAR, WRIST_NEAR, NORMAL, ELBOW_NEAR):
        send(node, q)
    assert [r['singularity_type'] for r in rows(node)] == ['normal', 'wrist', 'wrist', 'normal', 'elbow']
    entered = [w for w in warnings(node) if '진입' in w]
    assert len(entered) == 2 and 'wrist' in entered[0] and 'elbow' in entered[1]   # 같은 종류가 이어지면 다시 안 알림


def test_metric_topics_match_csv(node):
    for q in (NORMAL, WRIST_NEAR):
        send(node, q)
    data = rows(node)
    pubs = node.ros.pubs
    assert [m.data for m in pubs['/singularity/type']] == [r['singularity_type'] for r in data]
    assert [m.data for m in pubs['/singularity/manipulability']] == pytest.approx([float(r['manipulability']) for r in data])
    assert [m.data for m in pubs['/singularity/sigma_min']] == pytest.approx([float(r['sigma_min']) for r in data])


def test_ellipsoid_markers(node, robot):
    send(node, WRIST_NEAR)
    msg = node.ros.pubs['/manipulability_ellipsoid'][-1]
    arm, wrist = msg.markers
    assert (arm.ns, wrist.ns) == ('arm', 'wrist')
    assert arm.header.frame_id == 'base_link'
    center = robot.fkine(np.array(WRIST_NEAR), end=sg.WRIST_LINK)[:3, 3]      # 손목 중심에 그림
    assert [arm.pose.position.x, arm.pose.position.y, arm.pose.position.z] == pytest.approx(center)
    for m in (arm, wrist):
        o = m.pose.orientation
        assert o.x ** 2 + o.y ** 2 + o.z ** 2 + o.w ** 2 == pytest.approx(1.0)
    assert wrist.color.r > 0.5 and arm.color.r < 0.5          # 손목 특이점 → 손목 타원체만 빨강
    assert min(wrist.scale.x, wrist.scale.y, wrist.scale.z) < 0.03   # 그리고 납작함


def test_missing_joint_names_warn_once(node):
    for _ in range(3):
        node.ros.subs['/joint_states'](types.SimpleNamespace(name=['joint1', 'joint2'], position=[0.0, 0.0]))
    assert len([w for w in warnings(node) if '찾지 못해' in w]) == 1
    assert rows(node) == []


def test_bad_urdf_stops_with_error(ros_stubs, tmp_path):
    ros_stubs.urdf_to_send = '<robot name="x"><link name="base_link"/></robot>'   # link_6 없음
    ros_stubs.params['results_dir'] = str(tmp_path)
    an = importlib.import_module('singularity_analysis.m0609_ros_analyzer')
    with pytest.raises(RuntimeError):
        an.M0609SingularityMonitor()
    assert list(tmp_path.iterdir()) == []   # 실패하면 빈 CSV 가 남지 않음
