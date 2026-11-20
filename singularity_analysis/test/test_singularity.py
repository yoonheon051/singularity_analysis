"""
singularity.py 와 velocity_demo.py 검증 (ROS·Robotics Toolbox 없이 실행 가능)

Toolbox 대신, 같은 M0609 URDF로 직접 만든 NumPy 기구학(ReferenceRobot)을 쓴다.
    colcon test --packages-select singularity_analysis
    또는  cd singularity_analysis && python3 -m pytest test/
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG_DIR)
from singularity_analysis import singularity as sg  # noqa: E402

URDF = os.path.join(PKG_DIR, '..', 'my_robot_description', 'urdf', 'm0609.urdf.xacro')
pytestmark = pytest.mark.skipif(not os.path.isfile(URDF), reason='my_robot_description URDF 가 없음')

NORMAL = [0, -0.3, 1.2, 0, 1.0, 0]


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(a, t):
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K


class ReferenceRobot:
    """URDF 를 직접 읽어 만든 순기구학·기하학적 Jacobian (Toolbox 의 fkine/jacob0 와 같은 형태)"""

    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        self.joints = {}
        for j in root.findall('joint'):
            o, ax = j.find('origin'), j.find('axis')
            self.joints[j.find('child').get('link')] = dict(
                type=j.get('type'), parent=j.find('parent').get('link'),
                xyz=np.array([float(v) for v in (o.get('xyz') if o is not None else '0 0 0').split()]),
                rpy=[float(v) for v in (o.get('rpy') if o is not None and o.get('rpy') else '0 0 0').split()],
                axis=np.array([float(v) for v in ax.get('xyz').split()]) if ax is not None else None)
        self.link_dict = {l.get('name'): None for l in root.findall('link')}
        self.n = sum(j['type'] == 'revolute' for j in self.joints.values())

    def _frames(self, q, end):
        chain, link = [], end
        while link in self.joints:
            chain.append(self.joints[link])
            link = self.joints[link]['parent']
        T, i, axes = np.eye(4), 0, []
        for j in reversed(chain):
            A = np.eye(4); A[:3, :3] = _rpy(*j['rpy']); A[:3, 3] = j['xyz']; T = T @ A
            if j['type'] == 'revolute':
                axes.append((T[:3, :3] @ j['axis'], T[:3, 3].copy()))
                R = np.eye(4); R[:3, :3] = _axis_angle(j['axis'], q[i]); T = T @ R
                i += 1
        return T, axes

    def fkine(self, q, end=sg.END_LINK):
        return self._frames(q, end)[0]

    def jacob0(self, q, end=sg.END_LINK):
        T, axes = self._frames(q, end)
        return np.array([np.r_[np.cross(z, T[:3, 3] - p), z] for z, p in axes]).T


@pytest.fixture(scope='module')
def robot():
    return ReferenceRobot(sg.arm_only_urdf(sg.read_urdf(URDF)))


def shoulder_pose(robot):
    """팔꿈치를 굽힌 채(q3=1.5) 손목 중심이 1번 축 쪽으로 오는 q2 를 찾음"""
    q2 = min(np.linspace(-1.5, 0.5, 2001), key=lambda v: sg.analyze(robot, [0, v, 1.5, 0, 1.0, 0])['indices']['shoulder'])
    return [0, q2, 1.5, 0, 1.0, 0]


# ---------------------------------------------------------------- URDF 정리

def test_arm_only_keeps_base_to_link6_chain():
    xml = sg.arm_only_urdf(sg.read_urdf(URDF))
    root = ET.fromstring(xml)
    links = [l.get('name') for l in root.findall('link')]
    assert set(links) == {'base_link', 'link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6'}   # 'base', 'tool0' 제거
    assert [j.get('name') for j in root.findall('joint')] == sg.JOINT_NAMES
    assert not root.findall('.//visual') and not root.findall('.//collision')
    assert sg.root_link(xml) == 'base_link'


def test_arm_only_drops_gripper_and_mimic_joints():
    # link_6 뒤에 그리퍼(미믹 관절 포함)를 붙인 URDF 를 만들어도 팔 6축만 남아야 함
    root = ET.fromstring(sg.read_urdf(URDF))
    for name in ('gripper_base', 'left_finger', 'right_finger'):
        ET.SubElement(root, 'link', {'name': name})
    j = ET.SubElement(root, 'joint', {'name': 'tool_mount', 'type': 'fixed'})
    ET.SubElement(j, 'parent', {'link': 'link_6'}); ET.SubElement(j, 'child', {'link': 'gripper_base'})
    for side, mimic in (('left', None), ('right', 'left_finger_joint')):
        j = ET.SubElement(root, 'joint', {'name': f'{side}_finger_joint', 'type': 'revolute'})
        ET.SubElement(j, 'parent', {'link': 'gripper_base'}); ET.SubElement(j, 'child', {'link': f'{side}_finger'})
        ET.SubElement(j, 'axis', {'xyz': '0 0 1'})
        if mimic:
            ET.SubElement(j, 'mimic', {'joint': mimic})
    arm = ET.fromstring(sg.arm_only_urdf(ET.tostring(root, encoding='unicode')))
    assert [j.get('name') for j in arm.findall('joint')] == sg.JOINT_NAMES
    with pytest.raises(ValueError):
        sg.arm_only_urdf(ET.tostring(root, encoding='unicode'), end_link='no_such_link')


# ---------------------------------------------------------------- 기구학·지표

def test_jacobian_matches_finite_difference(robot):
    q = np.array([0.3, -0.5, 1.2, 0.4, 0.8, -0.2])
    J, e = robot.jacob0(q), 1e-6
    num = np.column_stack([(robot.fkine(q + e * np.eye(6)[i])[:3, 3] - robot.fkine(q)[:3, 3]) / e for i in range(6)])
    assert np.abs(J[:3] - num).max() < 1e-5


@pytest.mark.parametrize('q', [NORMAL, [0.3, -0.5, 1.2, 0.4, 0.8, -0.2], [1.0, 0.4, 0.3, -1.2, 0.2, 2.0]])
def test_determinant_factorizes_arm_times_wrist(robot, q):
    # URDF 의 90° 회전이 1.571(π/2 근사)로 적혀 있어 손목 세 축이 약 0.03 mm 어긋남 → 상대 오차 1e-4 수준까지만 일치
    J = robot.jacob0(np.array(q))
    J11 = robot.jacob0(np.array(q), end=sg.WRIST_LINK)[:3, :3]
    assert np.linalg.det(J) == pytest.approx(np.linalg.det(J11) * np.linalg.det(J[3:, 3:6]), rel=1e-3, abs=1e-9)


def test_manipulability_is_product_of_singular_values(robot):
    a = sg.analyze(robot, NORMAL)
    assert a['manipulability'] == pytest.approx(np.prod(np.linalg.svd(a['J'], compute_uv=False)))


def test_classification(robot):
    assert sg.analyze(robot, NORMAL)['types'] == []
    assert sg.analyze(robot, [0, -0.3, 1.2, 0, 0.05, 0])['types'] == ['wrist']
    assert sg.analyze(robot, [0, 0.5, 0.05, 0, 1.0, 0])['types'] == ['elbow']
    assert sg.analyze(robot, shoulder_pose(robot))['types'] == ['shoulder']
    assert sg.label(sg.analyze(robot, [0, -0.3, 1.2, 0, 0.12, 0])['types']) == 'normal'   # 임계값 0.1 바깥


def test_each_singularity_flattens_only_its_ellipsoid(robot):
    def flat(q):
        a = sg.analyze(robot, q)
        (_, s_arm), (_, s_wrist) = a['ellipsoid_arm'], a['ellipsoid_wrist']
        return s_arm[-1] / s_arm[0], s_wrist[-1] / s_wrist[0]
    arm0, wrist0 = flat(NORMAL)
    arm, wrist = flat([0, -0.3, 1.2, 0, 0.0, 0])        # 손목 특이점
    assert wrist < 1e-6 and arm == pytest.approx(arm0)
    arm, wrist = flat([0, 0.5, 0.0, 0, 1.0, 0])          # 팔꿈치 특이점 (1.571 근사 때문에 정확히 0은 아님)
    assert arm < 1e-3 and wrist == pytest.approx(wrist0)
    arm, wrist = flat(shoulder_pose(robot))               # 어깨 특이점
    assert arm < 1e-2 and wrist == pytest.approx(wrist0)


def test_quaternion_round_trip(robot):
    U, _ = sg.ellipsoid(robot.jacob0(np.array([0.3, -0.5, 1.2, 0.4, 0.8, -0.2]))[:3])
    x, y, z, w = sg.rotation_to_quaternion(U)
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    assert np.allclose(R, U) and np.linalg.det(U) == pytest.approx(1.0)


# ---------------------------------------------------------------- 속도 폭주 vs DLS

def test_dls_keeps_joint_speed_bounded(robot):
    pytest.importorskip('matplotlib')
    from singularity_analysis import velocity_demo as vd
    inverse = vd.simulate(robot, 'inverse', 0.05, 0.05)
    limited = vd.simulate(robot, 'inverse_limited', 0.05, 0.05)
    dls = vd.simulate(robot, 'dls', 0.05, 0.05)
    assert inverse['speed'].max() > 100 * vd.JOINT_SPEED_LIMIT     # 역행렬: 손목 특이점에서 폭주
    assert limited['speed'].max() <= vd.JOINT_SPEED_LIMIT + 1e-9    # 속도 제한: 폭주는 막지만
    assert limited['pos_err'].max() > 10                             #   경로를 1 cm 넘게 벗어남
    assert dls['speed'].max() < 1.0 and dls['pos_err'].max() < 1.0  # DLS: 느린 관절 속도로 1 mm 안쪽 추종
