"""
singularity.py 와 velocity_demo.py 검증 (ROS·Robotics Toolbox 없이 실행 가능)

Toolbox 대신, 같은 M0609 URDF로 직접 만든 NumPy 기구학(conftest.ReferenceRobot)을 쓴다.
"""
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from conftest import ELBOW_NEAR, NORMAL, URDF, WRIST_NEAR, needs_urdf
from singularity_analysis import singularity as sg

pytestmark = needs_urdf


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


def test_arm_only_drops_gripper_and_mimic_joints(gripper_urdf):
    # link_6 뒤에 그리퍼(미믹 관절 포함)를 붙인 URDF 를 줘도 팔 6축만 남아야 함
    arm = ET.fromstring(sg.arm_only_urdf(gripper_urdf))
    assert [j.get('name') for j in arm.findall('joint')] == sg.JOINT_NAMES
    with pytest.raises(ValueError):
        sg.arm_only_urdf(gripper_urdf, end_link='no_such_link')


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
    assert sg.analyze(robot, WRIST_NEAR)['types'] == ['wrist']
    assert sg.analyze(robot, ELBOW_NEAR)['types'] == ['elbow']
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
