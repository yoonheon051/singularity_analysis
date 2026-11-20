"""
실제 Robotics Toolbox 와 교차 검증 (Toolbox 를 불러올 수 있는 환경에서만 실행, 없으면 건너뜀)

다른 테스트는 Toolbox 대신 conftest.ReferenceRobot(URDF 로 직접 만든 NumPy 기구학)을 쓴다.
여기서는 두 결과가 같은지 확인해서, 그 대역이 믿을 만하다는 근거를 남긴다.
GitHub Actions(.github/workflows/test.yml)에서는 Toolbox 를 설치하므로 항상 실행된다.
"""
import importlib

import numpy as np
import pytest

from conftest import NORMAL, URDF, URDF_RG2, WRIST_NEAR, ReferenceRobot, needs_urdf
from singularity_analysis import singularity as sg

pytestmark = needs_urdf
try:
    import roboticstoolbox as rtb
except Exception as e:   # 설치 안 됨(ModuleNotFoundError)뿐 아니라, 설치됐지만 불러오지 못하는 경우(DLL 차단 등)도 건너뜀
    pytest.skip(f'Robotics Toolbox 를 불러올 수 없음: {type(e).__name__}', allow_module_level=True)

POSES = [NORMAL, WRIST_NEAR, [0.3, -0.5, 1.2, 0.4, 0.8, -0.2], [1.0, 0.4, 0.3, -1.2, 0.2, 2.0], [0, 0, 0, 0, 0, 0]]


def as_matrix(T):
    return np.asarray(T.A if hasattr(T, 'A') else T, dtype=float)


@pytest.fixture(scope='module')
def toolbox_robot():
    from singularity_analysis.calculate_jacobian import load_robot
    return load_robot(URDF)


@pytest.fixture(scope='module')
def reference():
    return ReferenceRobot(sg.arm_only_urdf(sg.read_urdf(URDF)))


def test_arm_only_model_has_six_joints(toolbox_robot):
    assert toolbox_robot.n == 6
    assert {'base_link', sg.WRIST_LINK, sg.END_LINK} <= set(toolbox_robot.link_dict)


@pytest.mark.parametrize('q', POSES)
def test_fkine_and_jacobian_match_reference(toolbox_robot, reference, q):
    q = np.array(q, dtype=float)
    assert as_matrix(toolbox_robot.fkine(q, end=sg.END_LINK)) == pytest.approx(reference.fkine(q), abs=1e-9)
    assert toolbox_robot.jacob0(q, end=sg.END_LINK) == pytest.approx(reference.jacob0(q), abs=1e-9)
    J_wc = toolbox_robot.jacob0(q, end=sg.WRIST_LINK)
    assert J_wc[:, :3] == pytest.approx(reference.jacob0(q, end=sg.WRIST_LINK)[:, :3], abs=1e-9)


@pytest.mark.parametrize('q', POSES[:4])
def test_analysis_matches_reference(toolbox_robot, reference, q):
    a, b = sg.analyze(toolbox_robot, q), sg.analyze(reference, q)
    assert a['types'] == b['types']
    assert a['manipulability'] == pytest.approx(b['manipulability'], rel=1e-9, abs=1e-12)
    assert a['indices'] == pytest.approx(b['indices'], abs=1e-9)


def test_raw_urdf_has_ambiguous_end_effector():
    """README '말단 링크를 link_6으로 지정하는 이유'의 근거: 원본 URDF 를 그대로 불러오면 끝 링크가 둘(base, tool0)"""
    raw = rtb.ERobot.URDF(URDF)
    names = [link.name for link in raw.ee_links]
    assert 'base' in names and 'tool0' in names


def test_gripper_xacro_loads_as_six_joints():
    pytest.importorskip('xacro')
    from singularity_analysis.calculate_jacobian import load_robot
    robot = load_robot(URDF_RG2)
    assert robot.n == 6


def test_velocity_demo_numbers(toolbox_robot):
    """README 'DLS' 표의 숫자가 실제 Toolbox 로도 나오는지"""
    pytest.importorskip('matplotlib')
    vd = importlib.import_module('singularity_analysis.velocity_demo')
    inverse = vd.simulate(toolbox_robot, 'inverse', 0.05, 0.05)
    limited = vd.simulate(toolbox_robot, 'inverse_limited', 0.05, 0.05)
    dls = vd.simulate(toolbox_robot, 'dls', 0.05, 0.05)
    assert inverse['speed'].max() > 1000
    assert limited['pos_err'].max() > 40
    assert dls['speed'].max() < 0.3 and dls['pos_err'].max() < 1.0
