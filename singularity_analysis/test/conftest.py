"""
테스트 공통 도구

- ReferenceRobot: Robotics Toolbox 대신 쓰는 NumPy 기구학 (같은 M0609 URDF를 직접 읽어 fkine/jacob0 계산)
- ros_stubs: rclpy·메시지 패키지를 흉내 내는 가짜 모듈 → ROS 없이 분석 노드를 처음부터 실행해 볼 수 있음
- gripper_urdf: M0609 끝에 그리퍼(미믹 관절 포함)를 붙인 URDF (m0609_rg2 와 같은 상황)
"""
import math
import os
import sys
import types
import xml.etree.ElementTree as ET

import numpy as np
import pytest

os.environ.setdefault('MPLBACKEND', 'Agg')   # 그래프 테스트: 화면 없이 그림만 저장

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)
from singularity_analysis import singularity as sg  # noqa: E402

URDF = os.path.normpath(os.path.join(PKG_DIR, '..', 'my_robot_description', 'urdf', 'm0609.urdf.xacro'))
URDF_RG2 = os.path.normpath(os.path.join(PKG_DIR, '..', 'my_robot_description', 'urdf', 'm0609_rg2.urdf.xacro'))
HAS_URDF = os.path.isfile(URDF)
needs_urdf = pytest.mark.skipif(not HAS_URDF, reason='my_robot_description URDF 가 없음')

NORMAL = [0, -0.3, 1.2, 0, 1.0, 0]                 # 특이점이 아닌 자세
WRIST_NEAR = [0, -0.3, 1.2, 0, 0.05, 0]            # 손목 특이점 근처 (q5 ≈ 0)
ELBOW_NEAR = [0, 0.5, 0.05, 0, 1.0, 0]             # 팔꿈치 특이점 근처 (q3 ≈ 0, 팔이 펴짐)


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

    @classmethod
    def from_file(cls, path):
        """Toolbox 의 ERobot.URDF(path) 자리에 끼워 넣을 때 사용"""
        with open(path, encoding='utf-8') as f:
            return cls(f.read())

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


@pytest.fixture(scope='session')
def robot():
    if not HAS_URDF:
        pytest.skip('my_robot_description URDF 가 없음')
    return ReferenceRobot(sg.arm_only_urdf(sg.read_urdf(URDF)))


@pytest.fixture(scope='session')
def gripper_urdf():
    """M0609 + 그리퍼(link_6 에 고정, 손가락 관절 2개 중 하나는 미믹)"""
    if not HAS_URDF:
        pytest.skip('my_robot_description URDF 가 없음')
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
    return ET.tostring(root, encoding='unicode')


@pytest.fixture
def toolbox_stub(monkeypatch):
    """roboticstoolbox 를 ReferenceRobot 으로 대신 (Toolbox 가 없어도, 있어도 같은 결과로 테스트)"""
    mod = types.ModuleType('roboticstoolbox')
    mod.ERobot = types.SimpleNamespace(URDF=ReferenceRobot.from_file)
    monkeypatch.setitem(sys.modules, 'roboticstoolbox', mod)
    return mod


class FakeROS:
    """분석 노드가 쓰는 rclpy 기능만 흉내: 구독 콜백 보관, 발행 기록, 로그 기록"""

    def __init__(self):
        self.subs, self.pubs, self.logs, self.timers = {}, {}, [], []
        self.urdf_to_send = None
        self.params = {}


@pytest.fixture
def ros_stubs(monkeypatch, toolbox_stub):
    ros = FakeROS()
    NS = types.SimpleNamespace

    class Logger:
        def info(self, m, **k): ros.logs.append(('info', m, k))
        def warn(self, m, **k): ros.logs.append(('warn', m, k))
        def error(self, m, **k): ros.logs.append(('error', m, k))

    class Node:
        def __init__(self, name): pass
        def get_logger(self): return Logger()
        def declare_parameter(self, name, default): return NS(value=ros.params.get(name, default))
        def create_subscription(self, typ, topic, cb, qos): ros.subs[topic] = cb
        def create_publisher(self, typ, topic, qos):
            ros.pubs[topic] = []
            return NS(publish=ros.pubs[topic].append)
        def create_timer(self, sec, cb): ros.timers.append((sec, cb))
        def get_clock(self): return NS(now=lambda: NS(to_msg=lambda: 'stamp'))
        def destroy_node(self): pass

    class Msg:
        def __init__(self): self.data = None

    class Marker:
        SPHERE, ADD = 2, 0
        def __init__(self):
            self.header = NS(); self.pose = NS(position=NS(), orientation=NS()); self.scale = NS(); self.color = NS()

    class MarkerArray:
        def __init__(self): self.markers = []

    def spin_once(node):   # 분석 노드가 /robot_description 을 기다릴 때 URDF 를 넘겨 줌
        if node.urdf_xml is None:
            ros.subs['/robot_description'](NS(data=ros.urdf_to_send))

    mods = {
        'rclpy': dict(spin_once=spin_once, init=lambda args=None: None, shutdown=lambda: None),
        'rclpy.node': dict(Node=Node),
        'rclpy.qos': dict(QoSProfile=lambda depth: NS(), DurabilityPolicy=NS(TRANSIENT_LOCAL=1)),
        'std_msgs': {}, 'std_msgs.msg': dict(String=Msg, Float64=Msg),
        'sensor_msgs': {}, 'sensor_msgs.msg': dict(JointState=object),
        'visualization_msgs': {}, 'visualization_msgs.msg': dict(Marker=Marker, MarkerArray=MarkerArray),
    }
    for name, attrs in mods.items():
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, m)
    # 가짜 모듈로 새로 불러오도록, 이전에 불러온 분석 노드 모듈은 지워 둠 (테스트가 끝나면 원래대로)
    monkeypatch.delitem(sys.modules, 'singularity_analysis.m0609_ros_analyzer', raising=False)
    return ros
