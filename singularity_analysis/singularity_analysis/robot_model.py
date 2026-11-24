"""
로봇 모델과 파일 위치

- URDF 정리: arm_only_urdf (그리퍼·곁가지를 떼고 base_link → link_6 팔 사슬만), root_link, read_urdf
- Toolbox 모델 만들기: load_robot (파일), load_robot_from_xml (문자열)
- 순기구학 도우미: to_matrix, link_position
- 기본 위치: default_urdf (M0609 URDF), results_dir (결과 저장 폴더)

URDF 정리와 경로 함수는 ROS·Toolbox 없이 동작한다. Toolbox 는 모델을 실제로 만들 때만 불러온다.
"""
import os
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

try:                                  # ros2 run / 패키지로 설치했을 때
    from .singularity import END_LINK
except ImportError:                   # python3 파일경로로 직접 실행할 때
    from singularity import END_LINK


# ---------------------------------------------------------------------------------------------
# URDF 정리
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


# ---------------------------------------------------------------------------------------------
# Robotics Toolbox 모델
# ---------------------------------------------------------------------------------------------

def toolbox_robot_from_file(path):
    """
    URDF 파일 → Robotics Toolbox 로봇 모델 (파일 그대로, 손대지 않음)
    - Toolbox 1.5 이상: URDF_read() 로 링크를 읽어 Robot 을 직접 만든다 (Toolbox 가 권장하는 방식)
    - 그보다 예전 Toolbox: ERobot.URDF() 를 쓴다. setup.sh 는 NumPy 1.x 와 맞는 Toolbox 1.4.x 를 깔기 때문에
      실제 ROS 환경에서는 이쪽을 탄다.
    """
    import roboticstoolbox as rtb   # 무거운 라이브러리라 실제로 모델을 만들 때만 불러옴 (테스트는 없이도 돌아감)
    try:
        from roboticstoolbox.models.URDF.URDFRobot import URDF_read
    except ImportError:
        return rtb.ERobot.URDF(path)
    elinks, name, _ = URDF_read(path)
    return rtb.Robot(elinks, name=name)


def load_robot_from_xml(urdf_xml):
    """URDF 문자열 → Toolbox 로봇 모델 (팔 6축만). 그리퍼가 달린 URDF(m0609_rg2)를 줘도 항상 6축 모델이 된다."""
    # Toolbox 는 파일 경로로 읽으므로 임시 파일에 써서 넘김
    with tempfile.NamedTemporaryFile('w', suffix='.urdf', delete=False, encoding='utf-8') as tmp:
        tmp.write(arm_only_urdf(urdf_xml))
        path = tmp.name
    try:
        return toolbox_robot_from_file(path)
    finally:
        os.remove(path)


def load_robot(urdf_path):
    """URDF/xacro 파일 → Toolbox 로봇 모델 (팔 6축만). 파일이 없으면 안내 메시지와 함께 종료."""
    if not os.path.isfile(urdf_path):
        sys.exit(f"URDF 파일을 찾을 수 없습니다: {urdf_path}\n"
                 "경로를 생략하면 my_robot_description 패키지의 urdf/m0609.urdf.xacro 를 씁니다.")
    return load_robot_from_xml(read_urdf(urdf_path))


def to_matrix(T):
    """Toolbox 의 SE3 또는 4x4 배열 → 4x4 numpy 배열"""
    return np.asarray(T.A if hasattr(T, 'A') else T, dtype=float)


def link_position(robot, q, link):
    """관절각 q 에서 link 원점의 위치 (base 좌표계, m)"""
    return to_matrix(robot.fkine(q, end=link))[:3, 3]


# ---------------------------------------------------------------------------------------------
# 기본 위치
# ---------------------------------------------------------------------------------------------

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
