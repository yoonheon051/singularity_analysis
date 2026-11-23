"""
시뮬레이션 또는 실제 M0609 + 특이점 분석 + 실시간 대시보드를 한 번에 실행

    ros2 launch singularity_analysis analysis.launch.py                          # 시뮬레이션 (관절 슬라이더)
    ros2 launch singularity_analysis analysis.launch.py model:=m0609             # 그리퍼 없는 모델
    ros2 launch singularity_analysis analysis.launch.py mode:=real               # 실제 로봇 (/dsr01/joint_states)
    ros2 launch singularity_analysis analysis.launch.py mode:=real joint_states_topic:=/joint_states
    ros2 launch singularity_analysis analysis.launch.py dashboard:=false         # 대시보드 없이
    ros2 launch singularity_analysis analysis.launch.py results_dir:=~/my_runs   # 결과 저장 폴더

mode:=sim (기본) 에서 실행되는 노드
- robot_state_publisher : my_robot_description 의 URDF(xacro) → /robot_description, /tf
- joint_state_publisher_gui : 관절 슬라이더 → /joint_states        (gui:=true 일 때)
- rviz2 : 로봇 모델 + 가동성 타원체(/manipulability_ellipsoid)      (rviz:=true 일 때)
- analyzer : 특이점 분석 노드 (CSV 기록, 지표 토픽, 타원체 마커, /singularity/status)
- rosbridge_websocket + dashboard : 실시간 대시보드 (http://localhost:8080)   (dashboard:=true 일 때)

mode:=real 에서는 두산 드라이버(doosan-robot2)를 따로 먼저 띄워 둔다. 드라이버가 로봇 모델·/tf·RViz 를 맡으므로
이 launch 는 슬라이더·robot_state_publisher·RViz 를 띄우지 않고, 분석 노드는
- 관절값: joint_states_topic (기본 /dsr01/joint_states)
- 로봇 모델: my_robot_description 의 URDF 파일을 직접 읽음 (드라이버가 robot_description 을 어디에 발행하든 상관없음)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

REAL_JOINT_STATES = '/dsr01/joint_states'   # doosan-robot2 기본 네임스페이스(dsr01). 다르면 joint_states_topic:= 으로


def _truthy(value):
    return value.strip().lower() in ('true', '1', 'yes', 'on')


def _nodes(context):
    description_share = get_package_share_directory('my_robot_description')
    analysis_share = get_package_share_directory('singularity_analysis')
    arg = lambda name: LaunchConfiguration(name).perform(context)   # noqa: E731

    mode = arg('mode').strip().lower()
    if mode not in ('sim', 'real'):
        raise RuntimeError(f"mode 는 sim 또는 real 이어야 합니다 (받은 값: {mode})")
    real = mode == 'real'
    model = arg('model')
    xacro_file = os.path.join(description_share, 'urdf', f'{model}.urdf.xacro')

    joint_topic = arg('joint_states_topic') or (REAL_JOINT_STATES if real else '/joint_states')
    gui = _truthy(arg('gui')) and not real
    rviz = _truthy(arg('rviz') or ('false' if real else 'true'))
    dashboard = _truthy(arg('dashboard'))

    analyzer_params = {
        'results_dir': arg('results_dir'),
        'joint_states_topic': joint_topic,
        'status_rate_hz': float(arg('status_rate_hz')),
    }
    if real:
        analyzer_params['urdf_file'] = xacro_file

    nodes = []
    if not real:
        robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
        nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        ))
    if gui:
        nodes.append(Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui', output='screen'))
    if rviz:
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(analysis_share, 'rviz', 'analysis.rviz')],
        ))
    nodes.append(Node(
        package='singularity_analysis',
        executable='analyzer',
        name='m0609_singularity_monitor',
        output='screen',
        parameters=[analyzer_params],
    ))
    if dashboard:
        nodes.append(Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': int(arg('rosbridge_port'))}],
        ))
        nodes.append(Node(
            package='singularity_analysis',
            executable='dashboard',
            name='singularity_dashboard',
            output='screen',
            arguments=['--port', arg('dashboard_port')],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim',
                              description='sim: 관절 슬라이더 시뮬레이션 / real: 실제 로봇(두산 드라이버를 먼저 실행)'),
        DeclareLaunchArgument('model', default_value='m0609_rg2',
                              description='로봇 모델: m0609_rg2 (RG2 그리퍼 포함) 또는 m0609'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='관절 슬라이더 창(joint_state_publisher_gui) 실행 여부 (real 이면 항상 끔)'),
        DeclareLaunchArgument('rviz', default_value='',
                              description='RViz 실행 여부 (비우면 sim: true, real: false)'),
        DeclareLaunchArgument('joint_states_topic', default_value='',
                              description=f'관절 상태 토픽 (비우면 sim: /joint_states, real: {REAL_JOINT_STATES})'),
        DeclareLaunchArgument('results_dir', default_value=os.path.expanduser('~/singularity_results'),
                              description='분석 CSV 저장 폴더'),
        DeclareLaunchArgument('dashboard', default_value='true',
                              description='실시간 대시보드(rosbridge + 웹 페이지) 실행 여부'),
        DeclareLaunchArgument('dashboard_port', default_value='8080', description='대시보드 웹 페이지 포트'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090', description='rosbridge WebSocket 포트'),
        DeclareLaunchArgument('status_rate_hz', default_value='15.0',
                              description='/singularity/status 발행 빈도(Hz)'),
        OpaqueFunction(function=_nodes),
    ])
