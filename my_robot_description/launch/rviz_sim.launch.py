import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # ROS2 Launch 시스템이 인식하는 메인 함수, 이 함수 내부에서 정의돈 노드들이 실행 시 순차적으로 처리

    # 1.패키지 이름 및 경로 설정
    pkg_name = 'my_robot_description'
    pkg_path = get_package_share_directory('my_robot_description')

    # 2. xacro 파일 경로 지정
    # URDF의 확장판인 xacro 파일 지정(로봇의 외형과 물리 정보가 담긴 파일)
    xacro_file = os.path.join(pkg_path, 'urdf', 'm0609_rg2.urdf.xacro')

    # 3. xacro 명령 실행
    # Command: 런치 시스템이 실행될 때 해당 명령어를 백그라운드에서 실행해 결과를 가져옴
    robot_description_content = Command(['xacro ', xacro_file])

    # 4. robot_state_publisher 노드 설정
    # 로봇의 URDF 정보를 받아 /tf(좌표계 변환) 정보를 계산하고 발행
    # 로봇 모델이 Rviz에서 제대로 조립되어보이도록 하는 노드
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen', 
        # ParameterValue를 사용하여 Command 결과를 문자열 타입의 'robot_description'파라미터로 전달
        parameters=[{
        'robot_description': ParameterValue(robot_description_content, value_type=str),
        # 실제 로봇 하드웨어나 시뮬레이션 시간 동기화 여부
        # False: 실제 로봇 하드웨어 구동 시, 시스템 시간 사용 시(기본값)
        # True: Gazebo, Webot, Isaac Sim등 가상 시뮬레이터의 시간(/clock토픽)에 동기화 할 때 사용
        'use_sim_time': False
    }]
    )

    # 5. joint_state_publisher_gui 노드 설정
    # 슬라이드 바 형태의 GUI 창을 띄워 로봇의 각 관절(joint)을 수동으로 움직일 수 있도록
    # 이 노드에서 나가는 관절의 값을 robot_state_publisher가 받아서 로봇의 움직임을 계산
    jsp_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 6. Rviz2 설정 및 실행 노드
    # 미리 저장된 설정 파일(.rviz)의 경로를 찾음
    rviz_config_path = os.path.join(get_package_share_directory('my_robot_description'),
    'rviz',
    'default.rviz')

    rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_path], 
)

    return LaunchDescription([
        rsp_node,
        jsp_node,
        rviz_node,
    ])
