"""
시뮬레이션 + 특이점 분석을 한 번에 실행

    ros2 launch singularity_analysis analysis.launch.py
    ros2 launch singularity_analysis analysis.launch.py model:=m0609            # 그리퍼 없는 모델
    ros2 launch singularity_analysis analysis.launch.py gui:=false              # 관절 슬라이더 창 없이 (실제 로봇 /joint_states 사용)
    ros2 launch singularity_analysis analysis.launch.py results_dir:=~/my_runs  # 결과 저장 폴더

실행되는 노드
- robot_state_publisher : my_robot_description 의 URDF(xacro) → /robot_description, /tf
- joint_state_publisher_gui : 관절 슬라이더 → /joint_states        (gui:=true 일 때)
- rviz2 : 로봇 모델 + 가동성 타원체(/manipulability_ellipsoid)      (rviz:=true 일 때)
- analyzer : 특이점 분석 노드 (CSV 기록, 지표 토픽, 타원체 마커)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('my_robot_description')
    analysis_share = get_package_share_directory('singularity_analysis')

    model = LaunchConfiguration('model')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    results_dir = LaunchConfiguration('results_dir')

    # my_robot_description/urdf/<model>.urdf.xacro 를 xacro 로 처리한 결과가 robot_description
    xacro_file = PathJoinSubstitution([description_share, 'urdf', [model, '.urdf.xacro']])
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='m0609_rg2',
                              description='로봇 모델: m0609_rg2 (RG2 그리퍼 포함) 또는 m0609'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='관절 슬라이더 창(joint_state_publisher_gui) 실행 여부'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz 실행 여부'),
        DeclareLaunchArgument('results_dir', default_value=os.path.expanduser('~/singularity_results'),
                              description='분석 CSV 저장 폴더'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(gui),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(analysis_share, 'rviz', 'analysis.rviz')],
            condition=IfCondition(rviz),
        ),
        Node(
            package='singularity_analysis',
            executable='analyzer',
            name='m0609_singularity_monitor',
            output='screen',
            parameters=[{'results_dir': results_dir}],
        ),
    ])
