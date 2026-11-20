# 🐊 my_robot_description

이 패키지는 **두산로보틱스 M0609** 로봇 팔과 **OnRobot RG2** 그리퍼를 통합하여 ROS 2 Humble 환경에서 시뮬레이션하기 위한 패키지입니다.

## 요구사항
ubuntu 22.04

ROS2 Humble

## 패키지 구성 요소 
**`urdf/`**: 로봇과 그리퍼의 결합 구조를 정의하는 Xacro 파일들이 포함되어 있습니다. 

**`meshes/`**: RViz 시각화를 위한 .stl 모델 파일들입니다. M0609와 RG2에 필수적인 파일만 남겨 최적화되었습니다.

**`launch/`**: `rviz_sim.launch.py`를 통해 로봇 상태 게시와 시각화를 한 번에 실행합니다. 

**`rviz/`**: `base_link` 고정 및 TF/RobotModel 자동 활성화 설정이 담긴 `default.rviz` 파일이 포함되어 있습니다. 

## 빌드 및 설치 방법 
```
# 패키지 리스트 업데이트
sudo apt update

# 개별 패키지 설치 (Humble 기준)
# 로봇 상태(TF) 발행 및 GUI 조인트 제어 도구
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui

# 시각화 도구 RViz2
sudo apt install ros-humble-rviz2

# Gazebo 시뮬레이션 연동 패키지
sudo apt install ros-humble-gazebo-ros-pkgs

# (선택 사항) 로봇 기구학 계산에 필요한 xacro 및 가제보 제어기
sudo apt install ros-humble-xacro
sudo apt install ros-humble-gazebo-ros2-control

# 워크스페이스 이동
# 이 패키지는 singularity_analysis 저장소 안에 들어 있습니다 (예전 별도 저장소 my_robot_description 에서 옮겨 옴)
cd ~/ros2_ws/src
git clone https://github.com/yoonheon051/singularity_analysis.git
```
```
# 의존성 설치 및 빌드
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build --packages-select my_robot_description
source install/setup.bash

# 실행 (모델만 보기)
ros2 launch my_robot_description rviz_sim.launch.py
```

특이점 분석까지 함께 실행하려면 저장소 최상위 README의 `ros2 launch singularity_analysis analysis.launch.py`를 보세요.
