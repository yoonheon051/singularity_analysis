# 🐊 singularity_analysis
동아대학교 기계공학과 2026-1학기 동역학1분반 취업공결 인정 과제

이 프로젝트는 m0609의 각 관절 각도를 Jacobian 행렬로 계산하여, Singularity를 분석합니다.

2026.04.24 현재, m0609 6축 로봇팔만 지원하며, m0609의 시뮬레이션(Rviz2)은 https://github.com/yoonheon051/my_robot_description 여기서 확인할 수 있습니다.

## 개발환경
OS: UBUNTU 22.04.05 LTS

Framework: ROS2 Humble

Language: Python 3.10.12

Key Library: NumPy 1.24.4, Roboticstoolbox-python, PyKDL

## 파일 구성
```
singularity_analysis
├── .gitignore              # git 제외 목록
├── README.md               # 프로젝트 메뉴얼
├── requirements.txt        # 의존성 라이브러리 목록
├── scripts/                # 실행할 메인 스크립트
│   ├── calculate_jacobian.py
│   └── m0609_ros_analyzer.py 
└── results/                # 분석 결과 이미지나 로그 저장
```

## 프로젝트 의도

### 1. 실시간 Jacobian 데이터 연산 및 획득
 - Kinematic Tree 구축: 로봇의 URDF 파일로부터 KDL(Kinematics and Dynamics Library) 트리를 생성하여 수학적 연산 기반 마련.
 - 데이터 획득 기법:
    - Roboticstoolbox(rtb): get_jacobian_matrix()를 호출하여 현재 포즈에 대한 $6 \times 6$ Geometric Jacobian 획득.
    - Numerical Analysis: 실시간 관절 각도($q$)를 입력으로 하여 Jacobian 행렬($J(q)$)을 즉시 산출.

### 2. 가동성 타원체(Manipulability Ellipsoid)를 통한 특이점(Singularity) 가시화
로봇 말단(End-effector)의 운동 성능을 기하학적으로 해석합니다.
- 특이점 판별: 
    - Normal State: 타원체가 구(Sphere)에 가까운 형태를 유지하며 모든 방향으로 균등한 이동성 확보.
    - Near Singularity: 타원체가 특정 축으로 편평(Flattened)해지며, 해당 방향으로의 속도 생성이 불가능한 물리적 한계점 분석.

### 3. 매칭 테이블
```
--------------------------------------------------------------------------------
항목            | 실제 구현 파일           | 구현 방식
--------------------------------------------------------------------------------
Kinematic Tree | m0609_ros_analyzer.py | rtb.models.URDF를 통한 모델 로드
--------------------------------------------------------------------------------
실시간 데이터 획득 | m0609_ros_analyzer.py | /joint_states 토픽 구독 및 실시간 연산
--------------------------------------------------------------------------------
Jacobian 산출   | calculate_jacobian.py | jacob0(q) 함수로 $6 \times 6$ 행렬 생성
--------------------------------------------------------------------------------
특이점 분석 지표  |          공통          | manipulability(), np.linalg.cond() 활용
--------------------------------------------------------------------------------
결과 저장        | m0609_ros_analyzer.py | results/analysis_log.csv 파일로 기록
--------------------------------------------------------------------------------
```

## 실행순서
1. 로봇 환경 실행
my_robot_description 패키지 또는 /joint_states 토픽을 발행하는 로봇 시뮬레이션/실제 로봇 노드를 실행합니다.

2. 분석 스크립트 실행
새 터미널에서 아래 명령어를 통해 실시간 특이점 모니터링을 시작합니다.

```bash
# new terminal
cd ~/singularity_analysis
python3 scripts/m0609_ros_analyzer.py
```
