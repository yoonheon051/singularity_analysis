# 사용법 상세

[← README](../README.md)

launch 옵션, 분석 노드가 내는 토픽·로그·CSV, 개별 스크립트 사용법입니다. 설치는 README 의 [빠른 시작](../README.md#빠른-시작)을 보세요.

- [launch 옵션](#launch-옵션)
- [분석 노드 (analyzer)](#분석-노드-analyzer)
- [CSV 결과](#csv-결과)
- [개별 스크립트](#개별-스크립트)
- [대시보드만 따로 띄우기](#대시보드만-따로-띄우기)

## launch 옵션

```bash
ros2 launch singularity_analysis analysis.launch.py [옵션:=값 ...]
```

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `mode:=` | `sim` | `real` 이면 실제 로봇 (README [실제 로봇에서 실행](../README.md#실제-로봇에서-실행)) |
| `model:=` | `m0609_rg2` | 로봇 모델. `m0609`(그리퍼 없음) 또는 `m0609_rg2`(RG2 그리퍼 포함) |
| `gui:=` | `true` | 관절 슬라이더 창 (`mode:=real` 이면 항상 끔) |
| `rviz:=` | sim: `true`, real: `false` | RViz (로봇 모델 + 가동성 타원체) |
| `joint_states_topic:=` | sim: `/joint_states`, real: `/dsr01/joint_states` | 관절 상태 토픽 |
| `results_dir:=` | `~/singularity_results` | CSV 저장 폴더 (환경변수 `SINGULARITY_RESULTS_DIR` 로도 지정) |
| `dashboard:=` | `true` | rosbridge + 대시보드 웹 서버 실행 |
| `dashboard_port:=` / `rosbridge_port:=` | `8080` / `9090` | 웹 페이지 / WebSocket 포트 |
| `status_rate_hz:=` | `15.0` | 대시보드 갱신 빈도 (0 이하면 관절 메시지마다) |

- 그리퍼 모델이어도 분석은 팔 6축(base_link → link_6)만 대상으로 하고, 관절값은 이름(`joint_1`~`joint_6`)으로 찾습니다. 순서가 섞여 있거나 그리퍼 관절이 같이 와도 됩니다.
- `mode:=real` 에서는 관절 슬라이더와 이 패키지의 robot_state_publisher·RViz 를 띄우지 않고(드라이버가 맡음), 분석 모델은 `my_robot_description/urdf/<model>.urdf.xacro` 에서 직접 읽습니다.
- 로봇 모델만 보려면 `ros2 launch my_robot_description rviz_sim.launch.py`.

## 분석 노드 (analyzer)

`m0609_ros_analyzer.py` 가 `/robot_description`(또는 `urdf_file`)으로 모델을 만들고, `/joint_states` 를 받을 때마다 Jacobian·지표·특이점 종류를 계산합니다. 노드 내부 흐름은 [개발자 문서](development.md#분석-흐름)에 있습니다.

### 터미널 로그
- 지표는 1초에 한 번 출력 (`/joint_states` 가 수십 Hz 라 매번 찍으면 넘침)
- 특이점 근처에 **들어가거나 나오는 순간** 경고: `특이점 근처 진입: elbow (지표: wrist=0.841, elbow=0.026, shoulder=0.500)`

### 발행 토픽

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/singularity/manipulability` | `std_msgs/Float64` | Manipulability $w$ |
| `/singularity/condition_number` | `std_msgs/Float64` | 조건수 (완전 특이면 inf) |
| `/singularity/sigma_min` | `std_msgs/Float64` | 최소 특이값 |
| `/singularity/type` | `std_msgs/String` | `normal` / `wrist` / `elbow` / `shoulder` / 겹치면 `wrist+elbow` |
| `/singularity/status` | `std_msgs/String` | 대시보드용 종합 상태 JSON ([형식](development.md#singularitystatus-형식)) |
| `/manipulability_ellipsoid` | `visualization_msgs/MarkerArray` | RViz 가동성 타원체 |

```bash
ros2 run rqt_plot rqt_plot /singularity/manipulability/data /singularity/sigma_min/data   # 실시간 그래프
ros2 topic echo /singularity/type
```

**RViz 타원체**: sim 모드의 RViz 에는 이미 들어 있습니다. 다른 RViz(예: 두산 드라이버)에서는 **Add → By topic → `/manipulability_ellipsoid` → MarkerArray**. 손목 중심에 두 개가 보입니다.
- `arm` (팔 1~3축 선속도): 팔꿈치·어깨 특이점 근처에서 납작해지고 빨간색
- `wrist` (손목 4~6축 각속도): 손목 특이점 근처에서 납작해지고 빨간색
- 크기는 반지름 × 0.3 m (`ELLIPSOID_SCALE`). 왜 둘로 나누는지는 [이론](theory.md#왜-타원체를-둘로-나누어-그리나)

### 노드 파라미터 (launch 가 넣어 주므로 직접 쓸 일은 드묾)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `joint_states_topic` | `/joint_states` | 관절 상태 토픽 |
| `robot_description_topic` | `/robot_description` | URDF 를 받을 토픽 |
| `urdf_file` | (빈 값) | 주면 토픽을 기다리지 않고 이 URDF/xacro 파일로 모델을 만듦 |
| `status_rate_hz` | `15.0` | `/singularity/status` 발행 빈도 (0 이하면 매 메시지) |
| `warn_factor` | `3.0` | 근접도 0% 가 되는 지점 = 임계값 × 이 값 |
| `results_dir` | `~/singularity_results` | CSV 저장 폴더 |

## CSV 결과

실행할 때마다 `~/singularity_results/analysis_날짜_시각.csv` 를 새로 만듭니다(노드 시작 시 터미널에 경로가 찍힘). 파일은 열어 둔 채로 쓰고 1초마다 디스크에 내보내며, 노드를 끌 때 닫습니다.

열: 시각, q1~q6, Manipulability, 조건수, 특이점 종류, 최소 특이값, 손목·팔꿈치·어깨 지표, 6×6 Jacobian 36개

```
timestamp | q1  | ... | q6  | manipulability | condition_number | singularity_type | sigma_min | wrist_idx | elbow_idx | shoulder_idx | ...
12:34:56  | 0.0 | ... | 0.0 |     0.0198     |       20.1       |      normal      |   0.12    |   0.841   |   0.595   |    0.214     | ...
12:34:57  | 0.0 | ... | 0.0 |     0.0012     |      270.6       |      wrist       |   0.004   |   0.050   |   0.595   |    0.214     | ...
```

## 개별 스크립트

모두 ROS 없이 URDF 만으로 동작합니다. URDF 를 주지 않으면 `my_robot_description` 의 `m0609.urdf.xacro` 를 쓰고, 결과 그림은 `~/singularity_results/` 에 저장합니다.
빌드하지 않고 `python3 singularity_analysis/singularity_analysis/<파일>.py` 로 바로 실행해도 됩니다.

### visualize_results — CSV 그래프
```bash
ros2 run singularity_analysis visualize_results                 # 가장 최근 CSV
ros2 run singularity_analysis visualize_results <CSV 경로>       # 다른 실행
```
Manipulability(경고선 0.01), 조건수(로그 스케일, inf 는 빨간 X), 손목·팔꿈치·어깨 지표와 임계값을 그리고, 배경을 특이점 종류별로 색칠합니다(손목 주황, 팔꿈치 보라, 어깨 청록). 화면에 띄우고 CSV 옆에 `..._plot.png` 로도 저장합니다. 예시는 [이론 문서](theory.md#특이점-종류-분류)의 그래프.

### calculate_jacobian — 한 자세 분석
```bash
ros2 run singularity_analysis calculate_jacobian                          # 기본: 손목 특이점 근처 (0 0 1.5708 0 0.01 0)
ros2 run singularity_analysis calculate_jacobian --q 0 -0.3 1.2 0 1.0 0   # 다른 자세 (rad)
ros2 run singularity_analysis calculate_jacobian 다른로봇.urdf
```
```
Joint angles (rad): [0.0, 0.0, 1.571, 0.0, 0.01, 0.0]
Determinant: -0.000557
Manipulability Index: 0.000557
Condition Number: 797.47
Singularity type: wrist
Hardest linear direction (base frame): [...]
```
특이점이 아닌 자세(`--q 0 -0.3 1.2 0 1.0 0`)는 Manipulability 0.0198, 조건수 20.1 정도로, 손목 특이점 근처보다 조건수가 약 40배 작습니다.

### plot_ellipsoid — 자세별 가동성 타원체
```bash
ros2 run singularity_analysis plot_ellipsoid                                         # 정상·손목·팔꿈치·어깨 4자세
ros2 run singularity_analysis plot_ellipsoid --q 0 -0.3 1.2 0 1 0 --q 0 0 0 0 0 0   # 원하는 자세 여러 개
```
윗줄 ARM, 아랫줄 WRIST 타원체, 빨간 화살표는 가장 움직이기 어려운 방향. 결과 `manipulability_ellipsoids.png`.

### velocity_demo — 속도 폭주 vs DLS
```bash
ros2 run singularity_analysis velocity_demo
ros2 run singularity_analysis velocity_demo --eps 0.1 --lam 0.15   # 감쇠를 키워 정확도와의 균형 비교
```
방법별 최대 관절 속도·위치 오차·자세 오차 표를 출력하고 `velocity_dls_demo.png` 를 저장합니다. 결과 해석은 [이론 문서](theory.md#속도-명령-폭주와-감쇠-최소제곱-dls).

## 대시보드만 따로 띄우기

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml   # WebSocket 9090
ros2 run singularity_analysis dashboard --port 8080
```
rosbridge 주소가 다르면 `http://PC:8080/?ws=ws://192.168.0.10:9090`. 같은 네트워크의 태블릿·폰에서는 `http://<PC IP>:8080`.
