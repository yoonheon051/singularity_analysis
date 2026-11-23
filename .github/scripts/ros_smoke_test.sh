#!/usr/bin/env bash
# CI: setup.sh 로 만든 워크스페이스에서 launch 를 실제로 띄워 확인 (화면 없이 되는 mode:=real 사용)
#   - 시스템 python3 로 Robotics Toolbox 를 불러올 수 있는지 (env.sh 의 PYTHONPATH)
#   - 분석 노드가 /dsr01/joint_states 를 받아 /singularity/status 를 내는지 (손목 특이점 자세 → "wrist")
#   - CSV 가 생기는지, 대시보드 웹 서버(8080)와 rosbridge(9090)가 뜨는지
#
#   .github/scripts/ros_smoke_test.sh <워크스페이스>
set -eo pipefail
WS="$1"
RESULTS=/tmp/sa_results
LOG=/tmp/sa_launch.log

# shellcheck source=/dev/null
source "$WS/env.sh"
set -u

echo "== 1. 시스템 python3 로 Toolbox 불러오기"
python3 -c "import roboticstoolbox as rtb, numpy; print('roboticstoolbox', rtb.__version__, '/ numpy', numpy.__version__)"

echo "== 2. launch (mode:=real: 분석 노드 + rosbridge + 대시보드)"
ros2 launch singularity_analysis analysis.launch.py mode:=real results_dir:="$RESULTS" status_rate_hz:=0.0 >"$LOG" 2>&1 &
LAUNCH=$!
PUB=""
cleanup() {
  [[ -n "$PUB" ]] && kill "$PUB" 2>/dev/null || true
  kill -INT "$LAUNCH" 2>/dev/null || true
  sleep 2
  echo "---- launch 로그 (끝부분) ----"
  tail -n 40 "$LOG" || true
}
trap cleanup EXIT

# 손목 특이점 근처 자세(q5 = 0.05 rad)를 두산 드라이버 토픽 이름으로 계속 발행
ros2 topic pub -r 10 /dsr01/joint_states sensor_msgs/msg/JointState \
  "{name: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], position: [0.0, -0.3, 1.2, 0.0, 0.05, 0.0]}" >/dev/null &
PUB=$!

echo "== 3. /singularity/status 한 개 받기 (최대 90초)"
python3 - <<'PY'
import json, time
import rclpy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('smoke_test')
got = []
node.create_subscription(String, '/singularity/status', lambda m: got.append(m.data), 10)
deadline = time.time() + 90
while not got and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.5)
assert got, '/singularity/status 를 90초 안에 받지 못함'
s = json.loads(got[0])
print({k: s[k] for k in ('type', 'level', 'risk', 'dominant', 'joint_risk')})
assert s['type'] == 'wrist' and s['level'] == 'danger', s['type']
assert s['joint_risk'][3:] == [1.0, 1.0, 1.0], s['joint_risk']
assert len(s['geometry']['joints']) == 6
node.destroy_node()
rclpy.shutdown()
PY

echo "== 4. CSV, 대시보드(8080), rosbridge(9090)"
ls "$RESULTS"/analysis_*.csv
python3 - <<'PY'
import socket, time, urllib.request
deadline = time.time() + 30
while True:
    try:
        html = urllib.request.urlopen('http://127.0.0.1:8080/', timeout=3).read().decode('utf-8')
        socket.create_connection(('127.0.0.1', 9090), timeout=3).close()
        break
    except OSError:
        if time.time() > deadline:
            raise
        time.sleep(1)
assert '/singularity/status' in html, 'dashboard page content'
print('dashboard 8080 OK, rosbridge 9090 OK')
PY

echo "== 모두 통과"
