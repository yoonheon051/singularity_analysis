#!/usr/bin/env bash
# singularity_analysis 설치·빌드 (처음 한 번, 또는 ROS 버전을 바꾼 뒤 다시)
#
#   mkdir -p ~/colcon_ws/src && cd ~/colcon_ws/src
#   git clone https://github.com/yoonheon051/singularity_analysis.git
#   ./singularity_analysis/setup.sh
#   source ~/colcon_ws/env.sh && ros2 launch singularity_analysis analysis.launch.py
#
# 하는 일
#   1. 설치된 ROS2 찾기 (이미 source 했으면 그 버전, 아니면 /opt/ros 의 jazzy → humble)
#   2. ROS 의존성 설치 (apt + rosdep) — sudo 비밀번호를 물을 수 있음
#   3. Robotics Toolbox 등 pip 패키지를 워크스페이스 가상환경(<ws>/.venv)에 설치
#      Ubuntu 24.04 는 시스템 Python 에 pip 설치를 막으므로 가상환경을 쓴다.
#      --system-site-packages 라서 apt 로 깐 ROS·NumPy 등도 그대로 보인다.
#   4. 빌드 (다른 ROS 버전으로 빌드된 결과가 남아 있으면 지우고 다시)
#   5. <ws>/env.sh 생성: ROS + 워크스페이스 + 가상환경 패키지를 한 번에 켜는 파일
#
# 환경변수
#   WS=<워크스페이스>  저장소를 <워크스페이스>/src 밖에 두었을 때 지정
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -z "${WS:-}" ]]; then
  if [[ "$(basename "$(dirname "$REPO_DIR")")" != "src" ]]; then
    echo "이 저장소는 <워크스페이스>/src 안에 있어야 합니다 (예: ~/colcon_ws/src/singularity_analysis)." >&2
    echo "다른 곳에 두었다면 워크스페이스를 지정하세요: WS=~/colcon_ws $0" >&2
    exit 1
  fi
  WS="$(dirname "$(dirname "$REPO_DIR")")"
fi
WS="$(cd "$WS" && pwd)"
VENV="$WS/.venv"
SUDO=""
if [[ $EUID -ne 0 ]]; then SUDO="sudo"; fi

step() { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }

# ---------------------------------------------------------------- 1. ROS2
step "1/5 ROS2 찾기"
if [[ -z "${ROS_DISTRO:-}" ]]; then
  for d in jazzy humble; do
    if [[ -f "/opt/ros/$d/setup.bash" ]]; then ROS_DISTRO="$d"; break; fi
  done
fi
if [[ -z "${ROS_DISTRO:-}" || ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS2 를 찾지 못했습니다. Ubuntu 24.04 는 Jazzy, 22.04 는 Humble 을 먼저 설치하세요." >&2
  echo "  https://docs.ros.org/en/jazzy/Installation.html" >&2
  exit 1
fi
set +u   # ROS 의 setup 스크립트는 정의되지 않은 변수를 참조함
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
echo "ROS2 ${ROS_DISTRO} / 워크스페이스 ${WS}"

# ---------------------------------------------------------------- 2. ROS 의존성
step "2/5 ROS 의존성 설치 (apt + rosdep, sudo 비밀번호를 물을 수 있음)"
tools=()
command -v rosdep >/dev/null 2>&1 || tools+=(python3-rosdep)
command -v colcon >/dev/null 2>&1 || tools+=(python3-colcon-common-extensions)
$SUDO apt-get update
if ((${#tools[@]})); then $SUDO apt-get install -y "${tools[@]}"; fi
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then $SUDO rosdep init; fi
rosdep update --rosdistro "$ROS_DISTRO"
rosdep install -i -y --from-paths "$REPO_DIR" --rosdistro "$ROS_DISTRO"

# ---------------------------------------------------------------- 3. pip 패키지 (가상환경)
step "3/5 Robotics Toolbox 설치 (가상환경 ${VENV})"
if [[ ! -x "$VENV/bin/python" ]]; then
  if ! python3 -m venv --system-site-packages "$VENV" 2>/dev/null; then
    rm -rf "$VENV"
    $SUDO apt-get install -y python3-venv     # Ubuntu 는 venv 가 별도 패키지
    python3 -m venv --system-site-packages "$VENV"
  fi
fi
touch "$VENV/COLCON_IGNORE"   # colcon 이 가상환경 안을 패키지로 뒤지지 않게
"$VENV/bin/python" -m pip install -q -r "$REPO_DIR/requirements.txt"

# ---------------------------------------------------------------- 4. 빌드
step "4/5 빌드"
cd "$WS"
if [[ -f install/setup.bash ]] && ! grep -q "/opt/ros/${ROS_DISTRO}" install/setup.bash; then
  echo "다른 ROS 버전으로 빌드된 결과가 있어 ${WS}/build, install, log 를 지우고 다시 빌드합니다."
  rm -rf build install log
fi
colcon build --packages-up-to singularity_analysis

# ---------------------------------------------------------------- 5. env.sh
step "5/5 env.sh 만들기"
SITE="$("$VENV/bin/python" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
cat > "$WS/env.sh" <<EOF
# singularity_analysis 실행 환경 (setup.sh 가 만든 파일). 새 터미널마다: source ${WS}/env.sh
# ROS2 ${ROS_DISTRO} + 이 워크스페이스 + 가상환경의 pip 패키지(Robotics Toolbox 등)
source "${WS}/install/setup.bash"
# ros2 run/launch 로 띄운 노드는 시스템 python3 로 실행되므로, 가상환경 패키지 경로를 PYTHONPATH 에 직접 넣는다
case ":\${PYTHONPATH:-}:" in
  *":${SITE}:"*) ;;
  *) export PYTHONPATH="${SITE}\${PYTHONPATH:+:\$PYTHONPATH}" ;;
esac
EOF

# 시스템 python3 로 Toolbox 를 불러올 수 있는지 (= ros2 run 으로 띄운 노드와 같은 조건) 확인
(
  set +u
  # shellcheck source=/dev/null
  source "$WS/env.sh"
  python3 -c 'import roboticstoolbox, singularity_analysis.singularity'
) || { echo "설치는 끝났지만 Robotics Toolbox 를 불러오지 못했습니다. 위의 오류를 확인하세요." >&2; exit 1; }

step "완료"
echo "새 터미널마다:  source ${WS}/env.sh"
echo "시뮬레이션:     ros2 launch singularity_analysis analysis.launch.py"
echo "실제 로봇:      ros2 launch singularity_analysis analysis.launch.py mode:=real"
echo "대시보드:       http://localhost:8080"
