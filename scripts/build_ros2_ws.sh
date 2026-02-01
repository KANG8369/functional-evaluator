#!/usr/bin/env bash
set -e
# set -u  # /opt/ros/.../setup.bash에서 unbound variable로 터질 수 있어 비추천

WS_ROOT="${1:-}"
ROS_DISTRO="${2:-${ROS_DISTRO:-humble}}"

if [[ -z "$WS_ROOT" ]]; then
  echo "[build_ros2_ws.sh] ERROR: ws_root path is required."
  exit 2
fi

if [[ ! -d "$WS_ROOT/src" ]]; then
  echo "[build_ros2_ws.sh] ERROR: '$WS_ROOT/src' not found (invalid workspace)."
  exit 2
fi

ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
  echo "[build_ros2_ws.sh] ERROR: ROS setup not found: $ROS_SETUP"
  exit 2
fi

LOG_DIR="${WS_ROOT}/build_logs"
mkdir -p "$LOG_DIR"
LOG_FILE="${LOG_DIR}/build.txt"

# 이전 빌드 잔재 제거
rm -rf "${WS_ROOT}/build" "${WS_ROOT}/log" "${WS_ROOT}/install"

echo "[build_ros2_ws.sh] Building workspace: $WS_ROOT" | tee "$LOG_FILE"
echo "[build_ros2_ws.sh] ROS_DISTRO: $ROS_DISTRO" | tee -a "$LOG_FILE"
echo "[build_ros2_ws.sh] Log: $LOG_FILE" | tee -a "$LOG_FILE"
echo "------------------------------------------------------------" | tee -a "$LOG_FILE"

# ✅ (중요) ROS를 source 하기 전에 prefix 변수 정리
#   - 이전 빌드 ws install 경로가 남아 경고가 뜨는걸 방지
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH

# ROS 환경 로드
# shellcheck disable=SC1090
source "$ROS_SETUP"

cd "$WS_ROOT"

# ✅ pip로 꼬인 /usr/local/bin/colcon 대신 apt 경로 colcon 고정
COLCON_BIN="/usr/bin/colcon"
if [[ ! -x "$COLCON_BIN" ]]; then
  echo "[build_ros2_ws.sh] ERROR: $COLCON_BIN not found or not executable." | tee -a "$LOG_FILE"
  echo "  Try: sudo apt update && sudo apt install -y python3-colcon-common-extensions" | tee -a "$LOG_FILE"
  exit 2
fi

set +e

# ✅ 로그 파일에는 전체 저장
# ✅ 콘솔에는 불필요 warning 일부 필터링해서 출력
"$COLCON_BIN" build --merge-install 2>&1 \
  | tee -a "$LOG_FILE" \
  | grep -vE '^\[[0-9]+\.[0-9]+s\] WARNING:colcon\.' \
  | grep -vE '^/home/.*PkgResourcesDeprecationWarning' \
  | grep -vE '^  warnings\.warn\('

BUILD_RC=${PIPESTATUS[0]}
set -e

echo "------------------------------------------------------------" | tee -a "$LOG_FILE"
if [[ $BUILD_RC -eq 0 ]]; then
  echo "[build_ros2_ws.sh] ✅ BUILD SUCCESS" | tee -a "$LOG_FILE"
  exit 0
else
  echo "[build_ros2_ws.sh] ❌ BUILD FAILED (exit=$BUILD_RC)" | tee -a "$LOG_FILE"
  exit $BUILD_RC
fi
