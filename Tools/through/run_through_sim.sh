#!/usr/bin/env bash
# Start the THROUGH simulation stack (THROUGH_MODE_SPEC.md 13.4):
#   PX4 SITL (x500_through in the through_ring world), MicroXRCEAgent, ros_gz_bridge,
#   through_vision (or fake_vision_target) and ring_judge.py.
#
# Usage: run_through_sim.sh [--headless] [--fake-vision] [--no-vision] [--no-judge]
#                           [--no-window] [--log-dir DIR]
#
# Ctrl-C stops everything. Each process logs to $LOG_DIR/<name>.log.
# PX4 runs as a daemon (px4 -d); use build/px4_sitl_default/bin/px4-<cmd> for its shell,
# e.g. px4-listener through_status. QGC connects automatically on UDP 14550.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
PX4_DIR="$ROOT_DIR"
ROS_WS="$ROOT_DIR/ros2_ws"
BUILD_DIR="$PX4_DIR/build/px4_sitl_default"

HEADLESS=0
VISION=real
JUDGE=1
SHOW_WINDOW=true
LOG_DIR="$ROOT_DIR/logs/through_sim_$(date +%Y%m%d_%H%M%S)"

while [[ $# -gt 0 ]]; do
	case "$1" in
	--headless) HEADLESS=1 ;;
	--fake-vision) VISION=fake ;;
	--no-vision) VISION=none ;;
	--no-judge) JUDGE=0 ;;
	--no-window) SHOW_WINDOW=false ;;
	--log-dir) LOG_DIR="$2"; shift ;;
	-h|--help) sed -n '2,13p' "$0"; exit 0 ;;
	*) echo "unknown option: $1" >&2; exit 1 ;;
	esac
	shift
done

mkdir -p "$LOG_DIR"
PIDS=()

cleanup() {
	echo "stopping..."
	for pid in "${PIDS[@]}"; do
		kill -INT -- "-$pid" 2>/dev/null || true
	done
	sleep 2
	for pid in "${PIDS[@]}"; do
		kill -KILL -- "-$pid" 2>/dev/null || true
	done
	# the gz server is started by PX4 (px4-rc.gzsim) in its own process group
	if [[ -n "${GZ_PID:-}" ]]; then
		kill "$GZ_PID" 2>/dev/null || true
	fi
	echo "logs: $LOG_DIR"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# start <name> <command...>: run in its own process group, log to $LOG_DIR/<name>.log
start() {
	local name="$1"; shift
	setsid "$@" > "$LOG_DIR/$name.log" 2>&1 < /dev/null &
	PIDS+=($!)
	echo "started $name (pid $!)"
}

wait_for_log() {
	local file="$1" pattern="$2" timeout="$3"
	for ((i = 0; i < timeout; i++)); do
		grep -q "$pattern" "$file" 2>/dev/null && return 0
		sleep 1
	done
	echo "timeout waiting for '$pattern' in $file" >&2
	return 1
}

# a leftover stack (PX4 / gz / agent bound to UDP 8888) makes the new one fail in confusing ways
LEFTOVER="$(pgrep -a -x px4; pgrep -af 'gz sim .*-s' ; pgrep -a -x MicroXRCEAgent)" || true
if [[ -n "$LEFTOVER" ]]; then
	echo "already running, stop these first:" >&2
	echo "$LEFTOVER" >&2
	trap - EXIT
	exit 1
fi

if [[ ! -x "$BUILD_DIR/bin/px4" ]]; then
	echo "PX4 not built: run 'make px4_sitl_default' in $PX4_DIR" >&2
	exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
set -u

# 1. PX4 SITL (starts gz sim through px4-rc.gzsim)
PX4_ENV=(PX4_SYS_AUTOSTART=4090 PX4_SIM_MODEL=gz_x500_through PX4_GZ_WORLD=through_ring)
[[ $HEADLESS -eq 1 ]] && PX4_ENV+=(HEADLESS=1)
start px4 env -C "$BUILD_DIR" "${PX4_ENV[@]}" ./bin/px4 -d
wait_for_log "$LOG_DIR/px4.log" "Startup script returned" 120
GZ_PID="$(pgrep -f 'gz sim .*through_ring.sdf' | head -1 || true)"

# 2. uXRCE-DDS agent
start agent MicroXRCEAgent udp4 -p 8888

# 3. gz -> ROS bridge (clock, camera image and info)
start bridge ros2 run ros_gz_bridge parameter_bridge --ros-args \
	-p config_file:="$SCRIPT_DIR/config/through_bridge.yaml"

# 4. vision
# PYTHONNOUSERSITE: ignore pip packages in ~/.local. A NumPy 2.x / opencv-python there breaks
# the ROS Humble cv_bridge (built against NumPy 1.x: "_ARRAY_API not found", segfault).
case "$VISION" in
real)
	start vision env PYTHONNOUSERSITE=1 ros2 run through_vision through_vision --ros-args \
		-p use_sim_time:=true -p show_window:="$SHOW_WINDOW"
	;;
fake)
	start vision ros2 run through_vision fake_vision_target --ros-args -p use_sim_time:=true
	;;
none) ;;
esac

# 5. ring judge (true poses from gz transport)
if [[ $JUDGE -eq 1 ]]; then
	start judge python3 "$SCRIPT_DIR/ring_judge.py" --csv-dir "$LOG_DIR"
fi

echo "running, Ctrl-C to stop (logs: $LOG_DIR)"
wait "${PIDS[0]}"
