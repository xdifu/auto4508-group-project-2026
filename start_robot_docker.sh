#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./start_robot_docker.sh [options] [-- extra args...]

Modes:
  --mode shell    Start an interactive shell in the robot container (default)
  --mode prep     Run ./setup_env.sh inside the container
  --mode build    Build the ROS workspace with colcon
  --mode real     Launch the full Part 2 real stack
  --mode vision   Launch the standalone vision stack

Options:
  --image <name>          Docker image to use
  --name <container>      Container name override
  --team <id>             Team number for run_id/artifacts naming (default: 17)
  --run-id <value>        Explicit run identifier
  --rviz                  Enable X11 forwarding and pass rviz:=true to real mode
  --record                Pass record:=true to real mode
  --robot-port <path>     Serial port for Pioneer base (default: /dev/ttyUSB0)
  --gps-port <path>       Serial port for GPS (default: /dev/ttyACM0)
  --gps-baud <baud>       GPS baud rate (default: 9600)
  --artifacts-root <dir>  Host-side artifacts root override
  --workspace <dir>       Host repo root override
  -h, --help              Show this help

Examples:
  ./start_robot_docker.sh
  ./start_robot_docker.sh --mode prep
  ./start_robot_docker.sh --mode build
  ./start_robot_docker.sh --mode real --record
  ./start_robot_docker.sh --mode real --image mobrob-ros2-ros2:latest
  ./start_robot_docker.sh --mode vision -- --ros-args -p launch_camera_driver:=false
EOF
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Required command not found: $1" >&2
    exit 1
  fi
}

shell_join() {
  local pieces=()
  local value
  for value in "$@"; do
    pieces+=("$(printf '%q' "$value")")
  done
  printf '%s' "${pieces[*]}"
}

detect_image() {
  local candidates=(
    "mobrob-ros2-ros2:latest"
    "docker-t5_paneer:latest"
  )
  local candidate
  for candidate in "${candidates[@]}"; do
    if docker image inspect "$candidate" >/dev/null 2>&1; then
      echo "$candidate"
      return 0
    fi
  done
  return 1
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null || printf '%s\n' "$SCRIPT_DIR")"

MODE="shell"
TEAM_ID="17"
IMAGE_NAME="${AUTO4508_DOCKER_IMAGE:-}"
CONTAINER_NAME=""
RUN_ID=""
ENABLE_RVIZ="false"
ENABLE_RECORD="false"
ROBOT_PORT="/dev/ttyUSB0"
GPS_PORT="/dev/ttyACM0"
GPS_BAUD="9600"
HOST_ARTIFACTS_ROOT=""
WORKSPACE_HOST="$REPO_ROOT"
CONTAINER_ARTIFACTS_ROOT=""
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      MODE="$2"
      shift 2
      ;;
    --image)
      IMAGE_NAME="$2"
      shift 2
      ;;
    --name)
      CONTAINER_NAME="$2"
      shift 2
      ;;
    --team)
      TEAM_ID="$2"
      shift 2
      ;;
    --run-id)
      RUN_ID="$2"
      shift 2
      ;;
    --rviz)
      ENABLE_RVIZ="true"
      shift
      ;;
    --record)
      ENABLE_RECORD="true"
      shift
      ;;
    --robot-port)
      ROBOT_PORT="$2"
      shift 2
      ;;
    --gps-port)
      GPS_PORT="$2"
      shift 2
      ;;
    --gps-baud)
      GPS_BAUD="$2"
      shift 2
      ;;
    --artifacts-root)
      HOST_ARTIFACTS_ROOT="$2"
      shift 2
      ;;
    --workspace)
      WORKSPACE_HOST="$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --)
      shift
      EXTRA_ARGS=("$@")
      break
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

require_cmd docker

WORKSPACE_HOST="$(cd "$WORKSPACE_HOST" && pwd)"

if [[ -z "$RUN_ID" ]]; then
  RUN_ID="team${TEAM_ID}_$(date +%Y%m%d_%H%M%S)"
fi

if [[ -z "$HOST_ARTIFACTS_ROOT" ]]; then
  HOST_ARTIFACTS_ROOT="${WORKSPACE_HOST}/artifacts/part2_runs/${RUN_ID}"
fi
HOST_ARTIFACTS_ROOT="$(mkdir -p "$HOST_ARTIFACTS_ROOT" && cd "$HOST_ARTIFACTS_ROOT" && pwd)"

CONTAINER_ARTIFACTS_ROOT="$HOST_ARTIFACTS_ROOT"

if [[ -z "$CONTAINER_NAME" ]]; then
  CONTAINER_NAME="t${TEAM_ID}-${MODE}"
fi

if [[ -z "$IMAGE_NAME" ]]; then
  if ! IMAGE_NAME="$(detect_image)"; then
    echo "No supported course image found." >&2
    echo "Set AUTO4508_DOCKER_IMAGE or pass --image <name>." >&2
    exit 1
  fi
fi

EXTRA_ARGS_STR=""
if [[ ${#EXTRA_ARGS[@]} -gt 0 ]]; then
  EXTRA_ARGS_STR="$(shell_join "${EXTRA_ARGS[@]}")"
fi

COMMON_ENV='source /opt/ros/jazzy/setup.bash'
WORKSPACE_ENV='if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi'
EXPORT_ENV="export ARIACODA_PREFIX=\${ARIACODA_PREFIX:-/usr/local}; export AUTO4508_TEAM_ID=$(printf '%q' "$TEAM_ID"); export AUTO4508_RUN_ID=$(printf '%q' "$RUN_ID"); export AUTO4508_ARTIFACTS_ROOT=$(printf '%q' "$CONTAINER_ARTIFACTS_ROOT")"

case "$MODE" in
  shell)
    if [[ -n "$EXTRA_ARGS_STR" ]]; then
      CMD="${COMMON_ENV}; cd /workspace; ${WORKSPACE_ENV}; ${EXPORT_ENV}; exec ${EXTRA_ARGS_STR}"
    else
      CMD="${COMMON_ENV}; cd /workspace; ${WORKSPACE_ENV}; ${EXPORT_ENV}; exec bash"
    fi
    ;;
  prep)
    CMD="${COMMON_ENV}; cd /workspace; ${EXPORT_ENV}; ./setup_env.sh ${EXTRA_ARGS_STR}"
    ;;
  build)
    CMD="${COMMON_ENV}; cd /workspace; ${EXPORT_ENV}; colcon build --symlink-install ${EXTRA_ARGS_STR}"
    ;;
  real)
    CMD="${COMMON_ENV}; cd /workspace; ${WORKSPACE_ENV}; ${EXPORT_ENV}; ros2 launch p3at_bringup real.launch.py run_id:=${RUN_ID} artifacts_root:=${CONTAINER_ARTIFACTS_ROOT} rviz:=${ENABLE_RVIZ} record:=${ENABLE_RECORD} robot_port:=${ROBOT_PORT} gps_port:=${GPS_PORT} gps_baud:=${GPS_BAUD} ${EXTRA_ARGS_STR}"
    ;;
  vision)
    CMD="${COMMON_ENV}; cd /workspace; ${WORKSPACE_ENV}; ${EXPORT_ENV}; ros2 launch p3at_vision vision.launch.py run_id:=${RUN_ID} save_dir:=${CONTAINER_ARTIFACTS_ROOT}/photos ${EXTRA_ARGS_STR}"
    ;;
  *)
    echo "Unsupported mode: $MODE" >&2
    exit 2
    ;;
esac

DOCKER_ARGS=(
  run -it --rm
  --name "$CONTAINER_NAME"
  --privileged
  --network host
  --ipc host
  -v /dev:/dev
  -v /run/udev:/run/udev:ro
  -v "${WORKSPACE_HOST}:/workspace"
  -v "${WORKSPACE_HOST}:${WORKSPACE_HOST}"
  -w /workspace
  -e TZ="${TZ:-Australia/Perth}"
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  -e AUTO4508_TEAM_ID="$TEAM_ID"
  -e AUTO4508_RUN_ID="$RUN_ID"
  -e AUTO4508_ARTIFACTS_ROOT="$CONTAINER_ARTIFACTS_ROOT"
)

if [[ "$HOST_ARTIFACTS_ROOT" != "$WORKSPACE_HOST"* ]]; then
  DOCKER_ARGS+=(-v "${HOST_ARTIFACTS_ROOT}:${HOST_ARTIFACTS_ROOT}")
fi

if [[ -S /var/run/dbus/system_bus_socket ]]; then
  DOCKER_ARGS+=(
    -v /var/run/dbus:/var/run/dbus:ro
    -e DBUS_SYSTEM_BUS_ADDRESS=unix:path=/var/run/dbus/system_bus_socket
  )
fi

if [[ "$ENABLE_RVIZ" == "true" ]]; then
  if [[ -n "${DISPLAY:-}" ]]; then
    DOCKER_ARGS+=(
      -e DISPLAY="$DISPLAY"
      -e QT_X11_NO_MITSHM=1
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    )
  fi
  if [[ -n "${XAUTHORITY:-}" && -f "${XAUTHORITY}" ]]; then
    DOCKER_ARGS+=(
      -v "${XAUTHORITY}:/root/.Xauthority:ro"
      -e XAUTHORITY=/root/.Xauthority
    )
  fi
fi

echo "Using image: $IMAGE_NAME"
echo "Workspace:   $WORKSPACE_HOST"
echo "Run ID:      $RUN_ID"
echo "Artifacts:   $HOST_ARTIFACTS_ROOT"
echo "Container artifacts: $CONTAINER_ARTIFACTS_ROOT"
echo "Mode:        $MODE"

exec docker "${DOCKER_ARGS[@]}" "$IMAGE_NAME" bash -lc "$CMD"
