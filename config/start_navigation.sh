#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ORIGINCAR_SRC="${ORIGINCAR_SRC:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
export ORIGINCAR_SRC
WORKSPACE="${WORKSPACE:-$(cd "${ORIGINCAR_SRC}/../.." && pwd)}"
SETUP_FILE="${SETUP_FILE:-${WORKSPACE}/install/setup.bash}"

# ===== Common tuning parameters =====
MAP_FILE="${MAP_FILE:-${ORIGINCAR_SRC}/map/race_modify.yaml}"                                   # Main map YAML.
WAYPOINTS_FILE="${WAYPOINTS_FILE:-${ORIGINCAR_SRC}/origincar_system/config/waypoints.yaml}"     # Route waypoint YAML.
RVIZ_CONFIG_FILE="${RVIZ_CONFIG_FILE:-${ORIGINCAR_SRC}/default.rviz}"                           # RViz config.
PARAMS_FILE="${PARAMS_FILE:-${ORIGINCAR_SRC}/origincar_system/config/controller_params_keepout.yaml}"


LIDAR_TYPE="${LIDAR_TYPE:-n10}"                         # Lidar type: vp100 or n10.
BASE_USART_PORT_NAME="${BASE_USART_PORT_NAME:-/dev/ttyACM0}" # Base serial port.
CAMERA_DEVICE="${CAMERA_DEVICE:-/dev/video0}"              # USB camera device.
WAIT_FOR_NAV_START="${WAIT_FOR_NAV_START:-1}"              # 1=preload navigation and wait for /nav_start.
NAV_START_TOPIC="${NAV_START_TOPIC:-/nav_start}"           # Start signal topic used by the screen Start button.

INITIAL_POSE_X="${INITIAL_POSE_X:-0.002}"                 # AMCL initial x, meters.
INITIAL_POSE_Y="${INITIAL_POSE_Y:--0.083}"                 # AMCL initial y, meters.
INITIAL_POSE_A="${INITIAL_POSE_A:-0.441}"                  # AMCL initial yaw, radians.

LINEAR_SPEED="${LINEAR_SPEED:-0.75}"                       # Normal speed outside channel, m/s.
CHANNEL_LINEAR_SPEED="${CHANNEL_LINEAR_SPEED:-0.65}"       # Channel speed, m/s.
NAV2_LINEAR_SPEED="${NAV2_LINEAR_SPEED:-${LINEAR_SPEED}}"  # Nav2 desired speed in nav2_goals mode.
NAV2_MAX_LINEAR_SPEED="${NAV2_MAX_LINEAR_SPEED:-${NAV2_LINEAR_SPEED}}" # Velocity smoother x limit for Nav2.
NAV2_MIN_APPROACH_LINEAR_SPEED="${NAV2_MIN_APPROACH_LINEAR_SPEED:-0.16}" # Nav2 final approach speed.
NAV2_REGULATED_MIN_SPEED="${NAV2_REGULATED_MIN_SPEED:-0.20}" # Nav2 regulated minimum speed.
NAV2_APPROACH_VELOCITY_SCALING_DIST="${NAV2_APPROACH_VELOCITY_SCALING_DIST:-0.35}" # Keep intermediate Nav2 goals from slowing too early.
CHANNEL_WAYPOINT_RANGES="${CHANNEL_WAYPOINT_RANGES:-3-11}" # Waypoint range using channel speed.
LOOKAHEAD_DISTANCE="${LOOKAHEAD_DISTANCE:-0.38}"           # Lookahead distance, meters.
MAX_ANGULAR_Z="${MAX_ANGULAR_Z:-1.9}"                      # Max angular speed, rad/s.
TURN_ANGULAR_GAIN="${TURN_ANGULAR_GAIN:-1.2}"              # Forward turn gain.
TURN_MIN_SPEED_SCALE="${TURN_MIN_SPEED_SCALE:-0.55}"       # Min speed scale while turning.

REVERSE_ANGULAR_GAIN="${REVERSE_ANGULAR_GAIN:-1.1}"        # Reverse turn gain.
REVERSE_MIN_SPEED_SCALE="${REVERSE_MIN_SPEED_SCALE:-0.30}" # Min reverse speed scale while turning.
REVERSE_GOAL_LOOKAHEAD_DISTANCE="${REVERSE_GOAL_LOOKAHEAD_DISTANCE:-0.25}" # Reverse goal lookahead, meters.
REVERSE_PASS_RADIUS="${REVERSE_PASS_RADIUS:-0.25}"         # Reverse waypoint pass radius, meters.
ENABLE_REVERSE_APPROACH_NUDGE="${ENABLE_REVERSE_APPROACH_NUDGE:-true}" # Add small steering nudge near reverse waypoint.
REVERSE_APPROACH_NUDGE_WAYPOINT="${REVERSE_APPROACH_NUDGE_WAYPOINT:-2}" # Waypoint index for reverse nudge.
REVERSE_APPROACH_NUDGE_DISTANCE="${REVERSE_APPROACH_NUDGE_DISTANCE:-0.35}" # Distance to start reverse nudge, meters.
REVERSE_APPROACH_NUDGE_ANGULAR_Z="${REVERSE_APPROACH_NUDGE_ANGULAR_Z:--0.35}" # Reverse nudge angular speed.

OBSTACLE_ENABLE_FROM_WAYPOINT="${OBSTACLE_ENABLE_FROM_WAYPOINT:-0}" # Enable lidar obstacle avoidance from waypoint; 0=always.
STOP_WITHOUT_SCAN="${STOP_WITHOUT_SCAN:-true}"              # Stop instead of driving when /scan is missing or stale.
OBSTACLE_SCAN_TIMEOUT="${OBSTACLE_SCAN_TIMEOUT:-0.8}"       # Max age of /scan before holding position, seconds.
OBSTACLE_STOP_DISTANCE="${OBSTACLE_STOP_DISTANCE:-0.30}"     # Front obstacle stop distance, meters.
OBSTACLE_SLOW_DISTANCE="${OBSTACLE_SLOW_DISTANCE:-0.85}"     # Front obstacle slow distance, meters.
OBSTACLE_AVOID_DISTANCE="${OBSTACLE_AVOID_DISTANCE:-0.83}"   # Front obstacle steering distance, meters.
MIN_OBSTACLE_RANGE="${MIN_OBSTACLE_RANGE:-0.15}"             # Ignore scan points closer than this, meters.
FRONT_ANGLE_DEG="${FRONT_ANGLE_DEG:-35.0}"                   # Front scan sector half angle, degrees.
BACKUP_TRIGGER_TIME="${BACKUP_TRIGGER_TIME:-1.0}"            # Stop duration before backup recovery, seconds.
BACKUP_DURATION="${BACKUP_DURATION:-0.5}"                    # Backup recovery duration, seconds.
BACKUP_SPEED="${BACKUP_SPEED:-0.25}"                         # Backup recovery speed, m/s.
BACKUP_ANGULAR_Z="${BACKUP_ANGULAR_Z:-0.35}"                 # Backup recovery angular speed, rad/s.
BACKUP_STOP_DISTANCE="${BACKUP_STOP_DISTANCE:-0.22}"         # Rear safety distance for backup, meters.

ENABLE_SEGMENT_LOOKAHEAD="${ENABLE_SEGMENT_LOOKAHEAD:-true}" # Use segment lookahead for smoother waypoint passing.
ENABLE_SEGMENT_KEEPOUT_BIAS="${ENABLE_SEGMENT_KEEPOUT_BIAS:-true}" # Bias lookahead away from keepout when possible.
SEGMENT_KEEPOUT_LATERAL_SAMPLES="${SEGMENT_KEEPOUT_LATERAL_SAMPLES:-2}" # Lateral keepout samples per side.
SMOOTH_PATH_KEEPOUT_CLEARANCE="${SMOOTH_PATH_KEEPOUT_CLEARANCE:-0.08}" # Preferred keepout clearance, meters.
SMOOTH_PATH_KEEPOUT_WEIGHT="${SMOOTH_PATH_KEEPOUT_WEIGHT:-0.45}" # Keepout score weight.
ENABLE_MISSED_WAYPOINT_RECOVERY="${ENABLE_MISSED_WAYPOINT_RECOVERY:-true}" # Recover to next point after passing a missed waypoint.
MISSED_WAYPOINT_MAX_DISTANCE="${MISSED_WAYPOINT_MAX_DISTANCE:-0.80}" # Max distance for missed waypoint recovery, meters.
MISSED_WAYPOINT_PROJECTION_MARGIN="${MISSED_WAYPOINT_PROJECTION_MARGIN:-0.08}" # Projection margin for missed waypoint recovery, meters.
KEEPOUT_MASK_TOPIC="${KEEPOUT_MASK_TOPIC:-/filter_mask}"      # Keepout mask topic for soft path bias.

ENABLE_QR_SKIP_FIRST="${ENABLE_QR_SKIP_FIRST:-false}"        # Keep waypoints 1-3 shared; do not skip waypoint 1 after QR.
QR_TOPIC="${QR_TOPIC:-/qr_info}"                             # QR result topic.

# ===== Internal defaults: normally do not tune =====
KEEPOUT_MAP_FILE="${KEEPOUT_MAP_FILE:-${ORIGINCAR_SRC}/map/race_keepout.yaml}" # map_only fallback keepout map.
RVIZ_DELAY="${RVIZ_DELAY:-3}"                              # Delay after starting RViz.
BRINGUP_DELAY="${BRINGUP_DELAY:-5}"                         # Delay after base/lidar bringup.
NAV2_DELAY="${NAV2_DELAY:-15}"                              # Delay after Nav2 startup.
QR_DELAY="${QR_DELAY:-0}"                                   # Delay after QR node startup.
USE_SIM_TIME="${USE_SIM_TIME:-false}"                       # Keep false on real robot.
BASE_SERIAL_BAUD_RATE="${BASE_SERIAL_BAUD_RATE:-115200}"    # Base serial baud rate.
LIDAR_SERIAL_PORT="${LIDAR_SERIAL_PORT:-/dev/ttyACM1}"      # N10 serial port; VP100 usually ignores this.

START_CAMERA="${START_CAMERA:-1}"                           # Start camera.
START_QR_DETECTOR="${START_QR_DETECTOR:-1}"                 # Start QR detector.
START_IMAGE_ANALYZER="${START_IMAGE_ANALYZER:-1}"           # Start image analyzer for waypoint image-to-text.
START_MULTI_POINT="${START_MULTI_POINT:-1}"                 # Start waypoint navigation.
NAV_MODE="${NAV_MODE:-path_follower}"                       # path_follower enables the custom lidar obstacle avoidance.
START_FULL_NAV2="${START_FULL_NAV2:-1}"                     # 1=full Nav2, 0=map_only fallback.
START_CONTROL_PANEL="${START_CONTROL_PANEL:-1}"             # Start the on-screen QR/AI display panel only.

ENABLE_QR_DIRECTION_SELECT="${ENABLE_QR_DIRECTION_SELECT:-true}" # Allow QR to select route direction.
LOCK_QR_DIRECTION_AFTER_SELECT="${LOCK_QR_DIRECTION_AFTER_SELECT:-true}" # Lock route direction after the first valid QR selection.
USE_QR_PARITY_DIRECTION="${USE_QR_PARITY_DIRECTION:-true}"  # true: odd=clockwise, even=counterclockwise.
CLOCKWISE_QR_VALUE="${CLOCKWISE_QR_VALUE:-1}"               # Clockwise QR value when parity rule is off.
COUNTERCLOCKWISE_QR_VALUE="${COUNTERCLOCKWISE_QR_VALUE:-2}" # Counterclockwise QR value when parity rule is off.
ROUTE_DIRECTION="${ROUTE_DIRECTION:-clockwise}"             # clockwise fallback; QR can switch to counterclockwise.
ROUTE_SWITCH_START_WAYPOINT="${ROUTE_SWITCH_START_WAYPOINT:-4}" # Apply QR route selection from this waypoint.
QR_IMAGE_TOPIC="${QR_IMAGE_TOPIC:-/image}"                  # QR input image topic.
QR_SCAN_HZ="${QR_SCAN_HZ:-30.0}"                            # QR scan rate.
QR_DETECTOR_ENGINE="${QR_DETECTOR_ENGINE:-zbar}"            # QR detector engine.
QR_RESIZE_WIDTH="${QR_RESIZE_WIDTH:-0}"                     # QR resize width; 0 disables resize.
QR_REPEAT_PUBLISH_INTERVAL="${QR_REPEAT_PUBLISH_INTERVAL:-5.0}" # Same-code publish interval.
QR_MIN_CONFIRMATIONS="${QR_MIN_CONFIRMATIONS:-1}"           # Publish QR immediately after one valid detection.
QR_CONFIRMATION_TIMEOUT="${QR_CONFIRMATION_TIMEOUT:-0.7}"   # Max seconds between matching QR confirmations.

IMAGE_ANALYSIS_WAYPOINT="${IMAGE_ANALYSIS_WAYPOINT:-7}"     # Optional image analyzer trigger waypoint.
IMAGE_ANALYSIS_TRIGGER_TOPIC="${IMAGE_ANALYSIS_TRIGGER_TOPIC:-/person_trigger}" # Image analyzer trigger topic.
IMAGE_ANALYSIS_RESULT_TOPIC="${IMAGE_ANALYSIS_RESULT_TOPIC:-/image_ai}" # Image analyzer result topic.
IMAGE_ANALYSIS_IMAGE_TOPIC="${IMAGE_ANALYSIS_IMAGE_TOPIC:-/image}" # Image analyzer image topic.
IMAGE_ANALYSIS_IMAGE_MSG_TYPE="${IMAGE_ANALYSIS_IMAGE_MSG_TYPE:-compressed}" # compressed or raw.

PASS_RADIUS="${PASS_RADIUS:-0.25}"                          # Global fallback pass radius; waypoint YAML can override.
NAV2_SUCCESS_DISTANCE_TOLERANCE="${NAV2_SUCCESS_DISTANCE_TOLERANCE:-0.45}" # Accept Nav2 success within this distance.
SMOOTH_INTERMEDIATE_GOALS="${SMOOTH_INTERMEDIATE_GOALS:-true}" # Send pass-through targets for intermediate Nav2 goals.
INTERMEDIATE_GOAL_LOOKAHEAD="${INTERMEDIATE_GOAL_LOOKAHEAD:-0.90}" # Distance past each intermediate waypoint for pass-through goals.
ENABLE_SMOOTH_KEEPOUT_PATH="${ENABLE_SMOOTH_KEEPOUT_PATH:-false}" # Old whole-path smoothing, default off.
SMOOTH_PATH_MIN_WAYPOINT="${SMOOTH_PATH_MIN_WAYPOINT:-4}"   # Old smoothing start waypoint.
SMOOTH_PATH_MAX_WAYPOINT="${SMOOTH_PATH_MAX_WAYPOINT:-12}"   # Old smoothing end waypoint.
SMOOTH_PATH_SAMPLES_PER_SEGMENT="${SMOOTH_PATH_SAMPLES_PER_SEGMENT:-16}" # Curve samples per waypoint segment.
ENABLE_SKIP_OVERSHOT_WAYPOINTS="${ENABLE_SKIP_OVERSHOT_WAYPOINTS:-false}" # Old overshot skip logic, default off.
OVERSHOT_WAYPOINT_MARGIN="${OVERSHOT_WAYPOINT_MARGIN:-0.05}" # Old overshot margin.
OVERSHOT_WAYPOINT_MAX_DISTANCE="${OVERSHOT_WAYPOINT_MAX_DISTANCE:-0.80}" # Old overshot max distance.
OVERSHOT_NEXT_WAYPOINT_MAX_DISTANCE="${OVERSHOT_NEXT_WAYPOINT_MAX_DISTANCE:-0.35}" # Old overshot next waypoint distance.
SKIP_OVERSHOT_MIN_WAYPOINT="${SKIP_OVERSHOT_MIN_WAYPOINT:-5}" # Old overshot skip min waypoint.
SKIP_OVERSHOT_MAX_WAYPOINT="${SKIP_OVERSHOT_MAX_WAYPOINT:-9}" # Old overshot skip max waypoint.

ENABLE_KEEPOUT_AVOIDANCE="${ENABLE_KEEPOUT_AVOIDANCE:-false}" # Custom hard keepout avoidance, default off.
KEEPOUT_OCCUPIED_THRESHOLD="${KEEPOUT_OCCUPIED_THRESHOLD:-80}" # Keepout occupied threshold.
KEEPOUT_STOP_DISTANCE="${KEEPOUT_STOP_DISTANCE:-0.12}"     # Custom keepout stop distance.
KEEPOUT_SLOW_DISTANCE="${KEEPOUT_SLOW_DISTANCE:-0.45}"     # Custom keepout slow distance.
KEEPOUT_AVOID_DISTANCE="${KEEPOUT_AVOID_DISTANCE:-0.35}"   # Custom keepout avoid distance.
KEEPOUT_SIDE_SAMPLE_DISTANCE="${KEEPOUT_SIDE_SAMPLE_DISTANCE:-0.35}" # Custom keepout side sample distance.
ENABLE_KEEPOUT_HARD_STOP="${ENABLE_KEEPOUT_HARD_STOP:-false}" # Treat keepout as hard stop.
KEEPOUT_MIN_SPEED_SCALE="${KEEPOUT_MIN_SPEED_SCALE:-0.60}" # Custom keepout min speed scale.
KEEPOUT_ESCAPE_DURATION="${KEEPOUT_ESCAPE_DURATION:-0.80}" # Custom keepout escape duration.
KEEPOUT_ESCAPE_SPEED="${KEEPOUT_ESCAPE_SPEED:-0.20}"       # Custom keepout escape speed.
MAX_GOAL_RETRIES="${MAX_GOAL_RETRIES:-5}"                  # nav2_goals retry count.

case "${ENABLE_QR_SKIP_FIRST}" in
  1|true|TRUE|True|yes|YES|Yes|on|ON|On)
    ENABLE_QR_SKIP_FIRST="true"
    ;;
  0|false|FALSE|False|no|NO|No|off|OFF|Off)
    ENABLE_QR_SKIP_FIRST="false"
    ;;
esac

case "${WAIT_FOR_NAV_START}" in
  1|true|TRUE|True|yes|YES|Yes|on|ON|On)
    WAIT_FOR_NAV_START="true"
    ;;
  0|false|FALSE|False|no|NO|No|off|OFF|Off)
    WAIT_FOR_NAV_START="false"
    ;;
esac

case "${LIDAR_TYPE}" in
  vp100|VP100|vp100l|VP100L)
    LIDAR_TYPE="vp100"
    START_VP100_LIDAR="true"
    START_N10_LIDAR="false"
    ;;
  n10|N10|lslidar|LSLIDAR)
    LIDAR_TYPE="n10"
    START_VP100_LIDAR="false"
    START_N10_LIDAR="true"
    ;;
  *)
    echo "[start_navigation] Unsupported LIDAR_TYPE='${LIDAR_TYPE}'. Use 'vp100' or 'n10'." >&2
    exit 1
    ;;
esac
PIDS=()
CLEANED_UP=0
RUNTIME_NAV2_PARAMS_FILE=""

load_dashscope_api_key_from_bashrc() {
  if [[ -n "${DASHSCOPE_API_KEY:-}" ]]; then
    export DASHSCOPE_API_KEY
    return
  fi

  local bashrc="${HOME}/.bashrc"
  if [[ ! -f "${bashrc}" ]]; then
    return
  fi

  local line
  line="$(grep -E '^[[:space:]]*(export[[:space:]]+)?DASHSCOPE_API_KEY=' "${bashrc}" | tail -n 1 || true)"
  if [[ -z "${line}" ]]; then
    return
  fi

  local value="${line#*=}"
  value="${value%$'\r'}"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  value="${value%\"}"
  value="${value#\"}"
  value="${value%\'}"
  value="${value#\'}"
  if [[ -n "${value}" ]]; then
    export DASHSCOPE_API_KEY="${value}"
  fi
}

cleanup() {
  if [[ "${CLEANED_UP}" == "1" ]]; then
    return
  fi
  CLEANED_UP=1
  echo
  echo "[start_navigation] Stopping launched processes..."
  for pid in "${PIDS[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
  if [[ "${RUNTIME_NAV2_PARAMS_FILE}" == /tmp/origincar_nav2_params.*.yaml ]]; then
    rm -f "${RUNTIME_NAV2_PARAMS_FILE}" 2>/dev/null || true
  fi
}

start_process() {
  local name="$1"
  shift
  echo "[start_navigation] Starting ${name}: $*"
  "$@" &
  PIDS+=("$!")
}

start_rviz() {
  if [[ -f "${RVIZ_CONFIG_FILE}" ]]; then
    start_process "RViz2" rviz2 -d "${RVIZ_CONFIG_FILE}" --ros-args -p use_sim_time:="${USE_SIM_TIME}"
  else
    start_process "RViz2" rviz2 --ros-args -p use_sim_time:="${USE_SIM_TIME}"
  fi
}

prepare_nav2_params() {
  if [[ -n "${RUNTIME_NAV2_PARAMS_FILE}" ]]; then
    return
  fi
  if [[ ! -f "${PARAMS_FILE}" ]]; then
    echo "[start_navigation] Missing Nav2 params file: ${PARAMS_FILE}" >&2
    exit 1
  fi

  RUNTIME_NAV2_PARAMS_FILE="$(mktemp /tmp/origincar_nav2_params.XXXXXX.yaml)"
  cp "${PARAMS_FILE}" "${RUNTIME_NAV2_PARAMS_FILE}"
  sed -i -E \
    -e "s/^([[:space:]]*desired_linear_vel:).*/\\1 ${NAV2_LINEAR_SPEED}/" \
    -e "s/^([[:space:]]*min_approach_linear_velocity:).*/\\1 ${NAV2_MIN_APPROACH_LINEAR_SPEED}/" \
    -e "s/^([[:space:]]*approach_velocity_scaling_dist:).*/\\1 ${NAV2_APPROACH_VELOCITY_SCALING_DIST}/" \
    -e "s/^([[:space:]]*regulated_linear_scaling_min_speed:).*/\\1 ${NAV2_REGULATED_MIN_SPEED}/" \
    -e "s/^([[:space:]]*max_velocity:).*/\\1 [${NAV2_MAX_LINEAR_SPEED}, 0.0, ${MAX_ANGULAR_Z}]/" \
    "${RUNTIME_NAV2_PARAMS_FILE}"
  echo "[start_navigation] Nav2 speed: desired=${NAV2_LINEAR_SPEED} m/s, max=${NAV2_MAX_LINEAR_SPEED} m/s, approach_scaling=${NAV2_APPROACH_VELOCITY_SCALING_DIST} m."
}

if [[ ! -f "${SETUP_FILE}" ]]; then
  echo "[start_navigation] Missing ROS setup file: ${SETUP_FILE}" >&2
  echo "[start_navigation] Build first: cd ${WORKSPACE} && colcon build --symlink-install" >&2
  exit 1
fi

source "${SETUP_FILE}"

# FastDDS 参数：避免共享内存端口锁失败，例如 "Failed init_port fastrtps_port..."。
export ROS_DISABLE_LOANED_MESSAGES=1
export RMW_FASTRTPS_USE_QOS_FROM_XML=0
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4


unset FASTRTPS_DEFAULT_PROFILES_FILE

trap cleanup EXIT INT TERM

start_process "base bringup" ros2 launch origincar_base origincar_bringup.launch.xml \
  start_camera:="${START_CAMERA}" \
  use_sim_time:="${USE_SIM_TIME}" \
  base_usart_port_name:="${BASE_USART_PORT_NAME}" \
  base_serial_baud_rate:="${BASE_SERIAL_BAUD_RATE}" \
  lidar_type:="${LIDAR_TYPE}" \
  start_vp100_lidar:="${START_VP100_LIDAR}" \
  start_n10_lidar:="${START_N10_LIDAR}" \
  lidar_serial_port:="${LIDAR_SERIAL_PORT}" \
  camera_device:="${CAMERA_DEVICE}"
sleep "${BRINGUP_DELAY}"

if [[ "${START_QR_DETECTOR}" == "1" ]]; then
  if [[ "${START_CAMERA}" != "1" ]]; then
    echo "[start_navigation] Warning: QR detector needs USB camera image topic, but START_CAMERA is not 1."
  fi
  start_process "QR detector" ros2 run qr_detector qr_detector_node --ros-args \
    -p image_topic:="${QR_IMAGE_TOPIC}" \
    -p qr_topic:="${QR_TOPIC}" \
    -p scan_hz:="${QR_SCAN_HZ}" \
    -p detector_engine:="${QR_DETECTOR_ENGINE}" \
    -p resize_width:="${QR_RESIZE_WIDTH}" \
    -p repeat_publish_interval:="${QR_REPEAT_PUBLISH_INTERVAL}" \
    -p min_confirmations:="${QR_MIN_CONFIRMATIONS}" \
    -p confirmation_timeout:="${QR_CONFIRMATION_TIMEOUT}"
  sleep "${QR_DELAY}"
fi

if [[ "${ENABLE_QR_DIRECTION_SELECT}" == "true" && "${START_QR_DETECTOR}" != "1" ]]; then
  echo "[start_navigation] Warning: QR route direction select is enabled, but START_QR_DETECTOR is not 1."
fi

if [[ "${START_IMAGE_ANALYZER}" == "1" && "${START_CAMERA}" != "1" ]]; then
  echo "[start_navigation] Warning: image analyzer needs USB camera image topic, but START_CAMERA is not 1."
fi
if [[ "${START_IMAGE_ANALYZER}" == "1" ]]; then
  load_dashscope_api_key_from_bashrc
  if [[ -z "${DASHSCOPE_API_KEY:-}" ]]; then
    echo "[start_navigation] Warning: DASHSCOPE_API_KEY is empty; image analyzer will show an error at waypoint ${IMAGE_ANALYSIS_WAYPOINT}."
  fi
fi

if [[ "${START_FULL_NAV2}" == "1" ]]; then
  prepare_nav2_params
  start_process "Nav2 AMCL keepout" ros2 launch origincar_system nav2_amcl_keepout.launch.xml \
    map:="${MAP_FILE}" \
    params_file:="${RUNTIME_NAV2_PARAMS_FILE}" \
    initial_pose_x:="${INITIAL_POSE_X}" \
    initial_pose_y:="${INITIAL_POSE_Y}" \
    initial_pose_a:="${INITIAL_POSE_A}" \
    use_sim_time:="${USE_SIM_TIME}"
else
  start_process "map and AMCL localization" ros2 launch origincar_system map_only.launch.xml \
    map:="${MAP_FILE}" \
    keepout_map:="${KEEPOUT_MAP_FILE}" \
    use_sim_time:="${USE_SIM_TIME}"
fi
sleep "${NAV2_DELAY}"

start_rviz
sleep "${RVIZ_DELAY}"

if [[ "${START_CONTROL_PANEL}" == "1" ]]; then
    start_process "control panel" ros2 run origincar_system control_panel --ros-args \
    -p qr_topic:="${QR_TOPIC}" \
    -p image_ai_topic:="${IMAGE_ANALYSIS_RESULT_TOPIC}" \
    -p start_topic:="${NAV_START_TOPIC}" \
    -p use_qr_parity_direction:="${USE_QR_PARITY_DIRECTION}" \
    -p clockwise_qr_value:="${CLOCKWISE_QR_VALUE}" \
    -p counterclockwise_qr_value:="${COUNTERCLOCKWISE_QR_VALUE}"
fi

if [[ "${START_MULTI_POINT}" == "1" ]]; then
  if [[ "${NAV_MODE}" == "nav2_goals" ]]; then
    start_process "multi-point navigation" ros2 launch origincar_system multi_point_nav.launch.xml \
      waypoints_file:="${WAYPOINTS_FILE}" \
      action_type:=ordered_smooth \
      wait_for_start:="${WAIT_FOR_NAV_START}" \
      start_topic:="${NAV_START_TOPIC}" \
      pass_radius:="${PASS_RADIUS}" \
      nav2_success_distance_tolerance:="${NAV2_SUCCESS_DISTANCE_TOLERANCE}" \
      smooth_intermediate_goals:="${SMOOTH_INTERMEDIATE_GOALS}" \
      intermediate_goal_lookahead:="${INTERMEDIATE_GOAL_LOOKAHEAD}" \
      missed_waypoint_max_distance:="${MISSED_WAYPOINT_MAX_DISTANCE}" \
      missed_waypoint_projection_margin:="${MISSED_WAYPOINT_PROJECTION_MARGIN}" \
      max_goal_retries:="${MAX_GOAL_RETRIES}" \
      route_direction:="${ROUTE_DIRECTION}" \
      route_switch_start_waypoint:="${ROUTE_SWITCH_START_WAYPOINT}" \
      enable_qr_direction_select:="${ENABLE_QR_DIRECTION_SELECT}" \
      qr_topic:="${QR_TOPIC}" \
      lock_qr_direction_after_select:="${LOCK_QR_DIRECTION_AFTER_SELECT}" \
      use_qr_parity_direction:="${USE_QR_PARITY_DIRECTION}" \
      clockwise_qr_value:="${CLOCKWISE_QR_VALUE}" \
      counterclockwise_qr_value:="${COUNTERCLOCKWISE_QR_VALUE}" \
      start_image_analyzer:="${START_IMAGE_ANALYZER}" \
      image_analysis_waypoint:="${IMAGE_ANALYSIS_WAYPOINT}" \
      image_analysis_trigger_topic:="${IMAGE_ANALYSIS_TRIGGER_TOPIC}" \
      image_topic:="${IMAGE_ANALYSIS_IMAGE_TOPIC}" \
      image_msg_type:="${IMAGE_ANALYSIS_IMAGE_MSG_TYPE}" \
      image_analysis_result_topic:="${IMAGE_ANALYSIS_RESULT_TOPIC}" \
      use_sim_time:="${USE_SIM_TIME}"
  else
    start_process "smooth path follower" ros2 launch origincar_system smooth_path_follower.launch.xml \
      waypoints_file:="${WAYPOINTS_FILE}" \
      wait_for_start:="${WAIT_FOR_NAV_START}" \
      start_topic:="${NAV_START_TOPIC}" \
      pass_radius:="${PASS_RADIUS}" \
      lookahead_distance:="${LOOKAHEAD_DISTANCE}" \
      enable_segment_lookahead:="${ENABLE_SEGMENT_LOOKAHEAD}" \
      enable_segment_keepout_bias:="${ENABLE_SEGMENT_KEEPOUT_BIAS}" \
      segment_keepout_lateral_samples:="${SEGMENT_KEEPOUT_LATERAL_SAMPLES}" \
      enable_smooth_keepout_path:="${ENABLE_SMOOTH_KEEPOUT_PATH}" \
      smooth_path_min_waypoint:="${SMOOTH_PATH_MIN_WAYPOINT}" \
      smooth_path_max_waypoint:="${SMOOTH_PATH_MAX_WAYPOINT}" \
      smooth_path_samples_per_segment:="${SMOOTH_PATH_SAMPLES_PER_SEGMENT}" \
      smooth_path_keepout_clearance:="${SMOOTH_PATH_KEEPOUT_CLEARANCE}" \
      smooth_path_keepout_weight:="${SMOOTH_PATH_KEEPOUT_WEIGHT}" \
      enable_skip_overshot_waypoints:="${ENABLE_SKIP_OVERSHOT_WAYPOINTS}" \
      enable_missed_waypoint_recovery:="${ENABLE_MISSED_WAYPOINT_RECOVERY}" \
      missed_waypoint_max_distance:="${MISSED_WAYPOINT_MAX_DISTANCE}" \
      missed_waypoint_projection_margin:="${MISSED_WAYPOINT_PROJECTION_MARGIN}" \
      overshot_waypoint_margin:="${OVERSHOT_WAYPOINT_MARGIN}" \
      overshot_waypoint_max_distance:="${OVERSHOT_WAYPOINT_MAX_DISTANCE}" \
      overshot_next_waypoint_max_distance:="${OVERSHOT_NEXT_WAYPOINT_MAX_DISTANCE}" \
      skip_overshot_min_waypoint:="${SKIP_OVERSHOT_MIN_WAYPOINT}" \
      skip_overshot_max_waypoint:="${SKIP_OVERSHOT_MAX_WAYPOINT}" \
      linear_speed:="${LINEAR_SPEED}" \
      channel_linear_speed:="${CHANNEL_LINEAR_SPEED}" \
      channel_waypoint_ranges:="${CHANNEL_WAYPOINT_RANGES}" \
      max_angular_z:="${MAX_ANGULAR_Z}" \
      turn_angular_gain:="${TURN_ANGULAR_GAIN}" \
      turn_min_speed_scale:="${TURN_MIN_SPEED_SCALE}" \
      reverse_angular_gain:="${REVERSE_ANGULAR_GAIN}" \
      reverse_min_speed_scale:="${REVERSE_MIN_SPEED_SCALE}" \
      reverse_goal_lookahead_distance:="${REVERSE_GOAL_LOOKAHEAD_DISTANCE}" \
      reverse_pass_radius:="${REVERSE_PASS_RADIUS}" \
      enable_reverse_approach_nudge:="${ENABLE_REVERSE_APPROACH_NUDGE}" \
      reverse_approach_nudge_waypoint:="${REVERSE_APPROACH_NUDGE_WAYPOINT}" \
      reverse_approach_nudge_distance:="${REVERSE_APPROACH_NUDGE_DISTANCE}" \
      reverse_approach_nudge_angular_z:="${REVERSE_APPROACH_NUDGE_ANGULAR_Z}" \
      stop_without_scan:="${STOP_WITHOUT_SCAN}" \
      scan_timeout:="${OBSTACLE_SCAN_TIMEOUT}" \
      obstacle_stop_distance:="${OBSTACLE_STOP_DISTANCE}" \
      obstacle_enable_from_waypoint:="${OBSTACLE_ENABLE_FROM_WAYPOINT}" \
      obstacle_slow_distance:="${OBSTACLE_SLOW_DISTANCE}" \
      obstacle_avoid_distance:="${OBSTACLE_AVOID_DISTANCE}" \
      min_obstacle_range:="${MIN_OBSTACLE_RANGE}" \
      front_angle_deg:="${FRONT_ANGLE_DEG}" \
      enable_keepout_avoidance:="${ENABLE_KEEPOUT_AVOIDANCE}" \
      keepout_mask_topic:="${KEEPOUT_MASK_TOPIC}" \
      keepout_occupied_threshold:="${KEEPOUT_OCCUPIED_THRESHOLD}" \
      keepout_stop_distance:="${KEEPOUT_STOP_DISTANCE}" \
      keepout_slow_distance:="${KEEPOUT_SLOW_DISTANCE}" \
      keepout_avoid_distance:="${KEEPOUT_AVOID_DISTANCE}" \
      keepout_side_sample_distance:="${KEEPOUT_SIDE_SAMPLE_DISTANCE}" \
      enable_keepout_hard_stop:="${ENABLE_KEEPOUT_HARD_STOP}" \
      keepout_min_speed_scale:="${KEEPOUT_MIN_SPEED_SCALE}" \
      keepout_escape_duration:="${KEEPOUT_ESCAPE_DURATION}" \
      keepout_escape_speed:="${KEEPOUT_ESCAPE_SPEED}" \
      backup_trigger_time:="${BACKUP_TRIGGER_TIME}" \
      backup_duration:="${BACKUP_DURATION}" \
      backup_speed:="${BACKUP_SPEED}" \
      backup_angular_z:="${BACKUP_ANGULAR_Z}" \
      backup_stop_distance:="${BACKUP_STOP_DISTANCE}" \
      enable_qr_skip_first:="${ENABLE_QR_SKIP_FIRST}" \
      qr_topic:="${QR_TOPIC}" \
      route_direction:="${ROUTE_DIRECTION}" \
      route_switch_start_waypoint:="${ROUTE_SWITCH_START_WAYPOINT}" \
      enable_qr_direction_select:="${ENABLE_QR_DIRECTION_SELECT}" \
      lock_qr_direction_after_select:="${LOCK_QR_DIRECTION_AFTER_SELECT}" \
      use_qr_parity_direction:="${USE_QR_PARITY_DIRECTION}" \
      clockwise_qr_value:="${CLOCKWISE_QR_VALUE}" \
      counterclockwise_qr_value:="${COUNTERCLOCKWISE_QR_VALUE}" \
      start_image_analyzer:="${START_IMAGE_ANALYZER}" \
      image_analysis_waypoint:="${IMAGE_ANALYSIS_WAYPOINT}" \
      image_analysis_trigger_topic:="${IMAGE_ANALYSIS_TRIGGER_TOPIC}" \
      image_topic:="${IMAGE_ANALYSIS_IMAGE_TOPIC}" \
      image_msg_type:="${IMAGE_ANALYSIS_IMAGE_MSG_TYPE}" \
      image_analysis_result_topic:="${IMAGE_ANALYSIS_RESULT_TOPIC}" \
      use_sim_time:="${USE_SIM_TIME}"
  fi

  if [[ "${WAIT_FOR_NAV_START}" == "true" ]]; then
    echo
    echo "[start_navigation] Navigation nodes are preloaded and waiting on ${NAV_START_TOPIC}."
    if [[ "${START_CONTROL_PANEL}" == "1" ]]; then
      echo "[start_navigation] Click Start on the control panel to start moving."
    else
      read -r -p "[start_navigation] Press Enter to send the start signal..." _ || true
      ros2 topic pub --once --qos-durability transient_local "${NAV_START_TOPIC}" std_msgs/msg/Bool "{data: true}" >/dev/null || \
        ros2 topic pub --once "${NAV_START_TOPIC}" std_msgs/msg/Bool "{data: true}" >/dev/null
    fi
  fi
else
  echo
  echo "[start_navigation] RViz2, base bringup, and Nav2 are running."
  echo "[start_navigation] Multi-point navigation is disabled; the car will not start moving."
  echo "[start_navigation] To start patrol, run with START_MULTI_POINT=1."
fi

echo "[start_navigation] All requested processes are running."
echo "[start_navigation] Press Ctrl+C in this terminal to stop them."
wait || true
