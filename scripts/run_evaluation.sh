#!/usr/bin/env bash
# run_evaluation.sh — automated AMCL 3D accuracy evaluation
#
# Usage:
#   ./scripts/run_evaluation.sh <short_test|husky>
#
# Prerequisites:
#   - colcon build completed
#   - Demo data present: demo_data/short_test_ros2/ or bags/*.mcap + maps/*.pcd
#   - Python deps: pip install rosbags matplotlib open3d (open3d only for husky)
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Source ROS 2 and workspace
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "${WS_DIR}/install/setup.bash"
set -u

PROFILE="${1:-}"
if [[ -z "$PROFILE" ]]; then
    echo "Usage: $0 <short_test|husky>"
    exit 1
fi

# ── Profile configuration ───────────────────────────────────────────
case "$PROFILE" in
    short_test)
        BAG_PATH="${WS_DIR}/demo_data/short_test_ros2"
        MAP_PCD_FILE=""
        ODOM_TOPIC="/odom"
        PC2_TOPIC="/cloud"
        INPUT_MAP="/mapcloud"
        INPUT_ODOM="/odom"
        QOS_OVERRIDES=""
        BAG_RATE="1.0"
        # Bag duration 267s at rate 1.0 → ~267s wall time + startup buffer
        BAG_WALL_TIMEOUT=300
        NEED_INITIALPOSE=true
        INITIALPOSE_X="0.075"
        INITIALPOSE_Y="-0.020"
        INITIALPOSE_Z="0.0"
        INITIALPOSE_QZ="0.9999"
        INITIALPOSE_QW="0.0143"
        TITLE_TRAJ="amcl_3d short_test trajectory"
        TITLE_CMP="amcl_3d short_test: AMCL vs odom XY"
        OUT_TRAJ="${WS_DIR}/reports/assets/short_test_trajectory.png"
        OUT_CMP="${WS_DIR}/reports/assets/short_test_amcl_vs_odom_xy.png"
        CSV_FILE="${WS_DIR}/reports/short_test_current_pose.csv"
        MAP_PLOT_ARGS=("--bag-path" "$BAG_PATH" "--map-topic" "$INPUT_MAP")
        ;;
    husky)
        BAG_PATH="${WS_DIR}/bags/2024-08-23-11-05-41_0_clipped.mcap"
        MAP_PCD_FILE="${WS_DIR}/maps/kinematic_icp_husky.pcd"
        ODOM_TOPIC="/husky_velocity_controller/odom"
        PC2_TOPIC="/lidar_points"
        INPUT_MAP="/mapcloud"
        INPUT_ODOM="/husky_velocity_controller/odom"
        QOS_OVERRIDES="${WS_DIR}/src/amcl_3d/config/qos_override.yaml"
        BAG_RATE="1.0"
        # Bag duration 122s at rate 1.0 → ~122s wall time + startup buffer
        BAG_WALL_TIMEOUT=160
        NEED_INITIALPOSE=true
        INITIALPOSE_X="0.0"
        INITIALPOSE_Y="0.0"
        INITIALPOSE_Z="0.0"
        INITIALPOSE_QZ="0.0"
        INITIALPOSE_QW="1.0"
        TITLE_TRAJ="amcl_3d Husky trajectory"
        TITLE_CMP="amcl_3d Husky: AMCL vs odom XY"
        OUT_TRAJ="${WS_DIR}/reports/assets/husky_trajectory.png"
        OUT_CMP="${WS_DIR}/reports/assets/husky_amcl_vs_odom_xy.png"
        CSV_FILE="${WS_DIR}/reports/husky_current_pose.csv"
        MAP_PLOT_ARGS=("--map-pcd-file" "$MAP_PCD_FILE" "--no-bag-map" "--bag-path" "$BAG_PATH")
        ;;
    *)
        echo "Unknown profile: $PROFILE (choose short_test or husky)"
        exit 1
        ;;
esac

echo "=== Evaluation profile: $PROFILE ==="
echo "  BAG:  $BAG_PATH"
echo "  CSV:  $CSV_FILE"

# ── Helpers ──────────────────────────────────────────────────────────
LAUNCH_PID=""
CSV_PID=""

cleanup() {
    echo ">>> Cleaning up background processes..."
    for pid_var in CSV_PID LAUNCH_PID; do
        pid="${!pid_var:-}"
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
        fi
    done
}
trap cleanup EXIT

wait_for_service() {
    local service_name="$1"
    local timeout_sec="$2"
    local elapsed=0
    while [[ $elapsed -lt $timeout_sec ]]; do
        if ros2 service type "$service_name" >/dev/null 2>&1; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    echo "Service not available: $service_name" >&2
    return 1
}

# ── 1. Launch amcl_3d + bag play together via rosbag launch ────────
echo ">>> Launching amcl_3d with bag play (simultaneous start)..."
LAUNCH_ARGS=(
    "amcl_3d" "amcl_3d_rosbag.launch.py"
    "use_sim_time:=true"
    "open_rviz:=false"
    "bag_path:=${BAG_PATH}"
    "bag_rate:=${BAG_RATE}"
    "bag_loop:=false"
    "bag_start_paused:=true"
    "input_map:=${INPUT_MAP}"
    "input_odom:=${INPUT_ODOM}"
    "input_pc2:=${PC2_TOPIC}"
)

if [[ -n "${MAP_PCD_FILE:-}" ]]; then
    LAUNCH_ARGS+=("map_pcd_file:=${MAP_PCD_FILE}")
fi
if [[ -n "${QOS_OVERRIDES:-}" ]]; then
    LAUNCH_ARGS+=("qos_overrides_path:=${QOS_OVERRIDES}")
fi

ros2 launch "${LAUNCH_ARGS[@]}" 2>/dev/null &
LAUNCH_PID=$!
echo ">>> Waiting for nodes to initialize..."
sleep 10
wait_for_service /rosbag2_player/resume 30

# ── 2. Publish initial pose (if bag does not contain /initialpose) ──
if [[ "${NEED_INITIALPOSE}" == "true" ]]; then
    echo ">>> Publishing initial pose (x=${INITIALPOSE_X}, y=${INITIALPOSE_Y}, z=${INITIALPOSE_Z})"
    ros2 topic pub -r 2 --times 5 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
        "{header: {frame_id: 'map'}, pose: {pose: {position: {x: ${INITIALPOSE_X}, y: ${INITIALPOSE_Y}, z: ${INITIALPOSE_Z}}, orientation: {z: ${INITIALPOSE_QZ}, w: ${INITIALPOSE_QW}}}}}" \
        2>/dev/null
    sleep 1
fi

# ── 3. Start CSV recording ──────────────────────────────────────────
echo ">>> Recording /current_pose to $CSV_FILE"
mkdir -p "$(dirname "$CSV_FILE")"
ros2 topic echo --csv /current_pose geometry_msgs/msg/PoseStamped > "$CSV_FILE" &
CSV_PID=$!
sleep 1

# ── 4. Resume bag playback after recorder + initial pose are ready ──
echo ">>> Resuming bag playback..."
ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume "{}" >/dev/null
sleep 1

# ── 5. Wait for bag playback to finish ─────────────────────────────
echo ">>> Waiting for bag playback (~${BAG_WALL_TIMEOUT}s timeout)..."
ELAPSED=0
while kill -0 "$LAUNCH_PID" 2>/dev/null && [[ $ELAPSED -lt $BAG_WALL_TIMEOUT ]]; do
    sleep 5
    ELAPSED=$((ELAPSED + 5))
    # Check if CSV is still growing (bag is still playing)
    CSV_LINES=$(wc -l < "$CSV_FILE" 2>/dev/null || echo "0")
    echo "  [${ELAPSED}s] CSV lines: ${CSV_LINES}"
    if ! ros2 service type /rosbag2_player/resume >/dev/null 2>&1; then
        break
    fi
done

echo ">>> Bag playback finished or timeout reached."
sleep 3

# ── 6. Stop recording & launch ──────────────────────────────────────
echo ">>> Stopping CSV recording and launch..."
kill "$CSV_PID" 2>/dev/null || true
wait "$CSV_PID" 2>/dev/null || true
CSV_PID=""

kill "$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true
LAUNCH_PID=""

# ── 7. Generate plots ───────────────────────────────────────────────
echo ">>> Generating plots..."
mkdir -p "$(dirname "$OUT_TRAJ")"

python3 "${WS_DIR}/reports/generate_trajectory.py" \
    "$CSV_FILE" \
    --trajectory-output-png "$OUT_TRAJ" \
    --title "$TITLE_TRAJ" \
    --compare-odom-output-png "$OUT_CMP" \
    --compare-title "$TITLE_CMP" \
    --odom-topic "$ODOM_TOPIC" \
    "${MAP_PLOT_ARGS[@]}"

echo ""
echo "=== Evaluation complete: $PROFILE ==="
echo "  Trajectory plot : $OUT_TRAJ"
echo "  Comparison plot : $OUT_CMP"
echo "  CSV data        : $CSV_FILE"
