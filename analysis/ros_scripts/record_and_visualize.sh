CONTROLS_REPO_PATH="$(realpath "$(dirname "${BASH_SOURCE[0]}")/../..")"
ROBOT_ID="${1:-0}"
BAG_DIR="$CONTROLS_REPO_PATH/analysis/data/bags"
TELEM_DIR="$CONTROLS_REPO_PATH/analysis/data/telemetry"

# Create directories if they don't exist
mkdir -p "$BAG_DIR"
mkdir -p "$TELEM_DIR"

# Find the next available index
INDEX=0
while [[ -d "$BAG_DIR/rosbag_telem_$INDEX" ]] || [[ -f "$TELEM_DIR/telemetry_$INDEX.npz" ]]; do
    ((INDEX++))
done

ROSBAG_PATH="$BAG_DIR/rosbag_telem_$INDEX"
TELEM_PATH="$TELEM_DIR/telemetry_$INDEX.npz"
TOPIC="/robot_feedback/extended/robot${ROBOT_ID}"
ros2 bag record -o "$ROSBAG_PATH" "$TOPIC"
python3 "$CONTROLS_REPO_PATH/analysis/ros_scripts/telem_bag2np.py" --bag "$ROSBAG_PATH" --robot "$ROBOT_ID" --output "$TELEM_PATH"
cd "$CONTROLS_REPO_PATH/analysis" && python3 telem_visualize.py --telemetry "$TELEM_PATH"