CONTROLS_REPO_PATH="$(realpath "$(dirname "${BASH_SOURCE[0]}")/../..")"
ROBOT_ID="2"
DATA_DIR="$CONTROLS_REPO_PATH/data"

# Create data directory if it doesn't exist
mkdir -p "$DATA_DIR"

# Find the next available index
INDEX=0
while [[ -d "$DATA_DIR/rosbag_telem_$INDEX" ]] || [[ -f "$DATA_DIR/telemetry_$INDEX.npz" ]]; do
    ((INDEX++))
done

ROSBAG_PATH="$DATA_DIR/rosbag_telem_$INDEX"
TELEM_PATH="$DATA_DIR/telemetry_$INDEX.npz"
ros2 bag record --all -o $ROSBAG_PATH
python3 $CONTROLS_REPO_PATH/analysis/scripts/telem_bag2np.py --bag $ROSBAG_PATH --robot $ROBOT_ID --output $TELEM_PATH
python3 $CONTROLS_REPO_PATH/analysis/scripts/telem_visualize.py --telemetry $TELEM_PATH