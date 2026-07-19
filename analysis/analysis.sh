#!/usr/bin/env bash
#
# Record (or load) robot telemetry and visualize it.
#
# Usage:
#   analysis.sh [-r ROBOT] [PATH]
#
#   -r ROBOT   Robot number to extract. If omitted, the robot is inferred from
#              the bag (a warning lists the available robots and the lowest
#              index is used when more than one stream contains data).
#   PATH       Optional input path:
#                * a bag directory      -> converted and visualized (no record)
#                * an .npz file         -> visualized directly (no record/convert)
#              If omitted, a new bag is recorded using the ROS2 default datetime
#              naming convention (rosbag2_<YYYY_MM_DD-HH_MM_SS>).
#
# Telemetry .npz files always share the bag's basename with an .npz extension.

set -euo pipefail

CONTROLS_REPO_PATH="$(realpath "$(dirname "${BASH_SOURCE[0]}")/..")"

# Activate the controls Python virtual environment if it isn't already active.
if [[ -z "${VIRTUAL_ENV:-}" ]]; then
    VENV_ACTIVATE="$CONTROLS_REPO_PATH/.venv/bin/activate"
    if [[ ! -f "$VENV_ACTIVATE" ]]; then
        echo "Error: Python virtual environment not found at $CONTROLS_REPO_PATH/.venv" >&2
        echo "       Run 'uv sync' in $CONTROLS_REPO_PATH first." >&2
        exit 1
    fi
    # shellcheck source=/dev/null
    source "$VENV_ACTIVATE"
fi

ANALYSIS_DIR="$CONTROLS_REPO_PATH/analysis"
BAG_DIR="$ANALYSIS_DIR/data/bags"
TELEM_DIR="$ANALYSIS_DIR/data/telemetry"
BAG2NP="$ANALYSIS_DIR/ros_scripts/telem_bag2np.py"

ROBOT=""
INPUT_PATH=""

usage() {
    echo "Usage: $(basename "$0") [-r ROBOT] [PATH]" >&2
    exit 1
}

while getopts ":r:h" opt; do
    case "$opt" in
        r) ROBOT="$OPTARG" ;;
        h) usage ;;
        :) echo "Error: -$OPTARG requires an argument" >&2; usage ;;
        \?) echo "Error: unknown option -$OPTARG" >&2; usage ;;
    esac
done
shift $((OPTIND - 1))

if [[ $# -gt 1 ]]; then
    echo "Error: at most one positional PATH may be provided" >&2
    usage
fi
INPUT_PATH="${1:-}"

mkdir -p "$BAG_DIR" "$TELEM_DIR"

# Resolve the telemetry .npz, recording/converting as needed.
if [[ -n "$INPUT_PATH" && -f "$INPUT_PATH" && "$INPUT_PATH" == *.npz ]]; then
    # Pre-existing telemetry: visualize directly.
    TELEM_PATH="$INPUT_PATH"
else
    if [[ -n "$INPUT_PATH" ]]; then
        # Pre-existing bag directory: do not record.
        if [[ ! -d "$INPUT_PATH" ]]; then
            echo "Error: '$INPUT_PATH' is neither a bag directory nor an .npz file" >&2
            exit 1
        fi
        BAG_PATH="$INPUT_PATH"
    else
        # Record a new bag using the ROS2 default datetime naming convention.
        BAG_NAME="rosbag2_$(date +%Y_%m_%d-%H_%M_%S)"
        BAG_PATH="$BAG_DIR/$BAG_NAME"
        ros2 bag record -a -o "$BAG_PATH"
    fi

    # Telemetry name always matches the bag name with an .npz extension.
    BAG_BASENAME="$(basename "$BAG_PATH")"
    TELEM_PATH="$TELEM_DIR/$BAG_BASENAME.npz"

    if [[ -n "$ROBOT" ]]; then
        python3 "$BAG2NP" --bag "$BAG_PATH" --robot "$ROBOT" --output "$TELEM_PATH"
    else
        python3 "$BAG2NP" --bag "$BAG_PATH" --output "$TELEM_PATH"
    fi
fi

cd "$ANALYSIS_DIR" && python3 telem_visualize.py --telemetry "$TELEM_PATH"
