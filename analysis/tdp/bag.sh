#!/usr/bin/env bash
# Record a TDP experiment to a bag (zstd MCAP), with parameter snapshots before and after.
#
#   analysis/tdp/bag.sh NAME [ROBOT ...]      e.g.  analysis/tdp/bag.sh f1_star_shipped 2
#
# Run from controls/ in a ROS-sourced shell, in its own terminal; Ctrl-C stops it cleanly.
# Writes analysis/data/tdp/bags/NAME_<date>_<time>/ containing the .mcap, metadata.yaml,
# params_start.json and params_end.json (config.py snapshot for the given robots, default 2).
#
# Recorded: robot feedback (all robots), motion commands, raw vision and referee, the
# vision filter's robots/ball/field, kenobi's playbook state and play info, all ROS
# parameter changes, and /rosout. Not recorded: UI overlays and kenobi's /world.
# Convert for the figure scripts with analysis/tdp/bag2npz.py.
#
# TDP_BAG_DIR overrides the output directory (e.g. a bigger disk).
#
# Size: robot telemetry is ~90 KB/s per robot before compression; raw vision depends on
# the camera count. Check with `du -sh` after the first session.

set -euo pipefail
cd "$(dirname "$0")/../.."   # controls/

NAME="${1:?usage: analysis/tdp/bag.sh NAME [ROBOT ...]}"
shift
ROBOTS=("$@")
[[ ${#ROBOTS[@]} -eq 0 ]] && ROBOTS=(2)

command -v ros2 >/dev/null || { echo "source /opt/ros/jazzy/setup.bash and the workspace first" >&2; exit 1; }

OUT="${TDP_BAG_DIR:-analysis/data/tdp/bags}/${NAME}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$(dirname "$OUT")"
TOPICS='^(/robot_feedback/.*|/robot_motion_commands/.*|/vision_messages|/referee_messages|/yellow_team/.*|/blue_team/.*|/ball|/field|/kenobi_node/playbook_state|/kenobi_node/status|/play_info|/parameter_events|/rosout|/joy)$'

# Kenobi publishes motion commands with SystemDefaultsQoS, which Fast DDS makes
# TRANSIENT_LOCAL; the recorder would then match that and silently drop commands from
# VOLATILE publishers (small_move_test.py and the other test scripts). A VOLATILE
# subscription accepts both. rosbag2 overrides are per topic (no wildcards).
QOS="${OUT}.qos.yaml"
for i in $(seq 0 15); do
    printf '/robot_motion_commands/robot%d:\n  history: keep_last\n  depth: 100\n  reliability: reliable\n  durability: volatile\n' "$i"
done > "$QOS"

echo "snapshot (start)..."
python3 analysis/tdp/config.py snapshot -r "${ROBOTS[@]}" --out "${OUT}.params_start.json" || true

# Ctrl-C goes to the whole foreground process group: the recorder closes the bag on it.
# Catch it here (no-op) so this script survives to take the end snapshot. Don't use
# `trap '' INT`: an ignored signal stays ignored in the child, and the recorder would
# never stop.
trap ':' INT
echo "recording to $OUT  (Ctrl-C to stop)"
set +e
ros2 bag record -s mcap --storage-preset-profile zstd_fast --qos-profile-overrides-path "$QOS" \
    -o "$OUT" -e "$TOPICS"
status=$?
set -e
trap - INT

echo "snapshot (end)..."
python3 analysis/tdp/config.py snapshot -r "${ROBOTS[@]}" --out "${OUT}.params_end.json" || true
for f in params_start.json params_end.json qos.yaml; do
    [[ -f "${OUT}.${f}" && -d "$OUT" ]] && mv "${OUT}.${f}" "$OUT/${f}"
done
if [[ -d "$OUT" ]]; then
    ros2 bag info "$OUT" 2>/dev/null | grep -E 'Duration|Messages|Bag size' || true
    du -sh "$OUT"
else
    echo "no bag written (recorder exit $status)" >&2
    exit 1
fi
