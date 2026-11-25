"""Robot / stack configuration for the TDP experiments: radio parameters, the bridge's
wheel velocity loop, and the kenobi field-test play.

Radio parameters are live until the robot reboots. Run in a ROS-sourced shell.

    python3 analysis/tdp/config.py show -r 2                      # current vs compiled values
    python3 analysis/tdp/config.py set -r 2 WHEEL_SLIP_COMP 0.8   # by name or numeric id
    python3 analysis/tdp/config.py friction-off -r 2              # E3: zero friction FF (keep efficiency)
    python3 analysis/tdp/config.py defaults -r 2                  # restore every compiled value
    python3 analysis/tdp/config.py wheel-loop on|off              # bridge: controls_enabled.wheel_vel
    python3 analysis/tdp/config.py play TdpMotionPlay             # kenobi play override ("" clears it)
    python3 analysis/tdp/config.py play-params pattern=star accel=3 star_directions=12
    python3 analysis/tdp/config.py play-params --show
    python3 analysis/tdp/config.py snapshot -r 2 3 --out params.json   # everything, as JSON (bag.sh does this)
    eval "$(python3 analysis/tdp/config.py area patch)"   # test area: TDP_AREA/TDP_HOME/TDP_HEADING + play
    python3 analysis/tdp/config.py area                   # list areas (areas.json)
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import time
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "calibration"))
from common import get_param, set_param  # noqa: E402

# id: (name, compiled default) -- controller_params.rs / ateam-controls defaults.rs
PARAMS = {
    6: ("PHYS_FRICTION_MODEL", [4.5, 0.0, 0.1, 0.0, 0.75, 0.8]),
    15: ("POSE_FB_XY", [8.0, 0.3, 0.01]),
    16: ("POSE_FB_W", [8.0, 2.0, 0.01, 2.0]),
    17: ("WHEEL_VEL_FB", [400.0, 0.0, 1.5, 4.0]),
    18: ("WHEEL_FF_CURRENT_LIMIT", [3.5]),
    19: ("WHEEL_SLIP_COMP", [1.2]),
}
BY_NAME = {name: pid for pid, (name, _) in PARAMS.items()}

KENOBI = "/kenobi_node"
PLAY_NS = "stp_parameters.TdpMotionPlay"
# kenobi declares these with fixed types; ros2 param set rejects "3" for a double
PLAY_PARAM_TYPES = {
    "pattern": str, "robot_id": int, "accel": float, "vel": float, "ang_accel": float,
    "ang_vel": float, "heading_deg": float, "star_directions": int, "star_radius_m": float,
    "cycles": int, "dwell_s": float, "margin_m": float, "arrive_tol_m": float,
    "arrive_timeout_s": float, "abort_speed_mps": float, "fallback_field_length_m": float,
    "fallback_field_width_m": float, "fallback_defense_depth_m": float,
    "area": str, "area_center_x_m": float, "area_center_y_m": float, "area_length_m": float,
    "area_width_m": float, "area_yaw_deg": float, "avoid_defense_areas": bool,
}
AREAS = HERE / "areas.json"


def fmt(values):
    return ",".join(f"{v:g}" for v in values)


def resolve(name_or_id):
    if name_or_id.isdigit():
        return int(name_or_id)
    if name_or_id.upper() in BY_NAME:
        return BY_NAME[name_or_id.upper()]
    raise SystemExit(f"unknown parameter {name_or_id}; known: {', '.join(BY_NAME)}")


def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    out = (r.stdout + r.stderr).strip()
    if r.returncode != 0:
        raise SystemExit(f"failed: {' '.join(cmd)}\n{out}")
    return out


def cmd_show(args):
    for pid, (name, default) in PARAMS.items():
        cur = get_param(args.robot, pid)
        flag = "" if cur is not None and len(cur) == len(default) and all(
            abs(a - b) < 1e-4 for a, b in zip(cur, default)) else "   <-- differs from compiled"
        print(f"{pid:3d} {name:24s} {fmt(cur) if cur else 'n/a':28s} (compiled {fmt(default)}){flag}")
    try:
        print(f"bridge controls_enabled.wheel_vel: {run(['ros2', 'param', 'get', '/radio_bridge', 'controls_enabled.wheel_vel'])}")
    except SystemExit as e:
        print(f"bridge controls_enabled.wheel_vel: unknown ({e})")


def cmd_set(args):
    pid = resolve(args.param)
    ok = set_param(args.robot, pid, args.values)
    print(f"{PARAMS.get(pid, (str(pid),))[0]} = {args.values}: {'ok' if ok else 'FAILED'}")
    if not ok:
        raise SystemExit(1)


def cmd_defaults(args):
    bad = [name for pid, (name, d) in PARAMS.items() if not set_param(args.robot, pid, fmt(d))]
    print("restored compiled values" + (f"; FAILED: {bad} (reboot the robot)" if bad else ""))
    if bad:
        raise SystemExit(1)


def cmd_friction_off(args):
    d = PARAMS[6][1]
    vals = fmt([0, 0, 0, 0, d[4], d[5]])
    ok = set_param(args.robot, 6, vals)
    print(f"PHYS_FRICTION_MODEL = {vals}: {'ok' if ok else 'FAILED'}  (restore: config.py defaults -r {args.robot})")


def cmd_wheel_loop(args):
    val = "true" if args.state == "on" else "false"
    print(run(["ros2", "param", "set", "/radio_bridge", "controls_enabled.wheel_vel", val]))


def cmd_play(args):
    print(run(["ros2", "service", "call", f"{KENOBI}/set_override_play", "ateam_msgs/srv/SetOverridePlay",
               f"{{play_name: '{args.name}'}}"]).splitlines()[-1])
    if args.name:
        print("the play reads its parameters when selected: change them first, or re-select it")


def cmd_play_params(args):
    if args.show or not args.assignments:
        for key in PLAY_PARAM_TYPES:
            try:
                print(f"{key:26s} {run(['ros2', 'param', 'get', KENOBI, f'{PLAY_NS}.{key}'])}")
            except SystemExit:
                print(f"{key:26s} (not available: is kenobi running this branch?)")
        return
    for a in args.assignments:
        key, _, val = a.partition("=")
        if key not in PLAY_PARAM_TYPES:
            raise SystemExit(f"unknown play parameter {key}; known: {', '.join(PLAY_PARAM_TYPES)}")
        typ = PLAY_PARAM_TYPES[key]
        if typ is bool:
            val = "true" if val.lower() in ("1", "true", "yes", "on") else "false"
        else:
            val = repr(float(val)) if typ is float else str(int(float(val))) if typ is int else val
        print(f"{key} = {val}: {run(['ros2', 'param', 'set', KENOBI, f'{PLAY_NS}.{key}', val])}")
    print("re-select the play (config.py play TdpMotionPlay) for the new values to take effect")


def try_run(cmd):
    try:
        return run(cmd).splitlines()[-1]
    except (SystemExit, subprocess.TimeoutExpired, FileNotFoundError) as e:
        return None


def dump_params(node):
    """All ROS parameters of a node in one call (ros2 param dump prints YAML)."""
    import yaml
    out = run(["ros2", "param", "dump", node])
    return next(iter(yaml.safe_load(out).values()))["ros__parameters"]


def cmd_snapshot(args):
    """Firmware radio params are not ROS params, so /parameter_events in a bag never sees
    them: record them (and the stack's settings) explicitly."""
    snap = {"time": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "unix_time": time.time(), "robots": {}}
    try:
        # --no-daemon: a freshly started ROS 2 daemon can still be discovering and miss nodes
        nodes = run(["ros2", "node", "list", "--no-daemon", "--spin-time", "2"]).split()
    except (SystemExit, subprocess.TimeoutExpired):
        nodes = []
    bridge_up = "/radio_bridge" in nodes  # get_param retries ~10 s per call: skip without it
    for robot in args.robots:
        snap["robots"][str(robot)] = {name: (get_param(robot, pid) if bridge_up else None)
                                      for pid, (name, _) in PARAMS.items()}
    snap["compiled_defaults"] = {name: d for name, d in PARAMS.values()}
    snap["radio_bridge_params"] = dump_params("/radio_bridge") if bridge_up else None
    snap["kenobi_play_params"] = None
    if KENOBI in nodes:
        stp = dump_params(KENOBI).get("stp_parameters", {})
        snap["kenobi_play_params"] = stp.get("TdpMotionPlay")
    snap["nodes"] = nodes
    if not bridge_up:
        snap["note"] = "radio bridge not running: firmware parameters not read"
    text = json.dumps(snap, indent=1)
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(text)
        missing = [r for r, ps in snap["robots"].items() if any(v is None for v in ps.values())]
        print(f"wrote {args.out}" + (f" (robots {missing} did not answer)" if missing else ""))
    else:
        print(text)


def cmd_area(args):
    """Select a test area: print shell exports (stdout) and set the play's area (if kenobi runs)."""
    areas = {k: v for k, v in json.loads(AREAS.read_text()).items() if not k.startswith("_")}
    if not args.name:
        for name, a in areas.items():
            dims = "TODO" if a["length_m"] is None else f"{a['length_m']} x {a['width_m']} m at ({a['center_x_m']}, {a['center_y_m']})"
            print(f"{name:8s} {dims:34s} {a.get('note', '')}")
        return
    if args.name not in areas:
        raise SystemExit(f"unknown area {args.name}; known: {', '.join(areas)}")
    a = areas[args.name]
    keys = ("center_x_m", "center_y_m", "length_m", "width_m", "yaw_deg", "heading_deg")
    if any(a.get(k) is None for k in keys):
        raise SystemExit(f"area '{args.name}' is incomplete in {AREAS}: {a.get('note', '')}")
    cx, cy, length, width, yaw, heading = (float(a[k]) for k in keys)
    hdg = math.radians(heading)
    print(f"export TDP_AREA={cx:g},{cy:g},{length:g},{width:g},{yaw:g}")
    print(f"export TDP_HOME={cx:g},{cy:g},{hdg:.4f}")
    print(f"export TDP_HEADING={hdg:.4f}")
    if args.max_speed:
        print(f"export TDP_MAX_SPEED={args.max_speed:g}")
    log = lambda m: print(m, file=sys.stderr)  # noqa: E731
    log(f"area {args.name}: {length:g} x {width:g} m at ({cx:g}, {cy:g}), yaw {yaw:g} deg; home heading {heading:g} deg")
    if args.no_play:
        return
    play = {"area": "field"} if args.name == "field" else {
        "area": "custom", "area_center_x_m": cx, "area_center_y_m": cy, "area_length_m": length,
        "area_width_m": width, "area_yaw_deg": yaw}
    try:
        for k, v in play.items():
            run(["ros2", "param", "set", KENOBI, f"{PLAY_NS}.{k}", v if isinstance(v, str) else repr(float(v))])
        log("kenobi play area set (re-select the play to apply)")
    except SystemExit as e:
        log(f"kenobi play area not set (is kenobi running?): {str(e).splitlines()[0]}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("show", "defaults", "friction-off"):
        p = sub.add_parser(name)
        p.add_argument("-r", "--robot", type=int, default=2)
    p = sub.add_parser("set")
    p.add_argument("-r", "--robot", type=int, default=2)
    p.add_argument("param", help="name (e.g. WHEEL_SLIP_COMP) or id")
    p.add_argument("values", help="comma separated, e.g. 400,0,1.5,4.0")
    p = sub.add_parser("wheel-loop")
    p.add_argument("state", choices=("on", "off"))
    p = sub.add_parser("play")
    p.add_argument("name", nargs="?", default="", help="play name; omit to clear the override")
    p = sub.add_parser("area")
    p.add_argument("name", nargs="?", help="area from areas.json (omit to list)")
    p.add_argument("--max-speed", type=float, help="also export TDP_MAX_SPEED (m/s) for the test monitor")
    p.add_argument("--no-play", action="store_true", help="only print the exports")
    p = sub.add_parser("snapshot")
    p.add_argument("-r", "--robots", type=int, nargs="+", default=[2])
    p.add_argument("--out")
    p = sub.add_parser("play-params")
    p.add_argument("assignments", nargs="*", help="key=value")
    p.add_argument("--show", action="store_true")
    args = ap.parse_args()
    {"show": cmd_show, "set": cmd_set, "defaults": cmd_defaults, "friction-off": cmd_friction_off,
     "wheel-loop": cmd_wheel_loop, "play": cmd_play, "play-params": cmd_play_params,
     "snapshot": cmd_snapshot, "area": cmd_area}[args.cmd](args)


if __name__ == "__main__":
    main()
