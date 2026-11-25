"""Convert a recorded bag (bag.sh) into the TDP telemetry npz format, for any robot.

Produces exactly what field_record.py writes (same COLUMNS, same row code), so every
fig_*.py script and calibration/common.load() reads it. t_host is the bag's receive
time in seconds. Everything is float64. (Nick's telem_bag2np.py casts to float32, which
quantizes the microsecond robot clock to ~1 ms after a couple of hours of uptime.)

    python3 analysis/tdp/bag2npz.py analysis/data/tdp/bags/f1_star_shipped_r2_20261201_1405 -r 2 \
        --out analysis/data/tdp/field_star_shipped_r2.npz
    python3 analysis/tdp/bag2npz.py <bag> --list          # robots and message counts
    python3 analysis/tdp/bag2npz.py <bag> -r 2 --start 14:07:30 --end 14:10:00 --out part.npz

--start/--end take wall-clock HH:MM[:SS] on the bag's first day, seconds from bag start
(e.g. 90), or unix time (sweep.py's t_start/t_end), to cut one experiment out of a long
session bag.
"""

from __future__ import annotations

import argparse
import datetime as dt
import re
import sys
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, str(Path(__file__).resolve().parent))
from field_record import COLUMNS, NO_CMD, command_fields, telemetry_row  # noqa: E402

TELEM = "/robot_feedback/extended/robot{}"
CMD = "/robot_motion_commands/robot{}"


def open_reader(bag, topics=None):
    r = SequentialReader()
    r.open(StorageOptions(uri=str(bag), storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    if topics:
        r.set_filter(StorageFilter(topics=[t for t in topics if t in types]))
    return r, types


def list_bag(bag):
    r, _ = open_reader(bag)
    counts, first, last = {}, None, None
    while r.has_next():
        topic, _, t = r.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        first = t if first is None else first
        last = t
    if first is None:
        raise SystemExit("empty bag")
    print(f"{dt.datetime.fromtimestamp(first * 1e-9):%Y-%m-%d %H:%M:%S} .. "
          f"{dt.datetime.fromtimestamp(last * 1e-9):%H:%M:%S} ({(last - first) * 1e-9:.0f} s)")
    for topic in sorted(counts):
        print(f"  {counts[topic]:8d}  {topic}")
    robots = sorted(int(m.group(1)) for t in counts if (m := re.match(r"^/robot_feedback/extended/robot(\d+)$", t)))
    print(f"robots with extended telemetry: {robots}")
    return first


def parse_time(s, bag_start_ns):
    if s is None:
        return None
    if re.fullmatch(r"[\d.]+", s):
        v = float(s)  # > 1e9: unix time (sweep.py's t_start/t_end); else seconds from bag start
        return int(v * 1e9) if v > 1e9 else bag_start_ns + int(v * 1e9)
    day = dt.datetime.fromtimestamp(bag_start_ns * 1e-9)
    hms = [int(x) for x in s.split(":")] + [0]
    t = day.replace(hour=hms[0], minute=hms[1], second=hms[2], microsecond=0)
    return int(t.timestamp() * 1e9)


def convert(bag, robot, start=None, end=None):
    telem_topic, cmd_topic = TELEM.format(robot), CMD.format(robot)
    r, types = open_reader(bag, [telem_topic, cmd_topic])
    if telem_topic not in types:
        raise SystemExit(f"{telem_topic} not in the bag (use --list)")
    cmd, rows = NO_CMD, []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == cmd_topic:  # keep the latest command even before the window
            cmd = command_fields(deserialize_message(data, types[topic]))
            continue
        if (start is not None and t < start) or (end is not None and t > end):
            continue
        rows.append(telemetry_row(deserialize_message(data, types[topic]), t * 1e-9, cmd))
    return np.array(rows, dtype=np.float64)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="bag directory")
    ap.add_argument("-r", "--robot", type=int)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--start")
    ap.add_argument("--end")
    ap.add_argument("--out")
    args = ap.parse_args()
    first = list_bag(args.bag) if args.list or args.robot is None else None
    if args.list or args.robot is None:
        if not args.list:
            print("pass -r ROBOT to convert")
        return
    if args.start or args.end:
        r, _ = open_reader(args.bag)
        first = r.read_next()[2]
    data = convert(args.bag, args.robot, parse_time(args.start, first), parse_time(args.end, first))
    if not args.out:
        raise SystemExit(f"{len(data)} rows; pass --out to save")
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    np.savez(args.out, data=data, columns=np.array(COLUMNS), source_bag=str(Path(args.bag).resolve()))
    print(f"saved {len(data)} rows ({len(COLUMNS)} columns) to {args.out}")
    if len(data) and np.isnan(data[:, -1]).all():
        print(f"note: no motion commands for robot {args.robot} in that window")


if __name__ == "__main__":
    main()
