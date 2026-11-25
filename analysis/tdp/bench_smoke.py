"""Bench smoke test: radio comms and data logging, robot on a stand (wheels off the ground)
on a bench power supply. Run before any field session (TDP_2027_EXPERIMENTS.md).

    python3 analysis/tdp/bench_smoke.py -r 2              # full test, ~1.5 min
    python3 analysis/tdp/bench_smoke.py -r 2 --no-motion  # comms + logging only
    python3 analysis/tdp/bench_smoke.py -r 2 --no-bag     # skip the bag recorder

Needs: radio bridge running (bringup_physical_core), extended telemetry DIP switch on,
the robot on a stand with all wheels clear, kicker irrelevant (never armed). Vision is
optional and better OFF or the robot out of camera view (see "state estimation" below).

Phases (each gives PASS / WARN / FAIL; exit code 1 on any FAIL):
  1 preflight     bridge node, telemetry flowing, radio connected
  2 bag           bag.sh started (zstd MCAP + parameter snapshots)
  3 link (idle)   telemetry rate and gaps, radio packet loss (basic telemetry sequence
                  numbers), error flags, supply voltage
  4 parameters    read all TDP firmware parameters; set the stand values (slip comp 0,
                  friction FF 0) and read them back
  5 motion        short BCM_LOCAL_VELOCITY segments (+x, -x, +y, -y, +w, -w, diagonal),
                  BCM_OFF between them. Per segment: wheel speed setpoints vs. the
                  kinematic model (frame, wheel order, sign), wheel tracking, currents,
                  command -> telemetry round trip, held maneuvers
  6 restore       compiled parameter values restored and verified (also on Ctrl-C/abort)
  7 logging       bag stopped and converted (bag2npz); same telemetry as the live log,
                  motion commands recorded, parameter snapshots complete

State estimation on a stand: the wheels spin but the robot doesn't move. The odometer's
heading comes from the gyro (which sees no rotation), and vision, if it sees the robot,
pins the position. The firmware's divergence check (1 m / 1.57 rad / 5 m/s / 10 rad/s
between trajectory and estimate) would then brake mid-segment. So segments are short
(linear 0.3 m/s x 1.5 s = 0.45 m, rotation 1.5 rad/s x 0.8 s = 1.2 rad), position and
line modes (which need vision) are never used, and BCM_OFF between segments resets the
trajectory. A segment whose setpoints collapse anyway is reported as "held" (WARN), not
as a wheel fault. Friction feed-forward and slip compensation are zeroed during the test:
on unloaded wheels they only add current the wheel loop has to fight.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from ateam_msgs.msg import RobotMotionCommand
from ateam_msgs.srv import GetFirmwareParameter, SetFirmwareParameter
from ateam_radio_msgs.msg import BasicTelemetry, ConnectionStatus, ErrorTelemetry, ExtendedTelemetry

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "calibration"))
import tdp_common as tc  # noqa: E402
from config import PARAMS, fmt  # noqa: E402
from field_record import COLUMNS, command_fields, telemetry_row  # noqa: E402

CONTROLS = HERE.parents[1]
COL = {c: i for i, c in enumerate(COLUMNS)}
VBUS = [COL[f"vbus{i}"] for i in range(4)]
BASIC_ERRORS = ("power_error", "power_board_error", "battery_error", "battery_low", "battery_crit",
                "shutdown_pending", "tipped_error", "accelerometer_0_error", "accelerometer_1_error",
                "gyroscope_0_error", "gyroscope_1_error", "motor_fl_general_error", "motor_fl_hall_error",
                "motor_bl_general_error", "motor_bl_hall_error", "motor_br_general_error",
                "motor_br_hall_error", "motor_fr_general_error", "motor_fr_hall_error")
MAX_WHEEL_RADS = 40.0
MAX_CURRENT_A = 4.2


class Bench(Node):
    def __init__(self, robot):
        super().__init__("tdp_bench_smoke")
        r = robot
        self.pub = self.create_publisher(RobotMotionCommand, f"/robot_motion_commands/robot{r}", 10)
        self.create_subscription(ExtendedTelemetry, f"/robot_feedback/extended/robot{r}", self.on_ext, 200)
        self.create_subscription(BasicTelemetry, f"/robot_feedback/basic/robot{r}", self.on_basic, 200)
        self.create_subscription(ConnectionStatus, f"/robot_feedback/connection/robot{r}", self.on_conn, 10)
        self.create_subscription(ErrorTelemetry, f"/robot_feedback/error/robot{r}", self.on_err, 50)
        self.robot = r
        self.get_cli = self.create_client(GetFirmwareParameter, "/get_firmware_param")
        self.set_cli = self.create_client(SetFirmwareParameter, "/set_firmware_param")
        self.param_latencies = []
        self.lock = threading.Lock()
        self.twist = None  # (vx, vy, w) local, or None for BCM_OFF
        self.cmd_fields = command_fields(self.make_cmd())
        self.rows, self.basic, self.errors = [], [], []
        self.connected = None
        self.tripped = None
        self.motion_active = False
        self.last_ext = None
        self.pending = None  # (mode, t_first_publish) awaiting the telemetry echo
        self.latencies = []
        self.create_timer(0.01, self.on_timer)

    def _call(self, cli, req, timeout=2.0, tries=3):
        """Service call through the bridge to the robot (direct rclpy: real round trip)."""
        for _ in range(tries):
            if not cli.wait_for_service(timeout_sec=2.0):
                return None
            t = time.monotonic()
            fut = cli.call_async(req)
            end = t + timeout
            while not fut.done() and time.monotonic() < end:
                time.sleep(0.002)
            if fut.done() and fut.result() is not None and fut.result().success:
                self.param_latencies.append(time.monotonic() - t)
                return fut.result()
        return None

    def get_param(self, pid):
        req = GetFirmwareParameter.Request(robot_id=self.robot, parameter_id=pid)
        res = self._call(self.get_cli, req)
        return [float(x) for x in res.data] if res is not None else None

    def set_param(self, pid, values):
        vals = [float(v) for v in (values.split(",") if isinstance(values, str) else values)]
        req = SetFirmwareParameter.Request(robot_id=self.robot, parameter_id=pid, data=vals)
        return self._call(self.set_cli, req) is not None

    def make_cmd(self):
        c = RobotMotionCommand()
        c.kick_request = RobotMotionCommand.KR_DISABLE
        if self.twist is None or self.tripped:
            c.body_control_mode = RobotMotionCommand.BCM_OFF
        else:
            c.body_control_mode = RobotMotionCommand.BCM_LOCAL_VELOCITY
            c.velocity.x, c.velocity.y, c.velocity.theta = map(float, self.twist)
            c.limit_vel_linear, c.limit_acc_linear = 1.0, 1.0
            c.limit_vel_angular, c.limit_acc_angular = 3.0, 6.0
        return c

    def set_twist(self, twist):
        with self.lock:
            self.twist = twist
            mode = RobotMotionCommand.BCM_OFF if twist is None else RobotMotionCommand.BCM_LOCAL_VELOCITY
            self.pending = (mode, None)

    def trip(self, reason):
        if self.tripped is None:
            self.tripped = reason
            self.get_logger().error(f"TRIP: {reason} -> BCM_OFF")

    def on_timer(self):
        now = time.monotonic()
        if self.motion_active and self.last_ext is not None and now - self.last_ext > 0.5:
            self.trip("telemetry stale > 0.5 s")
        with self.lock:
            c = self.make_cmd()
            self.cmd_fields = command_fields(c)
            if self.pending and self.pending[1] is None:
                self.pending = (self.pending[0], now)
        self.pub.publish(c)

    def on_ext(self, m):
        now = time.monotonic()
        self.last_ext = now
        with self.lock:
            row = telemetry_row(m, now, self.cmd_fields)
            self.rows.append(row)
            if self.pending and self.pending[1] is not None and int(row[COL["bcm"]]) == self.pending[0]:
                self.latencies.append(now - self.pending[1])
                self.pending = None
        if self.motion_active:
            wv = [row[COL[f"enc{i}"]] for i in range(4)]
            cur = [row[COL[f"i_sp{i}"]] for i in range(4)]
            if max(abs(w) for w in wv) > MAX_WHEEL_RADS:
                self.trip(f"wheel speed {np.round(wv, 1)} rad/s")
            elif max(abs(i) for i in cur) > MAX_CURRENT_A:
                self.trip(f"wheel current {np.round(cur, 2)} A")
            elif any(getattr(m, n).master_error for n in ("front_left_motor", "back_left_motor",
                                                        "back_right_motor", "front_right_motor")):
                self.trip("motor master_error")

    def on_basic(self, m):
        self.basic.append((time.monotonic(), m.transmission_sequence_number,
                           [k for k in BASIC_ERRORS if getattr(m, k)], m.battery_percent))

    def on_conn(self, m):
        self.connected = m.radio_connected

    def on_err(self, m):
        self.errors.append((time.monotonic(), bytes(m.error_message).decode(errors="replace")))


class Report:
    def __init__(self):
        self.items = []

    def add(self, phase, name, status, detail):
        self.items.append(dict(phase=phase, check=name, status=status, detail=detail))
        mark = {"PASS": "  ok ", "WARN": " WARN", "FAIL": " FAIL", "INFO": " info"}[status]
        print(f"[{mark}] {phase:10s} {name:30s} {detail}", flush=True)

    def failed(self):
        return any(i["status"] == "FAIL" for i in self.items)


def window(node, t0, t1):
    with node.lock:
        rows = [r for r in node.rows if t0 <= r[0] <= t1]
    return np.array(rows) if rows else np.zeros((0, len(COLUMNS)))


def link_stats(rep, node, t0, t1, phase):
    a = window(node, t0, t1)
    if len(a) < 10:
        rep.add(phase, "extended telemetry", "FAIL", f"only {len(a)} messages in {t1 - t0:.1f} s")
        return a
    tr = a[:, COL["t_robot"]]
    dt = np.diff(tr)
    rate = (len(tr) - 1) / max(tr[-1] - tr[0], 1e-6)
    gaps = dt[dt > 3 * np.median(dt)]
    status = "PASS" if rate >= 80 else "WARN" if rate >= 50 else "FAIL"
    rep.add(phase, "extended telemetry rate", status,
            f"{rate:.0f} Hz (robot clock), median dt {np.median(dt) * 1e3:.1f} ms")
    mx = dt.max() * 1e3
    rep.add(phase, "telemetry gaps", "PASS" if mx < 200 else "WARN" if mx < 1000 else "FAIL",
            f"max {mx:.0f} ms, {len(gaps)} gaps > 3x median (radio stalls ~0.5 s every 1-2 min are known)")
    if (np.diff(tr) <= 0).any():
        rep.add(phase, "robot clock", "FAIL", "t_robot not strictly increasing")
    b = [x for x in node.basic if t0 <= x[0] <= t1]
    if len(b) > 5:
        seq = np.array([x[1] for x in b], dtype=int)
        steps = np.mod(np.diff(seq), 256)
        lost = int(np.sum(steps[steps > 0] - 1))
        loss = lost / max(int(np.sum(steps)), 1)
        rep.add(phase, "radio packet loss (basic)", "PASS" if loss < 0.02 else "WARN" if loss < 0.1 else "FAIL",
                f"{loss:.1%} ({lost} of {int(np.sum(steps))} sequence steps missing, {len(b)} received)")
        errs = sorted({e for x in b for e in x[2]})
        rep.add(phase, "error flags (basic)", "FAIL" if errs else "PASS", ", ".join(errs) if errs else "none")
    else:
        rep.add(phase, "basic telemetry", "WARN", f"only {len(b)} messages")
    vb = a[:, VBUS].mean(axis=1)
    rep.add(phase, "supply voltage (motors)", "PASS" if vb.min() > 0 else "WARN",
            f"mean {vb.mean():.2f} V, min {vb.min():.2f} V")
    return a


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-r", "--robot", type=int, default=2)
    ap.add_argument("--no-motion", action="store_true")
    ap.add_argument("--no-bag", action="store_true")
    ap.add_argument("--speed", type=float, default=0.3, help="linear segment speed (m/s)")
    ap.add_argument("--wspeed", type=float, default=1.5, help="rotation segment speed (rad/s)")
    ap.add_argument("--out", default=None, help="output dir (default analysis/data/tdp/bench/<date>_r<robot>)")
    args = ap.parse_args()
    out = Path(args.out or CONTROLS / f"analysis/data/tdp/bench/{time.strftime('%Y%m%d_%H%M%S')}_r{args.robot}")
    out.mkdir(parents=True, exist_ok=True)
    rep = Report()
    print(f"bench smoke test, robot {args.robot}, output {out}\n")

    rclpy.init()
    node = Bench(args.robot)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    bag, bag_log, bag_dir = None, out / "bag.log", None
    changed_params = False
    try:
        # 1 preflight
        nodes = subprocess.run(["ros2", "node", "list", "--no-daemon", "--spin-time", "2"],
                               capture_output=True, text=True, timeout=30).stdout.split()
        rep.add("preflight", "radio bridge node", "PASS" if "/radio_bridge" in nodes else "FAIL",
                "running" if "/radio_bridge" in nodes else "not found: ros2 launch ateam_bringup bringup_physical_core.launch.py")
        end = time.monotonic() + 8
        while not node.rows and time.monotonic() < end:
            time.sleep(0.1)
        if not node.rows:
            rep.add("preflight", "extended telemetry", "FAIL",
                    f"nothing on /robot_feedback/extended/robot{args.robot} (robot ID? extended telemetry DIP switch?)")
            return 1
        rep.add("preflight", "extended telemetry", "PASS", "flowing")
        rep.add("preflight", "radio connected", "PASS" if node.connected else "WARN",
                f"connection status: {node.connected}")
        vis = window(node, 0, time.monotonic())[:, COL["vis_upd"]].sum()
        rep.add("preflight", "vision", "INFO" if vis == 0 else "WARN",
                "no vision updates (good on a stand)" if vis == 0 else
                f"{int(vis)} vision updates: vision sees the robot; linear segments may trip divergence (WARN 'held')")

        # 2 bag
        if not args.no_bag:
            env = dict(os.environ, TDP_BAG_DIR=str(out))
            bag = subprocess.Popen(["bash", "analysis/tdp/bag.sh", "bench_smoke", str(args.robot)], cwd=CONTROLS,
                                   env=env, stdout=open(bag_log, "w"), stderr=subprocess.STDOUT, start_new_session=True)
            end = time.monotonic() + 120
            while "recording to" not in bag_log.read_text() and bag.poll() is None and time.monotonic() < end:
                time.sleep(0.5)
            started = "recording to" in bag_log.read_text()
            rep.add("bag", "bag.sh recording", "PASS" if started else "FAIL",
                    bag_log.read_text().split("recording to ")[-1].split()[0] if started else f"see {bag_log}")
            if started:
                bag_dir = Path(bag_log.read_text().split("recording to ")[-1].split()[0])
                time.sleep(2.0)  # let the recorder discover the topics

        # 3 link, idle
        t0 = time.monotonic()
        time.sleep(4.0)
        idle = link_stats(rep, node, t0, time.monotonic(), "link idle")
        if node.errors:
            rep.add("link idle", "error telemetry", "WARN", f"{len(node.errors)} messages, last: {node.errors[-1][1][:80]}")

        # 4 parameters
        t = time.monotonic()
        cur = {pid: node.get_param(pid) for pid in PARAMS}
        missing = [PARAMS[p][0] for p, v in cur.items() if v is None]
        rep.add("params", "read all", "FAIL" if missing else "PASS",
                f"missing {missing}" if missing else f"{len(cur)} parameters in {time.monotonic() - t:.1f} s")
        differs = [PARAMS[p][0] for p, v in cur.items() if v and any(abs(a - b) > 1e-4 for a, b in zip(v, PARAMS[p][1]))]
        rep.add("params", "match compiled values", "WARN" if differs else "PASS",
                f"differ: {differs} (set live earlier? reboot clears)" if differs else "all")
        d = PARAMS[6][1]
        stand = {19: "0", 6: fmt([0, 0, 0, 0, d[4], d[5]])}
        changed_params = True
        ok = all(node.set_param(pid, v) for pid, v in stand.items())
        back = {pid: node.get_param(pid) for pid in stand}
        good = ok and all(back[p] and all(abs(a - float(b)) < 1e-4 for a, b in zip(back[p], v.split(",")))
                          for p, v in stand.items())
        rep.add("params", "set + read back", "PASS" if good else "FAIL",
                "WHEEL_SLIP_COMP 0, friction FF 0 (stand values)" if good else f"wrote {stand}, read {back}")
        lat = np.array(node.param_latencies) * 1e3
        if len(lat):
            rep.add("params", "service round trip", "PASS" if np.max(lat) < 300 else "WARN",
                    f"median {np.median(lat):.0f} ms, max {lat.max():.0f} ms over {len(lat)} calls (bridge -> robot -> bridge)")

        # 5 motion
        if not args.no_motion:
            v, w = args.speed, args.wspeed
            segs = [("+x", (v, 0, 0), 1.5), ("-x", (-v, 0, 0), 1.5), ("+y", (0, v, 0), 1.5), ("-y", (0, -v, 0), 1.5),
                    ("+w", (0, 0, w), 0.8), ("-w", (0, 0, -w), 0.8),
                    ("diag", (v / math.sqrt(2), v / math.sqrt(2), 0), 1.5), ("-diag", (-v / math.sqrt(2), -v / math.sqrt(2), 0), 1.5)]
            input_ok = os.environ.get("TDP_BENCH_NO_PROMPT") or input(
                "\nWheels clear of everything and the robot secure on its stand? [y/N] ").strip().lower() == "y"
            if not input_ok:
                rep.add("motion", "skipped", "WARN", "not confirmed")
            else:
                node.motion_active = True
                seg_times = []
                for name, twist, dur in segs:
                    if node.tripped:
                        break
                    node.set_twist(twist)
                    ts = time.monotonic()
                    time.sleep(dur)
                    node.set_twist(None)
                    seg_times.append((name, twist, ts, time.monotonic()))
                    time.sleep(1.0)
                node.motion_active = False
                if node.tripped:
                    rep.add("motion", "safety", "FAIL", node.tripped)
                for name, twist, ts, te in seg_times:
                    a = window(node, ts + 0.5, te)  # after the 1 m/s^2 ramp
                    if len(a) < 5:
                        rep.add("motion", f"segment {name}", "FAIL", "no telemetry")
                        continue
                    exp = tc.VEL2WHEEL @ np.array(twist, float)
                    sp = a[:, [COL[f"vel_sp{i}"] for i in range(4)]].mean(axis=0)
                    enc = a[:, [COL[f"enc{i}"] for i in range(4)]].mean(axis=0)
                    imax = np.abs(a[:, [COL[f"i_sp{i}"] for i in range(4)]]).max()
                    tlim = a[:, [COL[f"tlim{i}"] for i in range(4)]].any()
                    big = np.abs(exp) > 2.0
                    if np.all(np.abs(sp) < 0.1 * np.abs(exp).max()):
                        rep.add("motion", f"segment {name}", "WARN",
                                "held: setpoints ~0 (divergence recovery or halt; see the docstring), wheels not checked")
                        continue
                    sp_err = np.abs(sp - exp)
                    sp_ok = np.all(sp_err <= np.maximum(1.0, 0.15 * np.abs(exp)))
                    sign_ok = np.all(np.sign(enc[big]) == np.sign(exp[big]))
                    trk = np.abs(enc - sp)
                    trk_ok = np.all(trk <= np.maximum(1.5, 0.2 * np.abs(sp)))
                    status = "FAIL" if not (sp_ok and sign_ok) else "PASS" if trk_ok and not tlim else "WARN"
                    rep.add("motion", f"segment {name}", status,
                            f"setpoint vs model {np.round(sp, 1)} / {np.round(exp, 1)} rad/s"
                            f"{'' if sp_ok else ' MISMATCH'}; wheels {np.round(enc, 1)}"
                            f"{'' if sign_ok else ' WRONG DIRECTION'}; I max {imax:.2f} A"
                            f"{' torque_limited' if tlim else ''}")
                lat = np.array(node.latencies) * 1e3
                if len(lat):
                    rep.add("motion", "command -> telemetry", "PASS" if np.median(lat) < 60 else "WARN",
                            f"round trip median {np.median(lat):.0f} ms, max {lat.max():.0f} ms ({len(lat)} mode changes)")
                if seg_times:
                    moving = link_stats(rep, node, seg_times[0][2], seg_times[-1][3], "link motion")
                    if len(idle) and len(moving):
                        sag = idle[:, VBUS].mean() - moving[:, VBUS].mean(axis=1).min()
                        rep.add("motion", "supply sag", "PASS" if sag < 1.0 else "WARN",
                                f"{sag:.2f} V below idle at worst (bench supply current limit?)")
    except KeyboardInterrupt:
        rep.add("abort", "Ctrl-C", "WARN", "stopped by user")
    finally:
        node.set_twist(None)
        node.motion_active = False
        for _ in range(30):
            node.pub.publish(node.make_cmd())
            time.sleep(0.01)
        # 6 restore
        if changed_params:
            ok = all(node.set_param(pid, d) for pid, (_, d) in PARAMS.items())
            back = {pid: node.get_param(pid) for pid in PARAMS}
            good = ok and all(back[p] and all(abs(a - b) < 1e-4 for a, b in zip(back[p], d))
                              for p, (_, d) in PARAMS.items())
            rep.add("restore", "compiled values", "PASS" if good else "FAIL",
                    "restored and verified" if good else "NOT restored: reboot the robot")
        # live log
        with node.lock:
            live = np.array(node.rows, dtype=float)
        np.savez(out / "bench_live.npz", data=live, columns=np.array(COLUMNS))
        # 7 bag
        if bag is not None and bag.poll() is None:
            os.killpg(bag.pid, signal.SIGINT)  # what Ctrl-C in its terminal does
            try:
                bag.wait(timeout=180)
            except subprocess.TimeoutExpired:
                os.killpg(bag.pid, signal.SIGTERM)
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()

    if bag_dir is not None:
        verify_bag(rep, bag_dir, args.robot, live, bag_log)
    (out / "report.json").write_text(json.dumps(dict(robot=args.robot, time=time.strftime("%Y-%m-%d %H:%M:%S"),
                                                     checks=rep.items), indent=1))
    n_fail = sum(i["status"] == "FAIL" for i in rep.items)
    n_warn = sum(i["status"] == "WARN" for i in rep.items)
    print(f"\n{'FAILED' if n_fail else 'PASSED'}: {n_fail} fail, {n_warn} warn. Report and logs in {out}")
    return 1 if n_fail else 0


def verify_bag(rep, bag_dir, robot, live, bag_log):
    from bag2npz import convert
    if not (bag_dir / "metadata.yaml").exists():
        rep.add("logging", "bag closed", "FAIL", f"no metadata.yaml in {bag_dir} (see {bag_log})")
        return
    rep.add("logging", "bag closed", "PASS", f"{bag_dir.name}, {sum(f.stat().st_size for f in bag_dir.iterdir()) / 1e6:.1f} MB")
    b = convert(bag_dir, robot)
    if len(b) == 0 or len(live) == 0:
        rep.add("logging", "bag telemetry", "FAIL", f"bag rows {len(b)}, live rows {len(live)}")
        return
    tr_b = set(np.round(b[:, COL["t_robot"]], 6))
    tr_l = np.round(live[:, COL["t_robot"]], 6)
    inside = (tr_l >= min(tr_b)) & (tr_l <= max(tr_b))
    covered = np.mean([t in tr_b for t in tr_l[inside]]) if inside.any() else 0.0
    rep.add("logging", "bag vs live telemetry", "PASS" if covered > 0.99 else "WARN" if covered > 0.9 else "FAIL",
            f"{covered:.1%} of the live messages (while the bag ran) are in the bag; bag {len(b)} rows, live {len(live)}")
    # the bag starts after its parameter snapshot, but must run to the end of the test
    tail = tr_l.max() - max(tr_b)
    rep.add("logging", "bag covers the test", "PASS" if tail < 0.5 else "FAIL",
            "runs to the last live message" if tail < 0.5 else f"stopped {tail:.1f} s before the test ended")
    modes = set(np.unique(b[:, COL["cmd_mode"]][~np.isnan(b[:, COL["cmd_mode"]])]).astype(int))
    rep.add("logging", "commands recorded", "PASS" if 12 in modes else "FAIL",
            f"command modes in bag: {sorted(modes)} (12 = local velocity)")
    for f in ("params_start.json", "params_end.json"):
        p = bag_dir / f
        if not p.exists():
            rep.add("logging", f, "FAIL", "missing")
            continue
        snap = json.loads(p.read_text())
        vals = snap["robots"].get(str(robot), {})
        none = [k for k, v in vals.items() if v is None]
        rep.add("logging", f, "FAIL" if none or not vals else "PASS",
                f"firmware params missing: {none}" if none else f"{len(vals)} firmware params")
    end = json.loads((bag_dir / "params_end.json").read_text()) if (bag_dir / "params_end.json").exists() else None
    if end:
        diff = [n for n, d in PARAMS.values() if end["robots"].get(str(robot), {}).get(n)
                and any(abs(a - b) > 1e-4 for a, b in zip(end["robots"][str(robot)][n], d))]
        rep.add("logging", "end snapshot = compiled", "FAIL" if diff else "PASS", f"differ: {diff}" if diff else "yes")


if __name__ == "__main__":
    sys.exit(main())
