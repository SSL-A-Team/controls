"""Shared rclpy node, telemetry buffer, and param-push helpers for the
acceleration-model tuning toolkit.

Every concrete tuning routine subclasses :class:`BaseTuneNode` to inherit:

* 60 Hz ``RobotMotionCommand`` publisher with BCM_LOCAL_ACCEL_OPEN_LOOP / BCM_OFF helpers
* ``ExtendedTelemetry`` subscriber that fills a :class:`TelemetryBuffer`
* Phase-tagged sample collection (so per-phase slices can be extracted later)
* ``/set_firmware_param`` push of an in-memory PARAM_MAP dict
* "Robot turned around" detection between linear trials (via KF Δθ)
* Wheel-encoder fallback body-velocity estimate via the controls FFI
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

import numpy as np

import rclpy
from rclpy.node import Node

from ateam_msgs.msg import RobotMotionCommand
from ateam_msgs.srv import SetFirmwareParameter

SCRIPTS_DIR = Path(__file__).resolve().parents[1]
CONTROLS_DIR = next(p for p in SCRIPTS_DIR.parents if p.name == "controls")

# analysis/ is a flat (non-package) python dir; make it importable.
sys.path.insert(0, str(CONTROLS_DIR / "analysis"))
from params import upload_params_dict, default_params_dict  # noqa: E402


# Axis-to-acceleration-component mapping. ``y`` is allowed only for
# diagnostic runs (the firmware doesn't have a separate y-friction model).
AXIS_TO_COMPONENT = {"x": 0, "y": 1, "theta": 2}
LINEAR_AXES = ("x", "y")
ANGULAR_AXES = ("theta",)
# All axes now push params to firmware (linear x/y have independent friction
# coefficients in the firmware friction model).
DIAGNOSTIC_ONLY_AXES: set[str] = set()


PHASE_REST = "rest"
PHASE_SETTLE = "settle"
PHASE_PULSE = "pulse"          # used by coulomb step
PHASE_PULSE_POS = "pulse_pos"  # triangular profile
PHASE_COAST = "coast"
PHASE_PULSE_NEG = "pulse_neg"
PHASE_WAIT_TURN = "wait_turn"  # waiting for user to manually rotate robot

# Vision gating. Without vision the state estimate dead-reckons on wheel encoders,
# which run away if a wheel lifts or slips. A sample counts as vision-active if a
# vision update arrived within this window (vision reaches the robot at ~30 Hz).
VISION_ACTIVE_WINDOW_S = 0.25
VISION_WAIT_TIMEOUT_S = 30.0
# A trial (or coulomb pulse tail) is discarded and redone if less than this
# fraction of its scored samples were vision-active.
MIN_VISION_COVERAGE = 0.9

# Every firmware param write resets the controller and reseeds vision. Uploads
# also set the seed sample count to this so vision is back within one sample.
TUNING_VISION_SEED_SAMPLES = 1

# Turnaround / settle detection. Rotation is accumulated from per-sample heading
# deltas so wrapping at +/-pi does not matter; per-sample jumps larger than
# HEADING_JUMP_REJECT_RAD are estimator snaps (e.g. vision reacquired), not motion.
HEADING_JUMP_REJECT_RAD = 0.5
# Settled = wheels and gyro quiet for SETTLE_HOLD_S. Uses raw sensors because the
# state estimate's velocity drifts while the robot sits still.
SETTLE_WHEEL_RADS = 1.0
SETTLE_GYRO_RADS = 0.2
SETTLE_HOLD_S = 0.3


# --------------------------------------------------------------------------
# Telemetry buffering
# --------------------------------------------------------------------------

@dataclass
class TelemetrySample:
    t: float                                  # seconds since trial-start
    phase: str
    cmd_accel: tuple[float, float, float]     # last published (ax, ay, atheta)
    kf_pos: tuple[float, float, float]
    kf_vel: tuple[float, float, float]
    wheel_vel: tuple[float, float, float, float]    # FL, BL, BR, FR
    imu_gyro: tuple[float, float, float] = (0.0, 0.0, 0.0)
    imu_accel: tuple[float, float, float] = (0.0, 0.0, 0.0)
    vision_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)
    vision_update: bool = False
    vision_active: bool = False
    kf_pos_prediction: tuple[float, float, float] = (0.0, 0.0, 0.0)
    kf_vel_prediction: tuple[float, float, float] = (0.0, 0.0, 0.0)
    body_traj_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    body_traj_vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
    body_accel_u: tuple[float, float, float] = (0.0, 0.0, 0.0)
    body_accel_u_fric_comp: tuple[float, float, float] = (0.0, 0.0, 0.0)
    body_control_mode: int = 0
    # Software-side local accel cmd echo (for completeness with plot_telem):
    local_acc_cmd: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class TelemetryBuffer:
    """Append-only buffer of :class:`TelemetrySample` records.

    Samples are sequestered by phase so a routine can ask for "all samples
    during pulse_pos" without scanning a giant list.
    """

    samples: list[TelemetrySample] = field(default_factory=list)

    def add(self, sample: TelemetrySample) -> None:
        self.samples.append(sample)

    def clear(self) -> None:
        self.samples.clear()

    # ---- accessors ----

    def phase_slice(self, phase: str) -> list[TelemetrySample]:
        return [s for s in self.samples if s.phase == phase]

    def vision_coverage(self, phase: str) -> float:
        """Fraction of a phase's samples that were vision-active (0.0 if none)."""
        sl = self.phase_slice(phase)
        if not sl:
            return 0.0
        return sum(s.vision_active for s in sl) / len(sl)

    def to_arrays(self) -> dict[str, np.ndarray]:
        """Return all per-sample series as NumPy arrays.

        Emitted keys come in two flavours:

        * Compact convenience keys used by the tuning analysis code:
          ``t, phase, cmd_accel, kf_pos, kf_vel, wheel_vel, imu_gyro_z``.
        * Firmware-style slash-keyed arrays compatible with
          ``analysis.visualization.body.plot_telem`` so trial NPZs can be
          plotted directly with the existing telemetry visualizer.
        """
        n = len(self.samples)
        if n == 0:
            empty3 = np.zeros((0, 3))
            empty4 = np.zeros((0, 4))
            empty = np.zeros(0)
            keys = {
                "t": empty,
                "phase": np.zeros(0, dtype="U16"),
                "cmd_accel": empty3,
                "kf_pos": empty3,
                "kf_vel": empty3,
                "wheel_vel": empty4,
                "imu_gyro_z": empty,
                "vision_active": np.zeros(0, dtype=bool),
            }
            keys.update(self._firmware_slash_arrays(empty_only=True))
            return keys

        t = np.array([s.t for s in self.samples])
        phase = np.array([s.phase for s in self.samples], dtype="U16")
        cmd_accel = np.array([s.cmd_accel for s in self.samples])
        kf_pos = np.array([s.kf_pos for s in self.samples])
        kf_vel = np.array([s.kf_vel for s in self.samples])
        wheel_vel = np.array([s.wheel_vel for s in self.samples])
        imu_gyro = np.array([s.imu_gyro for s in self.samples])
        imu_accel = np.array([s.imu_accel for s in self.samples])
        vision_pose = np.array([s.vision_pose for s in self.samples])
        vision_update = np.array([s.vision_update for s in self.samples], dtype=bool)
        vision_active = np.array([s.vision_active for s in self.samples], dtype=bool)
        kf_pos_pred = np.array([s.kf_pos_prediction for s in self.samples])
        kf_vel_pred = np.array([s.kf_vel_prediction for s in self.samples])
        traj_pos = np.array([s.body_traj_pos for s in self.samples])
        traj_vel = np.array([s.body_traj_vel for s in self.samples])
        accel_u = np.array([s.body_accel_u for s in self.samples])
        accel_u_fric = np.array([s.body_accel_u_fric_comp for s in self.samples])
        bcm = np.array([s.body_control_mode for s in self.samples], dtype=np.uint8)
        local_acc_cmd = np.array([s.local_acc_cmd for s in self.samples])
        wheel_fl = wheel_vel[:, 0]
        wheel_bl = wheel_vel[:, 1]
        wheel_br = wheel_vel[:, 2]
        wheel_fr = wheel_vel[:, 3]
        # Synthetic monotonic microsecond timestamp (per-trial t starts at 0).
        timestamp_us = (t * 1e6).astype(np.int64)
        return {
            # Compact convenience keys (used internally by tuning analysis):
            "t": t,
            "phase": phase,
            "cmd_accel": cmd_accel,
            "kf_pos": kf_pos,
            "kf_vel": kf_vel,
            "wheel_vel": wheel_vel,
            "imu_gyro_z": imu_gyro[:, 2],
            "vision_active": vision_active,
            # Firmware-style slash-keyed arrays for analysis/visualization/body.py
            "timestamp_us": timestamp_us,
            "body_control_telemetry/vision_pose": vision_pose,
            "body_control_telemetry/vision_update": vision_update,
            "body_control_telemetry/imu_gyro": imu_gyro,
            "body_control_telemetry/imu_accel": imu_accel,
            "body_control_telemetry/kf_body_pos_estimate": kf_pos,
            "body_control_telemetry/kf_body_vel_estimate": kf_vel,
            "body_control_telemetry/kf_body_pos_prediction": kf_pos_pred,
            "body_control_telemetry/kf_body_vel_prediction": kf_vel_pred,
            "body_control_telemetry/body_traj_pos": traj_pos,
            "body_control_telemetry/body_traj_vel": traj_vel,
            "body_control_telemetry/body_accel_u": accel_u,
            "body_control_telemetry/body_accel_u_fric_comp": accel_u_fric,
            "body_control_telemetry/body_control_mode": bcm,
            "body_control_telemetry/maneuver_local_acc/cmd_echo/local_xdd":
                local_acc_cmd[:, 0],
            "body_control_telemetry/maneuver_local_acc/cmd_echo/local_ydd":
                local_acc_cmd[:, 1],
            "body_control_telemetry/maneuver_local_acc/cmd_echo/local_alpha":
                local_acc_cmd[:, 2],
            "front_left_motor/velocity_telemetry/wheel_vel_rads": wheel_fl,
            "back_left_motor/velocity_telemetry/wheel_vel_rads": wheel_bl,
            "back_right_motor/velocity_telemetry/wheel_vel_rads": wheel_br,
            "front_right_motor/velocity_telemetry/wheel_vel_rads": wheel_fr,
        }

    def _firmware_slash_arrays(self, empty_only: bool = False) -> dict[str, np.ndarray]:
        empty1 = np.zeros(0)
        empty3 = np.zeros((0, 3))
        return {
            "timestamp_us": np.zeros(0, dtype=np.int64),
            "body_control_telemetry/vision_pose": empty3,
            "body_control_telemetry/vision_update": np.zeros(0, dtype=bool),
            "body_control_telemetry/imu_gyro": empty3,
            "body_control_telemetry/imu_accel": empty3,
            "body_control_telemetry/kf_body_pos_estimate": empty3,
            "body_control_telemetry/kf_body_vel_estimate": empty3,
            "body_control_telemetry/kf_body_pos_prediction": empty3,
            "body_control_telemetry/kf_body_vel_prediction": empty3,
            "body_control_telemetry/body_traj_pos": empty3,
            "body_control_telemetry/body_traj_vel": empty3,
            "body_control_telemetry/body_accel_u": empty3,
            "body_control_telemetry/body_accel_u_fric_comp": empty3,
            "body_control_telemetry/body_control_mode": np.zeros(0, dtype=np.uint8),
            "body_control_telemetry/maneuver_local_acc/cmd_echo/local_xdd": empty1,
            "body_control_telemetry/maneuver_local_acc/cmd_echo/local_ydd": empty1,
            "body_control_telemetry/maneuver_local_acc/cmd_echo/local_alpha": empty1,
            "front_left_motor/velocity_telemetry/wheel_vel_rads": empty1,
            "back_left_motor/velocity_telemetry/wheel_vel_rads": empty1,
            "back_right_motor/velocity_telemetry/wheel_vel_rads": empty1,
            "front_right_motor/velocity_telemetry/wheel_vel_rads": empty1,
        }

    def axis_velocity(self, phase: str, axis: str) -> tuple[np.ndarray, np.ndarray]:
        """Return ``(t, v)`` arrays from the KF velocity estimate for one
        phase/axis, **projected into the robot's local frame**.

        Only vision-active samples are returned (see ``VISION_ACTIVE_WINDOW_S``).

        The KF state estimate is in the global frame; ``BCM_LOCAL_ACCEL_OPEN_LOOP``
        commands and the slope-fit metric we compare against are in the
        robot's local frame. For linear axes (``x``, ``y``) we rotate
        ``(vx_global, vy_global)`` by ``−θ`` per sample. For ``theta`` the
        scalar angular velocity is frame-invariant.
        """
        sl = [s for s in self.phase_slice(phase) if s.vision_active]
        if not sl:
            return np.zeros(0), np.zeros(0)
        t = np.array([s.t for s in sl])
        if axis == "theta":
            v = np.array([s.kf_vel[2] for s in sl])
            return t, v
        # Linear axis: rotate global velocity into local frame.
        vx_g = np.array([s.kf_vel[0] for s in sl])
        vy_g = np.array([s.kf_vel[1] for s in sl])
        theta = np.array([s.kf_pos[2] for s in sl])
        c, s_ = np.cos(theta), np.sin(theta)
        vx_l = c * vx_g + s_ * vy_g
        vy_l = -s_ * vx_g + c * vy_g
        v = vx_l if axis == "x" else vy_l
        return t, v


# --------------------------------------------------------------------------
# Base node
# --------------------------------------------------------------------------

class BaseTuneNode(Node):
    """rclpy node that all tuning routines extend.

    The base node only handles plumbing: telemetry I/O, command publishing,
    and parameter pushing. Subclasses implement ``on_tick`` to drive their
    routine-specific state machine.
    """

    PUBLISH_RATE_HZ = 60.0

    def __init__(self, node_name: str, robot_id: int, axis: str,
                 dry_run: bool = False, skip_turnaround: bool = False):
        super().__init__(node_name)
        if axis not in AXIS_TO_COMPONENT:
            raise ValueError(f"Unknown axis {axis!r}; expected one of "
                             f"{list(AXIS_TO_COMPONENT)}")
        self.robot_id = int(robot_id)
        self.axis = axis
        self.dry_run = bool(dry_run)
        self.skip_turnaround = bool(skip_turnaround)
        self.is_diagnostic_only = axis in DIAGNOSTIC_ONLY_AXES

        self._trial_t0 = self.get_clock().now()
        self.current_phase = PHASE_REST
        self.current_cmd_accel = (0.0, 0.0, 0.0)   # last published cmd
        self.buffer = TelemetryBuffer()

        # Latest snapshot from telemetry (used for cross-trial decisions like
        # "robot turned around?").
        self.latest_kf_pos: Optional[tuple[float, float, float]] = None
        self.latest_kf_vel: Optional[tuple[float, float, float]] = None
        # Node-clock time of the last vision update; persists across trials.
        self._last_vision_time = None
        # Accumulated (unwrapped) heading and latest raw sensors for turnaround.
        self._heading_unwrapped: Optional[float] = None
        self._last_heading: Optional[float] = None
        self.latest_wheel_vel: Optional[tuple[float, float, float, float]] = None
        self.latest_gyro_z: Optional[float] = None

        # --- ROS pubs/subs/clients ---
        self.cmd_pub = self.create_publisher(
            RobotMotionCommand,
            f"/robot_motion_commands/robot{self.robot_id}",
            10,
        )
        self.create_timer(1.0 / self.PUBLISH_RATE_HZ, self._on_tick_wrapper)

        try:
            from ateam_radio_msgs.msg import ExtendedTelemetry
        except ImportError as exc:  # pragma: no cover - depends on workspace
            raise RuntimeError(
                "ateam_radio_msgs is required for accel-model tuning; "
                "source your ROS 2 workspace before running."
            ) from exc
        self.create_subscription(
            ExtendedTelemetry,
            f"/robot_feedback/extended/robot{self.robot_id}",
            self._on_extended_telem,
            10,
        )

        self.param_client = self.create_client(
            SetFirmwareParameter, "/set_firmware_param",
        )
        self.get_logger().info(
            f"[{node_name}] robot={self.robot_id} axis={self.axis} "
            f"dry_run={self.dry_run}"
        )

    # ------------------------------------------------------------------
    # Command publication helpers
    # ------------------------------------------------------------------

    def publish_accel(self, accel_value: float) -> None:
        """Publish a BCM_LOCAL_ACCEL_OPEN_LOOP with ``accel_value`` on this node's axis.

        Open loop: the firmware integrates the commanded accel into its trajectory
        and commands that twist/accel directly, with no pose feedback.
        """
        msg = RobotMotionCommand()
        msg.body_control_mode = RobotMotionCommand.BCM_LOCAL_ACCEL_OPEN_LOOP
        ax, ay, ath = 0.0, 0.0, 0.0
        if self.axis == "x":
            ax = float(accel_value)
        elif self.axis == "y":
            ay = float(accel_value)
        else:  # theta
            ath = float(accel_value)
        msg.acceleration.x = ax
        msg.acceleration.y = ay
        msg.acceleration.theta = ath
        self.current_cmd_accel = (ax, ay, ath)
        if not self.dry_run:
            self.cmd_pub.publish(msg)

    def publish_off(self) -> None:
        msg = RobotMotionCommand()
        msg.body_control_mode = RobotMotionCommand.BCM_OFF
        self.current_cmd_accel = (0.0, 0.0, 0.0)
        if not self.dry_run:
            self.cmd_pub.publish(msg)

    # ------------------------------------------------------------------
    # Trial bookkeeping
    # ------------------------------------------------------------------

    def begin_trial(self) -> None:
        """Reset the telemetry buffer and trial clock."""
        self.buffer.clear()
        self._trial_t0 = self.get_clock().now()
        self.current_phase = PHASE_REST

    def set_phase(self, phase: str) -> None:
        if phase != self.current_phase:
            self.get_logger().info(f"  phase → {phase}")
            self.current_phase = phase

    def trial_elapsed(self) -> float:
        return (self.get_clock().now() - self._trial_t0).nanoseconds * 1e-9

    # ------------------------------------------------------------------
    # Telemetry intake
    # ------------------------------------------------------------------

    @staticmethod
    def _xyz(v) -> tuple[float, float, float]:
        """Extract a 3-tuple from either an array-like or an x/y/z struct.

        The ExtendedTelemetry message uses ``float32[]`` for KF pose/vel and
        IMU vectors, so this needs to handle list indexing first; older
        message layouts used .x/.y/.z structs (still handled as a fallback).
        """
        if v is None:
            return (0.0, 0.0, 0.0)
        try:
            return (float(v[0]), float(v[1]), float(v[2]))
        except (TypeError, IndexError, KeyError):
            pass
        return (float(getattr(v, "x", 0.0)),
                float(getattr(v, "y", 0.0)),
                float(getattr(v, "z", 0.0)))

    def _on_extended_telem(self, msg) -> None:
        # KF state from body_control_telemetry. Field names mirror the
        # BodyControlExtendedTelemetry message generated from
        # software-communication/extended_telemetry.h.
        bct = getattr(msg, "body_control_telemetry", None)
        kf_pos = (0.0, 0.0, 0.0)
        kf_vel = (0.0, 0.0, 0.0)
        kf_pos_pred = (0.0, 0.0, 0.0)
        kf_vel_pred = (0.0, 0.0, 0.0)
        traj_pos = (0.0, 0.0, 0.0)
        traj_vel = (0.0, 0.0, 0.0)
        accel_u = (0.0, 0.0, 0.0)
        accel_u_fric = (0.0, 0.0, 0.0)
        vision_pose = (0.0, 0.0, 0.0)
        vision_update = False
        bcm = 0
        imu_gyro = (0.0, 0.0, 0.0)
        imu_accel = (0.0, 0.0, 0.0)
        local_acc_cmd = (0.0, 0.0, 0.0)

        if bct is not None:
            kf_pos = self._xyz(getattr(bct, "kf_body_pos_estimate", None))
            kf_vel = self._xyz(getattr(bct, "kf_body_vel_estimate", None))
            kf_pos_pred = self._xyz(getattr(bct, "kf_body_pos_prediction", None))
            kf_vel_pred = self._xyz(getattr(bct, "kf_body_vel_prediction", None))
            traj_pos = self._xyz(getattr(bct, "body_traj_pos", None))
            traj_vel = self._xyz(getattr(bct, "body_traj_vel", None))
            accel_u = self._xyz(getattr(bct, "body_accel_u", None))
            accel_u_fric = self._xyz(getattr(bct, "body_accel_u_fric_comp", None))
            vision_pose = self._xyz(getattr(bct, "vision_pose", None))
            vision_update = bool(getattr(bct, "vision_update", False))
            bcm = int(getattr(bct, "body_control_mode", 0))
            imu_gyro = self._xyz(getattr(bct, "imu_gyro", None))
            imu_accel = self._xyz(getattr(bct, "imu_accel", None))

            sla = getattr(bct, "maneuver_local_acc_open_loop", None)
            if bcm != RobotMotionCommand.BCM_LOCAL_ACCEL_OPEN_LOOP:
                sla = getattr(bct, "maneuver_local_acc", None)
            if sla is not None:
                echo = getattr(sla, "cmd_echo", None)
                if echo is not None:
                    local_acc_cmd = (
                        float(getattr(echo, "local_xdd", 0.0)),
                        float(getattr(echo, "local_ydd", 0.0)),
                        float(getattr(echo, "local_alpha", 0.0)),
                    )

        wheel_vels = []
        for motor in ("front_left_motor", "back_left_motor",
                       "back_right_motor", "front_right_motor"):
            m = getattr(msg, motor, None)
            if m is None:
                wheel_vels.append(0.0)
                continue
            vt = getattr(m, "velocity_telemetry", None)
            wheel_vels.append(float(getattr(vt, "wheel_vel_rads", 0.0))
                              if vt is not None else 0.0)

        now = self.get_clock().now()
        if vision_update:
            self._last_vision_time = now
        sample = TelemetrySample(
            t=self.trial_elapsed(),
            phase=self.current_phase,
            cmd_accel=self.current_cmd_accel,
            kf_pos=kf_pos,
            kf_vel=kf_vel,
            wheel_vel=tuple(wheel_vels),  # type: ignore[arg-type]
            imu_gyro=imu_gyro,
            imu_accel=imu_accel,
            vision_pose=vision_pose,
            vision_update=vision_update,
            vision_active=self.vision_active(now),
            kf_pos_prediction=kf_pos_pred,
            kf_vel_prediction=kf_vel_pred,
            body_traj_pos=traj_pos,
            body_traj_vel=traj_vel,
            body_accel_u=accel_u,
            body_accel_u_fric_comp=accel_u_fric,
            body_control_mode=bcm,
            local_acc_cmd=local_acc_cmd,
        )
        self.latest_kf_pos = kf_pos
        self.latest_kf_vel = kf_vel
        self.latest_wheel_vel = tuple(wheel_vels)  # type: ignore[assignment]
        self.latest_gyro_z = imu_gyro[2]
        heading = kf_pos[2]
        if self._last_heading is None:
            self._heading_unwrapped = heading
        else:
            dh = _wrap_angle(heading - self._last_heading)
            if abs(dh) <= HEADING_JUMP_REJECT_RAD:
                self._heading_unwrapped += dh
        self._last_heading = heading
        self.buffer.add(sample)

    def vision_active(self, now=None) -> bool:
        if self._last_vision_time is None:
            return False
        now = now if now is not None else self.get_clock().now()
        age = (now - self._last_vision_time).nanoseconds * 1e-9
        return age <= VISION_ACTIVE_WINDOW_S

    def wait_for_vision(self, timeout_sec: float = VISION_WAIT_TIMEOUT_S) -> None:
        """Hold BCM_OFF until vision is active. Raises VisionUnavailable on timeout."""
        if self.dry_run or self.vision_active():
            return
        self.get_logger().info(
            f">>> Waiting for vision on robot {self.robot_id}..."
        )
        start = self.get_clock().now()
        while rclpy.ok():
            self.publish_off()
            rclpy.spin_once(self, timeout_sec=1.0 / self.PUBLISH_RATE_HZ)
            if self.vision_active():
                self.get_logger().info(">>> Vision active.")
                return
            if (self.get_clock().now() - start).nanoseconds * 1e-9 > timeout_sec:
                raise VisionUnavailable(
                    f"No vision for robot {self.robot_id} within {timeout_sec:.0f}s."
                )

    # ------------------------------------------------------------------
    # Tick (subclass override)
    # ------------------------------------------------------------------

    def _on_tick_wrapper(self) -> None:
        try:
            self.on_tick()
        except Exception:  # pragma: no cover
            self.publish_off()
            raise

    def on_tick(self) -> None:
        """Subclass override. Default: just publish BCM_OFF for safety."""
        self.publish_off()

    # ------------------------------------------------------------------
    # Parameter push
    # ------------------------------------------------------------------

    def push_params(self, params_dict: dict,
                     only_keys: Optional[list[str]] = None) -> bool:
        """Push a subset of PARAM_MAP entries to the robot.

        No-ops (returns ``True``) when ``self.dry_run`` is set or when the
        axis is diagnostic-only (``y``).
        """
        if self.dry_run:
            self.get_logger().info(
                f"[dry-run] would push params: keys={only_keys or 'all'}"
            )
            return True
        if self.is_diagnostic_only:
            self.get_logger().info(
                f"[axis={self.axis} diagnostic-only] skipping firmware param push"
            )
            return True

        self.get_logger().info("Waiting for /set_firmware_param service...")
        if not self.param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/set_firmware_param not available")
            return False

        params_dict = {**params_dict,
                       "VISION_SEED_SAMPLES": [float(TUNING_VISION_SEED_SAMPLES)]}
        keys = (list(only_keys) if only_keys is not None
                else [k for k in params_dict])
        if "VISION_SEED_SAMPLES" not in keys:
            keys.append("VISION_SEED_SAMPLES")
        ok = upload_params_dict(
            self, self.param_client, self.robot_id, params_dict,
            only_keys=keys,
            logger=lambda s: self.get_logger().info(s),
        )
        if not ok:
            self.get_logger().warn("Param push had failures")
        return ok

    # ------------------------------------------------------------------
    # Turn-around detection (for linear axes between trials)
    # ------------------------------------------------------------------

    def _sensors_quiet(self) -> bool:
        if self.latest_wheel_vel is None or self.latest_gyro_z is None:
            return False
        return (max(abs(w) for w in self.latest_wheel_vel) < SETTLE_WHEEL_RADS
                and abs(self.latest_gyro_z) < SETTLE_GYRO_RADS)

    def wait_for_turnaround(self, prompt: bool = True,
                            theta_threshold: float = math.pi / 2,
                            timeout_sec: float = 120.0) -> bool:
        """Block until the user has manually rotated the robot.

        Linear axes: the heading accumulated since the wait began (unwrapped, so
        rotating past +/-pi still counts) must reach ``theta_threshold`` and the
        robot must then be settled (wheels and gyro quiet for SETTLE_HOLD_S).
        Angular axes only wait for the robot to settle. When
        ``self.skip_turnaround`` is True, returns after a brief settle wait.

        Raises :class:`TurnaroundTimeout` if the user does not rotate the
        robot within ``timeout_sec``. The caller is expected to abort
        cleanly; the search can be resumed later from the cached progress.
        """
        if getattr(self, "skip_turnaround", False):
            # Brief settle so the next pulse doesn't start mid-motion.
            settle_t0 = self.get_clock().now()
            while rclpy.ok():
                self.publish_off()
                rclpy.spin_once(self, timeout_sec=1.0 / self.PUBLISH_RATE_HZ)
                if (self.get_clock().now() - settle_t0).nanoseconds * 1e-9 > 0.5:
                    return True
            return True
        start = self.get_clock().now()
        anchor = self._heading_unwrapped
        need_rotation = self.axis in LINEAR_AXES
        if prompt:
            if need_rotation:
                self.get_logger().info(
                    f">>> Please rotate robot {self.robot_id} by >= "
                    f"{math.degrees(theta_threshold):.0f} deg before the next "
                    "pulse. Waiting..."
                )
            else:
                self.get_logger().info(
                    f">>> Waiting for robot {self.robot_id} to settle (any axis)..."
                )

        quiet_since = None
        last_report = 0.0
        while rclpy.ok():
            self.publish_off()
            rclpy.spin_once(self, timeout_sec=1.0 / self.PUBLISH_RATE_HZ)
            now = self.get_clock().now()
            elapsed = (now - start).nanoseconds * 1e-9
            if elapsed > timeout_sec:
                self.get_logger().error(
                    f"Turnaround wait timed out after {timeout_sec:.1f}s. "
                    "Exiting; rerun with the same --run-id to resume "
                    "the search from cached progress."
                )
                raise TurnaroundTimeout(
                    f"Timed out after {timeout_sec:.1f}s waiting for "
                    f"robot {self.robot_id} to be rotated >= "
                    f"{math.degrees(theta_threshold):.0f} deg."
                )
            if self._heading_unwrapped is None:
                continue
            if anchor is None:
                anchor = self._heading_unwrapped

            if self._sensors_quiet():
                if quiet_since is None:
                    quiet_since = now
            else:
                quiet_since = None
            settled = (quiet_since is not None
                       and (now - quiet_since).nanoseconds * 1e-9 >= SETTLE_HOLD_S)

            rotated = self._heading_unwrapped - anchor
            if need_rotation and prompt and elapsed - last_report >= 2.0:
                last_report = elapsed
                self.get_logger().info(
                    f"    rotated {math.degrees(rotated):+.0f} deg "
                    f"(need {math.degrees(theta_threshold):.0f}), "
                    f"settled={settled}"
                )
            if not settled:
                continue
            if not need_rotation:
                return True
            if abs(rotated) >= theta_threshold:
                self.get_logger().info(
                    f">>> Turnaround detected (rotated "
                    f"{math.degrees(rotated):+.0f} deg, settled)."
                )
                return True
        return False


class TurnaroundTimeout(RuntimeError):
    """Raised when the user does not rotate the robot in time.

    The CLI catches this and exits with a non-zero status. Resume by
    re-running with the same ``--run-id``; previously-evaluated candidates
    will be served from the cached ``progress.json`` without re-pulsing the
    robot.
    """


class VisionUnavailable(RuntimeError):
    """Raised when vision is missing for too long to run valid trials.

    Like TurnaroundTimeout, resume by re-running with the same ``--run-id``.
    """


def _wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi
