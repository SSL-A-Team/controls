"""Shared rclpy node, telemetry buffer, and param-push helpers for the
acceleration-model tuning toolkit.

Every concrete tuning routine subclasses :class:`BaseTuneNode` to inherit:

* 60 Hz ``RobotMotionCommand`` publisher with BCM_LOCAL_ACCEL / BCM_OFF helpers
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
    kf_pos_prediction: tuple[float, float, float] = (0.0, 0.0, 0.0)
    kf_vel_prediction: tuple[float, float, float] = (0.0, 0.0, 0.0)
    body_traj_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    body_traj_vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
    body_accel_u: tuple[float, float, float] = (0.0, 0.0, 0.0)
    body_accel_u_fric_comp: tuple[float, float, float] = (0.0, 0.0, 0.0)
    body_control_mode: int = 0
    # Software-side BCM_LOCAL_ACCEL cmd echo (for completeness with plot_telem):
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
            "body_control_telemetry/skill_local_acc/cmd_echo/local_xdd":
                local_acc_cmd[:, 0],
            "body_control_telemetry/skill_local_acc/cmd_echo/local_ydd":
                local_acc_cmd[:, 1],
            "body_control_telemetry/skill_local_acc/cmd_echo/local_alpha":
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
            "body_control_telemetry/skill_local_acc/cmd_echo/local_xdd": empty1,
            "body_control_telemetry/skill_local_acc/cmd_echo/local_ydd": empty1,
            "body_control_telemetry/skill_local_acc/cmd_echo/local_alpha": empty1,
            "front_left_motor/velocity_telemetry/wheel_vel_rads": empty1,
            "back_left_motor/velocity_telemetry/wheel_vel_rads": empty1,
            "back_right_motor/velocity_telemetry/wheel_vel_rads": empty1,
            "front_right_motor/velocity_telemetry/wheel_vel_rads": empty1,
        }

    def axis_velocity(self, phase: str, axis: str) -> tuple[np.ndarray, np.ndarray]:
        """Return ``(t, v)`` arrays from the KF velocity estimate for one
        phase/axis, **projected into the robot's local frame**.

        The KF state estimate is in the global frame; ``BCM_LOCAL_ACCEL``
        commands and the slope-fit metric we compare against are in the
        robot's local frame. For linear axes (``x``, ``y``) we rotate
        ``(vx_global, vy_global)`` by ``−θ`` per sample. For ``theta`` the
        scalar angular velocity is frame-invariant.
        """
        idx = AXIS_TO_COMPONENT[axis]
        sl = self.phase_slice(phase)
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
        """Publish a BCM_LOCAL_ACCEL with ``accel_value`` on this node's axis."""
        msg = RobotMotionCommand()
        msg.body_control_mode = RobotMotionCommand.BCM_LOCAL_ACCEL
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

            sla = getattr(bct, "skill_local_acc", None)
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
        self.buffer.add(sample)

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

        ok = upload_params_dict(
            self, self.param_client, self.robot_id, params_dict,
            only_keys=only_keys,
            logger=lambda s: self.get_logger().info(s),
        )
        if not ok:
            self.get_logger().warn("Param push had failures")
        return ok

    # ------------------------------------------------------------------
    # Turn-around detection (for linear axes between trials)
    # ------------------------------------------------------------------

    def wait_for_turnaround(self, prompt: bool = True,
                            theta_threshold: float = math.pi / 2,
                            vel_settle: float = 0.05,
                            timeout_sec: float = 120.0) -> bool:
        """Block until the user has manually rotated the robot.

        Detected as ``|Δθ_kf| > theta_threshold`` *and* ``|kf_vel| < vel_settle``
        on all axes. For angular routines or when no prior pose is known, just
        waits for the velocity to settle. When ``self.skip_turnaround`` is
        True, returns immediately after a brief settle wait.

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
        anchor_pos = self.latest_kf_pos
        if prompt:
            if self.axis in LINEAR_AXES:
                self.get_logger().info(
                    f">>> Please rotate robot {self.robot_id} by >= 90 deg "
                    "before the next pulse. Waiting..."
                )
            else:
                self.get_logger().info(
                    f">>> Waiting for robot {self.robot_id} to settle (any axis)..."
                )

        while rclpy.ok():
            self.publish_off()
            rclpy.spin_once(self, timeout_sec=1.0 / self.PUBLISH_RATE_HZ)
            elapsed = (self.get_clock().now() - start).nanoseconds * 1e-9
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
            pos = self.latest_kf_pos
            vel = self.latest_kf_vel
            if pos is None or vel is None:
                continue
            settled = all(abs(v) < vel_settle for v in vel)
            if self.axis in LINEAR_AXES:
                if anchor_pos is None:
                    anchor_pos = pos
                    continue
                dtheta = _wrap_angle(pos[2] - anchor_pos[2])
                if abs(dtheta) > theta_threshold and settled:
                    self.get_logger().info(
                        f">>> Turnaround detected (Δθ={dtheta:.2f} rad, settled)."
                    )
                    return True
            else:
                if settled:
                    return True
        return False


class TurnaroundTimeout(RuntimeError):
    """Raised when the user does not rotate the robot in time.

    The CLI catches this and exits with a non-zero status. Resume by
    re-running with the same ``--run-id``; previously-evaluated candidates
    will be served from the cached ``progress.json`` without re-pulsing the
    robot.
    """


def _wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi
