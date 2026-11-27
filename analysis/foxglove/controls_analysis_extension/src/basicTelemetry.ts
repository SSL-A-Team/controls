// Copyright 2026 A Team
//
// Conversion from `ateam_radio_msgs/BasicTelemetry` into the flat
// `BasicAnalysis` shape the basic-telemetry layout binds to.
//
// BasicTelemetry is what every robot always sends (100 Hz); ExtendedTelemetry
// has to be enabled separately. Basic carries the firmware state estimate
// (quantized: position in mm / mrad, velocity in mm/s / mrad/s in the robot's
// local frame), the active body control mode and robot status, but no robot
// clock, so it plots on receive time only.

import { MessageSchemaDescription } from "@foxglove/extension";

const NAN = Number.NaN;

const DIMS = ["x", "y", "theta"] as const;

// Every *_error flag in BasicTelemetry; `error` is set when any of them is.
const ERROR_FIELDS = [
  "power_error",
  "power_board_error",
  "battery_error",
  "tipped_error",
  "breakbeam_error",
  "accelerometer_0_error",
  "accelerometer_1_error",
  "gyroscope_0_error",
  "gyroscope_1_error",
  "motor_fl_general_error",
  "motor_fl_hall_error",
  "motor_bl_general_error",
  "motor_bl_hall_error",
  "motor_br_general_error",
  "motor_br_hall_error",
  "motor_fr_general_error",
  "motor_fr_hall_error",
  "motor_drib_general_error",
  "motor_drib_hall_error",
  "kicker_board_error",
] as const;

// Per-converter state for packet-loss tracking. Reset when the converter is
// recreated (e.g. the selected robot changes).
export type BasicSequenceState = { lastSeq?: number };

type Vec = ArrayLike<number> | undefined;

function at(a: Vec, i: number): number {
  return a != undefined && i < a.length ? Number(a[i]) : NAN;
}

export function basicAnalysisSchemaDescription(): MessageSchemaDescription {
  const dim: MessageSchemaDescription = {
    pos_estimate: "number",
    vel_estimate: "number",
  };
  return {
    body_control_mode: "number",
    battery_percent: "number",
    kicker_charge_percent: "number",
    breakbeam_ball_detected: "number",
    controller_reset: "number",
    error: "number",
    packets_dropped: "number",
    x: dim,
    y: dim,
    theta: dim,
  };
}

// Convert one BasicTelemetry message. Position is converted to m / rad and the
// local-frame velocity is rotated into the global frame by the reported
// heading, matching the extended-telemetry `pos_estimate` / `vel_estimate`.
// `packets_dropped` is the number of transmission sequence numbers skipped since
// the previous packet from this robot (uint8, wraps), NaN for the first packet.
export function buildBasicAnalysis(
  msg: any,
  seq?: BasicSequenceState,
): Record<string, unknown> {
  const pos = [at(msg.pos_est, 0) / 1e3, at(msg.pos_est, 1) / 1e3, at(msg.pos_est, 2) / 1e3];
  const lvx = at(msg.vel_est, 0) / 1e3;
  const lvy = at(msg.vel_est, 1) / 1e3;
  const c = Math.cos(pos[2]!);
  const s = Math.sin(pos[2]!);
  const vel = [c * lvx - s * lvy, s * lvx + c * lvy, at(msg.vel_est, 2) / 1e3];

  let packetsDropped = NAN;
  const seqNum = Number(msg.transmission_sequence_number);
  if (seq != undefined && Number.isFinite(seqNum)) {
    if (seq.lastSeq != undefined) {
      packetsDropped = (seqNum - seq.lastSeq - 1 + 256) % 256;
    }
    seq.lastSeq = seqNum;
  }

  const out: Record<string, unknown> = {
    body_control_mode: Number(msg.body_control_mode),
    battery_percent: Number(msg.battery_percent),
    kicker_charge_percent: Number(msg.kicker_charge_percent),
    breakbeam_ball_detected: msg.breakbeam_ball_detected ? 1 : 0,
    controller_reset: msg.controller_reset ? 1 : 0,
    error: ERROR_FIELDS.some((f) => Boolean(msg[f])) ? 1 : 0,
    packets_dropped: packetsDropped,
  };
  DIMS.forEach((d, i) => {
    out[d] = { pos_estimate: pos[i], vel_estimate: vel[i] };
  });
  return out;
}
