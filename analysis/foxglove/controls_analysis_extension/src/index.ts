// Copyright 2026 A Team
//
// Foxglove extension: controls analysis.
//
// This extension does all controls analysis inside Foxglove. It registers four topic
// converters that read every robot's raw topics and emit, per stream, only the
// selected robot's converted message onto one stable output topic the layouts
// bind to:
//
//   /robot_feedback/extended/robot0..15  -> /analysis_telem    (ControlsAnalysis)
//   /robot_feedback/basic/robot0..15     -> /analysis_basic    (BasicAnalysis)
//   /robot_motion_commands/robot0..15    -> /analysis_control   (MotionCommand)
//   /{blue,yellow}_team/robot0..15       -> /analysis_vision    (VisionState)
//
// Selection is driven by the `robot` global variable (and `team` for vision):
// each converter is registered with `watchVariables`, so it is recreated when the
// selection changes; the recreated converter closes over the selected source
// topic name and drops every message whose topic doesn't match. Changing
// `robot`/`team` in the Variables tab re-points every panel with no bag reload.
//
// It operates at Foxglove's data-source layer, so it behaves identically for a
// loaded bag (full backfill) and a live connection.

import { ExtensionContext } from "@foxglove/extension";

import {
  BasicSequenceState,
  basicAnalysisSchemaDescription,
  buildBasicAnalysis,
} from "./basicTelemetry";
import {
  buildControlsAnalysis,
  controlsAnalysisSchemaDescription,
  RobotClockAnchor,
} from "./controlsAnalysis";
import {
  buildMotionCommand,
  motionCommandSchemaDescription,
} from "./motionCommand";
import { buildVisionState, visionStateSchemaDescription } from "./visionState";

// Global variables that select what is displayed.
export const ROBOT_VARIABLE = "robot"; // robot id, default 0
export const TEAM_VARIABLE = "team"; // "blue" | "yellow", default "blue"
export const DEFAULT_TEAM = "blue";

// Robot ids the converters listen to. The fleet is 0..15; input topics a given
// data source doesn't contain are simply never delivered.
export const ROBOT_IDS = Array.from({ length: 16 }, (_, i) => i);
export const TEAMS = ["blue", "yellow"] as const;

// Raw per-robot source topic templates.
export const EXTENDED_TOPIC_TEMPLATE = "/robot_feedback/extended/robot{robot}";
export const BASIC_TOPIC_TEMPLATE = "/robot_feedback/basic/robot{robot}";
export const COMMAND_TOPIC_TEMPLATE = "/robot_motion_commands/robot{robot}";
export const TEAM_TOPIC_TEMPLATE = "/{team}_team/robot{robot}";

// Converter output topics = what the layouts bind to.
export const TELEM_TOPIC = "/analysis_telem";
export const BASIC_TOPIC = "/analysis_basic";
export const CONTROL_TOPIC = "/analysis_control";
export const VISION_TOPIC = "/analysis_vision";

// Converted schema names.
export const CONTROLS_ANALYSIS_SCHEMA = "ateam_controls_analysis/ControlsAnalysis";
export const BASIC_ANALYSIS_SCHEMA = "ateam_controls_analysis/BasicAnalysis";
export const MOTION_COMMAND_SCHEMA = "ateam_controls_analysis/MotionCommand";
export const VISION_STATE_SCHEMA = "ateam_controls_analysis/VisionState";

function extendedTopic(robot: number): string {
  return EXTENDED_TOPIC_TEMPLATE.replace("{robot}", String(robot));
}

function basicTopic(robot: number): string {
  return BASIC_TOPIC_TEMPLATE.replace("{robot}", String(robot));
}

function commandTopic(robot: number): string {
  return COMMAND_TOPIC_TEMPLATE.replace("{robot}", String(robot));
}

function teamTopic(team: string, robot: number): string {
  return TEAM_TOPIC_TEMPLATE.replace("{team}", team).replace("{robot}", String(robot));
}

function robotIdFrom(value: unknown): number {
  const n =
    typeof value === "number"
      ? value
      : typeof value === "string"
        ? Number.parseInt(value, 10)
        : NaN;
  return Number.isFinite(n) && n >= 0 ? Math.trunc(n) : 0;
}

function teamFrom(value: unknown): string {
  return value === "yellow" || value === "blue" ? value : DEFAULT_TEAM;
}

// Latest estimated heading (rad) of the selected robot, written by the telemetry
// converters (extended or basic, whichever the data source has) and read by the command converter to rotate local-frame commands into
// the global frame. Cleared when the robot selection changes.
const selectedRobotHeading = { theta: Number.NaN };

export function activate(extensionContext: ExtensionContext): void {
  // ExtendedTelemetry -> ControlsAnalysis (the main analysis). Reads every
  // robot's telemetry topic, emits only the selected robot's.
  extensionContext.registerMessageConverter({
    type: "topic",
    inputTopics: ROBOT_IDS.map(extendedTopic),
    outputTopic: TELEM_TOPIC,
    outputSchemaName: CONTROLS_ANALYSIS_SCHEMA,
    outputSchemaDescription: controlsAnalysisSchemaDescription(),
    watchVariables: [ROBOT_VARIABLE],
    create: (globalVariables) => {
      const selected = extendedTopic(robotIdFrom(globalVariables[ROBOT_VARIABLE]));
      // Per-converter clock anchor: captures the robot->PC time offset from the
      // first packet so the synced `header.stamp` sits on the PC timeline. Reset
      // on recreation (robot selection change) so we re-anchor for the new robot.
      const clock: RobotClockAnchor = {};
      selectedRobotHeading.theta = Number.NaN;
      return (messageEvent) => {
        if (messageEvent.topic !== selected) {
          return undefined;
        }
        const msg: any = messageEvent.message;
        const pos = msg?.body_control_telemetry?.pos_est;
        if (pos != undefined && pos.length > 2) {
          selectedRobotHeading.theta = Number(pos[2]);
        }
        return buildControlsAnalysis(msg, messageEvent.receiveTime, clock);
      };
    },
  });

  // BasicTelemetry -> BasicAnalysis (estimate, mode and status; always sent by
  // the robots, unlike ExtendedTelemetry).
  extensionContext.registerMessageConverter({
    type: "topic",
    inputTopics: ROBOT_IDS.map(basicTopic),
    outputTopic: BASIC_TOPIC,
    outputSchemaName: BASIC_ANALYSIS_SCHEMA,
    outputSchemaDescription: basicAnalysisSchemaDescription(),
    watchVariables: [ROBOT_VARIABLE],
    create: (globalVariables) => {
      const selected = basicTopic(robotIdFrom(globalVariables[ROBOT_VARIABLE]));
      const seq: BasicSequenceState = {};
      selectedRobotHeading.theta = Number.NaN;
      return (messageEvent) => {
        if (messageEvent.topic !== selected) {
          return undefined;
        }
        const msg: any = messageEvent.message;
        const pos = msg?.pos_est;
        if (pos != undefined && pos.length > 2) {
          selectedRobotHeading.theta = Number(pos[2]) / 1e3;
        }
        return buildBasicAnalysis(msg, seq);
      };
    },
  });

  // RobotMotionCommand -> MotionCommand (pre-send software command).
  extensionContext.registerMessageConverter({
    type: "topic",
    inputTopics: ROBOT_IDS.map(commandTopic),
    outputTopic: CONTROL_TOPIC,
    outputSchemaName: MOTION_COMMAND_SCHEMA,
    outputSchemaDescription: motionCommandSchemaDescription(),
    watchVariables: [ROBOT_VARIABLE],
    create: (globalVariables) => {
      const selected = commandTopic(robotIdFrom(globalVariables[ROBOT_VARIABLE]));
      return (messageEvent) =>
        messageEvent.topic === selected
          ? buildMotionCommand(messageEvent.message, selectedRobotHeading.theta)
          : undefined;
    },
  });

  // VisionStateRobot -> VisionState (fresh software-side vision estimate). Reads
  // both teams' per-robot topics; emits only the selected robot on the selected
  // team.
  extensionContext.registerMessageConverter({
    type: "topic",
    inputTopics: TEAMS.flatMap((team) => ROBOT_IDS.map((id) => teamTopic(team, id))),
    outputTopic: VISION_TOPIC,
    outputSchemaName: VISION_STATE_SCHEMA,
    outputSchemaDescription: visionStateSchemaDescription(),
    watchVariables: [ROBOT_VARIABLE, TEAM_VARIABLE],
    create: (globalVariables) => {
      const selected = teamTopic(
        teamFrom(globalVariables[TEAM_VARIABLE]),
        robotIdFrom(globalVariables[ROBOT_VARIABLE]),
      );
      return (messageEvent) =>
        messageEvent.topic === selected
          ? buildVisionState(messageEvent.message)
          : undefined;
    },
  });
}
