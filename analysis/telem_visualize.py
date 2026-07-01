#!/usr/bin/env python3
"""Telemetry visualization — body state + wheel plots.

Usage:
    python analysis/telem_visualize.py -t data/telemetry/robot_telemetry.npz -p data/robot_params.json
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt

from build import compile_controls
from visualization.body import plot_telem
from visualization.overlays import TrajectoryOverlay, VelocityOverlay
from visualization.wheels import plot_wheel_current, plot_wheel_velocity


def main():
    parser = argparse.ArgumentParser(description="Visualize telemetry data (body + wheels)")
    parser.add_argument("-t", "--telemetry", type=str, required=True,
                        help="Path to the telemetry NPZ file")
    parser.add_argument("-p", "--param-json", type=str, default=None,
                        help="Path to a JSON file containing RobotModel parameters")
    parser.add_argument("-w", "--velocity-window", type=float, default=0.2,
                        help="Sliding window size in seconds for vision-derived velocity (default: 0.2)")
    args = parser.parse_args()

    compile_controls()
    telemetry = np.load(args.telemetry)

    # Body state visualization (3×3 grid)
    fig, axs, t, state_est = plot_telem(telemetry, param_json=args.param_json)

    # try:
    #     overlay = TrajectoryOverlay(fig, axs, t, state_est, telemetry, param_json=args.param_json)
    #     vel_overlay = VelocityOverlay(fig, axs, t, telemetry, window=args.velocity_window)

    #     def _toggle_overlays(event):
    #         if event.key == " ":
    #             overlay.toggle()
    #             vel_overlay.toggle()

    #     fig.canvas.mpl_connect("key_press_event", _toggle_overlays)
    #     print("[telem-visualize] Press Space to toggle trajectory & velocity overlay.")
    #     print(f"[telem-visualize] Velocity window: {args.velocity_window:.2f}s  (adjust with [ / ])")
    # except Exception as exc:
    #     print(f"Failed to set up trajectory overlay: {exc}")

    # Wheel visualizations
    plot_wheel_current(telemetry)
    plot_wheel_velocity(telemetry)

    plt.show()


if __name__ == "__main__":
    main()
