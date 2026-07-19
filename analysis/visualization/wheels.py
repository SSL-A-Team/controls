"""Wheel current and velocity visualization."""

import numpy as np
import matplotlib.pyplot as plt

from visualization.timeline import compute_timeline, draw_reboot_lines


WHEEL_NAMES = ["front_left", "front_right", "back_left", "back_right"]

WHEEL_GRID_POS = {
    "front_left":  (0, 0),
    "front_right": (0, 1),
    "back_left":   (1, 0),
    "back_right":  (1, 1),
}


def plot_wheel_current(telemetry):
    """Plot commanded vs measured current for each wheel in a 2x2 grid."""
    t, reboot_times = compute_timeline(telemetry)

    fig, axs = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    fig.suptitle("Wheel Current: Commanded vs Measured", fontsize=16)
    alpha = 0.7

    for name in WHEEL_NAMES:
        i, j = WHEEL_GRID_POS[name]
        ax = axs[i, j]

        current_cmd = telemetry[f'{name}_motor/current_telemetry/current_setpoint_ma']
        current_meas = np.mean(
            telemetry[f'{name}_motor/current_telemetry/current_samples_ma'], axis=1
        )
        current_meas = current_meas * np.sign(current_cmd)

        ax.plot(t, current_cmd, label='commanded', alpha=alpha)
        ax.plot(t, current_meas, label='measured', alpha=alpha)
        ax.set_title(name.replace('_', ' ').title(), fontsize=13)
        ax.set_ylabel('Current (mA)', fontsize=12)
        ax.legend()
        ax.grid(True, alpha=0.3)

    for ax in axs[1, :]:
        ax.set_xlabel('Time (s)', fontsize=12)

    draw_reboot_lines(axs, reboot_times)

    plt.tight_layout()
    return fig, axs


def plot_wheel_velocity(telemetry):
    """Plot commanded vs measured velocity for each wheel in a 2x2 grid."""
    t, reboot_times = compute_timeline(telemetry)

    fig, axs = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    fig.suptitle("Wheel Velocity: Commanded vs Measured", fontsize=16)
    alpha = 0.7

    for name in WHEEL_NAMES:
        i, j = WHEEL_GRID_POS[name]
        ax = axs[i, j]

        vel_cmd = telemetry[f'{name}_motor/velocity_telemetry/vel_setpoint_rads']
        vel_meas = telemetry[f'{name}_motor/velocity_telemetry/wheel_vel_rads']

        ax.plot(t, vel_cmd, label='commanded', alpha=alpha)
        ax.plot(t, vel_meas, label='measured', alpha=alpha)
        ax.set_title(name.replace('_', ' ').title(), fontsize=13)
        ax.set_ylabel('Velocity (rad/s)', fontsize=12)
        ax.legend()
        ax.grid(True, alpha=0.3)

    for ax in axs[1, :]:
        ax.set_xlabel('Time (s)', fontsize=12)

    draw_reboot_lines(axs, reboot_times)

    plt.tight_layout()
    return fig, axs
