"""Telemetry visualization — state estimates, measurements, and commands."""

import ctypes
import numpy as np
import matplotlib.pyplot as plt
from itertools import chain

from ateam_controls import (
    robot_model_transform_wheel2twist,
    robot_model_free,
)
from params import load_robot_model
from visualization.timeline import compute_timeline, draw_reboot_lines


# body_control_mode values — must match software-communication basic_control.h
# BodyControlMode (telemetry reports these raw enum values).
_BCM_GLOBAL_POS = 10
_BCM_GLOBAL_VEL = 11
_BCM_LOCAL_VEL = 12
_BCM_GLOBAL_ACC = 13
_BCM_LOCAL_ACC = 14


state_measurement_labels = [
    [["vision_x_meas"],              ["vision_y_meas"],            ["vision_theta_meas"]],
    [["encoder_xd_meas"],            ["encoder_yd_meas"],          ["encoder_thetad_meas", "gyro_thetad_meas"]],
]


def local_to_global_xy(theta, local_x, local_y):
    """Rotate local-frame X/Y vectors to global frame using the robot heading."""
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    gx = cos_theta * local_x - sin_theta * local_y
    gy = sin_theta * local_x + cos_theta * local_y
    return gx, gy


def compute_all_state_measurements(state_estimates, sensor_measurements, param_json=None):
    """
    Compute state measurements for all time steps.

    :param state_estimates: t x 6 array of state estimates (x, y, theta, xd, yd, thetad)
    :param sensor_measurements: t x 8 array of sensor measurements
    :param param_json: Optional path to a JSON file containing RobotModel parameters (uses defaults if None)
    :return: dictionary of state measurement time series
    """
    model = load_robot_model(param_json)
    try:
        all_labels = list(chain.from_iterable(chain.from_iterable(state_measurement_labels)))
        state_measurements = {label: [] for label in all_labels}

        for i in range(state_estimates.shape[0]):
            theta = float(state_estimates[i, 2])
            sensors = sensor_measurements[i]

            mat_c = robot_model_transform_wheel2twist(model, ctypes.c_float(theta))
            mat = np.array(list(mat_c.data), dtype=np.float32).reshape((3, 4), order='F')
            encoders = np.array(sensors[3:7], dtype=np.float32)
            twist = mat @ encoders

            meas = {
                "vision_x_meas": sensors[0],
                "vision_y_meas": sensors[1],
                "vision_theta_meas": sensors[2],
                "encoder_xd_meas": twist[0],
                "encoder_yd_meas": twist[1],
                "encoder_thetad_meas": twist[2],
                "gyro_thetad_meas": sensors[7],
            }
            for label in all_labels:
                state_measurements[label].append(meas[label])
    finally:
        robot_model_free(model)

    return {k: np.array(v) for k, v in state_measurements.items()}


def plot_telem(telemetry, param_json=None):
    """Create the main 3x3 telemetry plot (pos/vel/accel × x/y/theta)."""
    sensor_meas = np.stack([
        telemetry["body_control_telemetry/vision_pose"][:,0],
        telemetry["body_control_telemetry/vision_pose"][:,1],
        telemetry["body_control_telemetry/vision_pose"][:,2],
        telemetry['front_left_motor/velocity_telemetry/wheel_vel_rads'],
        telemetry['back_left_motor/velocity_telemetry/wheel_vel_rads'],
        telemetry['back_right_motor/velocity_telemetry/wheel_vel_rads'],
        telemetry['front_right_motor/velocity_telemetry/wheel_vel_rads'],
        telemetry['body_control_telemetry/imu_gyro'][:,2],
    ], axis=1)

    t, reboot_times = compute_timeline(telemetry)

    state_pred = np.append(
        telemetry['body_control_telemetry/kf_body_pos_prediction'],
        telemetry['body_control_telemetry/kf_body_vel_prediction'],
        axis=1
    )
    state_est = np.append(
        telemetry['body_control_telemetry/kf_body_pos_estimate'],
        telemetry['body_control_telemetry/kf_body_vel_estimate'],
        axis=1
    )
    state_meas = compute_all_state_measurements(state_est, sensor_meas, param_json=param_json)
    cmd = telemetry['body_control_telemetry/body_accel_u']
    cmd_meas = telemetry['body_control_telemetry/imu_accel'][:, :2]

    fig, axs = plt.subplots(3, 3, figsize=(12, 10), sharex=True)
    plt.tight_layout()
    alpha = 0.5
    axs[0, 1].sharey(axs[0, 0])
    axs[1, 1].sharey(axs[1, 0])
    axs[2, 1].sharey(axs[2, 0])

    for i, row in enumerate([
        ["x (m)", "y (m)", "theta (rad)"],
        ["xd (m/s)", "yd (m/s)", "thetad (rad/s)"],
        ["xdd (m/s^2)", "ydd (m/s^2)", "thetadd (rad/s^2)"],
    ]):
        for j, label in enumerate(row):
            ax = axs[i, j]
            ax.set_ylabel(label, fontsize=14)

    for ax in axs[2, :]:
        ax.set_xlabel('Time (s)', fontsize=14)

    for i in range(2):
        for j in range(3):
            ax = axs[i, j]
            ax.plot(t, state_pred[:, 3*i + j], label=f'predicted', color='tab:blue', alpha=alpha)
            ax.plot(t, state_est[:, 3*i + j], label=f'estimate', color='red', alpha=alpha, zorder=10)

            for meas_label in state_measurement_labels[i][j]:
                if "vision" in meas_label:
                    mask = telemetry["body_control_telemetry/vision_update"].astype(bool)
                    t_meas = t[mask]
                    m_meas = state_meas[meas_label][mask]
                    ax.plot(t_meas, m_meas, label=f'{meas_label}', color='lime', alpha=alpha)
                elif "gyro" in meas_label:
                    ax.plot(t, state_meas[meas_label], label=f'{meas_label}', color='cyan', alpha=alpha)
                else:
                    ax.plot(t, state_meas[meas_label], label=f'{meas_label}', color='tab:orange', alpha=alpha)

    i = 0
    for j in range(3):
        ax = axs[i, j]
        ax.plot(t, telemetry["body_control_telemetry/body_traj_pos"][:, j], label=f'trajectory', color='tab:green', alpha=alpha)
        ax.legend()
    i = 1
    for j in range(3):
        ax = axs[i, j]
        ax.plot(t, telemetry["body_control_telemetry/body_traj_vel"][:, j], label=f'trajectory', color='tab:green', alpha=alpha)
        ax.legend()

    cmd_fric_comp = telemetry['body_control_telemetry/body_accel_u_fric_comp']
    i = 2
    for j in range(3):
        ax = axs[i, j]
        ax.plot(t, cmd[:, j], label=f'firmware_cmd', color='tab:blue', alpha=alpha)
        ax.plot(t, cmd_fric_comp[:, j], label=f'firmware_cmd_fric_comp', color='tab:orange', alpha=alpha)
        ax.legend()

    bcm = telemetry["body_control_telemetry/body_control_mode"]
    theta = state_est[:, 2]

    i = 0
    for j in range(3):
        ax = axs[i, j]
        dim_name = ["x", "y", "theta"][j]
        mask = (bcm == _BCM_GLOBAL_POS)
        if any(mask):
            ax.plot(t[mask], telemetry[f"body_control_telemetry/maneuver_global_pos/cmd_echo/global_{dim_name}"][mask], label='software_cmd', color='purple', alpha=alpha)
    i = 1
    local_vel_mask = bcm == _BCM_LOCAL_VEL
    if any(local_vel_mask):
        local_vx = telemetry["body_control_telemetry/maneuver_local_vel/cmd_echo/local_xd"]
        local_vy = telemetry["body_control_telemetry/maneuver_local_vel/cmd_echo/local_yd"]
        global_vx, global_vy = local_to_global_xy(theta, local_vx, local_vy)
        local_omega = telemetry["body_control_telemetry/maneuver_local_vel/cmd_echo/local_omega"]
    for j in range(3):
        ax = axs[i, j]
        dim_name = ["xd", "yd", "omega"][j]
        mask = bcm == _BCM_GLOBAL_VEL
        if any(mask):
            ax.plot(t[mask], telemetry[f"body_control_telemetry/maneuver_global_vel/cmd_echo/global_{dim_name}"][mask], label='software_cmd', color='purple', alpha=alpha)
        if any(local_vel_mask):
            local_vel_global = [global_vx, global_vy, local_omega][j]
            ax.plot(t[local_vel_mask], local_vel_global[local_vel_mask], label='software_cmd (local→global)', color='purple', alpha=alpha)
    i = 2
    local_acc_mask = bcm == _BCM_LOCAL_ACC
    if any(local_acc_mask):
        local_ax = telemetry["body_control_telemetry/maneuver_local_acc/cmd_echo/local_xdd"]
        local_ay = telemetry["body_control_telemetry/maneuver_local_acc/cmd_echo/local_ydd"]
        global_ax, global_ay = local_to_global_xy(theta, local_ax, local_ay)
        local_alpha = telemetry["body_control_telemetry/maneuver_local_acc/cmd_echo/local_alpha"]
    for j in range(3):
        ax = axs[i, j]
        dim_name = ["xdd", "ydd", "alpha"][j]
        mask = bcm == _BCM_GLOBAL_ACC
        if any(mask):
            ax.plot(t[mask], telemetry[f"body_control_telemetry/maneuver_global_acc/cmd_echo/global_{dim_name}"][mask], label='software_cmd', color='purple', alpha=alpha)
        if any(local_acc_mask):
            local_acc_global = [global_ax, global_ay, local_alpha][j]
            ax.plot(t[local_acc_mask], local_acc_global[local_acc_mask], label='software_cmd (local→global)', color='purple', alpha=alpha)

    draw_reboot_lines(axs, reboot_times)

    for ax in axs.flat:
        ax.tick_params(labelbottom=True)
        ax.legend()

    return fig, axs, t, state_est
