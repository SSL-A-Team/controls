import argparse
import numpy as np
import matplotlib.pyplot as plt
from itertools import chain

from rust_interface import (
    compile_controls,
    compute_state_measurement,
)


state_measurement_labels = [
    [["vision_x_meas"],              ["vision_y_meas"],            ["vision_theta_meas"]],
    [["encoder_xd_meas"],            ["encoder_yd_meas"],          ["encoder_thetad_meas", "gyro_thetad_meas"]],
]

# 
def compute_all_state_measurements(state_estimates, sensor_measurements, param_json=None):
    """
    compute_all_state_measurements computes the state measurements for all time steps given the state estimates and sensor measurements.

    :param state_estimates: t x 6 array of state estimates (x, y, theta, xd, yd, thetad)
    :type state_estimates: np.ndarray
    :param sensor_measurements: t x 8 array of sensor measurements (vision_x, vision_y, vision_theta, encoder_vfl, encoder_vbl, encoder_vbr, encoder_vfr, gyro_thetad)
    :type sensor_measurements: np.ndarray
    :param param_json: Optional path to a JSON file containing RobotModel parameters
    :type param_json: str or None
    :return: dictionary of state measurement time series (vision_x_meas, vision_y_meas, vision_theta_meas, encoder_xd_meas, encoder_yd_meas, encoder_thetad_meas, gyro_thetad_meas)
    :rtype: dict of np.ndarray
    """
    state_measurements = {}
    for i in range(state_estimates.shape[0]):
        state_meas_csv_data = compute_state_measurement(state_estimates[i], sensor_measurements[i], param_json=param_json)
        for measurement_label in chain.from_iterable(chain.from_iterable(state_measurement_labels)):
            meas = state_meas_csv_data[measurement_label].values[0]
            if measurement_label in state_measurements:
                state_measurements[measurement_label].append(meas)
            else:
                state_measurements[measurement_label] = [meas]
    return {k: np.array(v) for k, v in state_measurements.items()}

def plot_state_meas(telemetry, param_json=None):
    # Construct sensor measurement array from time sequence of ExtendedTelemetry, t x n_meas, n_meas = 8 (vision x,y,theta + 4 encoders + gyro thetad)
    sensor_meas = np.stack([
        telemetry["vision_pose"][:,0], 
        telemetry["vision_pose"][:,1], 
        telemetry["vision_pose"][:,2], 
        telemetry['front_left_motor__velocity_telemetry__wheel_vel_rads'], 
        telemetry['back_left_motor__velocity_telemetry__wheel_vel_rads'], 
        telemetry['back_right_motor__velocity_telemetry__wheel_vel_rads'], 
        telemetry['front_right_motor__velocity_telemetry__wheel_vel_rads'], 
        telemetry['imu_gyro'][:,2], 
    ], axis=1)

    t = telemetry['timestamp_us'] * 1e-6

    # t x 6 (x, y, theta, xd, yd, thetad)
    state_pred = np.append(
        telemetry['kf_body_pose_prediction'], 
        telemetry['kf_body_twist_prediction'], 
        axis=1
    )
    # t x 6 (x, y, theta, xd, yd, thetad)
    state_est = np.append(
        telemetry['kf_body_pose_estimate'], 
        telemetry['kf_body_twist_estimate'], 
        axis=1
    )
    # dictionary with labels as keys and t len arrays as values
    state_meas = compute_all_state_measurements(state_est, sensor_meas, param_json=param_json)
    # t x 3 (xdd, ydd, thetadd)
    cmd = telemetry['body_accel_u']
    # t x 2 (xdd, ydd)
    cmd_meas = telemetry['imu_accel'][:, :2]


    ##### Configure the figure #####
    fig, axs = plt.subplots(3, 3, figsize=(12, 10), sharex=True)
    plt.tight_layout()
    alpha = 0.5
    # X and Y dimensions share y axes as they use the same units
    axs[0, 1].sharey(axs[0, 0])
    axs[1, 1].sharey(axs[1, 0])
    axs[2, 1].sharey(axs[2, 0])

    ##### Label the axes #####
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

    ##### Plot state prediction, estimate and state measurements #####
    for i in range(2):
        for j in range(3):
            ax = axs[i, j]
            ax.plot(t, state_pred[:, 3*i + j], label=f'predicted', alpha=alpha)
            ax.plot(t, state_est[:, 3*i + j], label=f'estimate', color="red", alpha=alpha)

            for meas_label in state_measurement_labels[i][j]:
                if "vision" in meas_label:
                    mask = telemetry["vision_update"].astype(bool)
                    t_meas = telemetry['timestamp_us'][mask] * 1e-6
                    m_meas = state_meas[meas_label][mask]
                    ax.plot(t_meas, m_meas, label=f'{meas_label}', alpha=alpha)
                else:
                    ax.plot(t, state_meas[meas_label], label=f'{meas_label}', alpha=alpha)

    ##### Plot onboard computed trajectory state #####
    i = 0
    for j in range(3):
        ax = axs[i, j]
        ax.plot(t, telemetry["body_traj_pose"][:, j], label=f'trajectory', alpha=alpha)
        ax.legend()
    i = 1
    for j in range(3):
        ax = axs[i, j]
        ax.plot(t, telemetry["body_traj_twist"][:, j], label=f'trajectory', alpha=alpha)
        ax.legend()

    ##### Plot body controller command  #####
    i = 2
    for j in range(3):
        ax = axs[i, j]
        ax.plot(t, cmd[:, j], label=f'firmware_cmd', alpha=alpha)
        # if j < 2:  # only plot accel measurements for x and y, not theta
            # ax.plot(t, cmd_meas[:, j], label=f'accelerometer_measured', alpha=alpha)
        ax.legend()

    ##### Plot software command #####
    # Use the first packet to determine which control type is active
    i = 0
    for j in range(3):
        ax = axs[i, j]
        mask = telemetry["body_pose_control_enabled"].astype(bool)
        if any(mask):
            ax.plot(t[mask], telemetry["body_cmd"][:, j][mask], label='software_cmd', alpha=alpha)
    i = 1
    for j in range(3):
        ax = axs[i, j]
        mask = telemetry["body_twist_control_enabled"].astype(bool)
        if any(mask):
            ax.plot(t[mask], telemetry["body_cmd"][:, j][mask], label='software_cmd', alpha=alpha)
    i = 2
    for j in range(3):
        ax = axs[i, j]
        mask = telemetry["body_accel_control_enabled"].astype(bool)
        if any(mask):
            ax.plot(t[mask], telemetry["body_cmd"][:, j][mask], label='software_cmd', alpha=alpha)


    ##### Add legends and make sure t is labeled for all subplots #####
    for ax in axs.flat:
        ax.tick_params(labelbottom=True)
        ax.legend()

    return fig, axs, t, state_est


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize telemetry data")
    parser.add_argument("--telemetry", type=str, required=True, help="Path to the telemetry NPZ file")
    parser.add_argument("--param-json", type=str, default=None, help="Path to a JSON file containing RobotModel parameters")
    args = parser.parse_args()
    
    compile_controls()
    telemetry = np.load(args.telemetry)
    fig, axs, t, state_est = plot_state_meas(telemetry, param_json=args.param_json)

    # Toggle trajectory overlay with the space key.
    try:
        from trajectory_overlay import TrajectoryOverlay
        overlay = TrajectoryOverlay(fig, axs, t, state_est, telemetry, param_json=args.param_json)
        fig.canvas.mpl_connect("key_press_event",
                               lambda event: overlay.toggle() if event.key == " " else None)
        print("[kalman_visualize] Press Space to toggle trajectory overlay.")
    except Exception as exc:
        print(f"Failed to set up trajectory overlay: {exc}")

    plt.show()
