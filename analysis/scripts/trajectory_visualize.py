import argparse
import matplotlib.pyplot as plt

from rust_interface import compile_controls, compute_trajectory


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize bang-bang trajectory")
    parser.add_argument("--param-json", type=str, default=None,
                        help="Path to JSON file with trajectory parameters")
    args = parser.parse_args()

    compile_controls()
    init_state = [0.0, 0.0, 3.0, 0.0, 0.0, 0.0]
    target_pose = [0.0, 0.0, -3.0]
    # Compute trajectory
    traj_data = compute_trajectory(init_state, target_pose, param_json=args.param_json)
    fig, axs = plt.subplots(3, 3, figsize=(15, 10), sharex=True)
    for ax_col, dim_col in zip(axs.transpose(), [[('x', '(m)'), ('xd', '(m/s)'), ('xdd', '(m/s²)')], [('y', '(m)'), ('yd', '(m/s)'), ('ydd', '(m/s²)')], [('theta', '(rad)'), ('thetad', '(rad/s)'), ('thetadd', '(rad/s²)')]]):
        ax_col[0].set_title(f'Trajectory {dim_col[0][0][0].upper() + dim_col[0][0][1:]} Dimension')
        ax_col[-1].set_xlabel('Time (s)', fontsize=14)
        for ax, dim in zip(ax_col, dim_col):
            ax.plot(traj_data['time'], traj_data[dim[0]], label=dim[0])
            ax.set_ylabel(f"{dim[0]} {dim[1]}", fontsize=14)
    plt.tight_layout()
    plt.show()
