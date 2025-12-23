import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import sys


data = None

def create_plots():
    ############### Static 3x3 plot: pos/vel/acc for x, y, theta ###############
    plot_fig, axes = plt.subplots(3, 3, figsize=(12, 9), sharex=True)

    # Column 0: X
    axes[0, 0].plot(data['time'], data['x'], color='C0')
    axes[0, 0].set_title('X Position')
    axes[0, 0].grid(True)

    axes[1, 0].plot(data['time'], data['vx'], color='C1')
    axes[1, 0].set_title('X Velocity')
    axes[1, 0].grid(True)

    axes[2, 0].plot(data['time'], data['ax'], color='C2')
    axes[2, 0].set_title('X Acceleration')
    axes[2, 0].grid(True)

    # Column 1: Y
    axes[0, 1].plot(data['time'], data['y'], color='C0')
    axes[0, 1].set_title('Y Position')
    axes[0, 1].grid(True)

    axes[1, 1].plot(data['time'], data['vy'], color='C1')
    axes[1, 1].set_title('Y Velocity')
    axes[1, 1].grid(True)

    axes[2, 1].plot(data['time'], data['ay'], color='C2')
    axes[2, 1].set_title('Y Acceleration')
    axes[2, 1].grid(True)

    # Column 2: Theta
    axes[0, 2].plot(data['time'], data['theta'], color='C0')
    axes[0, 2].set_title('Theta (rad)')
    axes[0, 2].grid(True)

    axes[1, 2].plot(data['time'], data['vtheta'], color='C1')
    axes[1, 2].set_title('Theta Velocity (rad/s)')
    axes[1, 2].grid(True)

    axes[2, 2].plot(data['time'], data['atheta'], color='C2')
    axes[2, 2].set_title('Theta Acceleration (rad/s^2)')
    axes[2, 2].grid(True)

    # Label x-axis on bottom row
    for ax in axes[2, :]:
        ax.set_xlabel('Time (s)')

    plot_fig.tight_layout()
    plot_fig.savefig('sim_plots.png', dpi=200)
    print('Saved static 3x3 plot as sim_plots.png')

def create_animation():
    frame_fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(-0.5, 2.0)
    ax.set_ylim(-0.5, 2.0)
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.6)

    # Robot body and direction marker
    robot_body = plt.Circle((0, 0), 0.1, color='blue', alpha=0.3)
    heading_line, = ax.plot([], [], 'k-', lw=2)
    ax.add_patch(robot_body)

    # Vectors (Velocity = Green, Acceleration = Blue)
    vel_arrow = ax.quiver(0, 0, 0, 0, color='green', scale=5, label='Velocity')
    accel_arrow = ax.quiver(0, 0, 0, 0, color='blue', scale=10, label='Accel')

    # Telemetry Text
    telemetry_box = ax.text(2.0, 1.8, "", fontsize=9, family='monospace', 
                            bbox=dict(facecolor='white', alpha=0.8))

    def update(i):
        row = data.iloc[i]
        
        # Update Robot Position
        robot_body.center = (row['x'], row['y'])
        
        # Update Heading line
        hx = row['x'] + 0.1 * np.cos(row['theta'])
        hy = row['y'] + 0.1 * np.sin(row['theta'])
        heading_line.set_data([row['x'], hx], [row['y'], hy])
        
        # Update Vectors
        vel_arrow.set_offsets([row['x'], row['y']])
        vel_arrow.set_UVC(row['vx'], row['vy'])
        
        accel_arrow.set_offsets([row['x'], row['y']])
        accel_arrow.set_UVC(row['ax'], row['ay'])
        
        # Update Telemetry HUD
        txt = (f"Time: {row['time']:.2f}s\n"
            f"Wheel Vels (rad/s):\n"
            f" FL: {row['wv1']:>6.2f}  FR: {row['wv4']:>6.2f}\n"
            f" BL: {row['wv2']:>6.2f}  BR: {row['wv3']:>6.2f}\n"
            f"Wheel Torques (Nm):\n"
            f" FL: {row['wt1']:>6.2f}  FR: {row['wt4']:>6.2f}\n"
            f" BL: {row['wt2']:>6.2f}  BR: {row['wt3']:>6.2f}")
        telemetry_box.set_text(txt)
        
        return robot_body, heading_line, vel_arrow, accel_arrow, telemetry_box

    downsample_factor = 1000
    ani = animation.FuncAnimation(frame_fig, update, frames=range(0, len(data), downsample_factor), interval=downsample_factor)

    plt.legend(loc='lower left')
    plt.title("Robot Trajectory Analysis Simulation")

    # Save video
    ani.save('sim_animation.mp4', writer='ffmpeg', fps=50)
    print("Video saved as sim_animation.mp4")


if __name__ == "__main__":
    # Load simulation data
    # data = pd.read_csv(sys.stdin)
    data = pd.read_csv('sim_data.csv')  # For testing purposes
    create_plots()
    create_animation()
