import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from rust_interface import compile_controls, compute_trajectory

class TelemVisualizer:
    def __init__(self, npz_path):
        self.data = np.load(npz_path)
        self.idx = 0
        self.max_idx = len(self.data['timestamp_us']) - 1
        
        if self.max_idx <= 0:
            raise ValueError("No telemetry data found in the provided NPZ file.")
        
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        
        # Robot visualization
        self.robot_body = Circle((0, 0), 0.1, color='blue', alpha=0.3)
        self.heading_line, = self.ax.plot([], [], 'k-', lw=2)
        self.trajectory_line, = self.ax.plot([], [], 'c-', lw=1.5, alpha=0.9, label='Planned Trajectory')
        self.ax.add_patch(self.robot_body)
        
        # Velocity and acceleration arrows
        self.vel_arrow = self.ax.quiver(0, 0, 0, 0, color='green', scale=2, label='Velocity')
        self.accel_arrow = self.ax.quiver(0, 0, 0, 0, color='blue', scale=10, label='Acceleration')
        
        # Text display
        self.info_text = self.ax.text(1.0, 0.75, "", fontsize=9, family='monospace',
                                       bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.set_xlabel('X Position (m)', fontsize=14)
        self.ax.set_ylabel('Y Position (m)', fontsize=14)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc='lower right')
        self.ax.set_title('Telemetry Visualizer')
        
        self.update_display()
    
    def on_key(self, event):
        if event.key == 'right':
            self.idx = min(self.idx + 1, self.max_idx)
        elif event.key == 'left':
            self.idx = max(self.idx - 1, 0)
        elif event.key == 'up':
            self.idx = min(self.idx + 100, self.max_idx)
        elif event.key == 'down':
            self.idx = max(self.idx - 100, 0)
        elif event.key == 'home':
            self.idx = 0
        elif event.key == 'end':
            self.idx = self.max_idx
        elif event.key == 'q':
            plt.close()
            return
        
        self.update_display()
    
    def update_display(self):
        t = self.data['timestamp_us'][self.idx] * 1e-6

        # Extract body command
        cmd = self.data['body_control_telemetry__body_cmd'][self.idx]
        
        # Extract pose (x, y, theta) from body pose estimate
        pose = self.data['body_control_telemetry__kf_body_pose_estimate'][self.idx]
        x, y, theta = pose[0], pose[1], pose[2]
        
        # Extract twist (xd, yd, thetad)
        twist = self.data['body_control_telemetry__kf_body_twist_estimate'][self.idx]
        xd, yd, thetad = twist[0], twist[1], twist[2]

        # Extract accelerations
        accel = self.data['body_control_telemetry__body_accel_u'][self.idx]
        xdd, ydd, thetadd = accel[0], accel[1], accel[2]
        
        # Extract wheel velocities
        wv_fl = self.data['front_left_motor__velocity_telemetry__wheel_vel_rads'][self.idx]
        wv_bl = self.data['back_left_motor__velocity_telemetry__wheel_vel_rads'][self.idx]
        wv_br = self.data['back_right_motor__velocity_telemetry__wheel_vel_rads'][self.idx]
        wv_fr = self.data['front_right_motor__velocity_telemetry__wheel_vel_rads'][self.idx]
        
        # Extract wheel torques
        wt_fl = np.mean(self.data['front_left_motor__current_telemetry__current_samples_ma'][self.idx])
        wt_bl = np.mean(self.data['back_left_motor__current_telemetry__current_samples_ma'][self.idx])
        wt_br = np.mean(self.data['back_right_motor__current_telemetry__current_samples_ma'][self.idx])
        wt_fr = np.mean(self.data['front_right_motor__current_telemetry__current_samples_ma'][self.idx])

        if self.data['body_control_telemetry__body_control_mode'][self.idx] == 1:  # BCM_GLOBAL_POSITION
            # Compute trajectory
            traj_data = compute_trajectory([x, y, theta, xd, yd, thetad], self.data['body_control_telemetry__body_cmd'][self.idx].tolist())
            
            # Update trajectory line
            self.trajectory_line.set_data(traj_data['x'], traj_data['y'])
        
        # Update robot position
        self.robot_body.center = (x, y)
        
        # Update heading line
        heading_scale = 0.1
        hx = x + heading_scale * np.cos(theta)
        hy = y + heading_scale * np.sin(theta)
        self.heading_line.set_data([x, hx], [y, hy])
        
        # Update arrows
        self.vel_arrow.set_offsets([x, y])
        self.vel_arrow.set_UVC(xd, yd)
        
        self.accel_arrow.set_offsets([x, y])
        self.accel_arrow.set_UVC(xdd, ydd)
        
        # Update info text
        info = (f"Frame: {self.idx}/{self.max_idx}  Time: {t:.2f}s\n"
                f"Pos: x={x:.3f}  y={y:.3f}  θ={theta:.3f}rad\n"
                f"Vel: xd={xd:.3f}  yd={yd:.3f}  ω={thetad:.3f}rad/s\n"
                f"Wheel Vels (rad/s):\n"
                f" FL: {wv_fl:>6.2f}  FR: {wv_fr:>6.2f}\n"
                f" BL: {wv_bl:>6.2f}  BR: {wv_br:>6.2f}\n"
                f"Wheel Torques (Nm):\n"
                f" FL: {wt_fl:>6.2f}  FR: {wt_fr:>6.2f}\n"
                f" BL: {wt_bl:>6.2f}  BR: {wt_br:>6.2f}\n"
                f"[←→] step 1 frame  [↑↓] skip 100 frames  [Home/End] first/last  [q] quit")
        self.info_text.set_text(info)
        
        self.fig.canvas.draw_idle()
    
    def show(self):
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize telemetry data")
    parser.add_argument("--telemetry", type=str, required=True, help="Path to the telemetry NPZ file")
    args = parser.parse_args()
    
    compile_controls()
    viz = TelemVisualizer(args.telemetry)
    viz.show()
