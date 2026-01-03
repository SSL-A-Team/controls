from pathlib import Path
import subprocess
from io import StringIO
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Circle


CONTROLS_REPO_PATH = next(p for p in Path(__file__).resolve().parents if p.name == "controls")
COMPUTE_TRAJECTORY_BIN_PATH = next(CONTROLS_REPO_PATH.rglob("target/release/compute_trajectory"))
TELEM_NPZ_PATH = next(CONTROLS_REPO_PATH.rglob("**/telemetry.npz"))

def compile_compute_trajectory():
    build_cmd = ["cargo", "build", "--release", "--bin", "compute_trajectory"]
    subprocess.run(build_cmd, cwd=CONTROLS_REPO_PATH, check=True)
    print("Compiled compute_trajectory binary.")

def compute_trajectory(init_state: list, target_pose: list):
    if len(init_state) != 6:
        raise ValueError("init_state must have 6 elements: [x, y, theta, vx, vy, vtheta]")
    if len(target_pose) != 3:
        raise ValueError("target_pose must have 3 elements: [x, y, theta]")
    init_state_str = " ".join(str(v) for v in init_state)
    target_pose_str = " ".join(str(v) for v in target_pose)
    
    p = subprocess.Popen(
        str(COMPUTE_TRAJECTORY_BIN_PATH),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        stdin=subprocess.PIPE,
        text=True
    )
    
    # Write input and close stdin to signal EOF
    p.stdin.write(init_state_str + '\n')
    p.stdin.write(target_pose_str + '\n')
    
    # Read output
    stdout, stderr = p.communicate()
    
    if p.returncode != 0:
        raise RuntimeError(f"Trajectory computation failed: {stderr}")
    
    # Parse CSV output
    traj_data = pd.read_csv(StringIO(stdout))
    return traj_data

class TelemVisualizer:
    def __init__(self, npz_path):
        self.data = np.load(npz_path)
        self.idx = 0
        self.max_idx = len(self.data['t']) - 1
        
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
        t = self.data['t'][self.idx]
        
        # Extract pose (x, y, theta) from body pose estimate
        pose = self.data['kf_body_pose_estimate'][self.idx]
        x, y, theta = pose[0], pose[1], pose[2]
        
        # Extract twist (vx, vy, vtheta)
        twist = self.data['kf_body_twist_estimate'][self.idx]
        vx, vy, vtheta = twist[0], twist[1], twist[2]

        # Extract command
        cmd = self.data['body_cmd'][self.idx]
        cmd_x, cmd_y, cmd_theta = cmd[0], cmd[1], cmd[2]

        # Extract accelerations
        accel = self.data['body_accel_u'][self.idx]
        ax, ay, atheta = accel[0], accel[1], accel[2]
        
        # Extract wheel velocities
        wv_fl = self.data['wheel_velocity_fl'][self.idx]
        wv_bl = self.data['wheel_velocity_bl'][self.idx]
        wv_br = self.data['wheel_velocity_br'][self.idx]
        wv_fr = self.data['wheel_velocity_fr'][self.idx]
        
        # Extract wheel torques
        wt = self.data['wheel_torque_u'][self.idx]
        wt_fl, wt_bl, wt_br, wt_fr = wt[0], wt[1], wt[2], wt[3]

        # Compute trajectory
        traj_data = compute_trajectory([x, y, theta, vx, vy, vtheta], [cmd_x, cmd_y, cmd_theta])
        
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
        self.vel_arrow.set_UVC(vx, vy)
        
        self.accel_arrow.set_offsets([x, y])
        self.accel_arrow.set_UVC(ax, ay)
        
        # Update info text
        info = (f"Frame: {self.idx}/{self.max_idx}  Time: {t:.2f}s\n"
                f"Pos: x={x:.3f}  y={y:.3f}  θ={theta:.3f}rad\n"
                f"Vel: vx={vx:.3f}  vy={vy:.3f}  ω={vtheta:.3f}rad/s\n"
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
    compile_compute_trajectory()
    viz = TelemVisualizer(TELEM_NPZ_PATH)
    viz.show()
