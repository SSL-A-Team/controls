# Controls Repository — Copilot Context

## Overview

This is the shared controls library for the A-Team SSL (Small Size League) robotics team. It provides state estimation (Kalman filter), kinematic transforms, bang-bang trajectory planning, and a physical robot model — all written in Rust (`no_std`-compatible) with C FFI and Python ctypes bindings. It also includes Python analysis/visualization tooling for telemetry and parameter tuning.

This library is consumed as a **git submodule** by the [firmware](https://github.com/SSL-A-Team/firmware) repository (at `firmware/controls/`), where it runs on an STM32H743 at 1 kHz inside the motion control loop. The Python bindings and analysis tools are used offline for telemetry visualization, trajectory debugging, and parameter tuning.

The companion **software** repository (ROS 2 Jazzy) sends motion commands to robots via WiFi. The companion **firmware** repository implements a multi-mode position/velocity/acceleration motion controller using this library's `RobotModel` and `BangBangTraj3D`.

**Build (Rust):** `cargo build --workspace` (or `nix develop` + `cargo build`).
**Test (Rust):** `cargo test --workspace`.
**Build + Test (C++):** `cmake -B build . && cmake --build build && cd build && ctest`.
**Python setup:** `uv sync && source .venv/bin/activate`.

---

## Repository Structure

```
controls/
├── ateam-controls/         # Core Rust library (no_std): KF, trajectory, robot model
│   └── src/
│       ├── lib.rs                  # Type aliases, z_rotation_mat, wrap_angle, ControlsError
│       ├── robot_model.rs          # RobotModel: KF, kinematic transforms, friction model
│       ├── bangbang_trajectory.rs  # BangBangTraj3D: time-optimal trajectory planning
│       ├── defaults.rs             # All default parameter constants
│       └── ctypes.rs               # C-compatible vector/matrix types (repr(C))
├── ateam-controls-c/       # C FFI wrapper crate (extern "C", #[unsafe(no_mangle)])
│   ├── src/lib.rs                  # All FFI function implementations
│   └── include/ateam_controls/
│       └── ateam_controls.h        # Public C header (source of truth for FFI API)
├── ateam-controls-py/      # Installable Python package with ctypes bindings
│   ├── pyproject.toml              # Hatchling build, package name: ateam-controls-py
│   └── ateam_controls/
│       ├── __init__.py             # Public re-exports
│       └── _bindings.py            # Hand-maintained ctypes wrapper of ateam_controls.h
├── analysis/               # Python telemetry visualization and ROS tooling
│   ├── telem_visualize.py          # CLI entry point: body state + wheel plots
│   ├── params.py                   # PARAM_MAP: JSON ↔ ctypes structs ↔ firmware param IDs
│   ├── build.py                    # Cargo build helper (release mode)
│   ├── visualization/
│   │   ├── body.py                 # 3×3 grid plot (pos/vel/accel × x/y/theta)
│   │   ├── overlays.py             # TrajectoryOverlay + VelocityOverlay (interactive)
│   │   └── wheels.py               # 2×2 wheel current/velocity plots
│   └── ros_scripts/
│       ├── telem_bag2np.py         # ROS2 bag (mcap) → NumPy NPZ conversion
│       ├── upload_params.py        # Upload JSON params to robot firmware via ROS2 services
│       ├── signal_input.py         # rclpy signal generator (pulse/sine/step/chirp) on pos/vel modes
│       ├── controller_tune.py      # One-shot pipeline: upload → bag → signal_input → convert → visualize
│       └── record_and_visualize.sh # Quick bag record + convert + visualize
├── cpp_tests/              # GTest tests for C FFI bindings
│   ├── test_bangbang.cpp
│   └── test_robot_model.cpp
├── Cargo.toml              # Workspace: members = [ateam-controls, ateam-controls-c]
├── pyproject.toml          # uv workspace: deps = ateam-controls-py, matplotlib, numpy, pandas
├── CMakeLists.txt          # CMake build: invokes cargo, exports static lib + header
└── flake.nix               # Nix development environment
```

### Workspace Layout

- **Rust workspace** (`Cargo.toml`): members are `ateam-controls` and `ateam-controls-c`.
- **Python workspace** (`pyproject.toml`): managed by `uv`, includes `ateam-controls-py` as a workspace member. Dependencies: numpy, matplotlib, pandas, pyyaml.
- **CMake** (`CMakeLists.txt`): drives `cargo build`, exports `ateam_controls` as a static imported library with the C header. Used by firmware's CMake build and by `cpp_tests/`.

---

## Core Library (`ateam-controls`)

### Design Principles

- **`no_std` by default** — the library runs on bare-metal STM32H743. The `std` feature is only enabled for the C FFI crate (`ateam-controls-c`).
- **`libm` for math** — uses `libm::{sinf, cosf, atan2f, sqrtf}` instead of std math.
- **`nalgebra` with `libm`** — all linear algebra uses `nalgebra` with `default-features = false` and `features = ["libm", "macros"]`.
- **Optional features:** `serde` (for serialization), `defmt` (for embedded logging on the firmware target).

### RobotModel (`robot_model.rs`)

The central struct combining state estimation and kinematics.

#### State Vector (6 states)
```
x = [x, y, θ, vx, vy, vθ]
```
- Position in global frame (meters, radians)
- Velocity in global frame (m/s, rad/s)

#### Measurement Vector (8 measurements)
```
z = [vision_x, vision_y, vision_θ, wheel_vel_FL, wheel_vel_BL, wheel_vel_BR, wheel_vel_FR, gyro_z]
```
- Wheel ordering: Front-Left, Back-Left, Back-Right, Front-Right

#### Key Fields
- `x: Vector6f` — state estimate
- `p: Matrix6f` — state covariance
- `a: Matrix6f` — state transition (constant-velocity model, integrates velocity → position over dt)
- `b: Matrix6x3f` — input matrix (maps acceleration → velocity)
- `h: Matrix8x6f` — measurement matrix (dynamically updated each tick via `update_h_transform`)
- `m: Matrix3x4f` — wheel geometry matrix (body twist → wheel velocities)
- `m_inv: Matrix4x3f` — pseudo-inverse of `m`
- `i: Matrix3f` — inertia matrix `[mass, mass, iz]`
- `kf_q: Matrix6f` — process noise covariance
- `kf_r: Matrix8f` — measurement noise covariance

#### Kalman Filter Methods
- `new(kf_dt, kf_params, phys_params)` — construct with timestep and parameter structs
- `kf_predict(accel)` — propagate state: `x = Ax + Bu`, wrap θ, propagate P
- `kf_update(z, mask_vision, mask_encoder, mask_gyro)` — standard Kalman update with innovation and gain
- `kf_set_pose(pose)` — snap position states to vision, collapse position covariance
- `update_h_transform(θ, masks)` — rebuild H matrix with sensor masks (zero rows for masked sensors)

#### Kinematic Transforms (all θ-dependent)
```
transform_twist2wheel(θ) = Mᵀ × R_z(-θ) / r     — body twist → wheel angular velocities
transform_wheel2twist(θ) = r × R_z(θ) × M_inv_pseudoᵀ  — wheel velocities → body twist
transform_accel2wheel(θ) = r × M_inv × I × R_z(-θ)      — body accel → wheel torques
transform_wheel2accel(θ) = R_z(θ) × I_inv × M / r        — wheel torques → body accel
```

#### Friction Model

Per local body axis (the firmware rotates global twist into local frame
before calling, and rotates the resulting force back to global):

```
Fx_local = -c_visc_x · vx_local - c_coul_x · sign(vx_local)
Fy_local = -c_visc_y · vy_local - c_coul_y · sign(vy_local)
Tz       = -c_visc_ang · v_theta - c_coul_ang · sign(v_theta)
```

Method: `compute_friction_force(body_twist) → Vector3f [Fx, Fy, Tz]` —
inputs and outputs are in the robot's local frame.

#### Motor Model
```
wheel_currents = wheel_torques / motor_torque_constant / motor_efficiency_factor
```
Method: `torques_to_currents(torques) → Vector4f`

### Parameter Structs

#### `KalmanFilterParams`
| Field | Default | Description |
|-------|---------|-------------|
| `process_noise_std_pos_linear` | 0.01 | Position process noise std |
| `process_noise_std_pos_angular` | 0.05 | Angular position process noise std |
| `process_noise_std_vel_linear` | 0.02 | Velocity process noise std |
| `process_noise_std_vel_angular` | 0.1 | Angular velocity process noise std |
| `measurement_noise_std_vision_pos_linear` | 1.0 | Vision position measurement noise std |
| `measurement_noise_std_vision_pos_angular` | 3.14 | Vision angle measurement noise std |
| `measurement_noise_std_encoder_vel_angular` | 50.0 | Encoder velocity measurement noise std |
| `measurement_noise_std_gyro_vel_angular` | 0.015 | Gyro measurement noise std |
| `max_pos_linear` | 64.0 | Initial covariance bound (position) |
| `max_pos_angular` | 3.14 | Initial covariance bound (angle) |
| `max_vel_linear` | 3.0 | Initial covariance bound (velocity) |
| `max_vel_angular` | 3π | Initial covariance bound (angular velocity) |

#### `RobotPhysicalParams`
| Field | Default | Description |
|-------|---------|-------------|
| `alpha` | π/6 (30°) | Front wheel angle from longitudinal axis |
| `beta` | π/4 (45°) | Back wheel angle from longitudinal axis |
| `l` | 0.0814 m | Wheel distance from center |
| `r` | 0.030 m | Wheel radius |
| `mass` | 2.7 kg | Robot mass |
| `iz` | 0.008 kg·m² | Moment of inertia about z-axis |
| `motor_torque_constant` | 0.0335 Nm/A | Motor torque constant |
| `motor_efficiency_factor` | 13.0 | Motor efficiency factor |
| `coulomb_friction_coefficient_linear_x` | 2.058 | Coulomb friction, local x (forward/backward) |
| `coulomb_friction_coefficient_linear_y` | 2.058 | Coulomb friction, local y (strafe) |
| `coulomb_friction_coefficient_angular`  | 0.05  | Coulomb friction (angular) |
| `viscous_friction_coefficient_linear_x` | 5.0   | Viscous friction, local x |
| `viscous_friction_coefficient_linear_y` | 5.0   | Viscous friction, local y |
| `viscous_friction_coefficient_angular`  | 0.0063 | Viscous friction (angular) |

#### `TrajectoryParams`
| Field | Default | Description |
|-------|---------|-------------|
| `max_vel_linear` | 3.0 m/s | Maximum linear velocity |
| `max_vel_angular` | 3π rad/s | Maximum angular velocity |
| `max_accel_linear` | 2.0 m/s² | Maximum linear acceleration |
| `max_accel_angular` | 2π rad/s² | Maximum angular acceleration |

### Bang-Bang Trajectory Planner (`bangbang_trajectory.rs`)

#### `BangBangTraj1D`
Each 1D trajectory has 3 acceleration segments with 4 time boundaries:
- `sdd1` (t1→t2), `sdd2` (t2→t3), `sdd3` (t3→t4)
- Profile types: **triangular** (accel → decel, no coast) or **trapezoidal** (accel → coast → decel)

#### `BangBangTraj3D`
Three independent `BangBangTraj1D` for x, y, and θ.

- **`from_target_pose(init_state, target_pose, params)`** — computes time-optimal trajectory to reach a target pose. Iteratively searches angle α to synchronize x and y completion times. θ uses shortest angular path via `wrap_angle`.
- **`from_target_twist(init_twist, target_twist, params)`** — computes constant acceleration to reach target velocity. Simpler than pose: proportional x/y acceleration split by magnitude.
- **`time_shift(dt)`** — shifts all segment times forward
- **`end_time()`** — max of x/y/θ segment end times
- **`state_at(current_state, current_time, t)`** — evaluate pose + twist at time t
- **`accel_at(t)`** — piecewise-constant acceleration command at time t

### Error Handling

```rust
enum ControlsError {
    InvalidInput = -1,     // Bad parameter values
    SingularMatrix = -2,   // Matrix inversion failed
    NoSolution = -3,       // Trajectory solver failed
    InvalidTime = -4,      // Time before trajectory start
    ExceedsLimits = -5,    // Target exceeds configured limits
}
```

All functions return `Result<T, ControlsError>`. In the C FFI, errors are returned as `i32` (0 = success).

### C Types (`ctypes.rs`)

`#[repr(C)]` wrapper structs with `From` trait conversions to/from nalgebra types:
- `Vector3C`, `Vector4C`, `Vector6C`, `Vector8C`
- `Matrix3C`, `Matrix3x4C`, `Matrix4x3C`

All use column-major storage (matching nalgebra's memory layout).

---

## C FFI Layer (`ateam-controls-c`)

Exposes all `RobotModel` and trajectory functions as `extern "C"` with `#[unsafe(no_mangle)]`.

### Naming Convention
All C functions are prefixed with `ateam_controls_`:
- `ateam_controls_default_{kf,phys,traj}_params()` — default parameter constructors
- `ateam_controls_robot_model_{new,free,get_state,set_state,...}()` — robot model lifecycle and operations
- `ateam_controls_traj_{from_target_pose,from_target_twist,state_at,accel_at,...}()` — trajectory operations

### Library Output
- **Static library:** `target/{debug,release}/libateam_controls_c.a` (linked by firmware CMake)
- **Shared library:** `target/{debug,release}/libateam_controls_c.so` (loaded by Python ctypes)

### C Header
The public API is defined in `ateam-controls-c/include/ateam_controls/ateam_controls.h`. This is the source of truth for the FFI interface. All struct types have `_t` suffixed typedefs. `RobotModel` is an opaque pointer type.

---

## Python Bindings (`ateam-controls-py`)

### Package Structure
- Installable via `uv pip install -e ./ateam-controls-py` or as a uv workspace member
- Import as `from ateam_controls import ...`
- The `_bindings.py` file is a hand-maintained 1:1 ctypes wrapper of `ateam_controls.h`

### Binding Style
- Wrapper functions strip the `ateam_controls_` prefix (e.g., `robot_model_new`, `traj_from_target_pose`)
- Out-pointer parameters are handled internally (allocated, populated, returned)
- `int32_t` return codes are checked via `_check_rc()`, raising `ControlsError` on failure
- The shared library is loaded from `{repo_root}/target/release/libateam_controls_c.so`

### Updating Bindings
There is no auto-generation script. When the C header changes, manually update `_bindings.py` to match. Keep the 1:1 mapping style: one Python wrapper per C function, same parameter types, same semantics.

---

## Analysis Tooling (`analysis/`)

The `analysis/` directory is a flat Python directory (not a package). It requires the virtual environment to be active and the Rust library to be built.

### Telemetry Visualization (`telem_visualize.py`)

Main entry point for offline telemetry analysis:
```bash
cd analysis
python telem_visualize.py -t data/telemetry/robot_telemetry.npz [-p data/robot_params.json]
```

Opens three matplotlib windows:
1. **Body state (3×3 grid):** position/velocity/acceleration × x/y/theta with traces for predicted, estimated, measured (vision/encoder/gyro), commanded, trajectory, and software command
2. **Wheel current (2×2):** commanded vs measured current per wheel
3. **Wheel velocity (2×2):** commanded vs measured velocity per wheel

Press **Space** to toggle interactive overlays:
- **TrajectoryOverlay:** click to compute bang-bang trajectory from that time point
- **VelocityOverlay:** sliding-window vision-derived velocity

### Parameter System (`params.py`)

`PARAM_MAP` is the source of truth mapping JSON parameter keys to:
1. Firmware `ParameterName` enum IDs (for uploading to robots via ROS2 services)
2. ctypes struct fields (for visualization computations via FFI)

| JSON Key | Param ID | Target Struct |
|----------|----------|---------------|
| `KF_PROCESS_STD` | 0 | `KalmanFilterParams` |
| `KF_MEASUREMENT_STD` | 1 | `KalmanFilterParams` |
| `KF_MAX_STATE` | 2 | `KalmanFilterParams` |
| `PHYS_WHEEL` | 3 | `RobotPhysicalParams` |
| `PHYS_INERTIA` | 4 | `RobotPhysicalParams` |
| `PHYS_MOTOR_MODEL` | 5 | `RobotPhysicalParams` |
| `PHYS_FRICTION_MODEL` | 6 | (upload only) |
| `FRICTION_COMP_GATING` | 7 | (upload only) — vec4 `[lin_vel_thresh, lin_accel_thresh, ang_vel_thresh, ang_accel_thresh]` |
| `POSE_CONTROL_GAIN` | 8 | (upload only) |
| `TRAJ_RECOMPUTE_ERROR` | 9 | (upload only) |
| `POSE_FB_PIDII_LINEAR` | 10 | (upload only) |
| `POSE_FB_PIDII_ANGULAR` | 11 | (upload only) |
| `TWIST_FB_PIDII_LINEAR` | 12 | (upload only) |
| `TWIST_FB_PIDII_ANGULAR` | 13 | (upload only) |

Helper functions:
- `load_traj_params(path)` — load `TrajectoryParams` from JSON (or defaults)
- `load_robot_model(path)` — create `RobotModel` via FFI from JSON (or defaults), returns opaque handle
- `default_kf_params()` / `default_phys_params()` — Rust-library defaults via FFI
- `phys_params_to_json_dict()` / `kf_params_to_json_dict()` — round-trip ctypes structs back to JSON dict form
- `default_controller_params_dict()` — firmware controller-side defaults (PID gains, friction-comp gating, traj recompute thresholds, pose control gain); kept in sync with `firmware/control-board/src/motion/params/controller_params.rs`. Excludes `TWIST_FB_PIDII_*` because the firmware's `expected_format` returns `None` for them
- `default_params_dict()` — full JSON dict merging KF + phys + controller defaults
- `merge_params_dict(base, overrides)` — overlay JSON dicts
- `upload_params_dict(node, client, params)` — push a subset of `PARAM_MAP` entries from an in-memory dict via an existing rclpy node + `/set_firmware_param` client

### ROS Scripts (`analysis/ros_scripts/`)

These scripts require a sourced ROS 2 Jazzy workspace.

#### `telem_bag2np.py`
Converts ROS2 bags (mcap format, `ExtendedTelemetry` messages) to NumPy `.npz` archives.
```bash
python ros_scripts/telem_bag2np.py --bag data/bags/robot_telemetry --robot 0 -o data/telemetry/robot_telemetry.npz
```
- Recursively flattens ROS message fields using `/` as delimiter
- Reconstructs 64-bit `timestamp_us` from `timestamp_us_hi` and `timestamp_us_lo` fields

#### `upload_params.py`
Uploads all parameters from a JSON file to a robot via the `/set_firmware_param` ROS2 service.
```bash
python ros_scripts/upload_params.py --param-json data/robot_params.json --robot-id 2
```

#### `signal_input.py`
Standalone rclpy node that drives a single robot with a parameterized signal
(`pulse`, `sinusoid`, `step`, `chirp`, `square`) on a single axis (`x`, `y`, `theta`)
in either `BCM_GLOBAL_POSITION` or `BCM_GLOBAL_VELOCITY` mode. Latches the
robot's pose at startup and uses it as the center for position-mode signals.

State machine: `WAIT_FOR_VISION → WARMUP → RUN → HOLD` (exits after HOLD when
`--duration > 0`).

```bash
# 5 seconds of 1.5 Hz sinusoid on x at ±0.3 m around the starting pose:
python ros_scripts/signal_input.py --robot-id 2 --axis x \
    --signal sinusoid --amplitude 0.3 --sine-frequency 1.5 --duration 5.0
```

#### `controller_tune.py`
One-shot controller-tuning pipeline:
1. Upload firmware params from a JSON file (in-process via `params.upload_params_dict`)
2. Start `ros2 bag record` on `/robot_feedback/extended/robot<id>`, `/robot_feedback/basic/robot<id>`, and `/robot_motion_commands/robot<id>`
3. Run `signal_input.py` for `--duration` seconds
4. Stop the bag, convert via `telem_bag2np.py`, visualize via `telem_visualize.py` (blocks)

Bag and NPZ outputs are auto-indexed under `analysis/data/{bags,telemetry}/`
as `signal_<N>` / `signal_<N>.npz`. Passes all signal-related flags through
to `signal_input.py`. Use `--skip-upload` / `--skip-viz` to skip phases.

```bash
# Full one-shot trial with default sinusoid:
python ros_scripts/controller_tune.py --robot-id 2 --axis x \
    --signal sinusoid --amplitude 0.3 --sine-frequency 1.5 --duration 8.0

# Pulse step-response in velocity mode, no visualization:
python ros_scripts/controller_tune.py --control-mode velocity --axis x \
    --signal pulse --amplitude 0.5 --pulse-frequency 0.5 --pulse-width 0.5 \
    --duration 6.0 --skip-viz
```

#### `record_and_visualize.sh`
Quick single-shot: record a ROS bag, convert to NPZ, and visualize.
```bash
bash ros_scripts/record_and_visualize.sh 0   # robot ID as argument
```

### Telemetry Data Format

Telemetry NPZ keys use `/` as delimiter matching ROS message hierarchy:
- `timestamp_us` — reconstructed 64-bit microsecond timestamp
- `ros_t` — ROS timestamp in seconds (relative to bag start)
- `body_control_telemetry/kf_body_pos_estimate` — (N, 3) KF position estimate
- `body_control_telemetry/kf_body_vel_estimate` — (N, 3) KF velocity estimate
- `body_control_telemetry/kf_body_pos_prediction` — (N, 3) KF position prediction
- `body_control_telemetry/kf_body_vel_prediction` — (N, 3) KF velocity prediction
- `body_control_telemetry/vision_pose` — (N, 3) raw vision measurements
- `body_control_telemetry/vision_update` — (N,) bool, whether vision was received
- `body_control_telemetry/body_control_mode` — (N,) enum value
- `body_control_telemetry/body_accel_u` — (N, 3) commanded acceleration
- `body_control_telemetry/body_accel_u_fric_comp` — (N, 3) friction-compensated accel
- `body_control_telemetry/body_traj_pos` — (N, 3) trajectory position setpoint
- `body_control_telemetry/body_traj_vel` — (N, 3) trajectory velocity setpoint
- `body_control_telemetry/imu_accel` — (N, 3) IMU accelerometer
- `body_control_telemetry/imu_gyro` — (N, 3) IMU gyroscope
- `body_control_telemetry/skill_*/cmd_echo/*` — per-mode software command echoes
- `{wheel}_motor/velocity_telemetry/wheel_vel_rads` — encoder velocity
- `{wheel}_motor/velocity_telemetry/vel_setpoint_rads` — velocity setpoint
- `{wheel}_motor/current_telemetry/current_setpoint_ma` — current setpoint
- `{wheel}_motor/current_telemetry/current_samples_ma` — measured current samples

Body control mode values: 0=off, 1=global_pos, 2=global_vel, 3=local_vel, 4=global_acc, 5=local_acc.

---

## How This Library Is Used by Firmware

The firmware's `BodyController` (in `control-board/src/motion/robot_controller.rs`) uses this library at 1 kHz:

1. **State estimation:** Calls `kf_set_pose()` when vision arrives, then `kf_update()` with 8 measurements (3 vision + 4 encoder + 1 gyro), then `kf_predict()` at end of tick
2. **Trajectory planning:** Calls `BangBangTraj3D::from_target_pose()` or `from_target_twist()` for position/velocity modes, recomputes on target change or tracking error
3. **Kinematic transforms:** Uses `transform_twist2wheel()` and `transform_accel2wheel()` to convert body-level commands to wheel velocities and torques
4. **Friction compensation:** Uses `compute_friction_force()` to compensate Coulomb + viscous friction before kinematic transform
5. **Motor control:** Uses `torques_to_currents()` to convert wheel torques to current setpoints (clamped at 1500 mA)

### Control Modes (BodyControlMode)

| Mode | Value | Firmware Behavior |
|------|-------|-------------------|
| `BCM_OFF` | 0 | Motors disabled |
| `BCM_GLOBAL_POSITION` | 1 | Bang-bang trajectory to pose + PID feedback (requires vision) |
| `BCM_GLOBAL_VELOCITY` | 2 | Bang-bang trajectory to twist + PID feedback |
| `BCM_LOCAL_VELOCITY` | 3 | Rotate local twist to global via R_z(θ), then same as global velocity |
| `BCM_GLOBAL_ACCEL` | 4 | Direct acceleration passthrough (no trajectory, no PID) |
| `BCM_LOCAL_ACCEL` | 5 | Rotate local accel to global, then passthrough |

### Trajectory Recomputation
Trajectories are recomputed when:
- The target command changes
- Actual state deviates from trajectory beyond `TRAJ_RECOMPUTE_ERROR = [0.5, 1.0, 4.0, 8.0]` (pos_lin, pos_ang, vel_lin, vel_ang)

### Firmware Parameter Protocol
Parameters can be tuned at runtime via `CC_ROBOT_PARAMETER_COMMAND` radio packets. The `PARAM_MAP` in `analysis/params.py` defines the mapping between JSON keys, firmware parameter IDs, and the ctypes structs used by the controls library.

---

## Typical Development Workflows

### Modifying the Controls Library

1. Edit Rust source in `ateam-controls/src/`
2. Run `cargo test --workspace` to verify
3. If the C API changed:
   - Update `ateam-controls-c/include/ateam_controls/ateam_controls.h`
   - Update `ateam-controls-c/src/lib.rs` with new FFI functions
   - Update `ateam-controls-py/ateam_controls/_bindings.py` to match
   - Run `cmake --build build && cd build && ctest` to verify C++ tests
4. If parameter structs changed, update `analysis/params.py` PARAM_MAP

### Analyzing Robot Telemetry

1. Record telemetry: `ros2 bag record -o data/bags/robot_telemetry --topics /robot_feedback/extended/robot0`
2. Convert: `python ros_scripts/telem_bag2np.py --bag data/bags/robot_telemetry -o data/telemetry/robot_telemetry.npz --robot 0`
3. Visualize: `cd analysis && python telem_visualize.py -t data/telemetry/robot_telemetry.npz`

### Tuning Parameters

Two workflows live under `analysis/ros_scripts/`:

- **Controller tuning one-shot trial** (upload params → record → drive parameterized signal → visualize):
  ```bash
  cd analysis
  python ros_scripts/controller_tune.py --robot-id 2 --axis x \
      --signal sinusoid --amplitude 0.3 --sine-frequency 1.5 --duration 8.0
  ```
  Replaces the legacy `param_tuning_loop.py` (which had been pointing at a
  non-existent `ateam_motion_input` package). The signal generator is
  `signal_input.py` (pulse / sinusoid / step / chirp / square on x / y / theta in
  position or velocity mode; `square` is position-mode only) and can also be run on its own without the
  upload-and-record wrapper.
- **Feed-forward acceleration model tuning** (`coulomb` → `viscous` → `efficiency` → `inertia` → `verify`):
  see `analysis/ros_scripts/accel_tuning/README.md` for the full workflow,
  scoring formula, recommended order, manual-turnaround/resume behavior,
  and quick-reference commands. **Read that README before modifying any
  routine** under `analysis/ros_scripts/accel_tuning/` so the documented
  invariants (c_coul recomputation formula, score weights, checkpoint
  layout) stay consistent. A complete validated run is documented in the
  README's "Validated tuning session" section — that flow has been shown
  to produce a feed-forward model accurate to ~3% of commanded accel at
  the tuning amplitude (1.0 m/s² on robot 2 linear x), with two passes of
  the η ↔ c_visc coupled iteration needed for clean convergence.

  **Known model limitation:** the linear `c_visc·v` friction term and the
  scalar `motor_efficiency_factor` are **overloaded surrogates** — they
  jointly absorb physical viscous drag AND the BLDC motor's velocity-
  dependent torque droop (back-EMF and current-controller characteristics
  reduce realized torque as wheel speed rises). The tuned model therefore
  tracks well only in roughly ±25% of the velocity range induced by the
  tuning amplitude; outside that range the robot under-realizes commanded
  acceleration. The reported `c_visc` and `η` values are **not** physical
  truth, they are best-linear-fit numbers for the tuning regime. Plan to
  address this is to add a non-linear term (e.g., `c_drag·v²`) or
  speed-dependent `η(ω)` table to the firmware friction model and extend
  the toolkit to search the extra coefficient.

---

## Conventions

- **All angles in radians**, wrapped to [-π, π] via `wrap_angle()`
- **Global frame:** field coordinates (x right, y up, θ counter-clockwise from x)
- **Wheel ordering:** Front-Left, Back-Left, Back-Right, Front-Right (consistent everywhere)
- **Column-major matrices** in C types (matching nalgebra's storage)
- **Error codes** are negative `i32` in C FFI; zero means success
- **`repr(C)`** on all structs that cross the FFI boundary
- **No auto-generated bindings** — Python `_bindings.py` is hand-maintained to match the C header
