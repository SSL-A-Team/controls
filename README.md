# controls

This repository holds shared code and analysis tools for the A-Team motion control stack.

The shared code is written primarily in Rust with C interface bindings.

## Contents

* **ateam-controls** — Primary Rust crate for control algorithms
* **ateam-controls-c** — C-compatible library and headers
* **ateam-controls-py** — Installable Python package with ctypes bindings (same interface as the C library)
* **analysis** — Telemetry visualization scripts and ROS tooling
* **cpp_tests** — Tests for the C-compatible bindings

## Dependencies

### For Rust users
* [nix](https://nixos.org/)

### For C++ users
* [Rust](https://rust-lang.org/)
  * `curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`
* [CMake](https://cmake.org/)
  * `apt install cmake`

### For Python / analysis users
* [uv](https://docs.astral.sh/uv/)
  * `curl -LsSf https://astral.sh/uv/install.sh | sh`

## Building and Testing

### Rust

```bash
nix develop
cargo build --workspace
cargo test --workspace
```

### C++

```bash
cmake -B build .
cmake --build build
cd build
ctest
```

### Python

```bash
uv sync
source .venv/bin/activate
```

> **WSL users:** You may need `sudo apt install python3-tk` for matplotlib GUI windows.

## Analysis

Activate the python virtual environment and change to the analysis directory

```bash
source .venv/bin/activate
cd analysis
```

While running robots, capture telemetry to a ros bag, convert it to a numpy archive, and visualize the telemetry

```bash
rm -rf data/bags/robot_telemetry && ros2 bag record -o data/bags/robot_telemetry --topics /robot_feedback/extended/robot0
python ros_scripts/telem_bag2np.py --bag data/bags/robot_telemetry -o data/telemetry/robot_telemetry.npz --robot 0
python telem_visualize.py -t data/telemetry/robot_telemetry.npz
```

This opens:
- **Body state** — 3×3 grid (pos/vel/accel × x/y/theta) with predicted, estimated,
  measured, and commanded traces. Press **Space** to toggle trajectory + velocity overlays.
- **Wheel current** — commanded vs measured current per wheel (2×2).
- **Wheel velocity** — commanded vs measured velocity per wheel (2×2).

### ROS Scripts

Scripts in `analysis/ros_scripts/` require a sourced ROS2 workspace:

- `telem_bag2np.py` — Convert ROS2 bags to NumPy archives
- `upload_params.py` — Upload parameters to robot firmware via ROS2 services
- `signal_input.py` — Signal generator (pulse / sinusoid / step / chirp) for one axis in position or velocity mode
- `controller_tune.py` — One-shot controller-tuning trial: upload → record → drive signal → visualize
- `record_and_visualize.sh` — Record a ROS bag and visualize
- `accel_model_tune.py` — Feed-forward acceleration model tuning toolkit (see below)

### Feed-forward acceleration model tuning

`accel_model_tune.py` is a modular tuning toolkit for the parameters of
`RobotModel` that participate in the feed-forward `accel → wheel current`
pipeline:

| Step          | Tunes                                                                           |
|---------------|---------------------------------------------------------------------------------|
| `coulomb`     | `coulomb_friction_coefficient_{linear,angular}` — minimum-motion pulse search   |
| `viscous`     | `viscous_friction_coefficient_{linear,angular}` — triangular profile linearity  |
| `efficiency`  | `motor_efficiency_factor` — realized vs. commanded acceleration                 |
| `inertia`     | `iz` — realized vs. commanded angular acceleration (theta axis only)            |
| `all`         | Runs the four steps in sequence with a re-convergence pass on angular coulomb   |

Each step pushes only the parameters it owns to the robot via
`/set_firmware_param`. Telemetry and parameter checkpoints land in
`analysis/data/accel_tuning/<run_id>/`. The `y` axis is allowed for
diagnostic-only runs (no firmware param push).

Examples:

```bash
# 1) Coulomb step on x (pushes η=1, c_*=0 first, then searches a_min)
python ros_scripts/accel_model_tune.py coulomb -r 0 --axis x

# 2) Chain viscous on x using the coulomb result as the starting params
python ros_scripts/accel_model_tune.py viscous -r 0 --axis x \
    --baseline-params data/accel_tuning/<run_id>/coulomb/x/result.json \
    --a-min 1.4 --v-min 0.12

# 3) Run a single user-chosen value instead of searching
python ros_scripts/accel_model_tune.py efficiency -r 0 --axis x \
    --mode single --value 13.0 --a-min 1.4 --v-min 0.12

# 4) End-to-end automated pipeline
python ros_scripts/accel_model_tune.py all -r 0
```

The friction model being tuned is

```
F_friction = -c_visc·v - c_coul·sign(v)
a_cmd_comp = a_cmd + I⁻¹·(c_visc·v + c_coul·sign(v))
i_wheel    ≈ (I·a_cmd + c_visc·v + c_coul) / (Kt·η)
```

so the coulomb step records `(a_min, v_min)` and the viscous / efficiency /
inertia steps recompute `c_coul = η·I·a_min − c_visc·v_min` on every
candidate to keep the friction balance satisfied as the other parameters
change.

Triangular-profile amplitudes (`TRIANGULAR_PULSE_ACCEL_{LIN,ANG}`), pulse
durations, and search bounds are constants at the top of the relevant
modules in `analysis/ros_scripts/accel_tuning/` for easy editing.

