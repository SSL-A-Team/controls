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
- `param_tuning_loop.py` — Interactive edit → upload → record → visualize workflow
- `record_and_visualize.sh` — Record a ROS bag and visualize
