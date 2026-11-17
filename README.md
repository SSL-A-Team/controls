# controls

This repository holds shared code and analysis tools for the A-Team motion control stack.

The shared code is written primarily in Rust with C interface bindings.

## Contents

* **ateam-controls** — Primary Rust crate for control algorithms
* **ateam-controls-c** — C-compatible library and headers
* **ateam-controls-py** — Installable Python package with ctypes bindings (same interface as the C library)
* **analysis** — Robot calibration routines and ROS motion test scripts
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

## Analysis

Telemetry is viewed in Foxglove. The scripts in `analysis/` drive the robot and
record telemetry for calibration and testing. They need a sourced ROS2 workspace
and are run from the `controls` directory (logs go to `analysis/data/`).

- `calibration/` — per-setup and per-robot calibration routines (vision latency,
  vision noise, output efficiency and friction, wheel slip). See
  [`analysis/calibration/README.md`](analysis/calibration/README.md).
- `ros_scripts/small_move_test.py` — short position moves with a fast safety
  monitor (tether box, speed, telemetry/vision loss). The other tests build on it.
- `ros_scripts/vel_test.py`, `theta_ramp_test.py`, `maneuver_test.py`,
  `joy_pos_test.py`, `replan_test.py` — velocity, angular-ramp, pivot/line,
  joystick position-mode and replanning tests.
- `ros_scripts/telem_record.py` — record extended telemetry to an `.npz`.
- `hf_capture_decode.py` — decode the 1 kHz control-loop capture from an RTT log
  (control board built with `--features hf-capture`).
- `feedback_sim.py` — offline closed-loop safety simulation of the motion output stage.
