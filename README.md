# controls

This repository holds shared code and analysis tools for the A-Team motion control stack.

The shared code is written primarily in Rust with C interface bindings.

## Contents

* **ateam-controls** — Primary Rust crate for control algorithms
* **ateam-controls-c** — C-compatible library and headers
* **analysis** — ROS tooling
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
