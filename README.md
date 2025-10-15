# controls

This repository holds shared code and analysis docs for the A-Team motion control stack.

The shared code is written primarily in Rust with C interface bindings.

## Contents
* analysis
  * Docs and scripts used to design and understand our motion controls
* ateam_controls
  * The primary Rust crate for shared code
* ateam_controls_c
  * Crate for building C-compatible library and headers
* cpp_tests
  * Tests for the C-compatible bindings

## Dependencies
### For Rust users
* [nix](https://nixos.org/)

### For C++ users
* [Rust](https://rust-lang.org/)
  * `curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`
* [CMake](https://cmake.org/)
  * `apt install cmake`

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
