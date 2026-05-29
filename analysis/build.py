"""Build helper for the Rust controls library."""

import subprocess
from ateam_controls._bindings import CONTROLS_REPO_PATH


def compile_controls():
    """Build the shared library (and binaries) in release mode."""
    build_cmd = ["cargo", "build", "--release", "--workspace"]
    subprocess.run(build_cmd, cwd=CONTROLS_REPO_PATH, check=True)
    print("Built controls workspace (release).")
