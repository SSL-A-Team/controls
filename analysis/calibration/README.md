# Calibration routines

Each script drives the robot through tether-safe test moves (the `ros_scripts` tests
and their safety monitor), records telemetry, and prints the value to use.
To roll out a value, set the firmware constant listed below and flash **all** robots.
The scripts can also try candidate values live over radio parameters (lost on reboot)
to compare them before you commit a number.

| Value | Scope | Script | Firmware constant | Current |
|---|---|---|---|---|
| Vision latency (camera exposure -> robot) | field setup | `vision_latency.py` | `DEFAULT_VISION_MIN_LATENCY_US` (`ateam-controls/src/defaults.rs`) | 56 ms |
| Vision measurement noise (EKF R) | field setup | `vision_noise.py` | `DEFAULT_EKF_VISION_VAR_M`, `DEFAULT_EKF_VISION_VAR_RAD` (std squared, `defaults.rs`) | 10 mm, 0.01 rad |
| Linear output efficiency, Coulomb friction | robot | `efficiency_friction.py` | `DEFAULT_PHYS_EFFICIENCY_XY`, `DEFAULT_PHYS_COULOMB_XY` (`defaults.rs`) | 0.75, 4.5 N |
| Wheel slip feed-forward gain | robot | `wheel_slip.py` | `WHEEL_SLIP_COMP` (`control-board/src/motion/params/controller_params.rs`) | 1.2 |

All current values were measured on robot 2 (Nov 12-13 2026) and are used for every robot.

Not calibrated by these scripts:

- **IMU bias** (gyro/accel offsets): stored on each robot in flash. Hold ENTER while
  powering on (robot upright and still) to calibrate; hold BACK to erase.
- **Accelerometer bias drift**: learned automatically at standstill by the state estimator.
- Mass, inertia, wheel geometry, motor constant: nominal values in `defaults.rs`.

## Setup

1. Robot on the field, tethered or free, with room to move ±0.2 m and turn ±60°
   around its home pose. Every test stops the robot if it leaves ±0.25 m / ±95° of
   home, goes too fast, or loses telemetry or vision.
2. **Extended telemetry on** (control board DIP switch 0). The scripts record the
   extended telemetry stream.
3. Vision running and the robot seen. Radio bridge running with the wheel velocity loop
   enabled (the default):

   ```
   ros2 launch ateam_bringup bringup_physical_core.launch.py
   ```

4. Run from the `controls` directory (the tests write logs to `analysis/data/`), with
   the workspace sourced:

   ```
   source /opt/ros/jazzy/setup.bash; source ~/workspace/ateam_ws/install/setup.bash
   cd controls
   ```

Every script takes `--home=x,y,theta` (an absolute pose to move from and return to)
and `--heading-ref theta` (the heading the ±95° limit is measured from, normally the
home heading), plus `-r ROBOT_ID` (default 2). Examples below use home (0, 0, π).
A script run takes 1-3 minutes; each validation candidate takes about 1 minute per repeat.

## Order

Do the field-setup values first (once per camera setup, on any one robot), then the
robot values. Robot values depend on a correct state estimate.

### 1. Vision latency (setup)

```
python3 analysis/calibration/vision_latency.py --home=0,0,3.14 --heading-ref 3.14
```

Two fast ±70° rotations. The script compares vision heading with gyro- and
encoder-integrated heading and prints the shift between them (it reads the firmware's
current latency and EKF delay from `defaults.rs`; pass `--min-latency-ms` /
`--ekf-delay-ms` if the robot runs something else). Within ±3 ms is already right.
Otherwise set `DEFAULT_VISION_MIN_LATENCY_US` to the recommendation, check the
printed max age stays below `DEFAULT_VISION_MAX_AGE_ACCEPTANCE_US` (and
`DEFAULT_EKF_BUFF_LEN` above it, 1 ms per frame), flash, and run again to confirm.

Redo this whenever cameras, camera settings, the vision PC or SSL-Vision change.

### 2. Vision noise / EKF R (setup)

```
python3 analysis/calibration/vision_noise.py --home=0,0,3.14 --heading-ref 3.14 \
    --r-values 0.05,0.1 0.01,0.01 0.003,0.005
```

Prints vision noise at rest, then for each R (radio parameter `KF_MEASUREMENT_STD`) the
real-time estimate error against vision while moving and at rest. Choose the largest R
whose moving error is close to the best: smaller R trusts vision more, which hurts with
noisy or multi-camera vision. The at-rest error should be about 1 mm with a near-zero
mean offset; a constant offset points at a bias, not R. Set the two variances
(std squared) in `defaults.rs`.

### 3. Efficiency and friction (robot)

```
python3 analysis/calibration/efficiency_friction.py --home=0,0,3.14 --heading-ref 3.14
python3 analysis/calibration/efficiency_friction.py --home=0,0,3.14 --heading-ref 3.14 \
    --skip-fit --validate 4.5,0,0.1,0,0.75,0.8 3.9,0,0.1,0,0.75,0.8
```

The fit uses forward, sideways and diagonal moves:
`m·a = efficiency·F_motor − k(direction)·coulomb`. Efficiency transfers directly. The
fitted Coulomb force covers only what the model captures: on robot 2 it fits 3.7-3.9 N,
but 4.5 N tracked as well or better. So validate it. `--validate` sets each
`PHYS_FRICTION_MODEL` candidate
(`coulomb_xy, viscous_xy, coulomb_w, viscous_w, efficiency_xy, efficiency_w`)
and reports trajectory tracking over the move set; use the default `--repeats 2` or
more, and treat differences under ~1 mm as noise.

### 4. Wheel slip (robot)

```
python3 analysis/calibration/wheel_slip.py --home=0,0,3.14 --heading-ref 3.14
python3 analysis/calibration/wheel_slip.py --home=0,0,3.14 --heading-ref 3.14 \
    --skip-fit --validate 0 0.8 1.2 1.6
```

Fits how much faster each wheel spins than the body moves, per amp of linear wheel
current (robot 2: K ≈ 1.0). It also checks rotation, which should show no slip (|K| <
~0.3); the firmware compensates only the linear part. Validate around the fitted K
(robot 2: 0 → 10.0 mm, 1.2 → 7.3 mm tracking RMS) and set `WHEEL_SLIP_COMP`.

## Rolling out

1. Edit the constants above, then build and flash every robot:
   `nix develop --command make control-board--control`, then flash each robot.
2. Spot-check one robot with the regression tests (`ros_scripts/small_move_test.py`,
   `replan_test.py`, `theta_ramp_test.py`).

## Troubleshooting

- *"no telemetry recorded"*: extended telemetry DIP switch off, or the bridge is not
  receiving; check `ros2 topic hz /robot_feedback/extended/robotN`.
- *Moves skipped / "no telemetry/vision"*: the bridge or vision stopped publishing.
  Restarting the bridge launch has fixed this.
- *Tests trip on "telemetry stale"*: Wi-Fi stalls longer than 0.8 s; rerun.
- Recordings are saved to `--out` (default `/tmp/calib.npz`). `vision_latency.py
  --no-run` and `efficiency_friction.py --no-run` re-analyze a saved recording.
