# Feed-Forward Acceleration Model Tuning

Modular CLI toolkit for tuning the parameters of the `ateam_controls::RobotModel`
that participate in the firmware's feed-forward `accel → wheel current`
pipeline. Implemented as subcommands of `analysis/ros_scripts/accel_model_tune.py`.

> **Important:** this tunes only the *feed-forward* model. PID gains,
> trajectory planning, and bang-bang parameters are **not** touched.

---

## Validated tuning session (2026-06-02, robot 2, linear x)

The pipeline below has been run end-to-end successfully on robot 2's
linear-x axis and produces a feed-forward model that tracks within ~3% of
commanded acceleration in the **±25% velocity range around the tuning
amplitude**. Outside that range the model is biased — see the "Known model
limitations" subsection below for why.

### Commands run, in order

```bash
# 1. Coulomb — kinetic friction breakaway via push-start
python analysis/ros_scripts/accel_model_tune.py coulomb -r 2 --axis x \
    --run-id coulomb_x1 \
    --start-accel 0.05 --step 0.005 --max-accel 0.1 \
    --pulse-duration 5.0 --rest-duration 1.0
# → a_min=0.050, v_min=0.145

# 2. Viscous (first pass) — search c_visc with η overridden to 13
python analysis/ros_scripts/accel_model_tune.py viscous -r 2 --axis x --mode search \
    --run-id viscous_x1 \
    --baseline-params analysis/data/accel_tuning/coulomb_x1/coulomb/x/result.json \
    --a-min 0.05 --v-min 0.145 --amplitude 1.0 \
    --motor-efficiency 13.0 \
    --max-iter 8 --replicates 3 --max-replacements 2 --plot

# 3. Efficiency (first pass) — bracketed root search on η
python analysis/ros_scripts/accel_model_tune.py efficiency -r 2 --axis x --mode search \
    --run-id efficiency_x1 \
    --baseline-params analysis/data/accel_tuning/viscous_x1/viscous/x/result.json \
    --a-min 0.05 --v-min 0.145 --amplitude 1.0 \
    --initial 13.0 \
    --max-iter 8 --min-iter 5 --tol 0.05 --tol-x-frac 0.05 \
    --replicates 3 --max-replacements 2 --plot

# 4. Viscous (second pass) — re-search c_visc at the new η
python analysis/ros_scripts/accel_model_tune.py viscous -r 2 --axis x --mode search \
    --run-id viscous_x2 \
    --baseline-params analysis/data/accel_tuning/efficiency_x1/efficiency/x/result.json \
    --a-min 0.05 --v-min 0.145 --amplitude 1.0 \
    --bounds-low 0.0 --bounds-high 5.0 \
    --max-iter 8 --replicates 3 --max-replacements 2 --plot

# 5. Efficiency (second pass) — confirm η at the new c_visc
python analysis/ros_scripts/accel_model_tune.py efficiency -r 2 --axis x --mode search \
    --run-id efficiency_x3 \
    --baseline-params analysis/data/accel_tuning/viscous_x2/viscous/x/result.json \
    --a-min 0.05 --v-min 0.145 --amplitude 1.0 \
    --initial 17.52 --bounds-low 12.0 --bounds-high 23.0 \
    --max-iter 8 --min-iter 5 --tol 0.05 --tol-x-frac 0.05 \
    --replicates 3 --max-replacements 2 --plot

# 6. Verify across an amplitude sweep (0.75, 1.0, 2.0 m/s²)
python analysis/ros_scripts/accel_model_tune.py verify -r 2 --axis x \
    --run-id verify_x_0.75mss_accel1 \
    --baseline-params analysis/data/accel_tuning/efficiency_x3/efficiency/x/result.json \
    --amplitude 0.75 --trials 5 --plot
python analysis/ros_scripts/accel_model_tune.py verify -r 2 --axis x \
    --run-id verify_x_1mss_accel1 \
    --baseline-params analysis/data/accel_tuning/efficiency_x3/efficiency/x/result.json \
    --amplitude 1.0 --trials 5 --plot
python analysis/ros_scripts/accel_model_tune.py verify -r 2 --axis x \
    --run-id verify_x_2mss_accel1 \
    --baseline-params analysis/data/accel_tuning/efficiency_x3/efficiency/x/result.json \
    --amplitude 2.0 --trials 5 --plot
```

### Final converged values for robot 2 linear x

| param | value |
|---|---|
| `motor_efficiency_factor` (η) | **17.36** |
| `c_visc_linear_x` | **3.30** |
| `c_coul_linear_x` | **1.86** (recomputed from `η·mass·a_min − c_visc·v_min`) |
| `mass` (measured, not tuned) | 2.7 |
| coulomb anchors | `a_min=0.05`, `v_min=0.145` |

### Verify-sweep results (amplitude → accel_err mean)

| amplitude | accel_err mean | accel_err / amplitude |
|---|---|---|
| 0.75 m/s² | −0.10 | −14% |
| **1.0 m/s²** | **−0.03** | **−3%** ← tuned point |
| 2.0 m/s² | −0.45 | −22% |

The model tracks within 3% at the tuning amplitude and degrades roughly
monotonically as amplitude (and therefore peak velocity) departs from it.

### Why the second viscous + efficiency pass was needed

The first viscous pass at `--motor-efficiency 13` converged on `c_visc≈0.67`
because some of the friction-comp was absorbing the η-13 mismatch (real η
is closer to 17). Once efficiency_x1 found η≈17.5, re-running viscous found
the *actual* friction-residual coefficient: `c_visc≈3.30`. Then a second
efficiency pass at the new `c_visc` confirmed η didn't drift (17.52 → 17.36,
a ~2% change — convergence). This **η ↔ c_visc coupled-fixpoint iteration
generally requires two passes for a clean solution.**

### Known model limitations (the linear surrogate is not the physical truth)

The `c_visc` and `motor_efficiency_factor` parameters are **overloaded**:
they absorb both physical friction and a velocity-dependent motor torque
droop that the firmware's current pipeline does not model directly. BLDC
torque output decreases as wheel speed rises (back-EMF + current-controller
characteristics), so the linear `c_visc·v` term is doing two jobs:

1. Compensating real viscous drag (probably the smaller share).
2. Mimicking the motor's velocity-dependent torque shortfall.

Because of this:

- The tuned model tracks well only inside roughly ±25% of the **velocity
  range** induced by the tuning amplitude. Outside that range the linear
  surrogate cannot keep up with the non-linear plant and the robot
  under-realizes commanded accel (negative `accel_err`).
- The reported `c_visc` is **not** the true physical viscous coefficient,
  and the reported `η` is **not** the true motor constant divisor — they
  are best-linear-fit numbers for the tuning regime.
- The closed-loop PID will absorb the residual deficit during execution,
  at the cost of more tracking error outside the tuning regime.

Plan to address this is to add a non-linear (e.g., `c_drag·v²`) or
piecewise term to the firmware friction model and extend the toolkit to
search the extra coefficient; the current toolkit searches one parameter
per routine, so a multi-parameter search needs a new routine. See
`Starting Generative Prompt` near the bottom of this file for direction.

---

## What gets tuned

| Param                                    | Step             | Notes                                                        |
|------------------------------------------|------------------|--------------------------------------------------------------|
| `coulomb_friction_coefficient_linear_x`  | `coulomb x`      | Min-motion pulse search; computes `c_coul = η·I·a_min − c_visc·v_min` |
| `coulomb_friction_coefficient_linear_y`  | `coulomb y`      | Strafing has its own constants                               |
| `coulomb_friction_coefficient_angular`   | `coulomb theta`  | Angular axis                                                 |
| `viscous_friction_coefficient_linear_x`  | `viscous x`      | Triangular profile, golden-section search for linearity      |
| `viscous_friction_coefficient_linear_y`  | `viscous y`      | Strafing viscous term                                        |
| `viscous_friction_coefficient_angular`   | `viscous theta`  | Angular axis                                                 |
| `motor_efficiency_factor`                | `efficiency x|y` | Secant search; drives realized accel to commanded            |
| `iz` (rotational inertia)                | `inertia`        | Angular only; uses the linear η                              |

Always tuned only via this toolkit:
- `mass` is treated as known (measure it).
- `Kt`, wheel geometry (`alpha`, `beta`, `l`, `r`) are not tuned here.

All three axes (`x`, `y`, `theta`) push parameter changes to the firmware;
the friction model has independent coefficients for local-frame `x` and
`y` so strafing and forward/backward friction can be tuned separately.

---

## Friction model being tuned

```
F_friction  = −c_visc · v_local − c_coul · sign(v_local)      (per local body axis)
a_cmd_comp  = a_cmd + I⁻¹·(c_visc · v + c_coul · sign(v))
i_wheel     ≈ (I · a_cmd + c_visc · v + c_coul) / (Kt · η)
```

Independent coefficients per local axis (the previous shared-linear model
gave bad strafing performance). The firmware rotates global twist and accel
into the robot's local frame before computing friction, then rotates the
result back out for subtraction from the global acceleration command.

The coulomb formula every routine uses to keep the friction-balance constraint
satisfied as other parameters change:

```
c_coul = η · I · a_min − c_visc · v_min
```

where `(a_min, v_min)` is the (sustained-motion threshold accel, mean
local-frame velocity during that pulse) pair recorded by the coulomb step.
For linear `I = mass`; for angular `I = iz`.

---

## Recommended tuning order

Run each step, inspect the result JSON + plots, then chain to the next step
by passing the previous step's `result.json` to `--baseline-params`. The
toolkit will also keep firmware in sync automatically (each step pushes only
the params it owns).

```
LINEAR (axis = x)
  1. coulomb x        → c_coul_lin from (a_min, v_min) at η=1, c_*=0
  2. viscous x        → searches c_visc keeping the +A / −A ramps linear
  3. efficiency x     → searches η until realized accel matches commanded
  4. viscous x (opt.) → re-run with tuned η for refinement
  5. verify x         → N back-and-forth pulses, mean ± std of score

ANGULAR (axis = theta)
  6. coulomb theta    → c_coul_ang, holding tuned linear η + linear friction
  7. viscous theta    → c_visc_ang
  8. inertia          → iz; theta only
  9. coulomb theta    → optional reconvergence pass after iz changed
 10. verify theta     → N rotational pulses, mean ± std of score
```

The end-to-end `--routine all` orchestrates 1–9 in one shot, but you almost
always want to run them individually so you can sanity-check between steps
and recover from a bad trial.

---

## Common CLI flags

Every routine accepts these:

| Flag                       | Purpose                                                                |
|----------------------------|------------------------------------------------------------------------|
| `-r N`                     | Robot ID (required)                                                    |
| `--axis x\|y\|theta`       | Tuning axis (theta is angular; y is diagnostic-only)                   |
| `--run-id NAME`            | Checkpoint dir name. Reuse to **resume** a search after a timeout exit. |
| `--baseline-params PATH`   | Path to a prior step's `result.json` (its `params_after` is loaded)    |
| `--motor-efficiency F`     | Override `η` in the loaded baseline params before running              |
| `--no-turnaround`          | Skip the manual-rotation prompt between trials (linear axes only)      |
| `--plot`                   | After the routine completes, open the 3×3 telemetry plot per trial    |
| `--dry-run`                | Don't publish motion or push params; exercise state machines only      |

Routine-specific flags are documented under each section below.

### Manual turnaround (linear axes)

Between each trial on linear axes the routine prints:

```
>>> Please rotate robot 2 by >= 90 deg before the next pulse. Waiting...
```

Pick up the robot, rotate it ≥ 90° (any direction), then set it down.
Detection: `|Δθ_kf| > π/2 AND |kf_vel| < 0.05`. If you don't rotate within
120 s, the routine raises `TurnaroundTimeout`, exits with status 2, and
prints how to resume.

### Resuming after timeout

```bash
# Just rerun the same command with the same --run-id.
# Previously-evaluated candidates are served from the on-disk progress
# cache (analysis/data/accel_tuning/<run-id>/<routine>/<axis>/progress.json)
# without re-pulsing the robot. The deterministic search algorithms
# (golden-section, secant) replay the same candidate sequence.
```

### Checkpoint layout

```
analysis/data/accel_tuning/<run-id>/
├── run_metadata.json
├── baseline_params.json
├── <routine>/<axis>/
│   ├── trial_0000.json     # candidate, score breakdown, metrics
│   ├── trial_0000.npz      # raw per-sample telemetry (firmware-style keys)
│   ├── progress.json       # resume cache: list of {candidate, score}
│   └── result.json         # final accepted params + summary
└── summary.json            # only written by --routine all
```

---

## Scoring

Every trial that uses the triangular profile (`viscous`, `efficiency`,
`inertia`, `verify`) produces a composite **score** where lower is better:

```
score = (residual_+ + residual_−)            # curvature on both ramps
      + 0.5 · ||slope_+| − |slope_−||         # ramp asymmetry
      + 1.0 · |slope_coast|                   # coast-phase flatness
      + (5.0  if peak_v < 0.25 · A · T_pulse) # low-motion penalty
```

Components, lower is better for each:

- **residual** — RMS of `v − (slope·t + intercept)` per ramp. Velocity should
  change linearly during a constant accel command. Curvature ⇒ model wrong.
- **asym** — `||slope_+| − |slope_−||`. The `+A` and `−A` slopes should be
  equal in magnitude when friction is modeled correctly. Asymmetry says
  velocity-dependent friction is off.
- **coast_flat** — `|slope during coast|`. During coast (zero commanded
  accel) the body should hold its velocity. Non-zero slope ⇒ friction
  imbalance at that velocity.
- **low_motion penalty** — fires when `peak_v < 25 % · A · T_pulse`. Catches
  trials where the model is so wrong the robot barely moves, so the
  algorithm doesn't accept "tiny but technically convergent" solutions.

Weights live at the top of `analysis.py`:

```python
ASYMMETRY_WEIGHT      = 0.5
COAST_FLATNESS_WEIGHT = 1.0
LOW_MOTION_FRACTION   = 0.25
LOW_MOTION_PENALTY    = 5.0
```

The `efficiency` and `inertia` routines additionally compute a signed
**accel_match_error** = `mean(slope_+, |slope_−|) − commanded_A`. The secant
search drives that to zero (the score is logged but not searched).

The KF velocity used in scoring is **rotated into the local frame** per
sample (using the KF θ), so it matches the local-frame `BCM_LOCAL_ACCEL`
command.

---

## Per-routine reference

### coulomb

Pulse-and-step with manual push-start. Publishes increasing acceleration
amplitudes; at the start of each pulse the routine logs `>>> PUSH NOW
(small forward shove)` — the user gives the robot a small nudge so the
measurement reflects **kinetic-friction sustain** (smallest accel that
keeps a pushed robot moving) rather than **static breakaway** (which has
high trial-to-trial variance from microcontact and surface conditions).
The first amplitude where the wheel-encoder velocity stays above threshold
for the trailing half of the pulse is the `a_min`. The KF local-frame
velocity during the same pulse is the `v_min`. Coulomb is then
`η·I·a_min − c_visc·v_min`.

Pre-trial state pushed (when no `--baseline-params` is supplied):
- `motor_efficiency = 1.0`
- `c_coul_* = 0`, `c_visc_* = 0`
- default inertia

| Flag                       | Default | Purpose                                              |
|----------------------------|---------|------------------------------------------------------|
| `--start-accel F`          | 0.0     | First pulse amplitude                                |
| `--step F`                 | axis-default (0.05 lin, 0.2 ang) | Increment per attempt   |
| `--pulse-duration F`       | 0.3     | Seconds per pulse                                    |
| `--rest-duration F`        | 1.0     | Seconds at BCM_OFF between pulses                    |
| `--max-accel F`            | axis-default (5.0 lin, 15.0 ang) | Ceiling before giving up |
| `--velocity-threshold F`   | 0.5     | Wheel encoder rad/s threshold                        |
| `--min-samples N`          | 3       | Min telemetry samples needed to evaluate a pulse     |

### viscous

Triangular profile with golden-section search on `c_visc`. Coulomb is
re-computed every candidate using stored `(a_min, v_min)`, so the
friction-balance constraint stays satisfied.

**Per-candidate replicates + outlier removal.** Every golden-section
evaluation runs `--replicates` trials at the same `c_visc` (default 3)
and returns the **median** to the search, so the bracket isn't poisoned
by a single noisy trial. After the initial replicates, any trial more
than `--outlier-mad · 1.4826 · MAD` from the median may be replaced by
a fresh trial, up to `--max-replacements` times per candidate (default
2). All raw replicate scores plus the per-candidate aggregated median
are saved to `result.json:candidates`. Setting `--replicates 1
--max-replacements 0` restores the old single-shot behavior.

> **Cost warning:** trial count grows as `max_iter × (replicates +
> avg_replacements_used)`. With defaults a 10-iter search costs 30–50
> physical trials, each with a manual turnaround on linear axes. Drop
> `--replicates 2 --max-replacements 1` for a faster pass.

| Flag                       | Default | Purpose                                              |
|----------------------------|---------|------------------------------------------------------|
| `--mode single\|search`    | search  | `single` runs one (replicated) trial set at `--value` |
| `--value F`                | —       | (single) `c_visc` to evaluate                        |
| `--a-min F`                | —       | From prior coulomb step (m/s² or rad/s²)             |
| `--v-min F`                | 0.0     | From prior coulomb step                              |
| `--amplitude F`            | axis-default (1.0 lin, 3.0 ang) | Triangular pulse amplitude |
| `--bounds-low F` / `--bounds-high F` | axis-default | Override golden-section bounds       |
| `--tol F`                  | 0.05    | Golden-section bracket-width stop threshold          |
| `--max-iter N`             | 8       | Max golden-section iterations                        |
| `--replicates N`           | 3       | Trials per candidate; median returned to GS (1 = single-shot) |
| `--max-replacements N`     | 2       | Per-candidate outlier-replacement cap                |
| `--outlier-mad F`          | 3.0     | MAD-σ multiplier for outlier detection (lower = stricter) |

> The outlier threshold has a floor of `max(0.05·|median|, 0.02)` so it
> never collapses to noise level when initial replicates happen to cluster
> very tightly. Without the floor, an unusually consistent first batch
> makes any normal-jitter third replicate look like an outlier and the
> replacement loop wastes trials chasing zero-mean noise. Tune
> `mad_floor_rel` / `mad_floor_abs` in `_search.replicated_evaluate` if
> you need different thresholds.

### efficiency

Triangular profile with a **bracketed root search** on
`motor_efficiency_factor`, driving `mean(slope_+, |slope_−|) − A` to zero.

The search has two phases:

1. **Bracketing.** Evaluate both seeds. If they fall on the same side of
   the root, expand the seed pair outward (clamped by `--bounds-low` /
   `--bounds-high`) by `--expand-factor` until a sign change is found.
   This guarantees the root lies inside a proven interval — the old
   plain-secant could happily extrapolate to a "root" that didn't exist
   in the search range.
2. **Refinement.** Once bracketed, false-position with the Illinois
   modification — converges as fast as secant in well-behaved cases and
   doesn't stagnate when one endpoint stays stuck.

**Termination** requires *all* of:
- at least `--min-iter` evaluations (forces real exploration; can't exit
  on a tol-skin from a single noisy seed),
- bracket width ≤ `--tol-x-frac · (bounds_high − bounds_low)`,
- `|f(x)| ≤ --tol`.

`--max-iter` is the safety cap. The returned `eta` is the x with smallest
|f| across all evaluations, not just the last one.

Combined with **per-candidate replicates + outlier removal** (3 replicates,
2 max replacements, 3.0 MAD-σ by default — same semantics as viscous), each
evaluation hands a noise-robust median signed-error back to the search.

| Flag                       | Default | Purpose                                              |
|----------------------------|---------|------------------------------------------------------|
| `--mode`, `--value`, `--a-min`, `--v-min`, `--amplitude` | (same as viscous) |              |
| `--initial F`              | 13.0    | Initial guess; seeds = `[0.7·initial, 1.3·initial]`  |
| `--bounds-low F` / `--bounds-high F` | 1.0 / 25.0 | Hard search clamp; also caps bracketing expansion |
| `--tol F`                  | 0.05    | `|f(x)|` stop threshold (tol_y)                      |
| `--tol-x-frac F`           | 0.05    | Bracket-width stop threshold as fraction of `(bounds_high − bounds_low)` |
| `--min-iter N`             | 4       | Floor on evaluations regardless of tol               |
| `--expand-factor F`        | 1.5     | Seed-pair expansion multiplier when not bracketed    |
| `--max-iter N`             | 8       | Safety cap on total evaluations                      |
| `--replicates N`           | 3       | Per-candidate replicate count (1 = single-shot)      |
| `--max-replacements N`     | 2       | Per-candidate outlier-replacement cap                |
| `--outlier-mad F`          | 3.0     | MAD-σ multiplier for outlier detection (lower = stricter) |

### inertia

Angular axis only. Same bracketed root search and replicate semantics as
efficiency, applied to `iz`. Same CLI flags (`--replicates`,
`--max-replacements`, `--outlier-mad`, `--tol-x-frac`, `--min-iter`,
`--expand-factor`).

### verify

No search. Uploads a baseline param set and runs N triangular pulses,
scoring each. Use after tuning to sanity-check open-loop performance.

| Flag                       | Default | Purpose                                              |
|----------------------------|---------|------------------------------------------------------|
| `--trials N`               | 5       | Number of triangular trials to run                   |
| `--amplitude F`            | axis-default | Triangular pulse amplitude                      |

Aggregate stats printed at the end (`mean ± std`, min/max of scores and
peak velocities). The `score_std / score_mean` ratio is your repeatability
metric — under ~20 % means the model is consistent.

### all

End-to-end optimizer. Runs `coulomb_x → viscous_x → efficiency_x →
coulomb_theta → viscous_theta → inertia_theta → coulomb_theta` and writes
a `summary.json`. Does not accept `--dry-run`. Prompts for manual
turnarounds the whole way through.

---

## Plotting

`--plot` opens one matplotlib window per trial using the existing
`analysis/visualization/body.py:plot_telem` (the 3×3 grid: pos/vel/accel ×
x/y/θ). Window titles include trial number, candidate value, and score.

Re-plot any old run without re-running the robot:

```bash
.venv/bin/python -c "
import sys; sys.path.insert(0,'analysis/ros_scripts')
from accel_tuning.plot import show_run_plot
show_run_plot('analysis/data/accel_tuning/<run-id>', 'viscous', 'x')
# or per_trial=False for a single concatenated window
"
```

---

## Quick-reference commands

These are the commands actually used during a previous tuning session
(robot 2). Edit `--run-id` and `--baseline-params` as needed.

### 1) Linear coulomb (fresh start)

```bash
python analysis/ros_scripts/accel_model_tune.py coulomb \
    -r 2 --axis x \
    --run-id coulomb_x_fresh \
    --start-accel 0.0 --step 0.01 --max-accel 0.5 \
    --pulse-duration 1.0 --rest-duration 0.5
```

Use `--step 0.005` and/or `--pulse-duration 1.5` for a finer search.

### 2) Verify coulomb-only model at low amplitude

```bash
python analysis/ros_scripts/accel_model_tune.py verify \
    -r 2 --axis x \
    --run-id verify_coulomb_x \
    --baseline-params analysis/data/accel_tuning/coulomb_x_fresh/coulomb/x/result.json \
    --amplitude 0.3 \
    --trials 5 \
    --plot
```

Small amplitude (`0.3`) because the coulomb-only model is only valid near
zero velocity; viscous isn't tuned yet, so don't push hard.

### 3) Linear viscous search at the coulomb baseline

```bash
python analysis/ros_scripts/accel_model_tune.py viscous \
    -r 2 --axis x --mode search \
    --run-id viscous_x_pass1 \
    --baseline-params analysis/data/accel_tuning/coulomb_x_fresh/coulomb/x/result.json \
    --a-min <copy from coulomb result> --v-min <copy from coulomb result> \
    --amplitude 1.5 \
    --motor-efficiency 10.0 \
    --max-iter 10 \
    --plot
```

`--motor-efficiency 10.0` overrides `η` in the loaded baseline (the
controls-lib default of 13.0 is a good starting alternative; 10 is more
conservative for first-pass).

Single-trial confirmation at a specific value:

```bash
python analysis/ros_scripts/accel_model_tune.py viscous \
    -r 2 --axis x --mode single --value 1.69 \
    --run-id viscous_x_confirm \
    --baseline-params <coulomb result.json> \
    --a-min 0.05 --v-min 0.0432 \
    --amplitude 1.5 \
    --motor-efficiency 10.0 \
    --plot
```

### 4) Linear motor-efficiency search

```bash
python analysis/ros_scripts/accel_model_tune.py efficiency \
    -r 2 --axis x --mode search \
    --run-id efficiency_x_pass1 \
    --baseline-params analysis/data/accel_tuning/viscous_x_pass1/viscous/x/result.json \
    --a-min 0.05 --v-min 0.0432 \
    --amplitude 1.5 \
    --initial 10.0 \
    --max-iter 6 \
    --plot
```

### 5) Optional viscous re-tune with tuned η

```bash
python analysis/ros_scripts/accel_model_tune.py viscous \
    -r 2 --axis x --mode search \
    --run-id viscous_x_pass2 \
    --baseline-params analysis/data/accel_tuning/efficiency_x_pass1/efficiency/x/result.json \
    --a-min 0.05 --v-min 0.0432 \
    --amplitude 1.5 \
    --max-iter 8 \
    --plot
```

No `--motor-efficiency` override now — the baseline already has the tuned η.

### 6) Verify tuned linear model

```bash
python analysis/ros_scripts/accel_model_tune.py verify \
    -r 2 --axis x \
    --run-id verify_x_final \
    --baseline-params analysis/data/accel_tuning/viscous_x_pass2/viscous/x/result.json \
    --amplitude 1.5 \
    --trials 6 \
    --plot
```

### 7) Angular coulomb (uses tuned linear baseline)

```bash
python analysis/ros_scripts/accel_model_tune.py coulomb \
    -r 2 --axis theta \
    --run-id coulomb_theta_pass1 \
    --baseline-params analysis/data/accel_tuning/viscous_x_pass2/viscous/x/result.json \
    --start-accel 0.0 --step 0.2 --max-accel 15.0 \
    --pulse-duration 1.0 --rest-duration 0.5
```

### 8) Angular viscous

```bash
python analysis/ros_scripts/accel_model_tune.py viscous \
    -r 2 --axis theta --mode search \
    --run-id viscous_theta_pass1 \
    --baseline-params analysis/data/accel_tuning/coulomb_theta_pass1/coulomb/theta/result.json \
    --a-min <from coulomb_theta> --v-min <from coulomb_theta> \
    --amplitude 3.0 \
    --max-iter 10 \
    --plot
```

### 9) Inertia (theta only)

```bash
python analysis/ros_scripts/accel_model_tune.py inertia \
    -r 2 \
    --run-id inertia_theta_pass1 \
    --baseline-params analysis/data/accel_tuning/viscous_theta_pass1/viscous/theta/result.json \
    --a-min <from coulomb_theta> --v-min <from coulomb_theta> \
    --amplitude 3.0 \
    --max-iter 8 \
    --plot
```

### 10) Verify tuned angular model

```bash
python analysis/ros_scripts/accel_model_tune.py verify \
    -r 2 --axis theta \
    --run-id verify_theta_final \
    --baseline-params analysis/data/accel_tuning/inertia_theta_pass1/inertia/theta/result.json \
    --amplitude 3.0 \
    --trials 5 \
    --plot
```

### Bonus — end-to-end automated run

```bash
python analysis/ros_scripts/accel_model_tune.py all \
    -r 2 \
    --run-id full_tune_2026_06_01 \
    --plot
```

Expect many manual rotations and a long run (≥ 40 trials total). Each
sub-routine still leaves its own `result.json` and `progress.json` under
the run dir.

### Bonus — push a tuned baseline back to the firmware (no search)

```bash
python analysis/ros_scripts/upload_params.py \
    -r 2 \
    -p analysis/data/accel_tuning/verify_x_final/verify/x/result.json
```

`upload_params.py` skips entries that aren't in PARAM_MAP form, so the
`result.json` files (which embed `params_after`) work directly.

---

## Tuning a single-step example: simulate it without driving the robot

```bash
python analysis/ros_scripts/accel_model_tune.py coulomb \
    -r 2 --axis x --dry-run \
    --start-accel 0.0 --step 0.05 --max-accel 0.3
```

`--dry-run` publishes nothing and pushes no params; the state machine
runs and the trial NPZ contains whatever telemetry was arriving. Useful
for verifying the telemetry subscription before risking robot motion.

---

## Operational notes / gotchas

- **Telemetry uses `float32[]` for KF pose/vel and IMU vectors.** The
  parser handles this; older message versions used `.x/.y/.z` structs and
  the parser falls back to those gracefully.
- **KF velocity is global-frame.** Scoring and `v_min` rotate by `−θ_kf`
  per sample to get the local-frame velocity that matches the
  `BCM_LOCAL_ACCEL` command.
- **Vision must be running** for the KF to estimate body velocity at all.
  Without vision, `kf_vel` stays near zero and the scoring degenerates.
- **The score landscape gets noisy at low amplitudes.** At
  `--amplitude 0.5` for linear-x, adjacent candidates 4 % apart in `c_visc`
  can have 2× different scores. Don't over-interpret a single trial; trust
  the aggregate behavior across the search.
- **Coulomb formula `c_coul = η·I·a_min − c_visc·v_min` is recomputed every
  candidate** by viscous / efficiency / inertia routines. So when one of
  those parameters changes, `c_coul` is automatically re-balanced.
  Recomputing assumes `(a_min, v_min)` is still valid — if you suspect the
  robot's static friction changed (different surface, dirty wheels), re-run
  coulomb first.
- **The `y` axis is allowed for diagnostic runs.** It captures telemetry
  and produces a score, but pushes no params to firmware (the firmware
  doesn't separate x/y friction).
- **Resume is exact only if the search params match.** Bounds, tolerance,
  max_iter, and amplitude must match the original run for the resumed
  search to replay the same candidate sequence. If you change those, the
  search restarts (existing cached candidates may still be reused on the
  rare occasions they happen to coincide).

## Starting Generative Prompt

first make sure you load context from here

firmware - /home/nwitten/workspace/firmware/.github/copilot-instructions.md
software - /home/nwitten/workspace/ateam_ws/src/software/.github/copilot-instructions.md
controls - /home/nwitten/workspace/controls/.github/copilot-instructions.md

I'm going to create multiple tuning steps for the accel_model_tune.py routine. We are going to tune the following params

- robot rotational inertia
- motor efficiency factor
- robot friction model

we will not be tuning any trajectories or controllers or controller gains. This is just to tune the feed forward acceleration model.


COULOMB CONSTANT TUNING

For local angular, linear_x, and linear_y we need to find the smallest amount of electrical current to get the robot to move. 

Set motor efficiency to 1.0, coulomb friction to 0.0, viscous friction to 0.0, and robot inertia to default. Now pulse the accel at increasing amplitude until the robot moves. The lowest acceleration (really the electrical current that this invoked) that got the robot moving at constant velocity should be noted. We will build our model off of this value. At extremely low velocity with acceleration set to 0, the coulomb constant should invoke this electrical current.

Now we can always compute the coulomb friction constant from the inertia, motor efficiency, and viscous constant that we are currently using. If accel_min is the accel that we used and v_min is the velocity at accel_min, the coulomb constant should be

motor_efficiency * inertia * accel_min - viscous_constant * v_min

(check that this is consistent with the friction model and tell me if this is wrong)

This is the amount of effort needed to move the robot at it's minimum velocity.


VISCOUS CONSTANT TUNING

Now we need to tune the viscous constant. We'll run a triangular velocity profile on the robot in the local angular, linear_x, and linear_y axes. Recompute the coulomb constant per above on every update of the viscous constant. We are looking for a linear ramp up and ramp down when acceleration is pulsed positive then negative. Define the acceleration amount in a constant that can be updated in code easily. Search for a viscous constant that keeps the triangular profile linear. The acceleration shouldn't slow down or speed up at higher velocities.


MOTOR EFFICIENCY TUNING

After tuning those two params, we can check if our realized acceleration was as expected. We'll need to search for a motor efficiency that gives a triangular profile with a realized acceleration that matches what was commanded.


ROTATIONAL INERTIA TUNING

Once we find a set of parameters that provides a good linear acceleration, we still need to use the same motor efficiency for the angular acceleration. Since we know what the robot mass is, this is not a tunable parameter for the linear acceleration, but the rotational inertia is unknown. Search for a rotational inertia that makes the realized angular acceleration as accurate as the linear model.


SIDE NOTES

The robot firmware currently doesn't support tuning separate local linear_x and linear_y friction models, so just tune linear_x for now.

The motor efficiency is currently used to compensate for non-linearities of the BLDC motor at low velocities. The motor has a stronger torque output than the datasheet's torque constant suggests. It is experimentally determined that a good starting point for the motor efficiency factor is around 13.0. If you start at 1.0, the robot will move way faster than expected.