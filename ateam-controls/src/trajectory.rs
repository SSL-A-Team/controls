use crate::{Vector3f, Vector6f};

/// A self-contained trajectory that owns its time state.
///
/// After construction, call `tick(dt)` each control period to advance the
/// trajectory's internal clock, then `sample()` to read the current state and
/// acceleration. Internally t = 0 always means "right now".
pub trait Trajectory {
    /// Advance the trajectory's internal clock by `dt` seconds.
    fn tick(&mut self, dt: f32);

    /// Sample the trajectory at the current internal time.
    ///
    /// Returns `(state, accel)` where:
    /// - `state` is `[x, y, θ, ẋ, ẏ, θ̇]` at t = 0 (current time).
    /// - `accel` is `[ẍ, ÿ, θ̈]` commanded at t = 0.
    fn sample(&self) -> (Vector6f, Vector3f);
}
