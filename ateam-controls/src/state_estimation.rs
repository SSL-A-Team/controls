#![allow(unused)]
use libm::{cosf, roundf, sinf};
use core::f32::consts::PI;
use nalgebra::{SMatrix, SVector};

use crate::defaults::{
    EKF_STATE_LEN, EKF_INPUT_LEN, EKF_MEAS_LEN, DEFAULT_CONTROL_DT_US,
    DEFAULT_EKF_R, DEFAULT_EKF_Q, DEFAULT_EKF_CORR_COEF,
    DEFAULT_VISION_ACCEPT_RADIUS_BASE_M, DEFAULT_VISION_ACCEPT_RADIUS_RATE_MPS,
    DEFAULT_VISION_ACCEPT_VARIANCE_M2, DEFAULT_VISION_INACTIVE_THRESHOLD_US,
    DEFAULT_VISION_MAX_AGE_ACCEPTANCE_US, DEFAULT_VISION_MIN_LATENCY_US,
    DEFAULT_VISION_SEED_SAMPLES
};


#[derive(Clone, Copy)]
pub struct VisionSample {
    /// x_m, y_m, w_rad
    pub meas: SVector<f32, 3>,
    pub t_capture_host_us: u64,
}

impl Default for VisionSample {
    fn default() -> Self {
        Self { 
            meas: SVector::<f32, 3>::zeros(),
            t_capture_host_us: 0,
        }
    }
}

#[derive(Clone, Copy)]
struct StateFrame {
    /// Dead reckoned state
    x_reck: SVector<f32, EKF_STATE_LEN>,
    /// IMU control input
    u: SVector<f32, EKF_INPUT_LEN>,
    /// Vision measurement
    z: Option<SVector<f32, EKF_MEAS_LEN>>,
    /// Age of the vision measurement when it was inserted in microseconds
    z_insert_age: u32,
}

impl Default for StateFrame {
    fn default() -> Self {
        StateFrame {
            x_reck: SMatrix::zeros(),
            u: SMatrix::zeros(),
            z: None,
            z_insert_age: 0,
        }
    }
}

#[derive(Clone, Copy)]
pub struct BufferedEKFParams {
    pub dt_us: u32,
    pub ekf_delay_us: u32,
    /// Observation covariance R
    pub r: SMatrix<f32, EKF_MEAS_LEN, EKF_MEAS_LEN>,
    /// Process covariance Q
    pub q: SMatrix<f32, EKF_STATE_LEN, EKF_STATE_LEN>,
    pub corr_coef: f32,
}

impl Default for BufferedEKFParams {
    fn default() -> Self {
        Self {
            dt_us: DEFAULT_CONTROL_DT_US,
            ekf_delay_us: DEFAULT_VISION_MAX_AGE_ACCEPTANCE_US as u32,
            r: DEFAULT_EKF_R,
            q: DEFAULT_EKF_Q,
            corr_coef: DEFAULT_EKF_CORR_COEF,
        }
    }
}

pub struct BufferedEKF<const L: usize> {
    params: BufferedEKFParams,
    buff: [StateFrame; L],
    idx_ekf: usize,
    idx_reck: usize,
    ekf_delay_frames: usize,
    dt_s: f32,
    /// EKF state
    x_ekf: SVector<f32, EKF_STATE_LEN>,
    /// EKF state estimate covariance
    p_ekf: SMatrix<f32, EKF_STATE_LEN, EKF_STATE_LEN>,
    /// Observation jacobian H
    h: SMatrix<f32, EKF_MEAS_LEN, EKF_STATE_LEN>,
    /// Gain for dead reckoning error correction
    corr_gain: f32
}

impl<const L: usize> BufferedEKF<L> {
    /// # Arguments
    ///
    /// * `dt_us` - Update period in microseconds. ```tick()``` must be
    ///   called at this interval.
    /// * `ekf_delay_us` - How far in the past to run the EKF, in microseconds.
    ///   Must be a whole multiple of `dt_us`.
    /// * `r` - EKF observation (measurement) covariance matrix `R`.
    /// * `q` - EKF process covariance matrix `Q`.
    /// * `corr_coef` - Sets the dead-reckoning correction time constant as a
    ///   multiple of `ekf_delay_us` (`tau = corr_coef * ekf_delay_us`):
    ///   * `1` - time constant equals the full EKF delay; the dead-reckoned
    ///     state is ~63% corrected after one delay period, and effectively
    ///     fully corrected after ~3 time constants.
    ///   * `> 1` - longer time constant: slower correction, smoother
    ///     dead reckoning.
    ///   * `< 1` - shorter time constant: faster correction, snappier
    ///     convergence.
    ///
    /// # Panics
    ///
    /// Panics if `ekf_delay_us` is not a whole multiple of `dt_us`, if the
    /// resulting delay in frames does not fit in the buffer (`>= L`), or if
    /// `corr_coef` is not greater than `0`.
    pub fn new(
        params: BufferedEKFParams,
        init_pos: SVector<f32, 3>,
        init_vel: SVector<f32, 3>,
    ) -> Self {
        assert!(params.ekf_delay_us % params.dt_us == 0, "EKF delay in microseconds must be a multiple of update period in microseconds");
        assert!(params.corr_coef > 0., "Correction coefficient must be greater than 0");
        // The buffer needs to have at least one frame in between the current
        // frame and the ekf (horizon) frame so that it's available in the
        // reckon_predict() step
        let ekf_delay_frames = (params.ekf_delay_us / params.dt_us) as usize;
        assert!(ekf_delay_frames < (L - 1), "The specified buffer size is too small for the specified EKF delay");
        let dt_s = (params.dt_us as f32) * 1e-6;
        let h = SMatrix::<f32, EKF_MEAS_LEN, EKF_STATE_LEN>::identity();
        let corr_gain = (params.dt_us as f32) / ((params.ekf_delay_us as f32) * params.corr_coef);
        let mut estimator = BufferedEKF {
            params,
            buff: [StateFrame::default(); L],
            idx_ekf: 0,
            idx_reck: 0,
            ekf_delay_frames,
            dt_s,
            x_ekf: SMatrix::zeros(),
            p_ekf: SMatrix::zeros(),
            h,
            corr_gain,
        };
        estimator.init(init_pos, init_vel);
        
        estimator
    }

    pub fn init(
        &mut self,
        init_pos: SVector<f32, 3>,
        init_vel: SVector<f32, 3>,
    ) {
        // Set the entire buffer to 0
        self.buff = [StateFrame::default(); L];  // reset buffer
        // Initialize buffer indices
        self.idx_ekf = 0;
        self.idx_reck = self.ekf_delay_frames;
        // Initialize the state estimate covariance
        self.p_ekf = SMatrix::<f32, EKF_STATE_LEN, EKF_STATE_LEN>::from_diagonal(
            &SVector::<f32, EKF_STATE_LEN>::from(
                [1000., 1000., PI*PI, 25., 25.]
            )
        );
        // Initialize the EKF state
        self.x_ekf = SVector::<f32, EKF_STATE_LEN>::zeros();
        self.x_ekf.fixed_rows_mut::<3>(0).copy_from(&init_pos);  // px, py, pw
        self.x_ekf.fixed_rows_mut::<2>(3).copy_from(&init_vel.fixed_rows::<2>(0));  // vx, vy
        // Initialize position and velocity in the current buffer window
        for i in self.idx_ekf..(self.idx_reck + 1) {
            // update dead reckon state
            self.buff[i].x_reck.fixed_rows_mut::<3>(0).copy_from(&init_pos);  // px, py, pw
            self.buff[i].x_reck.fixed_rows_mut::<2>(3).copy_from(&init_vel.fixed_rows::<2>(0));  // vx, vy
            // update input
            self.buff[i].u.fixed_rows_mut::<1>(2).copy_from(&init_vel.fixed_rows::<1>(2));  // vw goes into KF input u
        }
    }

    pub fn tick(
        &mut self,
        u: SVector<f32, EKF_INPUT_LEN>,
        z: Option<SVector<f32, EKF_MEAS_LEN>>,
        z_delay_us: u32,  // Microseconds elapsed since the frame that provided this measurement was taken
    ) -> Result<(), ()> {
        // Progress buffer indices
        self.idx_ekf = Self::move_idx(self.idx_ekf, 1, true);
        self.idx_reck = Self::move_idx(self.idx_reck, 1, true);
        // Reset the new frame
        self.buff[self.idx_reck] = StateFrame::default();

        if let Some(meas) = z {
            self.insert_meas(meas, z_delay_us);
        }
        self.reckon_predict(u);
        self.ekf_predict();
        self.ekf_update()?;
        self.reckon_correct();
        Ok(())
    }

    pub fn get_pos(&self) -> SVector<f32, 3> {
        // px, py, pw come from dead reckoned state
        let mut pos: SVector<f32, 3> = self.buff[self.idx_reck].x_reck.fixed_rows::<3>(0).into();
        pos[2] = Self::wrap_turns(pos[2]);

        pos
    }

    pub fn get_vel(&self) -> SVector<f32, 3> {
        let frame_reck = &self.buff[self.idx_reck];
        let mut vel = SVector::<f32, 3>::zeros();
        // vx, vy come from dead reckoned state, rotate local to global
        vel.fixed_rows_mut::<2>(0).copy_from(
            &Self::rotate_xy(&frame_reck.x_reck.fixed_rows::<2>(3).into(), frame_reck.x_reck.z)
        );
        // vw comes from input
        vel[2] = frame_reck.u[2];

        vel
    }

    pub fn get_pos_buff(&self) -> SVector<f32, 3> {
        // px, py, pw come from dead reckoned state
        let mut pos: SVector<f32, 3> = self.x_ekf.fixed_rows::<3>(0).into();
        pos[2] = Self::wrap_turns(pos[2]);

        pos
    }

    pub fn get_vel_buff(&self) -> SVector<f32, 3> {
        let mut vel = SVector::<f32, 3>::zeros();
        // vx, vy come from dead reckoned state, rotate local to global
        vel.fixed_rows_mut::<2>(0).copy_from(
            &Self::rotate_xy(&self.x_ekf.fixed_rows::<2>(3).into(), self.x_ekf.z)
        );
        // vw comes from input
        vel[2] = self.buff[self.idx_ekf].u[2];

        vel
    }

    pub fn get_params(&self) -> BufferedEKFParams {
        self.params
    }

    /// Get the measurement that was applied to the EKF at this instant
    /// Returns:
    ///     None - no measurement was applied
    ///     Some(meas, insert_age, used) - the measurement, it's age on insert
    ///       in microseconds, and whether it was actually used
    pub fn applied_measurement(&self) -> Option<(SVector<f32, EKF_MEAS_LEN>, u32, bool)> {
        let frame = &self.buff[self.idx_ekf];
        frame.z.map(|z| (z, frame.z_insert_age, self.meas_should_be_used(frame.z_insert_age)))
    }

    fn meas_should_be_used(&self, z_insert_age_us: u32) -> bool {
        z_insert_age_us < self.params.ekf_delay_us
    }

    /// Insert the vision measurement at the correct frame in the past
    fn insert_meas(
        &mut self,
        z: SVector<f32, EKF_MEAS_LEN>,
        z_delay_us: u32,
    ) {
        // How many frames have past since the time-of-capture
        let frames_past = ((z_delay_us + self.params.dt_us / 2) / self.params.dt_us) as usize;  // Delay gets rounded to nearest frame
        // clamp it so that it always gets inserted at most on the EKF horizon in the past
        let frames_past = frames_past.min(self.ekf_delay_frames);
        // Get the index in the buffer
        let vision_idx = Self::move_idx(self.idx_reck, frames_past, false);
        // Update that frame with the measurement and its received age
        self.buff[vision_idx].z = Some(z);
        self.buff[vision_idx].z_insert_age = z_delay_us;
    }

    fn reckon_predict(
        &mut self,
        u: SVector<f32, EKF_INPUT_LEN>
    ) {
        let idx_prev = Self::move_idx(self.idx_reck, 1, false);
        let x = self.buff[idx_prev].x_reck;
        let x1 = Self::f_xu(x, u, self.dt_s, None);

        self.buff[self.idx_reck].x_reck.copy_from(&x1);
        self.buff[self.idx_reck].u.copy_from(&u);
    }

    fn reckon_correct(
        &mut self
    ) {
        // Calculate the error between the EKF frame and the reckon frame in the past
        let err = self.buff[self.idx_ekf].x_reck - self.x_ekf;
        // Calculate the correction to apply via the correction gain
        let corr = - self.corr_gain * err;
        // Apply to all frames from now to the EKF frame, working backwards in time
        for i in 0..self.ekf_delay_frames {
            self.buff[Self::move_idx(self.idx_reck, i, false)].x_reck += corr;
        }
    }

    fn ekf_predict(&mut self) {
        let x = self.x_ekf;
        let p = self.p_ekf;
        let u = self.buff[self.idx_ekf].u;
        // F (jacobian evaluated at x, u)
        let mut f = SMatrix::<f32, EKF_STATE_LEN, EKF_STATE_LEN>::zeros();

        // Run state prediction and jacobian evaluation
        let x1 = Self::f_xu(x, u, self.dt_s, Some(&mut f));
        // Update the state covariance
        let p1 = f * p * f.transpose() + self.params.q;

        self.x_ekf.copy_from(&x1);
        self.p_ekf.copy_from(&p1);
    }

    fn ekf_update(
        &mut self,
    ) -> Result<(), ()> {
        let frame = self.buff[self.idx_ekf];
        if let Some(mut z) = frame.z {
            // Only apply if the measurement is as recent as the ekf horizon
            if !self.meas_should_be_used(frame.z_insert_age) {
                return Ok(());
            }
            // Unwrap measurement theta
            z[(2, 0)] = Self::unwrap_turns(z[(2, 0)], self.x_ekf[(2, 0)]);
            // Calculate residual
            let y = z - self.x_ekf.xyz();
            // Calculate residual covariance
            let s = self.h * self.p_ekf * self.h.transpose() + self.params.r;
            // Invert the residual covariance
            let s_inv = s.try_inverse().ok_or_else(|| ())?;
            // Calculate kalman gain
            let k = self.p_ekf * self.h.transpose() * s_inv;
            // Update EKF state
            self.x_ekf = self.x_ekf + k * y;
            self.p_ekf = (SMatrix::<f32, EKF_STATE_LEN, EKF_STATE_LEN>::identity() - k * self.h) * self.p_ekf;
        }

        Ok(())
    }

    fn f_xu(
        x: SVector<f32, EKF_STATE_LEN>,
        u: SVector<f32, EKF_INPUT_LEN>,
        dt: f32,
        jacobian_out: Option<&mut SMatrix<f32, EKF_STATE_LEN, EKF_STATE_LEN>>,
    ) -> SVector<f32, EKF_STATE_LEN> {
        // State vars
        let px = x[(0, 0)];  // global x
        let py = x[(1, 0)];  // global y
        let pw = x[(2, 0)];  // global theta
        let vx = x[(3, 0)];  // local x vel
        let vy = x[(4, 0)];  // local y vel

        // Input vars
        let acc_ax = u[(0, 0)];
        let acc_ay = u[(1, 0)];
        let gyr_vw = u[(2, 0)];

        // Prepare input measurements
        let vw = gyr_vw;  // theta velocity taken directly from gyro
        let ax = acc_ax + vy * vw;  // account for coriolis/transport term for the rotating local frame
        let ay = acc_ay - vx * vw;  // account for coriolis/transport term for the rotating local frame

        // Prepare subexpressions
        let a = pw + 0.5 * dt * vw;  // use midpoint theta between current and next frame
        let cosa = cosf(a);
        let sina = sinf(a);
        let dt2 = dt * dt;

        // Compute the jacobian at provided x, u
        if let Some(jac) = jacobian_out {
            let mut f = SMatrix::<f32, EKF_STATE_LEN, EKF_STATE_LEN>::identity();

            f[(0, 2)] = -ay * cosa * 0.5 * dt2 - vx * sina * dt - vy * cosa * dt - ax * sina * 0.5 * dt2;
            f[(0, 3)] = cosa * dt + vw * sina * 0.5 * dt2;
            f[(0, 4)] = vw * cosa * 0.5 * dt2 - sina * dt;

            f[(1, 2)] = vx * cosa * dt - ay * sina * 0.5 * dt2 - vy * sina * dt + ax * cosa * 0.5 * dt2;
            f[(1, 3)] = sina * dt - vw * cosa * 0.5 * dt2;
            f[(1, 4)] = cosa * dt + vw * sina * 0.5 * dt2;

            f[(3, 4)] = vw * dt;

            f[(4, 3)] = -vw * dt;

            (*jac).copy_from(&f);
        }

        // State update integration
        SVector::<f32, EKF_STATE_LEN>::from([
            px + (cosa * vx - sina * vy) * dt + 0.5 * (cosa * ax - sina * ay) * dt2,  // local vel and acc are rotated
            py + (sina * vx + cosa * vy) * dt + 0.5 * (sina * ax + cosa * ay) * dt2,  // local vel and acc are rotated
            pw + dt * vw,
            vx + dt * ax,
            vy + dt * ay,
        ])
    }

    fn unwrap_turns(
        a: f32,  // Angle to be unwrapped
        a_ref: f32,  // Reference angle to wrap closest to
    ) -> f32 {
        a + roundf((a_ref - a) / (2. * PI)) * 2. * PI
    }

    fn wrap_turns(
        a: f32,
    ) -> f32 {
        let mut a_wrap = (a + PI) % (2. * PI);
        if a_wrap.is_sign_negative() {
            a_wrap += 2. * PI;
        }
        a_wrap - PI
    }

    fn move_idx(
        i: usize,  // base index
        di: usize,  // change in index
        forward: bool,  // move forward in the buffer (+time)
    ) -> usize {
        if forward {
            (i + di) % L
        } else {
            (i + L - di) % L
        }
    }

    fn rotate_xy(
        xy: &SVector<f32, 2>,
        a: f32,
    ) -> SVector<f32, 2> {
        let cosa = cosf(a);
        let sina = sinf(a);
        SVector::<f32, 2>::new(
            xy.x * cosa - xy.y * sina,
            xy.x * sina + xy.y * cosa,
        )
    }
}

impl<const L: usize> Default for BufferedEKF<L> {
    fn default() -> Self {
        Self::new(
            BufferedEKFParams::default(),
            SVector::<f32, 3>::zeros(),
            SVector::<f32, 3>::zeros(),
        )
    }
}

pub enum VisionSampleReject {
    MalformedTimestamp,
    OutOfOrder,
    Seeding,
    VarianceThreshold,
    DistanceThreshold,
    AgeThreshold,
}

pub enum VisionSampleAccept {
    TeleportState,
    UpdateState,
}

pub enum VisionSampleAction {
    None,
    Reject(VisionSampleReject),
    Accept(
        VisionSampleAccept,
        // Measurement values (px, py, pw)
        SVector<f32, EKF_MEAS_LEN>,
        // Measurement age in microseconds
        u64,
    ),
}

#[derive(Clone, Copy)]
struct SeedState {
    seed_samples_ct: usize,
    sum: SVector<f32, 2>,
    sum2: SVector<f32, 2>,
}

impl Default for SeedState {
    fn default() -> Self {
        Self {
            seed_samples_ct: 0,
            sum: SVector::<f32, 2>::zeros(),
            sum2: SVector::<f32, 2>::zeros(),
        }
    }
}

enum VisionFilterState {
    Seeding(SeedState),
    SignalActive,
}

#[derive(Clone, Copy)]
pub struct VisionFilterParams {
    pub dt_us: u32,
    pub accept_radius_base_m: f32,
    pub accept_radius_rate_mps: f32,
    pub accept_variance_m2: f32,
    pub seed_samples: usize,
    pub min_latency_us: u64,
    pub max_age_acceptance_us: u64,
    pub inactive_threshold_us: u64,
}

impl Default for VisionFilterParams {
    fn default() -> Self {
        Self {
             dt_us: DEFAULT_CONTROL_DT_US,
             accept_radius_base_m: DEFAULT_VISION_ACCEPT_RADIUS_BASE_M,
             accept_radius_rate_mps: DEFAULT_VISION_ACCEPT_RADIUS_RATE_MPS,
             accept_variance_m2: DEFAULT_VISION_ACCEPT_VARIANCE_M2,
             seed_samples: DEFAULT_VISION_SEED_SAMPLES,
             min_latency_us: DEFAULT_VISION_MIN_LATENCY_US,
             max_age_acceptance_us: DEFAULT_VISION_MAX_AGE_ACCEPTANCE_US,
             inactive_threshold_us: DEFAULT_VISION_INACTIVE_THRESHOLD_US,
        }
    }
}

pub struct VisionFilter<const L: usize> {
    params: VisionFilterParams,
    state: VisionFilterState,
    /// time delta between calls to ```tick()``` in seconds
    dt_s: f32,
    /// Buffer holding each recent sample's measured clock skew
    meas_skew_buff: [u64; L],
    /// Buffer index of the next open slot in the buffer
    buff_idx: usize,
    /// Flag to indicate that the buffer is full of recent valid samples
    buff_full: bool,
    /// Last sample rx absolute robot time in microseconds
    t_last_sample_us: u64,
    /// Last sample capture absolute vision host time in microseconds
    t_last_sample_host_us: u64,
    /// Last sample accept absolute robot time in microseconds
    t_last_accept_us: u64,
    /// Maximum radius from current state for transition to active signal. Grows with time.
    accept_radius_m: f32,
}

impl<const L: usize> VisionFilter<L> {
    pub fn new(
        params: VisionFilterParams,
    ) -> VisionFilter<L> {
        VisionFilter {
            params,
            state: VisionFilterState::Seeding(SeedState::default()),
            dt_s: params.dt_us as f32 * 1e-6,
            meas_skew_buff: [0; L],
            buff_idx: 0,
            buff_full: false,
            t_last_sample_us: 0,
            t_last_sample_host_us: 0,
            t_last_accept_us: 0,
            accept_radius_m: params.accept_radius_base_m,
        }
    }

    pub fn reset(
        &mut self
    ) {
        self.reset_buff();
        self.reset_state();
        self.reset_times_and_radius();
    }

    pub fn signal_active(&self) -> bool {
        matches!(self.state, VisionFilterState::SignalActive)
    }

    /// # Arguments
    /// * `t_us` - current absolute robot time in microseconds
    /// * `pos_est` - current position estimate
    /// * `sample` - optional vision sample to feed into the vision filter
    pub fn tick(
        &mut self,
        t_us: u64,
        pos_est: &SVector<f32, 3>,
        sample: Option<&VisionSample>,
    ) -> VisionSampleAction {
        // Default action and accept type
        let mut action = VisionSampleAction::None;

        // Reset on inactivity
        self.check_inactive(t_us);

        // Increase acceptance radius if currently seeding
        if matches!(self.state, VisionFilterState::Seeding(_)) {
            self.accept_radius_m += self.params.accept_radius_rate_mps * self.dt_s;
        }

        // Handle a new sample
        if let Some(new_sample) = sample {
            // Ensure this sample came in order and return early if out of order, throwing this sample out
            if !self.sample_in_order(new_sample) {
                action = VisionSampleAction::Reject(VisionSampleReject::OutOfOrder);
                return action;
            }

            // Update last sample times
            self.t_last_sample_us = t_us;
            self.t_last_sample_host_us = new_sample.t_capture_host_us;

            // Calculate measured clock skew
            // meas_skew = skew - min_latency - jitter_latency
            if (t_us > new_sample.t_capture_host_us) {
                action = VisionSampleAction::Reject(VisionSampleReject::MalformedTimestamp);
                return action;
            }
            let meas_skew = new_sample.t_capture_host_us - t_us;

            // Insert into buffer, update flag when index wraps back to the
            // beginning of the buffer
            self.meas_skew_buff[self.buff_idx] = meas_skew;
            let next_idx = Self::move_idx(self.buff_idx, 1, true);
            if !self.buff_full && next_idx < self.buff_idx{
                self.buff_full = true;
            }
            self.buff_idx = next_idx;
            

            // Update a currently seeding state
            // note: this action will get overwritten in the signal_active
            // state update below if the seed just completed
            action = self.update_seed_state(
                new_sample,
                pos_est,
            );

            // Update an active signal state
            if self.signal_active() {

                let sample_age_us = self.estimate_sample_age(t_us, new_sample);
                if sample_age_us < self.params.max_age_acceptance_us {

                    // distance check
                    let dist_m = Self::sample_distance(new_sample, pos_est);
                    // special case if the seed just completed and the sample should be accepted with teleport
                    let accept_type = if matches!(action, VisionSampleAction::Accept(VisionSampleAccept::TeleportState, _, _)) {
                        VisionSampleAccept::TeleportState
                    } else {
                        VisionSampleAccept::UpdateState
                    };

                    if matches!(accept_type, VisionSampleAccept::TeleportState) ||
                        dist_m < self.accept_radius_m {
                        action = VisionSampleAction::Accept(
                            accept_type, 
                            new_sample.meas,
                            sample_age_us,
                        );
                        self.t_last_accept_us = t_us;
                    } else {
                        action = VisionSampleAction::Reject(VisionSampleReject::DistanceThreshold);
                    }
                } else {
                    action = VisionSampleAction::Reject(VisionSampleReject::AgeThreshold);
                }
            }
        }

        action
    }

    pub fn get_params(&self) -> VisionFilterParams {
        self.params
    }

    fn reset_buff(&mut self) {
        self.meas_skew_buff = [0; L];
        self.buff_idx = 0;
        self.buff_full = false;
    }

    fn reset_state(&mut self) {
        self.state = VisionFilterState::Seeding(SeedState::default());
    }

    fn reset_times_and_radius(&mut self) {
        self.t_last_sample_us = 0;
        self.t_last_sample_host_us = 0;
        self.t_last_accept_us = 0;
        self.accept_radius_m = self.params.accept_radius_base_m;
    }

    fn check_inactive(&mut self, t_us: u64) {
        match self.state {
            VisionFilterState::Seeding(seed_state) => {
                // Check if inactivity should reset filter state
                if seed_state.seed_samples_ct > 0 &&
                    (t_us - self.t_last_sample_us) > self.params.inactive_threshold_us {
                    self.reset();
                }
            },
            VisionFilterState::SignalActive => {
                // Check if inactivity should reset filter state
                if (t_us - self.t_last_accept_us) > self.params.inactive_threshold_us {
                    self.reset();
                }
            },
        }
    }

    fn sample_in_order(&self, sample: &VisionSample) -> bool {
        if let VisionFilterState::Seeding(seed_state) = self.state {
            if seed_state.seed_samples_ct == 0 {
                // This is the first sample since filter init/reset
                return true;
            }
        }
        // Check if the timestamp on this sample is greater than the last one
        sample.t_capture_host_us > self.t_last_sample_host_us
    }

    fn update_seed_state(&mut self,
        sample: &VisionSample,
        pos_est: &SVector<f32, 3>,
    ) -> VisionSampleAction {
        let mut action = VisionSampleAction::None;

        if let VisionFilterState::Seeding(seed_state) = &mut self.state {

            // State metric updates
            seed_state.sum += sample.meas.xy();
            seed_state.sum2 += sample.meas.xy().component_mul(&sample.meas.xy());
            seed_state.seed_samples_ct += 1;

            // If seed count threshold is hit, check if the sample can be accepted
            if (seed_state.seed_samples_ct >= self.params.seed_samples) {

                // Distance check
                let dist_m = Self::sample_distance(sample, pos_est);
                if dist_m < self.accept_radius_m {

                    // Variance check
                    let n = SVector::<f32, 2>::new(seed_state.seed_samples_ct as f32, seed_state.seed_samples_ct as f32);
                    let e_x = seed_state.sum.component_div(&n);
                    let e_x2 = seed_state.sum2.component_div(&n);
                    let e_x_2 = e_x.component_mul(&e_x);
                    let var_x = e_x2 - e_x_2;
                    if var_x.x < self.params.accept_variance_m2 &&
                    var_x.y < self.params.accept_variance_m2 {
                        action = VisionSampleAction::Accept(
                            VisionSampleAccept::TeleportState,
                            sample.meas,
                            0,  // age unknown, will be set later
                        );
                    } else {
                        action = VisionSampleAction::Reject(VisionSampleReject::VarianceThreshold);
                    }

                } else {
                    action = VisionSampleAction::Reject(VisionSampleReject::DistanceThreshold);
                }
            } else {
                action = VisionSampleAction::Reject(VisionSampleReject::Seeding);
            }

            match action {
                VisionSampleAction::Reject(VisionSampleReject::DistanceThreshold) |
                VisionSampleAction::Reject(VisionSampleReject::VarianceThreshold) => {
                    // Only reset buffer and seed state, let accept radius continue to expand
                    self.reset_buff();
                    self.reset_state();
                },
                _ => {},
            }

            // If the seed completed, set signal to active and update acceptance radius
            if matches!(action, VisionSampleAction::Accept(..)) {
                // set signal active
                self.state = VisionFilterState::SignalActive;
                // shrink acceptance radius back to the base
                self.accept_radius_m = self.params.accept_radius_base_m;
            }
        }

        action
    }

    fn estimate_sample_age(&self, t_us: u64, sample: &VisionSample) -> u64 {
        // Get the max measured skew from the buffer
        let end_idx = if self.buff_full {
            L
        } else {
            self.buff_idx
        };
        let max_meas_skew_us = self.meas_skew_buff[0..end_idx].iter().max().expect(
                "VisionFilter: failed to find maximum clock skew - measured clock skew buffer was unexpectedly empty"
            );

        // meas_skew = skew - min_latency - jitter_latency
        // jitter_latency assumed 0 for the max_meas_skew
        // skew = max_meas_skew + min_latency
        let clock_skew_us = max_meas_skew_us + self.params.min_latency_us;
        let t_host_us = t_us + clock_skew_us;

        t_host_us - sample.t_capture_host_us
    }

    fn sample_distance(sample: &VisionSample, pos_est: &SVector<f32, 3>) -> f32 {
        (sample.meas.xy() - pos_est.xy()).norm()
    }

    fn move_idx(
        i: usize,  // base index
        di: usize,  // change in index
        forward: bool,  // move forward in the buffer (+time)
    ) -> usize {
        if forward {
            (i + di) % L
        } else {
            (i + L - di) % L
        }
    }
}

impl<const L: usize> Default for VisionFilter<L> {
    fn default() -> Self {
        Self::new(
            VisionFilterParams::default()
        )
    }
}

pub struct StateEstimator<const L: usize, const K: usize> {
    pub ekf: BufferedEKF<L>,
    pub vision_filter: VisionFilter<K>,
    // odometer: Odometer,
}

impl<const L: usize, const K: usize> StateEstimator<L, K> {

    pub fn new(
        ekf: BufferedEKF<L>,
        vision_filter: VisionFilter<K>,
    ) -> Self {
        // Check that the vision filter max_age_acceptance is equal or less than EKF buffer size
        assert!(ekf.params.ekf_delay_us as u64 >= vision_filter.params.max_age_acceptance_us, "EKF buffer should be larger than the maximum acceptable vision sample age in the vision filter");
        Self {
            ekf,
            vision_filter,
        }
    }

    pub fn tick(
        &mut self,
        t_us: u64,
        imu: &SVector<f32, EKF_INPUT_LEN>,  // acc_x_mps2, acc_y_mps2, gyro_w_radps
        encoder: &SVector<f32, 4>,  // fl_radps, bl_radps, br_radps, fr_radps
        vision: Option<&VisionSample>,
    ) -> Result<(), ()> {

        // Feed vision sample through the vision filter
        let (mut z, mut z_age_us) = match self.vision_filter.tick(
            t_us,
            &self.ekf.get_pos(),
            vision,
        ) {
            VisionSampleAction::Accept(accept_type, meas, meas_age_us) => {
                match accept_type {
                    VisionSampleAccept::TeleportState => {
                        self.ekf.init(
                            meas,
                            SVector::<f32, 3>::zeros(),
                        )
                    },
                    VisionSampleAccept::UpdateState => {},
                };
                (Some(meas), meas_age_us)
            },
            VisionSampleAction::Reject(reject) => {
                (None, 0)
            },
            VisionSampleAction::None => {
                (None, 0)
            },
        };

        if (!self.vision_active()) {
            // TODO: Uncomment after odometer is implemented
            // if (!self.odometer.active()) {
            //     self.odometer.start(self.ekf.get_pos());
            // }
            // self.odometer.tick(encoder);
            // z = Some(self.odometer.get_pos());
            // z_age_us = 0;
        }

        self.ekf.tick(
            *imu,
            z,
            z_age_us as u32,
        )?;

        Ok(())
    }

    pub fn init(
        &mut self,
        init_pos: SVector<f32, 3>,
        init_vel: SVector<f32, 3>,
    ) {
        self.ekf.init(init_pos, init_vel);
        self.vision_filter.reset();
    }

    pub fn vision_active(&self) -> bool {
        self.vision_filter.signal_active()
    }

    pub fn get_pos(&self) -> SVector<f32, 3> {
        self.ekf.get_pos()
    }

    pub fn get_vel(&self) -> SVector<f32, 3> {
        self.ekf.get_vel()
    }

    pub fn get_pos_buff(&self) -> SVector<f32, 3> {
        self.ekf.get_pos_buff()
    }

    pub fn get_vel_buff(&self) -> SVector<f32, 3> {
        self.ekf.get_vel_buff()
    }

    pub fn applied_vision(&self) -> Option<(SVector<f32, 3>, u32, bool)> {
        self.ekf.applied_measurement()
    }
}

impl<const K: usize, const L: usize> Default for StateEstimator<K, L> {
    fn default() -> Self {
        let ekf = BufferedEKF::default();
        let vision_filter = VisionFilter::default();
        Self::new(ekf, vision_filter)
    }
}

// struct Odometer {
// }

// impl Odometer {
//     pub fn start(
//         &self,
//         start_pos: SVector<f32, 3>,
//     ) {
//         todo!()
//     }

//     pub fn tick(
//         &self,
//         enc: &EncoderSample,
//     ) {
//         todo!()
//     }

//     pub fn get_pos(&self) -> SVector<f32, MEAS_LEN> {
//         todo!()
//     }

//     pub fn active(&self) -> bool {
//         todo!()
//     }
// }


#[cfg(test)]
mod tests {
    #[test]
    fn test1() {
        assert!(true);
    }
}
