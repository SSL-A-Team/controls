#![allow(unused)]
use libm::{cosf, roundf, sinf};
use core::f32::consts::PI;
use nalgebra::{SMatrix, SVector};

use crate::defaults::{DEFAULT_VISION_INACTIVE_THRESHOLD_US, DEFAULT_VISION_MAX_AGE_ACCEPTANCE_US, DEFAULT_VISION_MAX_AGE_TIME_SYNC_US, DEFAULT_VISION_MAX_AGE_VARIANCE_US, DEFAULT_VISION_MIN_LATENCY_US, DEFAULT_VISION_MIN_SAMPLES_TIME_SYNC, DEFAULT_VISION_MIN_SAMPLES_VARIANCE};


pub const STATE_LEN: usize = 5;
pub const INPUT_LEN: usize = 3;
pub const MEAS_LEN: usize = 3;


#[derive(Clone, Copy)]
pub struct VisionSample {
    x_m: f32,
    y_m: f32,
    w_rad: f32,
    t_capture_us: u128,
    t_rx_us: u64,
}

pub struct ImuSample {
    x_acc_mps2: f32,
    y_acc_mps2: f32,
    w_gyro_radps: f32,
}

pub struct EncoderSample {
    fl_radps: f32,
    bl_radps: f32,
    br_radps: f32,
    fr_radps: f32,
}

impl Default for VisionSample {
    fn default() -> Self {
        Self { 
            x_m: 0.,
            y_m: 0.,
            w_rad: 0.,
            t_capture_us: 0,
            t_rx_us: 0
        }
    }
}

#[derive(Clone, Copy)]
struct StateFrame {
    /// EKF state
    x_ekf: SVector<f32, STATE_LEN>,
    /// EKF state estimate covariance
    p_ekf: SMatrix<f32, STATE_LEN, STATE_LEN>,
    /// Dead reckoned state
    x_reck: SVector<f32, STATE_LEN>,
    /// IMU control input
    u: SVector<f32, INPUT_LEN>,
    /// Vision measurement
    z: Option<SVector<f32, MEAS_LEN>>,
}

impl Default for StateFrame {
    fn default() -> Self {
        StateFrame {
            x_ekf: SMatrix::zeros(),
            p_ekf: SMatrix::zeros(),
            x_reck: SMatrix::zeros(),
            u: SMatrix::zeros(),
            z: None,
        }
    }
}

pub struct BufferedEKF<const L: usize> {
    buff: [StateFrame; L],
    idx_ekf: usize,
    idx_reck: usize,
    ekf_delay_frames: usize,
    dt_us: u32,
    dt_s: f32,
    /// Observation covariance R
    r: SMatrix<f32, MEAS_LEN, MEAS_LEN>,
    /// Process covariance Q
    q: SMatrix<f32, STATE_LEN, STATE_LEN>,
    /// Observation jacobian H
    h: SMatrix<f32, MEAS_LEN, STATE_LEN>,
    /// Gain for dead reckoning error correction
    corr_gain: f32
}

impl<const L: usize> BufferedEKF<L> {
    /// Creates a new buffered EKF with a delayed-measurement fusion horizon.
    ///
    /// The EKF runs `ekf_delay_us` in the past so that delayed vision
    /// measurements can be applied at their true time of capture, while a
    /// dead-reckoning estimate is tracked forward to the present.
    ///
    /// # Arguments
    ///
    /// * `dt_us` - Update period in microseconds. [`tick`](Self::tick) must be
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
        dt_us: u32,
        ekf_delay_us: u32,
        r: SMatrix<f32, MEAS_LEN, MEAS_LEN>,
        q: SMatrix<f32, STATE_LEN, STATE_LEN>,
        corr_coef: f32,
        init_pos: SVector<f32, 3>,
        init_vel: SVector<f32, 3>,
    ) -> Self {
        assert!(ekf_delay_us % dt_us == 0, "EKF delay in microseconds must be a multiple of update period in microseconds");
        assert!(corr_coef > 0., "Correction coefficient must be greater than 0");
        let ekf_delay_frames = (ekf_delay_us / dt_us) as usize;
        assert!(ekf_delay_frames < L, "EKF delay is too large for buffer size");
        let dt_s = (dt_us as f32) * 1e-6;
        let h = SMatrix::<f32, MEAS_LEN, STATE_LEN>::identity();
        let corr_gain = (dt_us as f32) / ((ekf_delay_us as f32) * corr_coef);
        let mut estimator = BufferedEKF {
            buff: [StateFrame::default(); L],
            idx_ekf: 0,
            idx_reck: 0,
            ekf_delay_frames,
            dt_us,
            dt_s,
            r,
            q,
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
        self.buff = unsafe{ core::mem::zeroed() };  // reset all bytes in the buffer to 0's
        // Initialize buffer indices
        self.idx_ekf = 0;
        self.idx_reck = self.ekf_delay_frames;
        // Initialize the state estimate covariance
        self.buff[self.idx_ekf].p_ekf = SMatrix::<f32, STATE_LEN, STATE_LEN>::from_diagonal(
            &SVector::<f32, STATE_LEN>::from(
                [1000., 1000., PI*PI, 25., 25.]
            )
        );
        // Initialize position and velocity in the current buffer window
        for i in self.idx_ekf..(self.idx_reck + 1) {
            // update ekf state
            self.buff[i].x_ekf.fixed_rows_mut::<3>(0).copy_from(&init_pos);  // px, py, pw
            self.buff[i].x_ekf.fixed_rows_mut::<2>(3).copy_from(&init_vel.fixed_rows::<2>(0));  // vx, vy
            // update dead reckon state
            self.buff[i].x_reck.fixed_rows_mut::<3>(0).copy_from(&init_pos);  // px, py, pw
            self.buff[i].x_reck.fixed_rows_mut::<2>(3).copy_from(&init_vel.fixed_rows::<2>(0));  // vx, vy
            // update input
            self.buff[i].u.fixed_rows_mut::<1>(2).copy_from(&init_vel.fixed_rows::<1>(2));  // vw goes into KF input u
        }
    }

    pub fn tick(
        &mut self,
        u: SVector<f32, INPUT_LEN>,
        z: Option<SVector<f32, MEAS_LEN>>,
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

    pub fn get_ekf_pos(&self) -> SVector<f32, 3> {
        // px, py, pw come from dead reckoned state
        let mut pos: SVector<f32, 3> = self.buff[self.idx_ekf].x_ekf.fixed_rows::<3>(0).into();
        pos[2] = Self::wrap_turns(pos[2]);

        pos
    }

    pub fn get_ekf_vel(&self) -> SVector<f32, 3> {
        let frame_ekf = &self.buff[self.idx_ekf];
        let mut vel = SVector::<f32, 3>::zeros();
        // vx, vy come from dead reckoned state, rotate local to global
        vel.fixed_rows_mut::<2>(0).copy_from(
            &Self::rotate_xy(&frame_ekf.x_ekf.fixed_rows::<2>(3).into(), frame_ekf.x_ekf.z)
        );
        // vw comes from input
        vel[2] = frame_ekf.u[2];

        vel
    }

    /// Insert the vision measurement at the correct frame in the past
    fn insert_meas(
        &mut self,
        z: SVector<f32, MEAS_LEN>,
        z_delay_us: u32,
    ) {
        // How many frames have past since the time-of-capture
        let frames_past = ((z_delay_us + self.dt_us / 2) / self.dt_us) as usize;  // Delay gets rounded to nearest frame
        // If it's further in the past than the EKF delay, throw it out
        if frames_past > self.ekf_delay_frames {
            return;
        }
        // Get the index in the buffer
        let vision_idx = Self::move_idx(self.idx_reck, frames_past, false);
        // Update that frame with the measurement
        self.buff[vision_idx].z = Some(z);
    }

    fn reckon_predict(
        &mut self,
        u: SVector<f32, INPUT_LEN>
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
        let err = self.buff[self.idx_ekf].x_reck - self.buff[self.idx_ekf].x_ekf;
        // Calculate the correction to apply via the correction gain
        let corr = - self.corr_gain * err;
        // Apply to all frames from now to the EKF frame, working backwards in time
        for i in 0..self.ekf_delay_frames {
            self.buff[Self::move_idx(self.idx_reck, i, false)].x_reck += corr;
        }
    }

    fn ekf_predict(&mut self) {
        let idx_prev = Self::move_idx(self.idx_ekf, 1, false);
        let x = self.buff[idx_prev].x_ekf;
        let p = self.buff[idx_prev].p_ekf;
        let u = self.buff[self.idx_ekf].u;
        // F (jacobian evaluated at x, u)
        let mut f = SMatrix::<f32, STATE_LEN, STATE_LEN>::zeros();

        // Run state prediction and jacobian evaluation
        let x1 = Self::f_xu(x, u, self.dt_s, Some(&mut f));
        // Update the state covariance
        let p1 = f * p * f.transpose() + self.q;

        self.buff[self.idx_ekf].x_ekf.copy_from(&x1);
        self.buff[self.idx_ekf].p_ekf.copy_from(&p1);
    }

    fn ekf_update(
        &mut self,
    ) -> Result<(), ()> {
        let frame = &mut self.buff[self.idx_ekf];
        if let Some(mut z) = frame.z {
            // Unwrap measurement theta
            z[(2, 0)] = Self::unwrap_turns(z[(2, 0)], frame.x_ekf[(2, 0)]);
            // Calculate residual
            let y = z - frame.x_ekf.xyz();
            // Calculate residual covariance
            let s = self.h * frame.p_ekf * self.h.transpose() + self.r;
            // Invert the residual covariance
            let s_inv = s.try_inverse().ok_or_else(|| ())?;
            // Calculate kalman gain
            let k = frame.p_ekf * self.h.transpose() * s_inv;
            // Update EKF state
            frame.x_ekf = frame.x_ekf + k * y;
            frame.p_ekf = (SMatrix::<f32, STATE_LEN, STATE_LEN>::identity() - k * self.h) * frame.p_ekf;
        }

        Ok(())
    }

    fn f_xu(
        x: SVector<f32, STATE_LEN>,
        u: SVector<f32, INPUT_LEN>,
        dt: f32,
        jacobian_out: Option<&mut SMatrix<f32, STATE_LEN, STATE_LEN>>,
    ) -> SVector<f32, STATE_LEN> {
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
            let mut f = SMatrix::<f32, STATE_LEN, STATE_LEN>::identity();

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
        SVector::<f32, STATE_LEN>::from([
            px + (cosa * vx - sina * vy) * dt + 0.5 * (cosa * ax - sina * ay) * dt2,  // local vel and acc are rotated
            py + (sina * vx + cosa * vy) * dt + 0.5 * (sina * ax + cosa * ay) * dt2,  // local vel and acc are rotated
            pw + dt * vw,
            vx + dt * ax,
            vy + dt * ay,
        ])
    }

    #[inline(always)]
    fn unwrap_turns(
        a: f32,  // Angle to be unwrapped
        a_ref: f32,  // Reference angle to wrap closest to
    ) -> f32 {
        a + roundf((a_ref - a) / (2. * PI)) * 2. * PI
    }

    #[inline(always)]
    fn wrap_turns(
        a: f32,
    ) -> f32 {
        let mut a_wrap = (a + PI) % (2. * PI);
        if a_wrap.is_sign_negative() {
            a_wrap += 2. * PI;
        }
        a_wrap - PI
    }

    #[inline(always)]
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

    #[inline(always)]
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

enum VisionSampleReject {
    Seeding,
    DistanceThreshold,
    VarianceThreshold,
    OutOfOrder,
    TooOld,
}

enum VisionSampleAccept {
    SnapState,
    UpdateState,
}

enum VisionSampleAction {
    None,
    Reject(VisionSampleReject),
    Accept(
        VisionSampleAccept,
        SVector<f32, MEAS_LEN>,
        u64,
    ),
}

struct VisionFilter<const L: usize> {
    buff: [Option<VisionSample>; L],
    buff_idx: usize,
    /// Estimated minimum possible latency from camera capture time to robot vision packet receive time
    min_latency_us: u32,
    /// Minimum number of samples to compute variance before accepting measurements
    min_samples_variance: usize,
    /// Maximum age of samples to compute variance before accepting measurements
    max_age_variance_us: u32,
    /// Minimum number of samples to compute robot time offset with vision source
    min_samples_time_sync: usize,
    /// Maximum age of samples to compute robot time offset with vision source
    max_age_time_sync_us: u32,
    /// Maximum age of a sample for measurement acceptance
    max_age_acceptance_us: u32,
    /// Duration until vision signal is considered inactive
    inactive_threshold_us: u32,
    /// Last sample accept absolute time in microseconds
    last_sample_accept_us: u64,
    signal_active: bool,
}

impl<const L: usize> Default for VisionFilter<L> {
    fn default() -> Self {
        Self::new(
            DEFAULT_VISION_MIN_LATENCY_US,
            DEFAULT_VISION_MIN_SAMPLES_VARIANCE,
            DEFAULT_VISION_MAX_AGE_VARIANCE_US,
            DEFAULT_VISION_MIN_SAMPLES_TIME_SYNC,
            DEFAULT_VISION_MAX_AGE_TIME_SYNC_US,
            DEFAULT_VISION_MAX_AGE_ACCEPTANCE_US,
            DEFAULT_VISION_INACTIVE_THRESHOLD_US,
        )
    }
}

impl<const L: usize> VisionFilter<L> {
    fn new(
        min_latency_us: u32,
        min_samples_variance: usize,
        max_age_variance_us: u32,
        min_samples_time_sync: usize,
        max_age_time_sync_us: u32,
        max_age_acceptance_us: u32,
        inactive_threshold_us: u32,
    ) -> VisionFilter<L> {
        // Check that min samples is less than max_age * 30hz vision for variance and time sync (sample count is achievable in the time window, accounting for dropped packets)
        // Check that max age for time sync is greater than max age for variance (we can drop samples after time sync max age)
        // Check that max_age_time_sync_us / inactive_threshold_us > min_samples_time_sync (as long as signal is active, we have enough time sync samples)
        VisionFilter {
            buff: [None; L],
            buff_idx: 0,
            min_latency_us,
            min_samples_variance,
            max_age_variance_us,
            min_samples_time_sync,
            max_age_time_sync_us,
            max_age_acceptance_us,
            inactive_threshold_us,
            last_sample_accept_us: 0,
            signal_active: false,
        }
    }

    /// # Arguments
    /// * `t_us` - current absolute robot time in microseconds
    /// * `pos_est` - current position estimate
    /// * `sample` - optional vision sample to feed into the vision filter
    pub fn tick(
        &mut self,
        t_us: u64,
        pos_est: SVector<f32, 3>,
        sample: Option<&VisionSample>,
    ) -> VisionSampleAction {
        let mut action = VisionSampleAction::None;

        if let Some(new_sample) = sample {
            if let Some(head_sample) = self.buff[self.buff_idx] {
                // ensure this sample is the latest, otherwise throw it away
                if new_sample.t_capture_us > head_sample.t_capture_us {
                    // insert into buffer at the head
                    self.buff_idx = Self::move_idx(self.buff_idx, 1, true);
                    self.buff[self.buff_idx] = Some(*new_sample);
                } else {
                    // this is older than the current buffer head
                    action = VisionSampleAction::Reject(VisionSampleReject::OutOfOrder);
                }
            }
        }

        // if (t_us - tail_sample.t_rx_us) > self.max_age_time_sync_us

        // Loop through buffer to get required metrics
        let mut idx = self.buff_idx;
        let mut meas_sum = SVector::<f32, MEAS_LEN>::zeros();
        let mut meas_sq_sum = SVector::<f32, MEAS_LEN>::zeros();
        let mut variance_samples = 0;
        let mut min_t_diff = u128::MAX;
        let mut time_sync_samples = 0;
        while let Some(sample) = self.buff[idx] {
            // Metrics for variance check before measurement acceptance
            if sample.t_rx_us > (t_us - self.max_age_variance_us as u64) {
                let meas_vec: SVector<f32, MEAS_LEN> = SVector::<f32, MEAS_LEN>::new(
                    sample.x_m,
                    sample.y_m,
                    sample.w_rad,
                );
                meas_sum += meas_vec;
                meas_sq_sum += meas_vec.component_mul(&meas_vec);
                variance_samples += 1;
            }

            // Metrics for time sync with vision clock
            if sample.t_rx_us > (t_us - self.max_age_time_sync_us as u64) {
                let t_diff = (t_us as u128) - sample.t_capture_us;
                if t_diff < min_t_diff {
                    min_t_diff = t_diff;
                }
                time_sync_samples += 1;
            } else {
                // exhausted the buffer for samples that are within the time sync age (variance age is smaller)
                break;
            }

            // move backward in the buffer
            idx = Self::move_idx(self.buff_idx, 1, false);
            if idx == self.buff_idx {
                // exhausted the entire buffer
                break;
            }
        }

        // Check for state transition to active signal
        if !self.signal_active() {
            // check if seeding is complete

            // check if within expanding radius and signal variance is acceptable

            // set accept action to snap

            // expand search radius
        };

        if self.signal_active() {
            // check if within radius of pos_est if not snapped and accept

            // check if signal has stopped

            // update time sync min

            // calculate age of sample

        }

        todo!()
    }

    pub fn signal_active(&self) -> bool {
        self.signal_active
    }

    #[inline(always)]
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

struct Odometer {
}

impl Odometer {
    pub fn start(
        &self,
        start_pos: SVector<f32, 3>,
    ) {
        todo!()
    }

    pub fn tick(
        &self,
        enc: &EncoderSample,
    ) {
        todo!()
    }

    pub fn get_pos(&self) -> SVector<f32, MEAS_LEN> {
        todo!()
    }

    pub fn active(&self) -> bool {
        todo!()
    }
}

struct StateEstimator<const L: usize, const K: usize> {
    ekf: BufferedEKF<L>,
    vision_filter: VisionFilter<K>,
    odometer: Odometer,
}

impl<const L: usize, const K: usize> StateEstimator<L, K> {

    pub fn new() -> Self {
        // Check that the vision filter max_age_acceptance is equal or less than EKF buffer size
        todo!()
    }

    pub fn tick(
        &mut self,
        t_us: u64,
        imu: &ImuSample,
        encoder: &EncoderSample,
        vision: Option<&VisionSample>,
    ) -> Result<(), ()> {

        let mut z: Option<SVector<f32, MEAS_LEN>> = None;
        let mut z_age_us = 0;

        match self.vision_filter.tick(
            t_us,
            self.ekf.get_pos(),
            vision,
        ) {
            VisionSampleAction::Accept(accept_action, meas, meas_age_us) => {
                z = Some(meas);
                z_age_us = meas_age_us;
                match accept_action {
                    VisionSampleAccept::SnapState => {
                        self.ekf.init(
                            meas,
                            SVector::<f32, 3>::zeros(),
                        )
                    },
                    VisionSampleAccept::UpdateState => {},
                };
            },
            VisionSampleAction::Reject(reject) => {
                todo!();
            },
            VisionSampleAction::None => {},
        }

        if (!self.vision_active()) {
            // TODO: Uncomment after odometer is implemented
            // if (!self.odometer.active()) {
            //     self.odometer.start(self.ekf.get_pos());
            // }
            // self.odometer.tick(encoder);
            // z = Some(self.odometer.get_pos());
            // z_age_us = 0;

            z = None
        }

        let u = SVector::<f32, INPUT_LEN>::new(
            imu.x_acc_mps2,
            imu.y_acc_mps2,
            imu.w_gyro_radps,
        );

        self.ekf.tick(
            u,
            z,
            z_age_us as u32,
        )?;

        Ok(())
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

    pub fn get_ekf_pos(&self) -> SVector<f32, 3> {
        self.ekf.get_ekf_pos()
    }

    pub fn get_ekf_vel(&self) -> SVector<f32, 3> {
        self.ekf.get_ekf_vel()
    }
}


#[cfg(test)]
mod tests {
    #[test]
    fn test1() {
        assert!(true);
    }
}
