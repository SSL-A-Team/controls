use libm::{cosf, roundf, sinf};
use core::f32::consts::PI;
use nalgebra::{SMatrix, SVector};


pub const STATE_LEN: usize = 5;
pub const INPUT_LEN: usize = 3;
pub const MEAS_LEN: usize = 3;


#[allow(unused)]
#[derive(Clone, Copy)]
struct StateFrame {
    /// EKF state
    x_ekf: SVector<f32, STATE_LEN>,
    /// EKF state estimate covariance
    p_ekf: SMatrix<f32, STATE_LEN, STATE_LEN>,
    /// Reckoning state
    x_reck: SVector<f32, STATE_LEN>,
    /// Gyro control input
    u: SVector<f32, INPUT_LEN>,
    /// Vision measurement
    z: SVector<f32, MEAS_LEN>,
    /// Indicates if this frame contains a valid vision measurement
    z_update: bool,
}

impl Default for StateFrame {
    fn default() -> Self {
        StateFrame {
            x_ekf: SMatrix::zeros(),
            p_ekf: SMatrix::zeros(),
            x_reck: SMatrix::zeros(),
            u: SMatrix::zeros(),
            z: SMatrix::zeros(),
            z_update: false,
        }
    }
}


#[allow(unused)]
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

#[allow(unused)]
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
        estimator.init(init_pos);
        
        estimator
    }

    pub fn init(
        &mut self,
        init_pos: SVector<f32, 3>,
        init_vel: SVector<f32, 2>,
    ) {
        self.buff = unsafe{ core::mem::zeroed() };  // reset all bytes in the buffer to 0's
        self.idx_ekf = 0;
        self.idx_reck = self.ekf_delay_frames;
        self.buff[self.idx_ekf].p_ekf = SMatrix::<f32, STATE_LEN, STATE_LEN>::from_diagonal(
            &SVector::<f32, STATE_LEN>::from(
                [1000., 1000., PI*PI, 25., 25.]
            )
        );
        self.buff[self.idx_ekf].x_ekf.fixed_rows_mut::<3>(0).copy_from(&init_pos);
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

        if let Some(vision) = z {
            self.insert_vision(vision, z_delay_us);
        }
        self.reckon_predict(u);
        self.ekf_predict();
        self.ekf_update()?;
        self.reckon_correct();
        Ok(())
    }

    pub fn get_state(&self) -> SVector<f32, STATE_LEN> {
        self.buff[self.idx_reck].x_reck
    }

    /// Insert the vision measurement at the correct frame in the past
    fn insert_vision(
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
        self.buff[vision_idx].z.copy_from(&z);
        self.buff[vision_idx].z_update = true;
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
        if !self.buff[self.idx_ekf].z_update {
            return Ok(());
        }
        let frame = &mut self.buff[self.idx_ekf];
        let mut z = frame.z;
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
        let gyr_vw = u[(0, 0)];
        let acc_ax = u[(1, 0)];
        let acc_ay = u[(2, 0)];

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
}




#[allow(unused)]
struct VisionTimeSync {
    sync_toc_us: u64,
    sync_seq_num: usize,
    max_seq_num: usize,
    cap_period_us: f32,
    min_process_latency_us: u64,
    active: bool,
    seed_count_thresh: u32,
    seed_count: u32,
}

#[allow(unused)]
impl VisionTimeSync {
    fn new(
        min_process_latency_us: u64,
        capture_hz: f32,
    ) -> VisionTimeSync {
        todo!()
    }

    /// When a new vision measurement comes in, update the time sync. Needs the packet RX time on the robot and the sequence number.
    /// - Wrap the sequence number if less than current sync seq num
    /// - Calculate the estimated time-of-capture robot tick for this seq num from the current sync tick
    /// - Calculate latency for this packet (rx_tick_us - estimated_toc_tick)
    /// - If it's lower than the min_process_latency_us, move the sync period forward in time (sync_toc_us = rx_tick_us - min_process_latency_us) and update latency to min
    /// - If it's higher than the min_process_latency_us, use the estimated toc tick to update the sync
    /// - Increment seed count, set to active once the threshold is reached
    /// 
    /// Returns: estimated time-of-capture robot tick for this vision measurment
    fn update(
        seq_num: u32,
        rx_tick_us: u64,
    ) -> u64 {
        todo!()
    }
}


#[cfg(test)]
mod tests {
    #[test]
    fn test1() {
        assert!(true);
    }
}
