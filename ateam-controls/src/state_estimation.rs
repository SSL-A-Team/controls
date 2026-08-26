use libm::{cosf, sinf};
use nalgebra::{SMatrix, SVector};


const STATE_LEN: usize = 5;
const INPUT_LEN: usize = 3;
const MEAS_LEN: usize = 3;


#[allow(unused)]
#[derive(Clone, Copy)]
struct StateFrame {
    /// EKF state
    x_ekf: SVector<f32, STATE_LEN>,
    /// EKF covariance
    p_ekf: SMatrix<f32, STATE_LEN, STATE_LEN>,
    /// Reckoning state
    x_reck: SVector<f32, STATE_LEN>,
    /// Gyro control input
    u: SVector<f32, INPUT_LEN>,
    /// Vision measurement
    z: SVector<f32, MEAS_LEN>,
    /// Indicates if this frame contains a valid vision measurement
    vision_update: bool,
}

impl Default for StateFrame {
    fn default() -> Self {
        StateFrame {
            x_ekf: SMatrix::zeros(),
            p_ekf: SMatrix::zeros(),
            x_reck: SMatrix::zeros(),
            u: SMatrix::zeros(),
            z: SMatrix::zeros(),
            vision_update: false,
        }
    }
}


#[allow(unused)]
struct BufferedEKF<const L: usize> {
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
}

#[allow(unused)]
impl<const L: usize> BufferedEKF<L> {
    fn new(
        ekf_delay_us: u32,
        dt_us: u32,
        r: SMatrix<f32, MEAS_LEN, MEAS_LEN>,
        q: SMatrix<f32, STATE_LEN, STATE_LEN>,
    ) -> Self {
        let ekf_delay_frames = (ekf_delay_us / dt_us) as usize;
        if ekf_delay_frames >= L {
            panic!("Unable to create EKF buffer, buffer is too small for the provided EKF delay")
        }
        BufferedEKF {
            buff: [StateFrame::default(); L],
            idx_ekf: 0,
            idx_reck: ekf_delay_frames,
            ekf_delay_frames,
            dt_us,
            dt_s: (dt_us as f32) * 1e-6,
            r,
            q,
        }
    }

    fn tick(
        &mut self,
        u: SVector<f32, INPUT_LEN>,
        z: Option<SVector<f32, MEAS_LEN>>,
        z_delay_us: u32,  // Microseconds elapsed since the frame that provided this measurement was taken
    ) {
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
        self.ekf_update();
        self.reckon_correct();
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
        self.buff[vision_idx].vision_update = true;
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
    ) {
        if !self.buff[self.idx_ekf].vision_update {
            return
        }
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
