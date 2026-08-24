use core::{default, mem::zeroed};

use libm::{cosf, sinf};
use nalgebra::SMatrix;


const STATE_ROWS: usize = 5;
const INPUT_ROWS: usize = 3;
const MEAS_ROWS: usize = 3;


#[allow(unused)]
#[derive(Clone, Copy)]
struct KalmanFrame {
    state: SMatrix<f32, STATE_ROWS, 1>,
    meas: SMatrix<f32, MEAS_ROWS, 1>,
    control: SMatrix<f32, INPUT_ROWS, 1>,
    state_var: SMatrix<f32, STATE_ROWS, STATE_ROWS>,
}

impl Default for KalmanFrame {
    fn default() -> Self {
        KalmanFrame {
            state: SMatrix::zeros(),
            meas: SMatrix::zeros(),
            control: SMatrix::zeros(),
            state_var: SMatrix::zeros(),
        }
    }
}




#[allow(unused)]
struct BufferedKalmanFilter<const L: usize> {
    obs_var_r: SMatrix<f32, MEAS_ROWS, MEAS_ROWS>,
    buffer: [KalmanFrame; L],
    buffer_pointer: usize,
    buffer_time_us: u64,
    /// Kalman filter time delta in seconds
    dt: f32,
}

#[allow(unused)]
impl<const L: usize> BufferedKalmanFilter<L> {
    /// Create a new buffer entry for the current time from the previous buffer frame
    fn state_update(
        &mut self,
        u: SMatrix<f32, INPUT_ROWS, 1>
    ) {
        let prev_frame = self.buffer[self.buffer_pointer];
        let mut next_frame = KalmanFrame::default();

        // State vars
        let x = prev_frame.state[(0, 0)];  // global x
        let y = prev_frame.state[(1, 0)];  // global y
        let w = prev_frame.state[(2, 0)];  // global theta
        let vx = prev_frame.state[(3, 0)];  // local x vel
        let vy = prev_frame.state[(4, 0)];  // local y vel

        // Input vars
        let gyr_vw = u[(0, 0)];
        let acc_ax = u[(1, 0)];
        let acc_ay = u[(2, 0)];

        // Prepare input measurements
        let vw = gyr_vw;  // theta velocity taken directly from gyro
        let ax = acc_ax + vy * vw;  // account for coriolis/transport term for the rotating local frame
        let ay = acc_ay - vx * vw;  // account for coriolis/transport term for the rotating local frame

        // Prepare for state update calculation
        let a = w + 0.5 * self.dt * vw;  // use midpoint theta between current and next frame
        let cosa = cosf(a);
        let sina = sinf(a);

        // State update integration
        next_frame.state[(0, 0)] = x + (cosa * vx - sina * vy) * self.dt + 0.5 * (cosa * ax - sina * ay) * self.dt * self.dt;  // local vel and acc are rotated
        next_frame.state[(1, 0)] = y + (sina * vx + cosa * vy) * self.dt + 0.5 * (sina * ax + cosa * ay) * self.dt * self.dt;  // local vel and acc are rotated
        next_frame.state[(2, 0)] = w + self.dt * vw;
        next_frame.state[(3, 0)] = vx + self.dt * ax;
        next_frame.state[(4, 0)] = vy + self.dt * ay;

        // Save in new buffer frame
        self.buffer_pointer += 1;
        if self.buffer_pointer >= L {
            self.buffer_pointer = 0;
        }
        self.buffer_time_us += (self.dt * 1e6) as u64;
        self.buffer[self.buffer_pointer] = next_frame;
    }

    /// Lookup where in the buffer the measurement happened with m_toc_us
    /// Run a kalman update on that buffer frame, then run kalman state updates all the way to current time
    /// Create a new buffer entry for current time
    fn measurement_update(
        z: SMatrix<f32, MEAS_ROWS, 1>,
        z_toc_us: u64,
    ) {
        todo!()
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
