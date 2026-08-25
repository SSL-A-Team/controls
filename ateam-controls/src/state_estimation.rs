use core::{default, mem::zeroed};

use libm::{cosf, sinf};
use nalgebra::SMatrix;


const STATE_ROWS: usize = 5;
const INPUT_ROWS: usize = 3;
const MEAS_ROWS: usize = 3;


#[allow(unused)]
#[derive(Clone, Copy)]
struct StateFrame {
    ekf_state: SMatrix<f32, STATE_ROWS, 1>,
    ekf_variance: SMatrix<f32, STATE_ROWS, STATE_ROWS>,
    reck_state: SMatrix<f32, STATE_ROWS, 1>,
    vision_meas: SMatrix<f32, MEAS_ROWS, 1>,
    vision_update: bool,
    imu_input: SMatrix<f32, INPUT_ROWS, 1>,
}

impl Default for StateFrame {
    fn default() -> Self {
        StateFrame {
            ekf_state: SMatrix::zeros(),
            ekf_variance: SMatrix::zeros(),
            reck_state: SMatrix::zeros(),
            vision_meas: SMatrix::zeros(),
            vision_update: false,
            imu_input: SMatrix::zeros(),
        }
    }
}


#[allow(unused)]
struct BufferedKalmanFilter<const L: usize> {
    buffer: [StateFrame; L],
    ekf_buffer_idx: usize,
    reckon_buffer_idx: usize,
    /// Absolute time of the latest frame in the buffer
    buffer_time_us: u64,
    /// Buffer time delta in seconds
    dt: f32,
    obs_var_r: SMatrix<f32, MEAS_ROWS, MEAS_ROWS>,
    proc_var_q: SMatrix<f32, STATE_ROWS, STATE_ROWS>,
}

#[allow(unused)]
impl<const L: usize> BufferedKalmanFilter<L> {
    fn tick(
        &mut self,
        gyro_meas: SMatrix<f32, INPUT_ROWS, 1>,
        vision_meas: Option<SMatrix<f32, MEAS_ROWS, 1>>,
        vision_toc_us: u64,
    ) {
        self.ekf_buffer_idx += 1;
        self.reckon_buffer_idx += 1;
        self.buffer_time_us += Instant::now();

        if let Some(vision) = vision_meas {
            self.insert_vision(vision, vision_toc_us);
        }
        self.reckon_predict(gyro_meas);
        self.ekf_predict(gyro_meas);
        self.ekf_update();
        self.reckon_correct();
    }

    fn reckon_predict(
        &mut self,
        gyro_meas: SMatrix<f32, INPUT_ROWS, 1>
    ) {
        todo!()
    }

    fn reckon_correct(
        &mut self
    ) {
    }

    fn ekf_predict(
        &mut self,
        gyro_meas: SMatrix<f32, INPUT_ROWS, 1>
    ) {

    }

    fn ekf_update(
        &mut self,
    ) {
        if !self.buffer[self.ekf_buffer_idx % L].vision_update {
            return
        }
    }

    fn insert_vision(
        &mut self,
        vision_meas: SMatrix<f32, MEAS_ROWS, 1>,
        vision_toc_us: u64,
    ) {

    }

    fn state_predict(
        x: SMatrix<f32, STATE_ROWS, 1>,
        u: SMatrix<f32, INPUT_ROWS, 1>,
        dt: f32,
    ) -> SMatrix<f32, STATE_ROWS, 1> {
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

        // Prepare for state update calculation
        let a = pw + 0.5 * self.dt * vw;  // use midpoint theta between current and next frame
        let cosa = cosf(a);
        let sina = sinf(a);

        let x_next = SMatrix::<f32, STATE_ROWS, 1>::zeros();

        // State update integration
        x_next[(0, 0)] = px + (cosa * vx - sina * vy) * dt + 0.5 * (cosa * ax - sina * ay) * dt * dt;  // local vel and acc are rotated
        x_next[(1, 0)] = py + (sina * vx + cosa * vy) * dt + 0.5 * (sina * ax + cosa * ay) * dt * dt;  // local vel and acc are rotated
        x_next[(2, 0)] = pw + dt * vw;
        x_next[(3, 0)] = vx + dt * ax;
        x_next[(4, 0)] = vy + dt * ay;

        x_next
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
