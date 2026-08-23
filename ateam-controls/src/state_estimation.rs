use nalgebra::SMatrix;


#[allow(unused)]
struct KalmanFrame<T, const S: usize, const C: usize, const M: usize> {
    state: SMatrix<T, S, 1>,
    meas: SMatrix<T, M, 1>,
    control: SMatrix<T, C, 1>,
    state_var: SMatrix<T, S, S>,
}   

#[allow(unused)]
struct BufferedKalmanFilter<T, const L: usize, const S: usize, const C: usize, const M: usize> {
    state_trans_a: SMatrix<T, S, S>,
    state_trans_b: SMatrix<T, C, C>,
    obs_var_r: SMatrix<T, M, M>,
    buffer: [KalmanFrame<T, S, C, M>; L],
    buffer_pointer: usize,
    buffer_time_us: u64,
    buffer_time_resolution_us: u32,
}

#[allow(unused)]
impl<T, const L: usize, const S: usize, const C: usize, const M: usize> BufferedKalmanFilter<T, L, S, C, M> {
    /// Create a new buffer entry for the current time from the previous buffer frame
    fn state_update(
        x: SMatrix<f32, S, 1>, 
        u: SMatrix<f32, C, 1>
    ) -> SMatrix<T, S, 1> {
        todo!()
    }

    /// Lookup where in the buffer the measurement happened with m_toc_us
    /// Run a kalman update on that buffer frame, then run kalman state updates all the way to current time
    /// Create a new buffer entry for current time
    fn measurement_update(
        m: SMatrix<f32, M, 1>,
        m_toc_us: u64,
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
