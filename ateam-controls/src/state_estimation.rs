use nalgebra::SMatrix;


struct KalmanFilterEntry<T, const S: usize, const C: usize, const M: usize> {
    state: SMatrix<T, S, 1>,
    meas: SMatrix<T, M, 1>,
    control: SMatrix<T, C, 1>,
    state_var: SMatrix<T, S, S>,
}   

struct BufferedKalmanFilter<T, const L: usize, const S: usize, const C: usize, const M: usize> {
    state_trans_a: SMatrix<T, S, S>,
    state_trans_b: SMatrix<T, C, C>,
    obs_var_r: SMatrix<T, M, M>,
    history_buffer: [KalmanFilterEntry<T, S, C, M>; L],
}

impl<T, const L: usize, const S: usize, const C: usize, const M: usize> BufferedKalmanFilter<T, L, S, C, M> {
    fn state_update(
        x: SMatrix<f32, S, 1>, 
        u: SMatrix<f32, C, 1>
    ) -> SMatrix<T, S, 1> {
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
