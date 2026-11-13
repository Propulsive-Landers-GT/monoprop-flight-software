use monoprop_common::TelemetryFrame;
use ndarray::Array1;

/// Smooth reference to the local origin: preserves initial velocity, zero final
/// velocity/acceleration, and holds initial yaw (the TVC model has no yaw actuator).
pub struct ReturnReference {
    position: [f64; 3],
    velocity: [f64; 3],
    yaw: f64,
    pub duration_s: f64,
}
impl ReturnReference {
    pub fn new(t: &TelemetryFrame) -> Self {
        let position = t.vehicle_state.position_m.map(f64::from);
        let velocity = t.vehicle_state.velocity_mps.map(f64::from);
        let q = t.vehicle_state.attitude_quat.map(f64::from);
        let yaw =
            (2.0 * (q[0] * q[3] + q[1] * q[2])).atan2(1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
        let distance = position.iter().map(|v| v * v).sum::<f64>().sqrt();
        // Nominal quintic peak speed about 0.5 m/s for zero initial velocity;
        // the actual rollout is checked separately, including the measured velocity.
        let duration_s = (1.875 * distance / 0.5).max(20.0);
        Self {
            position,
            velocity,
            yaw,
            duration_s,
        }
    }
    pub fn at(&self, time: f64) -> Array1<f64> {
        let s = (time / self.duration_s).clamp(0.0, 1.0);
        let mut r = Array1::zeros(13);
        for i in 0..3 {
            let d = -self.position[i];
            let v = self.velocity[i] * self.duration_s;
            let a = 10.0 * d - 6.0 * v;
            let b = -15.0 * d + 8.0 * v;
            let c = 6.0 * d - 3.0 * v;
            r[i] = self.position[i] + v * s + a * s.powi(3) + b * s.powi(4) + c * s.powi(5);
            r[7 + i] =
                (v + 3.0 * a * s * s + 4.0 * b * s.powi(3) + 5.0 * c * s.powi(4)) / self.duration_s;
        }
        r[5] = (self.yaw / 2.0).sin();
        r[6] = (self.yaw / 2.0).cos();
        r
    }
    pub fn horizon(&self, time: f64) -> Vec<Array1<f64>> {
        (0..=monoprop_common::MPC_HORIZON_STEPS)
            .map(|k| self.at(time + k as f64 * monoprop_common::DEFAULT_MPC_STEP_DT_S as f64))
            .collect()
    }
}
