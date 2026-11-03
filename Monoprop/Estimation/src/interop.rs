//! Conversions between the filter (f64) and the Drone software (f32 arrays, `Common::DroneTelemetry`).

use crate::filter::BareESEKF;
use crate::full_state::{FullState, ImuSample};
use crate::math::*;

/// An f32 sensor triple (driver output) as an f64 vector.
pub fn v3_from_f32(a: [f32; 3]) -> V3 {
    V3::new(a[0] as f64, a[1] as f64, a[2] as f64)
}

/// An f64 vector as an f32 triple (for telemetry).
pub fn v3_to_f32(v: &V3) -> [f32; 3] {
    [v.x as f32, v.y as f32, v.z as f32]
}

/// An IMU sample from f32 driver readings: accelerometer (m/s^2) and gyro (rad/s), body axes.
pub fn imu_from_f32(accel: [f32; 3], gyro: [f32; 3]) -> ImuSample {
    ImuSample { accel: v3_from_f32(accel), gyro: v3_from_f32(gyro) }
}

impl BareESEKF<FullState, 15> {
    /// Position (m, navigation frame): `DroneTelemetry::position`.
    pub fn position_f32(&self) -> [f32; 3] {
        v3_to_f32(&self.nominal_state.p)
    }

    /// Velocity (m/s, navigation frame): `DroneTelemetry::velocity`.
    pub fn velocity_f32(&self) -> [f32; 3] {
        v3_to_f32(&self.nominal_state.v)
    }

    /// Attitude (body -> navigation) as [qx, qy, qz, qw]: `DroneTelemetry::orientation`.
    pub fn orientation_xyzw_f32(&self) -> [f32; 4] {
        let q = self.nominal_state.q.quaternion();
        [q.i as f32, q.j as f32, q.k as f32, q.w as f32]
    }

    /// Body angular rate (rad/s) with the estimated gyro bias removed: `DroneTelemetry::angular_vel`.
    pub fn angular_velocity_f32(&self, gyro: &V3) -> [f32; 3] {
        v3_to_f32(&(gyro - self.nominal_state.wb))
    }
}
