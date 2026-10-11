//! Conversions at the edge of the filter: the filter runs in f64, the rest of the Drone software
//! (sensor drivers, `Common::DroneTelemetry`) uses f32 arrays.
//!
use crate::*;

/// An f32 sensor triple (driver output) as the filter's f64 vector.
pub fn v3_from_f32(a: [f32; 3]) -> V3 {
    V3::new(a[0] as f64, a[1] as f64, a[2] as f64)
}

/// A filter vector as an f32 triple (for telemetry).
pub fn v3_to_f32(v: &V3) -> [f32; 3] {
    [v.x as f32, v.y as f32, v.z as f32]
}

impl Esekf {
    /// Position estimate (m, navigation frame) as f32: `DroneTelemetry::position`.
    pub fn position_f32(&self) -> [f32; 3] {
        v3_to_f32(&self.x.p)
    }

    /// Velocity estimate (m/s, navigation frame) as f32: `DroneTelemetry::velocity`.
    pub fn velocity_f32(&self) -> [f32; 3] {
        v3_to_f32(&self.x.v)
    }

    /// Attitude (body -> navigation) as [qx, qy, qz, qw] in f32, the order `DroneTelemetry::orientation` uses.
    pub fn orientation_xyzw_f32(&self) -> [f32; 4] {
        let q = self.x.q.quaternion();
        [q.i as f32, q.j as f32, q.k as f32, q.w as f32]
    }

    /// Body angular rate (rad/s) with the estimated gyro bias removed, from a raw gyro reading:
    /// `DroneTelemetry::angular_vel`.
    pub fn angular_velocity_f32(&self, gyro: &V3) -> [f32; 3] {
        v3_to_f32(&(gyro - self.x.wb))
    }

    /// One standard deviation of each error-state component (sqrt of P's diagonal), for telemetry
    /// and logging: [position 0..3, velocity 3..6, attitude (rad) 6..9, accel bias 9..12, gyro bias 12..15].
    pub fn sigmas_f32(&self) -> [f32; 15] {
        let mut out = [0.0f32; 15];
        for (i, o) in out.iter_mut().enumerate() {
            *o = ComplexField::sqrt(self.p[(i, i)].max(0.0)) as f32;
        }
        out
    }
}
