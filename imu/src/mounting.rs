use crate::{ImuSample, Vec3};

/// Rotation from the sensor's axes into the vehicle body frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mounting {
    /// Row-major rotation matrix: `v_body = rotation * v_sensor`.
    pub rotation: [[f64; 3]; 3],
}

impl Mounting {
    /// Sensor axes already match the body frame.
    pub const IDENTITY: Self = Self {
        rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    };

    /// 180 degrees about X: forward-right-down sensor axes (e.g. VectorNav) into the Z-up body frame.
    pub const Z_DOWN_TO_Z_UP: Self = Self {
        rotation: [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]],
    };

    pub fn rotate(&self, v: Vec3) -> Vec3 {
        let r = &self.rotation;
        [
            r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2],
            r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2],
            r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2],
        ]
    }

    /// Rotate every vector field of a sample from sensor axes into the body frame.
    ///
    /// The device's own `attitude` estimate is left alone: its meaning depends on the
    /// device's reference frame, which a driver has to handle explicitly.
    pub fn apply(&self, sample: &mut ImuSample) {
        sample.accel = self.rotate(sample.accel);
        sample.gyro = self.rotate(sample.gyro);
        if let Some(m) = sample.mag.as_mut() {
            *m = self.rotate(*m);
        }
        if let Some(d) = sample.delta.as_mut() {
            d.delta_theta = self.rotate(d.delta_theta);
            d.delta_velocity = self.rotate(d.delta_velocity);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_leaves_sample_unchanged() {
        let s = ImuSample::new(0.0, [0.1, -0.2, 9.81], [0.01, 0.02, -0.03]).with_mag([1e-6, 2e-6, 3e-6]);
        let mut out = s;
        Mounting::IDENTITY.apply(&mut out);
        assert_eq!(out, s);
    }

    #[test]
    fn z_down_level_reading_becomes_z_up() {
        // A forward-right-down IMU sitting level reads -g on Z.
        let mut s = ImuSample::new(0.0, [0.0, 0.0, -9.81], [0.0, 0.1, 0.2]).with_mag([1.0, 2.0, 3.0]);
        Mounting::Z_DOWN_TO_Z_UP.apply(&mut s);
        assert_eq!(s.accel, [0.0, 0.0, 9.81]);
        assert_eq!(s.gyro, [0.0, -0.1, -0.2]);
        assert_eq!(s.mag, Some([1.0, -2.0, -3.0]));
    }
}
