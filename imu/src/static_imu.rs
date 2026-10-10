use crate::units::{deg_to_rad, g_to_mps2};
use crate::{Capabilities, Imu, ImuInfo, ImuSample, NoiseSpec};

/// Placeholder IMU that returns the same reading on every call.
///
/// Stands in for real hardware until a driver exists. Each `read` stamps the caller's
/// time and increments `seq`.
#[derive(Debug, Clone)]
pub struct StaticImu {
    info: ImuInfo,
    sample: ImuSample,
}

impl StaticImu {
    /// VectorNav VN-200 datasheet noise figures, the same values the navigation filter uses.
    pub const VN200_INFO: ImuInfo = ImuInfo {
        model: "Static placeholder (VN-200 noise figures)",
        capabilities: Capabilities {
            sensor_time: false,
            mag: true,
            temperature: false,
            pressure: false,
            delta: false,
            attitude: false,
        },
        max_rate_hz: 500.0,
        accel_range_mps2: g_to_mps2(16.0),
        gyro_range_radps: deg_to_rad(2000.0),
        noise: NoiseSpec {
            accel_noise_density: g_to_mps2(0.14e-3),
            gyro_noise_density: deg_to_rad(0.0035),
            accel_bias_instability: g_to_mps2(0.04e-3),
            gyro_bias_instability: deg_to_rad(10.0) / 3600.0,
            mag_noise: Some(14.0e-9),
        },
    };

    pub fn new(info: ImuInfo, sample: ImuSample) -> Self {
        Self { info, sample }
    }

    /// A level vehicle at rest: +9.81 m/s^2 on Z, no rotation, and the
    /// navigation filter's world magnetic field [T].
    pub fn level_at_rest() -> Self {
        let sample = ImuSample::new(0.0, [0.0, 0.0, 9.81], [0.0; 3])
            .with_mag([-2.0e-6, 22.0e-6, -44.3e-6]);
        Self::new(Self::VN200_INFO, sample)
    }
}

impl Imu for StaticImu {
    type Error = core::convert::Infallible;

    fn info(&self) -> &ImuInfo {
        &self.info
    }

    fn read(&mut self, now_s: f64) -> Result<Option<ImuSample>, Self::Error> {
        self.sample.timestamp_s = now_s;
        let out = self.sample;
        self.sample.seq = self.sample.seq.wrapping_add(1);
        Ok(Some(out))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stamps_time_and_counts() {
        let mut imu = StaticImu::level_at_rest();
        let a = imu.read(1.0).unwrap().unwrap();
        let b = imu.read(1.002).unwrap().unwrap();
        assert_eq!(a.timestamp_s, 1.0);
        assert_eq!(b.timestamp_s, 1.002);
        assert_eq!(a.seq, 0);
        assert_eq!(b.seq, 1);
        assert_eq!(b.accel, [0.0, 0.0, 9.81]);
        assert!(b.mag.is_some());
        assert!(imu.info().capabilities.mag);
    }
}
