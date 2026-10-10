/// A 3-vector in the body frame.
pub type Vec3 = [f64; 3];

/// One IMU measurement, already in SI units and the body frame (see the crate docs).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ImuSample {
    /// Time on the caller's clock (the control loop's mission time) [s].
    pub timestamp_s: f64,
    /// Time on the device's own clock, if it reports one [ns]. Gives jitter-free dt.
    pub sensor_time_ns: Option<u64>,
    /// Sample counter from the driver; a gap means samples were dropped.
    pub seq: u32,
    /// Specific force [m/s^2]. Level and at rest reads about [0, 0, +9.81].
    pub accel: Vec3,
    /// Angular rate [rad/s].
    pub gyro: Vec3,
    /// Magnetic field [T].
    pub mag: Option<Vec3>,
    /// Sensor die temperature [degC].
    pub temperature_c: Option<f64>,
    /// Static (barometric) pressure [Pa].
    pub pressure_pa: Option<f64>,
    /// Coning/sculling-corrected integrals since the previous sample.
    pub delta: Option<DeltaIntegrals>,
    /// The device's own attitude estimate, body to world, as [w, x, y, z].
    pub attitude: Option<[f64; 4]>,
    pub status: ImuStatus,
}

/// Integrated angle and velocity change over one output interval (e.g. VN-200 delta-theta/delta-velocity).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DeltaIntegrals {
    /// [rad]
    pub delta_theta: Vec3,
    /// [m/s]
    pub delta_velocity: Vec3,
    /// Integration interval [s].
    pub dt_s: f64,
}

/// Health and processing flags. All false when a device does not report them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ImuStatus {
    pub accel_saturated: bool,
    pub gyro_saturated: bool,
    /// Magnetometer reading is disturbed (e.g. nearby ferrous metal or current).
    pub mag_disturbed: bool,
    /// The device has already removed its own bias estimate from accel/gyro.
    /// The navigation filter estimates bias itself and must not correct twice.
    pub bias_compensated: bool,
    /// The device reports its calibration as complete.
    pub calibrated: bool,
}

impl ImuSample {
    /// A sample with only accelerometer and gyroscope; every optional field is empty.
    pub fn new(timestamp_s: f64, accel: Vec3, gyro: Vec3) -> Self {
        Self {
            timestamp_s,
            sensor_time_ns: None,
            seq: 0,
            accel,
            gyro,
            mag: None,
            temperature_c: None,
            pressure_pa: None,
            delta: None,
            attitude: None,
            status: ImuStatus::default(),
        }
    }

    pub fn with_mag(mut self, mag: Vec3) -> Self {
        self.mag = Some(mag);
        self
    }

    pub fn with_temperature(mut self, temperature_c: f64) -> Self {
        self.temperature_c = Some(temperature_c);
        self
    }

    pub fn with_pressure(mut self, pressure_pa: f64) -> Self {
        self.pressure_pa = Some(pressure_pa);
        self
    }

    /// True when every present value is finite (no NaN or infinity).
    pub fn is_finite(&self) -> bool {
        fn all(a: &[f64]) -> bool {
            a.iter().all(|x| x.is_finite())
        }
        self.timestamp_s.is_finite()
            && all(&self.accel)
            && all(&self.gyro)
            && self.mag.as_ref().is_none_or(|m| all(m))
            && self.temperature_c.is_none_or(f64::is_finite)
            && self.pressure_pa.is_none_or(f64::is_finite)
            && self
                .delta
                .as_ref()
                .is_none_or(|d| all(&d.delta_theta) && all(&d.delta_velocity) && d.dt_s.is_finite())
            && self.attitude.as_ref().is_none_or(|q| all(q))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_leaves_optional_fields_empty() {
        let s = ImuSample::new(1.0, [0.0, 0.0, 9.81], [0.0; 3]);
        assert_eq!(s.mag, None);
        assert_eq!(s.temperature_c, None);
        assert_eq!(s.pressure_pa, None);
        assert_eq!(s.status, ImuStatus::default());
    }

    #[test]
    fn builders_set_fields() {
        let s = ImuSample::new(0.0, [0.0; 3], [0.0; 3])
            .with_mag([1e-6, 2e-6, 3e-6])
            .with_temperature(25.0)
            .with_pressure(101_325.0);
        assert_eq!(s.mag, Some([1e-6, 2e-6, 3e-6]));
        assert_eq!(s.temperature_c, Some(25.0));
        assert_eq!(s.pressure_pa, Some(101_325.0));
    }

    #[test]
    fn is_finite_rejects_nan_anywhere() {
        let ok = ImuSample::new(0.0, [0.0, 0.0, 9.81], [0.0; 3]).with_mag([0.0; 3]);
        assert!(ok.is_finite());
        let mut bad = ok;
        bad.gyro[1] = f64::NAN;
        assert!(!bad.is_finite());
        let bad_mag = ok.with_mag([0.0, f64::INFINITY, 0.0]);
        assert!(!bad_mag.is_finite());
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_round_trip() {
        let s = ImuSample::new(2.5, [0.1, 0.2, 9.81], [0.01, 0.02, 0.03]).with_mag([-2.0e-6, 22.0e-6, -44.3e-6]);
        let json = serde_json::to_string(&s).unwrap();
        let back: ImuSample = serde_json::from_str(&json).unwrap();
        assert_eq!(s, back);
    }
}
