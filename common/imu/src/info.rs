/// Static description of an IMU: what it measures, how fast, and how noisy.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct ImuInfo {
    /// Human-readable model, e.g. "VectorNav VN-200".
    pub model: &'static str,
    pub capabilities: Capabilities,
    /// Highest output rate the driver is configured for [Hz].
    pub max_rate_hz: f64,
    /// Accelerometer full-scale range, per axis [m/s^2].
    pub accel_range_mps2: f64,
    /// Gyroscope full-scale range, per axis [rad/s].
    pub gyro_range_radps: f64,
    pub noise: NoiseSpec,
}

/// Which optional [`ImuSample`](crate::ImuSample) fields this device fills in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct Capabilities {
    pub sensor_time: bool,
    pub mag: bool,
    pub temperature: bool,
    pub pressure: bool,
    pub delta: bool,
    pub attitude: bool,
}

/// Datasheet noise figures, in the units the navigation filter uses.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct NoiseSpec {
    /// [m/s^2/sqrt(Hz)]
    pub accel_noise_density: f64,
    /// [rad/s/sqrt(Hz)]
    pub gyro_noise_density: f64,
    /// In-run bias stability [m/s^2]
    pub accel_bias_instability: f64,
    /// In-run bias stability [rad/s]
    pub gyro_bias_instability: f64,
    /// Magnetometer noise, 1 sigma [T]
    pub mag_noise: Option<f64>,
}
