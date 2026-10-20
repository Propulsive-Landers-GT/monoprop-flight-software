//! Sensor samples, timestamped on the firmware's microsecond clock. Readings are in SI units and the body
//! frame, already calibrated and rotated by the driver.

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuSample {
    pub t_us: u64,
    pub accel: [f64; 3], // specific force, m/s^2 (about +9.81 on z at rest)
    pub gyro: [f64; 3],  // rad/s
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagSample {
    pub t_us: u64,
    pub field: [f64; 3], // any consistent unit: only the direction is used
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LidarSample {
    pub t_us: u64,
    pub range: f64, // m, along the beam
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GpsSample {
    pub t_us: u64,
    pub position: [f64; 3], // m, east, north, up from home
    pub sigma: [f64; 3],    // m, east, north, up
}

/// The newest sample of each sensor, None if nothing new arrived.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SensorData {
    pub imu_a: Option<ImuSample>,
    pub imu_b: Option<ImuSample>,
    pub mag: Option<MagSample>,
    pub lidar: Option<LidarSample>,
    pub gps: Option<GpsSample>,
}

/// TODO: initialise the two BMI323s, the AK09918, the VL53L1X and the GNSS drivers.
pub fn init() {}

/// TODO: read the drivers. Until then there are no samples, so the navigator never aligns.
pub fn read(_now_us: u64) -> SensorData {
    SensorData::default()
}
