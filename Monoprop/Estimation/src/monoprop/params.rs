//! Monoprop sensor parameters. PRELIMINARY: review every PLACEHOLDER before flight.
//!
//! Labels:
//! - DATASHEET(BMI323): a value from the Bosch BMI323 datasheet (typical, high-performance mode).
//! - PLACEHOLDER(source): a stand-in, with where it came from and what should replace it.
use crate::*;

// ---- unit conversion ----

/// Standard gravity (m/s^2). Used ONLY to convert datasheet values given in ug or mg to m/s^2.
/// It is not the local gravity the filter uses: that is `GRAVITY` in env.rs.
pub const STANDARD_GRAVITY: f64 = 9.80665;

// ---- IMU: Bosch BMI323 ----

/// Bosch BMI323 values in SI units, in the same form as the `vn200` module of env.rs.
pub mod bmi323 {
    use super::STANDARD_GRAVITY;
    use crate::PI;

    // DATASHEET(BMI323): gyro noise density, typical, high-performance mode.
    /// Gyro noise density, rad/s/sqrt(Hz): 0.007 deg/s/sqrt(Hz) * pi/180.
    pub const GYRO_NOISE_DENSITY: f64 = 0.007 * PI / 180.0;

    // DATASHEET(BMI323): accelerometer noise density, typical, high-performance mode.
    /// Accelerometer noise density, m/s^2/sqrt(Hz): 180 ug/sqrt(Hz) * 1e-6 * standard gravity.
    pub const ACCEL_NOISE_DENSITY: f64 = 180.0e-6 * STANDARD_GRAVITY;

    // DATASHEET(BMI323): gyro zero-rate offset, +-1 deg/s.
    /// Gyro zero-rate offset (how large the bias can be), rad/s: 1 deg/s * pi/180.
    pub const GYRO_ZERO_RATE_OFFSET: f64 = 1.0 * PI / 180.0;

    // DATASHEET(BMI323): accelerometer offset, +-50 mg.
    /// Accelerometer offset (how large the bias can be), m/s^2: 50 mg * 1e-3 * standard gravity.
    pub const ACCEL_OFFSET: f64 = 50.0e-3 * STANDARD_GRAVITY;

    // PLACEHOLDER(VN-200 value from env.rs): the BMI323 datasheet gives no bias random walk. This is
    // probably too small for the BMI323; replace it with the result of an Allan variance test.
    /// Accelerometer bias random-walk intensity, m/s^2/sqrt(s).
    pub const ACCEL_BIAS_RANDOM_WALK: f64 = crate::vn200::ACCEL_BIAS_RANDOM_WALK;

    // PLACEHOLDER(VN-200 value from env.rs): the BMI323 datasheet gives no bias random walk. This is
    // probably too small for the BMI323; replace it with the result of an Allan variance test.
    /// Gyro bias random-walk intensity, rad/s/sqrt(s).
    pub const GYRO_BIAS_RANDOM_WALK: f64 = crate::vn200::GYRO_BIAS_RANDOM_WALK;
}

// PLACEHOLDER(no hardware yet): the output data rate the BMI323 is actually configured for.
/// IMU sample rate, Hz.
pub const IMU_RATE_HZ: f64 = 400.0;

/// The BMI323 sampled at `rate_hz`, built the same way as `ImuSpec::vn200`.
pub fn bmi323_spec(rate_hz: f64) -> ImuSpec {
    ImuSpec::from_densities(
        bmi323::ACCEL_NOISE_DENSITY,
        bmi323::GYRO_NOISE_DENSITY,
        bmi323::ACCEL_BIAS_RANDOM_WALK,
        bmi323::GYRO_BIAS_RANDOM_WALK,
        rate_hz,
    )
}

// ---- measurement noise ----

// PLACEHOLDER: the GNSS receiver's reported or measured position accuracy.
/// Position fix standard deviation per axis, m: the `sigma` of `update_gps` and `align`.
pub const SIGMA_POS: f64 = 0.5;

// PLACEHOLDER: the measured noise of the calibrated magnetometer's direction.
/// Magnetometer direction standard deviation per axis (unit vector, so unitless): the `sigma` of `update_mag`.
pub const SIGMA_MAG: f64 = 0.02;

// PLACEHOLDER: the measured noise of the tilt-corrected lidar height.
/// Height standard deviation, m: the `sigma` of `update_height`.
pub const SIGMA_HEIGHT: f64 = 0.05;

// ---- frames (assumptions) ----
//
// PLACEHOLDER(assumption): confirm against the vehicle drawings.
// Body frame: x forward, y left, z up (right-handed), origin at the reference IMU.
// Reference IMU: the BMI323. The vehicle has two; one of the two also carries the magnetometer.
// TODO: which of the two BMI323s is the reference.
//
// Magnetic reference: use `mag_reference()` from env.rs (set it for the test site there).

// ---- mounting ----
//
// For each sensor: the rotation taking a vector from the sensor's own axes into the body frame, and the
// sensor's position in the body frame (m). Functions, because nalgebra's identity and zeros are not `const`.
//
// TODO: the lidar beam direction in the body frame (needed to turn range into height); not known yet.
// TODO: mounting of the second BMI323; not known yet.

// PLACEHOLDER(no hardware yet): the measured mounting rotation of the reference IMU.
/// Reference IMU axes -> body frame.
pub fn imu_mount_rotation() -> M3 {
    M3::identity()
}

// PLACEHOLDER(no hardware yet): the measured position of the reference IMU.
/// Reference IMU position in the body frame, m.
pub fn imu_mount_offset() -> V3 {
    V3::zeros()
}

// PLACEHOLDER(no hardware yet): the measured mounting rotation of the magnetometer.
/// Magnetometer axes -> body frame.
pub fn mag_mount_rotation() -> M3 {
    M3::identity()
}

// PLACEHOLDER(no hardware yet): the measured position of the magnetometer.
/// Magnetometer position in the body frame, m.
pub fn mag_mount_offset() -> V3 {
    V3::zeros()
}

// PLACEHOLDER(no hardware yet): the measured mounting rotation of the lidar.
/// Lidar axes -> body frame.
pub fn lidar_mount_rotation() -> M3 {
    M3::identity()
}

// PLACEHOLDER(no hardware yet): the measured position of the lidar.
/// Lidar position in the body frame, m.
pub fn lidar_mount_offset() -> V3 {
    V3::zeros()
}

// PLACEHOLDER(no hardware yet): the measured mounting rotation of the GNSS antenna.
/// GNSS antenna axes -> body frame.
pub fn gnss_mount_rotation() -> M3 {
    M3::identity()
}

// PLACEHOLDER(no hardware yet): the measured position of the GNSS antenna.
/// GNSS antenna position in the body frame, m.
pub fn gnss_mount_offset() -> V3 {
    V3::zeros()
}
