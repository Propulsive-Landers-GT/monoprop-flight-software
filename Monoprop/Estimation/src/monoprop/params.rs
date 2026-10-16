//! Monoprop sensor parameters. PRELIMINARY: review every PLACEHOLDER before flight.
//!
//! Labels:
//! - DATASHEET(BMI323): a value from the Bosch BMI323 datasheet (typical, high-performance mode).
//! - PLACEHOLDER(source): a stand-in, with where it came from and what should replace it.
use crate::*;
use super::{ImuId, ImuPairConfig, NavConfig};

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

// PLACEHOLDER(derived from the IMU_RATE_HZ placeholder): half the IMU sample period, so two samples
// closer than that belong to the same tick (400 Hz: 1250 us).
// TODO: set this from how the two BMI323s are actually triggered.
/// Largest timestamp difference of an IMU pair that is combined, microseconds.
pub const IMU_PAIR_MAX_SKEW_US: u64 = 1_000_000 / (2 * IMU_RATE_HZ as u64);

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
// It is the main driver of the yaw error (in simulation the attitude-error peaks are nearly all yaw).
// TODO: measure it on the real magnetometer with the motor running (the motor and valves disturb the field).
/// Magnetometer direction standard deviation per axis (unit vector, so unitless): the `sigma` of `update_mag`.
pub const SIGMA_MAG: f64 = 0.02;

// PLACEHOLDER(conservative, from the existing height tests): probably far larger than the VL53L1X's noise
// at short range. TODO: set from the ST datasheet or bench data; it depends on the distance mode, the
// ambient light and the target.
/// Lidar range standard deviation, m: the `sigma` of `update_lidar`.
pub const SIGMA_LIDAR_RANGE: f64 = 0.05;

// PLACEHOLDER(team choice, typical for a vehicle standing on the ground): how still the pad really is.
// TODO: set this from a recording on the pad (vibration, wind, tether).
/// Zero-velocity standard deviation per axis while on the pad, m/s: the `sigma` of `update_zero_velocity`.
pub const SIGMA_ZERO_VELOCITY: f64 = 0.02;

// PLACEHOLDER(derived from the BMI323 datasheet noise density and the IMU_RATE_HZ placeholder): the gyro's
// own per-sample noise (about 2.4e-3 rad/s at 400 Hz).
// TODO: confirm from a still recording on the pad.
/// Zero angular-rate standard deviation per axis while on the pad, rad/s: the `sigma` of `update_zero_rate`.
/// A function, because sqrt is not available in a const.
pub fn sigma_zero_rate() -> f64 {
    per_sample_sigma(bmi323::GYRO_NOISE_DENSITY, IMU_RATE_HZ)
}

// ---- lidar limits ----

// PLACEHOLDER(VL53L1X: minimum accurate range 4 cm, per Pololu's datasheet summary; confirm against the
// ST datasheet): the minimum range from the ST datasheet.
/// Shortest lidar range that is used, m.
pub const LIDAR_MIN_RANGE: f64 = 0.04;

// PLACEHOLDER(VL53L1X reliable range, rough): the usable range measured on the vehicle, outdoors.
// The usable range depends on the distance mode and the ambient light (short about 1.3 m, medium about
// 3 m in the dark, long about 4 m), so set this from vehicle tests.
/// Longest lidar range that is used, m.
pub const LIDAR_MAX_RANGE: f64 = 3.0;

// PLACEHOLDER(first guess): tune on the vehicle.
/// Largest angle of the lidar beam from straight down that is used, rad: 30 deg * pi/180.
pub const LIDAR_MAX_TILT: f64 = 30.0 * PI / 180.0;

// ---- frames (assumptions) ----
//
// PLACEHOLDER(assumption): confirm against the vehicle drawings.
// Body frame: x forward, y left, z up (right-handed), origin at the reference IMU.
// Reference IMU: the BMI323. The vehicle has two; one of the two also carries the magnetometer.
// Which of the two BMI323s is the reference: REFERENCE_IMU below.
//
// Magnetic reference: use `mag_reference()` from env.rs (set it for the test site there).

// ---- mounting ----
//
// For each sensor: the rotation taking a vector from the sensor's own axes into the body frame, and the
// sensor's position in the body frame (m). Functions, because nalgebra's identity and zeros are not `const`.
//

// PLACEHOLDER(no hardware yet): which BMI323 is the reference.
/// The IMU the combined IMU sample (and so the filter) is expressed at.
pub const REFERENCE_IMU: ImuId = ImuId::A;

// PLACEHOLDER(no hardware yet): the measured mounting rotation of IMU A.
/// IMU A axes -> body frame.
pub fn imu_mount_rotation() -> M3 {
    M3::identity()
}

// PLACEHOLDER(no hardware yet): the measured position of IMU A.
/// IMU A position in the body frame, m.
pub fn imu_mount_offset() -> V3 {
    V3::zeros()
}

// PLACEHOLDER(no hardware yet): the measured mounting rotation of IMU B.
/// IMU B axes -> body frame.
pub fn imu_b_mount_rotation() -> M3 {
    M3::identity()
}

// PLACEHOLDER(no hardware yet): the measured position of IMU B.
/// IMU B position in the body frame, m.
pub fn imu_b_mount_offset() -> V3 {
    V3::zeros()
}

/// The two IMUs as mounted on the Monoprop, for `combine_imus`.
pub fn imu_pair_config() -> ImuPairConfig {
    ImuPairConfig {
        reference: REFERENCE_IMU,
        a_rotation: imu_mount_rotation(),
        a_offset: imu_mount_offset(),
        b_rotation: imu_b_mount_rotation(),
        b_offset: imu_b_mount_offset(),
        max_skew_us: IMU_PAIR_MAX_SKEW_US,
    }
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

// PLACEHOLDER(assumption: the lidar points straight down): the measured beam direction.
/// Lidar beam direction in the body frame (unit vector).
pub fn lidar_beam_direction() -> V3 {
    V3::new(0.0, 0.0, -1.0)
}

/// The lidar as mounted on the Monoprop, for `update_lidar`. `ground_z` is the ground height in the
/// navigation frame (m): get it at start-up from `lidar_ground_z` with a reading taken on the pad.
pub fn lidar_config(ground_z: f64) -> LidarConfig {
    LidarConfig {
        offset: lidar_mount_offset(),
        beam: lidar_beam_direction(),
        ground_z,
        min_range: LIDAR_MIN_RANGE,
        max_range: LIDAR_MAX_RANGE,
        max_tilt: LIDAR_MAX_TILT,
    }
}

// PLACEHOLDER(no hardware yet): the measured mounting rotation of the GNSS antenna.
/// GNSS antenna axes -> body frame.
pub fn gnss_mount_rotation() -> M3 {
    M3::identity()
}

// PLACEHOLDER(no hardware yet): the measured position of the GNSS antenna.
/// GNSS antenna position in the body frame, m: the `antenna_offset` of `update_gps_offset`.
pub fn gnss_mount_offset() -> V3 {
    V3::zeros()
}

// ---- navigator: start-up and pad ----

// PLACEHOLDER(task brief: 10 to 20 Hz, lower end): rate of the stationary updates on the pad.
/// Stationary (zero-velocity and zero angular-rate) updates per second while on the pad.
pub const STATIONARY_UPDATE_HZ: u64 = 10;

/// Time between stationary updates, microseconds.
pub const STATIONARY_UPDATE_PERIOD_US: u64 = 1_000_000 / STATIONARY_UPDATE_HZ;

// PLACEHOLDER(choice: one second of stillness at the IMU_RATE_HZ placeholder): tune on the pad.
/// Still IMU samples needed before aligning.
pub const STILL_WINDOW_SAMPLES: u32 = IMU_RATE_HZ as u32;

// PLACEHOLDER(choice: alignment needs one magnetometer direction; more are averaged if they arrive).
/// Magnetometer samples needed before aligning.
pub const STILL_MIN_MAG_SAMPLES: u32 = 1;

// PLACEHOLDER(choice): how many per-sample noise sigmas a still reading may be off by.
/// Multiplier of the per-sample noise in the stillness thresholds.
pub const STILL_NOISE_SIGMAS: f64 = 5.0;

// PLACEHOLDER(derived from the BMI323 datasheet zero-rate offset and noise, and the IMU_RATE_HZ placeholder):
// the worst-case gyro bias on all three axes (not known before alignment, so it must not count as motion)
// plus STILL_NOISE_SIGMAS per-sample sigmas; about 0.042 rad/s at 400 Hz.
// TODO: tighten once the gyro bias is measured.
/// Largest angular rate that still counts as still, rad/s.
pub fn still_gyro_max() -> f64 {
    ComplexField::sqrt(3.0) * bmi323::GYRO_ZERO_RATE_OFFSET
        + STILL_NOISE_SIGMAS * per_sample_sigma(bmi323::GYRO_NOISE_DENSITY, IMU_RATE_HZ)
}

// PLACEHOLDER(derived from the BMI323 datasheet accelerometer offset and noise, and the IMU_RATE_HZ
// placeholder): the worst-case offset on all three axes plus STILL_NOISE_SIGMAS per-sample sigmas; about
// 1.0 m/s^2 at 400 Hz, dominated by the offset.
// TODO: tighten once the accelerometer bias is measured.
/// Largest difference between the specific-force magnitude and gravity that still counts as still, m/s^2.
pub fn still_accel_tol() -> f64 {
    ComplexField::sqrt(3.0) * bmi323::ACCEL_OFFSET
        + STILL_NOISE_SIGMAS * per_sample_sigma(bmi323::ACCEL_NOISE_DENSITY, IMU_RATE_HZ)
}

// PLACEHOLDER(choice: well past normal jitter, short enough that one step is still a small rotation).
/// IMU sample periods in the longest gap that is still integrated.
pub const MAX_IMU_GAP_PERIODS: u64 = 10;

/// Longest gap between IMU samples that is still integrated, microseconds (25 000 at 400 Hz).
pub const MAX_IMU_GAP_US: u64 = MAX_IMU_GAP_PERIODS * 1_000_000 / IMU_RATE_HZ as u64;

// PLACEHOLDER(no hardware yet: the IMU sits on the pad until the drawings give its height).
// TODO: set from the vehicle drawings (reference IMU above the pad surface).
// Accuracy matters when the lidar can't read on the pad: the fallback is then used for the whole flight,
// and a wrong value gives a persistent vertical bias the filter can't see (simulated: 5 cm wrong gave a
// vertical NEES of 37 instead of about 1).
/// Height of the reference IMU above the pad surface, m: the lidar's ground height until pad readings refine it.
pub const IMU_HEIGHT_ABOVE_PAD: f64 = 0.0;

// PLACEHOLDER(choice): enough readings to average out the lidar's noise on the pad (1 s at 50 Hz). The
// filter treats the averaged ground as exact, so its error (lidar sigma / sqrt(N)) shows up as a vertical
// bias: in simulation 10 readings gave a vertical NEES of 3 to 17, 50 readings about 1 to 2.
// TODO: include the averaged ground's variance (sigma^2 / N) in the lidar measurement noise.
/// Valid lidar readings on the pad averaged into the ground height.
pub const LIDAR_GROUND_SAMPLES: u32 = 50;

// PLACEHOLDER(choice): when the pad is close to the lidar's minimum range, readings below it are skipped and
// the average of the rest is biased high; above this share of pad readings below the floor, the average is
// not used and the fallback (IMU_HEIGHT_ABOVE_PAD) is kept.
/// Largest share of pad readings below the lidar's minimum range for the pad average to be used.
pub const LIDAR_PAD_TRUNCATION_MAX: f64 = 0.05;

// PLACEHOLDER(choice: 10 s at the IMU_RATE_HZ placeholder): how slowly the A-B difference is followed.
/// IMU pairs in the running average of the A-B reading difference.
pub const IMU_DIFF_WINDOW: u32 = 4000;

// PLACEHOLDER(choice: 1 s at the IMU_RATE_HZ placeholder).
/// IMU pairs needed before single-IMU samples are corrected by the learned difference.
pub const IMU_DIFF_MIN_PAIRS: u32 = 400;

/// The navigator as set up for the Monoprop.
pub fn navigator_config() -> NavConfig {
    NavConfig {
        imu_pair: imu_pair_config(),
        imu_spec: bmi323_spec(IMU_RATE_HZ),
        lidar: lidar_config(0.0),
        antenna_offset: gnss_mount_offset(),
        gravity: GRAVITY,
        sigma_pos: SIGMA_POS,
        sigma_mag: SIGMA_MAG,
        sigma_lidar_range: SIGMA_LIDAR_RANGE,
        sigma_zero_velocity: SIGMA_ZERO_VELOCITY,
        sigma_zero_rate: sigma_zero_rate(),
        stationary_period_us: STATIONARY_UPDATE_PERIOD_US,
        max_imu_gap_us: MAX_IMU_GAP_US,
        still_window_samples: STILL_WINDOW_SAMPLES,
        still_min_mag_samples: STILL_MIN_MAG_SAMPLES,
        still_accel_tol: still_accel_tol(),
        still_gyro_max: still_gyro_max(),
        imu_height_above_pad: IMU_HEIGHT_ABOVE_PAD,
        lidar_ground_samples: LIDAR_GROUND_SAMPLES,
        lidar_pad_truncation_max: LIDAR_PAD_TRUNCATION_MAX,
        imu_diff_window: IMU_DIFF_WINDOW,
        imu_diff_min_pairs: IMU_DIFF_MIN_PAIRS,
    }
}
