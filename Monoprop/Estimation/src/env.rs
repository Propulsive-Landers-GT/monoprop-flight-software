//! Environment and sensor parameters the filter relies on.
//!
//! The values follow the team's existing ES-EKF
//! template (`Drone/src/esekf/full_state_esekf.rs`) and the rocket simulation it cites (device_sim.rs):
//!
//! - Navigation frame: x east, y north, z up (ENU), the frame of that template's magnetic field vector.
//!   With UWB positioning, use the anchor survey's frame instead and express `mag_reference()` in it.
//! - The attitude quaternion q rotates body-frame vectors into the navigation frame.
//!
//! REVIEW BEFORE FLIGHT: local gravity, the magnetic field for the test site (NOAA WMM calculator), and the
//! IMU noise measured with an Allan deviation log of the vehicle's own IMU (see the `noise` module).
use crate::*;

/// Magnitude of gravity (m/s^2), as in the template and simulation. Atlanta's local value is about 9.796.
pub const GRAVITY: f64 = 9.81;

/// Gravity in the navigation frame (z up).
pub fn g_inertial() -> V3 {
    V3::new(0.0, 0.0, -GRAVITY)
}

/// The Earth's magnetic field in the navigation frame (ENU), in tesla, as used by the template and the
/// simulation: 2 uT west of north, 22 uT north, 44.3 uT down (inclination about 64 deg).
pub fn mag_field_enu() -> V3 {
    V3::new(-2.0e-6, 22.0e-6, -44.3e-6)
}

/// Unit vector of the magnetic field in the navigation frame: what `update_mag` and `align` compare against.
/// (Magnetometer readings only need a consistent unit: the measurement model uses directions.)
pub fn mag_reference() -> V3 {
    mag_field_enu().normalize()
}

/// IMU noise model used for the process noise Q.
#[derive(Clone, Copy, Debug)]
pub struct ImuSpec {
    pub sigma_a: f64,  // accelerometer white noise, per sample (m/s^2)
    pub sigma_g: f64,  // gyro white noise, per sample (rad/s)
    pub sigma_aw: f64, // accelerometer bias random walk intensity (m/s^2 per sqrt(s))
    pub sigma_gw: f64, // gyro bias random walk intensity (rad/s per sqrt(s))
}

/// VectorNav VN-200 datasheet values, as in the template (`vn200` module of full_state_esekf.rs).
pub mod vn200 {
    use crate::PI;
    /// Accelerometer noise density, m/s^2/sqrt(Hz): 0.14 mg/sqrt(Hz).
    pub const ACCEL_NOISE_DENSITY: f64 = 0.14e-3 * 9.80665;
    /// Gyro noise density, rad/s/sqrt(Hz): 0.0035 deg/s/sqrt(Hz).
    pub const GYRO_NOISE_DENSITY: f64 = 0.0035 * PI / 180.0;
    /// Accelerometer in-run bias stability, m/s^2: 0.04 mg.
    pub const ACCEL_BIAS_INSTABILITY: f64 = 0.04e-3 * 9.80665;
    /// Gyro in-run bias stability, rad/s: 10 deg/h (datasheet worst case).
    pub const GYRO_BIAS_INSTABILITY: f64 = 10.0 / 3600.0 * PI / 180.0;
    /// Assumed correlation time of the bias wander, s (first-order Gauss-Markov approximation).
    pub const BIAS_CORRELATION_TIME: f64 = 100.0;
    /// Bias random-walk intensities = instability / sqrt(tau), m/s^2/sqrt(s) and rad/s/sqrt(s).
    pub const ACCEL_BIAS_RANDOM_WALK: f64 = ACCEL_BIAS_INSTABILITY / 10.0; // sqrt(100 s)
    pub const GYRO_BIAS_RANDOM_WALK: f64 = GYRO_BIAS_INSTABILITY / 10.0;
}

impl ImuSpec {
    /// From datasheet-style numbers: white-noise densities (per sqrt(Hz)) and bias random-walk
    /// intensities (per sqrt(s)), for an IMU sampled at `rate_hz`.
    pub fn from_densities(accel_density: f64, gyro_density: f64, accel_rw: f64, gyro_rw: f64, rate_hz: f64) -> ImuSpec {
        ImuSpec {
            sigma_a: per_sample_sigma(accel_density, rate_hz),
            sigma_g: per_sample_sigma(gyro_density, rate_hz),
            sigma_aw: accel_rw,
            sigma_gw: gyro_rw,
        }
    }

    /// The VN-200 at `rate_hz` (the template notes 300 Hz in the simulation, 100 Hz in logged data).
    pub fn vn200(rate_hz: f64) -> ImuSpec {
        ImuSpec::from_densities(
            vn200::ACCEL_NOISE_DENSITY,
            vn200::GYRO_NOISE_DENSITY,
            vn200::ACCEL_BIAS_RANDOM_WALK,
            vn200::GYRO_BIAS_RANDOM_WALK,
            rate_hz,
        )
    }
}
