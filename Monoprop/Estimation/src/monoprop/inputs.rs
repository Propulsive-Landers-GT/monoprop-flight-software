//! Sensor samples as the filter expects them: SI units, body or navigation frame, one timestamp each.
//!
//! Fields are private and each type is built with `new`, which returns None for values the filter
//! could not use (NaN, infinity, a non-positive range or sigma), so an invalid sample can't exist.
//! Timestamps are microseconds on one shared clock. the filter itself takes no timestamps, the caller uses them for dt and ordering.
use crate::*;

fn finite(v: &V3) -> bool {
    v.iter().all(|x| x.is_finite())
}

/// One IMU reading (accelerometer + gyro), for Esekf::predict.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuSample {
    /// Time the sample was taken (microseconds, shared clock).
    t_us: u64,
    /// Specific force (m/s^2, body frame). At rest and level it reads about (0, 0, +g): gravity is NOT removed.
    accel: V3,
    /// Angular rate (rad/s, body frame, right-hand rule). Bias is NOT removed.
    gyro: V3,
}

impl ImuSample {
    /// None if any component of `accel` or `gyro` is not finite.
    pub fn new(t_us: u64, accel: V3, gyro: V3) -> Option<ImuSample> {
        if !finite(&accel) || !finite(&gyro) {
            return None;
        }
        Some(ImuSample { t_us, accel, gyro })
    }

    /// Time the sample was taken (microseconds, shared clock).
    pub fn t_us(&self) -> u64 {
        self.t_us
    }

    /// Specific force (m/s^2, body frame).
    pub fn accel(&self) -> V3 {
        self.accel
    }

    /// Angular rate (rad/s, body frame).
    pub fn gyro(&self) -> V3 {
        self.gyro
    }
}

/// One magnetometer reading, for Esekf::update_mag.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagSample {
    /// Time the sample was taken (microseconds, shared clock).
    t_us: u64,
    /// Magnetic field (body frame, any consistent unit: only its direction is used). 
    /// calibrated androtated into the body frame before it gets here.
    field: V3,
}

impl MagSample {
    /// None if any component of `field` is not finite, or the field has zero length (no direction).
    pub fn new(t_us: u64, field: V3) -> Option<MagSample> {
        if !finite(&field) || field.norm() == 0.0 {
            return None;
        }
        Some(MagSample { t_us, field })
    }

    /// Time the sample was taken (microseconds, shared clock).
    pub fn t_us(&self) -> u64 {
        self.t_us
    }

    /// Magnetic field (body frame, any consistent unit).
    pub fn field(&self) -> V3 {
        self.field
    }
}

/// One lidar reading.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LidarSample {
    /// Time the sample was taken (microseconds, shared clock).
    t_us: u64,
    /// Distance along the beam (m). NOT height: `Esekf::update_height` needs it corrected for tilt and
    /// mounting first.
    range: f64,
}

impl LidarSample {
    /// None if `range` is not finite or not positive.
    pub fn new(t_us: u64, range: f64) -> Option<LidarSample> {
        if !range.is_finite() || range <= 0.0 {
            return None;
        }
        Some(LidarSample { t_us, range })
    }

    /// Time the sample was taken (microseconds, shared clock).
    pub fn t_us(&self) -> u64 {
        self.t_us
    }

    /// Distance along the beam (m), not height.
    pub fn range(&self) -> f64 {
        self.range
    }
}

/// One GNSS position fix, for Esekf::update_gps_offset or Esekf::update_gps.
///
/// update_gps_offset takes the three components of sigma as they are. update_gps takes a single sigma
/// for all three axes, so callers of that one should pass the largest of the three.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GnssSample {
    /// Time of the fix (microseconds, shared clock).
    t_us: u64,
    /// Position (m, navigation frame: east, north, up) relative to the home point.
    position: V3,
    /// Standard deviation of the position (m), separately for east, north and up.
    sigma: V3,
}

impl GnssSample {
    /// None if any component of position or sigma is not finite, or any sigma is not positive.
    pub fn new(t_us: u64, position: V3, sigma: V3) -> Option<GnssSample> {
        if !finite(&position) || !sigma.iter().all(|s| s.is_finite() && *s > 0.0) {
            return None;
        }
        Some(GnssSample { t_us, position, sigma })
    }

    /// Time of the fix (microseconds, shared clock).
    pub fn t_us(&self) -> u64 {
        self.t_us
    }

    /// Position (m, east-north-up) relative to the home point.
    pub fn position(&self) -> V3 {
        self.position
    }

    /// Standard deviation of the position (m): east, north, up.
    pub fn sigma(&self) -> V3 {
        self.sigma
    }
}
