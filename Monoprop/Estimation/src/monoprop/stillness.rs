//! Stillness window for start-up: running sums of the IMU and magnetometer readings taken while the vehicle
//! is still, so the alignment can use their means. Fixed-size sums, no stored samples.
use crate::*;

/// Whether one IMU reading looks still: the specific force is gravity (within `accel_tol`, m/s^2) and the
/// angular rate is small (at most `gyro_max`, rad/s; it must allow for the not yet known gyro bias).
pub fn is_still(accel: &V3, gyro: &V3, gravity: f64, accel_tol: f64, gyro_max: f64) -> bool {
    (accel.norm() - gravity).abs() <= accel_tol && gyro.norm() <= gyro_max
}

/// Running sums of still IMU readings and of magnetometer readings since the window was last cleared.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StillnessWindow {
    accel_sum: V3,
    gyro_sum: V3,
    imu_count: u32,
    mag_sum: V3,
    mag_count: u32,
}

impl Default for StillnessWindow {
    fn default() -> Self {
        StillnessWindow::new()
    }
}

impl StillnessWindow {
    pub fn new() -> Self {
        StillnessWindow { accel_sum: V3::zeros(), gyro_sum: V3::zeros(), imu_count: 0, mag_sum: V3::zeros(), mag_count: 0 }
    }

    /// Start again: forget everything collected so far.
    pub fn clear(&mut self) {
        *self = StillnessWindow::new();
    }

    /// Add an IMU reading if it is still (see `is_still`) and return true; a reading that is not still
    /// clears the window (the vehicle moved) and returns false.
    pub fn add_imu(&mut self, accel: &V3, gyro: &V3, gravity: f64, accel_tol: f64, gyro_max: f64) -> bool {
        if !is_still(accel, gyro, gravity, accel_tol, gyro_max) {
            self.clear();
            return false;
        }
        self.accel_sum += accel;
        self.gyro_sum += gyro;
        self.imu_count = self.imu_count.saturating_add(1);
        true
    }

    /// Add a magnetometer reading (body frame, any consistent unit).
    pub fn add_mag(&mut self, field: &V3) {
        self.mag_sum += field;
        self.mag_count = self.mag_count.saturating_add(1);
    }

    /// Number of still IMU readings collected.
    pub fn imu_count(&self) -> u32 {
        self.imu_count
    }

    /// Number of magnetometer readings collected.
    pub fn mag_count(&self) -> u32 {
        self.mag_count
    }

    /// Mean still specific force (m/s^2, body frame); None if no reading yet.
    pub fn mean_accel(&self) -> Option<V3> {
        mean(&self.accel_sum, self.imu_count)
    }

    /// Mean still angular rate (rad/s, body frame); None if no reading yet.
    pub fn mean_gyro(&self) -> Option<V3> {
        mean(&self.gyro_sum, self.imu_count)
    }

    /// Mean magnetometer reading (body frame); None if no reading yet.
    pub fn mean_mag(&self) -> Option<V3> {
        mean(&self.mag_sum, self.mag_count)
    }
}

fn mean(sum: &V3, count: u32) -> Option<V3> {
    if count == 0 { None } else { Some(sum / f64::from(count)) }
}
