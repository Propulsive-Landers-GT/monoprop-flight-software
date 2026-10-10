//! Vehicle-independent IMU abstraction.
//!
//! Every IMU driver (VN-200, BNO085, ICM-42688, ...) returns [`ImuSample`]s through the
//! [`Imu`] trait, so navigation code never depends on a particular sensor.
//!
//! # Conventions
//!
//! - **Units are SI only:** m/s^2, rad/s, tesla, degrees Celsius, pascal, seconds.
//! - **Body frame:** right-handed, +Z along the thrust axis, pointing up when the vehicle
//!   sits on the pad. A level vehicle at rest reads about +9.81 m/s^2 on Z (specific force).
//!   This is the frame the navigation ES-EKF assumes.
//! - **Drivers do the conversion.** A driver converts vendor units (see [`units`]) and
//!   rotates from the sensor's axes into the body frame (see [`Mounting`]) before it
//!   returns a sample. Consumers never see raw counts, vendor units or sensor axes.
//! - Accelerometer and gyroscope are always present. Everything else is optional,
//!   because only some IMUs provide it; [`ImuInfo::capabilities`] says which.

#![no_std]

mod info;
mod mounting;
mod sample;
mod static_imu;
mod traits;
pub mod units;

pub use info::{Capabilities, ImuInfo, NoiseSpec};
pub use mounting::Mounting;
pub use sample::{DeltaIntegrals, ImuSample, ImuStatus, Vec3};
pub use static_imu::StaticImu;
pub use traits::Imu;
