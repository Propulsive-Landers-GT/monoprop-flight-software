//! The 15-state error-state Kalman filter.
//!
//! - `filter`: the Esekf struct and predict
//! - `models`: position and magnetometer measurement models and Jacobians
//! - `update`: the generic gated update with injection and covariance reset
//! - `sensors`: position (GPS / UWB) and magnetometer updates
//! - `height`: height (barometer / downward lidar) updates
//! - `lidar`: lidar range updates (distance along the beam, tilt handled by the model)
//! - `gps_offset`: position (GPS) updates with an antenna offset and a sigma per axis
//! - `stationary`: zero-velocity and zero angular-rate updates while standing still (on the pad)

mod filter;
pub use filter::*;
mod models;
pub use models::*;
mod update;
pub use update::*;
mod sensors;
pub use sensors::*;
mod height;
pub use height::*;
mod lidar;
pub use lidar::*;
mod gps_offset;
pub use gps_offset::*;
mod stationary;
pub use stationary::*;
