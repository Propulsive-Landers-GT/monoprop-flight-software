//! Start-up: TRIAD alignment and the initial covariance.
//!
//! - `triad`: TRIAD: attitude from one accelerometer and one magnetometer reading
//! - `start`: initial covariance and align (building the filter from the first readings)

mod triad;
pub use triad::*;
mod start;
pub use start::*;
