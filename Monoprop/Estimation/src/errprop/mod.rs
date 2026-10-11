//! Error-state propagation: F and Q.
//!
//! - `transition`: the error-state transition matrix F
//! - `process_noise`: the process noise Q

mod transition;
pub use transition::*;
mod process_noise;
pub use process_noise::*;
