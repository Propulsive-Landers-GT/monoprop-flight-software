//! The nominal state, strapdown propagation, and the error state.
//!
//! - `strapdown`: the nominal state and strapdown propagation
//! - `error_state`: inject / difference between the nominal state and the 15-number error state

mod strapdown;
pub use strapdown::*;
mod error_state;
pub use error_state::*;
