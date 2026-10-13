//! Monoprop-specific sensor inputs and parameters. PRELIMINARY
//! most values are placeholders to be reviewed (search for PLACEHOLDER and TODO).
//!
//! - inputs: one validated sample type per sensor, with the units and frames the filter expects
//! - params: BMI323 noise values, measurement sigmas, frame assumptions and mounting
//!
//! Not re-exported from the crate root (use Estimation::monoprop::...): the test simulator has its own
//! ImuSample, and a second one at the root would make that name ambiguous in the tests.

mod inputs;
pub use inputs::*;
mod params;
pub use params::*;
