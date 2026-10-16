//! Monoprop-specific sensor inputs and parameters. PRELIMINARY
//! most values are placeholders to be reviewed (search for PLACEHOLDER and TODO).
//!
//! - inputs: one validated sample type per sensor, with the units and frames the filter expects
//! - params: BMI323 noise values, measurement sigmas, frame assumptions and mounting
//! - imu_pair: the two IMUs combined into one sample at the reference IMU
//! - stillness: running sums of still readings for start-up
//! - navigator: the Navigator, which runs the filter with the Monoprop sensors, start-up and pad logic
//!
//! Not re-exported from the crate root (use Estimation::monoprop::...): the test simulator has its own
//! ImuSample, and a second one at the root would make that name ambiguous in the tests.

mod inputs;
pub use inputs::*;
mod params;
pub use params::*;
mod imu_pair;
pub use imu_pair::*;
mod stillness;
pub use stillness::*;
mod navigator;
pub use navigator::*;
