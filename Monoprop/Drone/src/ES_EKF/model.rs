//! ES-EKF model trait, ported from rust-ekf/src/es_ekf/model.rs.
//!
//! Changes from the previous model.rs:
//!   - ndarray -> fixed-size nalgebra types (no heap on the Teensy). State sizes
//!     are const generics, so one filter still works for any model.
//!   - IMU input is a typed `Input` instead of `&[f64]`
//!   - added `process_noise(dt)` so Q tracks the actual step size
//!   - removed `measurement_prediction` / `measurement_jacobian`; each sensor
//!     now implements `Measurement` (measurement.rs)

use nalgebra::{SMatrix, SVector};

/// N = nominal state size, E = error state size (e.g. 16 and 15 for full state).
pub trait ESEKFModel<const N: usize, const E: usize> {
    type Input;

    /// Propagate the nominal state with one IMU sample.
    fn nominal_prediction(&self, nominal_state: &SVector<f32, N>, imu: &Self::Input, dt: f32) -> SVector<f32, N>;

    /// Error-state transition Jacobian F, evaluated at the start of the step.
    fn error_transition_jacobian(&self, nominal_state: &SVector<f32, N>, imu: &Self::Input, dt: f32) -> SMatrix<f32, E, E>;

    /// Discrete process noise Q for this dt.
    fn process_noise(&self, nominal_state: &SVector<f32, N>, dt: f32) -> SMatrix<f32, E, E>;

    /// Apply a correction to the nominal state and renormalize the quaternion.
    fn inject_error(&self, nominal_state: &SVector<f32, N>, error_state: &SVector<f32, E>) -> SVector<f32, N>;
}
