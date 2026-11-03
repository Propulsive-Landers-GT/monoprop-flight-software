//! The model trait: everything that is specific to one vehicle and sensor set.
//!
//! The filter in `filter.rs` is generic: it only knows how to propagate a covariance, fuse a
//! measurement and inject a correction. A model supplies the physics. To estimate something new,
//! implement this trait for a new struct (see `full_state.rs` for a complete example) and use
//! `BareESEKF<YourModel, E>`.

use super::math::*;

/// A model for the error-state filter, with an error state of `E` numbers.
///
/// The nominal state can have more numbers than the error state (a quaternion has 4 numbers but
/// only 3 degrees of freedom), which is why it is an associated type rather than a vector.
pub trait BareESEKFModel<const E: usize> {
    /// The nominal (best-guess) state, e.g. position, velocity, attitude quaternion, biases.
    /// `Copy` so it can live on the stack and be replaced atomically.
    type Nominal: Copy;
    /// One high-rate input sample, e.g. an accelerometer and gyro reading.
    type Input;

    /// False if the sample or the time step can't be used (NaN, infinity, dt <= 0).
    /// The filter then skips the step and changes nothing.
    fn input_is_valid(&self, input: &Self::Input, dt: f64) -> bool;

    /// Propagate the nominal state through the nonlinear dynamics for one step.
    fn nominal_prediction(&self, x: &Self::Nominal, input: &Self::Input, dt: f64) -> Self::Nominal;

    /// F: the error-state transition matrix for this step, evaluated at the state at the start of the step.
    fn error_transition_jacobian(&self, x: &Self::Nominal, input: &Self::Input, dt: f64) -> Mat<E, E>;

    /// Q: covariance of the noise added to the error state during this step.
    fn process_noise(&self, x: &Self::Nominal, input: &Self::Input, dt: f64) -> Mat<E, E>;

    /// x (+) dx: apply an error-state correction to the nominal state.
    fn inject_error(&self, x: &Self::Nominal, dx: &Vector<E>) -> Self::Nominal;

    /// G: re-expresses the covariance after `dx` was injected (needed when part of the error state is
    /// defined relative to the nominal, like a local attitude error). Identity if all errors are additive.
    fn reset_jacobian(&self, _dx: &Vector<E>) -> Mat<E, E> {
        Mat::<E, E>::identity()
    }
}
