//! The generic error-state Kalman filter: predict, gated update, injection and reset.
//!
//! Fixed-size matrices only (no heap), so it runs as-is on a microcontroller.

use super::math::*;
use super::model::BareESEKFModel;

/// What happened to a measurement handed to the filter. Rejections are normal, reportable
/// outcomes (sensor glitches, dropouts, NaNs from a driver), not errors.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UpdateOutcome {
    /// Fused. `nis` is the normalized innovation squared y^T S^-1 y, for consistency checks.
    Fused { nis: f64 },
    /// The innovation contained a NaN or an infinity; nothing changed.
    RejectedNonFinite,
    /// The NIS exceeded the chi-squared gate (an outlier); nothing changed.
    RejectedGate { nis: f64 },
    /// S = H P H^T + R was not positive definite; nothing changed.
    RejectedSingular,
}

impl UpdateOutcome {
    pub fn is_fused(&self) -> bool {
        matches!(self, UpdateOutcome::Fused { .. })
    }
}

/// Chi-squared gates at 99.9 % for 1, 2 and 3 measured numbers.
pub const GATE_1DOF: f64 = 10.83;
pub const GATE_2DOF: f64 = 13.82;
pub const GATE_3DOF: f64 = 16.27;

/// Error-state Kalman filter with an `E`-number error state.
pub struct BareESEKF<T: BareESEKFModel<E>, const E: usize> {
    /// Best estimate of the state.
    pub nominal_state: T::Nominal,
    /// Covariance of the error state (E x E).
    pub error_covariance: Mat<E, E>,
    /// The vehicle/sensor model.
    pub model: T,
    /// Number of measurements refused (non-finite or gated).
    pub rejected: u32,
}

impl<T: BareESEKFModel<E>, const E: usize> BareESEKF<T, E> {
    pub fn new(initial_nominal: T::Nominal, initial_p: Mat<E, E>, model: T) -> Self {
        Self { nominal_state: initial_nominal, error_covariance: initial_p, model, rejected: 0 }
    }

    /// High-rate step: nominal state through the nonlinear model, covariance through F P F^T + Q.
    /// Returns false, changing nothing, if the model says the input can't be used.
    pub fn predict(&mut self, input: &T::Input, dt: f64) -> bool {
        if !self.model.input_is_valid(input, dt) {
            return false;
        }
        // F and Q describe the step starting from the current state: compute them before x moves.
        let f = self.model.error_transition_jacobian(&self.nominal_state, input, dt);
        let q = self.model.process_noise(&self.nominal_state, input, dt);
        self.nominal_state = self.model.nominal_prediction(&self.nominal_state, input, dt);
        self.error_covariance = symmetrized(&(f * self.error_covariance * f.transpose() + q));
        true
    }

    /// Fuse one measurement, given as its innovation y = z - h(x) (`M` numbers), its Jacobian H with
    /// respect to the error state, its noise covariance R, and a chi-squared gate on the NIS.
    pub fn update<const M: usize>(&mut self, y: &Vector<M>, h: &Mat<M, E>, r: &Mat<M, M>, gate: f64) -> UpdateOutcome {
        if !y.iter().all(|v| v.is_finite()) {
            self.rejected += 1;
            return UpdateOutcome::RejectedNonFinite;
        }
        let pht = self.error_covariance * h.transpose();
        let s = h * pht + r;
        let s_inv_y = match solve_spd(&s, y) {
            Some(v) => v,
            None => return UpdateOutcome::RejectedSingular,
        };
        let nis = y.dot(&s_inv_y);
        // `!(nis <= gate)` also refuses a NaN NIS (every comparison with NaN is false).
        if !(nis <= gate) {
            self.rejected += 1;
            return UpdateOutcome::RejectedGate { nis };
        }
        // K = P H^T S^-1, computed as (S^-1 H P)^T because S and P are symmetric.
        let k = match solve_spd(&s, &pht.transpose()) {
            Some(kt) => kt.transpose(),
            None => return UpdateOutcome::RejectedSingular,
        };
        let dx = k * y;
        // Joseph form: stays symmetric positive definite even with rounding.
        let ikh = Mat::<E, E>::identity() - k * h;
        let p = ikh * self.error_covariance * ikh.transpose() + k * r * k.transpose();
        self.nominal_state = self.model.inject_error(&self.nominal_state, &dx);
        let g = self.model.reset_jacobian(&dx);
        self.error_covariance = symmetrized(&(g * p * g.transpose()));
        UpdateOutcome::Fused { nis }
    }

    pub fn get_state(&self) -> &T::Nominal {
        &self.nominal_state
    }

    pub fn get_covariance(&self) -> &Mat<E, E> {
        &self.error_covariance
    }
}
