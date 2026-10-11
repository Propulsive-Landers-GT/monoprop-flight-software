use crate::*;

pub struct Esekf {
    pub x: Nominal,     // nominal (best-guess) state
    pub p: Mat<15, 15>, // covariance of the error state
    pub spec: ImuSpec,  // IMU noise parameters, for Q
    pub rejected: u32,  // measurements refused by the gate
}

impl Esekf {
    pub fn new(x: Nominal, p: Mat<15, 15>, spec: ImuSpec) -> Self {
        Esekf { x, p, spec, rejected: 0 }
    }

    /// IMU step: nominal state through the strapdown equations, covariance through F.
    /// Returns false, changing nothing, if a reading or dt is not a finite number (or dt <= 0):
    /// one NaN from a sensor driver would otherwise corrupt the whole state in a single step.
    pub fn predict(&mut self, am: &V3, wm: &V3, dt: f64) -> bool {
        if !(dt > 0.0) || !dt.is_finite() || !am.iter().chain(wm.iter()).all(|v| v.is_finite()) {
            return false;
        }
        let f = error_jacobian(&self.x, am, wm, dt);
        self.x = propagate_nominal(&self.x, am, wm, dt);
        self.p = symmetrized(&(f * self.p * f.transpose() + process_noise(&self.spec, dt)));
        true
    }
}
