use crate::*;

impl Esekf {
    /// GPS position fix with per-axis standard deviation sigma.
    pub fn update_gps(&mut self, z: &V3, sigma: f64) -> bool {
        let y = z - self.x.p;
        self.update(&y, &gps_jacobian(), &(M3::identity() * (sigma * sigma)), GATE_3DOF)
    }

    /// Magnetometer reading (body axes, any unit) of the known inertial field direction m_ref (unit vector).
    /// Only the direction is used: the reading is normalised first, and sigma is the per-axis noise of
    /// that unit direction (0.02 is about 1 degree). A zero or non-finite reading is refused.
    pub fn update_mag(&mut self, m_meas: &V3, m_ref: &V3, sigma: f64) -> bool {
        let n = m_meas.norm();
        if !(n > 1e-12) || !n.is_finite() {
            self.rejected += 1;
            return false;
        }
        let y = m_meas / n - mag_prediction(&self.x, m_ref);
        let h = mag_jacobian(&self.x, m_ref);
        self.update(&y, &h, &(M3::identity() * (sigma * sigma)), GATE_3DOF)
    }
}
