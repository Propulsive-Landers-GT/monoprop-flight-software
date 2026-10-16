//! GPS position updates for an antenna that is not at the reference point of the vehicle, with a separate
//! standard deviation for east, north and up. `update_gps` (sensors.rs) is the simpler model: antenna at
//! the reference point, one sigma for all three axes.
//!
//! Model, with R the attitude matrix and a the antenna's position in the body frame:
//!   predicted fix h = p + R a.
//! Jacobian w.r.t. the error state (attitude error in body axes, R <- R (I + skew(dtheta))):
//!   d h / d(position) = I,   d h / d(attitude) = -R skew(a).
//! With a = 0 this is exactly the `update_gps` model.
use crate::*;

/// Where the antenna should be (m, navigation frame): the vehicle's position plus the rotated offset.
pub fn gps_offset_prediction(x: &Nominal, antenna_offset: &V3) -> V3 {
    x.p + x.q * antenna_offset
}

/// H for a fix taken at the antenna: 3 x 15, identity in the position columns and -R [a]x in the attitude columns.
pub fn gps_offset_jacobian(x: &Nominal, antenna_offset: &V3) -> Mat<3, 15> {
    let mut h = Mat::<3, 15>::zeros();
    set_block3(&mut h, 0, 0, &M3::identity());
    set_block3(&mut h, 0, 6, &(-(quat_to_rot(&x.q) * skew(antenna_offset))));
    h
}

impl Esekf {
    /// GPS position fix z (m, navigation frame: east, north, up) taken at an antenna mounted at
    /// `antenna_offset` (m, body frame), with standard deviations `sigma` (m) for east, north and up.
    /// Returns false, changing nothing, if a sigma is not positive and finite or the offset is not
    /// finite (not counted in `rejected`: these are not readings), or if the fix is not finite or is
    /// an outlier by the 3-DOF gate (counted, as for `update_gps`).
    pub fn update_gps_offset(&mut self, z: &V3, sigma: &V3, antenna_offset: &V3) -> bool {
        let sigma_ok = sigma.iter().all(|s| s.is_finite() && *s > 0.0);
        if !sigma_ok || !antenna_offset.iter().all(|v| v.is_finite()) {
            return false;
        }
        let y = z - gps_offset_prediction(&self.x, antenna_offset);
        let h = gps_offset_jacobian(&self.x, antenna_offset);
        let r = diag([sigma.x * sigma.x, sigma.y * sigma.y, sigma.z * sigma.z]);
        self.update(&y, &h, &r, GATE_3DOF)
    }
}
