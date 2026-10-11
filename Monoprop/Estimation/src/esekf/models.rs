use crate::*;

/// GPS measures position: H = [I 0 0 0 0].
pub fn gps_jacobian() -> Mat<3, 15> {
    let mut h = Mat::<3, 15>::zeros();
    set_block3(&mut h, 0, 0, &M3::identity());
    h
}

/// What the magnetometer should read: the reference field in body axes, R^T m.
pub fn mag_prediction(x: &Nominal, m_ref: &V3) -> V3 {
    x.q.inverse() * m_ref
}

/// d(mag prediction)/d(error state): only the attitude block is non-zero, [R^T m]x.
pub fn mag_jacobian(x: &Nominal, m_ref: &V3) -> Mat<3, 15> {
    let mut h = Mat::<3, 15>::zeros();
    set_block3(&mut h, 0, 6, &skew(&mag_prediction(x, m_ref)));
    h
}
