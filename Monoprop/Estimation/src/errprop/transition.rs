use crate::*;

/// F: how an error in the nominal state becomes an error one IMU step later.
pub fn error_jacobian(x: &Nominal, am: &V3, wm: &V3, dt: f64) -> Mat<15, 15> {
    let r = quat_to_rot(&x.q);
    let mut f = Mat::<15, 15>::identity();
    set_block3(&mut f, 0, 3, &(M3::identity() * dt));
    set_block3(&mut f, 3, 6, &(-(r * skew(&(am - x.ab))) * dt));
    set_block3(&mut f, 3, 9, &(-r * dt));
    set_block3(&mut f, 6, 6, &exp_so3(&((wm - x.wb) * dt)).transpose());
    set_block3(&mut f, 6, 12, &(M3::identity() * -dt));
    f
}
