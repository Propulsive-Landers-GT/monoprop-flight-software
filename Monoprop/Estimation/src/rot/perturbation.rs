use crate::*;

/// q (+) dtheta = q * Exp(dtheta): a small rotation in body axes.
pub fn boxplus(q: &Quat, dtheta: &V3) -> Quat {
    q * quat_exp(dtheta)
}

/// q1 (-) q2 = Log(q2^-1 * q1): the body-frame rotation taking q2 to q1.
pub fn boxminus(q1: &Quat, q2: &Quat) -> V3 {
    quat_log(&(q2.inverse() * q1))
}

/// Attitude after turning at body rate w for dt.
pub fn integrate_gyro(q: &Quat, w: &V3, dt: f64) -> Quat {
    let mut a = boxplus(q, &(w * dt));
    a.renormalize();
    a
}
