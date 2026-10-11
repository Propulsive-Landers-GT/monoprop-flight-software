use crate::*;

/// Hamilton product p (x) q.
pub fn hamilton(p: &Quaternion<f64>, q: &Quaternion<f64>) -> Quaternion<f64> {
    Quaternion::new(p.w * q.w - p.i * q.i - p.j * q.j - p.k * q.k,
                    p.w * q.i + p.i * q.w + p.j * q.k - p.k * q.j,
                    p.w * q.j - p.i * q.k + p.j * q.w + p.k * q.i,
                    p.w * q.k + p.i * q.j - p.j * q.i + p.k * q.w
    )
}

/// Rotation matrix R(q): body to inertial.
pub fn quat_to_rot(q: &Quat) -> M3 {
    let (w, x, y, z) = (q.w, q.i, q.j, q.k);
    M3::new(1.0 - 2.0 * (y*y + z*z), 2.0*(x*y - w*z), 2.0*(x*z+w*y),
            2.0*(x*y+w*z), 1.0 - 2.0 * (x*x + z*z), 2.0 * (y*z-w*x),
            2.0*(x*z - w*y), 2.0*(y*z+w*x), 1.0 - 2.0 * (x*x + y*y)
    )
}
