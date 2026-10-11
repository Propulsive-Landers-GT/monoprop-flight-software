use crate::*;

/// TRIAD: attitude (body -> inertial) from one accelerometer and one magnetometer reading.
/// At rest the accelerometer reads "up" (0, 0, 1) in inertial axes; it is the primary (trusted) vector.
/// None if the readings can't define an attitude: a zero vector, or the two vectors (almost) parallel.
pub fn triad(f_body: &V3, m_body: &V3, m_ref: &V3) -> Option<Quat> {
    let frame = |a: &V3, b: &V3| -> Option<M3> {
        let c = a.cross(b);
        if !(c.norm() > 0.01 * a.norm() * b.norm()) {
            return None; // parallel, zero or NaN: the second axis is undefined
        }
        let t1 = a.normalize();
        let t2 = c.normalize();
        let t3 = t1.cross(&t2);
        Some(M3::from_columns(&[t1, t2, t3]))
    };
    let r = frame(&V3::z(), m_ref)? * frame(f_body, m_body)?.transpose();
    Some(Quat::from_rotation_matrix(&na::Rotation3::from_matrix_unchecked(r)))
}
