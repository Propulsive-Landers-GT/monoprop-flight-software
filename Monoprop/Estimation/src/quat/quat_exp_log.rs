use crate::*;

/// Exp map: the unit quaternion of a rotation vector (axis * angle, radians).
pub fn quat_exp(r: &V3) -> Quat {
    let phi = r.norm();

    // Small-angle limit: sin(phi/2)/phi -> 1/2 - phi^2/48 (avoids 0/0 at phi = 0).
    let scale = if phi < 1e-8 { 0.5 - phi * phi / 48.0 } else { (phi / 2.0).sin() / phi };
    let q = Quaternion::new(
        (phi / 2.0).cos(), 
        r[0] * scale, 
        r[1] * scale, 
        r[2] * scale
    );
    Quat::new_normalize(q)
}

/// Log map: rotation vector of a unit quaternion, angle in [0, pi].
pub fn quat_log(q: &Quat) -> V3 {
    let (w, i, j, k) = if q.w < 0.0 {
        (-q.w, -q.i, -q.j, -q.k)
    } else {
        (q.w, q.i, q.j, q.k)
    };

    let v_norm = (i * i + j * j + k * k).sqrt();
    
    let phi = 2.0 * v_norm.atan2(w);
    // Small-angle limit: phi = 2 atan2(v_norm, w) ~ 2 v_norm / w (avoids 0/0 at the identity).
    let scale = if v_norm < 1e-12 { 2.0 / w } else { phi / v_norm };
    
    V3::new(i * scale, j * scale, k * scale)
}
