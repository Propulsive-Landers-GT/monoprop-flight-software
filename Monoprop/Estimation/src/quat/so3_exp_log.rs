use crate::*;

/// Rodrigues' formula: the rotation matrix Exp(r).
pub fn exp_so3(r: &V3) -> M3 {
    let theta_sq = r.x * r.x + r.y * r.y + r.z * r.z;
    let theta = theta_sq.sqrt();

    // Use Taylor series approximation for small angles to avoid division by zero
    let (a, b) = if theta_sq < 1e-6 {
        (1.0 - theta_sq / 6.0, 0.5 - theta_sq / 24.0)
    } else {
        (theta.sin() / theta, (1.0 - theta.cos()) / theta_sq)
    };

    let wx2 = r.x * r.x;
    let wy2 = r.y * r.y;
    let wz2 = r.z * r.z;

    // Scale-free Rodrigues formulation
    M3::new(
        1.0 - b * (wy2 + wz2),   b * r.x * r.y - a * r.z, b * r.x * r.z + a * r.y,
        b * r.x * r.y + a * r.z, 1.0 - b * (wx2 + wz2),   b * r.y * r.z - a * r.x,
        b * r.x * r.z - a * r.y, b * r.y * r.z + a * r.x, 1.0 - b * (wx2 + wy2),
    )
}

/// Log map of a rotation matrix (angle in [0, pi)).
pub fn log_so3(m: &M3) -> V3 {
    let x = m[(2, 1)] - m[(1, 2)];
    let y = m[(0, 2)] - m[(2, 0)];
    let z = m[(1, 0)] - m[(0, 1)];

    // Magnitude of the skew-symmetric part is 2 * sin(theta)
    let sin_theta_2 = (x * x + y * y + z * z).sqrt();
    let trace = m[(0, 0)] + m[(1, 1)] + m[(2, 2)];
    
    // atan2(2*sin(theta), 2*cos(theta)) is much more stable than acos(trace)
    let theta = sin_theta_2.atan2(trace - 1.0);

    let k = if theta < 1e-5 {
        // Taylor expansion of (theta / (2 * sin(theta))) around 0
        0.5 + (theta * theta) / 12.0
    } else {
        theta / sin_theta_2
    };

    V3::new(x * k, y * k, z * k)
}
