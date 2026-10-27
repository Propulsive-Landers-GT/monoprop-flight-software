//! Linear algebra and rotation helpers on fixed-size nalgebra types (no heap).
//!
//! Conventions: Hamilton quaternions; `q * v` rotates a body-frame vector into the navigation frame;
//! attitude errors are local (right) perturbations, `q (+) dtheta = q * Exp(dtheta)`.
#![allow(unused_variables, unused_mut, clippy::needless_return, clippy::needless_range_loop)]

pub use core::f64::consts::PI;
pub use nalgebra as na;
#[allow(unused_imports)]
pub use nalgebra::{ComplexField, Matrix3, Quaternion, RealField, SMatrix, SVector, UnitQuaternion, Vector3};

pub type V3 = Vector3<f64>;
pub type M3 = Matrix3<f64>;
pub type Mat<const R: usize, const C: usize> = SMatrix<f64, R, C>;
pub type Vector<const N: usize> = SVector<f64, N>;
/// Attitude quaternion (unit, Hamilton). `q * v` rotates a body-frame vector into the navigation frame.
pub type Quat = UnitQuaternion<f64>;

// ---- matrices ----

/// N x N matrix with d on the diagonal and zeros elsewhere.
pub fn diag<const N: usize>(d: [f64; N]) -> Mat<N, N> {
    let mut m = Mat::<N, N>::zeros();
    for (index, value) in d.iter().enumerate() {
        m[(index, index)] = d[index];
    }
    m
}

/// [v]x: the 3x3 matrix with skew(v) * w == v x w for every w.
pub fn skew(v: &V3) -> M3 {
    M3::new(0.0, -v[2], v[1],
            v[2], 0.0, -v[0],
            -v[1], v[0], 0.0)
}

/// Copy the 3x3 matrix b into m at rows r0..r0+3, columns c0..c0+3 (modifies m).
pub fn set_block3<const R: usize, const C: usize>(m: &mut Mat<R, C>, r0: usize, c0: usize, b: &M3) {
    m.fixed_view_mut::<3,3>(r0,c0).copy_from(b);
}

/// The 3-vector (x[i], x[i+1], x[i+2]).
pub fn sub3<const N: usize>(x: &Vector<N>, i: usize) -> V3 {
    V3::new(x[i], x[i+1], x[i+2])

}

/// (P + P^T) / 2: exactly symmetric, removes the asymmetry that rounding adds.
pub fn symmetrized<const N: usize>(p: &Mat<N, N>) -> Mat<N, N> {
    (0.5) * (p + p.transpose())
}

/// Symmetric (to 1e-9 relative to the largest entry) and positive definite.
pub fn is_spd<const N: usize>(p: &Mat<N, N>) -> bool {
    // multiplied by 1e-9 so that the "scale" of the numbers is taken into consideration
    // if the numbers are really big, the error might occur a few decimal places earlier
    // rather than after the dot
    (p - p.transpose()).amax() < 1e-9 * p.amax() && p.cholesky().is_some()
}

/// Lower-triangular L with L L^T = a, or None if a is not positive definite.
pub fn cholesky_lower<const N: usize>(a: &Mat<N, N>) -> Option<Mat<N, N>> {
    let mut l = Mat::<N, N>::zeros();
    for i in 0..N {
        for j in 0..=i {
            let mut s = 0.0;
            for k in 0..j {
                s += l[(i,k)] * l[(j,k)];
            }
            s = a[(i,j)] - s;
            if (s <= 0.0 && i == j) || !s.is_finite() {
                return None;
            }
            else if i == j {
                l[(i,j)] = s.sqrt();
            }
            else {
                l[(i, j)] = s / l[(j, j)]
            }
        }
    }
    Some(l)
}

/// Solve (L L^T) X = B by forward then back substitution, column by column.
pub fn cholesky_solve<const N: usize, const K: usize>(l: &Mat<N, N>, b: &Mat<N, K>) -> Mat<N, K> {
    let mut x = *b;
    for c in 0..K {
        for i in 0..N {
            let mut s = x[(i, c)];
            for k in 0..i {
                s -= l[(i, k)] * x[(k, c)];
            }
            x[(i, c)] = s / l[(i, i)];
        }
        for i in (0..N).rev() {
            let mut s = x[(i, c)];
            for k in (i + 1)..N {
                s -= l[(k, i)] * x[(k, c)];
            }
            x[(i, c)] = s / l[(i, i)];
        }
    }
    x
}

/// Solve A X = B for symmetric positive definite A. None if A is not positive definite.
pub fn solve_spd<const N: usize, const K: usize>(a: &Mat<N, N>, b: &Mat<N, K>) -> Option<Mat<N, K>> {
    // Computes the lower cholesky decomposition
    // if none, return none
    // else, solve
    let mut l = cholesky_lower(a)?;
    Some(cholesky_solve(&l,b))
}

// ---- rotations ----

/// Rotation matrix R(q): body to inertial.
pub fn quat_to_rot(q: &Quat) -> M3 {
    let (w, x, y, z) = (q.w, q.i, q.j, q.k);
    M3::new(1.0 - 2.0 * (y*y + z*z), 2.0*(x*y - w*z), 2.0*(x*z+w*y),
            2.0*(x*y+w*z), 1.0 - 2.0 * (x*x + z*z), 2.0 * (y*z-w*x),
            2.0*(x*z - w*y), 2.0*(y*z+w*x), 1.0 - 2.0 * (x*x + y*y)
    )
}

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

/// q (+) dtheta = q * Exp(dtheta): a small rotation in body axes.
pub fn boxplus(q: &Quat, dtheta: &V3) -> Quat {
    q * quat_exp(dtheta)
}

/// q1 (-) q2 = Log(q2^-1 * q1): the body-frame rotation taking q2 to q1.
pub fn boxminus(q1: &Quat, q2: &Quat) -> V3 {
    quat_log(&(q2.inverse() * q1))
}

