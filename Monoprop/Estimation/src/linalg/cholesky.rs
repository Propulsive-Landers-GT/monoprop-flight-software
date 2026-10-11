use crate::*;

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
