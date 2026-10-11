use crate::*;

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

/// Inverse of a symmetric positive definite matrix.
pub fn inverse_spd<const N: usize>(a: &Mat<N, N>) -> Option<Mat<N, N>> {
    solve_spd(a, &Mat::<N, N>::identity())
}
