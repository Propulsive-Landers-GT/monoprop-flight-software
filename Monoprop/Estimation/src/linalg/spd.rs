use crate::*;

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
