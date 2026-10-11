use crate::*;

/// N x N matrix with d on the diagonal and zeros elsewhere.
pub fn diag<const N: usize>(d: [f64; N]) -> Mat<N, N> {
    let mut m = Mat::<N, N>::zeros();
    for (index, value) in d.iter().enumerate() {
        m[(index, index)] = d[index];
    }
    m
}

/// Outer product a b^T: entry (i, j) is a[i] * b[j].
pub fn outer<const R: usize, const C: usize>(a: &Vector<R>, b: &Vector<C>) -> Mat<R, C> {
    let mut m = Mat::<R, C>::zeros();
    for (i, v1) in a.iter().enumerate() {
        for (j, v2) in b.iter().enumerate() {
            m[(i, j)] = a[i] * b[j];
        } 
    } 
    m
}
