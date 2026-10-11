use crate::*;

/// Copy the 3x3 matrix b into m at rows r0..r0+3, columns c0..c0+3 (modifies m).
pub fn set_block3<const R: usize, const C: usize>(m: &mut Mat<R, C>, r0: usize, c0: usize, b: &M3) {
    m.fixed_view_mut::<3,3>(r0,c0).copy_from(b);
}

/// The 3-vector (x[i], x[i+1], x[i+2]).
pub fn sub3<const N: usize>(x: &Vector<N>, i: usize) -> V3 {
    V3::new(x[i], x[i+1], x[i+2])
    
}
