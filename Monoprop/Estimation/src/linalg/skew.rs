use crate::*;

/// [v]x: the 3x3 matrix with skew(v) * w == v x w for every w.
pub fn skew(v: &V3) -> M3 {
    M3::new(0.0, -v[2], v[1],
            v[2], 0.0, -v[0],
            -v[1], v[0], 0.0)
}
