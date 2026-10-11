use crate::*;

/// Chi-squared threshold, 3 degrees of freedom, 99.9 %.
pub const GATE_3DOF: f64 = 16.27;

/// G: re-expresses the error covariance after the nominal attitude moved by dtheta.
pub fn reset_jacobian(dtheta: &V3) -> Mat<15, 15> {
    let mut g = Mat::<15, 15>::identity();
    set_block3(&mut g, 6, 6, &(M3::identity() - skew(dtheta) * 0.5));
    g
}

impl Esekf {
    /// Generic update with innovation y, Jacobian h (w.r.t. the error state) and noise r.
    /// Returns false, changing nothing but `rejected`, if y is not finite or the NIS exceeds `gate`
    /// (or false with no change at all if S is not positive definite).
    pub fn update<const M: usize>(&mut self, y: &Vector<M>, h: &Mat<M, 15>, r: &Mat<M, M>, gate: f64) -> bool {
        if !y.iter().all(|v| v.is_finite()) {
            self.rejected += 1; // a NaN or infinite reading: refuse it like an outlier
            return false;
        }
        let pht = self.p * h.transpose();
        let s = h * pht + r;
        let s_inv_y = match solve_spd(&s, y) {
            Some(v) => v,
            None => return false,
        };
        let nis = y.dot(&s_inv_y);
        if !(nis <= gate) {
            // written so that a NaN NIS is refused too (every comparison with NaN is false)
            self.rejected += 1;
            return false;
        }
        // K = P H^T S^-1, computed as (S^-1 H P)^T because S and P are symmetric.
        let k = solve_spd(&s, &pht.transpose()).unwrap().transpose();
        let dx = k * y;
        let ikh = Mat::<15, 15>::identity() - k * h;
        let p = ikh * self.p * ikh.transpose() + k * r * k.transpose();
        self.x = inject(&self.x, &dx);
        let g = reset_jacobian(&sub3(&dx, 6));
        self.p = symmetrized(&(g * p * g.transpose()));
        true
    }
}
