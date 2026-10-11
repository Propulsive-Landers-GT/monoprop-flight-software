#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

fn spd4() -> Mat<4, 4> {
    let a = Mat::<4, 4>::new(2.0, 0.3, -0.1, 0.5, 0.1, 1.5, 0.4, 0.0, 0.0, -0.2, 1.0, 0.3, 0.6, 0.1, 0.0, 1.2);
    a * a.transpose() + Mat::<4, 4>::identity() * 0.1
}

pub fn run() {
    test("worked example: [[4, 2], [2, 3]] gives [[2, 0], [1, sqrt 2]]", || {
        let l = cholesky_lower(&Mat::<2, 2>::new(4.0, 2.0, 2.0, 3.0)).expect("this matrix is SPD, expected Some");
        assert!((l - Mat::<2, 2>::new(2.0, 0.0, 1.0, 2f64.sqrt())).amax() < 1e-12, "got\n{}", l);
    });
    test("worked example: [[1, 2], [2, 1]] is not positive definite", || {
        assert!(cholesky_lower(&Mat::<2, 2>::new(1.0, 2.0, 2.0, 1.0)).is_none());
    });
    test("4x4: lower-triangular, L L^T = A, and the same as nalgebra's", || {
        let a = spd4();
        let l = cholesky_lower(&a).expect("SPD matrix");
        for i in 0..4 {
            for j in (i + 1)..4 {
                assert_eq!(l[(i, j)], 0.0, "entry ({}, {}) is above the diagonal and must be zero", i, j);
            }
        }
        assert!((l * l.transpose() - a).amax() < 1e-12, "L L^T != A");
        assert!((l - a.cholesky().unwrap().l()).amax() < 1e-12, "differs from nalgebra's L");
    });
    test("zero and NaN matrices give None", || {
        assert!(cholesky_lower(&Mat::<3, 3>::zeros()).is_none(), "zero matrix");
        assert!(cholesky_lower(&Mat::<2, 2>::new(f64::NAN, 0.0, 0.0, 1.0)).is_none(), "NaN");
    });
    test("15x15 covariance", || {
        let mut p = Mat::<15, 15>::identity();
        for i in 0..14 {
            p[(i, i + 1)] = 0.3;
            p[(i + 1, i)] = 0.3;
        }
        let l = cholesky_lower(&p).expect("SPD");
        assert!((l * l.transpose() - p).amax() < 1e-12);
    });
}
