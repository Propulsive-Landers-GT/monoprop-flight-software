#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

pub fn run() {
    test("symmetrized averages P and its transpose", || {
        let p = M3::new(4.0, 1.0, 0.0, 1.2, 3.0, 0.5, 0.0, 0.4, 2.0);
        let s = symmetrized(&p);
        assert_eq!(s, s.transpose(), "result must be exactly symmetric:\n{}", s);
        assert_close(s[(1, 0)], 1.1, 1e-15, "average of 1.0 and 1.2");
        assert_close(s[(1, 2)], 0.45, 1e-15, "average of 0.5 and 0.4");
        assert_eq!(s[(0, 0)], 4.0, "the diagonal doesn't change");
    });
    test("symmetrized leaves an already-symmetric matrix alone", || {
        let p = Mat::<2, 2>::new(2.0, 0.3, 0.3, 1.0);
        assert_eq!(symmetrized(&p), p);
    });
    test("the identity is SPD", || {
        assert!(is_spd(&Mat::<15, 15>::identity()));
    });
    test("the worked example [[1, 2], [2, 1]] is not positive definite", || {
        assert!(!is_spd(&Mat::<2, 2>::new(1.0, 2.0, 2.0, 1.0)));
    });
    test("a non-symmetric matrix is rejected even if Cholesky would succeed", || {
        assert!(!is_spd(&Mat::<2, 2>::new(2.0, 0.5, 0.4, 2.0)));
    });
    test("tiny and large variances together are fine (like a real ESEKF covariance)", || {
        assert!(is_spd(&Mat::<2, 2>::new(1e-10, 0.0, 0.0, 1e2)));
    });
    test("rounding-level asymmetry is accepted", || {
        let p = Mat::<2, 2>::new(4.0, 1.0, 1.0 + 1e-15, 3.0);
        assert!(is_spd(&p), "an asymmetry of 1e-15 is just rounding");
    });
}
