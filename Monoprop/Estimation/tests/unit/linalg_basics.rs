#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

pub fn run() {
    test("diag([1, 0.25]) is [[1, 0], [0, 0.25]]", || {
        let m = diag([1.0, 0.25]);
        assert_eq!(m, Mat::<2, 2>::new(1.0, 0.0, 0.0, 0.25), "got\n{}", m);
    });
    test("diag works for any size (a 15x15 initial covariance)", || {
        let d: [f64; 15] = core::array::from_fn(|i| (i + 1) as f64);
        let m = diag(d);
        for i in 0..15 {
            for j in 0..15 {
                let want = if i == j { (i + 1) as f64 } else { 0.0 };
                assert_eq!(m[(i, j)], want, "entry ({}, {})", i, j);
            }
        }
    });
    test("outer((1, 2), (3, 4, 5)) is the worked example", || {
        let m = outer(&Vector::<2>::new(1.0, 2.0), &V3::new(3.0, 4.0, 5.0));
        assert_eq!(m, Mat::<2, 3>::new(3.0, 4.0, 5.0, 6.0, 8.0, 10.0), "got\n{}", m);
    });
    test("outer(d, d) is symmetric with the squares on the diagonal", || {
        let d = V3::new(1.0, -2.0, 0.5);
        let m = outer(&d, &d);
        assert_eq!(m, m.transpose(), "d d^T must equal its transpose");
        assert_eq!((m[(0, 0)], m[(1, 1)], m[(2, 2)]), (1.0, 4.0, 0.25));
    });
    test("a 15x15 matrix is 1800 bytes and lives on the stack", || {
        assert_eq!(core::mem::size_of::<Mat<15, 15>>(), 15 * 15 * 8);
    });
}
