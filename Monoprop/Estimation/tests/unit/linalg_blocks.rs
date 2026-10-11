#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

pub fn run() {
    test("writing 2I at (3, 6) of a 15x15 identity (the worked example)", || {
        let mut f = Mat::<15, 15>::identity();
        set_block3(&mut f, 3, 6, &(M3::identity() * 2.0));
        assert_eq!((f[(3, 6)], f[(4, 7)], f[(5, 8)]), (2.0, 2.0, 2.0), "the block's diagonal");
        assert_eq!((f[(3, 3)], f[(4, 4)], f[(5, 5)]), (1.0, 1.0, 1.0), "the identity's own entries are untouched");
        assert_eq!(f.sum(), 15.0 + 6.0, "nothing else changed");
    });
    test("every entry of the block lands in the right place", || {
        let b = M3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let mut m = Mat::<4, 5>::zeros();
        set_block3(&mut m, 1, 2, &b);
        for i in 0..3 {
            for j in 0..3 {
                assert_eq!(m[(1 + i, 2 + j)], b[(i, j)], "b[({}, {})] should be at m[({}, {})]", i, j, 1 + i, 2 + j);
            }
        }
        assert_eq!(m.sum(), 45.0, "nothing written outside the block");
    });
    test("sub3 reads three consecutive entries", || {
        let x = Vector::<15>::from_fn(|i, _| i as f64);
        assert_eq!(sub3(&x, 0), V3::new(0.0, 1.0, 2.0), "position part");
        assert_eq!(sub3(&x, 6), V3::new(6.0, 7.0, 8.0), "attitude part");
        assert_eq!(sub3(&x, 12), V3::new(12.0, 13.0, 14.0), "gyro bias part");
    });
}
