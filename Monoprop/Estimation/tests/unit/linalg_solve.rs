#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

fn spd4() -> Mat<4, 4> {
    let a = Mat::<4, 4>::new(2.0, 0.3, -0.1, 0.5, 0.1, 1.5, 0.4, 0.0, 0.0, -0.2, 1.0, 0.3, 0.6, 0.1, 0.0, 1.2);
    a * a.transpose() + Mat::<4, 4>::identity() * 0.1
}

pub fn run() {
    test("the worked example: [[4, 2], [2, 3]] x = (8, 7) gives x = (1.25, 1.5)", || {
        let l = Mat::<2, 2>::new(2.0, 0.0, 1.0, 2f64.sqrt());
        let x = cholesky_solve(&l, &Vector::<2>::new(8.0, 7.0));
        assert!((x - Vector::<2>::new(1.25, 1.5)).amax() < 1e-12, "got {}", x.transpose());
    });
    test("solve_spd with two right-hand sides at once", || {
        let a = spd4();
        let x_true = Mat::<4, 2>::new(1.0, -2.0, 0.5, 3.0, -1.0, 0.0, 2.0, 1.0);
        let x = solve_spd(&a, &(a * x_true)).expect("A is SPD, expected Some");
        assert!((x - x_true).amax() < 1e-10, "got\n{}", x);
    });
    test("solve_spd refuses a matrix that is not positive definite", || {
        assert!(solve_spd(&Mat::<2, 2>::new(1.0, 2.0, 2.0, 1.0), &Vector::<2>::new(1.0, 1.0)).is_none());
    });
    test("inverse_spd: A times its inverse is the identity", || {
        let a = spd4();
        let inv = inverse_spd(&a).expect("invertible");
        assert!((a * inv - Mat::<4, 4>::identity()).amax() < 1e-10);
        assert!(inverse_spd(&Mat::<2, 2>::new(1.0, 2.0, 2.0, 1.0)).is_none(), "should refuse an indefinite matrix");
    });
    test("15x15: agrees with nalgebra's solver", || {
        let mut p = Mat::<15, 15>::identity() * 2.0;
        for i in 0..14 {
            p[(i, i + 1)] = 0.4;
            p[(i + 1, i)] = 0.4;
        }
        let b = Vector::<15>::from_fn(|i, _| (i as f64).sin());
        let x = solve_spd(&p, &b).expect("SPD");
        assert!((x - p.cholesky().unwrap().solve(&b)).amax() < 1e-12);
    });
}
