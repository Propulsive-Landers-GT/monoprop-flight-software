#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

fn sample() -> Nominal {
    Nominal {
        p: V3::new(1.0, 2.0, 3.0),
        v: V3::new(0.5, -0.3, 0.2),
        q: quat_exp(&V3::new(0.3, -0.2, 0.9)),
        ab: V3::new(0.1, -0.05, 0.08),
        wb: V3::new(0.01, 0.02, -0.01),
    }
}

pub fn run() {
    test("the worked example", || {
        let x = Nominal { p: V3::zeros(), v: V3::zeros(), q: Quat::identity(), ab: V3::zeros(), wb: V3::zeros() };
        let mut dx = Vector::<15>::zeros();
        dx[0] = 1.0;
        dx[8] = 0.1;
        let y = inject(&x, &dx);
        assert_eq!(y.p, V3::new(1.0, 0.0, 0.0), "position");
        assert!((y.q.scaled_axis() - V3::new(0.0, 0.0, 0.1)).amax() < 1e-12, "attitude should be yawed by 0.1");
        assert!((difference(&y, &x) - dx).amax() < 1e-12, "difference should give dx back");
    });
    test("each block goes to the right field", || {
        let x = sample();
        for (b, name) in ["p", "v", "theta", "ab", "wb"].iter().enumerate() {
            let mut dx = Vector::<15>::zeros();
            dx[3 * b + 1] = 0.01;
            let y = inject(&x, &dx);
            let changed = [y.p != x.p, y.v != x.v, y.q != x.q, y.ab != x.ab, y.wb != x.wb];
            for (i, c) in changed.iter().enumerate() {
                assert_eq!(*c, i == b, "perturbing the {} block (rows {}..{}) should change only that field", name, 3 * b, 3 * b + 3);
            }
        }
    });
    test("inject then difference is the identity", || {
        let x = sample();
        let mut rng = Rng::new(4);
        for _ in 0..20 {
            let dx = rng.normal_vec::<15>(0.05);
            let back = difference(&inject(&x, &dx), &x);
            assert!((back - dx).amax() < 1e-12, "got back\n{}", back.transpose());
        }
    });
    test("the attitude error is in body axes (q * Exp, not Exp * q)", || {
        let x = sample();
        let mut dx = Vector::<15>::zeros();
        dx[6] = 0.05;
        let y = inject(&x, &dx);
        assert!(y.q.angle_to(&(x.q * quat_exp(&V3::new(0.05, 0.0, 0.0)))) < 1e-12);
    });
    test("difference of a state with itself is zero", || {
        assert!(difference(&sample(), &sample()).amax() < 1e-15);
    });
}
