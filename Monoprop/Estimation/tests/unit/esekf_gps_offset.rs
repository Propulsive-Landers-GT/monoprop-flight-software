// Math checks for src/esekf/gps_offset.rs.
#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

fn state(p: V3, q: Quat) -> Nominal {
    Nominal { p, v: V3::zeros(), q, ab: V3::zeros(), wb: V3::zeros() }
}

pub fn run() {
    test("zero offset is the plain GPS model", || {
        let x = state(V3::new(1.0, 2.0, 3.0), quat_exp(&V3::new(0.1, -0.2, 0.3)));
        assert!((gps_offset_prediction(&x, &V3::zeros()) - x.p).amax() < 1e-12);
        assert!((gps_offset_jacobian(&x, &V3::zeros()) - gps_jacobian()).amax() == 0.0);
    });
    test("the prediction is p + R a", || {
        // yawed +90 degrees, an antenna 0.5 m forward is 0.5 m north of the vehicle
        let x = state(V3::new(1.0, 2.0, 3.0), quat_exp(&V3::new(0.0, 0.0, FRAC_PI_2)));
        assert!((gps_offset_prediction(&x, &V3::new(0.5, 0.0, 0.0)) - V3::new(1.0, 2.5, 3.0)).amax() < 1e-12);
    });
    test("H matches finite differences of the prediction, at random states", || {
        let a = V3::new(0.3, -0.2, 0.15);
        let mut rng = Rng::new(21);
        let eps = 1e-6;
        for _ in 0..8 {
            let x = Nominal {
                p: rng.normal_vec::<3>(5.0),
                v: rng.normal_vec::<3>(1.0),
                q: quat_exp(&rng.normal_vec::<3>(0.6)),
                ab: rng.normal_vec::<3>(0.1),
                wb: rng.normal_vec::<3>(0.01),
            };
            let h = gps_offset_jacobian(&x, &a);
            for j in 0..15 {
                let mut d = Vector::<15>::zeros();
                d[j] = eps;
                let num = (gps_offset_prediction(&inject(&x, &d), &a) - gps_offset_prediction(&inject(&x, &(-d)), &a)) / (2.0 * eps);
                for i in 0..3 {
                    assert_close(h[(i, j)], num[i], 1e-6, "dh/d(error)");
                }
            }
        }
    });
}
