#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

fn sample_state() -> Nominal {
    Nominal {
        p: V3::new(1.0, 2.0, 3.0),
        v: V3::new(0.5, -0.3, 0.2),
        q: quat_exp(&V3::new(0.3, -0.2, 0.9)),
        ab: V3::new(0.1, -0.05, 0.08),
        wb: V3::new(0.01, 0.02, -0.01),
    }
}

pub fn run() {
    test("GPS Jacobian: identity in the position columns, zero elsewhere", || {
        let h = gps_jacobian();
        for r in 0..3 {
            for c in 0..15 {
                assert_eq!(h[(r, c)], if r == c { 1.0 } else { 0.0 }, "H[({}, {})]", r, c);
            }
        }
    });
    test("GPS Jacobian matches h(x (+) dx) - h(x)", || {
        let x = sample_state();
        let mut rng = Rng::new(1);
        let dx = rng.normal_vec::<15>(1e-5);
        let actual = inject(&x, &dx).p - x.p;
        assert!((actual - gps_jacobian() * dx).norm() < 1e-6 * dx.norm());
    });
    test("the worked example: level, magnetometer prediction is the reference field", || {
        let x = Nominal { p: V3::zeros(), v: V3::zeros(), q: Quat::identity(), ab: V3::zeros(), wb: V3::zeros() };
        let m = mag_reference();
        assert!((mag_prediction(&x, &m) - m).amax() < 1e-15);
        let h = mag_jacobian(&x, &m);
        assert_close(h[(1, 8)], -m.x, 1e-15, "H[(1, 8)]: a yaw error swings the field sideways");
    });
    test("mag_prediction is R^T m", || {
        let x = sample_state();
        let m = mag_reference();
        assert!((mag_prediction(&x, &m) - quat_to_rot(&x.q).transpose() * m).amax() < 1e-12);
    });
    test("magnetometer Jacobian matches h(x (+) dx) - h(x)", || {
        let x = sample_state();
        let m = mag_reference();
        let h = mag_jacobian(&x, &m);
        let mut rng = Rng::new(2);
        for _ in 0..5 {
            let dx = rng.normal_vec::<15>(1e-5);
            let actual = mag_prediction(&inject(&x, &dx), &m) - mag_prediction(&x, &m);
            let e = (actual - h * dx).norm();
            assert!(e < 1e-4 * dx.norm(), "error {:.2e} for a nudge of {:.2e}: check the sign of the attitude block", e, dx.norm());
        }
    });
}
