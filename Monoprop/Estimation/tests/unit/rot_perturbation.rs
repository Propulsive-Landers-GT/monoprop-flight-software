#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

pub fn run() {
    let qs = [quat_exp(&V3::new(0.3, -0.5, 1.1)), quat_exp(&V3::new(-2.0, 0.4, 0.9)), Quat::identity()];
    test("boxplus then boxminus gives the perturbation back", || {
        let d = V3::new(0.02, -0.03, 0.01);
        for q in qs.iter() {
            let back = boxminus(&boxplus(q, &d), q);
            assert!((back - d).amax() < 1e-12, "got {}", back.transpose());
        }
    });
    test("the worked example: a body-frame roll on a yawed vehicle", || {
        let q = quat_exp(&V3::new(0.0, 0.0, FRAC_PI_2));
        let tipped = boxplus(&q, &V3::new(0.1, 0.0, 0.0));
        let up = tipped * V3::z();
        assert!(up.y.abs() < 1e-12 && up.x > 0.09, "the body z-axis should tip towards inertial +x (rotation about inertial y): got {}", up.transpose());
    });
    test("boxplus multiplies on the right", || {
        let q = quat_exp(&V3::new(0.0, 0.0, 1.0));
        let d = V3::new(0.1, 0.0, 0.0);
        assert!(boxplus(&q, &d).angle_to(&(q * quat_exp(&d))) < 1e-12, "q (+) d should be q * Exp(d), not Exp(d) * q");
    });
    test("boxminus of identical attitudes is zero", || {
        for q in qs.iter() {
            assert!(boxminus(q, q).norm() < 1e-12);
        }
    });
    test("integrate a constant body rate for 1 s", || {
        let w = V3::new(0.2, -0.1, 0.5);
        let mut q = Quat::identity();
        for _ in 0..1000 {
            q = integrate_gyro(&q, &w, 0.001);
        }
        let err = boxminus(&q, &quat_exp(&w));
        assert!(err.norm() < 1e-9, "error {:e} rad", err.norm());
        assert_close(q.quaternion().norm(), 1.0, 1e-12, "unit length");
    });
}
