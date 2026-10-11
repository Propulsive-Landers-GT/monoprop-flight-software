#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

fn vclose(a: &V3, b: &V3, tol: f64, what: &str) {
    assert!((a - b).amax() < tol, "{}: got {}, expected {}", what, a.transpose(), b.transpose());
}

pub fn run() {
    let rs = [V3::new(0.3, -0.5, 1.1), V3::new(-2.0, 0.4, 0.9), V3::new(1e-7, -2e-7, 5e-8), V3::new(0.0, 0.0, 3.0), V3::new(1e-14, 0.0, 0.0)];
    test("the worked example: 90 degrees about z", || {
        let q = quat_exp(&V3::new(0.0, 0.0, FRAC_PI_2));
        assert!((q.w - (PI / 4.0).cos()).abs() < 1e-12 && (q.k - (PI / 4.0).sin()).abs() < 1e-12, "got {}", q.quaternion());
        vclose(&(q * V3::x()), &V3::y(), 1e-12, "rotating the x-axis gives the y-axis");
    });
    test("quat_exp matches nalgebra, including tiny angles", || {
        for r in rs.iter() {
            let (a, b) = (quat_exp(r), Quat::from_scaled_axis(*r));
            assert!((a.quaternion() - b.quaternion()).norm() < 1e-12, "for r = {}: got {}, nalgebra {}", r.transpose(), a.quaternion(), b.quaternion());
        }
    });
    test("quat_log(quat_exp(r)) = r, including tiny angles", || {
        for r in rs.iter() {
            vclose(&quat_log(&quat_exp(r)), r, 1e-12, "log of exp");
            vclose(&quat_log(&Quat::from_scaled_axis(*r)), r, 1e-12, "log of nalgebra's quaternion");
        }
    });
    test("exactly zero: Exp(0) is the identity and Log(identity) is zero (no 0/0)", || {
        let q = quat_exp(&V3::zeros());
        assert!(q.quaternion().coords.iter().all(|v| v.is_finite()), "quat_exp(0) gave {}: a division by phi = 0?", q.quaternion());
        assert!((q.quaternion() - Quat::identity().quaternion()).norm() < 1e-15, "quat_exp(0) = {}, expected the identity", q.quaternion());
        let r = quat_log(&Quat::identity());
        assert!(r.iter().all(|v| v.is_finite()), "quat_log(identity) gave {}: a division by |v| = 0?", r.transpose());
        assert!(r.norm() < 1e-15, "quat_log(identity) = {}, expected zero", r.transpose());
    });
    test("quat_log takes the short way round (q and -q)", || {
        let q = Quat::from_scaled_axis(V3::new(0.0, 0.5, 0.0));
        let neg = Quat::new_unchecked(-q.into_inner());
        vclose(&quat_log(&neg), &V3::new(0.0, 0.5, 0.0), 1e-12, "log of -q");
    });
    test("tiny angles keep their precision (this is where acos fails)", || {
        let r = V3::new(3e-9, -1e-9, 2e-9);
        let back = quat_log(&quat_exp(&r));
        assert!((back - r).norm() < 1e-6 * r.norm(), "relative error {:e}", (back - r).norm() / r.norm());
    });
}
