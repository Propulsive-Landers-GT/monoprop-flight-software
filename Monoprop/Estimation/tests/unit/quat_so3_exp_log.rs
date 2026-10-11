#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;
use Estimation::na::Rotation3;

pub fn run() {
    let rs = [V3::new(0.3, -0.5, 1.1), V3::new(-2.0, 0.4, 0.9), V3::new(1e-7, -2e-7, 5e-8), V3::new(1e-12, 0.0, 0.0)];
    test("the worked example: 90 degrees about z", || {
        let r = exp_so3(&V3::new(0.0, 0.0, FRAC_PI_2));
        assert!((r - M3::new(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)).amax() < 1e-12, "got\n{}", r);
        let back = log_so3(&r);
        assert!((back - V3::new(0.0, 0.0, FRAC_PI_2)).amax() < 1e-12, "log gave {}", back.transpose());
    });
    test("exp_so3 matches nalgebra and agrees with the quaternion route", || {
        for r in rs.iter() {
            assert!((exp_so3(r) - Rotation3::from_scaled_axis(*r).into_inner()).amax() < 1e-12, "r = {}", r.transpose());
            assert!((exp_so3(r) - quat_to_rot(&quat_exp(r))).amax() < 1e-12, "r = {}", r.transpose());
        }
    });
    test("log_so3 inverts exp_so3, including tiny angles", || {
        for r in rs.iter() {
            let back = log_so3(&exp_so3(r));
            assert!((back - r).amax() < 1e-9, "r = {}: got {}", r.transpose(), back.transpose());
        }
    });
    test("log of the identity is zero, not NaN", || {
        let v = log_so3(&M3::identity());
        assert!(v.iter().all(|x| x.is_finite()) && v.norm() == 0.0, "got {}", v.transpose());
    });
}
