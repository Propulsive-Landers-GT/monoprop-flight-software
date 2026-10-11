#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

pub fn run() {
    let m = mag_reference();
    test("the worked example: level and pointing north is the identity", || {
        let q = triad(&V3::new(0.0, 0.0, 9.81), &m, &m).expect("valid readings");
        assert!(q.angle() < 1e-9, "got a rotation of {:.2e} rad", q.angle());
    });
    test("yawed 90 degrees: TRIAD returns a 90 degree yaw", || {
        let truth = quat_exp(&V3::new(0.0, 0.0, FRAC_PI_2));
        let q = triad(&V3::new(0.0, 0.0, 9.81), &(truth.inverse() * m), &m).expect("valid readings");
        assert!(boxminus(&q, &truth).norm() < 1e-9, "error {:.2e} rad", boxminus(&q, &truth).norm());
    });
    test("recovers random attitudes from perfect vectors", || {
        let mut rng = Rng::new(1);
        for _ in 0..20 {
            let q = quat_exp(&V3::new(rng.gauss(), rng.gauss(), 0.7 * rng.gauss()));
            let f = specific_force(&q, &V3::zeros());
            let mb = q.inverse() * m;
            let e = boxminus(&triad(&f, &mb, &m).expect("valid readings"), &q).norm();
            assert!(e < 1e-9, "attitude error {:e} rad", e);
        }
    });
    test("trusts the accelerometer: 'up' is exact even with a bad magnetometer", || {
        let q = quat_exp(&V3::new(0.3, -0.4, 2.0));
        let f = specific_force(&q, &V3::zeros());
        let mb = q.inverse() * m + V3::new(0.1, -0.05, 0.08);
        let up_est = triad(&f, &mb, &m).expect("valid readings").inverse() * V3::z();
        assert!((up_est - f.normalize()).amax() < 1e-12, "the estimated 'up' in body axes must equal the accelerometer direction");
    });
    test("readings that can't define an attitude give None (not NaN)", || {
        assert!(triad(&V3::zeros(), &m, &m).is_none(), "zero accelerometer reading");
        assert!(triad(&V3::new(0.0, 0.0, 9.81), &V3::zeros(), &m).is_none(), "zero magnetometer reading");
        assert!(triad(&V3::new(0.0, 0.0, 9.81), &V3::new(0.0, 0.0, 30.0), &m).is_none(), "magnetometer parallel to gravity");
        assert!(triad(&V3::new(0.0, 0.0, 9.81), &V3::new(f64::NAN, 0.0, 1.0), &m).is_none(), "NaN magnetometer");
    });
}
