// Tests for src/esekf/height.rs.
#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

fn filter() -> Esekf {
    let q = quat_exp(&V3::new(0.1, -0.2, 0.3));
    let x = Nominal { p: V3::new(1.0, 2.0, 3.0), v: V3::new(0.2, 0.0, -0.1), q, ab: V3::zeros(), wb: V3::zeros() };
    Esekf::new(x, diag([1.0, 1.0, 1.0, 0.25, 0.25, 0.25, 0.01, 0.01, 0.01, 0.04, 0.04, 0.04, 1e-3, 1e-3, 1e-3]), ImuSpec::vn200(200.0))
}

pub fn run() {
    test("H matches finite differences of the height prediction", || {
        let f = filter();
        let h = height_jacobian();
        let eps = 1e-6;
        for j in 0..15 {
            let mut d = Vector::<15>::zeros();
            d[j] = eps;
            let num = (height_prediction(&inject(&f.x, &d)) - height_prediction(&inject(&f.x, &(-d)))) / (2.0 * eps);
            assert_close(h[(0, j)], num, 1e-6, "dh/d(error)");
        }
    });
    test("a precise height pins z and leaves x, y alone", || {
        let mut f = filter();
        let (x0, y0) = (f.x.p.x, f.x.p.y);
        assert!(f.update_height(3.5, 0.01));
        assert_close(f.x.p.z, 3.5, 1e-3, "z after the update");
        assert_close(f.x.p.x, x0, 1e-9, "x");
        assert_close(f.x.p.y, y0, 1e-9, "y");
        assert!(is_spd(&f.p), "P must stay symmetric positive definite");
    });
    test("a 20 m height glitch and a NaN are rejected and change nothing", || {
        let mut f = filter();
        let (x0, p0) = (f.x, f.p);
        assert!(!f.update_height(23.0, 0.1), "glitch");
        assert!(!f.update_height(f64::NAN, 0.1), "NaN");
        assert_eq!(f.rejected, 2);
        assert!(f.x == x0 && f.p == p0);
    });
    test("ImuSpec::vn200: per-sample sigma = density * sqrt(rate)", || {
        let s = ImuSpec::vn200(100.0);
        assert_close(s.sigma_a, vn200::ACCEL_NOISE_DENSITY * 10.0, 1e-15, "sigma_a at 100 Hz");
        assert_close(s.sigma_g, vn200::GYRO_NOISE_DENSITY * 10.0, 1e-15, "sigma_g at 100 Hz");
        assert_close(mag_reference().norm(), 1.0, 1e-12, "mag_reference is a unit vector");
    });
}
