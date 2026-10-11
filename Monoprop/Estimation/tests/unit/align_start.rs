#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

fn spec() -> ImuSpec {
    ImuSpec { sigma_a: 0.03, sigma_g: 0.003, sigma_aw: 1e-4, sigma_gw: 1e-5 }
}

pub fn run() {
    test("P0 is diagonal, positive, covers the GPS noise and an air start", || {
        let p = initial_covariance(0.5);
        assert!(is_spd(&p), "P0 must be symmetric positive definite");
        for i in 0..15 {
            for j in 0..15 {
                if i != j {
                    assert_eq!(p[(i, j)], 0.0, "off-diagonal ({}, {})", i, j);
                }
            }
        }
        for i in 0..3 {
            assert!(p[(i, i)] >= 0.25 - 1e-12, "position variance must cover the GPS noise (0.5^2)");
            assert!(p[(3 + i, 3 + i)] >= 1.0, "air start: velocity is unknown by metres per second");
            assert!(p[(6 + i, 6 + i)] > 1e-4 && p[(6 + i, 6 + i)] < 0.05, "attitude variance should cover a few degrees, not tens");
        }
    });
    test("P0 position block follows the GPS sigma", || {
        assert_close(initial_covariance(2.0)[(1, 1)], 4.0, 1e-12, "sigma_gps = 2");
    });
    test("align: position from GPS, attitude from TRIAD, velocity and biases zero", || {
        let q = quat_exp(&V3::new(0.1, -0.2, 1.3));
        let f = align(&specific_force(&q, &V3::zeros()), &(q.inverse() * mag_reference()), &V3::new(1.0, 2.0, 3.0), spec(), 0.5).expect("valid readings");
        assert_eq!(f.x.p, V3::new(1.0, 2.0, 3.0), "position should come from the GPS fix");
        assert!(f.x.v == V3::zeros() && f.x.ab == V3::zeros() && f.x.wb == V3::zeros(), "velocity and biases start at zero");
        assert!(boxminus(&f.x.q, &q).norm() < 1e-9, "attitude should come from TRIAD");
        assert!(f.p == initial_covariance(0.5) && f.rejected == 0, "P0 and a fresh rejection counter");
    });
    test("align refuses readings it can't use (None), so the caller tries the next ones", || {
        let up = V3::new(0.0, 0.0, 9.81);
        assert!(align(&up, &V3::new(0.0, 0.0, 30.0), &V3::zeros(), spec(), 0.5).is_none(), "magnetometer parallel to gravity");
        assert!(align(&V3::zeros(), &mag_reference(), &V3::zeros(), spec(), 0.5).is_none(), "zero accelerometer");
        assert!(align(&up, &mag_reference(), &V3::new(f64::NAN, 0.0, 0.0), spec(), 0.5).is_none(), "NaN position fix");
    });
}
