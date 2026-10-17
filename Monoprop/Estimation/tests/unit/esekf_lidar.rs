// Math checks for src/esekf/lidar.rs.
#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

fn state(p: V3, q: Quat) -> Nominal {
    Nominal { p, v: V3::zeros(), q, ab: V3::zeros(), wb: V3::zeros() }
}

fn down() -> LidarConfig {
    LidarConfig { offset: V3::zeros(), beam: V3::new(0.0, 0.0, -1.0), ground_z: 0.0, min_range: 0.1, max_range: 3.0, max_tilt: 30.0 * PI / 180.0 }
}

pub fn run() {
    test("level, zero offset: the predicted range is the height above the ground", || {
        let x = state(V3::new(1.0, 2.0, 1.5), Quat::identity());
        assert_close(lidar_prediction(&x, &down()).unwrap_or(f64::NAN), 1.5, 1e-12, "range");
        let h = lidar_jacobian(&x, &down()).unwrap_or_else(Mat::zeros);
        assert!((h - height_jacobian()).amax() < 1e-12, "H equals the height model's");
    });
    test("tilted 20 degrees: the predicted range is height / cos(20 deg)", || {
        let tilt = 20.0 * PI / 180.0;
        let x = state(V3::new(0.0, 0.0, 2.0), quat_exp(&(V3::x() * tilt)));
        assert_close(lidar_prediction(&x, &down()).unwrap_or(f64::NAN), 2.0 / tilt.cos(), 1e-12, "slant range");
    });
    test("H matches finite differences of the prediction, at random states", || {
        let cfg = LidarConfig {
            offset: V3::new(0.05, -0.02, -0.1),
            beam: V3::new(0.1, -0.05, -1.0).normalize(),
            ground_z: 0.3,
            min_range: 0.0,
            max_range: 50.0,
            max_tilt: 70.0 * PI / 180.0,
        };
        let mut rng = Rng::new(11);
        let eps = 1e-6;
        for _ in 0..8 {
            let x = Nominal {
                p: rng.normal_vec::<3>(2.0) + V3::new(0.0, 0.0, 8.0),
                v: rng.normal_vec::<3>(1.0),
                q: quat_exp(&rng.normal_vec::<3>(0.2)),
                ab: rng.normal_vec::<3>(0.1),
                wb: rng.normal_vec::<3>(0.01),
            };
            let h = lidar_jacobian(&x, &cfg);
            assert!(h.is_some(), "the beam must be usable at this state");
            let Some(h) = h else { return };
            for j in 0..15 {
                let mut d = Vector::<15>::zeros();
                d[j] = eps;
                let plus = lidar_prediction(&inject(&x, &d), &cfg).unwrap_or(f64::NAN);
                let minus = lidar_prediction(&inject(&x, &(-d)), &cfg).unwrap_or(f64::NAN);
                assert_close(h[(0, j)], (plus - minus) / (2.0 * eps), 1e-6, "dh/d(error)");
            }
        }
    });
    test("lidar_ground_z inverts the prediction", || {
        let cfg = LidarConfig { offset: V3::new(0.03, 0.0, -0.1), ..down() };
        let x = state(V3::new(0.4, -0.2, 0.0), quat_exp(&V3::new(0.05, -0.08, 0.6)));
        let ground_z = lidar_ground_z(&x, 0.25, &cfg).unwrap_or(f64::NAN);
        assert_close(lidar_prediction(&x, &LidarConfig { ground_z, ..cfg }).unwrap_or(f64::NAN), 0.25, 1e-12, "range");
    });
}
