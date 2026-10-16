// Tests for src/esekf/lidar.rs (and the lidar values of src/monoprop/params.rs).
#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::monoprop as mp;
use Estimation::*;

fn spec() -> ImuSpec {
    ImuSpec { sigma_a: 0.03, sigma_g: 0.003, sigma_aw: 1e-4, sigma_gw: 1e-5 }
}

fn p0() -> Mat<15, 15> {
    diag([1.0, 1.0, 1.0, 0.25, 0.25, 0.25, 0.01, 0.01, 0.01, 0.04, 0.04, 0.04, 1e-3, 1e-3, 1e-3])
}

fn state(p: V3, q: Quat) -> Nominal {
    Nominal { p, v: V3::zeros(), q, ab: V3::zeros(), wb: V3::zeros() }
}

/// Straight down, no offset, ground at z = 0, usable from 0.1 m to 3 m and up to 30 degrees of tilt.
fn down() -> LidarConfig {
    LidarConfig {
        offset: V3::zeros(),
        beam: V3::new(0.0, 0.0, -1.0),
        ground_z: 0.0,
        min_range: 0.1,
        max_range: 3.0,
        max_tilt: 30.0 * PI / 180.0,
    }
}

pub fn run() {
    test("level, zero offset: the predicted range is the height above the ground", || {
        let x = state(V3::new(1.0, 2.0, 1.5), Quat::identity());
        assert_eq!(lidar_prediction(&x, &down()), Some(1.5));
        let raised = LidarConfig { ground_z: 0.4, ..down() };
        let r = lidar_prediction(&x, &raised);
        assert!(r.is_some(), "prediction");
        assert_close(r.unwrap_or(f64::NAN), 1.1, 1e-12, "ground at 0.4 m");
    });
    test("level, zero offset: update_lidar is update_height", || {
        let x = state(V3::new(1.0, 2.0, 1.5), Quat::identity());
        let mut a = Esekf::new(x, p0(), spec());
        let mut b = Esekf::new(x, p0(), spec());
        assert!(a.update_lidar(1.6, 0.05, &down()), "lidar accepted");
        assert!(b.update_height(1.6, 0.05), "height accepted");
        assert!(difference(&a.x, &b.x).amax() < 1e-12, "same state");
        assert!((a.p - b.p).amax() < 1e-12, "same covariance");
        assert!(a.x.p.z > 1.5 && a.x.p.z < 1.6, "z moved towards the reading");
    });
    test("tilted 20 degrees: the predicted range is height / cos(20 deg)", || {
        let tilt = 20.0 * PI / 180.0;
        for axis in [V3::x(), V3::y()] {
            let x = state(V3::new(0.0, 0.0, 2.0), quat_exp(&(axis * tilt)));
            let r = lidar_prediction(&x, &down());
            assert!(r.is_some(), "prediction");
            assert_close(r.unwrap_or(f64::NAN), 2.0 / tilt.cos(), 1e-12, "slant range");
        }
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
            assert!(h[(0, 6)].abs() + h[(0, 7)].abs() > 1e-3, "the attitude columns are exercised");
        }
    });
    test("bad readings are rejected and change nothing", || {
        let x = state(V3::new(0.0, 0.0, 1.5), Quat::identity());
        let mut f = Esekf::new(x, p0(), spec());
        let (x0, p0) = (f.x, f.p);
        // not a positive finite number: counted, like a bad magnetometer reading
        for range in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 0.0, -1.0] {
            assert!(!f.update_lidar(range, 0.05, &down()), "range {}", range);
        }
        assert_eq!(f.rejected, 5);
        // readings the sensor can't give, or a bad sigma: not counted
        for sigma in [0.0, -0.05, f64::NAN, f64::INFINITY] {
            assert!(!f.update_lidar(1.5, sigma, &down()), "sigma {}", sigma);
        }
        assert!(!f.update_lidar(0.05, 0.05, &down()), "below min_range");
        assert!(!f.update_lidar(3.5, 0.05, &down()), "beyond max_range");
        assert_eq!(f.rejected, 5, "limits and sigma are not outliers");
        assert!(f.x == x0 && f.p == p0, "state and covariance unchanged");
        // the limits themselves are still usable
        assert!(f.update_lidar(1.5, 0.05, &down()), "a good reading is accepted");
    });
    test("a beam tilted beyond max_tilt is rejected and changes nothing", || {
        let x = state(V3::new(0.0, 0.0, 1.5), quat_exp(&V3::new(40.0 * PI / 180.0, 0.0, 0.0)));
        let mut f = Esekf::new(x, p0(), spec());
        let (x0, p0) = (f.x, f.p);
        assert!(lidar_prediction(&x, &down()).is_none() && lidar_jacobian(&x, &down()).is_none());
        assert!(!f.update_lidar(1.9, 0.05, &down()), "40 degrees with a 30 degree limit");
        assert!(f.x == x0 && f.p == p0 && f.rejected == 0);
        // upside down, and a limit of 90 degrees or more, must not divide by (almost) zero either
        let flipped = state(V3::new(0.0, 0.0, 1.5), quat_exp(&V3::new(PI, 0.0, 0.0)));
        assert!(lidar_prediction(&flipped, &down()).is_none(), "pointing up");
        let wide = LidarConfig { max_tilt: PI / 2.0, ..down() };
        assert!(lidar_prediction(&state(V3::new(0.0, 0.0, 1.5), Quat::identity()), &wide).is_none(), "max_tilt = 90 deg");
        let no_beam = LidarConfig { beam: V3::zeros(), ..down() };
        assert!(lidar_prediction(&state(V3::new(0.0, 0.0, 1.5), Quat::identity()), &no_beam).is_none(), "zero beam");
    });
    test("hover with a height error: lidar updates pull the height to the truth", || {
        let truth_z = 2.0;
        let x = state(V3::new(0.0, 0.0, 2.5), Quat::identity());
        let mut f = Esekf::new(x, p0(), spec());
        let mut rng = Rng::new(12);
        let (dt, sigma) = (1.0 / 400.0, 0.05);
        let am = specific_force(&Quat::identity(), &V3::zeros());
        let mut used = 0;
        for k in 0..800 {
            assert!(f.predict(&am, &V3::zeros(), dt));
            if k % 8 == 7 && f.update_lidar(truth_z + rng.normal(sigma), sigma, &down()) {
                used += 1;
            }
        }
        assert!(used >= 95, "{} of 100 readings used", used);
        assert_close(f.x.p.z, truth_z, 0.05, "height after 2 s");
        assert!(f.x.v.z.abs() < 0.1, "vertical velocity {}", f.x.v.z);
        assert!(is_spd(&f.p), "P must stay symmetric positive definite");
    });
    test("lidar_ground_z: the pad reads as the ground", || {
        let cfg = LidarConfig { offset: V3::new(0.03, 0.0, -0.1), ..down() };
        for q in [Quat::identity(), quat_exp(&V3::new(0.05, -0.08, 0.6))] {
            let x = state(V3::new(0.4, -0.2, 0.0), q);
            let ground = lidar_ground_z(&x, 0.25, &cfg);
            assert!(ground.is_some(), "a usable pad reading");
            let Some(ground_z) = ground else { return };
            let on_pad = LidarConfig { ground_z, ..cfg };
            assert_close(lidar_prediction(&x, &on_pad).unwrap_or(f64::NAN), 0.25, 1e-12, "predicted range on the pad");
        }
        let level = state(V3::zeros(), Quat::identity());
        assert_close(lidar_ground_z(&level, 0.25, &cfg).unwrap_or(f64::NAN), -0.35, 1e-12, "0.1 m mount + 0.25 m range below the IMU");
    });
    test("lidar_ground_z refuses readings it can't use", || {
        let level = state(V3::zeros(), Quat::identity());
        for range in [f64::NAN, f64::INFINITY, 0.0, -0.2] {
            assert!(lidar_ground_z(&level, range, &down()).is_none(), "range {}", range);
        }
        assert!(lidar_ground_z(&level, 0.05, &down()).is_none(), "below min_range");
        assert!(lidar_ground_z(&level, 3.5, &down()).is_none(), "beyond max_range");
        let tilted = state(V3::zeros(), quat_exp(&V3::new(0.0, 40.0 * PI / 180.0, 0.0)));
        assert!(lidar_ground_z(&tilted, 0.25, &down()).is_none(), "tilted beyond max_tilt");
    });
    test("monoprop lidar parameters make a usable configuration", || {
        let cfg = mp::lidar_config(0.2);
        assert_close(cfg.beam.norm(), 1.0, 1e-12, "beam is a unit vector");
        assert_eq!(cfg.beam, mp::lidar_beam_direction());
        assert_eq!(cfg.offset, mp::lidar_mount_offset());
        assert_eq!(cfg.ground_z, 0.2);
        assert!(cfg.min_range.is_finite() && cfg.min_range >= 0.0 && cfg.min_range < cfg.max_range, "0 <= min < max");
        assert!(cfg.max_range.is_finite(), "max_range");
        assert!(cfg.max_tilt > 0.0 && cfg.max_tilt < PI / 2.0, "max_tilt in (0, 90 deg)");
        assert!(mp::SIGMA_LIDAR_RANGE.is_finite() && mp::SIGMA_LIDAR_RANGE > 0.0, "range sigma");
        let x = state(V3::new(0.0, 0.0, 1.2), Quat::identity());
        assert_close(lidar_prediction(&x, &cfg).unwrap_or(f64::NAN), 1.0, 1e-12, "1.2 m up, ground at 0.2 m");
        let mut f = Esekf::new(x, p0(), spec());
        assert!(f.update_lidar(1.05, mp::SIGMA_LIDAR_RANGE, &cfg), "a good reading is used");
    });
}
