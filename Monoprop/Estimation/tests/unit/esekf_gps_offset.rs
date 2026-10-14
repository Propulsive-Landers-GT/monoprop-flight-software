// Tests for src/esekf/gps_offset.rs.
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

pub fn run() {
    test("zero offset and equal sigmas: update_gps_offset is update_gps", || {
        let x = state(V3::new(1.0, 2.0, 3.0), quat_exp(&V3::new(0.1, -0.2, 0.3)));
        let mut a = Esekf::new(x, p0(), spec());
        let mut b = Esekf::new(x, p0(), spec());
        let z = V3::new(1.4, 1.7, 3.2);
        assert!(a.update_gps_offset(&z, &V3::new(0.5, 0.5, 0.5), &V3::zeros()), "offset update accepted");
        assert!(b.update_gps(&z, 0.5), "plain update accepted");
        assert!(difference(&a.x, &b.x).amax() < 1e-12, "same state");
        assert!((a.p - b.p).amax() < 1e-12, "same covariance");
        assert!((gps_offset_jacobian(&x, &V3::zeros()) - gps_jacobian()).amax() == 0.0, "same H");
    });
    test("the prediction is p + R a", || {
        // the worked example: yawed +90 degrees, an antenna 0.5 m forward is 0.5 m north of the vehicle
        let x = state(V3::new(1.0, 2.0, 3.0), quat_exp(&V3::new(0.0, 0.0, FRAC_PI_2)));
        let forward = V3::new(0.5, 0.0, 0.0);
        assert!((gps_offset_prediction(&x, &forward) - V3::new(1.0, 2.5, 3.0)).amax() < 1e-12, "yawed 90 degrees");
        let x = state(V3::new(-4.0, 0.5, 2.0), quat_exp(&V3::new(0.3, -0.2, 0.9)));
        let a = V3::new(0.3, -0.2, 0.15);
        assert!((gps_offset_prediction(&x, &a) - (x.p + quat_to_rot(&x.q) * a)).amax() < 1e-12, "tilted and yawed");
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
            assert!(h.fixed_view::<3, 3>(0, 6).amax() > 0.1, "the attitude columns are exercised");
        }
    });
    test("each axis uses its own sigma", || {
        // level, no offset, equal prior variances of 1: each axis moves by P / (P + sigma^2) of its innovation
        let x = state(V3::zeros(), Quat::identity());
        let mut f = Esekf::new(x, p0(), spec());
        assert!(f.update_gps_offset(&V3::new(1.0, 1.0, 1.0), &V3::new(0.1, 0.5, 2.0), &V3::zeros()));
        assert_close(f.x.p.x, 1.0 / 1.01, 1e-9, "east, sigma 0.1");
        assert_close(f.x.p.y, 1.0 / 1.25, 1e-9, "north, sigma 0.5");
        assert_close(f.x.p.z, 1.0 / 5.0, 1e-9, "up, sigma 2.0");
        assert!(is_spd(&f.p), "P must stay symmetric positive definite");
    });
    test("the offset matters: a fix at the antenna does not move a correct estimate", || {
        let x = state(V3::new(1.0, 2.0, 3.0), quat_exp(&V3::new(0.0, 0.0, FRAC_PI_2)));
        let a = V3::new(0.5, 0.0, 0.0);
        let fix = V3::new(1.0, 2.5, 3.0); // where the antenna really is
        let mut with = Esekf::new(x, p0(), spec());
        assert!(with.update_gps_offset(&fix, &V3::new(0.1, 0.1, 0.1), &a));
        assert!(difference(&with.x, &x).amax() < 1e-12, "nothing to correct");
        let mut without = Esekf::new(x, p0(), spec());
        assert!(without.update_gps(&fix, 0.1));
        assert!(without.x.p.y > 2.4, "ignoring the offset drags the position north, to {}", without.x.p.y);
    });
    test("bad fixes, sigmas and offsets are rejected and change nothing", || {
        let x = state(V3::zeros(), quat_exp(&V3::new(0.1, 0.0, 0.2)));
        let mut f = Esekf::new(x, p0(), spec());
        let (x0, p0) = (f.x, f.p);
        let (sigma, a) = (V3::new(0.1, 0.1, 0.3), V3::new(0.2, 0.0, 0.1));
        // readings: counted, as for update_gps
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(!f.update_gps_offset(&V3::new(bad, 0.0, 0.0), &sigma, &a), "fix {}", bad);
        }
        assert!(!f.update_gps_offset(&V3::new(100.0, 0.0, 0.0), &sigma, &a), "a 100 m jump must be gated");
        assert_eq!(f.rejected, 4);
        // not readings: not counted
        for bad in [0.0, -0.1, f64::NAN, f64::INFINITY] {
            assert!(!f.update_gps_offset(&V3::zeros(), &V3::new(bad, 0.1, 0.3), &a), "east sigma {}", bad);
            assert!(!f.update_gps_offset(&V3::zeros(), &V3::new(0.1, bad, 0.3), &a), "north sigma {}", bad);
            assert!(!f.update_gps_offset(&V3::zeros(), &V3::new(0.1, 0.1, bad), &a), "up sigma {}", bad);
        }
        for bad in [f64::NAN, f64::INFINITY] {
            assert!(!f.update_gps_offset(&V3::zeros(), &sigma, &V3::new(0.2, bad, 0.1)), "offset {}", bad);
        }
        assert_eq!(f.rejected, 4, "bad sigmas and offsets are not outliers");
        assert!(f.x == x0 && f.p == p0, "state and covariance unchanged");
    });
    test("a monoprop GnssSample feeds update_gps_offset directly", || {
        let offset = mp::gnss_mount_offset();
        assert!(offset.iter().all(|v| v.is_finite()), "antenna offset is finite");
        let sample = mp::GnssSample::new(1, V3::new(0.3, -0.2, 0.1), V3::new(0.5, 0.5, 1.5));
        assert!(sample.is_some(), "valid sample");
        let Some(s) = sample else { return };
        let mut f = Esekf::new(state(V3::zeros(), Quat::identity()), p0(), spec());
        assert!(f.update_gps_offset(&s.position(), &s.sigma(), &offset), "the fix is used");
        assert!(f.x.p.x > 0.0 && f.x.p.y < 0.0 && f.x.p.z > 0.0, "moved towards the fix");
        assert!(is_spd(&f.p), "P must stay symmetric positive definite");
    });
}
