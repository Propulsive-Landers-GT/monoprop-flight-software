// Tests for src/esekf/stationary.rs.
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

fn still(p: V3, q: Quat) -> Nominal {
    Nominal { p, v: V3::zeros(), q, ab: V3::zeros(), wb: V3::zeros() }
}

/// The Jacobian matches central finite differences of the prediction h under `inject`.
fn check_jacobian(x: &Nominal, h: &Mat<3, 15>, predict: fn(&Nominal) -> V3) {
    let eps = 1e-6;
    for j in 0..15 {
        let mut d = Vector::<15>::zeros();
        d[j] = eps;
        let num = (predict(&inject(x, &d)) - predict(&inject(x, &(-d)))) / (2.0 * eps);
        for i in 0..3 {
            assert_close(h[(i, j)], num[i], 1e-6, "dh/d(error)");
        }
    }
}

/// 10 s on the pad at 400 Hz with ideal still readings, from a 0.05 rad roll error; returns
/// (final speed, final roll error) with or without a zero-velocity update every step.
fn pad_run(zupt: bool) -> (f64, f64) {
    let start = Nominal { q: quat_exp(&V3::new(0.05, 0.0, 0.0)), ..still(V3::zeros(), Quat::identity()) };
    let mut f = Esekf::new(start, p0(), spec());
    let am = specific_force(&Quat::identity(), &V3::zeros());
    for _ in 0..4000 {
        assert!(f.predict(&am, &V3::zeros(), 1.0 / 400.0));
        if zupt {
            f.update_zero_velocity(0.02);
        }
    }
    assert!(is_spd(&f.p), "P must stay symmetric positive definite");
    (f.x.v.norm(), boxminus(&f.x.q, &Quat::identity()).x.abs())
}

pub fn run() {
    test("zero velocity: each velocity component shrinks by sigma^2 / (P + sigma^2)", || {
        let x = Nominal { v: V3::new(0.3, -0.2, 0.1), ..still(V3::new(1.0, 2.0, 3.0), quat_exp(&V3::new(0.1, 0.2, 0.3))) };
        let mut f = Esekf::new(x, p0(), spec());
        assert!(f.update_zero_velocity(0.1));
        let k = 0.01 / (0.25 + 0.01);
        assert!((f.x.v - x.v * k).amax() < 1e-12, "velocity");
        assert!((f.x.p - x.p).amax() < 1e-12 && boxminus(&f.x.q, &x.q).amax() < 1e-12, "P diagonal: nothing else moves");
        check_jacobian(&x, &zero_velocity_jacobian(), |x| x.v);
    });
    test("zero velocity keeps a still vehicle still and corrects its tilt", || {
        let (speed_with, roll_with) = pad_run(true);
        let (speed_without, roll_without) = pad_run(false);
        assert!(speed_with < 0.01, "speed with ZUPT {}", speed_with);
        assert!(roll_with < 0.01, "roll error with ZUPT {} (started at 0.05)", roll_with);
        assert!(speed_without > 1.0, "without ZUPT the tilt error makes velocity grow: {}", speed_without);
        assert!(roll_without > 0.04, "without ZUPT the roll error stays: {}", roll_without);
    });
    test("zero velocity refuses a vehicle that is moving", || {
        let x = Nominal { v: V3::new(2.0, 0.0, 0.0), ..still(V3::zeros(), Quat::identity()) };
        let mut p = p0();
        for i in 3..6 {
            p[(i, i)] = 0.01; // confident it is moving at 2 m/s
        }
        let mut f = Esekf::new(x, p, spec());
        let (x0, p_before) = (f.x, f.p);
        assert!(!f.update_zero_velocity(0.02), "gated");
        assert_eq!(f.rejected, 1);
        assert!(f.x == x0 && f.p == p_before, "unchanged");
    });
    test("zero angular rate: the gyro bias moves to the reading by P / (P + sigma^2)", || {
        let x = Nominal { wb: V3::new(0.001, 0.0, -0.002), ..still(V3::zeros(), quat_exp(&V3::new(0.2, -0.1, 0.4))) };
        let mut f = Esekf::new(x, p0(), spec());
        let wm = V3::new(0.011, -0.004, 0.018);
        let sigma = 0.01;
        assert!(f.update_zero_rate(&wm, sigma));
        let k = 1e-3 / (1e-3 + sigma * sigma);
        assert!((f.x.wb - (x.wb + (wm - x.wb) * k)).amax() < 1e-12, "gyro bias");
        check_jacobian(&x, &zero_rate_jacobian(), |x| x.wb);
    });
    test("zero angular rate estimates the gyro bias on the pad", || {
        let bias = V3::new(0.01, -0.005, 0.02);
        let sigma = 0.0024;
        let mut f = Esekf::new(still(V3::zeros(), Quat::identity()), p0(), spec());
        let am = specific_force(&Quat::identity(), &V3::zeros());
        let mut rng = Rng::new(31);
        let n = 4000;
        for _ in 0..n {
            let wm = bias + rng.normal_vec::<3>(sigma);
            assert!(f.predict(&am, &wm, 1.0 / 400.0));
            f.update_zero_rate(&wm, sigma);
            f.update_zero_velocity(0.02);
        }
        let tol = 5.0 * sigma / (n as f64).sqrt();
        assert!((f.x.wb - bias).amax() < tol, "bias error {:.2e} (tolerance {:.2e})", (f.x.wb - bias).amax(), tol);
        assert!(is_spd(&f.p), "P must stay symmetric positive definite");
    });
    test("bad sigmas and readings are rejected and change nothing", || {
        let mut f = Esekf::new(still(V3::zeros(), Quat::identity()), p0(), spec());
        let (x0, p_before) = (f.x, f.p);
        for sigma in [0.0, -0.01, f64::NAN, f64::INFINITY] {
            assert!(!f.update_zero_velocity(sigma), "ZUPT sigma {}", sigma);
            assert!(!f.update_zero_rate(&V3::zeros(), sigma), "ZARU sigma {}", sigma);
        }
        assert_eq!(f.rejected, 0, "bad sigmas are not counted");
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(!f.update_zero_rate(&V3::new(0.0, bad, 0.0), 0.01), "gyro {}", bad);
        }
        assert_eq!(f.rejected, 3, "bad readings are counted");
        assert!(f.x == x0 && f.p == p_before, "state and covariance unchanged");
    });
    test("monoprop pad sigmas", || {
        assert!(mp::SIGMA_ZERO_VELOCITY.is_finite() && mp::SIGMA_ZERO_VELOCITY > 0.0, "zero-velocity sigma");
        let s = mp::sigma_zero_rate();
        assert!(s.is_finite() && s > 0.0, "zero-rate sigma");
        assert_close(s, per_sample_sigma(mp::bmi323::GYRO_NOISE_DENSITY, mp::IMU_RATE_HZ), 1e-18, "the gyro's per-sample noise");
    });
}
