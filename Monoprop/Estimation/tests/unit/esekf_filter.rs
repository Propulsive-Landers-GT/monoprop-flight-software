#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

fn spec() -> ImuSpec {
    ImuSpec { sigma_a: 0.03, sigma_g: 0.003, sigma_aw: 1e-4, sigma_gw: 1e-5 }
}

fn sample_state() -> Nominal {
    Nominal {
        p: V3::new(1.0, 2.0, 3.0),
        v: V3::new(0.5, -0.3, 0.2),
        q: quat_exp(&V3::new(0.3, -0.2, 0.9)),
        ab: V3::new(0.1, -0.05, 0.08),
        wb: V3::new(0.01, 0.02, -0.01),
    }
}

pub fn run() {
    test("new starts with no rejections", || {
        let f = Esekf::new(sample_state(), Mat::<15, 15>::identity(), spec());
        assert_eq!(f.rejected, 0);
    });
    test("predict = propagate_nominal for x, F P F^T + Q for P", || {
        let p0 = diag([1.0, 1.0, 1.0, 0.25, 0.25, 0.25, 0.01, 0.01, 0.03, 0.04, 0.04, 0.04, 1e-3, 1e-3, 1e-3]);
        let mut f = Esekf::new(sample_state(), p0, spec());
        let (am, wm, dt) = (V3::new(0.4, -0.3, 9.9), V3::new(0.2, -0.4, 0.3), 0.005);
        let fx = error_jacobian(&f.x, &am, &wm, dt);
        let want_x = propagate_nominal(&f.x, &am, &wm, dt);
        let want_p = fx * p0 * fx.transpose() + process_noise(&f.spec, dt);
        assert!(f.predict(&am, &wm, dt), "a normal step returns true");
        assert!(f.x == want_x, "the nominal state should follow propagate_nominal");
        assert!((f.p - want_p).amax() < 1e-12, "P should be F P F^T + Q, with F computed before x moved");
    });
    test("the worked example: from P = 0, one step gives P = Q", || {
        let x = Nominal { p: V3::zeros(), v: V3::zeros(), q: Quat::identity(), ab: V3::zeros(), wb: V3::zeros() };
        let mut f = Esekf::new(x, Mat::<15, 15>::zeros(), spec());
        f.predict(&V3::new(0.0, 0.0, 9.81), &V3::zeros(), 0.005);
        assert!((f.p - process_noise(&spec(), 0.005)).amax() < 1e-20);
    });
    test("10 s of dead reckoning: P stays symmetric positive definite and position uncertainty grows", || {
        let p0 = diag([0.01; 15]);
        let mut f = Esekf::new(sample_state(), p0, spec());
        for k in 0..2000 {
            let t = k as f64 * 0.005;
            f.predict(&V3::new(0.1 * t.sin(), 0.2, 9.8), &V3::new(0.1, -0.05 * t.cos(), 0.2), 0.005);
        }
        assert!(is_spd(&f.p), "P lost symmetry or positive-definiteness");
        assert!(f.p[(0, 0)] > 0.01, "position variance should have grown");
    });
    test("a NaN reading or a bad dt is refused and changes nothing", || {
        let mut f = Esekf::new(sample_state(), diag([0.01; 15]), spec());
        let (x0, p0) = (f.x, f.p);
        assert!(!f.predict(&V3::new(0.0, f64::NAN, 9.8), &V3::zeros(), 0.005), "NaN accelerometer");
        assert!(!f.predict(&V3::new(0.0, 0.0, 9.8), &V3::new(f64::INFINITY, 0.0, 0.0), 0.005), "infinite gyro");
        assert!(!f.predict(&V3::new(0.0, 0.0, 9.8), &V3::zeros(), f64::NAN), "NaN dt");
        assert!(!f.predict(&V3::new(0.0, 0.0, 9.8), &V3::zeros(), 0.0), "dt = 0");
        assert!(!f.predict(&V3::new(0.0, 0.0, 9.8), &V3::zeros(), -0.005), "negative dt (clock went backwards)");
        assert!(f.x == x0 && f.p == p0, "state and covariance must be untouched");
    });
}
