#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

fn sample_state() -> Nominal {
    Nominal {
        p: V3::new(1.0, 2.0, 3.0),
        v: V3::new(0.5, -0.3, 0.2),
        q: quat_exp(&V3::new(0.3, -0.2, 0.9)),
        ab: V3::new(0.1, -0.05, 0.08),
        wb: V3::new(0.01, 0.02, -0.01),
    }
}

fn spec() -> ImuSpec {
    ImuSpec { sigma_a: 0.03, sigma_g: 0.003, sigma_aw: 1e-4, sigma_gw: 1e-5 }
}

fn p0() -> Mat<15, 15> {
    diag([1.0, 1.0, 1.0, 0.25, 0.25, 0.25, 0.01, 0.01, 0.03, 0.04, 0.04, 0.04, 1e-3, 1e-3, 1e-3])
}

fn gps(f: &mut Esekf, z: &V3, sigma: f64, gate: f64) -> bool {
    let y = z - f.x.p;
    f.update(&y, &gps_jacobian(), &(M3::identity() * (sigma * sigma)), gate)
}

pub fn run() {
    test("the reset Jacobian is the first-order effect of moving the attitude reference", || {
        // After injecting dtheta_hat, a remaining error e (old reference) becomes Log(Exp(-dtheta_hat) Exp(dtheta_hat + e)).
        let a = V3::new(0.01, -0.02, 0.015);
        let g = reset_jacobian(&a);
        let eps = 1e-6;
        let after = |e: V3| quat_log(&(quat_exp(&a).inverse() * quat_exp(&(a + e))));
        for j in 0..3 {
            let mut e = V3::zeros();
            e[j] = eps;
            let col = (after(e) - after(-e)) / (2.0 * eps);
            let gcol: V3 = g.fixed_view::<3, 1>(6, 6 + j).into_owned();
            assert!((col - gcol).amax() < 1e-3, "column {} of the attitude block: numerical {}, yours {}", j, col.transpose(), gcol.transpose());
        }
        let mut rest = g;
        set_block3(&mut rest, 6, 6, &M3::identity());
        assert!((rest - Mat::<15, 15>::identity()).amax() < 1e-15, "only the attitude block differs from the identity");
    });
    test("the worked example: a precise GPS fix pins the position", || {
        let mut f = Esekf::new(sample_state(), p0(), spec());
        let z = f.x.p + V3::new(0.5, -0.4, 0.3);
        assert!(gps(&mut f, &z, 0.01, GATE_3DOF), "a 0.7 m innovation with 1 m uncertainty must be accepted");
        assert!((f.x.p - z).amax() < 1e-3, "position {}, fix {}", f.x.p.transpose(), z.transpose());
        for i in 0..3 {
            assert!((f.p[(i, i)] - 1e-4).abs() < 1e-6, "position variance {:e}, expected about 1e-4", f.p[(i, i)]);
        }
        assert!(is_spd(&f.p), "P must stay symmetric positive definite");
    });
    test("a useless sensor (huge R) changes almost nothing", || {
        let mut f = Esekf::new(sample_state(), p0(), spec());
        let x0 = f.x;
        gps(&mut f, &(x0.p + V3::new(1.0, 1.0, 1.0)), 1e6, GATE_3DOF);
        assert!((f.x.p - x0.p).amax() < 1e-9, "moved by {}", (f.x.p - x0.p).transpose());
    });
    test("a 30 m glitch is rejected, counted, and changes nothing else", || {
        let mut f = Esekf::new(sample_state(), p0(), spec());
        let (x0, p_before) = (f.x, f.p);
        assert!(!gps(&mut f, &(x0.p + V3::new(30.0, 0.0, 0.0)), 0.5, GATE_3DOF), "should return false");
        assert_eq!(f.rejected, 1, "rejected counter");
        assert!(f.x == x0 && f.p == p_before, "state and covariance must be untouched: gate before you modify anything");
    });
    test("a magnetometer update removes an observable attitude error", || {
        let m = mag_reference();
        let truth = sample_state();
        let mb = truth.q.inverse() * m;
        // an error perpendicular to the field (rotation about the field is invisible to one vector)
        let axis = mb.cross(&V3::z());
        let err = axis * (0.05 / axis.norm());
        let mut x = truth;
        x.q = truth.q * quat_exp(&err);
        let mut f = Esekf::new(x, p0(), spec());
        for _ in 0..20 {
            let y = mb - mag_prediction(&f.x, &m);
            assert!(f.update(&y, &mag_jacobian(&f.x, &m), &(M3::identity() * 1e-4), GATE_3DOF), "noise-free readings should be accepted");
        }
        let left = boxminus(&truth.q, &f.x.q).norm();
        assert!(left < 2e-3, "attitude error still {:.4} rad (started at 0.05)", left);
        assert!(is_spd(&f.p), "P must stay symmetric positive definite");
    });
    test("a NaN reading is refused, counted, and changes nothing", || {
        let mut f = Esekf::new(sample_state(), p0(), spec());
        let (x0, p_before) = (f.x, f.p);
        let z = V3::new(f64::NAN, x0.p.y, x0.p.z);
        assert!(!gps(&mut f, &z, 0.5, GATE_3DOF), "should return false");
        assert_eq!(f.rejected, 1, "rejected counter");
        assert!(f.x == x0 && f.p == p_before, "state and covariance must be untouched");
        assert!(f.x.p.iter().all(|v| v.is_finite()), "no NaN may reach the state");
    });
}
