#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

const NAMES: [&str; 5] = ["dp", "dv", "dtheta", "da_b", "dw_b"];

fn sample_state() -> (Nominal, V3, V3) {
    let x = Nominal {
        p: V3::new(1.0, 2.0, 3.0),
        v: V3::new(0.5, -0.3, 0.2),
        q: quat_exp(&V3::new(0.3, -0.2, 0.9)),
        ab: V3::new(0.1, -0.05, 0.08),
        wb: V3::new(0.01, 0.02, -0.01),
    };
    (x, V3::new(0.4, -0.3, 9.9), V3::new(0.2, -0.4, 0.3))
}

pub fn run() {
    test("the worked example: level, still, gravity leaks sideways", || {
        let x = Nominal { p: V3::zeros(), v: V3::zeros(), q: Quat::identity(), ab: V3::zeros(), wb: V3::zeros() };
        let f = error_jacobian(&x, &V3::new(0.0, 0.0, 9.81), &V3::zeros(), 0.01);
        let block: M3 = f.fixed_view::<3, 3>(3, 6).into_owned();
        assert!((block - M3::new(0.0, 0.0981, 0.0, -0.0981, 0.0, 0.0, 0.0, 0.0, 0.0)).amax() < 1e-12, "(dv, dtheta) block\n{}", block);
    });
    test("F matches the nonlinear strapdown, block by block", || {
        let (x, am, wm) = sample_state();
        // F drops O(dt^2) terms (e.g. 0.5 R [a]x dtheta dt^2 in position), so allow ~0.6 % of |dx|;
        // every term F does contain is at least dt = 2 % of |dx|.
        let dt = 0.02;
        let f = error_jacobian(&x, &am, &wm, dt);
        let x1 = propagate_nominal(&x, &am, &wm, dt);
        let mut rng = Rng::new(3);
        for b in 0..5 {
            let mut dx = Vector::<15>::zeros();
            for i in 0..3 {
                dx[3 * b + i] = 1e-5 * rng.gauss();
            }
            let actual = difference(&propagate_nominal(&inject(&x, &dx), &am, &wm, dt), &x1);
            let pred = f * dx;
            for r in 0..5 {
                let e = (sub3(&actual, 3 * r) - sub3(&pred, 3 * r)).norm();
                assert!(
                    e < 6e-3 * dx.norm(),
                    "perturbing {} gives the wrong change in {}: block ({}, {}) of F is off (error {:.2e} for a nudge of {:.2e})",
                    NAMES[b], NAMES[r], NAMES[r], NAMES[b], e, dx.norm()
                );
            }
        }
    });
    test("bias rows are identity (biases don't change between updates)", || {
        let (x, am, wm) = sample_state();
        let f = error_jacobian(&x, &am, &wm, 0.01);
        for r in 9..15 {
            for c in 0..15 {
                let want = if r == c { 1.0 } else { 0.0 };
                assert!((f[(r, c)] - want).abs() < 1e-15, "F[({}, {})] = {}, expected {}", r, c, f[(r, c)], want);
            }
        }
    });
}
