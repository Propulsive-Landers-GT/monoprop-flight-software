#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

const NAMES: [&str; 5] = ["dp", "dv", "dtheta", "da_b", "dw_b"];

pub fn run() {
    test("the worked example", || {
        let spec = ImuSpec { sigma_a: 0.03, sigma_g: 0.0, sigma_aw: 1e-3, sigma_gw: 0.0 };
        let q = process_noise(&spec, 0.005);
        assert_close(q[(3, 3)], 2.25e-8, 1e-20, "velocity entry");
        assert_close(q[(9, 9)], 5e-9, 1e-20, "accel-bias entry");
    });
    test("Q has the right diagonal and nothing else", || {
        let spec = ImuSpec { sigma_a: 0.03, sigma_g: 0.002, sigma_aw: 1e-3, sigma_gw: 1e-4 };
        let dt = 0.005;
        let q = process_noise(&spec, dt);
        let want = [0.0, 0.0, 0.0,
            9e-4 * dt * dt, 9e-4 * dt * dt, 9e-4 * dt * dt,
            4e-6 * dt * dt, 4e-6 * dt * dt, 4e-6 * dt * dt,
            1e-6 * dt, 1e-6 * dt, 1e-6 * dt,
            1e-8 * dt, 1e-8 * dt, 1e-8 * dt];
        for r in 0..15 {
            for c in 0..15 {
                let w = if r == c { want[r] } else { 0.0 };
                assert!((q[(r, c)] - w).abs() <= 1e-12 * (w.abs() + 1e-20), "Q[({}, {})] = {:e}, expected {:e} ({} block)", r, c, q[(r, c)], w, NAMES[r / 3]);
            }
        }
    });
    test("Monte Carlo: 400 vehicles, 2 s, average NEES about 15", || {
        let x0 = Nominal {
            p: V3::new(1.0, 2.0, 3.0),
            v: V3::new(0.5, -0.3, 0.2),
            q: quat_exp(&V3::new(0.3, -0.2, 0.9)),
            ab: V3::new(0.1, -0.05, 0.08),
            wb: V3::new(0.01, 0.02, -0.01),
        };
        let (am, wm) = (V3::new(0.4, -0.3, 9.9), V3::new(0.2, -0.4, 0.3));
        let spec = ImuSpec { sigma_a: 0.05, sigma_g: 0.005, sigma_aw: 1e-3, sigma_gw: 1e-4 };
        let (dt, steps, runs) = (0.01, 200, 400);
        // Small initial uncertainty so the result is dominated by Q (the part being tested).
        let p0d = [1e-4, 1e-4, 1e-4, 1e-6, 1e-6, 1e-6, 1e-7, 1e-7, 1e-7, 1e-8, 1e-8, 1e-8, 1e-9, 1e-9, 1e-9];
        let mut p = diag(p0d);
        let mut x = x0;
        for _ in 0..steps {
            let f = error_jacobian(&x, &am, &wm, dt);
            x = propagate_nominal(&x, &am, &wm, dt);
            p = symmetrized(&(f * p * f.transpose() + process_noise(&spec, dt)));
        }
        let mut rng = Rng::new(11);
        let mut total = 0.0;
        for _ in 0..runs {
            let dx = Vector::<15>::from_fn(|i, _| p0d[i].sqrt() * rng.gauss());
            let mut xt = inject(&x0, &dx);
            for _ in 0..steps {
                let na: V3 = rng.normal_vec(spec.sigma_a);
                let ng: V3 = rng.normal_vec(spec.sigma_g);
                xt = propagate_nominal(&xt, &(am - na), &(wm - ng), dt);
                xt.ab += rng.normal_vec::<3>(spec.sigma_aw * dt.sqrt());
                xt.wb += rng.normal_vec::<3>(spec.sigma_gw * dt.sqrt());
            }
            let e = difference(&xt, &x);
            total += e.dot(&solve_spd(&p, &e).expect("P should stay positive definite"));
        }
        let nees = total / runs as f64;
        assert!(nees > 13.5 && nees < 16.5, "average NEES {:.2}, expected about 15 (13.5 to 16.5). Above: Q too small. Below: Q too large.", nees);
    });
}
