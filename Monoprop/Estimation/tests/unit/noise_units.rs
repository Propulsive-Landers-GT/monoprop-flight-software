#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

pub fn run() {
    test("the worked example: 2e-4 at 400 Hz is 4e-3 per sample", || {
        assert_close(per_sample_sigma(2e-4, 400.0), 4e-3, 1e-15, "per-sample sigma");
        assert_close(per_sample_sigma(2e-4, 100.0), 2e-3, 1e-15, "at 100 Hz");
    });
    test("density <-> per-sample sigma round trip", || {
        assert_close(density_from_sigma(per_sample_sigma(3e-4, 1000.0), 1000.0), 3e-4, 1e-15, "round trip");
    });
    test("random-walk step grows with sqrt(dt)", || {
        assert_close(rw_step_sigma(1e-4, 0.01), 1e-5, 1e-18, "1e-4 over 10 ms");
        assert_close(rw_step_sigma(2e-5, 4.0), 4e-5, 1e-18, "2e-5 over 4 s");
    });
    test("a simulated random walk wanders by sigma_w sqrt(t)", || {
        let mut rng = Rng::new(9);
        let (sw, dt, steps, runs) = (1e-3, 0.01, 1000, 400);
        let mut sum_sq = 0.0;
        for _ in 0..runs {
            let mut b = 0.0;
            for _ in 0..steps {
                b += rng.normal(rw_step_sigma(sw, dt));
            }
            sum_sq += b * b;
        }
        let rms = (sum_sq / runs as f64).sqrt();
        let want = sw * (steps as f64 * dt).sqrt();
        assert!((rms / want - 1.0).abs() < 0.1, "rms wander after 10 s: {:.3e}, expected {:.3e}", rms, want);
    });
}
