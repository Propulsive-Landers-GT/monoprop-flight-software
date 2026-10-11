// Whole-filter checks: the complete ESEKF on simulated flights
// with the VN-200 noise model, judged by its own covariance (NEES) as well as by its error.
#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

struct Run {
    nees_avg: f64,
    pos_rms: f64,
    att_max_deg: f64,
    finite: bool,
}

fn flight(seed: u64, rate_hz: f64, seconds: f64) -> Run {
    let spec = ImuSpec::vn200(rate_hz);
    let dt = 1.0 / rate_hz;
    let mut rng = Rng::new(seed);
    // turn-on biases at the VN-200's scale (generous, as the template's P0 notes)
    let ab = V3::new(rng.normal(0.01), rng.normal(0.01), rng.normal(0.01));
    let wb = V3::new(rng.normal(5e-4), rng.normal(5e-4), rng.normal(5e-4));
    let q0 = quat_exp(&V3::new(0.0, 0.0, rng.uniform() * 6.0 - 3.0));
    let mut sim = Simulator::new(dt, spec, ab, wb, q0, seed);
    let (sigma_pos, sigma_mag) = (0.5, 0.02);
    let (pos_every, mag_every) = ((rate_hz / 10.0) as usize, (rate_hz / 50.0) as usize);
    let mag0 = sim.mag(sigma_mag);
    let pos0 = sim.gps(sigma_pos);
    let mut f: Option<Esekf> = None;
    let (mut nees_sum, mut n, mut pos_sq, mut att_max, mut finite) = (0.0, 0usize, 0.0, 0.0f64, true);
    let steps = (seconds * rate_hz) as usize;
    for k in 0..steps {
        let imu = sim.step();
        if f.is_none() {
            f = align(&imu.accel, &mag0, &pos0, spec, sigma_pos);
        }
        let f = f.as_mut().expect("the simulator's readings are never degenerate");
        assert!(f.predict(&imu.accel, &imu.gyro, dt));
        if k % pos_every == pos_every - 1 {
            let z = sim.gps(sigma_pos);
            f.update_gps(&z, sigma_pos);
        }
        if k % mag_every == mag_every - 1 {
            let m = sim.mag(sigma_mag);
            f.update_mag(&m, &mag_reference(), sigma_mag);
        }
        finite &= f.x.p.iter().chain(f.p.iter()).all(|v| v.is_finite());
        if sim.truth.t > 20.0 {
            let truth = Nominal { p: sim.truth.p, v: sim.truth.v, q: sim.truth.q, ab: sim.truth.accel_bias, wb: sim.truth.gyro_bias };
            let e = difference(&truth, &f.x);
            // NEES = e^T P^-1 e (inline, so this test doesn't need the consistency module)
            if let Some(w) = solve_spd(&f.p, &e) {
                let v = e.dot(&w);
                nees_sum += v;
                n += 1;
            }
            pos_sq += (sim.truth.p - f.x.p).norm_squared();
            att_max = att_max.max(boxminus(&sim.truth.q, &f.x.q).norm().to_degrees());
        }
    }
    Run { nees_avg: nees_sum / n as f64, pos_rms: (pos_sq / n as f64).sqrt(), att_max_deg: att_max, finite }
}

pub fn run() {
    test("VN-200 noise, 200 Hz, 10 flights of 60 s: accurate, finite, and honest about its uncertainty (NEES)", || {
        let runs: Vec<Run> = (0..10).map(|s| flight(100 + s, 200.0, 60.0)).collect();
        let nees = runs.iter().map(|r| r.nees_avg).sum::<f64>() / runs.len() as f64;
        for (i, r) in runs.iter().enumerate() {
            println!("          flight {}: NEES {:.1}, position rms {:.3} m, attitude max {:.2} deg", i, r.nees_avg, r.pos_rms, r.att_max_deg);
            assert!(r.finite, "flight {}: a NaN or infinity appeared in the state or covariance", i);
            assert!(r.pos_rms < 0.35, "flight {}: position rms {:.3} m", i, r.pos_rms);
            assert!(r.att_max_deg < 2.0, "flight {}: attitude error up to {:.2} deg", i, r.att_max_deg); // the magnetometer alone is about 1 deg noisy (sigma 0.02 on a unit vector)
        }
        println!("          average NEES over all flights: {:.2} (15 states: a consistent filter averages about 15)", nees);
        assert!(nees > 5.0 && nees < 45.0, "average NEES {:.1}: far from 15 means P does not describe the real error", nees);
    });
    test("100 Hz (the logged-data rate in the template) works too", || {
        let r = flight(7, 100.0, 60.0);
        assert!(r.finite && r.pos_rms < 0.35 && r.att_max_deg < 2.0, "pos rms {:.3} m, att max {:.2} deg", r.pos_rms, r.att_max_deg);
    });
}
