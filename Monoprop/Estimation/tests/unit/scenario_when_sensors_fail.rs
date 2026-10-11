#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

fn spec() -> ImuSpec {
    ImuSpec { sigma_a: 0.03, sigma_g: 0.003, sigma_aw: 1e-4, sigma_gw: 1e-5 }
}

fn mission() -> Mission {
    Mission { steps: 12000, gps_every: 20, mag_every: 4, sigma_gps: 0.5, sigma_mag: 0.02 }
}

fn sim(seed: u64) -> Simulator {
    let mut r = Rng::new(seed + 1000);
    let q0 = quat_exp(&V3::new(0.2 * r.gauss(), 0.2 * r.gauss(), 3.0 * r.uniform()));
    let ab = V3::new(0.1 * r.gauss(), 0.1 * r.gauss(), 0.1 * r.gauss());
    let wb = V3::new(0.01 * r.gauss(), 0.01 * r.gauss(), 0.01 * r.gauss());
    Simulator::new(0.005, spec(), ab, wb, q0, seed)
}

pub fn run() {
    test("no faults: same requirements as before", || {
        for seed in [1u64, 2] {
            let mut s = sim(seed);
            let (f, rep) = fly(&mut s, &mission(), &|_, z| Some(z));
            println!("seed {}: {:?}", seed, rep);
            assert!(rep.pos_rms < 0.35 && rep.att_max_deg < 1.0, "seed {}: {:?}", seed, rep);
            assert!((f.x.ab - s.truth.accel_bias).norm() < 0.03 && (f.x.wb - s.truth.gyro_bias).norm() < 1e-3, "seed {}: biases not estimated", seed);
        }
    });
    test("a 30 m GPS glitch at t = 30 s is rejected and doesn't disturb the estimate", || {
        let mut s = sim(3);
        let (_, rep) = fly(&mut s, &mission(), &|k, z| Some(if k == 6019 { z + V3::new(30.0, 0.0, 0.0) } else { z }));
        println!("seed 3 with glitch: {:?}", rep);
        assert!(rep.gps_rejected >= 1, "the glitch was accepted");
        assert!(rep.pos_max < 1.5, "position error reached {:.2} m", rep.pos_max);
        assert!(rep.pos_rms < 0.35, "position rms {:.3} m", rep.pos_rms);
    });
    test("10 s GPS dropout: dead reckoning stays within a few metres, then recovers", || {
        let mut s = sim(4);
        let (f, rep) = fly(&mut s, &mission(), &|k, z| if (6000..8000).contains(&k) { None } else { Some(z) });
        println!("seed 4 with dropout: {:?}", rep);
        assert!(rep.pos_max < 3.0, "position error reached {:.2} m during the dropout", rep.pos_max);
        assert!(rep.att_max_deg < 1.5, "attitude error up to {:.2} deg", rep.att_max_deg);
        assert_eq!(rep.gps_used, 500, "fixes during the dropout must not be used (600 - 100)");
        let e = (s.truth.p - f.x.p).norm();
        assert!(e < 1.0, "20 s after the dropout the position error is still {:.2} m", e);
    });
}
