//! System test: the full-state filter flying simulated missions.
//!
//! A smooth 3-D trajectory with a tumbling attitude is simulated exactly; the IMU readings get the
//! VN-200 noise and random turn-on biases, and position, magnetometer and height readings are noisy.
//! The filter must stay accurate, report an uncertainty that matches its real error (NEES), and ride
//! through a position glitch, a dropout and NaN readings.

use super::filter::UpdateOutcome;
use super::full_state::*;
use super::math::*;

// ---- deterministic random numbers (SplitMix64 + Box-Muller) ----

struct Rng {
    state: u64,
    spare: Option<f64>,
}

impl Rng {
    fn new(seed: u64) -> Self {
        Rng { state: seed ^ 0x9E37_79B9_7F4A_7C15, spare: None }
    }
    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn uniform(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }
    fn gauss(&mut self) -> f64 {
        if let Some(s) = self.spare.take() {
            return s;
        }
        let (u1, u2) = (self.uniform(), self.uniform());
        let r = (-2.0 * u1.ln()).sqrt();
        let t = 2.0 * PI * u2;
        self.spare = Some(r * t.sin());
        r * t.cos()
    }
    fn v3(&mut self, sigma: f64) -> V3 {
        V3::new(self.gauss() * sigma, self.gauss() * sigma, self.gauss() * sigma)
    }
}

// ---- truth and sensors ----

/// A figure-eight with altitude changes: position, velocity, acceleration at time t.
fn truth_motion(t: f64) -> (V3, V3, V3) {
    let (w1, w2, w3) = (0.4, 0.8, 0.3);
    let p = V3::new(3.0 * (w1 * t).sin(), 2.0 * (w2 * t).sin(), 5.0 + 1.5 * (w3 * t).sin());
    let v = V3::new(3.0 * w1 * (w1 * t).cos(), 2.0 * w2 * (w2 * t).cos(), 1.5 * w3 * (w3 * t).cos());
    let a = V3::new(-3.0 * w1 * w1 * (w1 * t).sin(), -2.0 * w2 * w2 * (w2 * t).sin(), -1.5 * w3 * w3 * (w3 * t).sin());
    (p, v, a)
}

/// A gently tumbling body rate.
fn truth_rate(t: f64) -> V3 {
    V3::new(0.3 * (0.7 * t).sin(), 0.2 * (0.5 * t).cos(), 0.15 * (0.3 * t + 1.0).sin())
}

struct Sim {
    dt: f64,
    imu: ImuSpec,
    t: f64,
    truth: Nominal, // true position, velocity, attitude and biases
    rng: Rng,
}

impl Sim {
    fn new(seed: u64, rate_hz: f64) -> Self {
        let mut rng = Rng::new(seed);
        let q0 = quat_exp(&V3::new(0.2 * rng.gauss(), 0.2 * rng.gauss(), 6.0 * rng.uniform() - 3.0));
        let (ab, wb) = (rng.v3(0.01), rng.v3(5e-4)); // turn-on biases
        let (p, v, _) = truth_motion(0.0);
        Sim { dt: 1.0 / rate_hz, imu: ImuSpec::vn200(rate_hz), t: 0.0, truth: Nominal { p, v, q: q0, ab, wb }, rng }
    }

    /// IMU sample for the current truth, then advance the truth by dt (attitude integrated exactly).
    fn step(&mut self) -> ImuSample {
        let (_, _, a) = truth_motion(self.t);
        let w = truth_rate(self.t);
        let x = self.truth;
        let accel = x.q.inverse() * (a - g_inertial()) + x.ab + self.rng.v3(self.imu.sigma_a);
        let gyro = w + x.wb + self.rng.v3(self.imu.sigma_g);
        self.t += self.dt;
        let (p, v, _) = truth_motion(self.t);
        let mut q = x.q * quat_exp(&(w * self.dt));
        q.renormalize();
        let sq = self.dt.sqrt();
        let ab = x.ab + self.rng.v3(self.imu.sigma_aw * sq);
        let wb = x.wb + self.rng.v3(self.imu.sigma_gw * sq);
        self.truth = Nominal { p, v, q, ab, wb };
        ImuSample { accel, gyro }
    }

    fn position(&mut self, sigma: f64) -> V3 {
        self.truth.p + self.rng.v3(sigma)
    }

    /// Magnetometer in tesla (the filter only uses the direction).
    fn mag(&mut self, sigma: f64) -> V3 {
        (self.truth.q.inverse() * mag_reference() + self.rng.v3(sigma)) * mag_field_enu().norm()
    }

    fn height(&mut self, sigma: f64) -> f64 {
        self.truth.p.z + self.rng.gauss() * sigma
    }
}

// ---- one flight ----

const SIGMA_POS: f64 = 0.5;
const SIGMA_MAG: f64 = 0.02;
const SIGMA_HEIGHT: f64 = 0.1;
const SETTLE: f64 = 20.0;

#[derive(Default, Debug)]
struct Report {
    nees_avg: f64,
    pos_rms: f64,
    pos_max: f64,
    att_max_deg: f64,
    pos_used: u32,
    pos_rejected: u32,
    final_pos_err: f64,
    bias_err: (f64, f64),
}

/// Fly `seconds` at `rate_hz`: position at 10 Hz, magnetometer at 50 Hz, height at 20 Hz.
/// `fault(k, fix)` may corrupt a position fix (Some(bad)) or drop it (None).
fn fly(seed: u64, rate_hz: f64, seconds: f64, fault: &dyn Fn(usize, V3) -> Option<V3>) -> Report {
    let mut sim = Sim::new(seed, rate_hz);
    let (pos_every, mag_every, height_every) = ((rate_hz / 10.0) as usize, (rate_hz / 50.0) as usize, (rate_hz / 20.0) as usize);
    let (mag0, pos0) = (sim.mag(SIGMA_MAG), sim.position(SIGMA_POS));
    let mut filter: Option<FullStateESEKF> = None;
    let mut rep = Report::default();
    let (mut nees_sum, mut pos_sq, mut n) = (0.0, 0.0, 0usize);
    for k in 0..(seconds * rate_hz) as usize {
        let imu = sim.step();
        if filter.is_none() {
            filter = FullStateESEKF::align(FullState::new(ImuSpec::vn200(rate_hz)), &imu.accel, &mag0, &pos0, SIGMA_POS);
        }
        let f = filter.as_mut().expect("the simulator's first readings are never degenerate");
        assert!(f.predict(&imu, sim.dt), "a valid IMU sample was refused");
        if k % pos_every == pos_every - 1 {
            let z = sim.position(SIGMA_POS);
            if let Some(z) = fault(k, z) {
                match f.update_position(&z, SIGMA_POS) {
                    UpdateOutcome::Fused { .. } => rep.pos_used += 1,
                    _ => rep.pos_rejected += 1,
                }
            }
        }
        if k % mag_every == mag_every - 1 {
            let m = sim.mag(SIGMA_MAG);
            f.update_mag(&m, SIGMA_MAG);
        }
        if k % height_every == height_every - 1 {
            let h = sim.height(SIGMA_HEIGHT);
            f.update_height(h, SIGMA_HEIGHT);
        }
        let x = &f.nominal_state;
        assert!(
            x.p.iter().chain(x.v.iter()).chain(f.error_covariance.iter()).all(|v| v.is_finite()),
            "seed {}: a NaN or infinity appeared at step {}",
            seed,
            k
        );
        if sim.t > SETTLE {
            let e = difference(&sim.truth, x);
            if let Some(w) = solve_spd(&f.error_covariance, &e) {
                nees_sum += e.dot(&w);
            }
            let ep = (sim.truth.p - x.p).norm();
            pos_sq += ep * ep;
            n += 1;
            rep.pos_max = rep.pos_max.max(ep);
            rep.att_max_deg = rep.att_max_deg.max(boxminus(&sim.truth.q, &x.q).norm().to_degrees());
        }
    }
    let f = filter.unwrap();
    rep.nees_avg = nees_sum / n as f64;
    rep.pos_rms = (pos_sq / n as f64).sqrt();
    rep.final_pos_err = (sim.truth.p - f.nominal_state.p).norm();
    rep.bias_err = ((f.nominal_state.ab - sim.truth.ab).norm(), (f.nominal_state.wb - sim.truth.wb).norm());
    rep
}

#[test]
fn full_state_esekf_flies_simulated_missions() {
    // 1. Normal flights (VN-200 noise, 200 Hz, 60 s): accurate, and P describes the actual error.
    let reports: std::vec::Vec<Report> = (0..6).map(|s| fly(100 + s, 200.0, 60.0, &|_, z| Some(z))).collect();
    let mut nees = 0.0;
    for (i, r) in reports.iter().enumerate() {
        std::println!("flight {}: {:?}", i, r);
        assert!(r.pos_rms < 0.35, "flight {}: position rms {:.3} m", i, r.pos_rms);
        assert!(r.att_max_deg < 2.0, "flight {}: attitude error up to {:.2} deg", i, r.att_max_deg);
        // the 99.9 % gate refuses about 1 in 1000 good fixes: ~0.6 of the 600 per flight
        assert!(r.pos_rejected <= 3, "flight {}: {} good position fixes were rejected", i, r.pos_rejected);
        nees += r.nees_avg / reports.len() as f64;
    }
    std::println!("average NEES {:.2} (a consistent 15-state filter averages about 15)", nees);
    assert!(nees > 7.5 && nees < 30.0, "average NEES {:.1}: P does not describe the actual error", nees);

    // 2. A 30 m position glitch is rejected and doesn't disturb the estimate.
    let r = fly(7, 200.0, 60.0, &|k, z| Some(if k == 6019 { z + V3::new(30.0, 0.0, 0.0) } else { z }));
    assert!(r.pos_rejected >= 1, "the 30 m glitch was fused");
    assert!(r.pos_max < 1.5, "position error reached {:.2} m around the glitch", r.pos_max);

    // 3. A 10 s position dropout: dead reckoning with magnetometer and height stays within a few metres,
    //    then recovers.
    let r = fly(8, 200.0, 60.0, &|k, z| if (6000..8000).contains(&k) { None } else { Some(z) });
    assert!(r.pos_max < 3.0, "position error reached {:.2} m during the dropout", r.pos_max);
    assert!(r.final_pos_err < 1.0, "20 s after the dropout the position error is still {:.2} m", r.final_pos_err);

    // 4. 100 Hz works too.
    let r = fly(9, 100.0, 60.0, &|_, z| Some(z));
    assert!(r.pos_rms < 0.35 && r.att_max_deg < 2.0, "100 Hz: {:?}", r);

    // 5. Bad input changes nothing; bad start-up readings give None instead of a NaN filter.
    let model = FullState::new(ImuSpec::vn200(200.0));
    let up = V3::new(0.0, 0.0, GRAVITY);
    let mut f = FullStateESEKF::align(model, &up, &mag_reference(), &V3::zeros(), SIGMA_POS).expect("level, valid readings");
    let (x0, p0) = (f.nominal_state, f.error_covariance);
    let nan = V3::new(f64::NAN, 0.0, 0.0);
    assert!(!f.predict(&ImuSample { accel: nan, gyro: V3::zeros() }, 0.005), "NaN accelerometer");
    assert!(!f.predict(&ImuSample { accel: up, gyro: V3::zeros() }, 0.0), "dt = 0");
    assert_eq!(f.update_position(&nan, SIGMA_POS), UpdateOutcome::RejectedNonFinite);
    assert_eq!(f.update_mag(&V3::zeros(), SIGMA_MAG), UpdateOutcome::RejectedNonFinite);
    assert_eq!(f.update_height(f64::NAN, SIGMA_HEIGHT), UpdateOutcome::RejectedNonFinite);
    assert!(matches!(f.update_height(25.0, SIGMA_HEIGHT), UpdateOutcome::RejectedGate { .. }), "a 25 m height jump must be gated");
    assert!(f.nominal_state == x0 && f.error_covariance == p0, "refused input must not change the filter");
    assert_eq!(f.rejected, 4);
    assert!(FullStateESEKF::align(model, &up, &V3::new(0.0, 0.0, 30.0), &V3::zeros(), SIGMA_POS).is_none(), "magnetometer parallel to gravity");
    assert!(FullStateESEKF::align(model, &V3::zeros(), &mag_reference(), &V3::zeros(), SIGMA_POS).is_none(), "zero accelerometer");
}
