#![allow(dead_code)]
use Estimation::*;
use super::rng::*;
use super::helpers::*;

// ---------------------------------------------------------------------------
// sim.rs: truth trajectory and sensor simulator for the ESEKF exercises (provided).
// Inertial frame: z up, gravity g_I = (0, 0, -9.81). Body attitude q_IB.
// Test-only: uses the flight library (Estimation) and rng.rs. ImuSpec, g_inertial and mag_reference live in src/env.rs.
// ---------------------------------------------------------------------------


/// Smooth 3-D flight: a figure-eight with altitude changes, and a gently tumbling attitude.
pub fn truth_position(t: f64) -> (V3, V3, V3) {
    let (w1, w2, w3) = (0.4, 0.8, 0.3);
    let p = V3::new(3.0 * (w1 * t).sin(), 2.0 * (w2 * t).sin(), 5.0 + 1.5 * (w3 * t).sin());
    let v = V3::new(3.0 * w1 * (w1 * t).cos(), 2.0 * w2 * (w2 * t).cos(), 1.5 * w3 * (w3 * t).cos());
    let a = V3::new(-3.0 * w1 * w1 * (w1 * t).sin(), -2.0 * w2 * w2 * (w2 * t).sin(), -1.5 * w3 * w3 * (w3 * t).sin());
    (p, v, a)
}
pub fn truth_body_rate(t: f64) -> V3 {
    V3::new(0.3 * (0.7 * t).sin(), 0.2 * (0.5 * t).cos(), 0.15 * (0.3 * t + 1.0).sin())
}

#[derive(Clone, Copy, Debug)]
pub struct ImuSample {
    pub t: f64,
    pub accel: V3, // specific force, body frame, m/s^2
    pub gyro: V3,  // body rate, rad/s
}

#[derive(Clone, Copy, Debug)]
pub struct TruthState {
    pub t: f64,
    pub p: V3,
    pub v: V3,
    pub q: Quat,
    pub accel_bias: V3,
    pub gyro_bias: V3,
}

pub struct Simulator {
    pub dt: f64,
    pub spec: ImuSpec,
    pub truth: TruthState,
    rng: Rng,
}

impl Simulator {
    pub fn new(dt: f64, spec: ImuSpec, accel_bias: V3, gyro_bias: V3, q0: Quat, seed: u64) -> Self {
        let (p, v, _) = truth_position(0.0);
        Simulator {
            dt,
            spec,
            truth: TruthState { t: 0.0, p, v, q: q0, accel_bias, gyro_bias },
            rng: Rng::new(seed),
        }
    }
    fn noise3(&mut self, s: f64) -> V3 {
        V3::new(self.rng.normal(s), self.rng.normal(s), self.rng.normal(s))
    }
    /// IMU sample for the *current* truth state, then advance the truth by dt.
    /// Truth attitude follows q_{k+1} = q_k * Exp(omega_k dt) exactly.
    pub fn step(&mut self) -> ImuSample {
        let t = self.truth.t;
        let (_, _, a_i) = truth_position(t);
        let w = truth_body_rate(t);
        let f_b = self.truth.q.inverse() * (a_i - g_inertial());
        let (sa, sg) = (self.spec.sigma_a, self.spec.sigma_g);
        let accel = f_b + self.truth.accel_bias + self.noise3(sa);
        let gyro = w + self.truth.gyro_bias + self.noise3(sg);
        // advance truth
        self.truth.q = self.truth.q * quat_exp(&(w * self.dt));
        self.truth.q.renormalize();
        self.truth.t = t + self.dt;
        let (p, v, _) = truth_position(self.truth.t);
        self.truth.p = p;
        self.truth.v = v;
        let sq = self.dt.sqrt();
        let (saw, sgw) = (self.spec.sigma_aw, self.spec.sigma_gw);
        let (dba, dbg) = (self.noise3(saw * sq), self.noise3(sgw * sq));
        self.truth.accel_bias += dba;
        self.truth.gyro_bias += dbg;
        ImuSample { t, accel, gyro }
    }
    /// Noisy GPS position of the current truth.
    pub fn gps(&mut self, sigma: f64) -> V3 {
        self.truth.p + self.noise3(sigma)
    }
    /// Noisy magnetometer: reference field in body axes plus noise.
    pub fn mag(&mut self, sigma: f64) -> V3 {
        self.truth.q.inverse() * mag_reference() + self.noise3(sigma)
    }
}

// ---------------------------------------------------------------------------
// Monoprop flight: the Monoprop sensor set (two IMUs, magnetometer, lidar, optional GNSS) for flights
// through the Navigator. Every sensor value comes from the NavConfig passed in (by default
// navigator_config(), whose values are PLACEHOLDERS), so the simulator and the navigator agree.
// Readings for the interval ending at a sample's timestamp are taken at its start, and the truth is then
// advanced exactly as the filter integrates (q <- q * Exp(w dt)), as in `Simulator`.
// ---------------------------------------------------------------------------

use Estimation::monoprop as mp;

/// Hover flight: still on the pad, climb CLIMB_DELAY after arming, hover, descend, land.
pub const HOVER_HEIGHT: f64 = 1.0; // m, well inside the lidar's range
pub const CLIMB_DELAY: f64 = 1.0; // s after arming
pub const CLIMB_TIME: f64 = 2.0; // s, also the descent time
pub const HOVER_TIME: f64 = 10.0; // s
pub const LANDED_TIME: f64 = 1.5; // s on the ground at the end

/// Height (m), vertical speed and acceleration of a smooth (cosine) climb or descent u seconds in.
fn ramp(u: f64, up: bool) -> (f64, f64, f64) {
    let k = PI / CLIMB_TIME;
    let h = 0.5 * HOVER_HEIGHT;
    let (c, s) = ((k * u).cos(), (k * u).sin());
    if up { (h * (1.0 - c), h * k * s, h * k * k * c) } else { (h * (1.0 + c), -h * k * s, -h * k * k * c) }
}

/// Time the flight ends (s).
pub fn hover_end(t_arm: f64) -> f64 {
    t_arm + CLIMB_DELAY + 2.0 * CLIMB_TIME + HOVER_TIME + LANDED_TIME
}

/// Truth position, velocity and acceleration (navigation frame) of the reference IMU; home = origin.
pub fn hover_profile(t: f64, t_arm: f64) -> (V3, V3, V3) {
    let t_up = t_arm + CLIMB_DELAY;
    let t_hover = t_up + CLIMB_TIME;
    let t_down = t_hover + HOVER_TIME;
    let t_land = t_down + CLIMB_TIME;
    let (z, vz, az) = if t < t_up {
        (0.0, 0.0, 0.0)
    } else if t < t_hover {
        ramp(t - t_up, true)
    } else if t < t_down {
        (HOVER_HEIGHT, 0.0, 0.0)
    } else if t < t_land {
        ramp(t - t_down, false)
    } else {
        (0.0, 0.0, 0.0)
    };
    (V3::new(0.0, 0.0, z), V3::new(0.0, 0.0, vz), V3::new(0.0, 0.0, az))
}

/// Truth body rate and angular acceleration: a gentle wobble (a few degrees of tilt, slow yaw) scaled by
/// height, so it is zero on the pad.
pub fn hover_body_rate(t: f64, t_arm: f64) -> (V3, V3) {
    let (p, v, _) = hover_profile(t, t_arm);
    let (e, de) = (p.z / HOVER_HEIGHT, v.z / HOVER_HEIGHT);
    let base = V3::new(0.05 * (1.3 * t).sin(), 0.04 * (0.9 * t).cos(), 0.1 * (0.5 * t).sin());
    let dbase = V3::new(0.05 * 1.3 * (1.3 * t).cos(), -0.04 * 0.9 * (0.9 * t).sin(), 0.1 * 0.5 * (0.5 * t).cos());
    (base * e, base * de + dbase * e)
}

#[derive(Clone, Copy, Debug)]
pub struct MonopropOptions {
    pub seed: u64,
    pub t_arm: f64,                       // s
    pub pad_height: f64,                  // true height of the reference IMU above the pad (m)
    pub gnss: bool,
    pub b_dropout: Option<(f64, f64)>,    // IMU B gives no samples between these times (s)
    pub b_clock_offset_us: i64,           // IMU B's timestamp minus A's
    pub b_clock_jitter_us: i64,           // plus a uniform jitter of up to this, each sample
}

impl MonopropOptions {
    /// The standard hover: arm at 3 s, IMU 20 cm above the pad, GNSS on, both IMUs, same clock. (20 cm keeps
    /// the pad well above the lidar's 4 cm floor even with the conservative 5 cm noise placeholder, so the
    /// measured-ground path is used.)
    pub fn hover(seed: u64) -> Self {
        MonopropOptions { seed, t_arm: 3.0, pad_height: 0.20, gnss: true, b_dropout: None, b_clock_offset_us: 0, b_clock_jitter_us: 0 }
    }
}

/// One IMU's truth: turn-on bias in its own axes (test choice: accel sigma 0.05 m/s^2, gyro 0.005 rad/s).
#[derive(Clone, Copy, Debug)]
pub struct ImuTruth {
    pub accel_bias: V3,
    pub gyro_bias: V3,
}

/// GNSS noise used by the simulator (test choice: SIGMA_POS horizontally, twice that vertically).
pub fn gnss_sigma() -> V3 {
    V3::new(mp::SIGMA_POS, mp::SIGMA_POS, 2.0 * mp::SIGMA_POS)
}

pub struct MonopropSim {
    pub cfg: mp::NavConfig,
    pub opt: MonopropOptions,
    pub dt: f64,
    pub k: u64,
    pub t: f64,
    pub p: V3,
    pub v: V3,
    pub q: Quat,
    pub imu_a: ImuTruth,
    pub imu_b: ImuTruth,
    spec: ImuSpec,
    rng: Rng,
    jitter_rng: Rng, // IMU B's clock jitter only, so enabling it doesn't change any other noise
}

pub const SIM_START_US: u64 = 1_000_000;

impl MonopropSim {
    pub fn new(cfg: mp::NavConfig, opt: MonopropOptions) -> Self {
        let mut rng = Rng::new(opt.seed);
        let imu = |rng: &mut Rng| ImuTruth { accel_bias: rng.normal_vec::<3>(0.05), gyro_bias: rng.normal_vec::<3>(0.005) };
        let (imu_a, imu_b) = (imu(&mut rng), imu(&mut rng));
        let q = quat_exp(&V3::new(0.0, 0.0, rng.uniform() * 6.0 - 3.0)); // random heading, level
        MonopropSim {
            cfg,
            opt,
            dt: 1.0 / mp::IMU_RATE_HZ,
            k: 0,
            t: 0.0,
            p: V3::zeros(),
            v: V3::zeros(),
            q,
            imu_a,
            imu_b,
            spec: cfg.imu_spec,
            rng,
            jitter_rng: Rng::new(opt.seed ^ 0x6A09_E667_F3BC_C908),
        }
    }

    pub fn period_us(&self) -> u64 {
        1_000_000 / mp::IMU_RATE_HZ as u64
    }

    /// Timestamp of the current truth time on IMU A's clock.
    pub fn t_us(&self) -> u64 {
        SIM_START_US + self.k * self.period_us()
    }

    fn noise3(&mut self, s: f64) -> V3 {
        self.rng.normal_vec::<3>(s)
    }

    /// The truth as the filter's nominal state: biases are the mean of the two IMUs' biases in body axes,
    /// which is what the navigator's combined sample carries while both IMUs are used.
    pub fn truth_nominal(&self) -> Nominal {
        let c = &self.cfg.imu_pair;
        let ab = (c.a_rotation * self.imu_a.accel_bias + c.b_rotation * self.imu_b.accel_bias) * 0.5;
        let wb = (c.a_rotation * self.imu_a.gyro_bias + c.b_rotation * self.imu_b.gyro_bias) * 0.5;
        Nominal { p: self.p, v: self.v, q: self.q, ab, wb }
    }

    /// One loop: the IMU readings over the next interval, the truth advanced, and the other sensors read at
    /// the new time. Returns the navigator inputs, `armed` set from `t_arm` on.
    pub fn step(&mut self) -> mp::NavInputs {
        let (t, t_arm) = (self.t, self.opt.t_arm);
        let (_, _, a) = hover_profile(t, t_arm);
        let (w, alpha) = hover_body_rate(t, t_arm);
        let f_ref = self.q.inverse() * (a - g_inertial()); // specific force at the reference IMU
        let c = self.cfg.imu_pair;
        let ref_offset = match c.reference {
            mp::ImuId::A => c.a_offset,
            mp::ImuId::B => c.b_offset,
        };
        let (sa, sg) = (self.spec.sigma_a, self.spec.sigma_g);
        let read = |imu: ImuTruth, rot: M3, offset: V3, rng: &mut Rng| {
            let d = offset - ref_offset;
            let f = f_ref + alpha.cross(&d) + w.cross(&w.cross(&d));
            let accel = rot.transpose() * f + imu.accel_bias + rng.normal_vec::<3>(sa);
            let gyro = rot.transpose() * w + imu.gyro_bias + rng.normal_vec::<3>(sg);
            (accel, gyro)
        };
        let (acc_a, gyr_a) = read(self.imu_a, c.a_rotation, c.a_offset, &mut self.rng);
        let (acc_b, gyr_b) = read(self.imu_b, c.b_rotation, c.b_offset, &mut self.rng);
        // advance the truth
        self.q *= quat_exp(&(w * self.dt));
        self.q.renormalize();
        self.k += 1;
        self.t = self.k as f64 * self.dt;
        let (p, v, _) = hover_profile(self.t, t_arm);
        self.p = p;
        self.v = v;
        let sq = self.dt.sqrt();
        let (saw, sgw) = (self.spec.sigma_aw * sq, self.spec.sigma_gw * sq);
        for imu in [&mut self.imu_a, &mut self.imu_b] {
            imu.accel_bias += self.rng.normal_vec::<3>(saw);
            imu.gyro_bias += self.rng.normal_vec::<3>(sgw);
        }
        // timestamps and the other sensors, at the new time
        let t_a = self.t_us();
        let jitter = if self.opt.b_clock_jitter_us > 0 {
            ((self.jitter_rng.uniform() * 2.0 - 1.0) * self.opt.b_clock_jitter_us as f64).round() as i64
        } else {
            0
        };
        let t_b = t_a.saturating_add_signed(self.opt.b_clock_offset_us + jitter);
        let b_off = self.opt.b_dropout.is_some_and(|(from, to)| self.t >= from && self.t < to);
        let mag = if self.k.is_multiple_of(4) {
            let field = (self.q.inverse() * mag_reference() + self.noise3(self.cfg.sigma_mag)) * 50.0;
            mp::MagSample::new(t_a, field)
        } else {
            None
        };
        let lidar = if self.k.is_multiple_of(8) {
            let l = self.cfg.lidar;
            let ps = self.p + self.q * l.offset;
            let d = self.q * l.beam.normalize();
            let true_range = (-self.opt.pad_height - ps.z) / d.z;
            // Modelling choice: below the lidar's minimum range the reading is the true range with no
            // noise, so it stays below the minimum and the navigator skips it. (The real VL53L1X reports a
            // range-status error there instead.)
            let range = if true_range < l.min_range { true_range } else { true_range + self.rng.normal(self.cfg.sigma_lidar_range) };
            mp::LidarSample::new(t_a, range)
        } else {
            None
        };
        let gnss = if self.opt.gnss && self.k.is_multiple_of(80) {
            let s = gnss_sigma();
            let noise = V3::new(self.rng.normal(s.x), self.rng.normal(s.y), self.rng.normal(s.z));
            mp::GnssSample::new(t_a, self.p + self.q * self.cfg.antenna_offset + noise, s)
        } else {
            None
        };
        mp::NavInputs {
            imu_a: mp::ImuSample::new(t_a, acc_a, gyr_a),
            imu_b: if b_off { None } else { mp::ImuSample::new(t_b, acc_b, gyr_b) },
            mag,
            lidar,
            gnss,
            armed: self.t >= t_arm,
        }
    }
}
