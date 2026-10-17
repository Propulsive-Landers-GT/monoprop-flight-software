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
