#![allow(dead_code)]
use Estimation::*;

// rng.rs: a tiny deterministic random number generator (provided, no_std).
// SplitMix64 + Box-Muller. Same seed -> same numbers on every machine, so tests are repeatable.

pub struct Rng {
    state: u64,
    spare: Option<f64>,
}

impl Rng {
    pub fn new(seed: u64) -> Self {
        Rng { state: seed ^ 0x9E37_79B9_7F4A_7C15, spare: None }
    }
    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    /// Uniform in (0, 1).
    pub fn uniform(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }
    /// Standard normal N(0, 1).
    pub fn gauss(&mut self) -> f64 {
        if let Some(s) = self.spare.take() {
            return s;
        }
        let (u1, u2) = (self.uniform(), self.uniform());
        let r = (-2.0 * u1.ln()).sqrt();
        let t = 2.0 * PI * u2;
        self.spare = Some(r * t.sin());
        r * t.cos()
    }
    /// N(0, sigma^2)
    pub fn normal(&mut self, sigma: f64) -> f64 {
        sigma * self.gauss()
    }
    /// Vector of independent N(0, sigma^2) entries.
    pub fn normal_vec<const N: usize>(&mut self, sigma: f64) -> Vector<N> {
        Vector::<N>::from_fn(|_, _| self.normal(sigma))
    }
}
