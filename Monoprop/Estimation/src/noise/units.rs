use crate::*;

/// Per-sample standard deviation of white noise with density `density` (units/sqrt(Hz)) sampled at `rate_hz`.
pub fn per_sample_sigma(density: f64, rate_hz: f64) -> f64 {
    density * rate_hz.sqrt()
}

/// Inverse of per_sample_sigma.
pub fn density_from_sigma(sigma: f64, rate_hz: f64) -> f64 {
    sigma / rate_hz.sqrt()
}

/// Standard deviation of one random-walk step of length dt for intensity sigma_w.
pub fn rw_step_sigma(sigma_w: f64, dt: f64) -> f64 {
    sigma_w * dt.sqrt()
}
