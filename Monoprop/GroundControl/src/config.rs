use anyhow::{Context, Result, ensure};
use serde::Deserialize;
use std::{net::SocketAddr, path::Path};

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub bind: SocketAddr,
    pub bridge: SocketAddr,
    pub telemetry_timeout_ms: u64,
    pub result_max_age_ms: u64,
    pub control_rate_hz: f64,
    pub guidance_rate_hz: f64,
    pub hover_position_m: [f64; 3],
    pub ascent_margin_m: f64,
    pub landing_position_m: [f64; 3],
    pub log_path: String,
    pub vehicle: Vehicle,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Vehicle {
    pub mass_kg: f64,
    pub dry_mass_kg: f64,
    pub inertia_kg_m2: [f64; 3],
    /// Signed body-Z location of the thrust pivot relative to the COM.
    pub thrust_pivot_z_m: f64,
    pub min_thrust_n: f64,
    pub max_thrust_n: f64,
    /// [ESC fraction, combined thrust in N], interpolated linearly.
    pub thrust_curve: Vec<[f64; 2]>,
    pub gimbal_limit_deg: f64,
    /// Zero for an electric prototype with constant mass.
    pub fuel_burn_kg_per_ns: f64,
    pub mpc_budget_ms: u64,
    pub guidance_duration_s: f64,
    pub guidance_max_velocity_mps: f64,
}

impl Config {
    pub fn load(path: &Path) -> Result<Self> {
        let config: Self = toml::from_str(
            &std::fs::read_to_string(path).with_context(|| format!("read {}", path.display()))?,
        )?;
        ensure!(
            (1..=1000).contains(&config.telemetry_timeout_ms),
            "telemetry timeout must be 1..1000 ms"
        );
        ensure!(
            config.result_max_age_ms > 0 && config.result_max_age_ms <= config.telemetry_timeout_ms,
            "result age must be positive and no larger than telemetry timeout"
        );
        ensure!(
            config.control_rate_hz.is_finite() && (1.0..=50.0).contains(&config.control_rate_hz),
            "control rate must be 1..50 Hz"
        );
        ensure!(
            config.guidance_rate_hz.is_finite() && (0.1..=5.0).contains(&config.guidance_rate_hz),
            "guidance rate must be 0.1..5 Hz"
        );
        ensure!(
            config
                .hover_position_m
                .iter()
                .chain(&config.landing_position_m)
                .all(|x| x.is_finite()),
            "targets must be finite"
        );
        ensure!(
            config.landing_position_m[2] >= 0.0
                && config.hover_position_m[2] > config.landing_position_m[2],
            "hover must be above landing height"
        );
        ensure!(
            config.ascent_margin_m.is_finite() && (0.0..=0.5).contains(&config.ascent_margin_m),
            "ascent margin must be 0..0.5 m"
        );
        config.vehicle.validate()?;
        Ok(config)
    }
}

impl Vehicle {
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.mass_kg.is_finite()
                && self.mass_kg > 0.0
                && self.dry_mass_kg.is_finite()
                && self.dry_mass_kg > 0.0
                && self.dry_mass_kg <= self.mass_kg,
            "invalid vehicle mass"
        );
        ensure!(
            self.inertia_kg_m2.iter().all(|x| x.is_finite() && *x > 0.0),
            "moments of inertia must be positive"
        );
        ensure!(
            self.thrust_pivot_z_m.is_finite() && self.thrust_pivot_z_m != 0.0,
            "invalid signed thrust pivot offset"
        );
        ensure!(
            self.min_thrust_n.is_finite()
                && self.max_thrust_n.is_finite()
                && self.min_thrust_n >= 0.0
                && self.min_thrust_n < self.mass_kg * 9.81
                && self.max_thrust_n > self.mass_kg * 9.81,
            "thrust bounds must bracket hover thrust"
        );
        ensure!(
            self.gimbal_limit_deg.is_finite()
                && self.gimbal_limit_deg > 0.0
                && self.gimbal_limit_deg <= 15.0,
            "gimbal limit must be in (0,15] degrees"
        );
        ensure!(
            self.fuel_burn_kg_per_ns.is_finite() && self.fuel_burn_kg_per_ns >= 0.0,
            "invalid fuel burn coefficient"
        );
        ensure!(
            (1..=100).contains(&self.mpc_budget_ms),
            "MPC budget must be 1..100 ms"
        );
        ensure!(
            self.guidance_duration_s.is_finite()
                && (1.0..=20.0).contains(&self.guidance_duration_s),
            "guidance duration must be 1..20 s"
        );
        ensure!(
            self.guidance_max_velocity_mps.is_finite() && self.guidance_max_velocity_mps > 0.0,
            "invalid guidance velocity bound"
        );
        ensure!(
            self.thrust_curve.len() >= 2 && self.thrust_curve.first() == Some(&[0.0, 0.0]),
            "thrust curve must start at [0,0] with at least two points"
        );
        ensure!(
            self.thrust_curve
                .iter()
                .all(|p| p.iter().all(|v| v.is_finite())
                    && (0.0..=1.0).contains(&p[0])
                    && p[1] >= 0.0),
            "invalid thrust curve point"
        );
        ensure!(
            self.thrust_curve
                .windows(2)
                .all(|p| p[1][0] > p[0][0] && p[1][1] > p[0][1]),
            "thrust curve must be strictly increasing"
        );
        ensure!(
            self.thrust_curve.last().unwrap()[1] >= self.max_thrust_n,
            "thrust curve does not cover maximum thrust"
        );
        ensure!(
            self.mass_kg - self.fuel_burn_kg_per_ns * self.max_thrust_n * self.guidance_duration_s
                >= self.dry_mass_kg,
            "guidance duration exhausts fuel"
        );
        Ok(())
    }

    pub fn throttle(&self, thrust_n: f64) -> Result<f32> {
        ensure!(
            thrust_n.is_finite() && thrust_n >= 0.0 && thrust_n <= self.max_thrust_n + 1e-6,
            "invalid MPC thrust"
        );
        for pair in self.thrust_curve.windows(2) {
            if thrust_n <= pair[1][1] {
                let fraction = (thrust_n - pair[0][1]) / (pair[1][1] - pair[0][1]);
                return Ok((pair[0][0] + fraction * (pair[1][0] - pair[0][0])) as f32);
            }
        }
        anyhow::bail!("thrust outside calibration curve")
    }
}
