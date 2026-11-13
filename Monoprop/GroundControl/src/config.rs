use anyhow::{Result, ensure};
use serde::Deserialize;

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Model {
    pub mass_kg: f64,
    pub inertia_kg_m2: [f64; 3],
    /// Thrust application point is [0, 0, -lever_arm_m] relative to CoM.
    pub lever_arm_m: f64,
    /// [Normalized ESC command, combined thrust N], strictly increasing.
    /// May be a provisional assumed curve in dry runs.
    /// Starts at the minimum stable running command; force may be zero at that endpoint.
    pub thrust_curve: Vec<[f64; 2]>,
    pub gimbal_sign: [f64; 2],
    pub hover_height_m: f64,
}
impl Model {
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.mass_kg.is_finite() && self.mass_kg > 0.0,
            "invalid mass"
        );
        ensure!(
            self.inertia_kg_m2.iter().all(|v| v.is_finite() && *v > 0.0),
            "supply positive inertia_kg_m2"
        );
        ensure!(
            self.lever_arm_m.is_finite() && self.lever_arm_m > 0.0,
            "supply positive lever_arm_m"
        );
        ensure!(
            self.hover_height_m.is_finite() && self.hover_height_m > 0.0,
            "invalid hover height"
        );
        ensure!(
            self.gimbal_sign.iter().all(|v| *v == 1.0 || *v == -1.0),
            "gimbal_sign must be +/-1"
        );
        ensure!(
            self.thrust_curve.len() >= 2,
            "supply a combined thrust curve in [ESC command, N]"
        );
        ensure!(
            self.thrust_curve
                .iter()
                .all(|p| p.iter().all(|v| v.is_finite())
                    && p[0] >= 0.15
                    && p[0] <= 1.0
                    && p[1] >= 0.0),
            "invalid running thrust curve"
        );
        ensure!(
            self.thrust_curve
                .windows(2)
                .all(|w| w[0][0] < w[1][0] && w[0][1] < w[1][1]),
            "thrust curve must increase strictly"
        );
        let weight = self.mass_kg * 9.81;
        ensure!(
            self.thrust_curve[0][1] < weight && self.thrust_curve.last().unwrap()[1] > weight,
            "configured thrust range must bracket hover force"
        );
        Ok(())
    }
    pub fn throttle(&self, force: f64) -> Result<f32> {
        ensure!(
            force.is_finite()
                && force >= self.thrust_curve[0][1] - 1e-6
                && force <= self.thrust_curve.last().unwrap()[1] + 1e-6,
            "force outside calibration"
        );
        for w in self.thrust_curve.windows(2) {
            if force <= w[1][1] {
                let t = ((force - w[0][1]) / (w[1][1] - w[0][1])).clamp(0.0, 1.0);
                return Ok((w[0][0] + t * (w[1][0] - w[0][0])) as f32);
            }
        }
        Ok(self.thrust_curve.last().unwrap()[0] as f32)
    }
}
