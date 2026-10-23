//! Same Lossless solver as Lander, solved on a separate worker. A fixed bounded
//! duration supports electric vehicles (alpha=0) without the fuel-based time search.
use crate::{
    config::Vehicle,
    mpc_crate::{DT, N, References},
    protocol::Sample,
};
use anyhow::{Result, ensure};
use rust_lossless::{LosslessSolver, TrajectoryResult};

#[derive(Clone, Debug)]
pub struct Plan {
    pub start_us: u64,
    pub trajectory: TrajectoryResult,
}

pub fn solve(vehicle: &Vehicle, sample: &Sample, target: [f64; 3]) -> Result<Plan> {
    let mut solver = LosslessSolver {
        initial_position: sample.nav.position.map(f64::from),
        initial_velocity: sample.nav.velocity.map(f64::from),
        landing_point: target,
        dry_mass: vehicle.dry_mass_kg,
        fuel_mass: vehicle.mass_kg - vehicle.dry_mass_kg,
        alpha: vehicle.fuel_burn_kg_per_ns,
        lower_thrust_bound: vehicle.min_thrust_n,
        upper_thrust_bound: vehicle.max_thrust_n,
        tvc_range_rad: vehicle.gimbal_limit_deg.to_radians(),
        max_velocity: vehicle.guidance_max_velocity_mps,
        N: 30,
        delta_t: vehicle.guidance_duration_s / 30.0,
        ..Default::default()
    };
    let result = solver.solve_at_current_time();
    let trajectory = result
        .trajectory
        .ok_or_else(|| anyhow::anyhow!("Lossless failed: {:?}", result.metrics.last_status))?;
    ensure!(
        trajectory.positions.len() >= 2
            && trajectory.positions.len() == trajectory.velocities.len()
            && trajectory.masses.len() == trajectory.positions.len()
            && trajectory.thrusts.len() + 1 == trajectory.positions.len()
            && trajectory.time_of_flight_s.is_finite()
            && trajectory.time_of_flight_s > 0.0
            && trajectory
                .positions
                .iter()
                .flatten()
                .chain(trajectory.velocities.iter().flatten())
                .chain(trajectory.thrusts.iter().flatten())
                .all(|v| v.is_finite())
            && trajectory.masses.iter().all(|m| m.is_finite() && *m > 0.0),
        "invalid Lossless trajectory"
    );
    Ok(Plan {
        start_us: sample.nav.timestamp_us,
        trajectory,
    })
}

impl Plan {
    pub fn references(&self, timestamp_us: u64, vehicle: &Vehicle) -> (References, [f64; N]) {
        let t0 = timestamp_us.saturating_sub(self.start_us) as f64 * 1e-6;
        let trajectory = &self.trajectory;
        let step = trajectory.time_of_flight_s / (trajectory.positions.len() - 1) as f64;
        let mut refs = [[0.0; 13]; N + 1];
        let mut forces = [vehicle.mass_kg * 9.81; N];
        for k in 0..=N {
            let t = (t0 + k as f64 * DT).min(trajectory.time_of_flight_s);
            let index = (t / step).min((trajectory.positions.len() - 1) as f64);
            let i = index.floor() as usize;
            let j = (i + 1).min(trajectory.positions.len() - 1);
            let fraction = index - i as f64;
            refs[k][6] = 1.0;
            for axis in 0..3 {
                refs[k][axis] = trajectory.positions[i][axis]
                    + fraction * (trajectory.positions[j][axis] - trajectory.positions[i][axis]);
                refs[k][7 + axis] = trajectory.velocities[i][axis]
                    + fraction * (trajectory.velocities[j][axis] - trajectory.velocities[i][axis]);
            }
            if k < N && t < trajectory.time_of_flight_s {
                let idx = i.min(trajectory.thrusts.len() - 1);
                // Lossless thrusts are specific force (m/s²), not newtons.
                forces[k] = trajectory.thrusts[idx]
                    .iter()
                    .map(|a| a * a)
                    .sum::<f64>()
                    .sqrt()
                    * trajectory.masses[idx];
            }
        }
        (refs, forces)
    }

    pub fn active_at(&self, timestamp_us: u64) -> bool {
        timestamp_us >= self.start_us
            && (timestamp_us - self.start_us) as f64 * 1e-6 < self.trajectory.time_of_flight_s
    }
}
