//! Nonlinear, box-constrained MPC. Solver variables are scaled gimbal angles and
//! force/max_force; only the final packet uses the ESC calibration curve.
use crate::{config::Vehicle, protocol::validate_horizon};
use anyhow::{Result, ensure};
use monoprop_common::{DEFAULT_MPC_STEP_DT_S, LegacyGroundCommand, MPC_HORIZON_STEPS};
use optimization_engine::{
    Optimizer, Problem, SolverError,
    constraints::Rectangle,
    panoc::{PANOCCache, PANOCOptimizer},
};
use std::{cell::RefCell, time::Duration};

pub const N: usize = MPC_HORIZON_STEPS;
pub const DT: f64 = DEFAULT_MPC_STEP_DT_S as f64;
pub type State = [f64; 13];
pub type References = [State; N + 1];

pub struct Mpc {
    vehicle: Vehicle,
    warm: [f64; N * 3],
    last_us: Option<u64>,
    cache: PANOCCache,
}

#[derive(Debug)]
pub struct Solution {
    pub command: LegacyGroundCommand,
    pub status: String,
    pub cost: f64,
    pub elapsed_ms: f64,
}

impl Mpc {
    pub fn new(vehicle: Vehicle) -> Self {
        let mut warm = [0.0; N * 3];
        for k in 0..N {
            warm[3 * k + 2] = vehicle.mass_kg * 9.81 / vehicle.max_thrust_n;
        }
        Self {
            vehicle,
            warm,
            last_us: None,
            cache: PANOCCache::new(N * 3, 1e-4, 10),
        }
    }

    pub fn solve(
        &mut self,
        x: &State,
        refs: &References,
        force_refs: &[f64; N],
        timestamp_us: u64,
    ) -> Result<Solution> {
        ensure!(
            x.iter()
                .chain(refs.iter().flatten())
                .chain(force_refs)
                .all(|x| x.is_finite()),
            "non-finite MPC input"
        );
        if let Some(previous) = self.last_us {
            let shift = timestamp_us.saturating_sub(previous) as f64 * 1e-6 / DT;
            let old = self.warm;
            for k in 0..N {
                let t = (k as f64 + shift).min((N - 1) as f64);
                let i = t.floor() as usize;
                let j = (i + 1).min(N - 1);
                for axis in 0..3 {
                    self.warm[k * 3 + axis] = old[i * 3 + axis]
                        + (t - i as f64) * (old[j * 3 + axis] - old[i * 3 + axis]);
                }
            }
        }
        let vehicle = &self.vehicle;
        let objective = |u: &[f64]| cost(vehicle, x, u, refs, force_refs);
        let initial_cost = objective(&self.warm);
        ensure!(initial_cost.is_finite(), "non-finite warm-start prediction");
        let best = RefCell::new((initial_cost, self.warm));
        let feasible = |u: &[f64]| {
            u.as_chunks::<3>().0.iter().all(|c| {
                c.iter().all(|v| v.is_finite())
                    && c[0].abs() <= 1.0
                    && c[1].abs() <= 1.0
                    && c[2] >= vehicle.min_thrust_n / vehicle.max_thrust_n
                    && c[2] <= 1.0
            })
        };
        let cost_fn = |u: &[f64], value: &mut f64| {
            *value = objective(u);
            if value.is_finite() {
                let mut candidate = best.borrow_mut();
                if *value < candidate.0 && feasible(u) {
                    candidate.0 = *value;
                    candidate.1.copy_from_slice(u);
                }
                Ok(())
            } else {
                Err(SolverError::NotFiniteComputation)
            }
        };
        let gradient = |u: &[f64], out: &mut [f64]| {
            let mut trial = [0.0; N * 3];
            trial.copy_from_slice(u);
            for i in 0..N * 3 {
                let epsilon = 1e-5;
                trial[i] = u[i] + epsilon;
                let plus = objective(&trial);
                trial[i] = u[i] - epsilon;
                let minus = objective(&trial);
                trial[i] = u[i];
                out[i] = (plus - minus) / (2.0 * epsilon);
            }
            if out.iter().all(|v| v.is_finite()) {
                Ok(())
            } else {
                Err(SolverError::NotFiniteComputation)
            }
        };
        let mut lower = [-1.0; N * 3];
        let upper = [1.0; N * 3];
        for k in 0..N {
            lower[3 * k + 2] = vehicle.min_thrust_n / vehicle.max_thrust_n;
        }
        let bounds = Rectangle::new(Some(&lower), Some(&upper));
        let problem = Problem::new(&bounds, gradient, cost_fn);
        let mut controls = self.warm;
        let status = PANOCOptimizer::new(problem, &mut self.cache)
            .with_max_iter(150)
            .with_max_duration(Duration::from_millis(vehicle.mpc_budget_ms))
            .solve(&mut controls)
            .map_err(|e| anyhow::anyhow!("MPC: {e:?}"))?;
        let mut final_cost = objective(&controls);
        let mut status_label = format!("{:?}", status.exit_status());
        let best = best.into_inner();
        // PANOC is nonmonotone. Its final step need not be its best feasible
        // step, especially when stopped by the real-time budget.
        if !final_cost.is_finite() || final_cost > best.0 || !feasible(&controls) {
            final_cost = best.0;
            controls = best.1;
            status_label = format!("BestFeasibleIterate ({status_label})");
        }
        ensure!(
            final_cost <= initial_cost + 1e-6,
            "MPC cost exceeded warm start"
        );
        ensure!(
            controls
                .iter()
                .zip(lower)
                .zip(upper)
                .all(|((&u, lo), hi)| u.is_finite() && u >= lo - 1e-8 && u <= hi + 1e-8),
            "MPC infeasible output"
        );
        let mut command = LegacyGroundCommand::default();
        for k in 0..N {
            command.thrust_seq[k] = vehicle.throttle(
                (controls[3 * k + 2] * vehicle.max_thrust_n)
                    .clamp(vehicle.min_thrust_n, vehicle.max_thrust_n),
            )?;
            command.gimbal_theta_seq[k] =
                (controls[3 * k] * vehicle.gimbal_limit_deg.to_radians()) as f32;
            command.gimbal_phi_seq[k] =
                (controls[3 * k + 1] * vehicle.gimbal_limit_deg.to_radians()) as f32;
        }
        validate_horizon(&command)?;
        self.warm = controls;
        self.last_us = Some(timestamp_us);
        Ok(Solution {
            command,
            status: status_label,
            cost: final_cost,
            elapsed_ms: status.solve_time().as_secs_f64() * 1000.0,
        })
    }
}

pub fn setpoint(target: [f64; 3]) -> References {
    let mut x = [0.0; 13];
    x[..3].copy_from_slice(&target);
    x[6] = 1.0;
    [x; N + 1]
}

fn cost(v: &Vehicle, initial: &State, u: &[f64], refs: &References, force_refs: &[f64; N]) -> f64 {
    let weights = [
        20.0, 20.0, 80.0, 200.0, 200.0, 20.0, 0.0, 5.0, 5.0, 20.0, 3.0, 3.0, 0.5,
    ];
    let mut x = *initial;
    let mut value = 0.0;
    for k in 0..N {
        x = dynamics(v, &x, &[u[k * 3], u[k * 3 + 1], u[k * 3 + 2]], DT);
        // Quaternion q and -q represent the same attitude.
        let sign = if (3..7).map(|j| x[j] * refs[k + 1][j]).sum::<f64>() < 0.0 {
            -1.0
        } else {
            1.0
        };
        for j in 0..13 {
            let actual = if (3..7).contains(&j) {
                sign * x[j]
            } else {
                x[j]
            };
            value +=
                weights[j] * (actual - refs[k + 1][j]).powi(2) * if k == N - 1 { 3.0 } else { 1.0 };
        }
        value += 0.1 * (u[k * 3 + 2] - force_refs[k] / v.max_thrust_n).powi(2)
            + 0.01 * (u[k * 3].powi(2) + u[k * 3 + 1].powi(2));
        if k > 0 {
            for j in 0..3 {
                value += 0.05 * (u[k * 3 + j] - u[(k - 1) * 3 + j]).powi(2);
            }
        }
    }
    value
}

/// Rigid body model with quaternion [x,y,z,w], body angular rates and world
/// velocity. RK4 substeps avoid Euler instability at prototype-scale inertia.
pub fn dynamics(v: &Vehicle, state: &State, control: &[f64; 3], dt: f64) -> State {
    let steps = (dt / 0.02).ceil().max(1.0) as usize;
    let h = dt / steps as f64;
    let mut x = *state;
    for _ in 0..steps {
        let a = derivative(v, &x, control);
        let b = derivative(v, &std::array::from_fn(|j| x[j] + 0.5 * h * a[j]), control);
        let c = derivative(v, &std::array::from_fn(|j| x[j] + 0.5 * h * b[j]), control);
        let d = derivative(v, &std::array::from_fn(|j| x[j] + h * c[j]), control);
        for j in 0..13 {
            x[j] += h / 6.0 * (a[j] + 2.0 * b[j] + 2.0 * c[j] + d[j]);
        }
        let norm = (3..7).map(|j| x[j] * x[j]).sum::<f64>().sqrt().max(1e-12);
        for q in &mut x[3..7] {
            *q /= norm;
        }
    }
    x
}

fn derivative(v: &Vehicle, x: &State, u: &[f64; 3]) -> State {
    let theta = u[0] * v.gimbal_limit_deg.to_radians();
    let phi = u[1] * v.gimbal_limit_deg.to_radians();
    let thrust = u[2] * v.max_thrust_n;
    let f = [
        thrust * theta.sin() * phi.cos(),
        thrust * phi.sin(),
        thrust * theta.cos() * phi.cos(),
    ];
    let [qx, qy, qz, qw] = [x[3], x[4], x[5], x[6]];
    let rotated = [
        (1.0 - 2.0 * (qy * qy + qz * qz)) * f[0]
            + 2.0 * (qx * qy - qz * qw) * f[1]
            + 2.0 * (qx * qz + qy * qw) * f[2],
        2.0 * (qx * qy + qz * qw) * f[0]
            + (1.0 - 2.0 * (qx * qx + qz * qz)) * f[1]
            + 2.0 * (qy * qz - qx * qw) * f[2],
        2.0 * (qx * qz - qy * qw) * f[0]
            + 2.0 * (qy * qz + qx * qw) * f[1]
            + (1.0 - 2.0 * (qx * qx + qy * qy)) * f[2],
    ];
    let [wx, wy, wz] = [x[10], x[11], x[12]];
    let [ix, iy, iz] = v.inertia_kg_m2;
    [
        x[7],
        x[8],
        x[9],
        0.5 * (qw * wx + qy * wz - qz * wy),
        0.5 * (qw * wy + qz * wx - qx * wz),
        0.5 * (qw * wz + qx * wy - qy * wx),
        -0.5 * (qx * wx + qy * wy + qz * wz),
        rotated[0] / v.mass_kg,
        rotated[1] / v.mass_kg,
        rotated[2] / v.mass_kg - 9.81,
        (-v.thrust_pivot_z_m * f[1] - (iz - iy) * wy * wz) / ix,
        (v.thrust_pivot_z_m * f[0] - (ix - iz) * wz * wx) / iy,
        -(iy - ix) * wx * wy / iz,
    ]
}
