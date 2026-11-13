//! Ideal plant propagation for offline diagnostics only; no hardware I/O.
use crate::{config::Model, mpc_crate};
use anyhow::{Result, ensure};
use monoprop_common::{ActuatorCommand, TelemetryFrame};
use ndarray::Array1;

pub fn force(model: &Model, throttle: f32) -> Result<f64> {
    let u = throttle as f64;
    ensure!(
        u.is_finite() && (0.0..=1.0).contains(&u),
        "invalid throttle"
    );
    if u == 0.0 {
        return Ok(0.0);
    }
    for w in model.thrust_curve.windows(2) {
        if u <= w[1][0] + 1e-6 {
            let fraction = ((u - w[0][0]) / (w[1][0] - w[0][0])).clamp(0.0, 1.0);
            return Ok(w[0][1] + fraction * (w[1][1] - w[0][1]));
        }
    }
    anyhow::bail!("throttle outside modeled curve")
}
pub fn advance(
    model: &Model,
    t: &mut TelemetryFrame,
    command: ActuatorCommand,
    dt: f64,
) -> Result<()> {
    let l = t.as_legacy();
    let x = Array1::from_iter(
        l.position
            .iter()
            .chain(l.orientation.iter())
            .chain(l.velocity.iter())
            .chain(l.angular_vel.iter())
            .map(|v| *v as f64),
    );
    let u = Array1::from_vec(vec![
        command.gimbal_rad[0] as f64 / model.gimbal_sign[0],
        command.gimbal_rad[1] as f64 / model.gimbal_sign[1],
        force(model, command.thrust_norm)?,
    ]);
    let next = mpc_crate::dynamics_step(model, &x, &u, dt);
    ensure!(
        next.iter().all(|v| v.is_finite()),
        "plant produced nonfinite state"
    );
    for j in 0..3 {
        t.vehicle_state.position_m[j] = next[j] as f32;
        t.vehicle_state.velocity_mps[j] = next[7 + j] as f32;
        t.vehicle_state.angular_velocity_rps[j] = next[10 + j] as f32;
    }
    t.vehicle_state.attitude_quat = [
        next[6] as f32,
        next[3] as f32,
        next[4] as f32,
        next[5] as f32,
    ];
    t.sensor_data.timestamp_us += (dt * 1e6).round() as u64;
    Ok(())
}

pub use crate::trajectory::ReturnReference;
