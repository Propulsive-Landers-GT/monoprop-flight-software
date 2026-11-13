//! Offline MPC rollout from an incoming telemetry snapshot toward the local origin.
//! Never sends packets or arms hardware. The phase override exists only in simulation.
#[path = "../src/config.rs"]
mod config;
#[path = "../src/controller.rs"]
mod controller;
#[path = "../src/mpc_crate.rs"]
mod mpc_crate;
mod support;
#[path = "../src/trajectory.rs"]
mod trajectory;
use anyhow::{Context, Result};
use monoprop_common::{FlightLimits, FlightPhase, TelemetryFrame};
use std::{
    io::{BufWriter, Write},
    time::Instant,
};
fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let packet = args
        .next()
        .context("usage: plan_return TELEMETRY.bin OUTPUT.csv")?;
    let output = args.next().context("provide output CSV path")?;
    let model: config::Model = toml::from_str(include_str!("../ground.toml"))?;
    model.validate()?;
    let mut bytes = std::fs::read(&packet)?;
    let mut t: TelemetryFrame = postcard::from_bytes_cobs(&mut bytes)?;
    println!(
        "INPUT packet={packet} sequence={} timestamp_us={} actual_phase={:?} position_m={:?} velocity_mps={:?}",
        t.sequence,
        t.sensor_data.timestamp_us,
        t.phase,
        t.vehicle_state.position_m,
        t.vehicle_state.velocity_mps
    );
    // Honor the request to solve as if in flight, without changing hardware state.
    t.phase = FlightPhase::Descent;
    let mut c = controller::Controller::new(model.clone());
    let reference = support::ReturnReference::new(&t);
    println!(
        "REFERENCE local origin, smooth duration_s={:.3}, holding initial yaw",
        reference.duration_s
    );
    let limits = FlightLimits::default();
    let mut csv = BufWriter::new(std::fs::File::create(&output)?);
    writeln!(
        csv,
        "time_s,x_m,y_m,z_m,vx_mps,vy_mps,vz_mps,tilt_deg,throttle_norm,gimbal_x_deg,gimbal_y_deg,thrust_n,solve_ms"
    )?;
    let mut peak_throttle = 0.0f32;
    let mut max_tilt = 0.0f64;
    let mut worst_ms = 0.0f64;
    let mut completed = 0;
    let mut result = "time_limit".to_string();
    for i in 0..((reference.duration_s + 20.0) / 0.02).ceil() as u32 {
        let time = i as f64 * 0.02;
        let v = t.vehicle_state;
        let q = v.attitude_quat;
        let tilt = (1.0 - 2.0 * ((q[1] as f64).powi(2) + (q[2] as f64).powi(2)))
            .clamp(-1.0, 1.0)
            .acos()
            .to_degrees();
        max_tilt = max_tilt.max(tilt);
        if tilt > limits.max_tilt_deg as f64 {
            result = format!("tilt_limit at {time:.2}s");
            break;
        }
        // Mirror the firmware's landed predicate; don't simulate continued thrust after it.
        if time >= 0.5
            && v.position_m[2] <= limits.land_altitude_m
            && v.velocity_mps[2].abs() < 0.25
        {
            result = format!(
                "landed at {time:.2}s; horizontal_error_m={:.4}",
                v.position_m[0].hypot(v.position_m[1])
            );
            break;
        }
        let started = Instant::now();
        let command = match c.solve(&t, i) {
            Ok(c) => c.actuator_command,
            Err(e) => {
                result = format!("solver_failure at {time:.2}s: {e}");
                break;
            }
        };
        let ms = started.elapsed().as_secs_f64() * 1000.0;
        worst_ms = worst_ms.max(ms);
        if ms >= 150.0 {
            result = format!("deadline at {time:.2}s");
            break;
        }
        peak_throttle = peak_throttle.max(command.thrust_norm);
        writeln!(
            csv,
            "{time:.4},{},{},{},{},{},{},{tilt:.5},{},{},{},{},{ms:.5}",
            v.position_m[0],
            v.position_m[1],
            v.position_m[2],
            v.velocity_mps[0],
            v.velocity_mps[1],
            v.velocity_mps[2],
            command.thrust_norm,
            command.gimbal_rad[0].to_degrees(),
            command.gimbal_rad[1].to_degrees(),
            support::force(&model, command.thrust_norm)?
        )?;
        support::advance(&model, &mut t, command, 0.02)?;
        completed += 1;
    }
    csv.flush()?;
    println!(
        "RESULT {result}; solves={completed}; terminal_position_m={:?}; terminal_velocity_mps={:?}; peak_throttle={peak_throttle:.4}; max_tilt_deg={max_tilt:.3}; worst_solve_ms={worst_ms:.3}; output={output}",
        t.vehicle_state.position_m, t.vehicle_state.velocity_mps
    );
    Ok(())
}
