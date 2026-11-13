//! Offline only: reuse the production controller and dynamics, never open a socket.
#[path = "../src/config.rs"]
mod config;
#[path = "../src/controller.rs"]
mod controller;
#[path = "../src/mpc_crate.rs"]
mod mpc_crate;
#[path = "../src/trajectory.rs"]
mod trajectory;
use monoprop_common::{FlightPhase, TelemetryFrame};
mod support;
use std::time::Instant;
fn main() -> anyhow::Result<()> {
    let model: config::Model = toml::from_str(include_str!("../ground.toml"))?;
    model.validate()?;
    let captured = std::env::args()
        .nth(1)
        .map(|path| -> anyhow::Result<TelemetryFrame> {
            let mut bytes = std::fs::read(path)?;
            Ok(postcard::from_bytes_cobs(&mut bytes)?)
        })
        .transpose()?;
    for (name, tilted, propagate) in [
        ("origin-held", false, false),
        ("tilted-held", true, false),
        ("origin-flight", false, true),
        ("tilted-flight", true, true),
        ("telemetry-held", false, false),
        ("telemetry-flight", false, true),
    ] {
        if name.starts_with("telemetry-") && captured.is_none() {
            continue;
        }
        let mut c = controller::Controller::new(model.clone());
        let mut t = TelemetryFrame::default();
        t.phase = FlightPhase::Hover;
        t.vehicle_state.mass_kg = model.mass_kg as f32;
        t.vehicle_state.attitude_quat = if tilted {
            [0.9997114, -0.010514061, -0.008098555, 0.02002437]
        } else {
            [1., 0., 0., 0.]
        };
        if name.starts_with("telemetry-") {
            t = captured.unwrap();
            t.phase = FlightPhase::Hover;
        }
        let mut full = 0;
        let mut worst = 0.0f64;
        let mut completed = 0;
        for i in 0..500 {
            let start = Instant::now();
            let cmd = match c.solve(&t, i) {
                Ok(c) => c,
                Err(e) => {
                    println!(
                        "{name}: FAILED at step={i}, time_s={:.2}: {e}",
                        i as f64 * 0.02
                    );
                    break;
                }
            };
            worst = worst.max(start.elapsed().as_secs_f64() * 1000.0);
            if cmd.actuator_command.thrust_norm >= 0.999 {
                full += 1;
            }
            completed += 1;
            if propagate {
                support::advance(&model, &mut t, cmd.actuator_command, 0.02)?;
            }
        }
        println!(
            "{name}: solves={completed}/500, full_throttle_steps={full}, worst_ms={worst:.3}, position={:?}, velocity={:?}",
            t.vehicle_state.position_m, t.vehicle_state.velocity_mps
        );
    }
    Ok(())
}
