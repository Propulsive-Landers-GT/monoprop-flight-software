//! Live TVC checkout uses the existing Ready gate and never requests motor thrust.
use anyhow::{Context, Result, ensure};
use monoprop_common::{ActuatorCommand, ControlCommand, FlightPhase, MAX_TVC_GIMBAL_RAD};
pub fn parse_angles(input: &str) -> Result<[f32; 2]> {
    let words: Vec<_> = input.split_whitespace().collect();
    ensure!(
        words.len() == 3 && words[0] == "tvc",
        "usage: tvc X_DEG Y_DEG"
    );
    let x: f32 = words[1].parse().context("invalid X angle")?;
    let y: f32 = words[2].parse().context("invalid Y angle")?;
    let angles = [x.to_radians(), y.to_radians()];
    ensure!(
        angles.iter().all(|v| v.is_finite()) && angles[0].hypot(angles[1]) <= MAX_TVC_GIMBAL_RAD,
        "TVC angles must be finite and inside the shared 15 degree circle"
    );
    Ok(angles)
}
pub fn command(phase: FlightPhase, sequence: u32, angles: [f32; 2]) -> Result<ControlCommand> {
    ensure!(
        phase == FlightPhase::Ready,
        "TVC checkout requires Ready telemetry"
    );
    ensure!(
        angles.iter().all(|v| v.is_finite()) && angles[0].hypot(angles[1]) <= MAX_TVC_GIMBAL_RAD,
        "invalid TVC angles"
    );
    Ok(ControlCommand {
        sequence,
        target_phase: FlightPhase::Ready,
        actuator_command: ActuatorCommand {
            gimbal_rad: angles,
            ..ActuatorCommand::default()
        },
    })
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn checkout_is_ready_only_zero_thrust_and_polar_limited() {
        let angles = parse_angles("tvc 1 -1").unwrap();
        let c = command(FlightPhase::Ready, 7, angles).unwrap();
        assert_eq!(c.actuator_command.thrust_norm, 0.0);
        assert_eq!(c.actuator_command.body_rate_cmd_rps, [0.0; 3]);
        for p in [
            FlightPhase::Standby,
            FlightPhase::Armed,
            FlightPhase::Hover,
            FlightPhase::Descent,
        ] {
            assert!(command(p, 1, angles).is_err());
        }
        assert!(parse_angles("tvc 15 15").is_err());
        assert!(parse_angles("tvc NaN 0").is_err());
        assert!(parse_angles("tvc 1 0 0.5").is_err());
    }
}
