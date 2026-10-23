use anyhow::{Result, bail, ensure};
use monoprop_common::{
    FlightPhase, LegacyDroneTelemetry, LegacyGroundCommand, TelemetryFrame, TextCommand,
};
use serde::Serialize;
use std::time::Instant;

#[derive(Clone, Debug)]
pub struct Sample {
    pub nav: LegacyDroneTelemetry,
    pub modern: Option<TelemetryFrame>,
    pub received: Instant,
}

impl Sample {
    pub fn phase(&self) -> Option<FlightPhase> {
        self.modern.map(|t| t.phase)
    }

    pub fn state(&self) -> [f64; 13] {
        let mut x = [0.0; 13];
        for i in 0..3 {
            x[i] = self.nav.position[i] as f64;
            x[7 + i] = self.nav.velocity[i] as f64;
            x[10 + i] = self.nav.angular_vel[i] as f64;
        }
        let norm = self
            .nav
            .orientation
            .iter()
            .map(|q| (*q as f64).powi(2))
            .sum::<f64>()
            .sqrt();
        for i in 0..4 {
            x[3 + i] = self.nav.orientation[i] as f64 / norm;
        }
        x
    }
}

pub fn encode<T: Serialize>(value: &T) -> Result<Vec<u8>> {
    let mut buffer = [0u8; 256];
    Ok(postcard::to_slice_cobs(value, &mut buffer)?.to_vec())
}

/// ESP32 delimiter mode sends complete frames including the zero.
/// Never join fragments across UDP loss.
pub fn decode_datagram(bytes: &[u8], now: Instant) -> Result<Vec<Sample>> {
    ensure!(
        !bytes.is_empty() && bytes.len() <= 1024 && bytes.last() == Some(&0),
        "incomplete or oversized COBS datagram"
    );
    let mut samples = Vec::new();
    for frame in bytes.split_inclusive(|b| *b == 0).filter(|f| f.len() > 1) {
        let mut scratch = frame.to_vec();
        let modern = postcard::take_from_bytes_cobs::<TelemetryFrame>(&mut scratch)
            .ok()
            .filter(|(_, rest)| rest.is_empty())
            .map(|(t, _)| t);
        let nav = if let Some(t) = modern {
            t.as_legacy()
        } else {
            scratch.copy_from_slice(frame);
            let (nav, rest) = postcard::take_from_bytes_cobs::<LegacyDroneTelemetry>(&mut scratch)?;
            ensure!(rest.is_empty(), "trailing telemetry bytes");
            nav
        };
        ensure!(
            nav.timestamp_us > 0
                && nav
                    .orientation
                    .iter()
                    .chain(&nav.position)
                    .chain(&nav.velocity)
                    .chain(&nav.angular_vel)
                    .all(|v| v.is_finite()),
            "invalid navigation telemetry"
        );
        let norm2 = nav.orientation.iter().map(|q| q * q).sum::<f32>();
        ensure!(
            (0.81..=1.21).contains(&norm2),
            "invalid navigation quaternion"
        );
        samples.push(Sample {
            nav,
            modern,
            received: now,
        });
    }
    ensure!(!samples.is_empty(), "empty telemetry");
    Ok(samples)
}

pub fn text_command(input: &str) -> Result<TextCommand> {
    Ok(match input.trim().to_ascii_lowercase().as_str() {
        "ready" => TextCommand::Ready,
        "arm" => TextCommand::Arm,
        "disarm" => TextCommand::Disarm,
        "launch" | "go" => TextCommand::Launch,
        "hover" => TextCommand::Hover,
        "land" | "descent" => TextCommand::Land,
        "abort" => TextCommand::Abort,
        "kill" => TextCommand::Kill,
        "mission standard" | "mission full" | "standard" | "mission_standard" => {
            TextCommand::MissionStandard
        }
        "mission hover" | "mission tethered" | "tethered" | "hover_only" | "mission_hover" => {
            TextCommand::MissionTetheredHover
        }
        _ => bail!("unknown command; type help"),
    })
}

pub fn validate_horizon(cmd: &LegacyGroundCommand) -> Result<()> {
    ensure!(
        cmd.is_valid()
            && cmd.thrust_seq.iter().all(|v| (0.0..=1.0).contains(v))
            && cmd
                .gimbal_theta_seq
                .iter()
                .chain(&cmd.gimbal_phi_seq)
                .all(|v| v.abs() <= monoprop_common::MAX_TVC_GIMBAL_RAD + 1e-6),
        "invalid normalized control horizon"
    );
    Ok(())
}
