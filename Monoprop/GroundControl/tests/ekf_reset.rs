use ground_control::{
    config::Config,
    protocol::{self, Sample},
    station::Station,
};
use monoprop_common::{FlightPhase, TelemetryFrame, TextCommand};
use monoprop_teensy41_firmware::link::{CommandParser, LinkCommand};
use std::{
    path::Path,
    time::{Duration, Instant},
};

fn config() -> Config {
    Config::load(Path::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/ground.toml"
    )))
    .unwrap()
}

fn sample(phase: FlightPhase) -> Sample {
    let mut modern = TelemetryFrame {
        phase,
        ..Default::default()
    };
    modern.sensor_data.timestamp_us = 1_000_000;
    Sample {
        nav: modern.as_legacy(),
        modern: Some(modern),
        received: Instant::now(),
    }
}

#[test]
fn ekf_reset_reaches_teensy_in_enum_string_and_ascii_forms() {
    assert_eq!(
        protocol::text_command(" EKF RESET ").unwrap(),
        TextCommand::ResetEkf
    );
    assert!(protocol::text_command("reset").is_err()); // Ground-local reset stays local.
    for packet in [
        protocol::encode(&TextCommand::ResetEkf).unwrap(),
        protocol::encode(&"ekf reset").unwrap(),
        b"ekf reset\0".to_vec(),
        b"EKF RESET\r\n".to_vec(),
    ] {
        let mut parser = CommandParser::<256>::new();
        let messages: Vec<_> = packet
            .into_iter()
            .filter_map(|b| parser.ingest(b))
            .collect();
        assert_eq!(messages.len(), 1);
        assert!(matches!(
            messages[0],
            LinkCommand::Text(TextCommand::ResetEkf)
        ));
    }
}

#[test]
fn ekf_reset_preserves_existing_wire_command_numbers() {
    for (index, command) in [
        TextCommand::Ready,
        TextCommand::Arm,
        TextCommand::Disarm,
        TextCommand::Launch,
        TextCommand::Hover,
        TextCommand::Land,
        TextCommand::Abort,
        TextCommand::Kill,
        TextCommand::MissionStandard,
        TextCommand::MissionTetheredHover,
        TextCommand::ResetEkf,
    ]
    .into_iter()
    .enumerate()
    {
        let mut bytes = [0; 8];
        assert_eq!(
            postcard::to_slice(&command, &mut bytes).unwrap(),
            &[index as u8]
        );
    }
}

#[test]
fn ekf_reset_stops_solvers_and_invalidates_cached_work() {
    let config = config();
    let mut station = Station::default();
    let old = sample(FlightPhase::Standby);
    station.accept(old.clone());
    station.mpc_enabled = true;
    station.lossless_enabled = true;
    station.awaiting_phase = true;
    let generation = station.generation;
    assert!(station.prepare_ekf_reset(Instant::now(), &config));
    assert!(!station.mpc_enabled && !station.lossless_enabled && !station.awaiting_phase);
    assert!(station.latest.is_none());
    assert_ne!(station.generation, generation);
    assert!(!station.can_transmit(generation, &old, Instant::now(), &config));
}

#[test]
fn ekf_reset_rejects_known_unsafe_phase_but_allows_missing_or_stale_navigation() {
    let config = config();
    for phase in [
        FlightPhase::Ready,
        FlightPhase::Armed,
        FlightPhase::Ascent,
        FlightPhase::Hover,
        FlightPhase::Descent,
        FlightPhase::Landed,
    ] {
        let mut station = Station::default();
        station.accept(sample(phase));
        let generation = station.generation;
        assert!(!station.prepare_ekf_reset(Instant::now(), &config));
        assert_eq!(station.generation, generation);
        assert_eq!(station.latest.as_ref().unwrap().phase(), Some(phase));
    }
    assert!(Station::default().prepare_ekf_reset(Instant::now(), &config));
    let mut station = Station::default();
    station.accept(sample(FlightPhase::Hover));
    let stale = Instant::now() + Duration::from_millis(config.telemetry_timeout_ms + 1);
    assert!(station.prepare_ekf_reset(stale, &config)); // Onboard phase gate still applies.
}
