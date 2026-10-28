use ground_control::{
    config::Config,
    guidance,
    mpc_crate::{Mpc, N, dynamics, setpoint},
    protocol::{self, Sample},
    station::Station,
};
use monoprop_common::{
    ActuatorCommand, FlightPhase, ImuData, LegacyGroundCommand, SensorData, TelemetryFrame,
    TextCommand,
};
use monoprop_teensy41_firmware::{
    link::{CommandParser, LinkCommand, TelemetryFormat, encode_telemetry},
    runtime::Runtime,
};
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

fn sample(phase: FlightPhase, time: u64) -> Sample {
    let mut frame = TelemetryFrame {
        phase,
        ..Default::default()
    };
    frame.sensor_data.timestamp_us = time;
    Sample {
        nav: frame.as_legacy(),
        modern: Some(frame),
        received: Instant::now(),
    }
}

fn through_parser<T: serde::Serialize>(value: &T) -> LinkCommand {
    let mut parser = CommandParser::<256>::new();
    let packet = protocol::encode(value).unwrap();
    assert_eq!(packet.last(), Some(&0));
    let mut messages = packet.into_iter().filter_map(|b| parser.ingest(b));
    let result = messages.next().expect("prototype must parse ground bytes");
    assert!(messages.next().is_none());
    result
}

#[test]
fn all_operator_commands_reach_the_real_teensy_parser() {
    for (text, command) in [
        ("ready", TextCommand::Ready),
        ("arm", TextCommand::Arm),
        ("disarm", TextCommand::Disarm),
        ("launch", TextCommand::Launch),
        ("go", TextCommand::Launch),
        ("hover", TextCommand::Hover),
        ("land", TextCommand::Land),
        ("abort", TextCommand::Abort),
        ("kill", TextCommand::Kill),
        ("mission standard", TextCommand::MissionStandard),
        ("mission hover", TextCommand::MissionTetheredHover),
    ] {
        let parsed = protocol::text_command(text).unwrap();
        assert_eq!(parsed, command);
        match through_parser(&parsed) {
            LinkCommand::Text(actual) => assert_eq!(actual, command),
            _ => panic!("wrong packet type"),
        }
    }
}

#[test]
fn calibrated_horizon_reaches_teensy_with_ten_steps_and_correct_units() {
    let model = config().vehicle;
    let mut cmd = LegacyGroundCommand::default();
    for i in 0..N {
        cmd.thrust_seq[i] = model.throttle(i as f64 + 1.0).unwrap();
        cmd.gimbal_theta_seq[i] = i as f32 * 0.01;
        cmd.gimbal_phi_seq[i] = -(i as f32) * 0.01;
    }
    cmd.seq_id = 128;
    match through_parser(&cmd) {
        LinkCommand::LegacyControl(actual) => {
            assert_eq!(actual.seq_id, 128);
            assert_eq!(actual.thrust_seq, cmd.thrust_seq);
            let horizon = actual.to_default_mpc_sequence(2.0);
            let (i, applied) = horizon.command_at(2.42);
            assert_eq!(i, 4);
            assert!((applied.thrust_norm - 5.0 / 16.0).abs() < 1e-6);
        }
        _ => panic!("wrong packet type"),
    }
    assert!(model.throttle(f64::NAN).is_err());
    assert_eq!(model.throttle(16.0).unwrap(), 1.0);
    assert!(model.throttle(17.0).is_err());
}

#[test]
fn modern_and_legacy_frames_are_strictly_decoded() {
    let mut frame = sample(FlightPhase::Hover, 123_456).modern.unwrap();
    frame.vehicle_state.attitude_quat = [0.9238795, 0.38268343, 0.0, 0.0];
    frame.sensor_data.imu_data = Some(ImuData::default());
    frame.sensor_data.gps_data = Some([1.0, 2.0, 3.0]);
    frame.sensor_data.gps_velocity_mps = Some([0.0; 3]);
    frame.sensor_data.lidar_range_m = Some(0.3);
    frame.sensor_data.chamber_pressure_pa = Some(1.0);
    frame.sensor_data.tank_pressure_pa = Some(1.0);
    frame.sequence = u32::MAX;
    frame.sensor_data.timestamp_us = u64::MAX;
    for format in [TelemetryFormat::Legacy, TelemetryFormat::Modern] {
        let mut bytes = [0; 192];
        let size = encode_telemetry(&frame, format, &mut bytes).unwrap();
        assert!(size <= 192);
        let decoded = protocol::decode_datagram(&bytes[..size], Instant::now()).unwrap();
        assert_eq!(
            decoded[0].nav.orientation,
            [0.38268343, 0.0, 0.0, 0.9238795]
        );
        assert_eq!(
            decoded[0].modern.is_some(),
            format == TelemetryFormat::Modern
        );
        assert!(protocol::decode_datagram(&bytes[..size - 1], Instant::now()).is_err());
        let mut combined = bytes[..size].to_vec();
        combined.extend_from_slice(&bytes[..size]);
        assert_eq!(
            protocol::decode_datagram(&combined, Instant::now())
                .unwrap()
                .len(),
            2
        );
    }
    let mut bad = frame.as_legacy();
    bad.position[0] = f32::NAN;
    assert!(protocol::decode_datagram(&protocol::encode(&bad).unwrap(), Instant::now()).is_err());
    assert!(protocol::decode_datagram(&[0], Instant::now()).is_err());
}

#[test]
fn stale_duplicate_reordered_and_cancelled_solves_cannot_transmit() {
    let config = config();
    let mut station = Station::default();
    let s = sample(FlightPhase::Hover, 2_000_000);
    assert!(station.accept(s.clone()));
    station.mpc_enabled = true;
    let generation = station.generation;
    assert!(station.can_transmit(generation, &s, s.received, &config));
    let mut repeated = s.clone();
    repeated.received += Duration::from_secs(1);
    assert!(!station.accept(repeated));
    assert!(!station.can_transmit(generation, &s, s.received + Duration::from_secs(1), &config));
    let old = sample(FlightPhase::Hover, 1_900_000);
    assert!(!station.accept(old));
    station.stop();
    assert!(!station.can_transmit(generation, &s, s.received, &config));
    station.mpc_enabled = true;
    assert!(!station.can_transmit(generation, &s, s.received, &config));
    assert!(station.accept(sample(FlightPhase::Descent, 2_020_000)));
    assert_eq!(station.target(&config), config.landing_position_m);
    let generation = station.generation;
    assert!(station.accept(sample(FlightPhase::Landed, 2_040_000)));
    assert!(!station.mpc_enabled);
    assert!(!station.can_transmit(generation, &s, s.received, &config));
}

#[test]
fn hover_model_balances_gravity_and_mpc_corrects_altitude() {
    let model = config().vehicle;
    let initial = setpoint([0.0, 0.0, 1.0])[0];
    let hover = model.mass_kg * 9.81 / model.max_thrust_n;
    let predicted = dynamics(&model, &initial, &[0.0, 0.0, hover], 1.0);
    assert!((predicted[2] - 1.0).abs() < 1e-8);
    assert!(predicted[9].abs() < 1e-8);
    for (target, upward) in [(1.2, true), (0.8, false)] {
        let mut mpc = Mpc::new(model.clone());
        let solution = mpc
            .solve(
                &initial,
                &setpoint([0.0, 0.0, target]),
                &[model.mass_kg * 9.81; N],
                1_000_000,
            )
            .unwrap();
        protocol::validate_horizon(&solution.command).unwrap();
        let force_fraction = solution.command.thrust_seq[0] as f64;
        assert_eq!(force_fraction > hover, upward, "{solution:?}");
    }
}

#[test]
fn lossless_solves_ascent_and_descent_for_constant_mass_prototype() {
    let model = config().vehicle;
    for (height, target) in [(0.0, 1.0), (1.0, 0.07)] {
        let mut s = sample(FlightPhase::Ascent, 1_000_000);
        s.nav.position[2] = height;
        let plan = guidance::solve(&model, &s, [0.0, 0.0, target]).unwrap();
        assert!((plan.trajectory.positions.last().unwrap()[2] - target).abs() < 1e-4);
        assert!(
            plan.trajectory
                .masses
                .iter()
                .all(|m| (*m - model.mass_kg).abs() < 1e-4)
        );
        let (refs, forces) = plan.references(s.nav.timestamp_us + 500_000, &model);
        assert!(refs.iter().flatten().all(|v| v.is_finite()));
        assert!(
            forces
                .iter()
                .all(|f| *f >= 0.0 && *f <= model.max_thrust_n + 1e-3)
        );
        let mut mpc = Mpc::new(model.clone());
        let solution = mpc
            .solve(&s.state(), &refs, &forces, s.nav.timestamp_us)
            .unwrap();
        protocol::validate_horizon(&solution.command).unwrap();
    }
}

fn sensors(t: u64) -> SensorData {
    SensorData {
        timestamp_us: t,
        imu_data: Some(ImuData {
            accel_mps2: [0.0, 0.0, 9.80665],
            ..Default::default()
        }),
        lidar_range_m: Some(0.3),
        // Already projected into the launch frame, as supplied by the GPS driver.
        gps_data: Some([0.0; 3]),
        gps_velocity_mps: Some([0.0; 3]),
        ..Default::default()
    }
}

#[test]
fn sensors_navigation_startup_solver_and_kill_use_the_actual_runtime() {
    let mut runtime = Runtime::new();
    assert!(
        runtime
            .telemetry(0, sensors(0), ActuatorCommand::default())
            .is_none()
    );
    for tick in 1..=220 {
        runtime.step(sensors(tick * 10_000));
    }
    let telem = runtime
        .telemetry(1, sensors(2_200_000), ActuatorCommand::default())
        .unwrap();
    let decoded = protocol::decode_datagram(&protocol::encode(&telem).unwrap(), Instant::now())
        .unwrap()
        .remove(0);
    assert!(decoded.nav.position[2].abs() < 0.01);
    assert_eq!(decoded.nav.orientation, [0.0, 0.0, 0.0, 1.0]);
    let sensor_data = &decoded.modern.as_ref().unwrap().sensor_data;
    assert_eq!(sensor_data.gps_data, Some([0.0; 3]));
    assert_eq!(sensor_data.gps_velocity_mps, Some([0.0; 3]));
    for command in [
        TextCommand::MissionTetheredHover,
        TextCommand::Ready,
        TextCommand::Arm,
        TextCommand::Launch,
    ] {
        runtime.handle_link_command(through_parser(&command), 2.201);
    }
    assert_eq!(runtime.core().state().flight_phase, FlightPhase::Hover);
    let model = config().vehicle;
    let mut mpc = Mpc::new(model.clone());
    let result = mpc
        .solve(
            &decoded.state(),
            &setpoint([0.0, 0.0, 1.0]),
            &[model.mass_kg * 9.81; N],
            2_200_000,
        )
        .unwrap();
    runtime.handle_link_command(through_parser(&result.command), 2.202);
    let step = runtime.step(sensors(2_220_000));
    assert!(step.actuator_command.thrust_norm > 0.0 && step.actuator_command.thrust_norm <= 1.0);
    runtime.handle_link_command(through_parser(&TextCommand::Kill), 2.223);
    assert_eq!(
        runtime
            .step(sensors(2_240_000))
            .actuator_command
            .thrust_norm,
        0.0
    );
}

#[test]
fn provisional_mpc_model_recovers_small_position_errors_in_closed_loop() {
    let model = config().vehicle;
    let mut solver = Mpc::new(model.clone());
    let mut state = setpoint([0.2, -0.1, 0.5])[0];
    let reference = setpoint([0.0, 0.0, 1.0]);
    let limit = model.gimbal_limit_deg.to_radians();
    for k in 0..100 {
        let result = solver
            .solve(
                &state,
                &reference,
                &[model.mass_kg * 9.81; N],
                1_000_000 + k * 50_000,
            )
            .unwrap();
        let cmd = result.command;
        // Default linear curve maps ESC fraction directly to force/max_force.
        state = dynamics(
            &model,
            &state,
            &[
                cmd.gimbal_theta_seq[0] as f64 / limit,
                cmd.gimbal_phi_seq[0] as f64 / limit,
                cmd.thrust_seq[0] as f64,
            ],
            0.05,
        );
        assert!(state.iter().all(|v| v.is_finite()));
        assert!(
            state[2] > 0.0 && state[2] < 2.0,
            "altitude left test envelope: {state:?}"
        );
    }
    assert!((state[2] - 1.0).abs() < 0.05, "vertical error: {state:?}");
    assert!(state[0].hypot(state[1]) < 0.1, "lateral error: {state:?}");
}

#[test]
fn reference_targets_cross_the_prototypes_phase_thresholds() {
    let config = config();
    let limits = monoprop_common::FlightLimits::default();
    let mut station = Station::default();
    station.accept(sample(FlightPhase::Ascent, 1_000_000));
    assert!(station.target(&config)[2] > limits.hover_altitude_m as f64);
    station.accept(sample(FlightPhase::Hover, 1_020_000));
    assert_eq!(station.target(&config)[2], limits.hover_altitude_m as f64);
    station.accept(sample(FlightPhase::Descent, 1_040_000));
    assert!(station.target(&config)[2] < limits.land_altitude_m as f64);
}
