//! Runs the real ground executable against a localhost ESP32 bridge emulator.
//! No network packets leave loopback and no hardware is actuated.
use ground_control::protocol;
use monoprop_common::{ActuatorCommand, FlightPhase, ImuData, SensorData};
use monoprop_teensy41_firmware::{
    link::{CommandParser, LinkCommand},
    runtime::Runtime,
};
use std::{
    io::Write,
    net::{SocketAddr, UdpSocket},
    process::{Command, Stdio},
    thread,
    time::{Duration, Instant},
};

struct ChildGuard(std::process::Child);
impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

#[test]
fn ground_process_controls_through_udp_cobs_and_stops_after_kill() {
    let bridge = UdpSocket::bind("127.0.0.1:0").unwrap();
    bridge.set_nonblocking(true).unwrap();
    let directory =
        std::env::temp_dir().join(format!("monoprop-ground-udp-{}", std::process::id()));
    std::fs::create_dir_all(&directory).unwrap();
    let mut child = ChildGuard(
        Command::new(env!("CARGO_BIN_EXE_GroundControl"))
            .args([
                "--bind",
                "127.0.0.1:0",
                "--bridge",
                &bridge.local_addr().unwrap().to_string(),
            ])
            .current_dir(&directory)
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .unwrap(),
    );
    let mut input = child.0.stdin.take().unwrap();
    let mut runtime = Runtime::new();
    let sensor = |t| SensorData {
        timestamp_us: t,
        imu_data: Some(ImuData {
            accel_mps2: [0.0, 0.0, 9.80665],
            ..Default::default()
        }),
        lidar_range_m: Some(0.3),
        ..Default::default()
    };
    for tick in 1..=220 {
        runtime.step(sensor(tick * 10_000));
    }
    let mut parser = CommandParser::<256>::new();
    let started = Instant::now();
    let mut next_tick = started;
    let mut peer: Option<SocketAddr> = None;
    let mut sequence = 0u32;
    let mut stage = 0;
    let mut stage_at = started;
    let mut horizons = 0;
    let mut got_kill = false;
    let mut after_kill = 0;
    let mut sent_preview = false;
    loop {
        assert!(
            started.elapsed() < Duration::from_secs(15),
            "loopback timeout at stage {stage}, horizons {horizons}; see {}",
            directory.display()
        );
        if let Some(exit) = child.0.try_wait().unwrap() {
            panic!("ground exited early: {exit}");
        }
        let now_us = 2_200_000 + started.elapsed().as_micros() as u64;
        let now_s = now_us as f32 * 1e-6;
        let mut bytes = [0u8; 2048];
        while let Ok((len, sender)) = bridge.recv_from(&mut bytes) {
            peer = Some(sender);
            for &b in &bytes[..len] {
                if let Some(command) = parser.ingest(b) {
                    if matches!(command, LinkCommand::LegacyControl(_)) {
                        horizons += 1;
                        if got_kill {
                            after_kill += 1;
                        }
                    }
                    if matches!(
                        command,
                        LinkCommand::Text(monoprop_common::TextCommand::Kill)
                    ) {
                        got_kill = true;
                    }
                    runtime.handle_link_command(command, now_s);
                }
            }
        }
        if Instant::now() >= next_tick {
            let data = sensor(now_us);
            let step = runtime.step(data);
            if let Some(peer) = peer
                && let Some(telem) = runtime.telemetry(sequence, data, step.actuator_command)
            {
                bridge
                    .send_to(&protocol::encode(&telem).unwrap(), peer)
                    .unwrap();
                sequence = sequence.wrapping_add(1);
            }
            next_tick = Instant::now() + Duration::from_millis(10);
        }
        let phase = runtime.core().state().flight_phase;
        match stage {
            0 if peer.is_some() && started.elapsed() > Duration::from_millis(100) => {
                writeln!(input, "mission hover\nready").unwrap();
                stage = 1;
                stage_at = Instant::now();
            }
            1 if phase == FlightPhase::Ready && stage_at.elapsed() > Duration::from_millis(100) => {
                writeln!(input, "arm").unwrap();
                stage = 2;
                stage_at = Instant::now();
            }
            2 if phase == FlightPhase::Armed && stage_at.elapsed() > Duration::from_millis(100) => {
                // Preview must not stream or launch the vehicle.
                writeln!(input, "solve mpc").unwrap();
                sent_preview = true;
                stage = 3;
                stage_at = Instant::now();
            }
            3 if stage_at.elapsed() > Duration::from_millis(250) => {
                assert_eq!(horizons, 0);
                assert_eq!(phase, FlightPhase::Armed);
                writeln!(input, "mpc on\nlaunch").unwrap();
                stage = 4;
                stage_at = Instant::now();
            }
            4 if horizons >= 3 => {
                assert_eq!(phase, FlightPhase::Hover);
                assert!(runtime.core().state().last_command.thrust_norm > 0.0);
                writeln!(input, "kill").unwrap();
                stage = 5;
                stage_at = Instant::now();
            }
            5 if got_kill && stage_at.elapsed() > Duration::from_millis(300) => {
                assert_eq!(after_kill, 0);
                assert_eq!(
                    runtime.core().state().last_command,
                    ActuatorCommand::default()
                );
                writeln!(input, "quit").unwrap();
                break;
            }
            _ => {}
        }
        thread::sleep(Duration::from_millis(2));
    }
    let deadline = Instant::now() + Duration::from_secs(3);
    while child.0.try_wait().unwrap().is_none() {
        assert!(Instant::now() < deadline, "ground did not exit");
        thread::sleep(Duration::from_millis(10));
    }
    assert!(sent_preview);
    assert!(horizons >= 3);
}
