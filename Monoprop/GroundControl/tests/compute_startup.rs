use std::{
    io::{BufRead, BufReader},
    net::{SocketAddr, UdpSocket},
    process::{Child, Command, Stdio},
    sync::mpsc,
    time::{Duration, Instant},
};
struct Running(Child);
impl Drop for Running {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}
#[test]
fn queued_compute_waits_for_telemetry_and_sends_nothing() {
    let peer = UdpSocket::bind("127.0.0.1:0").unwrap();
    peer.set_read_timeout(Some(Duration::from_millis(100)))
        .unwrap();
    let mut child = Running(
        Command::new(env!("CARGO_BIN_EXE_GroundControl"))
            .args([
                "--compute",
                "--bind",
                "127.0.0.1:0",
                "--target",
                &peer.local_addr().unwrap().to_string(),
                "--config",
                concat!(env!("CARGO_MANIFEST_DIR"), "/ground.toml"),
            ])
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .unwrap(),
    );
    let out = child.0.stdout.take().unwrap();
    let (tx, rx) = mpsc::channel();
    std::thread::spawn(move || {
        for line in BufReader::new(out).lines() {
            if tx.send(line.unwrap()).is_err() {
                break;
            }
        }
    });
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut destination: Option<SocketAddr> = None;
    loop {
        let line = rx
            .recv_timeout(deadline.saturating_duration_since(Instant::now()))
            .unwrap();
        if line.starts_with("Listening ") {
            destination = Some(
                line.split_whitespace()
                    .nth(1)
                    .unwrap()
                    .trim_end_matches(';')
                    .parse()
                    .unwrap(),
            );
        }
        assert!(!line.contains("MPC started"));
        if line.starts_with("Waiting for telemetry:") {
            assert!(line.contains("compute_pending=true"));
            break;
        }
    }
    let mut telemetry = monoprop_common::TelemetryFrame::default();
    telemetry.sensor_data.timestamp_us = 100_000;
    telemetry.vehicle_state.position_m = [0., 0., 1.];
    telemetry.vehicle_state.attitude_quat = [1., 0., 0., 0.];
    telemetry.vehicle_state.mass_kg = 1.05;
    peer.send_to(
        &postcard::to_stdvec_cobs(&telemetry).unwrap(),
        destination.unwrap(),
    )
    .unwrap();
    loop {
        let line = rx
            .recv_timeout(deadline.saturating_duration_since(Instant::now()))
            .unwrap();
        if line.contains("compute-only MPC started") {
            assert!(line.contains("transmission=false"));
            break;
        }
    }
    let mut packet = [0u8; 1024];
    assert!(matches!(
        peer.recv_from(&mut packet).unwrap_err().kind(),
        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
    ));
}
#[test]
fn automatic_compute_cannot_enable_live_transmission() {
    let output = Command::new(env!("CARGO_BIN_EXE_GroundControl"))
        .args(["--compute", "--send"])
        .output()
        .unwrap();
    assert!(!output.status.success());
    assert!(
        String::from_utf8_lossy(&output.stderr).contains("--compute requires compute-only mode")
    );
}

#[test]
fn live_ready_checkout_uses_zero_thrust_without_gps() {
    use std::io::Write;
    let peer = UdpSocket::bind("127.0.0.1:0").unwrap();
    peer.set_read_timeout(Some(Duration::from_millis(50)))
        .unwrap();
    let mut child = Running(
        Command::new(env!("CARGO_BIN_EXE_GroundControl"))
            .args([
                "--send",
                "--bind",
                "127.0.0.1:0",
                "--target",
                &peer.local_addr().unwrap().to_string(),
                "--config",
                concat!(env!("CARGO_MANIFEST_DIR"), "/ground.toml"),
            ])
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::null())
            .spawn()
            .unwrap(),
    );
    let out = child.0.stdout.take().unwrap();
    let (tx, rx) = mpsc::channel();
    std::thread::spawn(move || {
        for line in BufReader::new(out).lines() {
            if tx.send(line.unwrap()).is_err() {
                break;
            }
        }
    });
    let line = rx.recv_timeout(Duration::from_secs(3)).unwrap();
    assert!(line.contains("transmission=true"));
    let destination: SocketAddr = line
        .split_whitespace()
        .nth(1)
        .unwrap()
        .trim_end_matches(';')
        .parse()
        .unwrap();
    let mut t = monoprop_common::TelemetryFrame::default();
    let mut bytes = [0u8; 1024];
    // A fake peer verifies that Ready can be requested with no position fix.
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        assert!(Instant::now() < deadline);
        t.sensor_data.timestamp_us += 20_000;
        peer.send_to(&postcard::to_stdvec_cobs(&t).unwrap(), destination)
            .unwrap();
        writeln!(child.0.stdin.as_mut().unwrap(), "ready").unwrap();
        if let Ok((n, _)) = peer.recv_from(&mut bytes) {
            assert_eq!(
                postcard::from_bytes_cobs::<monoprop_common::TextCommand>(&mut bytes[..n]).unwrap(),
                monoprop_common::TextCommand::Ready
            );
            break;
        }
    }
    t.phase = monoprop_common::FlightPhase::Ready;
    loop {
        assert!(Instant::now() < deadline);
        t.sensor_data.timestamp_us += 20_000;
        peer.send_to(&postcard::to_stdvec_cobs(&t).unwrap(), destination)
            .unwrap();
        writeln!(child.0.stdin.as_mut().unwrap(), "tvc 1 -1").unwrap();
        if let Ok((n, _)) = peer.recv_from(&mut bytes) {
            if let Ok(c) =
                postcard::from_bytes_cobs::<monoprop_common::ControlCommand>(&mut bytes[..n])
            {
                assert_eq!(c.target_phase, monoprop_common::FlightPhase::Ready);
                assert_eq!(c.actuator_command.thrust_norm, 0.0);
                assert_eq!(c.actuator_command.body_rate_cmd_rps, [0.0; 3]);
                assert!((c.actuator_command.gimbal_rad[0] - 1.0f32.to_radians()).abs() < 1e-6);
                break;
            }
        }
    }
}

#[test]
fn live_descent_sends_throttle_without_gps_and_stops_on_stale_navigation() {
    use std::io::Write;
    let peer = UdpSocket::bind("127.0.0.1:0").unwrap();
    peer.set_read_timeout(Some(Duration::from_millis(40)))
        .unwrap();
    let mut child = Running(
        Command::new(env!("CARGO_BIN_EXE_GroundControl"))
            .args([
                "--send",
                "--bind",
                "127.0.0.1:0",
                "--target",
                &peer.local_addr().unwrap().to_string(),
                "--config",
                concat!(env!("CARGO_MANIFEST_DIR"), "/ground.toml"),
            ])
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .unwrap(),
    );
    let mut reader = BufReader::new(child.0.stdout.take().unwrap());
    let mut line = String::new();
    reader.read_line(&mut line).unwrap();
    let destination: SocketAddr = line
        .split_whitespace()
        .nth(1)
        .unwrap()
        .trim_end_matches(';')
        .parse()
        .unwrap();
    // Drain stdout throughout the test, so logging cannot block the process.
    std::thread::spawn(move || for _ in reader.lines() {});
    let mut t = monoprop_common::TelemetryFrame::default();
    t.phase = monoprop_common::FlightPhase::Descent;
    t.vehicle_state.position_m = [0.1, -0.1, 1.0];
    t.vehicle_state.mass_kg = 1.05;
    assert!(t.sensor_data.gps_data.is_none());
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut bytes = [0u8; 1024];
    let mut count = 0;
    while count < 5 {
        assert!(Instant::now() < deadline);
        t.sensor_data.timestamp_us += 20_000;
        peer.send_to(&postcard::to_stdvec_cobs(&t).unwrap(), destination)
            .unwrap();
        if count == 0 {
            writeln!(child.0.stdin.as_mut().unwrap(), "go").unwrap();
        }
        if let Ok((n, _)) = peer.recv_from(&mut bytes) {
            let c: monoprop_common::ControlCommand =
                postcard::from_bytes_cobs(&mut bytes[..n]).unwrap();
            assert_eq!(c.target_phase, t.phase);
            assert!(c.actuator_command.thrust_norm > 0.15);
            assert!(c.actuator_command.thrust_norm <= 1.0);
            assert!(
                c.actuator_command.gimbal_rad[0].hypot(c.actuator_command.gimbal_rad[1])
                    <= monoprop_common::MAX_TVC_GIMBAL_RAD + 1e-6
            );
            count += 1;
        }
    }
    std::thread::sleep(Duration::from_millis(250));
    while peer.recv_from(&mut bytes).is_ok() {}
    // New telemetry does not silently restart an interrupted flight stream.
    for _ in 0..5 {
        t.sensor_data.timestamp_us += 20_000;
        peer.send_to(&postcard::to_stdvec_cobs(&t).unwrap(), destination)
            .unwrap();
        assert!(peer.recv_from(&mut bytes).is_err());
    }
}
