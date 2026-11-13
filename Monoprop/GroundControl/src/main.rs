mod checkout;
mod config;
mod controller;
mod mpc_crate;
mod trajectory;

use anyhow::{Context, Result, bail};
use crossbeam_channel::bounded;
use monoprop_common::{FlightPhase, TelemetryFrame, TextCommand};
use std::{
    io::{self, BufRead},
    net::{SocketAddr, UdpSocket},
    thread,
    time::{Duration, Instant},
};

const FRESH: Duration = Duration::from_millis(150);
fn main() -> Result<()> {
    let mut target: Option<SocketAddr> = None;
    let mut bind = "0.0.0.0:9999".to_string();
    let mut config_path = "ground.toml".to_string();
    let mut telemetry_mirror: Option<SocketAddr> = None;
    let mut live = false;
    let mut observe = false;
    let mut compute_on_telemetry = false;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--target" => target = Some(args.next().context("--target IP:8888")?.parse()?),
            "--bind" => bind = args.next().context("--bind IP:9999")?,
            "--config" => config_path = args.next().context("--config path")?,
            "--telemetry-mirror" => {
                telemetry_mirror = Some(
                    args.next()
                        .context("--telemetry-mirror 127.0.0.1:10000")?
                        .parse()?,
                )
            }
            "--send" => live = true,
            "--observe" => observe = true,
            "--compute" => compute_on_telemetry = true,
            "--help" => {
                println!(
                    "GroundControl --target ESP32_IP:8888 [--bind 0.0.0.0:9999] [--config ground.toml] [--send | --observe | --compute] [--telemetry-mirror 127.0.0.1:10000]\nDefault: receive and compute only, no datagrams transmitted. --observe reads telemetry without requiring calibration.\n--compute waits for telemetry then begins compute-only MPC automatically; it cannot be combined with --send.
Commands: go, stop, ready, disarm, tvc X_DEG Y_DEG, arm, launch, hover, land, kill, reset-ekf, quit.
Live tvc commands require Ready and always request zero thrust; MPC uses telemetry position and velocity; fresh navigation and a successful solve are required.\nlaunch selects tethered hover and enables the MPC after Armed telemetry; go enables an existing flight.\nstop/quit stop the stream; firmware timeout applies. kill also sends the firmware Kill command."
                );
                return Ok(());
            }
            _ => bail!("unknown argument {arg}"),
        }
    }
    anyhow::ensure!(
        !(compute_on_telemetry && (live || observe)),
        "--compute requires compute-only mode; conflicts with --send and --observe"
    );
    let target = target.context("supply current ESP32 address with --target IP:8888")?;
    let model: config::Model = toml::from_str(&std::fs::read_to_string(config_path)?)?;
    anyhow::ensure!(!(observe && live), "--observe and --send conflict");
    if !observe {
        model.validate()?;
    }
    let mut mpc = controller::Controller::new(model.clone());
    let socket = UdpSocket::bind(&bind).with_context(|| {
        format!("bind {bind}; close the existing receiver first (no process will be killed)")
    })?;
    socket.set_nonblocking(true)?;
    let mirror = if let Some(address) = telemetry_mirror {
        anyhow::ensure!(
            address.ip().is_loopback() && address.port() != 0,
            "telemetry mirror must use a local loopback address and nonzero port"
        );
        let sender = UdpSocket::bind(if address.is_ipv4() {
            "127.0.0.1:0"
        } else {
            "[::1]:0"
        })?;
        sender.set_nonblocking(true)?;
        Some((sender, address))
    } else {
        None
    };

    let (tx, rx) = bounded(16);
    thread::spawn(move || {
        for line in io::stdin().lock().lines() {
            match line {
                Ok(l) => {
                    if tx.send(l).is_err() {
                        break;
                    }
                }
                Err(_) => break,
            }
        }
        let _ = tx.send("stop".into());
    });
    println!(
        "Listening {}; ESP32 {target}; transmission={live}. MPC inactive. Type go to enable, or ready/arm/launch for a new flight.",
        socket.local_addr()?
    );
    println!("MPC uses telemetry position/velocity; GPS availability does not gate commands");
    if let Some((_, address)) = &mirror {
        println!("Telemetry copy -> {address} (Postcard/COBS, receive-only viewer)");
    }
    let mut active = false;
    let mut compute_pending = compute_on_telemetry;
    if compute_pending {
        println!(
            "Compute-only MPC queued: waiting for the first valid fresh telemetry packet. No commands will be transmitted."
        );
    }
    let mut checkout_angles: Option<[f32; 2]> = None;
    let mut pending_phase: Option<(FlightPhase, Instant)> = None;
    let mut latest: Option<(TelemetryFrame, Instant)> = None;
    let mut last_timestamp = None;
    let mut sequence = 1u32;
    let mut last_report = Instant::now();
    let mut buffer = [0u8; 65535];
    let mut received = 0u64;
    let mut rejected = 0u64;
    let mut wrong_peer = 0u64;
    let mut last_packet = None;
    let mut last_decode_error = String::new();
    loop {
        let start = Instant::now();
        let mut fresh_packet = false;
        for _ in 0..64 {
            let (len, source) = match socket.recv_from(&mut buffer) {
                Ok(v) => v,
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(e) => return Err(e.into()),
            };
            received = received.saturating_add(1);
            last_packet = Some((source, len));
            if source.ip() != target.ip() {
                wrong_peer = wrong_peer.saturating_add(1);
                continue;
            }
            let t = match decode(&buffer[..len]) {
                Ok(t) => t,
                Err(error) => {
                    rejected = rejected.saturating_add(1);
                    last_decode_error = error.to_string();
                    continue;
                }
            };
            if let Some((sender, address)) = &mirror {
                // Best effort copy: closing or slowing the viewer cannot block MPC.
                let _ = sender.send_to(&buffer[..len], address);
            }
            let stamp = t.sensor_data.timestamp_us;
            if last_timestamp == Some(stamp) {
                continue;
            }
            if last_timestamp.is_some_and(|last| stamp < last) {
                active = false;
                checkout_angles = None;
                compute_pending = false;
                pending_phase = None;
                mpc = controller::Controller::new(model.clone());
                eprintln!(
                    "Drone timestamp moved backwards: restart/reordered packet; controller deactivated."
                );
            }
            last_timestamp = Some(stamp);
            if pending_phase.is_some_and(|(phase, _)| phase == t.phase) {
                pending_phase = None;
            }
            latest = Some((t, Instant::now()));
            fresh_packet = true;
        }
        for input in rx.try_iter() {
            let input = input.trim().to_lowercase();
            if input == "quit" {
                return Ok(());
            }
            if observe {
                eprintln!("Observe only; type quit to exit");
                continue;
            }
            if input == "stop" || input == "kill" {
                active = false;
                checkout_angles = None;
                compute_pending = false;
                pending_phase = None;
                mpc = controller::Controller::new(model.clone());
                if input == "kill" && live {
                    send_text(&socket, target, TextCommand::Kill)?;
                }
                println!(
                    "Controller stopped; {}",
                    if input == "kill" && live {
                        "Kill submitted (verify telemetry)"
                    } else {
                        "firmware command timeout remains in effect"
                    }
                );
                continue;
            }
            if input == "go" && !live {
                mpc = controller::Controller::new(model.clone());
                active = false;
                compute_pending = true;
                println!("Compute-only MPC queued; awaiting fresh telemetry");
                continue;
            }
            let Some((t, at)) = latest.as_ref().filter(|(_, at)| at.elapsed() < FRESH) else {
                eprintln!("No fresh telemetry; command rejected");
                continue;
            };
            if input.starts_with("tvc ") {
                match checkout::parse_angles(&input)
                    .and_then(|angles| checkout::command(t.phase, sequence, angles).map(|_| angles))
                {
                    Ok(angles) => {
                        active = false;
                        compute_pending = false;
                        checkout_angles = Some(angles);
                        println!(
                            "TVC checkout requested: gimbal_rad={angles:?}, thrust=0, transmission={live}"
                        );
                    }
                    Err(error) => eprintln!("Checkout rejected: {error}"),
                }
                continue;
            }
            let phase_command = match input.as_str() {
                "go" if t.phase.is_flying() || !live => {
                    mpc = controller::Controller::new(model.clone());
                    active = true;
                    None
                }
                "reset-ekf" if t.phase == FlightPhase::Standby => {
                    active = false;
                    compute_pending = false;
                    checkout_angles = None;
                    pending_phase = None;
                    mpc = controller::Controller::new(model.clone());
                    if live {
                        send_text(&socket, target, TextCommand::ResetEkf)?;
                    }
                    println!("ResetEkf submitted={live}; wait for fresh navigation alignment");
                    continue;
                }
                "ready" if t.phase == FlightPhase::Standby => Some(TextCommand::Ready),
                "disarm" if matches!(t.phase, FlightPhase::Ready | FlightPhase::Armed) => {
                    active = false;
                    compute_pending = false;
                    checkout_angles = None;
                    Some(TextCommand::Disarm)
                }
                "arm" if t.phase == FlightPhase::Ready => Some(TextCommand::Arm),
                "launch" if t.phase == FlightPhase::Armed => {
                    // Preflight the solve before requesting a flight phase.
                    let mut launch_state = *t;
                    launch_state.phase = FlightPhase::Hover;
                    if let Err(e) = mpc.solve(&launch_state, sequence) {
                        eprintln!("Launch rejected: {e}");
                        continue;
                    }
                    if at.elapsed() >= FRESH {
                        eprintln!("Launch rejected: solver/telemetry deadline");
                        continue;
                    }
                    if live {
                        send_text(&socket, target, TextCommand::MissionTetheredHover)?;
                    }
                    active = true;
                    Some(TextCommand::Launch)
                }
                "hover" if t.phase.is_flying() && active => Some(TextCommand::Hover),
                "land" if t.phase.is_flying() && active => Some(TextCommand::Land),
                _ => {
                    eprintln!("Unknown command or invalid phase {:?}: {input}", t.phase);
                    continue;
                }
            };
            if let Some(command) = phase_command {
                if live {
                    let phase = match command {
                        TextCommand::Disarm => FlightPhase::Standby,
                        TextCommand::Ready => FlightPhase::Ready,
                        TextCommand::Arm => FlightPhase::Armed,
                        TextCommand::Launch | TextCommand::Hover => FlightPhase::Hover,
                        TextCommand::Land => FlightPhase::Descent,
                        _ => unreachable!(),
                    };
                    pending_phase = Some((phase, Instant::now()));
                }
                if live {
                    send_text(&socket, target, command)?;
                }
                println!(
                    "{command:?}: {}",
                    if live {
                        "submitted, awaiting telemetry"
                    } else {
                        "dry run only"
                    }
                );
            }
        }
        if compute_pending && fresh_packet {
            compute_pending = false;
            active = true;
            println!("Fresh telemetry received: compute-only MPC started; transmission=false");
        }
        if pending_phase.is_some_and(|(_, at)| at.elapsed() >= FRESH) {
            pending_phase = None;
            active = false;
            eprintln!("Phase transition not confirmed: controller stopped");
        }
        if let Some(angles) = checkout_angles {
            if let Some((t, _)) = latest
                .as_ref()
                .filter(|(t, at)| t.phase == FlightPhase::Ready && at.elapsed() < FRESH)
            {
                if pending_phase.is_none() {
                    let command = checkout::command(t.phase, sequence, angles)?;
                    if live {
                        socket.send_to(&postcard::to_stdvec_cobs(&command)?, target)?;
                    }
                    sequence = sequence.wrapping_add(1);
                }
            } else {
                checkout_angles = None;
                eprintln!("TVC checkout stopped: stale telemetry or phase changed");
            }
        }
        if active && pending_phase.is_none() {
            if let Some((t, at)) = latest.as_ref().filter(|(_, at)| at.elapsed() < FRESH) {
                if fresh_packet && (t.phase.is_flying() || !live) {
                    match mpc.solve(t, sequence) {
                        Ok(command) if at.elapsed() < FRESH => {
                            if live {
                                socket.send_to(&postcard::to_stdvec_cobs(&command)?, target)?;
                            }
                            sequence = sequence.wrapping_add(1);
                            if last_report.elapsed() >= Duration::from_secs(1) {
                                println!(
                                    "phase={:?} link={} sequence={} throttle={:.3} gimbal={:?} solve_ms={} sent={live}",
                                    t.phase,
                                    t.link_ok,
                                    command.sequence,
                                    command.actuator_command.thrust_norm,
                                    command.actuator_command.gimbal_rad,
                                    start.elapsed().as_millis()
                                );
                                last_report = Instant::now();
                            }
                        }
                        result => {
                            active = false;
                            eprintln!(
                                "Controller deactivated: solver failure or deadline: {result:?}"
                            );
                        }
                    }
                } else if matches!(t.phase, FlightPhase::Landed | FlightPhase::Standby) && live {
                    active = false;
                }
            } else {
                active = false;
                eprintln!("Telemetry stale: stopped stream; firmware timeout applies");
            }
        }
        if last_report.elapsed() >= Duration::from_secs(1) {
            if let Some((t, at)) = &latest {
                println!(
                    "phase={:?} link={} telemetry_age_ms={} active={active} udp_received={received} rejected={rejected} wrong_peer={wrong_peer}",
                    t.phase,
                    t.link_ok,
                    at.elapsed().as_millis()
                );
                if live {
                    println!(
                        "Actuator feedback: thrust={} gimbal_rad={:?} checkout_target={checkout_angles:?}",
                        t.actuator_command.thrust_norm, t.actuator_command.gimbal_rad
                    );
                }
            } else {
                println!(
                    "Waiting for telemetry: compute_pending={compute_pending} udp_received={received} rejected={rejected} wrong_peer={wrong_peer} last_packet={last_packet:?} last_error={last_decode_error:?}"
                );
            }
            last_report = Instant::now();
        }
        thread::sleep(Duration::from_millis(20).saturating_sub(start.elapsed()));
    }
}
fn send_text(socket: &UdpSocket, target: SocketAddr, command: TextCommand) -> Result<()> {
    socket.send_to(&postcard::to_stdvec_cobs(&command)?, target)?;
    Ok(())
}
fn decode(bytes: &[u8]) -> Result<TelemetryFrame> {
    anyhow::ensure!(
        bytes.len() <= 1024 && bytes.last() == Some(&0) && !bytes[..bytes.len() - 1].contains(&0),
        "invalid frame"
    );
    let mut bytes = bytes.to_vec();
    let (frame, rest) = postcard::take_from_bytes_cobs(&mut bytes)?;
    anyhow::ensure!(rest.is_empty(), "trailing data");
    Ok(frame)
}

#[cfg(test)]
mod tests;
