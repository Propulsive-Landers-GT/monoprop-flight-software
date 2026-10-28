use anyhow::{Context, Result, ensure};
use crossbeam_channel::{bounded, unbounded};
use ground_control::{
    config::Config,
    guidance::{self, Plan},
    mpc_crate::{Mpc, N, setpoint},
    protocol::{self, Sample},
    station::Station,
};
use monoprop_common::{FlightPhase, TextCommand};
use serde_json::json;
use std::{
    fs::OpenOptions,
    io::{self, BufRead, Write},
    net::UdpSocket,
    path::PathBuf,
    sync::Arc,
    thread,
    time::{Duration, Instant},
};

const HELP: &str = "\
Vehicle commands: mission standard | mission hover | ready | arm | launch (go)
                  hover | land | disarm | abort | kill | ekf reset
Ground controls:  mpc on | mpc off | lossless on | lossless off
                  solve mpc | solve lossless | status | reset | quit | help
Startup: wait for modern navigation, mission hover, ready, arm, mpc on, launch.
'kill' sends the vehicle kill command and keeps monitoring. 'quit' only stops
ground streaming. 'reset' clears telemetry ordering after a vehicle reboot.
'ekf reset' requests onboard realignment in Standby and disables both solvers.
Solves run without transmission until 'mpc on' and confirmed flying telemetry.";

#[derive(Clone)]
struct Job {
    generation: u64,
    sample: Sample,
    target: [f64; 3],
    plan: Option<Arc<Plan>>,
    preview: bool,
}

fn main() -> Result<()> {
    let mut config_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("ground.toml");
    let mut bind_override = None;
    let mut bridge_override = None;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--config" => config_path = args.next().context("--config requires a path")?.into(),
            "--bind" => {
                bind_override = Some(args.next().context("--bind requires IP:port")?.parse()?)
            }
            "--bridge" => {
                bridge_override = Some(args.next().context("--bridge requires IP:port")?.parse()?)
            }
            "--help" | "-h" => {
                println!(
                    "GroundControl [--config PATH] [--bind IP:port] [--bridge IP:port]\n{HELP}"
                );
                return Ok(());
            }
            _ => anyhow::bail!("unknown argument {arg}"),
        }
    }
    let mut config = Config::load(&config_path)?;
    if let Some(bind) = bind_override {
        config.bind = bind;
    }
    if let Some(bridge) = bridge_override {
        config.bridge = bridge;
    }
    ensure!(
        !config.bridge.ip().is_unspecified() && config.bridge.port() != 0,
        "bridge must be a concrete address"
    );
    let socket = UdpSocket::bind(config.bind).with_context(|| {
        format!(
            "bind {}; close another ground instance if this port is busy",
            config.bind
        )
    })?;
    socket.set_nonblocking(true)?;
    let mut log = OpenOptions::new()
        .create(true)
        .append(true)
        .open(&config.log_path)?;
    let started = Instant::now();
    let mut record = |event: serde_json::Value| -> Result<()> {
        writeln!(
            log,
            "{}",
            json!({"elapsed_s": started.elapsed().as_secs_f64(), "event": event})
        )?;
        log.flush()?;
        Ok(())
    };
    record(
        json!({"type":"session", "config":config_path, "bind":config.bind.to_string(), "bridge":config.bridge.to_string()}),
    )?;
    // A lone COBS delimiter is ignored by the Teensy, and lets a dynamic-IP
    // ESP32 learn our IP without issuing an arm/ready/flight command.
    socket.send_to(&[0], config.bridge)?;
    println!(
        "Listening on {}; ESP32 {}; log {}",
        socket.local_addr()?,
        config.bridge,
        config.log_path
    );
    println!(
        "Model: {:.3} kg, {:.2} N maximum thrust. Defaults are provisional.\n{HELP}",
        config.vehicle.mass_kg, config.vehicle.max_thrust_n
    );

    let (input_tx, input_rx) = unbounded::<String>();
    thread::spawn(move || {
        for line in io::stdin().lock().lines() {
            match line {
                Ok(line) => {
                    if input_tx.send(line).is_err() {
                        return;
                    }
                }
                Err(_) => break,
            }
        }
        let _ = input_tx.send("quit".into());
    });
    let (mpc_tx, mpc_rx) = bounded::<Job>(1);
    let (mpc_result_tx, mpc_result_rx) = bounded(1);
    let model = config.vehicle.clone();
    thread::spawn(move || {
        let mut solver = Mpc::new(model.clone());
        let mut generation = None;
        while let Ok(job) = mpc_rx.recv() {
            if generation != Some(job.generation) {
                solver = Mpc::new(model.clone());
                generation = Some(job.generation);
            }
            let (refs, forces) = job
                .plan
                .as_ref()
                .filter(|p| p.active_at(job.sample.nav.timestamp_us))
                .map(|p| p.references(job.sample.nav.timestamp_us, &model))
                .unwrap_or_else(|| (setpoint(job.target), [model.mass_kg * 9.81; N]));
            let result = solver.solve(
                &job.sample.state(),
                &refs,
                &forces,
                job.sample.nav.timestamp_us,
            );
            if mpc_result_tx.send((job, result)).is_err() {
                break;
            }
        }
    });
    let (guidance_tx, guidance_rx) = bounded::<Job>(1);
    let (guidance_result_tx, guidance_result_rx) = bounded(1);
    let model = config.vehicle.clone();
    thread::spawn(move || {
        while let Ok(job) = guidance_rx.recv() {
            let result = guidance::solve(&model, &job.sample, job.target);
            if guidance_result_tx.send((job, result)).is_err() {
                break;
            }
        }
    });

    let mut station = Station::default();
    let mut plan: Option<(u64, Arc<Plan>)> = None;
    let mut mpc_busy = false;
    let mut guidance_busy = false;
    let mut last_mpc = started;
    let mut last_guidance = started;
    let mut last_status = started;
    let mut last_probe = started;
    let mut last_sent_us = None;
    let mut sequence = 0u32;
    let mut received = 0u64;
    let mut invalid = 0u64;
    let mut ignored_peer = 0u64;
    let mut reordered = 0u64;
    let mut sent = 0u64;
    let mut solve_status = String::from("idle");
    let mut pending: Option<(TextCommand, Instant)> = None;
    loop {
        let now = Instant::now();
        // Bounded drain keeps operator commands responsive under a packet flood.
        // Read a complete UDP datagram before enforcing the bridge's 1024-byte cap.
        // A smaller buffer returns WSAEMSGSIZE on Windows for an oversized packet.
        let mut buffer = [0u8; 65535];
        for _ in 0..256 {
            match socket.recv_from(&mut buffer) {
                Ok((len, peer)) if peer == config.bridge => {
                    match protocol::decode_datagram(&buffer[..len], now) {
                        Ok(samples) => {
                            for sample in samples {
                                if station.accept(sample) {
                                    received += 1;
                                    let sample = station.latest.as_ref().unwrap();
                                    record(
                                        json!({"type":"telemetry","navigation":sample.nav,"modern":sample.modern}),
                                    )?;
                                    if let Some((command, _)) = pending
                                        && confirms(command, sample.phase())
                                    {
                                        println!(
                                            "Vehicle phase confirmed: {:?}",
                                            sample.phase().unwrap()
                                        );
                                        pending = None;
                                        station.awaiting_phase = false;
                                    }
                                } else {
                                    reordered += 1;
                                }
                            }
                        }
                        Err(error) => {
                            invalid += 1;
                            if invalid == 1 {
                                eprintln!("Rejected telemetry: {error}");
                            }
                        }
                    }
                }
                Ok(_) => ignored_peer += 1,
                Err(error) if error.kind() == io::ErrorKind::WouldBlock => break,
                Err(error) => return Err(error.into()),
            }
        }
        if station.mpc_enabled && !station.fresh(now, &config) {
            station.stop();
            plan = None;
            eprintln!("Navigation stale: MPC transmission disabled; use 'mpc on' after recovery.");
            record(json!({"type":"control_disabled","reason":"stale_navigation"}))?;
        }
        // Operator input is processed before worker results, so kill/off cannot
        // be followed by a result already waiting in a channel.
        while let Ok(input) = input_rx.try_recv() {
            let input = input.trim().to_ascii_lowercase();
            if input.is_empty() {
                continue;
            }
            match input.as_str() {
                "help" => println!("{HELP}"),
                "status" => last_status = now - Duration::from_secs(2),
                "quit" | "exit" => {
                    station.stop();
                    record(json!({"type":"quit","sent":sent,"received":received}))?;
                    println!("Ground streaming stopped. Vehicle commands use kill/abort/land.");
                    return Ok(());
                }
                "reset" => {
                    station.reset();
                    plan = None;
                    last_sent_us = None;
                    pending = None;
                    println!("Telemetry ordering reset; streaming disabled.");
                }
                "mpc off" => {
                    station.stop();
                    plan = None;
                    println!("MPC transmission disabled.");
                }
                "mpc on" => {
                    if !station.fresh(now, &config)
                        || station.latest.as_ref().and_then(Sample::phase).is_none()
                    {
                        eprintln!(
                            "Need fresh modern navigation; legacy telemetry supports monitoring and previews only."
                        );
                    } else {
                        station.invalidate();
                        station.mpc_enabled = true;
                        station.awaiting_phase = false;
                        last_sent_us = None;
                        println!("MPC enabled; transmission waits for a confirmed flying phase.");
                    }
                }
                "lossless on" | "lossless off" => {
                    station.lossless_enabled = input == "lossless on";
                    station.invalidate();
                    plan = None;
                    println!("Lossless enabled: {}", station.lossless_enabled);
                }
                "solve mpc" | "solve lossless" => {
                    if !station.fresh(now, &config) {
                        eprintln!("Need fresh navigation for a preview.");
                        continue;
                    }
                    let job = make_job(&station, &config, None, true);
                    if input == "solve mpc" && !mpc_busy {
                        mpc_tx.send(job)?;
                        mpc_busy = true;
                    } else if input == "solve lossless" && !guidance_busy {
                        guidance_tx.send(job)?;
                        guidance_busy = true;
                    } else {
                        eprintln!("Solver already busy.");
                    }
                }
                _ => match protocol::text_command(&input) {
                    Ok(command) => {
                        if command == TextCommand::ResetEkf {
                            if !station.prepare_ekf_reset(now, &config) {
                                eprintln!(
                                    "EKF reset requires Standby; disarm first and wait for Standby telemetry."
                                );
                                continue;
                            }
                            plan = None;
                            last_sent_us = None;
                            pending = None;
                            socket.send_to(&protocol::encode(&command)?, config.bridge)?;
                            record(json!({"type":"command_sent","command":command}))?;
                            println!(
                                "EKF reset sent; MPC/Lossless disabled. Teensy requires Standby in Auto mode. Check Teensy USB for acceptance; keep stationary for alignment and GPS origin. UDP send is not an acknowledgement."
                            );
                            continue;
                        }
                        if command == TextCommand::Launch
                            && (!station.mpc_enabled
                                || !station.fresh(now, &config)
                                || station.latest.as_ref().and_then(Sample::phase)
                                    != Some(FlightPhase::Armed))
                        {
                            eprintln!("Launch requires fresh Armed telemetry and 'mpc on'.");
                            continue;
                        }
                        if matches!(
                            command,
                            TextCommand::Kill | TextCommand::Abort | TextCommand::Disarm
                        ) {
                            station.stop();
                            plan = None;
                        }
                        if matches!(
                            command,
                            TextCommand::Launch | TextCommand::Land | TextCommand::Hover
                        ) {
                            station.invalidate();
                            station.awaiting_phase = true;
                            plan = None;
                        }
                        socket.send_to(&protocol::encode(&command)?, config.bridge)?;
                        record(json!({"type":"command_sent","command":command}))?;
                        println!("Sent {command:?}; UDP send is not an acknowledgement.");
                        pending = Some((command, now));
                    }
                    Err(error) => eprintln!("{error}"),
                },
            }
        }
        if let Ok((job, result)) = guidance_result_rx.try_recv() {
            guidance_busy = false;
            match result {
                Ok(new_plan) => {
                    record(
                        json!({"type":"lossless","preview":job.preview,"source_us":new_plan.start_us,
                        "duration_s":new_plan.trajectory.time_of_flight_s,"positions":new_plan.trajectory.positions,
                        "velocities":new_plan.trajectory.velocities,"specific_force":new_plan.trajectory.thrusts}),
                    )?;
                    if job.preview {
                        println!(
                            "Lossless preview solved: {:.2} s trajectory; no transmission.",
                            new_plan.trajectory.time_of_flight_s
                        );
                    } else if job.generation == station.generation
                        && station.fresh(now, &config)
                        && new_plan.active_at(station.latest.as_ref().unwrap().nav.timestamp_us)
                    {
                        plan = Some((job.generation, Arc::new(new_plan)));
                    }
                }
                Err(error) => {
                    eprintln!(
                        "Lossless: {error}; MPC uses the target setpoint when no valid trajectory exists."
                    );
                    record(json!({"type":"lossless_error","error":error.to_string()}))?;
                }
            }
        }
        if let Ok((job, result)) = mpc_result_rx.try_recv() {
            mpc_busy = false;
            match result {
                Ok(mut solution) => {
                    solve_status = format!("{} {:.1} ms", solution.status, solution.elapsed_ms);
                    if job.preview {
                        println!(
                            "MPC preview: {solve_status}, throttle {:.3}; no transmission.",
                            solution.command.thrust_seq[0]
                        );
                    }
                    let transmit = !job.preview
                        && station.can_transmit(
                            job.generation,
                            &job.sample,
                            Instant::now(),
                            &config,
                        );
                    solution.command.seq_id = sequence;
                    if transmit {
                        socket.send_to(&protocol::encode(&solution.command)?, config.bridge)?;
                        sequence = sequence.wrapping_add(1);
                        sent += 1;
                    }
                    record(
                        json!({"type":"mpc","source_us":job.sample.nav.timestamp_us,"transmitted":transmit,
                        "status":solution.status,"elapsed_ms":solution.elapsed_ms,"cost":solution.cost,"command":solution.command}),
                    )?;
                }
                Err(error) => {
                    solve_status = error.to_string();
                    record(json!({"type":"mpc_error","error":error.to_string()}))?;
                }
            }
        }
        if station.mpc_enabled && !station.awaiting_phase && station.fresh(now, &config) {
            let sample = station.latest.as_ref().unwrap();
            let flying = sample.phase().is_some_and(|p| p.is_flying());
            if flying
                && !mpc_busy
                && now.duration_since(last_mpc).as_secs_f64() >= 1.0 / config.control_rate_hz
                && last_sent_us != Some(sample.nav.timestamp_us)
            {
                let active_plan = plan
                    .as_ref()
                    .filter(|(g, p)| {
                        *g == station.generation && p.active_at(sample.nav.timestamp_us)
                    })
                    .map(|(_, p)| p.clone());
                mpc_tx.send(make_job(&station, &config, active_plan, false))?;
                mpc_busy = true;
                last_sent_us = Some(sample.nav.timestamp_us);
                last_mpc = now;
            }
            if station.lossless_enabled
                && flying
                && sample.phase() != Some(FlightPhase::Hover)
                && !guidance_busy
                && now.duration_since(last_guidance).as_secs_f64() >= 1.0 / config.guidance_rate_hz
            {
                guidance_tx.send(make_job(&station, &config, None, false))?;
                guidance_busy = true;
                last_guidance = now;
            }
        }
        if pending.is_some_and(|(_, t)| now.duration_since(t) > Duration::from_secs(2)) {
            eprintln!(
                "No phase confirmation for {:?}; inspect status. Mission mode and termination have no explicit acknowledgement in this protocol.",
                pending.take().unwrap().0
            );
        }
        if now.duration_since(last_status) >= Duration::from_secs(1) {
            if let Some(s) = &station.latest {
                println!(
                    "nav={:?} age={}ms pos={:.3?} vel={:.3?} rx={received} tx={sent} bad={invalid} old={reordered} other={ignored_peer} mpc={} lossless={} solve={solve_status}",
                    s.phase(),
                    now.saturating_duration_since(s.received).as_millis(),
                    s.nav.position,
                    s.nav.velocity,
                    station.mpc_enabled,
                    station.lossless_enabled
                );
            } else {
                println!(
                    "Waiting for aligned navigation. Check BMI323/LiDAR, UART, Wi-Fi and ground IP. bad={invalid} other={ignored_peer}"
                );
            }
            last_status = now;
        }
        if now.duration_since(last_probe) >= Duration::from_secs(1) {
            socket.send_to(&[0], config.bridge)?;
            last_probe = now;
        }
        thread::sleep(Duration::from_millis(2));
    }
}

fn make_job(station: &Station, config: &Config, plan: Option<Arc<Plan>>, preview: bool) -> Job {
    Job {
        generation: station.generation,
        sample: station.latest.as_ref().unwrap().clone(),
        target: station.target(config),
        plan,
        preview,
    }
}

fn confirms(command: TextCommand, phase: Option<FlightPhase>) -> bool {
    match command {
        TextCommand::Ready => phase == Some(FlightPhase::Ready),
        TextCommand::Arm => phase == Some(FlightPhase::Armed),
        TextCommand::Disarm => phase == Some(FlightPhase::Standby),
        TextCommand::Launch => matches!(phase, Some(FlightPhase::Ascent | FlightPhase::Hover)),
        TextCommand::Hover => phase == Some(FlightPhase::Hover),
        TextCommand::Land => matches!(phase, Some(FlightPhase::Descent | FlightPhase::Landed)),
        _ => false,
    }
}
