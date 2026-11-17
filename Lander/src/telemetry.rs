//! Ground-station link: telemetry downlink and command uplink over UDP (`gs-protocol`).
//!
//! `GroundLink` is used by the flight binary (`main.rs`) and by the simulator's
//! `--ground-station` mode, so anything the GUI does against the sim runs the same code that flies.
//!
//! The command interlocks live here and in the FSM (see "Command rules enforced on the vehicle"
//! in `ground-station/docs/DESIGN.md`). The building of messages is kept in pure functions
//! (`build_flight_telemetry`, `build_stand_telemetry`, ...) separate from the socket code.

use std::io;

use gs_protocol::{
    AckResult, CommandKind, Downlink, EventMsg, FlightParams, FlightTelemetry, MpcWeights,
    ParamsMsg, SensorSnapshot, Severity, Source, StandChannel, StandTelemetry, TrajectoryMsg,
    TruthState, Uplink, ValveId, ValveState, ValveStatus, VehicleLink,
};

use crate::fsm::FlightStateMachine;
use crate::state::{ControlMode, FlightLimits, FlightPhase, SensorData};

const FLIGHT_RATE_HZ: f64 = 50.0;
const STAND_RATE_HZ: f64 = 20.0;

/// P&ID tag used as the key in `ControlLoopState::valve_overrides` for each protocol valve.
const VALVE_TAGS: [(ValveId, &str); 15] = [
    (ValveId::Omv, "OMV"),
    (ValveId::Mtv, "MTV"),
    (ValveId::IgV, "IG-V"),
    (ValveId::OFill, "O-FILL"),
    (ValveId::OIso, "O-ISO"),
    (ValveId::OVnt, "O-VNT"),
    (ValveId::PuMv, "PU-MV"),
    (ValveId::PuFill, "PU-FILL"),
    (ValveId::PuIso, "PU-ISO"),
    (ValveId::PuVnt, "PU-VNT"),
    (ValveId::PuMvnt, "PU-MVNT"),
    (ValveId::LfVnt, "LF-VNT"),
    (ValveId::TVnt, "T-VNT"),
    (ValveId::Rcs1, "RCS1"),
    (ValveId::Rcs2, "RCS2"),
];

pub fn valve_tag(id: ValveId) -> &'static str {
    VALVE_TAGS.iter().find(|(v, _)| *v == id).map(|(_, tag)| *tag).unwrap_or("?")
}

pub struct GroundLink {
    link: VehicleLink,
    source: Source,
    flight_seq: u32,
    next_flight_tx: f64,
    next_stand_tx: f64,
    /// `trajectory_generation_time` of the last trajectory sent; None = nothing / "cleared" sent.
    sent_trajectory_time: Option<f64>,
}

impl GroundLink {
    /// Binds the vehicle command port (`gs_protocol::DEFAULT_VEHICLE_PORT` = 8888 by convention).
    /// Telemetry starts flowing to whoever sends the first valid packet (the bridge heartbeats).
    pub fn bind(port: u16, source: Source) -> io::Result<Self> {
        Ok(Self {
            link: VehicleLink::bind(port)?,
            source,
            flight_seq: 0,
            next_flight_tx: f64::NEG_INFINITY,
            next_stand_tx: f64::NEG_INFINITY,
            sent_trajectory_time: None,
        })
    }

    pub fn source(&self) -> Source {
        self.source
    }

    pub fn link_age_s(&self) -> Option<f32> {
        self.link.link_age_s()
    }

    /// Drains the socket (non-blocking) and applies every pending command. Each command except
    /// Heartbeat / Jog is acked; `Params` is sent after any parameter change or on request.
    pub fn handle_commands(&mut self, fsm: &mut FlightStateMachine, now: f64) {
        while let Some(cmd) = self.link.poll() {
            let result = apply_command(fsm, &cmd, now);
            let send_params = result.is_ok()
                && matches!(
                    cmd.kind,
                    CommandKind::SetFlightParams(_) | CommandKind::SetMpcWeights(_) | CommandKind::RequestParams
                );
            let ack = match result {
                Ok(()) => AckResult::Accepted,
                Err(reason) => {
                    // Jog is not acked, and arrives at up to 50 Hz: do not spam the console with it.
                    if cmd.kind.wants_ack() {
                        println!("[ground link] rejected {:?}: {}", cmd.kind, reason);
                    }
                    AckResult::Rejected(reason)
                }
            };
            self.link.ack(&cmd, now, ack);
            if send_params {
                self.link.send(&Downlink::Params(build_params(fsm)));
            }
        }
    }

    /// Sends the current `Params` unprompted (e.g. after the sim rebuilt the vehicle in `--loop` mode).
    pub fn send_params(&mut self, fsm: &mut FlightStateMachine) {
        self.link.send(&Downlink::Params(build_params(fsm)));
    }

    /// Call every loop iteration. Sends `Flight` at 50 Hz and `Trajectory` whenever guidance
    /// regenerated it. Events go through `publish_events`, stand data through `publish_stand`.
    pub fn publish(&mut self, fsm: &FlightStateMachine, sensor_data: &SensorData, truth: Option<TruthState>, now: f64) {
        if rate_due(&mut self.next_flight_tx, 1.0 / FLIGHT_RATE_HZ, now) {
            self.send_flight(fsm, sensor_data, truth, now);
        }
        self.publish_trajectory(fsm);
    }

    /// Sends a `Flight` frame right now, ignoring the rate limit (used for the final frame
    /// before the flight loop exits, so the GUI sees the end state).
    pub fn send_flight(&mut self, fsm: &FlightStateMachine, sensor_data: &SensorData, truth: Option<TruthState>, now: f64) {
        let msg = build_flight_telemetry(fsm, sensor_data, truth, self.source, self.flight_seq, self.link.link_age_s(), now);
        self.flight_seq = self.flight_seq.wrapping_add(1);
        self.link.send(&Downlink::Flight(msg));
    }

    fn publish_trajectory(&mut self, fsm: &FlightStateMachine) {
        // Nothing is sent (or marked as sent) until a ground station is known, so the
        // pre-launch trajectory still reaches a ground station that connects later.
        if self.link.ground_addr().is_none() {
            return;
        }
        let state = fsm.get_state();
        match &state.trajectory_state {
            Some(trajectory) => {
                if self.sent_trajectory_time != Some(state.trajectory_generation_time) {
                    self.sent_trajectory_time = Some(state.trajectory_generation_time);
                    let target = trajectory.positions.last().copied().unwrap_or(fsm.goal());
                    self.link.send(&Downlink::Trajectory(TrajectoryMsg::from_positions(
                        state.trajectory_generation_time,
                        trajectory.time_of_flight_s,
                        &trajectory.positions,
                        target,
                    )));
                }
            }
            None => {
                // Guidance dropped the trajectory (Hover, Landed): tell the ground with an empty one.
                if self.sent_trajectory_time.is_some() {
                    self.sent_trajectory_time = None;
                    self.link.send(&Downlink::Trajectory(TrajectoryMsg::from_positions(
                        state.trajectory_generation_time,
                        0.0,
                        &[],
                        fsm.goal(),
                    )));
                }
            }
        }
    }

    /// FSM diagnostics -> `Event`s. The caller drains `diagnostics_queue` once and hands the same
    /// messages to the MCAP logger and to this.
    pub fn publish_events(&mut self, messages: &[String], now: f64) {
        for text in messages {
            self.link.send(&Downlink::Event(EventMsg {
                time_s: now,
                severity: event_severity(text),
                text: text.clone(),
            }));
        }
    }

    /// Stand telemetry passthrough, limited to 20 Hz. The flight binary feeds it from
    /// `build_stand_telemetry`, the sim from its propulsion model.
    pub fn publish_stand(&mut self, stand: StandTelemetry) {
        if rate_due(&mut self.next_stand_tx, 1.0 / STAND_RATE_HZ, stand.time_s) {
            self.link.send(&Downlink::Stand(stand));
        }
    }
}

/// Fixed-rate schedule: deadlines advance by `period` (not from "now"), so the average rate is
/// exact even when the caller's loop period does not divide it. Re-syncs after a stall, or
/// when the clock jumps backwards (the sim restarts its mission clock in `--loop` mode).
fn rate_due(next_due: &mut f64, period: f64, now: f64) -> bool {
    let clock_went_back = *next_due - now > period + 1e-6;
    if now < *next_due - 1e-6 && !clock_went_back {
        return false;
    }
    *next_due = if clock_went_back || now - *next_due > period { now + period } else { *next_due + period };
    true
}

/// Applies one uplink command to the FSM. Err carries the rejection reason for the ack.
pub fn apply_command(fsm: &mut FlightStateMachine, cmd: &Uplink, now: f64) -> Result<(), String> {
    let phase = fsm.get_state().flight_phase;
    let terminated = fsm.get_state().flight_terminated;
    let require_not_terminated = || if terminated { Err("flight terminated".to_string()) } else { Ok(()) };

    match &cmd.kind {
        CommandKind::Heartbeat | CommandKind::RequestParams => Ok(()),
        CommandKind::Arm => {
            require_not_terminated()?;
            if phase != FlightPhase::Standby {
                return Err(format!("Arm only allowed in Standby (currently {:?})", phase));
            }
            if fsm.control_mode() != ControlMode::Auto {
                return Err("Arm only allowed in control mode Auto (currently Jog)".to_string());
            }
            fsm.arm(now);
            Ok(())
        }
        CommandKind::Disarm => {
            require_not_terminated()?;
            if phase != FlightPhase::Armed {
                return Err(format!("Disarm only allowed when Armed (currently {:?})", phase));
            }
            fsm.disarm(now);
            Ok(())
        }
        CommandKind::Launch => {
            require_not_terminated()?;
            if phase != FlightPhase::Armed {
                return Err(format!("Launch only allowed when Armed (currently {:?})", phase));
            }
            fsm.launch(now);
            Ok(())
        }
        CommandKind::Abort => {
            fsm.abort(now);
            Ok(())
        }
        CommandKind::SetPhase(target) => fsm.command_phase(phase_from_protocol(*target), now),
        CommandKind::SetFlightParams(params) => fsm.set_flight_limits(limits_from_protocol(params)),
        CommandKind::SetMpcWeights(Some(weights)) => {
            fsm.set_mpc_weights(&weights.q.map(f64::from), &weights.r.map(f64::from), &weights.qn.map(f64::from))
        }
        CommandKind::SetMpcWeights(None) => fsm.clear_mpc_weights(),
        CommandKind::SetControlMode(mode) => fsm.set_control_mode(match mode {
            gs_protocol::ControlMode::Auto => ControlMode::Auto,
            gs_protocol::ControlMode::Jog => ControlMode::Jog,
        }),
        CommandKind::Jog(jog) => fsm.set_jog(
            jog.gimbal_theta as f64,
            jog.gimbal_phi as f64,
            jog.thrust as f64,
            jog.rcs as f64,
            now,
        ),
        CommandKind::SetValve { id, open } => {
            require_not_terminated()?;
            if phase != FlightPhase::Standby {
                return Err(format!("valve commands only allowed in Standby (currently {:?})", phase));
            }
            let state = fsm.get_state_mut();
            state.valve_overrides.insert(valve_tag(*id), *open);
            state.diagnostics_queue.push(format!("Valve {} commanded {}", valve_tag(*id), if *open { "OPEN" } else { "CLOSED" }));
            Ok(())
        }
        CommandKind::Stand(_) => Err("test-stand commands are handled by gs-stand, not the vehicle".to_string()),
    }
}

pub fn phase_to_protocol(phase: FlightPhase) -> gs_protocol::FlightPhase {
    match phase {
        FlightPhase::Standby => gs_protocol::FlightPhase::Standby,
        FlightPhase::Armed => gs_protocol::FlightPhase::Armed,
        FlightPhase::Ascent => gs_protocol::FlightPhase::Ascent,
        FlightPhase::Hover => gs_protocol::FlightPhase::Hover,
        FlightPhase::Descent => gs_protocol::FlightPhase::Descent,
        FlightPhase::Landed => gs_protocol::FlightPhase::Landed,
    }
}

pub fn phase_from_protocol(phase: gs_protocol::FlightPhase) -> FlightPhase {
    match phase {
        gs_protocol::FlightPhase::Standby => FlightPhase::Standby,
        gs_protocol::FlightPhase::Armed => FlightPhase::Armed,
        gs_protocol::FlightPhase::Ascent => FlightPhase::Ascent,
        gs_protocol::FlightPhase::Hover => FlightPhase::Hover,
        gs_protocol::FlightPhase::Descent => FlightPhase::Descent,
        gs_protocol::FlightPhase::Landed => FlightPhase::Landed,
    }
}

fn limits_from_protocol(params: &FlightParams) -> FlightLimits {
    FlightLimits {
        hover_altitude_m: params.hover_altitude_m as f64,
        hover_duration_s: params.hover_duration_s as f64,
        max_tilt_deg: params.max_tilt_deg as f64,
        max_trajectory_deviation_m: params.max_trajectory_deviation_m as f64,
    }
}

/// "terminated" / "Emergency" -> Critical, "Warning" -> Warning, everything else Info.
pub fn event_severity(text: &str) -> Severity {
    if text.contains("terminated") || text.contains("Emergency") {
        Severity::Critical
    } else if text.contains("Warning") {
        Severity::Warning
    } else {
        Severity::Info
    }
}

/// Takes `&mut` only because reading the MPC weights goes through the `mpc_mut()` downcast.
pub fn build_params(fsm: &mut FlightStateMachine) -> ParamsMsg {
    let limits = fsm.flight_limits();
    ParamsMsg {
        flight: FlightParams {
            hover_altitude_m: limits.hover_altitude_m as f32,
            hover_duration_s: limits.hover_duration_s as f32,
            max_tilt_deg: limits.max_tilt_deg as f32,
            max_trajectory_deviation_m: limits.max_trajectory_deviation_m as f32,
        },
        manual_mpc_weights: fsm.manual_mpc_weights().map(|(q, r, qn)| MpcWeights {
            q: q.map(|w| w as f32),
            r: r.map(|w| w as f32),
            qn: qn.map(|w| w as f32),
        }),
    }
}

fn vec3_f32(v: &nalgebra::Vector3<f64>) -> [f32; 3] {
    [v.x as f32, v.y as f32, v.z as f32]
}

fn arr3_f32(v: &[f64; 3]) -> [f32; 3] {
    [v[0] as f32, v[1] as f32, v[2] as f32]
}

/// Pure: everything in a `Flight` frame comes from the arguments.
pub fn build_flight_telemetry(
    fsm: &FlightStateMachine,
    sensor_data: &SensorData,
    truth: Option<TruthState>,
    source: Source,
    seq: u32,
    link_age_s: Option<f32>,
    now: f64,
) -> FlightTelemetry {
    let state = fsm.get_state();
    let vehicle = &state.vehicle_state;
    // nalgebra stores quaternion coordinates as [i, j, k, w], the protocol's [x, y, z, w] order.
    let q = vehicle.attitude.quaternion();
    let imu = sensor_data.imu_data.as_ref();

    FlightTelemetry {
        seq,
        time_s: now,
        source,
        phase: phase_to_protocol(state.flight_phase),
        phase_time_s: (now - state.last_state_time) as f32,
        control_mode: match state.control_mode {
            ControlMode::Auto => gs_protocol::ControlMode::Auto,
            ControlMode::Jog => gs_protocol::ControlMode::Jog,
        },
        terminated: state.flight_terminated,
        position: vec3_f32(&vehicle.position),
        velocity: vec3_f32(&vehicle.velocity),
        attitude: [q.i as f32, q.j as f32, q.k as f32, q.w as f32],
        angular_velocity: vec3_f32(&vehicle.angular_velocity),
        mass: state.mass as f32,
        gimbal_theta: state.last_gimbal_theta as f32,
        gimbal_phi: state.last_gimbal_phi as f32,
        thrust: state.last_thrust as f32,
        rcs: state.last_rcs_command as i8,
        tilt_deg: fsm.tilt_from_vertical().to_degrees() as f32,
        trajectory_deviation_m: fsm.trajectory_deviation().map(|d| d as f32),
        position_age_s: (now - state.last_position_update) as f32,
        link_age_s,
        sensors: SensorSnapshot {
            imu_ok: imu.is_some_and(|s| s.is_finite()),
            gps_ok: sensor_data.gps_data.is_some(),
            uwb_ok: sensor_data.uwb_data.is_some(),
            accel: imu.map(|d| arr3_f32(&d.accel)).unwrap_or([0.0; 3]),
            gyro: imu.map(|d| arr3_f32(&d.gyro)).unwrap_or([0.0; 3]),
            chamber_pressure: sensor_data.chamber_pressure.map(|p| p as f32),
            tank_pressure: sensor_data.tank_pressure.map(|p| p as f32),
        },
        truth,
    }
}

/// Valve list for stand telemetry from what the flight software knows: the operator valve
/// overrides plus RCS1 / RCS2 from the roll command (+1 opens RCS1, -1 opens RCS2).
pub fn build_valve_statuses(fsm: &FlightStateMachine) -> Vec<ValveStatus> {
    let state = fsm.get_state();
    let mut valves = Vec::new();
    for (id, tag) in VALVE_TAGS {
        let override_open = state.valve_overrides.get(tag).copied();
        let open = match id {
            ValveId::Rcs1 => Some(state.last_rcs_command > 0.5 || override_open == Some(true)),
            ValveId::Rcs2 => Some(state.last_rcs_command < -0.5 || override_open == Some(true)),
            _ => override_open,
        };
        if let Some(open) = open {
            valves.push(ValveStatus {
                id,
                state: if open { ValveState::Open } else { ValveState::Closed },
                position_deg: None,
            });
        }
    }
    valves
}

/// Minimal stand telemetry for the real vehicle, from what exists in `SensorData` today:
/// chamber_pressure -> E-PT, tank_pressure -> O-PT (both assumed to be in bar).
pub fn build_stand_telemetry(fsm: &FlightStateMachine, sensor_data: &SensorData, source: Source, now: f64) -> StandTelemetry {
    let mut channels = Vec::new();
    if let Some(p) = sensor_data.chamber_pressure {
        channels.push((StandChannel::Ept, p as f32));
    }
    if let Some(p) = sensor_data.tank_pressure {
        channels.push((StandChannel::Opt, p as f32));
    }
    StandTelemetry {
        time_s: now,
        source,
        channels,
        valves: build_valve_statuses(fsm),
        outputs_on: Vec::new(),
        mtv_percent: None,
    }
}
