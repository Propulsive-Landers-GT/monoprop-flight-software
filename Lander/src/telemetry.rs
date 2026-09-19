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
/// exact even when the caller's loop period does not divide it. Re-syncs after a stall.
fn rate_due(next_due: &mut f64, period: f64, now: f64) -> bool {
    if now < *next_due - 1e-6 {
        return false;
    }
    *next_due = if now - *next_due > period { now + period } else { *next_due + period };
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
            imu_ok: imu.is_some(),
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
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::ImuData;
    use gs_protocol::JogSetpoint;
    use nalgebra::{UnitQuaternion, Vector3};

    fn sensors(t: f64) -> SensorData {
        SensorData {
            timestamp: t,
            imu_data: Some(ImuData { accel: [0.1, 0.2, 9.81], gyro: [0.01, 0.02, 0.03], mag: [-2.0e-6, 22.0e-6, -44.3e-6] }), // the EKF's world field [T], level vehicle
            gps_data: Some([0.0, 0.0, 0.0]),
            uwb_data: Some([0.0, 0.0, 0.0]),
            chamber_pressure: Some(15.0),
            tank_pressure: Some(300.0),
        }
    }

    fn cmd(kind: CommandKind) -> Uplink {
        Uplink { seq: 1, kind }
    }

    fn build(fsm: &FlightStateMachine, s: &SensorData, now: f64) -> FlightTelemetry {
        build_flight_telemetry(fsm, s, None, Source::Vehicle, 3, Some(0.25), now)
    }

    #[test]
    fn rate_limiter_holds_average_rate() {
        // 400 Hz caller (2.5 ms loop), 50 Hz limit, 2 s
        let mut next_due = f64::NEG_INFINITY;
        let sent = (0..800).filter(|i| rate_due(&mut next_due, 0.02, *i as f64 * 0.0025)).count();
        assert_eq!(sent, 100);
        // After a stall it re-syncs instead of bursting
        assert!(rate_due(&mut next_due, 0.02, 10.0));
        assert!(!rate_due(&mut next_due, 0.02, 10.001));
    }

    #[test]
    fn phase_mapping_round_trips() {
        let phases = [
            (FlightPhase::Standby, gs_protocol::FlightPhase::Standby),
            (FlightPhase::Armed, gs_protocol::FlightPhase::Armed),
            (FlightPhase::Ascent, gs_protocol::FlightPhase::Ascent),
            (FlightPhase::Hover, gs_protocol::FlightPhase::Hover),
            (FlightPhase::Descent, gs_protocol::FlightPhase::Descent),
            (FlightPhase::Landed, gs_protocol::FlightPhase::Landed),
        ];
        for (ours, theirs) in phases {
            assert_eq!(phase_to_protocol(ours), theirs);
            assert_eq!(phase_from_protocol(theirs), ours);
        }
    }

    #[test]
    fn flight_telemetry_reflects_fsm_state() {
        let mut fsm = FlightStateMachine::new();
        fsm.set_flight_phase(FlightPhase::Hover, 10.0);
        {
            let state = fsm.get_state_mut();
            state.vehicle_state.position = Vector3::new(1.0, -2.0, 50.0);
            state.last_position_update = 11.5;
            state.last_thrust = 730.0;
            state.last_rcs_command = -1.0;
        }
        let msg = build(&fsm, &sensors(12.0), 12.0);
        assert_eq!(msg.seq, 3);
        assert_eq!(msg.phase, gs_protocol::FlightPhase::Hover);
        assert!((msg.phase_time_s - 2.0).abs() < 1e-6);
        assert_eq!(msg.control_mode, gs_protocol::ControlMode::Auto);
        assert_eq!(msg.position, [1.0, -2.0, 50.0]);
        assert!((msg.position_age_s - 0.5).abs() < 1e-6);
        assert_eq!(msg.thrust, 730.0);
        assert_eq!(msg.rcs, -1);
        assert_eq!(msg.link_age_s, Some(0.25));
        assert_eq!(msg.trajectory_deviation_m, None);
        assert!(msg.truth.is_none());
        assert!(!msg.terminated);
    }

    #[test]
    fn quaternion_order_is_xyzw_and_tilt_is_true_tilt() {
        let mut fsm = FlightStateMachine::new();
        // 20 deg pitch about Y: q = [0, sin(10 deg), 0, cos(10 deg)]
        let angle = 20.0_f64.to_radians();
        fsm.get_state_mut().vehicle_state.attitude = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), angle);
        let msg = build(&fsm, &sensors(0.0), 0.0);
        let expected = [0.0, (angle / 2.0).sin() as f32, 0.0, (angle / 2.0).cos() as f32];
        for i in 0..4 {
            assert!((msg.attitude[i] - expected[i]).abs() < 1e-6, "attitude[{}] = {}", i, msg.attitude[i]);
        }
        assert!((msg.tilt_deg - 20.0).abs() < 1e-3);
        // The termination check's angle (roll about X only) does not see a pure pitch at all.
        assert!(fsm.termination_tilt_angle().abs() < 1e-9);

        fsm.get_state_mut().vehicle_state.attitude = UnitQuaternion::identity();
        assert_eq!(build(&fsm, &sensors(0.0), 0.0).attitude, [0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn sensor_flags_follow_options() {
        let fsm = FlightStateMachine::new();
        let mut s = sensors(1.0);
        s.uwb_data = None;
        s.tank_pressure = None;
        let msg = build(&fsm, &s, 1.0);
        assert!(msg.sensors.imu_ok && msg.sensors.gps_ok && !msg.sensors.uwb_ok);
        assert_eq!(msg.sensors.accel, [0.1, 0.2, 9.81]);
        assert_eq!(msg.sensors.chamber_pressure, Some(15.0));
        assert_eq!(msg.sensors.tank_pressure, None);

        s.imu_data = None;
        s.gps_data = None;
        s.uwb_data = Some([0.0; 3]);
        let msg = build(&fsm, &s, 1.0);
        assert!(!msg.sensors.imu_ok && !msg.sensors.gps_ok && msg.sensors.uwb_ok);
        assert_eq!(msg.sensors.accel, [0.0; 3]);
    }

    #[test]
    fn trajectory_deviation_matches_termination_helper() {
        let mut fsm = FlightStateMachine::new();
        fsm.get_state_mut().trajectory_state = Some(rust_lossless::TrajectoryResult {
            positions: vec![[0.0, 0.0, 0.0], [0.0, 0.0, 10.0]],
            velocities: Vec::new(),
            masses: Vec::new(),
            thrusts: Vec::new(),
            sigmas: Vec::new(),
            time_of_flight_s: 5.0,
        });
        fsm.get_state_mut().vehicle_state.position = Vector3::new(3.0, 4.0, 10.0);
        let msg = build(&fsm, &sensors(0.0), 0.0);
        assert_eq!(msg.trajectory_deviation_m, Some(5.0));
    }

    #[test]
    fn event_severity_rules() {
        assert_eq!(event_severity("Flight terminated! Reason: Operator abort"), Severity::Critical);
        assert_eq!(event_severity("Emergency landing triggered: ..."), Severity::Critical);
        assert_eq!(event_severity("Warning: Ascent trajectory regeneration failed!"), Severity::Warning);
        assert_eq!(event_severity("Flight phase transition: Standby -> Armed"), Severity::Info);
    }

    #[test]
    fn arm_launch_interlocks() {
        let mut fsm = FlightStateMachine::new();
        assert!(apply_command(&mut fsm, &cmd(CommandKind::Launch), 0.0).is_err());
        assert!(apply_command(&mut fsm, &cmd(CommandKind::Disarm), 0.0).is_err());
        assert!(apply_command(&mut fsm, &cmd(CommandKind::Arm), 0.0).is_ok());
        assert_eq!(fsm.get_state().flight_phase, FlightPhase::Armed);
        assert!(apply_command(&mut fsm, &cmd(CommandKind::Arm), 0.0).is_err());
        assert!(apply_command(&mut fsm, &cmd(CommandKind::Disarm), 0.0).is_ok());
        assert_eq!(fsm.get_state().flight_phase, FlightPhase::Standby);
        assert!(apply_command(&mut fsm, &cmd(CommandKind::Arm), 1.0).is_ok());
        assert!(apply_command(&mut fsm, &cmd(CommandKind::Launch), 1.0).is_ok());
        assert_eq!(fsm.get_state().flight_phase, FlightPhase::Ascent);
    }

    #[test]
    fn phase_override_only_hover_or_descent_in_flight() {
        let mut fsm = FlightStateMachine::new();
        let set = |p| cmd(CommandKind::SetPhase(p));
        assert!(apply_command(&mut fsm, &set(gs_protocol::FlightPhase::Descent), 0.0).is_err());
        fsm.arm(0.0);
        assert!(apply_command(&mut fsm, &set(gs_protocol::FlightPhase::Hover), 0.0).is_err());
        fsm.launch(0.0);
        assert!(apply_command(&mut fsm, &set(gs_protocol::FlightPhase::Landed), 1.0).is_err());
        assert!(apply_command(&mut fsm, &set(gs_protocol::FlightPhase::Standby), 1.0).is_err());
        assert!(apply_command(&mut fsm, &set(gs_protocol::FlightPhase::Descent), 1.0).is_ok());
        assert_eq!(fsm.get_state().flight_phase, FlightPhase::Descent);
        assert!(apply_command(&mut fsm, &set(gs_protocol::FlightPhase::Hover), 2.0).is_ok());
        assert_eq!(fsm.get_state().flight_phase, FlightPhase::Hover);
        assert_eq!(fsm.get_state().last_state_time, 2.0);
    }

    #[test]
    fn abort_is_always_accepted_and_zeroes_controls() {
        for launch in [false, true] {
            let mut fsm = FlightStateMachine::new();
            if launch {
                fsm.arm(0.0);
                fsm.launch(0.0);
                fsm.get_state_mut().last_thrust = 800.0;
            }
            assert!(apply_command(&mut fsm, &cmd(CommandKind::Abort), 1.0).is_ok());
            let state = fsm.get_state();
            assert!(state.flight_terminated);
            assert_eq!(state.termination_reason.as_deref(), Some("Operator abort"));
            assert_eq!(state.last_thrust, 0.0);
            assert!(state.diagnostics_queue.iter().any(|m| event_severity(m) == Severity::Critical));
            assert!(fsm.step(&sensors(1.0)).is_none());
            assert!(apply_command(&mut fsm, &cmd(CommandKind::Arm), 2.0).is_err());
        }
    }

    #[test]
    fn jog_only_in_standby_and_expires() {
        let mut fsm = FlightStateMachine::new();
        let jog = |thrust: f32| cmd(CommandKind::Jog(JogSetpoint { gimbal_theta: 1.0, gimbal_phi: -0.1, thrust, rcs: 1 }));
        let jog_mode = cmd(CommandKind::SetControlMode(gs_protocol::ControlMode::Jog));

        // Not in Jog mode yet
        assert!(apply_command(&mut fsm, &jog(100.0), 0.0).is_err());
        assert!(apply_command(&mut fsm, &jog_mode, 0.0).is_ok());
        // Arm is refused while jogging
        assert!(apply_command(&mut fsm, &cmd(CommandKind::Arm), 0.0).is_err());

        assert!(apply_command(&mut fsm, &jog(5000.0), 1.0).is_ok());
        let out = fsm.step(&sensors(1.1)).unwrap();
        assert!((out[0] - 15.0_f64.to_radians()).abs() < 1e-9, "gimbal clamped to 15 deg");
        assert!((out[1] + 0.1).abs() < 1e-6);
        assert_eq!(out[2], 1200.0);
        assert_eq!(out[3], 1.0);
        assert_eq!(fsm.get_state().last_thrust, 1200.0);
        assert_eq!(fsm.get_state().last_rcs_command, 1.0);

        // Deadman: not refreshed for more than 0.5 s
        let out = fsm.step(&sensors(1.7)).unwrap();
        assert_eq!(out, [0.0, 0.0, 0.0, 0.0]);
        assert_eq!(fsm.get_state().last_thrust, 0.0);

        // Back to Auto, then any phase change keeps Auto and Jog is refused outside Standby
        assert!(apply_command(&mut fsm, &cmd(CommandKind::SetControlMode(gs_protocol::ControlMode::Auto)), 2.0).is_ok());
        assert!(apply_command(&mut fsm, &cmd(CommandKind::Arm), 2.0).is_ok());
        assert!(apply_command(&mut fsm, &jog_mode, 2.0).is_err());
        assert_eq!(fsm.control_mode(), ControlMode::Auto);
    }

    #[test]
    fn flight_params_are_range_checked() {
        let mut fsm = FlightStateMachine::new();
        let defaults = build_params(&mut fsm);
        assert_eq!(defaults.flight, FlightParams { hover_altitude_m: 50.0, hover_duration_s: 10.0, max_tilt_deg: 30.0, max_trajectory_deviation_m: 10.0 });
        assert_eq!(defaults.manual_mpc_weights, None);

        let bad = FlightParams { hover_altitude_m: 500.0, ..defaults.flight.clone() };
        assert!(apply_command(&mut fsm, &cmd(CommandKind::SetFlightParams(bad)), 0.0).is_err());
        assert_eq!(build_params(&mut fsm).flight, defaults.flight);

        let good = FlightParams { hover_altitude_m: 20.0, hover_duration_s: 5.0, max_tilt_deg: 25.0, max_trajectory_deviation_m: 8.0 };
        assert!(apply_command(&mut fsm, &cmd(CommandKind::SetFlightParams(good.clone())), 0.0).is_ok());
        assert_eq!(build_params(&mut fsm).flight, good);
        assert_eq!(fsm.goal()[2], 20.0);
    }

    #[test]
    fn mpc_weights_set_and_restore() {
        let mut fsm = FlightStateMachine::new();
        let weights = MpcWeights { q: [1.0; 13], r: [2.0; 3], qn: [3.0; 13] };
        assert!(apply_command(&mut fsm, &cmd(CommandKind::SetMpcWeights(Some(weights.clone()))), 0.0).is_ok());
        assert_eq!(build_params(&mut fsm).manual_mpc_weights, Some(weights.clone()));
        // Manual weights survive phase changes
        fsm.arm(0.0);
        fsm.launch(0.0);
        assert_eq!(build_params(&mut fsm).manual_mpc_weights, Some(weights));

        let negative = MpcWeights { q: [-1.0; 13], r: [2.0; 3], qn: [3.0; 13] };
        assert!(apply_command(&mut fsm, &cmd(CommandKind::SetMpcWeights(Some(negative))), 0.0).is_err());
        let nan = MpcWeights { q: [1.0; 13], r: [f32::NAN; 3], qn: [3.0; 13] };
        assert!(apply_command(&mut fsm, &cmd(CommandKind::SetMpcWeights(Some(nan))), 0.0).is_err());

        assert!(apply_command(&mut fsm, &cmd(CommandKind::SetMpcWeights(None)), 0.0).is_ok());
        assert_eq!(build_params(&mut fsm).manual_mpc_weights, None);
        // Built-in Ascent weights are back in the MPC (R thrust weight 0.005)
        let mpc = fsm.autopilot_mut().mpc_mut().unwrap();
        assert_eq!(mpc.r[(2, 2)], 0.005);
    }

    #[test]
    fn valves_only_in_standby_and_reported_back() {
        let mut fsm = FlightStateMachine::new();
        let open_iso = cmd(CommandKind::SetValve { id: ValveId::OIso, open: true });
        assert!(apply_command(&mut fsm, &open_iso, 0.0).is_ok());
        fsm.get_state_mut().last_rcs_command = 1.0;

        let stand = build_stand_telemetry(&fsm, &sensors(0.0), Source::Vehicle, 0.0);
        assert_eq!(stand.channels, vec![(StandChannel::Ept, 15.0), (StandChannel::Opt, 300.0)]);
        let state_of = |id| stand.valves.iter().find(|v| v.id == id).map(|v| v.state);
        assert_eq!(state_of(ValveId::OIso), Some(ValveState::Open));
        assert_eq!(state_of(ValveId::Rcs1), Some(ValveState::Open));
        assert_eq!(state_of(ValveId::Rcs2), Some(ValveState::Closed));
        assert_eq!(state_of(ValveId::Omv), None);

        fsm.arm(0.0);
        assert!(apply_command(&mut fsm, &open_iso, 0.0).is_err());
    }
}
