use crate::state::{SensorData, FlightPhase, ControlLoopState, ControlMode, FlightLimits, JogSetpoint};
use crate::state::{JOG_TIMEOUT_S, JOG_MAX_GIMBAL_RAD, JOG_MAX_THRUST_N};
use crate::algorithms::{Navigator, GuidancePlanner, Controller};
use ndarray::{Array1, Array2};

mod scheduler;
mod actuator;
mod autopilot;

pub use scheduler::Scheduler;
pub use actuator::ActuatorController;
pub use autopilot::Autopilot;

pub struct FlightStateMachine {
    autopilot: Autopilot,
    actuator_controller: ActuatorController,
    scheduler: Scheduler,
    state: ControlLoopState,
    goal: [f64; 3],
    limits: FlightLimits,
}

impl FlightStateMachine {
    pub fn new() -> Self {
        Self {
            autopilot: Autopilot::new(),
            actuator_controller: ActuatorController::new(),
            scheduler: Scheduler::new(500.0, 1.0, 50.0),
            state: ControlLoopState::default(),
            goal: [0.0, 0.0, 50.0], // The ascent target. The descent targets the origin [0.0, 0.0, 0.0] as the landing pad.
            limits: FlightLimits::default(),
        }
    }
    
    pub fn new_with_algorithms(
        navigator: Box<dyn Navigator>,
        guidance: Box<dyn GuidancePlanner>,
        controller: Box<dyn Controller>,
    ) -> Self {
        Self {
            autopilot: Autopilot::new_with_algorithms(navigator, guidance, controller),
            actuator_controller: ActuatorController::new(),
            scheduler: Scheduler::new(500.0, 1.0, 50.0),
            state: ControlLoopState::default(),
            goal: [0.0, 0.0, 50.0],
            limits: FlightLimits::default(),
        }
    }
    
    pub fn initialize(&mut self) {
        self.state.start_time = 0.0;
        self.state.last_sensor_update = 0.0;
        self.state.last_navigation_update = 0.0;
        self.state.last_mpc_update = 0.0;
        self.state.last_state_time = 0.0;
        self.state.last_position_update = 0.0;
        self.state.mass = 80.0;
        self.state.flight_terminated = false;
        self.state.flight_phase = FlightPhase::Standby;

        println!("Flight State Machine initialized");
    }
    
    pub fn arm(&mut self, now: f64) {
        if self.state.flight_phase == FlightPhase::Standby {
            self.on_transition(FlightPhase::Standby, FlightPhase::Armed, now);
            println!("Command received: ARMED");
        }
    }
    
    pub fn disarm(&mut self, now: f64) {
        if self.state.flight_phase == FlightPhase::Armed {
            self.on_transition(FlightPhase::Armed, FlightPhase::Standby, now);
            println!("Command received: DISARMED");
        }
    }
    
    pub fn launch(&mut self, now: f64) {
        if self.state.flight_phase == FlightPhase::Armed {
            self.on_transition(FlightPhase::Armed, FlightPhase::Ascent, now);
            println!("Command received: LAUNCHED");
        }
    }

    pub fn set_flight_phase(&mut self, phase: FlightPhase, now: f64) {
        let from = self.state.flight_phase;
        self.on_transition(from, phase, now);
    }

    /// Operator abort from the ground station / console. Accepted in every phase.
    /// Same end state as an automatic flight termination: controls zeroed, `step()` returns None.
    /// In flight this cuts thrust and the vehicle falls; "land now" is `command_phase(Descent)`.
    pub fn abort(&mut self, now: f64) {
        if self.state.flight_terminated {
            return;
        }
        let reason = "Operator abort".to_string();
        self.state.flight_terminated = true;
        self.state.termination_reason = Some(reason.clone());
        self.state.last_gimbal_theta = 0.0;
        self.state.last_gimbal_phi = 0.0;
        self.state.last_thrust = 0.0;
        self.state.last_rcs_command = 0.0;
        self.state.jog_setpoint = None;
        let msg = format!("Flight terminated! Reason: {} (t = {:.2}s, phase {:?})", reason, now, self.state.flight_phase);
        println!("{}", msg);
        self.state.diagnostics_queue.push(msg);
    }

    /// Operator phase override. Only Hover and Descent can be commanded, and only while flying
    /// (Ascent / Hover / Descent). `set_flight_phase` stays unrestricted for the sim and tuner.
    ///
    /// Hover exits a fixed duration after it was entered (`last_state_time`), so commanding Hover
    /// (from Ascent, Descent, or again from Hover) simply re-enters Hover: the vehicle goes to /
    /// holds the hover goal (pad x/y, hover altitude) for one more hover duration, then descends
    /// as usual. It is not a "freeze where you are".
    pub fn command_phase(&mut self, phase: FlightPhase, now: f64) -> Result<(), String> {
        if phase != FlightPhase::Hover && phase != FlightPhase::Descent {
            return Err(format!("{:?} cannot be commanded (only Hover or Descent)", phase));
        }
        if self.state.flight_terminated {
            return Err("flight terminated".to_string());
        }
        let from = self.state.flight_phase;
        match from {
            FlightPhase::Ascent | FlightPhase::Hover | FlightPhase::Descent => {
                self.state.diagnostics_queue.push(format!("Operator phase override: {:?}", phase));
                self.on_transition(from, phase, now);
                // Replan on the next step instead of waiting out the 1 Hz guidance period
                // (a stale ascent trajectory must not be tracked during descent).
                self.state.last_navigation_update = now - 1.0;
                Ok(())
            }
            _ => Err(format!("phase override only allowed in flight (currently {:?})", from)),
        }
    }

    pub fn flight_limits(&self) -> FlightLimits {
        self.limits
    }

    /// Range-checked. The hover altitude is also the ascent target (`goal[2]`).
    pub fn set_flight_limits(&mut self, limits: FlightLimits) -> Result<(), String> {
        limits.validate()?;
        self.limits = limits;
        self.goal[2] = limits.hover_altitude_m;
        self.state.diagnostics_queue.push(format!(
            "Flight params set: hover {:.1} m for {:.1} s, max tilt {:.1} deg, max deviation {:.1} m",
            limits.hover_altitude_m, limits.hover_duration_s, limits.max_tilt_deg, limits.max_trajectory_deviation_m
        ));
        Ok(())
    }

    pub fn control_mode(&self) -> ControlMode {
        self.state.control_mode
    }

    /// Jog is only accepted in Standby. Any phase transition forces Auto (see `on_transition`).
    pub fn set_control_mode(&mut self, mode: ControlMode) -> Result<(), String> {
        if mode == ControlMode::Jog && self.state.flight_phase != FlightPhase::Standby {
            return Err(format!("Jog only allowed in Standby (currently {:?})", self.state.flight_phase));
        }
        if mode == ControlMode::Jog && self.state.flight_terminated {
            return Err("flight terminated".to_string());
        }
        if mode != self.state.control_mode {
            self.state.diagnostics_queue.push(format!("Control mode: {:?}", mode));
        }
        self.state.control_mode = mode;
        self.state.jog_setpoint = None;
        Ok(())
    }

    /// Jog setpoint from the ground. Clamped to gimbal +/-15 deg, thrust 0-1200 N, rcs -1/0/+1.
    /// Must be refreshed within `JOG_TIMEOUT_S` or `step()` outputs zeros again.
    pub fn set_jog(&mut self, gimbal_theta: f64, gimbal_phi: f64, thrust: f64, rcs: f64, now: f64) -> Result<(), String> {
        if self.state.control_mode != ControlMode::Jog || self.state.flight_phase != FlightPhase::Standby {
            return Err("not in Jog control mode".to_string());
        }
        if !(gimbal_theta.is_finite() && gimbal_phi.is_finite() && thrust.is_finite() && rcs.is_finite()) {
            return Err("jog setpoint is not finite".to_string());
        }
        self.state.jog_setpoint = Some(JogSetpoint {
            gimbal_theta: gimbal_theta.clamp(-JOG_MAX_GIMBAL_RAD, JOG_MAX_GIMBAL_RAD),
            gimbal_phi: gimbal_phi.clamp(-JOG_MAX_GIMBAL_RAD, JOG_MAX_GIMBAL_RAD),
            thrust: thrust.clamp(0.0, JOG_MAX_THRUST_N),
            rcs: if rcs > 0.5 { 1.0 } else if rcs < -0.5 { -1.0 } else { 0.0 },
            time: now,
        });
        Ok(())
    }

    /// Switch the MPC to operator weights given as the diagonals of Q (13), R (3) and QN (13).
    /// State order [x y z | qx qy qz qw | vx vy vz | wx wy wz], input order [theta phi thrust].
    pub fn set_mpc_weights(&mut self, q: &[f64; 13], r: &[f64; 3], qn: &[f64; 13]) -> Result<(), String> {
        if q.iter().chain(r.iter()).chain(qn.iter()).any(|w| !w.is_finite() || *w < 0.0) {
            return Err("MPC weights must be finite and non-negative".to_string());
        }
        let mpc = self.autopilot.mpc_mut().ok_or("controller is not the built-in MPC")?;
        mpc.set_manual_weights(
            true,
            Some(Array2::from_diag(&Array1::from(q.to_vec()))),
            Some(Array2::from_diag(&Array1::from(r.to_vec()))),
            Some(Array2::from_diag(&Array1::from(qn.to_vec()))),
        );
        self.state.diagnostics_queue.push("MPC weights: manual (set from ground)".to_string());
        Ok(())
    }

    /// Back to the built-in per-phase weights (re-applied for the current phase right away).
    pub fn clear_mpc_weights(&mut self) -> Result<(), String> {
        let phase = self.state.flight_phase;
        let mpc = self.autopilot.mpc_mut().ok_or("controller is not the built-in MPC")?;
        mpc.set_manual_weights(false, None, None, None);
        self.autopilot.set_flight_phase(phase);
        self.state.diagnostics_queue.push("MPC weights: built-in".to_string());
        Ok(())
    }

    /// Diagonals (Q, R, QN) while manual weights are active, None on the built-in ones.
    pub fn manual_mpc_weights(&mut self) -> Option<([f64; 13], [f64; 3], [f64; 13])> {
        let mpc = self.autopilot.mpc_mut()?;
        if !mpc.manual_weights || mpc.q.nrows() != 13 || mpc.r.nrows() != 3 || mpc.qn.nrows() != 13 {
            return None;
        }
        let mut q = [0.0; 13];
        let mut r = [0.0; 3];
        let mut qn = [0.0; 13];
        for i in 0..13 {
            q[i] = mpc.q[(i, i)];
            qn[i] = mpc.qn[(i, i)];
        }
        for i in 0..3 {
            r[i] = mpc.r[(i, i)];
        }
        Some((q, r, qn))
    }

    /// The angle the flight-termination tilt check compares against its limit [rad].
    /// NOTE: this is only the first Euler angle (rotation about body X), see `check_flight_termination`.
    pub fn termination_tilt_angle(&self) -> f64 {
        let euler_attitude = self.state.vehicle_state.attitude.euler_angles();
        euler_attitude.0.abs()
    }

    /// True tilt: angle between body +Z (thrust axis) and world +Z [rad]. Used for telemetry.
    pub fn tilt_from_vertical(&self) -> f64 {
        let body_z = self.state.vehicle_state.attitude * nalgebra::Vector3::z();
        body_z.z.clamp(-1.0, 1.0).acos()
    }

    /// Distance from the estimated position to the closest node of the active trajectory [m].
    /// None without a trajectory. Shared by the termination check and telemetry.
    pub fn trajectory_deviation(&self) -> Option<f64> {
        let trajectory = self.state.trajectory_state.as_ref()?;
        if trajectory.positions.is_empty() {
            return None;
        }
        let current_pos = &self.state.vehicle_state.position;
        let mut min_dist = f64::MAX;
        for pos in &trajectory.positions {
            let dist = ((current_pos.x - pos[0]).powi(2) + 
                        (current_pos.y - pos[1]).powi(2) + 
                        (current_pos.z - pos[2]).powi(2)).sqrt();
            if dist < min_dist {
                min_dist = dist;
            }
        }
        Some(min_dist)
    }

    /// Where Ascent / Hover are heading. `goal[2]` follows `FlightLimits::hover_altitude_m`.
    pub fn goal(&self) -> [f64; 3] {
        self.goal
    }

    pub fn get_state(&self) -> &ControlLoopState {
        &self.state
    }

    pub fn get_state_mut(&mut self) -> &mut ControlLoopState {
        &mut self.state
    }

    fn next_phase(&self, goal_position: [f64; 3], now: f64) -> FlightPhase {
        let current_altitude = self.state.vehicle_state.position.z;
        let goal_altitude = goal_position[2];

        if self.state.flight_phase == FlightPhase::Ascent || self.state.flight_phase == FlightPhase::Hover {
            if self.state.last_position_update > 0.0 && now - self.state.last_position_update > 5.0 {
                return FlightPhase::Descent;
            }
        }

        match self.state.flight_phase {
            FlightPhase::Ascent => {
                if current_altitude >= goal_altitude {
                    FlightPhase::Hover
                } else {
                    FlightPhase::Ascent
                }
            },
            FlightPhase::Hover => {
                if now - self.state.last_state_time >= self.limits.hover_duration_s {
                    FlightPhase::Descent
                } else {
                    FlightPhase::Hover
                }
            },
            FlightPhase::Descent => {
                let v = &self.state.vehicle_state.velocity;
                let speed = (v.x.powi(2) + v.y.powi(2) + v.z.powi(2)).sqrt();
                if current_altitude <= 0.1 && speed < 0.2 {
                    FlightPhase::Landed
                } else {
                    FlightPhase::Descent
                }
            }
            FlightPhase::Standby => FlightPhase::Standby,
            FlightPhase::Armed => FlightPhase::Armed,
            FlightPhase::Landed => FlightPhase::Landed,
        }
    }

    fn on_transition(&mut self, from: FlightPhase, to: FlightPhase, now: f64) {
        self.state.flight_phase = to;
        self.state.last_state_time = now;
        // Jog is a Standby-only ground checkout mode: any phase change hands control back to the autopilot.
        self.state.control_mode = ControlMode::Auto;
        self.state.jog_setpoint = None;
        self.state.diagnostics_queue.push(format!("Flight phase transition: {:?} -> {:?}", from, to));
        self.autopilot.set_flight_phase(to);

        // Emergency landing contingency log if GPS/UWB denied
        if (from == FlightPhase::Ascent || from == FlightPhase::Hover) && to == FlightPhase::Descent {
            if self.state.last_position_update > 0.0 && now - self.state.last_position_update > 5.0 {
                let msg = format!(
                    "Emergency landing triggered: absolute position data (GPS/UWB) denied for {:.2}s",
                    now - self.state.last_position_update
                );
                self.state.diagnostics_queue.push(msg.clone());
                println!("{}", msg);
            }
        }
    }
    
    pub fn step(&mut self, sensor_data: &SensorData) -> Option<[f64; 4]> {
        if self.state.flight_terminated {
            return None;
        }
        
        let now = sensor_data.timestamp;

        // Centralized scheduling for Sensor Fusion (500 Hz)
        let sensor_fusion_due = self.scheduler.is_sensor_fusion_due(self.state.last_sensor_update, now);
        let sensor_dt = self.scheduler.sensor_dt(self.state.last_sensor_update, now);

        if sensor_fusion_due {
            self.autopilot.update_navigator(&mut self.state, sensor_data, now, sensor_dt);
        }
        
        // Mass depletion model (based on last_thrust from operator / MPC and actual elapsed sensor_dt)
        if self.state.flight_phase == FlightPhase::Standby || self.state.flight_phase == FlightPhase::Armed {
            self.state.mass = 80.0;
        } else if self.state.flight_phase != FlightPhase::Landed && sensor_fusion_due {
            let mass_flow = self.state.last_thrust / (180.0 * 9.81);
            self.state.mass -= mass_flow * sensor_dt;
            self.state.mass = self.state.mass.max(50.0);
        }
        
        // Run termination check on the freshest state (after sensor fusion update)
        if let Some(reason) = self.check_flight_termination(sensor_data, now) {
            self.state.flight_terminated = true;
            self.state.termination_reason = Some(reason.clone());
            // step() returns None from here on (callers zero the controls); make the reported
            // actuation state say the same, as `abort` does, so telemetry does not show stale thrust.
            self.state.last_gimbal_theta = 0.0;
            self.state.last_gimbal_phi = 0.0;
            self.state.last_thrust = 0.0;
            self.state.last_rcs_command = 0.0;
            self.state.diagnostics_queue.push(format!("Flight terminated! Reason: {}", reason));
            println!("Flight terminated! Reason: {}", reason);
            return None;
        }
        
        let old_phase = self.state.flight_phase;
        let new_phase = self.next_phase(self.goal, now);
        if new_phase != old_phase {
            self.on_transition(old_phase, new_phase, now);
        }
        
        // Centralized scheduling for Guidance planner (1 Hz)
        if self.scheduler.is_navigation_due(self.state.last_navigation_update, now) {
            self.autopilot.update_guidance(&mut self.state, self.goal, now);
        }
        
        // Centralized scheduling for MPC (50 Hz) and Actuator output
        let mut mpc_control_output = None;
        if self.state.flight_phase == FlightPhase::Standby || self.state.flight_phase == FlightPhase::Armed || self.state.flight_phase == FlightPhase::Landed {
            self.state.last_gimbal_theta = 0.0;
            self.state.last_gimbal_phi = 0.0;
            self.state.last_thrust = 0.0;
        } else if self.scheduler.is_mpc_due(self.state.last_mpc_update, now) {
            self.state.last_mpc_update = now;
            mpc_control_output = self.autopilot.update_mpc(&mut self.state, self.goal, now);
        }

        // Run actuator controller (gimbal step/clamping, roll control, thrust clamping)
        let mut control_signals = self.actuator_controller.update(&mut self.state, mpc_control_output, now);

        // Ground jog (Standby only): actuators follow the operator setpoint while it is fresh, zeros otherwise.
        if self.state.flight_phase == FlightPhase::Standby && self.state.control_mode == ControlMode::Jog {
            let (theta, phi, thrust, rcs) = match self.state.jog_setpoint {
                Some(jog) if now - jog.time <= JOG_TIMEOUT_S => (jog.gimbal_theta, jog.gimbal_phi, jog.thrust, jog.rcs),
                _ => (0.0, 0.0, 0.0, 0.0),
            };
            self.state.last_gimbal_theta = theta;
            self.state.last_gimbal_phi = phi;
            self.state.last_thrust = thrust;
            self.state.last_rcs_command = rcs;
            control_signals = [theta, phi, thrust, rcs];
        }

        Some(control_signals)
    }
    
    fn check_flight_termination(&self, sensor_data: &SensorData, now: f64) -> Option<String> {
        if sensor_data.imu_data.is_none() {
            return Some("IMU data missing".to_string());
        }
        
        if sensor_data.uwb_data.is_none() && sensor_data.gps_data.is_none() {
            let elapsed_since_pos = now - self.state.last_position_update;
            if elapsed_since_pos > 15.0 {
                return Some(format!("No GPS or UWB position data for {:.2}s (exceeded dead-reckoning safety limit)", elapsed_since_pos));
            }
        }
        
        if sensor_data.chamber_pressure.is_none() || sensor_data.tank_pressure.is_none() {
            return Some("Pressure data missing".to_string());
        }
        
        // NOTE: (GN&C team) this check only looks at the first Euler angle (`euler_angles().0`, roll
        //       about X), which is not the vehicle's tilt: a pure pitch about Y never trips it, however
        //       large. `tilt_from_vertical()` is the true angle between body +Z and world +Z and is
        //       what the ground station displays, so the GUI can show > limit without a termination.
        //       The check is deliberately left as it was; switching it to `tilt_from_vertical()`
        //       changes flight-termination behaviour and is your call.
        let tilt_angle = self.termination_tilt_angle();
        let max_tilt_deg = self.limits.max_tilt_deg;
        if tilt_angle > max_tilt_deg.to_radians() {
            return Some(format!("Tilt angle {:.2} deg exceeds maximum ({} deg)", tilt_angle.to_degrees(), max_tilt_deg));
        }
        
        if let Some(min_dist) = self.trajectory_deviation() {
            let max_deviation = self.limits.max_trajectory_deviation_m;
            if min_dist > max_deviation {
                return Some(format!("Deviation from trajectory {:.2}m exceeds {}m limit", min_dist, max_deviation));
            }
        }
        
        None
    }

    pub fn autopilot_mut(&mut self) -> &mut autopilot::Autopilot {
        &mut self.autopilot
    }
}
