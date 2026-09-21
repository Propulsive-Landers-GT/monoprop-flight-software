use ndarray::Array1;
use nalgebra::{Vector3, UnitQuaternion};
use std::collections::BTreeMap;

#[derive(Debug, Clone, serde::Serialize)]
pub struct ImuData {
    pub accel: [f64; 3],
    pub gyro: [f64; 3],
    pub mag: [f64; 3],
}

// TODO: Update the SensorData struct to be more representative of the data provided by the VN-200
    // Bonus points if you can create an IMU abstraction to allow any IMU and it's data to be utilized
#[derive(Debug, Clone, serde::Serialize)]
pub struct SensorData {
    pub timestamp: f64,
    pub imu_data: Option<ImuData>,
    pub gps_data: Option<[f64; 3]>,
    pub uwb_data: Option<[f64; 3]>,
    pub chamber_pressure: Option<f64>,
    pub tank_pressure: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub enum FlightPhase {
    Standby,
    Armed,
    Ascent,
    Hover,
    Descent,
    Landed,
}

/// Who drives the actuators. `Jog` (ground checkout of gimbal / throttle / RCS from the
/// ground station) is only reachable in `Standby`; any phase transition forces `Auto`.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub enum ControlMode {
    Auto,
    Jog,
}

/// Operator-tunable flight parameters. Defaults are the values that used to be hardcoded in the FSM.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlightLimits {
    pub hover_altitude_m: f64,
    pub hover_duration_s: f64,
    pub max_tilt_deg: f64,
    pub max_trajectory_deviation_m: f64,
}

impl Default for FlightLimits {
    fn default() -> Self {
        Self {
            hover_altitude_m: 50.0,
            hover_duration_s: 10.0,
            max_tilt_deg: 30.0,
            max_trajectory_deviation_m: 10.0,
        }
    }
}

impl FlightLimits {
    pub fn validate(&self) -> Result<(), String> {
        fn check(name: &str, value: f64, min: f64, max: f64) -> Result<(), String> {
            if value.is_finite() && value >= min && value <= max {
                Ok(())
            } else {
                Err(format!("{} {} out of range ({} to {})", name, value, min, max))
            }
        }
        check("hover altitude [m]", self.hover_altitude_m, 1.0, 200.0)?;
        check("hover duration [s]", self.hover_duration_s, 0.0, 120.0)?;
        check("max tilt [deg]", self.max_tilt_deg, 5.0, 60.0)?;
        check("max trajectory deviation [m]", self.max_trajectory_deviation_m, 1.0, 50.0)?;
        Ok(())
    }
}

/// Ground jog setpoint, already clamped. `time` is the mission time it was received at;
/// it is ignored once older than `JOG_TIMEOUT_S` (deadman).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JogSetpoint {
    pub gimbal_theta: f64,
    pub gimbal_phi: f64,
    pub thrust: f64,
    pub rcs: f64,
    pub time: f64,
}

pub const JOG_TIMEOUT_S: f64 = 0.5;
pub const JOG_MAX_GIMBAL_RAD: f64 = 15.0 * std::f64::consts::PI / 180.0;
pub const JOG_MAX_THRUST_N: f64 = 1200.0;

#[derive(Debug, Clone, serde::Serialize)]
pub struct VehicleState {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub attitude: UnitQuaternion<f64>,
    pub angular_velocity: Vector3<f64>,
    pub mass: f64,
    pub dry_mass: f64,
}

impl Default for VehicleState {
    fn default() -> Self {
        Self {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            attitude: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
            mass: 80.0,
            dry_mass: 50.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct ControlLoopState {
    pub sensor_fusion_state: Option<Array1<f64>>,
    pub trajectory_state: Option<rust_lossless::TrajectoryResult>,
    pub last_sensor_update: f64,
    pub last_navigation_update: f64,
    pub last_mpc_update: f64,
    pub last_position_update: f64,
    pub start_time: f64,
    pub vehicle_state: VehicleState,
    pub flight_terminated: bool,
    pub flight_phase: FlightPhase,
    pub last_state_time: f64,
    pub trajectory_generation_time: f64,
    pub last_gimbal_theta: f64,
    pub last_gimbal_phi: f64,
    pub last_thrust: f64,
    /// Last roll command sent to the RCS valves: +1 = CW thruster, -1 = CCW, 0 = both closed.
    pub last_rcs_command: f64,
    pub mass: f64,
    pub termination_reason: Option<String>,
    pub diagnostics_queue: Vec<String>,
    pub control_mode: ControlMode,
    pub jog_setpoint: Option<JogSetpoint>,
    /// Operator valve commands from the ground station, keyed by P&ID tag ("OMV", "O-ISO", ...).
    /// TODO: nothing actuates these yet. Lander has no valve hardware layer; this is where one
    ///       should read its commands from. For now they are only echoed back in stand telemetry.
    pub valve_overrides: BTreeMap<&'static str, bool>,
}

impl Default for ControlLoopState {
    fn default() -> Self {
        Self {
            sensor_fusion_state: None,
            trajectory_state: None,
            last_sensor_update: 0.0,
            last_navigation_update: 0.0,
            last_mpc_update: 0.0,
            last_position_update: 0.0,
            start_time: 0.0,
            vehicle_state: VehicleState::default(),
            flight_terminated: false,
            flight_phase: FlightPhase::Standby,
            last_state_time: 0.0,
            trajectory_generation_time: 0.0,
            last_gimbal_theta: 0.0,
            last_gimbal_phi: 0.0,
            last_thrust: 80.0 * 9.81,
            last_rcs_command: 0.0,
            mass: 80.0,
            termination_reason: None,
            diagnostics_queue: Vec::new(),
            control_mode: ControlMode::Auto,
            jog_setpoint: None,
            valve_overrides: BTreeMap::new(),
        }
    }
}
