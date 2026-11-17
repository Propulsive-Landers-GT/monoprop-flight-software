use crate::state::ControlLoopState;
use crate::vehicle::{TvcModel, TvcParameters, VehicleParameters, SlewLimitedTvc};

pub struct ActuatorController {
    rcs_controller: crate::algorithms::rcs::RcsController,
    tvc: Box<dyn TvcModel>,
    tvc_parameters: TvcParameters,
}

impl ActuatorController {
    pub fn rcs_mut(&mut self) -> &mut crate::algorithms::rcs::RcsController {
        &mut self.rcs_controller
    }

    pub fn new() -> Self {
        Self {
            rcs_controller: crate::algorithms::rcs::RcsController::new(),
            tvc: Box::new(SlewLimitedTvc),
            tvc_parameters: VehicleParameters::default().tvc,
        }
    }

    pub fn set_tvc(&mut self, model: Box<dyn TvcModel>, parameters: TvcParameters) {
        self.tvc = model;
        self.tvc_parameters = parameters;
    }

    pub fn reset_tvc(&mut self, now: f64) {
        self.tvc.reset(now);
    }

    pub fn jog(&mut self, state: &ControlLoopState, requested: [f64; 3], now: f64) -> [f64; 3] {
        let previous = [state.last_gimbal_theta, state.last_gimbal_phi, state.last_thrust];
        self.tvc.jog(requested, previous, &state.vehicle_state, &self.tvc_parameters, now)
    }

    pub fn update(
        &mut self,
        state: &mut ControlLoopState,
        mpc_control_output: Option<[f64; 3]>,
        now: f64,
    ) -> [f64; 4] {
        if let Some(first_control) = mpc_control_output {
            let previous = [state.last_gimbal_theta, state.last_gimbal_phi, state.last_thrust];
            let applied = if first_control.iter().all(|v| v.is_finite()) {
                self.tvc.update(first_control, previous, &state.vehicle_state, &self.tvc_parameters, now)
            } else {
                previous
            };
            state.last_gimbal_theta = applied[0];
            state.last_gimbal_phi = applied[1];
            state.last_thrust = applied[2];
        }
        
        let (_, _, roll) = state.vehicle_state.attitude.euler_angles();
        let roll_rate = state.vehicle_state.angular_velocity.z;
        let rcs_command = self.rcs_controller.update(roll, roll_rate, now);
        state.last_rcs_command = rcs_command;
        
        [
            state.last_gimbal_theta,
            state.last_gimbal_phi,
            state.last_thrust,
            rcs_command,
        ]
    }
}
