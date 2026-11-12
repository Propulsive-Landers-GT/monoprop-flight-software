use super::TvcParameters;
use crate::state::VehicleState;

/// Maps requested control to realized [theta rad, phi rad, thrust N]. Implementations
/// may retain vehicle-specific servo/linkage state; outputs stay in physical units.
pub trait TvcModel {
    fn reset(&mut self, now: f64);
    fn update(
        &mut self,
        requested: [f64; 3],
        previous: [f64; 3],
        state: &VehicleState,
        parameters: &TvcParameters,
        now: f64,
    ) -> [f64; 3];

    /// Ground checkout uses the same vehicle mapping unless explicitly overridden.
    fn jog(
        &mut self,
        requested: [f64; 3],
        previous: [f64; 3],
        state: &VehicleState,
        parameters: &TvcParameters,
        now: f64,
    ) -> [f64; 3] {
        self.update(requested, previous, state, parameters, now)
    }
}

/// The original per-update slew limiter, extracted from the actuator controller.
pub struct SlewLimitedTvc;

impl TvcModel for SlewLimitedTvc {
    fn reset(&mut self, _now: f64) {}

    fn jog(
        &mut self,
        requested: [f64; 3],
        _previous: [f64; 3],
        _state: &VehicleState,
        _parameters: &TvcParameters,
        _now: f64,
    ) -> [f64; 3] {
        requested // Preserve the original direct (already clamped) ground jog.
    }

    fn update(
        &mut self,
        requested: [f64; 3],
        previous: [f64; 3],
        _state: &VehicleState,
        parameters: &TvcParameters,
        _now: f64,
    ) -> [f64; 3] {
        let mut applied = previous;
        for i in 0..2 {
            let target = requested[i].clamp(-parameters.max_angle_rad, parameters.max_angle_rad);
            applied[i] += (target - previous[i]).clamp(
                -parameters.max_angle_step_rad,
                parameters.max_angle_step_rad,
            );
        }
        applied[2] += (requested[2] - previous[2])
            .clamp(-parameters.max_thrust_step_n, parameters.max_thrust_step_n);
        applied
    }
}
