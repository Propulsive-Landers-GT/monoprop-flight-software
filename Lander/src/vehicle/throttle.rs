use std::any::Any;
use std::fmt;

use super::{EngineParameters, MassComponent, PropellantFeed};
use crate::state::{FlightPhase, SensorData};

/// Inputs to one feed's actuator dynamics, before engine dynamics run.
pub struct ThrottleUpdateContext<'a> {
    pub dt_s: f64,
    pub sensors: &'a SensorData,
    pub flight_phase: FlightPhase,
    /// Last applied net thrust command for the vehicle [N].
    pub thrust_n: f64,
    pub engine: &'a EngineParameters,
    pub feed: &'a PropellantFeed,
    pub components: &'a [MassComponent],
}

/// Implement this and derive Clone for a custom valve/actuator model. Opening is
/// normalized: 0 is closed, 1 is fully open; it is not necessarily a flow fraction.
/// Keep actuator-specific inputs and history in the implementation.
pub trait ThrottleModel: ThrottleModelClone + Any {
    fn update(
        &mut self,
        commanded_opening: f64,
        previous_opening: f64,
        context: &ThrottleUpdateContext<'_>,
    ) -> f64;
}

/// Allows configured models to be cloned into independent live feed actuators.
/// Supplied automatically for Clone implementations of ThrottleModel.
pub trait ThrottleModelClone {
    fn clone_box(&self) -> Box<dyn ThrottleModel>;
}

impl<T: ThrottleModel + Clone> ThrottleModelClone for T {
    fn clone_box(&self) -> Box<dyn ThrottleModel> {
        Box::new(self.clone())
    }
}

/// Configuration stores the initial value; the runtime maintains a separate clone.
pub struct FeedThrottle {
    pub commanded_opening: f64,
    pub opening: f64,
    model: Box<dyn ThrottleModel>,
}

impl FeedThrottle {
    pub fn new(initial_opening: f64, model: impl ThrottleModel) -> Self {
        Self {
            commanded_opening: initial_opening,
            opening: initial_opening,
            model: Box::new(model),
        }
    }

    /// Set the target. The realized opening changes on the next dynamics update.
    pub fn set_command(&mut self, opening: f64) {
        self.commanded_opening = opening;
    }

    /// Replace actuator behavior while retaining the command and realized opening.
    /// Any internal history comes from the newly supplied model.
    pub fn set_model(&mut self, model: impl ThrottleModel) {
        self.model = Box::new(model);
    }

    pub fn model<T: ThrottleModel>(&self) -> Option<&T> {
        (self.model.as_ref() as &dyn Any).downcast_ref()
    }

    pub fn model_mut<T: ThrottleModel>(&mut self) -> Option<&mut T> {
        (self.model.as_mut() as &mut dyn Any).downcast_mut()
    }

    pub fn update(&mut self, context: &ThrottleUpdateContext<'_>) {
        self.opening = self
            .model
            .update(self.commanded_opening, self.opening, context);
    }
}

impl Clone for FeedThrottle {
    fn clone(&self) -> Self {
        Self {
            commanded_opening: self.commanded_opening,
            opening: self.opening,
            model: self.model.clone_box(),
        }
    }
}

impl fmt::Debug for FeedThrottle {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("FeedThrottle")
            .field("commanded_opening", &self.commanded_opening)
            .field("opening", &self.opening)
            .finish_non_exhaustive()
    }
}

impl Default for FeedThrottle {
    fn default() -> Self {
        Self::new(1.0, DirectThrottle)
    }
}

/// Ideal actuator: apply the command without lag. Default feeds start fully open.
#[derive(Debug, Clone, Copy)]
pub struct DirectThrottle;

impl ThrottleModel for DirectThrottle {
    fn update(&mut self, commanded_opening: f64, _: f64, _: &ThrottleUpdateContext<'_>) -> f64 {
        commanded_opening
    }
}

/// Simple servo valve: linear angle-to-opening mapping with a speed limit.
/// Supply measured travel angles and speed; nonlinear linkage/flow, backlash and
/// actuator loading can be implemented in a custom ThrottleModel/EngineModel.
#[derive(Debug, Clone, Copy)]
pub struct ServoValve {
    pub closed_angle_rad: f64,
    pub open_angle_rad: f64,
    pub max_speed_rad_s: f64,
}

impl ServoValve {
    pub fn angle_rad(&self, opening: f64) -> f64 {
        self.closed_angle_rad + opening * (self.open_angle_rad - self.closed_angle_rad)
    }
}

impl ThrottleModel for ServoValve {
    fn update(
        &mut self,
        commanded_opening: f64,
        previous_opening: f64,
        context: &ThrottleUpdateContext<'_>,
    ) -> f64 {
        let previous_angle = self.angle_rad(previous_opening);
        let target_angle = self.angle_rad(commanded_opening.clamp(0.0, 1.0));
        let max_step = self.max_speed_rad_s * context.dt_s;
        let angle = previous_angle + (target_angle - previous_angle).clamp(-max_step, max_step);
        (angle - self.closed_angle_rad) / (self.open_angle_rad - self.closed_angle_rad)
    }
}
