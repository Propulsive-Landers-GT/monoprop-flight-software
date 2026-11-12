use std::any::Any;

use super::{EngineParameters, FeedThrottle, MassComponent};
use crate::state::{FlightPhase, SensorData};

/// One dynamics step for the whole propulsion system. Components and queued draws
/// use the same indices. The component list stays fixed for the duration of a run.
pub struct EngineUpdateContext<'a> {
    pub dt_s: f64,
    pub sensors: &'a SensorData,
    pub flight_phase: FlightPhase,
    /// Last applied net thrust command [N]. Custom models choose their allocation.
    pub thrust_n: f64,
    pub engines: &'a [EngineParameters],
    /// Updated actuator states, indexed [engine][feed] like `engines`.
    /// Read these openings for live behavior; parameters contain initial values.
    pub feed_throttles: &'a [Vec<FeedThrottle>],
    pub initial_components: &'a [MassComponent],
    /// Live masses, positions and inertia; a model may update multiple components.
    pub components: &'a mut [MassComponent],
    /// Optional draws [kg] to pass to component callbacks after this method returns.
    /// Starts at zero each step. Leave zero for mass already changed directly.
    pub propellant_consumed_kg: &'a mut [f64],
}

/// Vehicle-specific propulsion dynamics, called after feed actuators once per
/// fusion tick in all phases.
/// Own arbitrary typed inputs, component state and submodels in the implementation
/// (e.g. throttle channels, temperatures, pressure, cooling or erosion history).
/// Access those fields through the FSM's typed engine_model[_mut] accessors.
pub trait EngineModel: Any {
    /// Restore model history to configured initial values when starting a run.
    /// Stateful implementations should override this; retain configuration.
    fn reset(&mut self) {}

    /// Coordinate updates across any number of components. Component callbacks run
    /// afterwards, then the FSM synchronizes aggregate mass. Choose phase behavior
    /// here, including any cooling or pressure changes while in Standby.
    fn update_dynamics(&mut self, context: &mut EngineUpdateContext<'_>);

    /// Constant kg/(N s) approximation used by the built-in guidance solver.
    /// More general planning models can use the existing GuidancePlanner interface.
    fn guidance_alpha(&self, engine: &EngineParameters) -> f64;
}

/// Nominal thrust-based flow with each feed's opening used as a linear multiplier.
/// Custom engines can use a calibrated valve/flow map and their own thrust model.
pub struct ConstantSpecificImpulse;

impl EngineModel for ConstantSpecificImpulse {
    fn update_dynamics(&mut self, context: &mut EngineUpdateContext<'_>) {
        if !matches!(
            context.flight_phase,
            FlightPhase::Ascent | FlightPhase::Hover | FlightPhase::Descent
        ) {
            return;
        }
        let max_thrust: f64 = context.engines.iter().map(|e| e.max_thrust_n).sum();
        for (engine_index, engine) in context.engines.iter().enumerate() {
            let thrust = context.thrust_n * (engine.max_thrust_n / max_thrust);
            let mass = engine.propulsion.mass_flow_rate(thrust) * context.dt_s;
            for (feed_index, feed) in engine.feeds.iter().enumerate() {
                let opening = context.feed_throttles[engine_index][feed_index].opening;
                for (component, draw) in context
                    .components
                    .iter()
                    .zip(context.propellant_consumed_kg.iter_mut())
                {
                    if component.name == feed.component {
                        *draw += mass * feed.mass_fraction * opening;
                    }
                }
            }
        }
        // Preserve the requested draw ratios when the default bank runs low.
        let mut available_fraction: f64 = 1.0;
        for (component, draw) in context
            .components
            .iter()
            .zip(context.propellant_consumed_kg.iter())
        {
            if *draw > 0.0 {
                available_fraction = available_fraction.min(component.propellant_mass_kg / draw);
            }
        }
        for draw in context.propellant_consumed_kg.iter_mut() {
            *draw *= available_fraction;
        }
    }

    fn guidance_alpha(&self, engine: &EngineParameters) -> f64 {
        engine.propulsion.alpha()
    }
}
