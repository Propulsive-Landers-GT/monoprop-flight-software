#![doc = include_str!("README.md")]

pub mod engine;
pub mod mass;
mod parameters;
pub mod profiles;
pub mod throttle;
pub mod tvc;

use crate::state::{FlightPhase, PropulsionParameters, SensorData};
pub use engine::{ConstantSpecificImpulse, EngineModel, EngineUpdateContext};
pub use mass::{MassComponent, MassProperties, MassUpdateContext, MassUpdateFn};
pub use parameters::{EngineParameters, PropellantFeed, TvcParameters, VehicleParameters};
pub use throttle::{
    DirectThrottle, FeedThrottle, ServoValve, ThrottleModel, ThrottleUpdateContext,
};
pub use tvc::{SlewLimitedTvc, TvcModel};

/// Select vehicle-specific implementations without changing the FSM.
pub struct VehicleModules {
    pub engine: Box<dyn EngineModel>,
    pub tvc: Box<dyn TvcModel>,
}

impl Default for VehicleModules {
    fn default() -> Self {
        Self {
            engine: Box::new(ConstantSpecificImpulse),
            tvc: Box::new(SlewLimitedTvc),
        }
    }
}

/// Keeps the editable initial description separate from live component masses.
pub(crate) struct Vehicle {
    parameters: VehicleParameters,
    components: Vec<MassComponent>,
    engine: Box<dyn EngineModel>,
    feed_throttles: Vec<Vec<FeedThrottle>>,
}

impl Default for Vehicle {
    fn default() -> Self {
        Self::new(
            VehicleParameters::default(),
            Box::new(ConstantSpecificImpulse),
        )
    }
}

impl Vehicle {
    pub fn new(parameters: VehicleParameters, engine: Box<dyn EngineModel>) -> Self {
        Self {
            components: parameters.mass_components.clone(),
            feed_throttles: Self::initial_throttles(&parameters),
            parameters,
            engine,
        }
    }

    pub fn parameters(&self) -> &VehicleParameters {
        &self.parameters
    }
    pub fn components(&self) -> &[MassComponent] {
        &self.components
    }

    fn initial_throttles(parameters: &VehicleParameters) -> Vec<Vec<FeedThrottle>> {
        parameters
            .engines
            .iter()
            .map(|engine| {
                engine
                    .feeds
                    .iter()
                    .map(|feed| feed.throttle.clone())
                    .collect()
            })
            .collect()
    }

    pub fn feed_throttle(&self, engine_index: usize, feed_index: usize) -> Option<&FeedThrottle> {
        self.feed_throttles.get(engine_index)?.get(feed_index)
    }

    pub fn feed_throttle_mut(
        &mut self,
        engine_index: usize,
        feed_index: usize,
    ) -> Option<&mut FeedThrottle> {
        self.feed_throttles
            .get_mut(engine_index)?
            .get_mut(feed_index)
    }

    pub fn engine_model<T: EngineModel>(&self) -> Option<&T> {
        (self.engine.as_ref() as &dyn std::any::Any).downcast_ref()
    }

    pub fn engine_model_mut<T: EngineModel>(&mut self) -> Option<&mut T> {
        (self.engine.as_mut() as &mut dyn std::any::Any).downcast_mut()
    }

    pub fn mass_kg(&self) -> f64 {
        self.components
            .iter()
            .map(MassComponent::total_mass_kg)
            .sum()
    }

    pub fn dry_mass_kg(&self) -> f64 {
        self.components
            .iter()
            .map(MassComponent::retained_mass_kg)
            .sum()
    }

    pub fn mass_properties(&self) -> MassProperties {
        MassProperties::from_components(&self.components)
    }

    /// Engines share the requested net thrust in proportion to their rated maximum.
    pub fn guidance_alpha(&self) -> f64 {
        let max_thrust = self.parameters.max_thrust_n();
        self.parameters
            .engines
            .iter()
            .map(|e| (e.max_thrust_n / max_thrust) * self.engine.guidance_alpha(e))
            .sum()
    }

    pub fn set_propulsion_parameters(&mut self, propulsion: PropulsionParameters) {
        for engine in &mut self.parameters.engines {
            engine.propulsion = propulsion;
        }
    }

    pub fn reset(&mut self) {
        self.components = self.parameters.mass_components.clone();
        self.feed_throttles = Self::initial_throttles(&self.parameters);
        self.engine.reset();
    }

    /// Backwards compatibility for the original aggregate-mass initializer. Only
    /// used by the default vehicle, whose sole component is named "vehicle".
    pub fn set_legacy_mass(&mut self, mass: f64, dry_mass: f64) {
        self.components[0].dry_mass_kg = dry_mass;
        self.components[0].propellant_mass_kg = (mass - dry_mass).max(0.0);
    }

    /// External mass accounting (e.g. a measured refill, vent, or payload release).
    /// Changes live mass only; reinitialization restores the parameter values.
    pub fn set_component_mass(&mut self, name: &str, dry_mass_kg: f64, propellant_mass_kg: f64) {
        for component in self.components.iter_mut().filter(|c| c.name == name) {
            component.dry_mass_kg = dry_mass_kg;
            component.propellant_mass_kg = propellant_mass_kg;
        }
    }

    pub fn update_dynamics(
        &mut self,
        thrust_n: f64,
        sensors: &SensorData,
        dt: f64,
        phase: FlightPhase,
    ) {
        for (engine, throttles) in self.parameters.engines.iter().zip(&mut self.feed_throttles) {
            for (feed, throttle) in engine.feeds.iter().zip(throttles) {
                throttle.update(&ThrottleUpdateContext {
                    dt_s: dt,
                    sensors,
                    flight_phase: phase,
                    thrust_n,
                    engine,
                    feed,
                    components: &self.components,
                });
            }
        }
        let mut consumed = vec![0.0; self.components.len()];
        self.engine.update_dynamics(&mut EngineUpdateContext {
            dt_s: dt,
            sensors,
            flight_phase: phase,
            thrust_n,
            engines: &self.parameters.engines,
            feed_throttles: &self.feed_throttles,
            initial_components: &self.parameters.mass_components,
            components: &mut self.components,
            propellant_consumed_kg: &mut consumed,
        });
        for ((component, initial), draw) in self
            .components
            .iter_mut()
            .zip(&self.parameters.mass_components)
            .zip(consumed)
        {
            let context = MassUpdateContext {
                dt_s: dt,
                sensors,
                flight_phase: phase,
                propellant_consumed_kg: draw,
                initial,
            };
            (component.update_mass)(component, &context);
        }
    }
}
