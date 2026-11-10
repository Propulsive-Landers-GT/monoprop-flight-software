use super::{FeedThrottle, MassComponent, MassProperties};
use crate::state::PropulsionParameters;

/// Default engine model's nominal fraction of consumed mass from a named component.
/// Fractions sum to one before applying the live feed openings.
/// Custom engine dynamics may use their own feed allocation instead.
#[derive(Debug, Clone)]
pub struct PropellantFeed {
    pub component: String,
    pub mass_fraction: f64,
    /// Initial command, opening and actuator model; runtime state is cloned from it.
    pub throttle: FeedThrottle,
}

impl PropellantFeed {
    /// Fully open, immediate-response throttle by default.
    pub fn new(component: impl Into<String>, mass_fraction: f64) -> Self {
        Self {
            component: component.into(),
            mass_fraction,
            throttle: FeedThrottle::default(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct EngineParameters {
    pub name: String,
    pub min_thrust_n: f64,
    pub max_thrust_n: f64,
    pub propulsion: PropulsionParameters,
    pub feeds: Vec<PropellantFeed>,
}

/// Limits for the default TVC module. Steps are per control update, matching the
/// existing actuator behavior; a custom TVC module can implement time-based dynamics.
#[derive(Debug, Clone, Copy)]
pub struct TvcParameters {
    pub max_angle_rad: f64,
    pub max_angle_step_rad: f64,
    pub max_thrust_step_n: f64,
    pub jog_max_thrust_n: f64,
}

/// Editable vehicle description. All masses are kg, thrusts N, angles rad, and
/// principal moments of inertia kg m². No live estimator/control history is stored here.
#[derive(Debug, Clone)]
pub struct VehicleParameters {
    pub mass_components: Vec<MassComponent>,
    pub engines: Vec<EngineParameters>,
    pub tvc: TvcParameters,
    pub inertia_kg_m2: [f64; 3],
    pub ascent_max_velocity_m_s: f64,
    pub descent_max_velocity_m_s: f64,
}

impl Default for VehicleParameters {
    fn default() -> Self {
        super::profiles::monarch()
    }
}

impl VehicleParameters {
    pub fn dry_mass_kg(&self) -> f64 {
        self.mass_components
            .iter()
            .map(MassComponent::retained_mass_kg)
            .sum()
    }

    pub fn initial_mass_kg(&self) -> f64 {
        self.mass_components
            .iter()
            .map(MassComponent::total_mass_kg)
            .sum()
    }

    pub fn mass_properties(&self) -> MassProperties {
        MassProperties::from_components(&self.mass_components)
    }

    pub fn min_thrust_n(&self) -> f64 {
        // All engines use the same normalized throttle. The engine with the
        // highest minimum throttle sets the bank's lower bound.
        let min_throttle = self
            .engines
            .iter()
            .map(|e| e.min_thrust_n / e.max_thrust_n)
            .fold(0.0_f64, f64::max);
        min_throttle * self.max_thrust_n()
    }

    pub fn max_thrust_n(&self) -> f64 {
        self.engines.iter().map(|e| e.max_thrust_n).sum()
    }
}
