//! Vehicle-specific presets. Add new named profiles here or define them in your caller.

use super::{EngineParameters, MassComponent, PropellantFeed, TvcParameters, VehicleParameters};
use crate::state::PropulsionParameters;

/// Monarch vehicle settings; used by all compatibility constructors.
pub fn monarch() -> VehicleParameters {
    // The Monarch profile. The original aggregate mass is kept as a
    // single component because no measured tank/engine mass breakdown exists here.
    let inertia_kg_m2 = [6.65, 6.65, 2.3];
    let mut component = MassComponent::new("vehicle", 50.0, 30.0);
    component.inertia_kg_m2 =
        nalgebra::Matrix3::from_diagonal(&nalgebra::Vector3::from(inertia_kg_m2));
    VehicleParameters {
        mass_components: vec![component],
        engines: vec![EngineParameters {
            name: "main engine".into(),
            min_thrust_n: 300.0,
            max_thrust_n: 1000.0,
            propulsion: PropulsionParameters::default(),
            feeds: vec![PropellantFeed::new("vehicle", 1.0)],
        }],
        tvc: TvcParameters {
            max_angle_rad: 15.0_f64.to_radians(),
            max_angle_step_rad: 2.0_f64.to_radians(),
            max_thrust_step_n: 40.0,
            jog_max_thrust_n: 1200.0,
        },
        inertia_kg_m2,
        ascent_max_velocity_m_s: 50.0,
        descent_max_velocity_m_s: 15.0,
    }
}
