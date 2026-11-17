//! Vehicle-specific presets. Add new named profiles here or define them in your caller.

use super::super::{
    EngineParameters, MassComponent, PropellantFeed, TvcParameters, VehicleParameters,
};
use crate::state::PropulsionParameters;

/// Original flight-software defaults: 80 kg wet, 50 kg dry and fixed inertia.
/// This is the control-loop baseline, not the simulator's physical mass model.
pub fn old_monarch() -> VehicleParameters {
    // Preserve the original aggregate estimator for compatibility comparisons.
    let mut component = MassComponent::new("vehicle", 50.0, 30.0);
    component.update_mass = update_legacy_mass;
    component.inertia_kg_m2 =
        nalgebra::Matrix3::from_diagonal(&nalgebra::Vector3::new(6.65, 6.65, 2.3));
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
        ascent_max_velocity_m_s: 50.0,
        descent_max_velocity_m_s: 15.0,
    }
}

/// Match the old total-mass subtraction order, including prelaunch reset.
fn update_legacy_mass(
    component: &mut super::super::MassComponent,
    context: &super::super::MassUpdateContext<'_>,
) {
    use crate::state::FlightPhase;
    if matches!(
        context.flight_phase,
        FlightPhase::Standby | FlightPhase::Armed
    ) {
        component.propellant_mass_kg = context.initial.propellant_mass_kg;
    } else {
        let retained = component.retained_mass_kg();
        let total = (component.total_mass_kg() - context.propellant_consumed_kg).max(retained);
        component.propellant_mass_kg = total - retained;
    }
}
