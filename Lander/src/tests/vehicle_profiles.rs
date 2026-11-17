use nalgebra::{Matrix3, Vector3};

use crate::fsm::FlightStateMachine;
use crate::state::{FlightPhase, SensorData};
use crate::vehicle::{
    ComponentGeometry, ConstantSpecificImpulse, LiquidTankGeometry, MassComponent, Vehicle,
    VehicleParameters, profiles,
};

fn near(actual: f64, expected: f64) {
    assert!((actual - expected).abs() < 1e-10, "{actual} != {expected}");
}

fn advance(vehicle: &mut Vehicle, thrust: f64, dt: f64, phase: FlightPhase) {
    let sensors = SensorData {
        timestamp: dt,
        imu_data: None,
        gps_data: None,
        uwb_data: None,
        chamber_pressure: None,
        tank_pressure: None,
    };
    vehicle.update_dynamics(thrust, &sensors, dt, phase);
}

#[test]
fn compatibility_defaults_keep_old_parameters() {
    let parameters = VehicleParameters::default();
    assert_eq!(parameters.mass_components.len(), 1);
    assert_eq!(parameters.mass_components[0].name, "vehicle");
    assert_eq!(parameters.initial_mass_kg(), 80.0);
    assert_eq!(parameters.dry_mass_kg(), 50.0);
    assert_eq!(
        parameters.mass_properties().inertia_kg_m2,
        Matrix3::from_diagonal(&Vector3::new(6.65, 6.65, 2.3))
    );
    assert_eq!(parameters.min_thrust_n(), 300.0);
    assert_eq!(parameters.max_thrust_n(), 1000.0);
    assert_eq!(
        parameters.engines[0].propulsion.alpha(),
        1.0 / (180.0 * 9.81)
    );
    assert_eq!(parameters.tvc.max_angle_rad, 15_f64.to_radians());
    assert_eq!(parameters.tvc.max_angle_step_rad, 2_f64.to_radians());
    assert_eq!(parameters.tvc.max_thrust_step_n, 40.0);
    assert_eq!(parameters.ascent_max_velocity_m_s, 50.0);
    assert_eq!(parameters.descent_max_velocity_m_s, 15.0);
}

#[test]
fn old_mass_updates_match_original_arithmetic_and_prelaunch_reset() {
    let mut vehicle = Vehicle::new(profiles::old_monarch(), Box::new(ConstantSpecificImpulse));
    let initial_inertia = vehicle.mass_properties().inertia_kg_m2;
    let mut expected: f64 = 80.0;
    for index in 0..1000 {
        let thrust = 300.0 + (index % 701) as f64;
        let dt = if index == 0 { 0.002 } else { 0.01 };
        expected = (expected - thrust / (180.0 * 9.81) * dt).max(50.0);
        advance(&mut vehicle, thrust, dt, FlightPhase::Ascent);
        assert_eq!(vehicle.mass_kg(), expected);
        assert_eq!(vehicle.mass_properties().inertia_kg_m2, initial_inertia);
    }
    advance(&mut vehicle, 1000.0, 100.0, FlightPhase::Descent);
    assert_eq!(vehicle.mass_kg(), 50.0);
    advance(&mut vehicle, 1000.0, 1.0, FlightPhase::Landed);
    assert_eq!(vehicle.mass_kg(), 50.0);
    advance(&mut vehicle, 1000.0, 1.0, FlightPhase::Standby);
    assert_eq!(vehicle.mass_kg(), 80.0);
    advance(&mut vehicle, 1000.0, 1.0, FlightPhase::Ascent);
    advance(&mut vehicle, 1000.0, 1.0, FlightPhase::Armed);
    assert_eq!(vehicle.mass_kg(), 80.0);
}

#[test]
fn monarch_parts_and_feed_draws_conserve_mass() {
    let parameters = profiles::monarch();
    assert_eq!(parameters.mass_components.len(), 4);
    near(parameters.initial_mass_kg(), 79.88);
    // Structure is 55.88 kg; default engine retains all 5 kg of N2.
    near(parameters.dry_mass_kg(), 60.88);
    let mut vehicle = Vehicle::new(parameters, Box::new(ConstantSpecificImpulse));
    let initial = vehicle.mass_properties().clone();
    advance(&mut vehicle, 180.0 * 9.81, 1.0, FlightPhase::Ascent);
    near(vehicle.mass_kg(), 78.88);
    let oxidizer = vehicle
        .components()
        .iter()
        .find(|c| c.name == "nitrous tank")
        .unwrap();
    let engine = vehicle
        .components()
        .iter()
        .find(|c| c.name == "engine")
        .unwrap();
    near(oxidizer.propellant_mass_kg, 16.0 - 16.0 / 19.0);
    near(engine.propellant_mass_kg, 3.0 - 3.0 / 19.0);
    assert_ne!(
        vehicle.mass_properties().center_of_mass_m,
        initial.center_of_mass_m
    );
    assert_ne!(
        vehicle.mass_properties().inertia_kg_m2,
        initial.inertia_kg_m2
    );
    assert!(
        vehicle
            .mass_properties()
            .inertia_kg_m2
            .symmetric_eigen()
            .eigenvalues
            .min()
            > 0.0
    );
}

#[test]
fn monarch_empty_feeds_retain_nitrogen_and_restore_on_reset() {
    let mut vehicle = Vehicle::new(profiles::monarch(), Box::new(ConstantSpecificImpulse));
    let initial = vehicle.mass_properties().clone();
    advance(&mut vehicle, 180.0 * 9.81, 100.0, FlightPhase::Descent);
    near(vehicle.mass_kg(), 60.88);
    for component in vehicle.components() {
        near(component.propellant_mass_kg, 0.0);
        assert!(
            component
                .inertia_kg_m2
                .iter()
                .all(|value| value.is_finite())
        );
    }
    vehicle.reset();
    assert_eq!(vehicle.mass_properties(), &initial);
}

#[test]
fn liquid_tank_uses_contents_centers_and_intrinsic_tensors() {
    let mut tank = MassComponent::new("test tank", 2.0, 2.0);
    let shell = Matrix3::from_diagonal(&Vector3::new(1.0, 2.0, 3.0));
    tank.geometry = Some(ComponentGeometry::LiquidTank(LiquidTankGeometry {
        shell_position_m: Vector3::zeros(),
        shell_inertia_kg_m2: shell,
        radius_m: 1.0,
        length_m: 4.0,
        full_liquid_mass_kg: 2.0,
        full_liquid_height_m: 2.0,
    }));
    tank.refresh_geometry();
    near(tank.position_m.z, -0.5);
    near(tank.inertia_kg_m2[(0, 0)], 19.0 / 6.0);
    near(tank.inertia_kg_m2[(1, 1)], 25.0 / 6.0);
    near(tank.inertia_kg_m2[(2, 2)], 4.0);
    tank.propellant_mass_kg = 0.0;
    tank.refresh_geometry();
    assert_eq!(tank.position_m, Vector3::zeros());
    assert_eq!(tank.inertia_kg_m2, shell);
    tank.additional_masses_kg.insert("gas".into(), 2.0);
    tank.refresh_geometry();
    assert_eq!(tank.position_m, Vector3::zeros());
    near(tank.inertia_kg_m2[(0, 0)], 25.0 / 6.0);
    near(tank.inertia_kg_m2[(1, 1)], 31.0 / 6.0);
    near(tank.inertia_kg_m2[(2, 2)], 4.0);
}

#[test]
fn engine_inertia_interpolates_and_nitrogen_can_be_updated_independently() {
    let mut parameters = profiles::monarch();
    let engine = parameters
        .mass_components
        .iter_mut()
        .find(|c| c.name == "engine")
        .unwrap();
    engine.propellant_mass_kg = 1.5;
    engine.refresh_geometry();
    near(engine.inertia_kg_m2[(0, 0)], 0.74);
    near(engine.inertia_kg_m2[(2, 2)], 0.065);
    let nitrogen = parameters
        .mass_components
        .iter_mut()
        .find(|c| c.name.starts_with("nitrogen tanks"))
        .unwrap();
    nitrogen.additional_masses_kg.insert("nitrogen".into(), 2.5);
    nitrogen.refresh_geometry();
    near(nitrogen.total_mass_kg(), 18.1);
    near(nitrogen.inertia_kg_m2[(0, 0)], 0.9675);
}

#[test]
fn external_mass_setter_updates_geometry_and_full_mpc_tensor_immediately() {
    let mut fsm = FlightStateMachine::new_with_vehicle(profiles::monarch());
    let initial = fsm.mass_properties();
    fsm.set_component_mass("nitrous tank", 15.86, 8.0);
    let properties = fsm.mass_properties();
    near(properties.mass_kg, 71.88);
    assert_ne!(properties.center_of_mass_m, initial.center_of_mass_m);
    assert_ne!(properties.inertia_kg_m2, initial.inertia_kg_m2);
    assert_ne!(properties.inertia_kg_m2[(0, 1)], 0.0);
    let mpc = fsm.autopilot_mut().mpc_mut().unwrap();
    assert_eq!(mpc.mass, properties.mass_kg);
    assert_eq!(mpc.inertia_kg_m2, properties.inertia_kg_m2);
    fsm.initialize();
    assert_eq!(fsm.mass_properties(), initial);
}

#[test]
fn edited_initial_masses_and_geometry_are_resolved_before_initialization() {
    let mut parameters = profiles::monarch();
    let tank = parameters
        .mass_components
        .iter_mut()
        .find(|c| c.name == "nitrous tank")
        .unwrap();
    tank.propellant_mass_kg = 4.0;
    if let Some(ComponentGeometry::LiquidTank(geometry)) = &mut tank.geometry {
        geometry.shell_position_m.x = 0.5;
    }
    let expected = parameters.mass_properties();
    let fsm = FlightStateMachine::new_with_vehicle(parameters);
    assert_eq!(fsm.mass_properties(), expected);
    assert!(fsm.mass_properties().center_of_mass_m.x > 0.0);
}

#[test]
fn explicit_legacy_initial_mass_survives_prelaunch() {
    let mut fsm = FlightStateMachine::new();
    let mut initial = crate::state::VehicleState::default();
    initial.mass = 65.0;
    initial.dry_mass = 45.0;
    fsm.initialize_with_state(initial, 0.0);
    fsm.step(&SensorData {
        timestamp: 0.002,
        imu_data: Some(crate::state::ImuSample::new(
            0.002,
            [0.0, 0.0, 9.81],
            [0.0; 3],
        )),
        gps_data: None,
        uwb_data: Some([0.0; 3]),
        chamber_pressure: Some(1.0),
        tank_pressure: Some(1.0),
    });
    assert_eq!(fsm.get_state().mass, 65.0);
    assert_eq!(fsm.get_state().vehicle_state.dry_mass, 45.0);
}

#[test]
fn removed_liquid_tank_does_not_contaminate_vehicle_geometry() {
    let mut fsm = FlightStateMachine::new_with_vehicle(profiles::monarch());
    fsm.set_component_mass("nitrous tank", 0.0, 0.0);
    let properties = fsm.mass_properties();
    near(properties.mass_kg, 48.02);
    assert!(properties.center_of_mass_m.iter().all(|v| v.is_finite()));
    assert!(properties.inertia_kg_m2.iter().all(|v| v.is_finite()));
}
