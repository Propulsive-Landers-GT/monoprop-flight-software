use super::*;
fn model() -> config::Model {
    // Synthetic test fixture, not a measured or approved vehicle model.
    config::Model {
        mass_kg: 1.05,
        inertia_kg_m2: [0.02, 0.02, 0.01],
        lever_arm_m: 0.15,
        thrust_curve: vec![[0.15, 2.0], [0.5, 10.3005], [0.9, 18.0]],
        gimbal_sign: [1., 1.],
        hover_height_m: 1.0,
    }
}
#[test]
fn estimated_model_is_accepted_but_invalid_parameters_are_rejected() {
    let m = model();
    m.validate().unwrap();
    assert!((m.throttle(10.3005).unwrap() - 0.5).abs() < 1e-6);
    assert!(m.throttle(300.).is_err());
    let configured: config::Model = toml::from_str(include_str!("../ground.toml")).unwrap();
    configured.validate().unwrap();
    let mut invalid = configured.clone();
    invalid.inertia_kg_m2[0] = 0.0;
    assert!(invalid.validate().is_err());
}
#[test]
fn modern_framing_and_quaternion_order() {
    let mut t = TelemetryFrame::default();
    t.vehicle_state.attitude_quat = [0.5, 0.5, 0.5, 0.5];
    t.sensor_data.timestamp_us = 123;
    let wire = postcard::to_stdvec_cobs(&t).unwrap();
    assert_eq!(decode(&wire).unwrap().sensor_data.timestamp_us, 123);
    assert!(decode(&postcard::to_stdvec(&t).unwrap()).is_err());
    assert!(decode(&postcard::to_stdvec_cobs(&t.as_legacy()).unwrap()).is_err());
    t.vehicle_state.attitude_quat = [1., 0., 0., 0.];
    assert_eq!(t.as_legacy().orientation, [0., 0., 0., 1.]);
}
#[test]
fn equilibrium_hover_and_vertical_response() {
    let mut t = TelemetryFrame::default();
    t.phase = FlightPhase::Hover;
    t.vehicle_state.attitude_quat = [1., 0., 0., 0.];
    t.vehicle_state.mass_kg = 1.05;
    t.vehicle_state.position_m = [0., 0., 1.];
    let mut controller = controller::Controller::new(model());
    let command = controller.solve(&t, 7).unwrap();
    assert_eq!(command.sequence, 7);
    assert!(
        (command.actuator_command.thrust_norm - 0.5).abs() < 0.01,
        "{:?}",
        command
    );
    assert!(
        command
            .actuator_command
            .gimbal_rad
            .iter()
            .all(|v| v.abs() < 1e-3)
    );
    t.vehicle_state.position_m[2] = 0.8;
    let command = controller.solve(&t, 8).unwrap();
    assert!(command.actuator_command.thrust_norm > 0.5);
}

#[test]
fn configured_cylinder_model_and_linear_curve_hover() {
    let m: config::Model = toml::from_str(include_str!("../ground.toml")).unwrap();
    m.validate().unwrap();
    let h: f64 = 22.5 * 0.0254;
    let r: f64 = 3.0 * 0.0254;
    assert!((m.lever_arm_m - 7.25 * 0.0254).abs() < 1e-12);
    assert!((m.inertia_kg_m2[0] - m.mass_kg * (3.0 * r * r + h * h) / 12.0).abs() < 1e-12);
    assert!((m.inertia_kg_m2[2] - m.mass_kg * r * r / 2.0).abs() < 1e-12);
    assert!((m.throttle(0.0).unwrap() - 0.15).abs() < 1e-6);
    assert!((m.throttle(8.0).unwrap() - 0.575).abs() < 1e-6);
    assert_eq!(m.throttle(16.0).unwrap(), 1.0);
    assert!(m.throttle(-0.01).is_err());
    assert!(m.throttle(16.01).is_err());
    let expected = 0.15 + 0.85 * (m.mass_kg * 9.81) / 16.0;
    let mut t = TelemetryFrame::default();
    t.phase = FlightPhase::Hover;
    t.vehicle_state.mass_kg = m.mass_kg as f32;
    t.vehicle_state.position_m = [0., 0., m.hover_height_m as f32];
    t.vehicle_state.attitude_quat = [1., 0., 0., 0.];
    let mut c = controller::Controller::new(m);
    let command = c.solve(&t, 1).unwrap();
    assert!((command.actuator_command.thrust_norm as f64 - expected).abs() < 0.01);
    t.vehicle_state.position_m[2] -= 0.2;
    assert!(
        c.solve(&t, 2).unwrap().actuator_command.thrust_norm > command.actuator_command.thrust_norm
    );
}

#[test]
fn displaced_telemetry_regression_converges_without_relaxing_tolerance() {
    // Captured relative state that exhausted the old 150-iteration allowance.
    // This tests numerical behavior; it is not a valid launch-position assumption.
    let m: config::Model = toml::from_str(include_str!("../ground.toml")).unwrap();
    let mut c = controller::Controller::new(m);
    let mut t = TelemetryFrame::default();
    t.phase = FlightPhase::Hover;
    t.vehicle_state.mass_kg = 1.05;
    t.vehicle_state.position_m = [-2.506249, -2.5520184, 5.2469606];
    t.vehicle_state.velocity_mps = [0.0009580564, 0.0010729904, -0.0014371923];
    t.vehicle_state.attitude_quat = [0.94459844, -0.008561156, -0.00019549993, -0.32811654];
    t.vehicle_state.angular_velocity_rps = [0.000908615, 0.0018145887, 0.003240732];
    for sequence in 1..=25 {
        let command = c.solve(&t, sequence).unwrap().actuator_command;
        assert!((0.0..=1.0).contains(&command.thrust_norm));
        assert!(
            command.gimbal_rad[0].hypot(command.gimbal_rad[1])
                <= monoprop_common::MAX_TVC_GIMBAL_RAD + 1e-6
        );
    }
}

#[path = "../examples/support/mod.rs"]
mod offline_support;

#[test]
fn return_reference_preserves_incoming_state_and_stops_at_origin() {
    let mut t = TelemetryFrame::default();
    t.vehicle_state.position_m = [-10.19387, -5.342737, 21.068245];
    t.vehicle_state.velocity_mps = [0.090914875, 0.12512952, -0.20976207];
    t.vehicle_state.attitude_quat = [1., 0., 0., 0.];
    let reference = offline_support::ReturnReference::new(&t);
    let start = reference.at(0.0);
    let end = reference.at(reference.duration_s);
    for i in 0..3 {
        assert!((start[i] - t.vehicle_state.position_m[i] as f64).abs() < 1e-10);
        assert!((start[7 + i] - t.vehicle_state.velocity_mps[i] as f64).abs() < 1e-10);
        assert!(end[i].abs() < 1e-10);
        assert!(end[7 + i].abs() < 1e-10);
    }
    assert_eq!(
        reference.horizon(0.0).len(),
        monoprop_common::MPC_HORIZON_STEPS + 1
    );
    let m: config::Model = toml::from_str(include_str!("../ground.toml")).unwrap();
    let mut c = controller::Controller::new(m);
    assert!(c.solve_tracking(&t, 1, &reference.horizon(0.0)).is_ok());
    assert!(c.solve_tracking(&t, 2, &vec![]).is_err());
}
