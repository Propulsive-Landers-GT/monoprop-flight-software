//! Component-based Monarch estimate. Dimensions and initial masses come from the
//! simulation's Rocket::default; these are starting estimates, not measured CAD data.

use nalgebra::{Matrix3, Vector3};

use super::super::{
    ComponentGeometry, DryWetInertia, LiquidTankGeometry, MassComponent, PropellantFeed,
    VehicleParameters,
};

/// Frame, nitrogen storage, one nitrous tank, and engine/TVC assembly.
/// Tank geometry and engine inertia update with their live contents. The common
/// body reference is the frame CoM; mounts are fixed, vehicle CoM is calculated.
pub fn monarch() -> VehicleParameters {
    let mut frame = MassComponent::new("frame", 15.66, 0.0);
    frame.inertia_kg_m2 = Matrix3::new(6.65, 0.35, -0.02, 0.35, 6.65, -0.02, -0.02, -0.02, 2.3);

    // TODO: Replace this combined N2 placeholder with each physical nitrogen tank.
    // Obtain accurate individual shell/gas masses, positions and dry/wet inertias.
    // Do not divide the aggregate values evenly without the actual tank data.
    let mut nitrogen = MassComponent::new("nitrogen tanks (combined placeholder)", 15.6, 0.0);
    nitrogen.additional_masses_kg.insert("nitrogen".into(), 5.0);
    nitrogen.position_m = Vector3::new(-0.02, -0.01, 0.76);
    nitrogen.geometry = Some(ComponentGeometry::DryWet(DryWetInertia {
        full_contents_mass_kg: 5.0,
        dry_inertia_kg_m2: diagonal(0.9275, 0.9275, 1.055),
        wet_inertia_kg_m2: diagonal(1.0075, 1.0075, 1.055),
    }));

    let mut oxidizer = MassComponent::new("nitrous tank", 15.86, 16.0);
    oxidizer
        .additional_masses_kg
        .insert("pressurant".into(), 0.0);
    oxidizer.geometry = Some(ComponentGeometry::LiquidTank(LiquidTankGeometry {
        shell_position_m: Vector3::new(-0.02, -0.01, 0.592),
        shell_inertia_kg_m2: diagonal(1.31, 1.31, 0.21),
        radius_m: 0.15,
        length_m: 0.75,
        full_liquid_mass_kg: 16.0,
        full_liquid_height_m: 0.85 * 0.75,
    }));

    let mut engine = MassComponent::new("engine", 8.76, 3.0);
    // Neutral gimbal: frame-to-pivot + pivot-to-engine CoM.
    engine.position_m = Vector3::new(-0.02, -0.01, -0.24 - 0.25);
    engine.geometry = Some(ComponentGeometry::DryWet(DryWetInertia {
        full_contents_mass_kg: 3.0,
        dry_inertia_kg_m2: diagonal(0.64, 0.64, 0.06),
        wet_inertia_kg_m2: diagonal(0.84, 0.84, 0.07),
    }));

    // Provisional constant mixture ratio from the initial oxidizer/fuel inventory.
    // Replace this approximation with calibrated hybrid EngineModel dynamics.
    let oxidizer_to_fuel_mass_ratio = 16.0 / 3.0;
    let fuel_fraction = 1.0 / (1.0 + oxidizer_to_fuel_mass_ratio);
    let mut parameters = super::old_monarch();
    parameters.engines[0].feeds = vec![
        PropellantFeed::new("nitrous tank", 1.0 - fuel_fraction),
        PropellantFeed::new("engine", fuel_fraction),
    ];
    parameters.mass_components = vec![frame, nitrogen, oxidizer, engine];
    for component in &mut parameters.mass_components {
        component.refresh_geometry();
    }
    parameters
}

fn diagonal(x: f64, y: f64, z: f64) -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(x, y, z))
}
