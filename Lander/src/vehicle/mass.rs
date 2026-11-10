use std::collections::BTreeMap;

use nalgebra::{Matrix3, Vector3};

use crate::state::{FlightPhase, SensorData};

/// Inputs for one component's update, after the engine dynamics method has run.
/// The callback applies the draw queued by that model. The default engine limits
/// draws by available propellant and shared feed ratios; custom models own that policy.
pub struct MassUpdateContext<'a> {
    pub dt_s: f64,
    pub sensors: &'a SensorData,
    pub flight_phase: FlightPhase,
    pub propellant_consumed_kg: f64,
    /// Original configuration, useful for geometry, fill fractions and reset values.
    pub initial: &'a MassComponent,
}

/// A stateless function or non-capturing closure. Keep changing state in the
/// component so cloning and reinitialization behave predictably.
pub type MassUpdateFn = fn(&mut MassComponent, &MassUpdateContext<'_>);

/// One named contribution to vehicle mass. All geometry uses the same body axes.
/// For parts with different centers of mass, use separate components.
#[derive(Debug, Clone)]
pub struct MassComponent {
    pub name: String,
    pub dry_mass_kg: f64,
    pub propellant_mass_kg: f64,
    /// Other non-engine-propellant masses, e.g. pressurant or retained payload.
    /// These contribute to total and dry mass; a custom updater may change them.
    pub additional_masses_kg: BTreeMap<String, f64>,
    /// Vector from the common body-fixed reference point to this component's CoM [m].
    pub position_m: Vector3<f64>,
    /// Inertia of the entire component about its own CoM, in body axes [kg m²].
    /// Includes dry, propellant and additional masses. Zero models a point mass.
    pub inertia_kg_m2: Matrix3<f64>,
    pub update_mass: MassUpdateFn,
}

impl MassComponent {
    /// Existing three-argument construction remains available. Geometry defaults to
    /// a point mass at the reference point; provide measured geometry as needed.
    pub fn new(name: impl Into<String>, dry_mass_kg: f64, propellant_mass_kg: f64) -> Self {
        Self {
            name: name.into(),
            dry_mass_kg,
            propellant_mass_kg,
            additional_masses_kg: BTreeMap::new(),
            position_m: Vector3::zeros(),
            inertia_kg_m2: Matrix3::zeros(),
            update_mass: Self::deplete_propellant,
        }
    }

    pub fn total_mass_kg(&self) -> f64 {
        self.retained_mass_kg() + self.propellant_mass_kg
    }

    /// Everything except the propellant available to the engine feed system.
    pub fn retained_mass_kg(&self) -> f64 {
        self.dry_mass_kg + self.additional_masses_kg.values().sum::<f64>()
    }

    /// Default updater. A custom updater can call this before updating other mass
    /// terms or recalculating geometry. Intrinsic inertia is not automatically scaled.
    pub fn deplete_propellant(&mut self, context: &MassUpdateContext<'_>) {
        self.propellant_mass_kg =
            (self.propellant_mass_kg - context.propellant_consumed_kg).max(0.0);
    }
}

/// Aggregate geometry, expressed in the common body axes. This is independent of
/// vehicle attitude/world position and the existing MPC's configured inertia.
#[derive(Debug, Clone, PartialEq)]
pub struct MassProperties {
    pub mass_kg: f64,
    pub dry_mass_kg: f64,
    /// Vector from the shared reference point to the vehicle CoM [m].
    pub center_of_mass_m: Vector3<f64>,
    /// Full vehicle inertia about the vehicle CoM, including parallel-axis terms.
    pub inertia_kg_m2: Matrix3<f64>,
}

impl MassProperties {
    pub fn from_components(components: &[MassComponent]) -> Self {
        let mass_kg: f64 = components.iter().map(MassComponent::total_mass_kg).sum();
        let dry_mass_kg: f64 = components.iter().map(MassComponent::retained_mass_kg).sum();
        let center_of_mass_m = components.iter().fold(Vector3::zeros(), |sum, c| {
            sum + c.position_m * (c.total_mass_kg() / mass_kg)
        });
        let mut inertia_kg_m2 = Matrix3::zeros();
        for component in components {
            let mass = component.total_mass_kg();
            if mass == 0.0 {
                continue;
            }
            let offset = component.position_m - center_of_mass_m;
            inertia_kg_m2 += component.inertia_kg_m2
                + mass
                    * (offset.norm_squared() * Matrix3::identity() - offset * offset.transpose());
        }
        Self {
            mass_kg,
            dry_mass_kg,
            center_of_mass_m,
            inertia_kg_m2,
        }
    }
}
