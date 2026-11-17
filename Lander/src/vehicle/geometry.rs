//! Optional component geometry models, evaluated before vehicle CoM/inertia aggregation.

use nalgebra::{Matrix3, Vector3};

use super::MassComponent;

/// Geometry is separate from mass consumption: an engine/callback updates masses,
/// then this model updates the component's own CoM and intrinsic inertia.
#[derive(Debug, Clone)]
pub enum ComponentGeometry {
    /// Fixed mounting position with inertia interpolated by remaining contents.
    /// Contents = propellant + additional named masses; dry mass is the structure.
    DryWet(DryWetInertia),
    /// Upright cylindrical tank: shell, bottom-filled liquid and uniform ullage gas.
    /// Propellant is liquid; additional named masses are gas in the ullage.
    LiquidTank(LiquidTankGeometry),
}

#[derive(Debug, Clone)]
pub struct DryWetInertia {
    pub full_contents_mass_kg: f64,
    pub dry_inertia_kg_m2: Matrix3<f64>,
    pub wet_inertia_kg_m2: Matrix3<f64>,
}

/// All positions use the shared body reference; all tensors use body axes.
/// The tank axis is body Z. Fill height assumes constant liquid density.
#[derive(Debug, Clone)]
pub struct LiquidTankGeometry {
    pub shell_position_m: Vector3<f64>,
    pub shell_inertia_kg_m2: Matrix3<f64>,
    pub radius_m: f64,
    pub length_m: f64,
    pub full_liquid_mass_kg: f64,
    pub full_liquid_height_m: f64,
}

impl ComponentGeometry {
    pub(crate) fn properties(&self, component: &MassComponent) -> (Vector3<f64>, Matrix3<f64>) {
        match self {
            Self::DryWet(model) => {
                let contents = component.propellant_mass_kg
                    + component.additional_masses_kg.values().sum::<f64>();
                let fraction = contents / model.full_contents_mass_kg;
                let inertia = model.dry_inertia_kg_m2
                    + fraction * (model.wet_inertia_kg_m2 - model.dry_inertia_kg_m2);
                (component.position_m, inertia)
            }
            Self::LiquidTank(model) => model.properties(component),
        }
    }
}

impl LiquidTankGeometry {
    fn properties(&self, component: &MassComponent) -> (Vector3<f64>, Matrix3<f64>) {
        // A removed/massless tank contributes no inertia; retain its mount as the
        // reference position rather than attempting to divide by zero total mass.
        if component.total_mass_kg() == 0.0 {
            return (self.shell_position_m, Matrix3::zeros());
        }
        let liquid_mass = component.propellant_mass_kg;
        let gas_mass = component.additional_masses_kg.values().sum::<f64>();
        let height = self.full_liquid_height_m * liquid_mass / self.full_liquid_mass_kg;
        let liquid_position =
            self.shell_position_m + Vector3::new(0.0, 0.0, (height - self.length_m) / 2.0);
        let gas_position = self.shell_position_m + Vector3::new(0.0, 0.0, height / 2.0);
        let center = (component.dry_mass_kg * self.shell_position_m
            + liquid_mass * liquid_position
            + gas_mass * gas_position)
            / component.total_mass_kg();

        let inertia = self.shell_inertia_kg_m2
            + parallel_axis(component.dry_mass_kg, self.shell_position_m - center)
            + cylinder_inertia(liquid_mass, self.radius_m, height)
            + parallel_axis(liquid_mass, liquid_position - center)
            + cylinder_inertia(gas_mass, self.radius_m, self.length_m - height)
            + parallel_axis(gas_mass, gas_position - center);
        (center, inertia)
    }
}

fn cylinder_inertia(mass: f64, radius: f64, height: f64) -> Matrix3<f64> {
    let transverse = mass * (3.0 * radius * radius + height * height) / 12.0;
    Matrix3::from_diagonal(&Vector3::new(
        transverse,
        transverse,
        mass * radius * radius / 2.0,
    ))
}

fn parallel_axis(mass: f64, offset: Vector3<f64>) -> Matrix3<f64> {
    mass * (offset.norm_squared() * Matrix3::identity() - offset * offset.transpose())
}
