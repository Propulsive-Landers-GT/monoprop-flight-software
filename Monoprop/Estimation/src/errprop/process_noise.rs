use crate::*;

/// Q: covariance of the new error added to the error state over one IMU step.
pub fn process_noise(spec: &ImuSpec, dt: f64) -> Mat<15, 15> {
    let mut q = Mat::<15, 15>::zeros();
    set_block3(&mut q, 3, 3, &(M3::identity() * (spec.sigma_a * spec.sigma_a * dt * dt)));
    set_block3(&mut q, 6, 6, &(M3::identity() * (spec.sigma_g * spec.sigma_g * dt * dt)));
    set_block3(&mut q, 9, 9, &(M3::identity() * (spec.sigma_aw * spec.sigma_aw * dt)));
    set_block3(&mut q, 12, 12, &(M3::identity() * (spec.sigma_gw * spec.sigma_gw * dt)));
    q
}
