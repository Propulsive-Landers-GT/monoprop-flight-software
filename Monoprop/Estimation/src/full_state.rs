//! Full-state inertial navigation model: the reference implementation of `BareESEKFModel`.
//!
//! Nominal state: position p, velocity v (navigation frame), attitude q (body -> navigation),
//! accelerometer bias ab, gyro bias wb. Error state (15 numbers): [dp, dv, dtheta, dab, dwb], with a
//! local attitude error dtheta (J. Sola, "Quaternion kinematics for the error-state Kalman filter",
//! arXiv:1711.02508). Input: one IMU sample. Measurements: 3-D position (GPS or UWB), magnetometer
//! direction, height (barometer or downward lidar).
//!
//! Navigation frame: x east, y north, z up (ENU); gravity (0, 0, -g).

use super::filter::{BareESEKF, GATE_1DOF, GATE_3DOF, UpdateOutcome};
use super::math::*;
use super::model::BareESEKFModel;

// ---- environment and IMU parameters (set these for the vehicle) ----

/// Magnitude of gravity, m/s^2.
pub const GRAVITY: f64 = 9.81;

/// Gravity in the navigation frame (z up).
pub fn g_inertial() -> V3 {
    V3::new(0.0, 0.0, -GRAVITY)
}

/// The Earth's magnetic field in the navigation frame (ENU), tesla: 2 uT west of north, 22 uT north,
/// 44.3 uT down. Replace with the World Magnetic Model value for the test site.
pub fn mag_field_enu() -> V3 {
    V3::new(-2.0e-6, 22.0e-6, -44.3e-6)
}

/// Unit vector of the magnetic field: magnetometer updates and alignment only use its direction.
pub fn mag_reference() -> V3 {
    mag_field_enu().normalize()
}

/// IMU noise model used for the process noise Q.
#[derive(Clone, Copy, Debug)]
pub struct ImuSpec {
    pub sigma_a: f64,  // accelerometer white noise, per sample (m/s^2)
    pub sigma_g: f64,  // gyro white noise, per sample (rad/s)
    pub sigma_aw: f64, // accelerometer bias random walk intensity (m/s^2 per sqrt(s))
    pub sigma_gw: f64, // gyro bias random walk intensity (rad/s per sqrt(s))
}

/// VectorNav VN-200 datasheet values.
pub mod vn200 {
    use super::PI;
    /// Accelerometer noise density, m/s^2/sqrt(Hz): 0.14 mg/sqrt(Hz).
    pub const ACCEL_NOISE_DENSITY: f64 = 0.14e-3 * 9.80665;
    /// Gyro noise density, rad/s/sqrt(Hz): 0.0035 deg/s/sqrt(Hz).
    pub const GYRO_NOISE_DENSITY: f64 = 0.0035 * PI / 180.0;
    /// Accelerometer in-run bias stability, m/s^2: 0.04 mg.
    pub const ACCEL_BIAS_INSTABILITY: f64 = 0.04e-3 * 9.80665;
    /// Gyro in-run bias stability, rad/s: 10 deg/h (datasheet worst case).
    pub const GYRO_BIAS_INSTABILITY: f64 = 10.0 / 3600.0 * PI / 180.0;
    /// Bias random-walk intensities = instability / sqrt(tau), with an assumed tau of 100 s.
    pub const ACCEL_BIAS_RANDOM_WALK: f64 = ACCEL_BIAS_INSTABILITY / 10.0;
    pub const GYRO_BIAS_RANDOM_WALK: f64 = GYRO_BIAS_INSTABILITY / 10.0;
}

/// Per-sample standard deviation of white noise with density `density` (units/sqrt(Hz)) sampled at `rate_hz`.
pub fn per_sample_sigma(density: f64, rate_hz: f64) -> f64 {
    density * rate_hz.sqrt()
}

impl ImuSpec {
    /// From white-noise densities (per sqrt(Hz)) and bias random-walk intensities (per sqrt(s)),
    /// for an IMU sampled at `rate_hz`.
    pub fn from_densities(accel_density: f64, gyro_density: f64, accel_rw: f64, gyro_rw: f64, rate_hz: f64) -> ImuSpec {
        ImuSpec {
            sigma_a: per_sample_sigma(accel_density, rate_hz),
            sigma_g: per_sample_sigma(gyro_density, rate_hz),
            sigma_aw: accel_rw,
            sigma_gw: gyro_rw,
        }
    }

    /// The VN-200 sampled at `rate_hz`.
    pub fn vn200(rate_hz: f64) -> ImuSpec {
        ImuSpec::from_densities(
            vn200::ACCEL_NOISE_DENSITY,
            vn200::GYRO_NOISE_DENSITY,
            vn200::ACCEL_BIAS_RANDOM_WALK,
            vn200::GYRO_BIAS_RANDOM_WALK,
            rate_hz,
        )
    }
}

// ---- nominal state and strapdown propagation ----

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Nominal {
    pub p: V3,   // position, inertial (m)
    pub v: V3,   // velocity, inertial (m/s)
    pub q: Quat, // attitude, body -> inertial
    pub ab: V3,  // accelerometer bias estimate (m/s^2)
    pub wb: V3,  // gyro bias estimate (rad/s)
}

/// One strapdown step with accelerometer reading am and gyro reading wm (body axes).
pub fn propagate_nominal(x: &Nominal, am: &V3, wm: &V3, dt: f64) -> Nominal {
    let a = x.q * (am - x.ab) + g_inertial();
    let mut q = x.q * quat_exp(&((wm - x.wb) * dt));
    q.renormalize();
    Nominal {
        p: x.p + x.v * dt + a * (0.5 * dt * dt),
        v: x.v + a * dt,
        q,
        ab: x.ab,
        wb: x.wb,
    }
}

/// One IMU sample, body axes: specific force (m/s^2) and angular rate (rad/s).
#[derive(Clone, Copy, Debug)]
pub struct ImuSample {
    pub accel: V3,
    pub gyro: V3,
}

// ---- error state: injection, transition, process noise, reset ----

/// x (+) dx: apply a 15-number error to a nominal state (attitude: q * Exp(dtheta)).
pub fn inject(x: &Nominal, dx: &Vector<15>) -> Nominal {
    let mut q = x.q * quat_exp(&sub3(dx, 6));
    q.renormalize();
    Nominal {
        p: x.p + sub3(dx, 0),
        v: x.v + sub3(dx, 3),
        q,
        ab: x.ab + sub3(dx, 9),
        wb: x.wb + sub3(dx, 12),
    }
}

/// x_t (-) x: the 15-number error that takes x to x_t.
pub fn difference(x_t: &Nominal, x: &Nominal) -> Vector<15> {
    let mut d = Vector::<15>::zeros();
    d.fixed_rows_mut::<3>(0).copy_from(&(x_t.p - x.p));
    d.fixed_rows_mut::<3>(3).copy_from(&(x_t.v - x.v));
    d.fixed_rows_mut::<3>(6).copy_from(&boxminus(&x_t.q, &x.q));
    d.fixed_rows_mut::<3>(9).copy_from(&(x_t.ab - x.ab));
    d.fixed_rows_mut::<3>(12).copy_from(&(x_t.wb - x.wb));
    d
}

/// F: how an error in the nominal state becomes an error one IMU step later.
pub fn error_jacobian(x: &Nominal, am: &V3, wm: &V3, dt: f64) -> Mat<15, 15> {
    let r = quat_to_rot(&x.q);
    let mut f = Mat::<15, 15>::identity();
    set_block3(&mut f, 0, 3, &(M3::identity() * dt));
    set_block3(&mut f, 3, 6, &(-(r * skew(&(am - x.ab))) * dt));
    set_block3(&mut f, 3, 9, &(-r * dt));
    set_block3(&mut f, 6, 6, &exp_so3(&((wm - x.wb) * dt)).transpose());
    set_block3(&mut f, 6, 12, &(M3::identity() * -dt));
    f
}

/// Q: covariance of the new error added to the error state over one IMU step.
pub fn process_noise(spec: &ImuSpec, dt: f64) -> Mat<15, 15> {
    let mut q = Mat::<15, 15>::zeros();
    set_block3(&mut q, 3, 3, &(M3::identity() * (spec.sigma_a * spec.sigma_a * dt * dt)));
    set_block3(&mut q, 6, 6, &(M3::identity() * (spec.sigma_g * spec.sigma_g * dt * dt)));
    set_block3(&mut q, 9, 9, &(M3::identity() * (spec.sigma_aw * spec.sigma_aw * dt)));
    set_block3(&mut q, 12, 12, &(M3::identity() * (spec.sigma_gw * spec.sigma_gw * dt)));
    q
}

/// G: re-expresses the error covariance after the nominal attitude moved by dtheta.
pub fn reset_jacobian(dtheta: &V3) -> Mat<15, 15> {
    let mut g = Mat::<15, 15>::identity();
    set_block3(&mut g, 6, 6, &(M3::identity() - skew(dtheta) * 0.5));
    g
}

// ---- the model ----

/// The full-state model: IMU noise and the magnetic reference direction.
#[derive(Clone, Copy, Debug)]
pub struct FullState {
    pub imu: ImuSpec,
    pub mag_reference: V3,
}

impl FullState {
    pub fn new(imu: ImuSpec) -> Self {
        FullState { imu, mag_reference: mag_reference() }
    }
}

impl BareESEKFModel<15> for FullState {
    type Nominal = Nominal;
    type Input = ImuSample;

    fn input_is_valid(&self, u: &ImuSample, dt: f64) -> bool {
        dt > 0.0 && dt.is_finite() && u.accel.iter().chain(u.gyro.iter()).all(|v| v.is_finite())
    }

    fn nominal_prediction(&self, x: &Nominal, u: &ImuSample, dt: f64) -> Nominal {
        propagate_nominal(x, &u.accel, &u.gyro, dt)
    }

    fn error_transition_jacobian(&self, x: &Nominal, u: &ImuSample, dt: f64) -> Mat<15, 15> {
        error_jacobian(x, &u.accel, &u.gyro, dt)
    }

    fn process_noise(&self, _x: &Nominal, _u: &ImuSample, dt: f64) -> Mat<15, 15> {
        process_noise(&self.imu, dt)
    }

    fn inject_error(&self, x: &Nominal, dx: &Vector<15>) -> Nominal {
        inject(x, dx)
    }

    fn reset_jacobian(&self, dx: &Vector<15>) -> Mat<15, 15> {
        reset_jacobian(&sub3(dx, 6))
    }
}

/// The filter with the full-state model.
pub type FullStateESEKF = BareESEKF<FullState, 15>;

// ---- measurement models ----

/// GPS measures position: H = [I 0 0 0 0].
pub fn gps_jacobian() -> Mat<3, 15> {
    let mut h = Mat::<3, 15>::zeros();
    set_block3(&mut h, 0, 0, &M3::identity());
    h
}

/// What the magnetometer should read: the reference field in body axes, R^T m.
pub fn mag_prediction(x: &Nominal, m_ref: &V3) -> V3 {
    x.q.inverse() * m_ref
}

/// d(mag prediction)/d(error state): only the attitude block is non-zero, [R^T m]x.
pub fn mag_jacobian(x: &Nominal, m_ref: &V3) -> Mat<3, 15> {
    let mut h = Mat::<3, 15>::zeros();
    set_block3(&mut h, 0, 6, &skew(&mag_prediction(x, m_ref)));
    h
}

/// H for a height measurement (navigation-frame z): a single 1 at the position-error z column.
pub fn height_jacobian() -> Mat<1, 15> {
    let mut h = Mat::<1, 15>::zeros();
    h[(0, 2)] = 1.0;
    h
}

impl BareESEKF<FullState, 15> {
    /// A 3-D position fix (GPS or UWB, navigation frame) with per-axis standard deviation `sigma`.
    pub fn update_position(&mut self, z: &V3, sigma: f64) -> UpdateOutcome {
        let y = z - self.nominal_state.p;
        self.update(&y, &gps_jacobian(), &(M3::identity() * (sigma * sigma)), GATE_3DOF)
    }

    /// A magnetometer reading (body axes, any unit: only its direction is used). `sigma` is the
    /// per-axis noise of the unit direction (0.02 is about 1 degree).
    pub fn update_mag(&mut self, m: &V3, sigma: f64) -> UpdateOutcome {
        let n = m.norm();
        if !(n > 1e-12) || !n.is_finite() {
            self.rejected += 1;
            return UpdateOutcome::RejectedNonFinite;
        }
        let m_ref = self.model.mag_reference;
        let y = m / n - mag_prediction(&self.nominal_state, &m_ref);
        let h = mag_jacobian(&self.nominal_state, &m_ref);
        self.update(&y, &h, &(M3::identity() * (sigma * sigma)), GATE_3DOF)
    }

    /// A height measurement (barometer zeroed on the pad, or a downward lidar over flat ground), m.
    pub fn update_height(&mut self, z: f64, sigma: f64) -> UpdateOutcome {
        let y = Vector::<1>::new(z - self.nominal_state.p.z);
        self.update(&y, &height_jacobian(), &Mat::<1, 1>::new(sigma * sigma), GATE_1DOF)
    }
}

// ---- start-up ----

/// TRIAD: attitude (body -> inertial) from one accelerometer and one magnetometer reading.
/// At rest the accelerometer reads "up" (0, 0, 1) in inertial axes; it is the primary (trusted) vector.
/// None if the readings can't define an attitude: a zero vector, or the two vectors (almost) parallel.
pub fn triad(f_body: &V3, m_body: &V3, m_ref: &V3) -> Option<Quat> {
    let frame = |a: &V3, b: &V3| -> Option<M3> {
        let c = a.cross(b);
        if !(c.norm() > 0.01 * a.norm() * b.norm()) {
            return None; // parallel, zero or NaN: the second axis is undefined
        }
        let t1 = a.normalize();
        let t2 = c.normalize();
        let t3 = t1.cross(&t2);
        Some(M3::from_columns(&[t1, t2, t3]))
    };
    let r = frame(&V3::z(), m_ref)? * frame(f_body, m_body)?.transpose();
    Some(Quat::from_rotation_matrix(&na::Rotation3::from_matrix_unchecked(r)))
}

/// Initial uncertainty for an air start: position from one GPS fix, velocity unknown (a few m/s),
/// attitude from TRIAD (a few degrees), biases unknown up to their datasheet limits.
pub fn initial_covariance(sigma_gps: f64) -> Mat<15, 15> {
    let s2 = sigma_gps * sigma_gps;
    diag([
        s2, s2, s2,
        4.0, 4.0, 4.0,
        0.003, 0.003, 0.003,
        0.04, 0.04, 0.04,
        1e-3, 1e-3, 1e-3,
    ])
}

impl BareESEKF<FullState, 15> {
    /// Build the filter from readings taken with the vehicle still: accelerometer, magnetometer and a
    /// position fix. None if they can't define an attitude (a zero vector, the magnetometer parallel to
    /// gravity, a NaN): try again with the next readings.
    pub fn align(model: FullState, accel: &V3, mag: &V3, position: &V3, sigma_pos: f64) -> Option<Self> {
        if !position.iter().all(|v| v.is_finite()) {
            return None;
        }
        let q = triad(accel, mag, &model.mag_reference)?;
        let x = Nominal { p: *position, v: V3::zeros(), q, ab: V3::zeros(), wb: V3::zeros() };
        Some(BareESEKF::new(x, initial_covariance(sigma_pos), model))
    }
}
