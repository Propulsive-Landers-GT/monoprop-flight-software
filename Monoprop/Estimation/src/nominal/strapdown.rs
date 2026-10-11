use crate::*;

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
