//! Functions only the tests use (not needed by the filter itself).
#![allow(dead_code)]
use Estimation::*;
use super::rng::*;

/// What an ideal accelerometer reads (body axes) for attitude q and inertial acceleration a_i.
pub fn specific_force(q: &Quat, a_i: &V3) -> V3 {
    q.inverse() * (a_i - g_inertial())
}
