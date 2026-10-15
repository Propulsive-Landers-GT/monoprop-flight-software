//! Combines two IMUs into one sample at the reference IMU: both readings are rotated
//! into the body frame, the gyros are averaged, and the other IMU's accelerometer is
//! moved to the reference point before averaging.
//!
//! At offset d from the reference, a rigid body reads
//!   f_other = f_ref + alpha x d + w x (w x d)   (tangential + centripetal),
//! and the same w everywhere. So:
//!   w = (w_ref + w_other) / 2,   f = (f_ref + f_other - w x (w x d)) / 2.
//!
//! The tangential term (alpha x d) is left out: alpha would need differenced gyro
//! samples, which add about as much noise as the accelerometer itself. The cost is
//! |alpha x d| / 2, about 0.015 m/s^2 per rad/s^2 with the IMUs 3 cm apart, and only
//! while the rotation rate is changing.
//!
//! TODO: revisit once the IMU layout is known. With the reference point at the
//! midpoint of the two IMUs, the plain average is exact and both terms vanish.
use crate::*;
use super::ImuSample;

/// One of the two IMUs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImuId {
    A,
    B,
}

/// How the two IMUs are mounted, which one is the reference, and how far apart in time two samples may
/// be and still count as the same instant.
#[derive(Clone, Copy, Debug)]
pub struct ImuPairConfig {
    pub reference: ImuId, // the IMU the combined sample is expressed at
    pub a_rotation: M3,   // IMU A axes -> body frame
    pub a_offset: V3,     // IMU A position in the body frame (m)
    pub b_rotation: M3,   // IMU B axes -> body frame
    pub b_offset: V3,     // IMU B position in the body frame (m)
    pub max_skew_us: u64, // largest timestamp difference of a pair that is combined (microseconds)
}

/// Why only one IMU was used.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SingleReason {
    /// The other IMU had no (valid) sample.
    OtherMissing,
    /// The two timestamps were more than `max_skew_us` apart: the reference IMU was used.
    TimestampMismatch,
}

/// Which IMUs went into a combined sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImuPairStatus {
    Both,
    Single { used: ImuId, reason: SingleReason },
}

/// One IMU sample at the reference IMU, in the body frame, and which IMUs it came from.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CombinedImu {
    pub sample: ImuSample,
    pub status: ImuPairStatus,
}

fn finite3(v: &V3) -> bool {
    v.iter().all(|x| x.is_finite())
}

fn finite33(m: &M3) -> bool {
    m.iter().all(|x| x.is_finite())
}

/// Centripetal specific force at offset d for rate w: w x (w x d).
fn centripetal(w: &V3, d: &V3) -> V3 {
    w.cross(&w.cross(d))
}

/// Combine the latest sample of IMU A and of IMU B (None: no valid sample) into one sample at the
/// reference IMU. Uses one IMU alone, and says so in `status`, when the other is missing or the two
/// timestamps are more than `cfg.max_skew_us` apart (then the reference wins). The result carries the
/// timestamp of the reference IMU, or of the IMU used. None if neither IMU has a sample, the config is
/// not finite, or the result would not be finite.
pub fn combine_imus(a: Option<&ImuSample>, b: Option<&ImuSample>, cfg: &ImuPairConfig) -> Option<CombinedImu> {
    if !(finite33(&cfg.a_rotation) && finite33(&cfg.b_rotation) && finite3(&cfg.a_offset) && finite3(&cfg.b_offset)) {
        return None;
    }
    // (reference, other) samples, mountings and ids
    let (ref_s, ref_r, ref_o, oth_s, oth_r, oth_o, oth_id) = match cfg.reference {
        ImuId::A => (a, cfg.a_rotation, cfg.a_offset, b, cfg.b_rotation, cfg.b_offset, ImuId::B),
        ImuId::B => (b, cfg.b_rotation, cfg.b_offset, a, cfg.a_rotation, cfg.a_offset, ImuId::A),
    };
    let d = oth_o - ref_o; // other IMU relative to the reference, body frame
    let single = |used: ImuId, reason: SingleReason| ImuPairStatus::Single { used, reason };
    let (t_us, accel, gyro, status) = match (ref_s, oth_s) {
        (Some(r), Some(o)) if r.t_us().abs_diff(o.t_us()) <= cfg.max_skew_us => {
            let w = (ref_r * r.gyro() + oth_r * o.gyro()) * 0.5;
            let f = (ref_r * r.accel() + oth_r * o.accel() - centripetal(&w, &d)) * 0.5;
            (r.t_us(), f, w, ImuPairStatus::Both)
        }
        (Some(r), Some(_)) => (r.t_us(), ref_r * r.accel(), ref_r * r.gyro(), single(cfg.reference, SingleReason::TimestampMismatch)),
        (Some(r), None) => (r.t_us(), ref_r * r.accel(), ref_r * r.gyro(), single(cfg.reference, SingleReason::OtherMissing)),
        (None, Some(o)) => {
            let w = oth_r * o.gyro();
            (o.t_us(), oth_r * o.accel() - centripetal(&w, &d), w, single(oth_id, SingleReason::OtherMissing))
        }
        (None, None) => return None,
    };
    ImuSample::new(t_us, accel, gyro).map(|sample| CombinedImu { sample, status })
}
