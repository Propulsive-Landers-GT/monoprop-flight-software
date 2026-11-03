//! Two IMUs into one sample at the reference IMU (body frame), on the reference IMU's clock.
//!
//! The other IMU's accelerometer is moved to the reference point with the centripetal term w x (w x d). The
//! tangential term alpha x d is left out: alpha from gyro differences would add about as much noise as the
//! accelerometer's own; leaving it out costs |alpha x d| / 2 while the rate is changing.
//!
//! A combined sample carries the mean of the two biases. A sample from one IMU alone is corrected by half the
//! learned reference-minus-other difference, so its bias doesn't step on a dropout. This corrects the
//! difference, not the biases, and follows a drifting difference with a lag of about `diff_window` pairs.
use Estimation::{M3, V3};

use crate::config::Config;
use crate::sensors::ImuSample;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImuId {
    A,
    B,
}

#[derive(Clone, Copy, Debug)]
pub struct ImuPair {
    reference: ImuId,
    ref_rot: M3, // reference IMU axes -> body
    oth_rot: M3, // other IMU axes -> body
    d: V3,       // other IMU's position minus the reference's, body frame (m)
    max_skew_us: u64,
    window: u32,
    min_pairs: u32,
    diff_accel: V3, // reference minus other, at the reference point
    diff_gyro: V3,
    pairs: u32,
    offset_us: i64, // other IMU's clock minus the reference's
}

fn v3(a: [f64; 3]) -> V3 {
    V3::new(a[0], a[1], a[2])
}

fn m3(r: [[f64; 3]; 3]) -> M3 {
    M3::new(r[0][0], r[0][1], r[0][2], r[1][0], r[1][1], r[1][2], r[2][0], r[2][1], r[2][2])
}

fn centripetal(w: &V3, d: &V3) -> V3 {
    w.cross(&w.cross(d))
}

impl ImuPair {
    pub fn new(c: &Config) -> Self {
        let (a_rot, b_rot) = (m3(c.imu_pair_a_rotation), m3(c.imu_pair_b_rotation));
        let (a_off, b_off) = (v3(c.imu_pair_a_offset), v3(c.imu_pair_b_offset));
        let (ref_rot, oth_rot, d) = match c.imu_pair_reference {
            ImuId::A => (a_rot, b_rot, b_off - a_off),
            ImuId::B => (b_rot, a_rot, a_off - b_off),
        };
        ImuPair {
            reference: c.imu_pair_reference,
            ref_rot,
            oth_rot,
            d,
            max_skew_us: c.imu_pair_max_skew_us,
            window: c.imu_pair_diff_window,
            min_pairs: c.imu_pair_diff_min_pairs,
            diff_accel: V3::zeros(),
            diff_gyro: V3::zeros(),
            pairs: 0,
            offset_us: 0,
        }
    }

    pub fn pairs(&self) -> u32 {
        self.pairs
    }

    /// One sample from the newest readings of IMU A and B (None: no reading); None if neither has one.
    pub fn combine(&mut self, a: Option<&ImuSample>, b: Option<&ImuSample>) -> Option<ImuSample> {
        let (r, o) = match self.reference {
            ImuId::A => (a, b),
            ImuId::B => (b, a),
        };
        let half = if self.pairs >= self.min_pairs { 0.5 } else { 0.0 };
        let (accel, gyro, t_us) = match (r, o) {
            (Some(r), Some(o)) if r.t_us.abs_diff(o.t_us) <= self.max_skew_us => {
                let (fr, wr) = (self.ref_rot * v3(r.accel), self.ref_rot * v3(r.gyro));
                let wo = self.oth_rot * v3(o.gyro);
                let w = (wr + wo) * 0.5;
                let fo = self.oth_rot * v3(o.accel) - centripetal(&w, &self.d);
                self.learn(fr - fo, wr - wo, o.t_us, r.t_us);
                ((fr + fo) * 0.5, w, r.t_us)
            }
            (Some(r), _) => {
                let (f, w) = (self.ref_rot * v3(r.accel), self.ref_rot * v3(r.gyro));
                (f - self.diff_accel * half, w - self.diff_gyro * half, r.t_us)
            }
            (None, Some(o)) => {
                let w = self.oth_rot * v3(o.gyro);
                let f = self.oth_rot * v3(o.accel) - centripetal(&w, &self.d);
                let t = self.offset_us.checked_neg().and_then(|n| o.t_us.checked_add_signed(n)).unwrap_or(o.t_us);
                (f + self.diff_accel * half, w + self.diff_gyro * half, t)
            }
            (None, None) => return None,
        };
        Some(ImuSample { t_us, accel: accel.into(), gyro: gyro.into() })
    }

    fn learn(&mut self, da: V3, dg: V3, t_other: u64, t_ref: u64) {
        if let Ok(off) = i64::try_from(i128::from(t_other) - i128::from(t_ref)) {
            self.offset_us = off;
        }
        self.pairs = self.pairs.saturating_add(1);
        let k = 1.0 / f64::from(self.pairs.min(self.window.max(1)));
        self.diff_accel += (da - self.diff_accel) * k;
        self.diff_gyro += (dg - self.diff_gyro) * k;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::CONFIG;

    fn reading(t_us: u64, f: V3, w: V3) -> ImuSample {
        ImuSample { t_us, accel: f.into(), gyro: w.into() }
    }

    #[test]
    fn centripetal_correction_is_exact_for_a_steady_spin() {
        let d = V3::new(0.05, 0.02, 0.0);
        let mut c = CONFIG;
        c.imu_pair_b_offset = d.into();
        let mut pair = ImuPair::new(&c);
        let (w, f_ref) = (V3::new(0.4, -0.3, 1.2), V3::new(0.2, -0.1, 9.81));
        let s = pair.combine(Some(&reading(0, f_ref, w)), Some(&reading(0, f_ref + centripetal(&w, &d), w)));
        assert!(s.is_some_and(|s| (v3(s.accel) - f_ref).amax() < 1e-12));
    }

    #[test]
    fn a_single_imu_carries_the_mean_bias_and_the_reference_clock() {
        for reference in [ImuId::A, ImuId::B] {
            let mut c = CONFIG;
            c.imu_pair_reference = reference;
            let mut pair = ImuPair::new(&c);
            let (f, w) = (V3::new(0.0, 0.0, 9.81), V3::zeros());
            let (ba, bb) = ((V3::new(0.03, -0.07, 0.15), V3::new(0.004, 0.0, -0.002)), (V3::new(-0.02, 0.0, -0.05), V3::new(-0.001, 0.003, 0.0)));
            let at = |t: u64, bias: (V3, V3), skew: u64| reading(t + skew, f + bias.0, w + bias.1);
            // learned from the first pairs, before any alignment; B's clock is 300 us ahead of A's
            let b_skew = 300;
            let mut both = None;
            for k in 0..c.imu_pair_diff_min_pairs as u64 {
                both = pair.combine(Some(&at(k * 2500, ba, 0)), Some(&at(k * 2500, bb, b_skew)));
            }
            assert_eq!(pair.pairs(), c.imu_pair_diff_min_pairs, "learned from the first pairs");
            let Some(both) = both else { return assert!(both.is_some()) };
            let t = 400 * 2500;
            for (only_a, only_b) in [(true, false), (false, true)] {
                let s = pair.combine(only_a.then(|| at(t, ba, 0)).as_ref(), only_b.then(|| at(t, bb, b_skew)).as_ref());
                let Some(s) = s else { return assert!(s.is_some()) };
                assert!((v3(s.accel) - v3(both.accel)).amax() < 1e-12, "{:?} reference, A {} B {}: accel", reference, only_a, only_b);
                assert!((v3(s.gyro) - v3(both.gyro)).amax() < 1e-12, "{:?} reference, A {} B {}: gyro", reference, only_a, only_b);
                let expected = if reference == ImuId::A { t } else { t + b_skew };
                assert_eq!(s.t_us, expected, "on the reference clock");
            }
        }
    }
}
