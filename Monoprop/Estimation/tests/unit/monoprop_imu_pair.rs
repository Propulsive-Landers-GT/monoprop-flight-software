// Tests for src/monoprop/imu_pair.rs.
#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::monoprop as mp;
use Estimation::*;

/// What an IMU mounted with rotation `r` (its axes -> body) at offset `d` from the reference point reads,
/// for body rate w, angular acceleration alpha and specific force f_ref at the reference point.
fn reading(t_us: u64, r: &M3, d: &V3, w: &V3, alpha: &V3, f_ref: &V3) -> Option<mp::ImuSample> {
    let f = f_ref + alpha.cross(d) + w.cross(&w.cross(d));
    let s = mp::ImuSample::new(t_us, r.transpose() * f, r.transpose() * w);
    assert!(s.is_some(), "test reading must be finite");
    s
}

fn cfg(reference: mp::ImuId, b_rotation: M3, b_offset: V3) -> mp::ImuPairConfig {
    mp::ImuPairConfig {
        reference,
        a_rotation: M3::identity(),
        a_offset: V3::zeros(),
        b_rotation,
        b_offset,
        max_skew_us: 1000,
    }
}

fn rot(v: V3) -> M3 {
    quat_to_rot(&quat_exp(&v))
}

/// The combined sample, failing the test (assert) if there is none.
fn combined(a: Option<&mp::ImuSample>, b: Option<&mp::ImuSample>, c: &mp::ImuPairConfig) -> Option<mp::CombinedImu> {
    let out = mp::combine_imus(a, b, c);
    assert!(out.is_some(), "a combined sample");
    out
}

pub fn run() {
    let w = V3::new(0.4, -0.3, 1.2);
    let f_ref = V3::new(0.2, -0.1, 9.81);
    let zero = V3::zeros();

    test("same mounting, zero offset: the plain average, from both IMUs", || {
        let a = mp::ImuSample::new(10, V3::new(0.1, 0.0, 9.8), V3::new(0.01, 0.02, 0.03));
        let b = mp::ImuSample::new(10, V3::new(0.3, 0.2, 9.9), V3::new(0.03, 0.00, 0.05));
        assert!(a.is_some() && b.is_some(), "valid samples");
        let (Some(a), Some(b)) = (a, b) else { return };
        let Some(out) = combined(Some(&a), Some(&b), &cfg(mp::ImuId::A, M3::identity(), zero)) else { return };
        assert_eq!(out.status, mp::ImuPairStatus::Both);
        assert!((out.sample.accel() - V3::new(0.2, 0.1, 9.85)).amax() < 1e-12, "accel average");
        assert!((out.sample.gyro() - V3::new(0.02, 0.01, 0.04)).amax() < 1e-12, "gyro average");
        assert_eq!(out.sample.t_us(), 10);
    });
    test("mounting rotations are applied", || {
        for rb in [rot(V3::new(0.0, 0.0, FRAC_PI_2)), rot(V3::new(FRAC_PI_2, 0.0, 0.0)), rot(V3::new(0.3, -0.7, 2.0))] {
            let ra = rot(V3::new(0.1, 0.2, -0.3));
            let c = mp::ImuPairConfig { a_rotation: ra, ..cfg(mp::ImuId::A, rb, zero) };
            let Some(a) = reading(5, &ra, &zero, &w, &zero, &f_ref) else { return };
            let Some(b) = reading(5, &rb, &zero, &w, &zero, &f_ref) else { return };
            let Some(out) = combined(Some(&a), Some(&b), &c) else { return };
            assert!((out.sample.accel() - f_ref).amax() < 1e-12, "accel back in body axes");
            assert!((out.sample.gyro() - w).amax() < 1e-12, "gyro back in body axes");
        }
    });
    test("centripetal correction: a steady spin gives the reference point's specific force", || {
        let d = V3::new(0.05, 0.02, 0.0);
        let c = cfg(mp::ImuId::A, M3::identity(), d);
        let Some(a) = reading(0, &M3::identity(), &zero, &w, &zero, &f_ref) else { return };
        let Some(b) = reading(0, &M3::identity(), &d, &w, &zero, &f_ref) else { return };
        let Some(out) = combined(Some(&a), Some(&b), &c) else { return };
        assert!((out.sample.accel() - f_ref).amax() < 1e-12, "corrected");
        let naive = (a.accel() + b.accel()) * 0.5;
        let expected = w.cross(&w.cross(&d)) * 0.5;
        assert!(((naive - f_ref) - expected).amax() < 1e-12, "a plain average would be off by w x (w x d) / 2");
        assert!(expected.norm() > 1e-3, "the correction is not trivially small here");
    });
    test("the tangential part is left out: the error is (alpha x d) / 2, as documented", || {
        let d = V3::new(0.03, -0.01, 0.02);
        let alpha = V3::new(5.0, -10.0, 3.0);
        let c = cfg(mp::ImuId::A, M3::identity(), d);
        let Some(a) = reading(0, &M3::identity(), &zero, &w, &alpha, &f_ref) else { return };
        let Some(b) = reading(0, &M3::identity(), &d, &w, &alpha, &f_ref) else { return };
        let Some(out) = combined(Some(&a), Some(&b), &c) else { return };
        assert!(((out.sample.accel() - f_ref) - alpha.cross(&d) * 0.5).amax() < 1e-12);
    });
    test("one IMU missing: the other alone, and the status says which and why", || {
        let d = V3::new(0.05, 0.02, -0.01);
        for reference in [mp::ImuId::A, mp::ImuId::B] {
            let c = mp::ImuPairConfig { reference, ..cfg(reference, M3::identity(), d) };
            let other = if reference == mp::ImuId::A { mp::ImuId::B } else { mp::ImuId::A };
            // A at the origin, B at d; the reference point is the reference IMU
            let (off_ref, off_oth) = if reference == mp::ImuId::A { (zero, d) } else { (d, zero) };
            let f_at_ref = f_ref + w.cross(&w.cross(&off_ref));
            let Some(r_ref) = reading(7, &M3::identity(), &zero, &w, &zero, &f_at_ref) else { return };
            let Some(r_oth) = reading(7, &M3::identity(), &(off_oth - off_ref), &w, &zero, &f_at_ref) else { return };
            let (a, b) = if reference == mp::ImuId::A { (r_ref, r_oth) } else { (r_oth, r_ref) };
            let (Some(only_a), Some(only_b)) = (combined(Some(&a), None, &c), combined(None, Some(&b), &c)) else { return };
            let (only_ref, only_oth) = if reference == mp::ImuId::A { (only_a, only_b) } else { (only_b, only_a) };
            assert_eq!(only_ref.status, mp::ImuPairStatus::Single { used: reference, reason: mp::SingleReason::OtherMissing });
            assert_eq!(only_oth.status, mp::ImuPairStatus::Single { used: other, reason: mp::SingleReason::OtherMissing });
            assert!((only_ref.sample.accel() - f_at_ref).amax() < 1e-12, "reference alone");
            assert!((only_oth.sample.accel() - f_at_ref).amax() < 1e-12, "other alone, moved to the reference");
            assert!((only_oth.sample.gyro() - w).amax() < 1e-12, "gyro");
            assert!(mp::combine_imus(None, None, &c).is_none(), "neither");
        }
    });
    test("timestamps: within the skew combine, beyond it the reference is used alone", || {
        let c = cfg(mp::ImuId::B, M3::identity(), zero);
        let at = |t: u64| reading(t, &M3::identity(), &zero, &w, &zero, &f_ref);
        let Some(both) = combined(at(5000).as_ref(), at(6000).as_ref(), &c) else { return };
        assert_eq!(both.status, mp::ImuPairStatus::Both, "1000 us apart, limit 1000 us");
        assert_eq!(both.sample.t_us(), 6000, "the reference's (B's) timestamp");
        let Some(apart) = combined(at(5000).as_ref(), at(6001).as_ref(), &c) else { return };
        assert_eq!(apart.status, mp::ImuPairStatus::Single { used: mp::ImuId::B, reason: mp::SingleReason::TimestampMismatch });
        assert_eq!(apart.sample.t_us(), 6001);
        let Some(extreme) = combined(at(0).as_ref(), at(u64::MAX).as_ref(), &c) else { return };
        assert_eq!(extreme.status, mp::ImuPairStatus::Single { used: mp::ImuId::B, reason: mp::SingleReason::TimestampMismatch });
        assert_eq!(extreme.sample.t_us(), u64::MAX, "no overflow");
    });
    test("a non-finite configuration gives no sample", || {
        let Some(a) = reading(0, &M3::identity(), &zero, &w, &zero, &f_ref) else { return };
        let mut bad_rot = M3::identity();
        bad_rot[(1, 1)] = f64::NAN;
        let c1 = mp::ImuPairConfig { a_rotation: bad_rot, ..cfg(mp::ImuId::A, M3::identity(), zero) };
        let c2 = cfg(mp::ImuId::A, M3::identity(), V3::new(0.0, f64::INFINITY, 0.0));
        assert!(mp::combine_imus(Some(&a), Some(&a), &c1).is_none(), "NaN rotation");
        assert!(mp::combine_imus(Some(&a), None, &c2).is_none(), "infinite offset");
    });
    test("monoprop IMU pair parameters make a usable configuration", || {
        let c = mp::imu_pair_config();
        for r in [c.a_rotation, c.b_rotation] {
            assert!((r * r.transpose() - M3::identity()).amax() < 1e-12, "R R^T = I");
            assert_close(r.determinant(), 1.0, 1e-12, "det R = 1");
        }
        assert!(c.a_offset.iter().chain(c.b_offset.iter()).all(|v| v.is_finite()), "offsets finite");
        const { assert!(mp::IMU_PAIR_MAX_SKEW_US > 0, "skew limit positive") };
        assert_eq!(mp::IMU_PAIR_MAX_SKEW_US, 1250, "half a period at the 400 Hz placeholder");
        assert!(matches!(mp::REFERENCE_IMU, mp::ImuId::A | mp::ImuId::B));
        assert_eq!(c.reference, mp::REFERENCE_IMU);
        let Some(a) = reading(0, &M3::identity(), &zero, &w, &zero, &f_ref) else { return };
        assert!(mp::combine_imus(Some(&a), Some(&a), &c).is_some(), "combines");
    });
}
