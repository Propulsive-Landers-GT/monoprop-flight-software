// Tests for src/monoprop/navigator.rs and src/monoprop/stillness.rs.
#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::monoprop as mp;
use Estimation::*;

const PERIOD_US: u64 = 2500; // 400 Hz

/// A still, level vehicle on the pad: both IMUs read gravity and `gyro`; the magnetometer reads the
/// reference field (level, so body = navigation); the lidar reads `pad_range`.
struct Pad {
    gyro: V3,
    mag: V3,
    pad_range: f64,
}

impl Pad {
    fn new() -> Self {
        Pad { gyro: V3::zeros(), mag: mag_reference() * 50.0, pad_range: 0.3 }
    }

    /// Loop k: IMU every loop, magnetometer every 4th (100 Hz), lidar every 8th (50 Hz).
    fn inputs(&self, k: u64, armed: bool) -> mp::NavInputs {
        let t = 1_000_000 + k * PERIOD_US;
        let imu = mp::ImuSample::new(t, V3::new(0.0, 0.0, GRAVITY), self.gyro);
        mp::NavInputs {
            imu_a: imu,
            imu_b: imu,
            mag: if k.is_multiple_of(4) { mp::MagSample::new(t, self.mag) } else { None },
            lidar: if k.is_multiple_of(8) { mp::LidarSample::new(t, self.pad_range) } else { None },
            gnss: None,
            armed,
        }
    }
}

/// Run loops `from..to` and return the last output.
fn drive(nav: &mut mp::Navigator, pad: &Pad, from: u64, to: u64, armed: bool) -> mp::NavOutput {
    let mut out = nav.step(&mp::NavInputs::default());
    for k in from..to {
        out = nav.step(&pad.inputs(k, armed));
    }
    out
}

/// A navigator taken through start-up to OnPad (loop 0..450), and the next loop index.
fn on_pad(pad: &Pad) -> (mp::Navigator, u64) {
    let mut nav = mp::Navigator::new(mp::navigator_config());
    let out = drive(&mut nav, pad, 0, 450, false);
    assert_eq!(out.status.startup, mp::StartupState::OnPad, "on the pad after start-up");
    (nav, 450)
}

pub fn run() {
    test("stillness window: still readings add up, a moving one clears it", || {
        let mut w = mp::StillnessWindow::new();
        let (g, tol, wmax) = (GRAVITY, 0.5, 0.05);
        assert!(w.add_imu(&V3::new(0.0, 0.0, g), &V3::new(0.01, 0.0, 0.0), g, tol, wmax));
        assert!(w.add_imu(&V3::new(0.2, 0.0, g), &V3::new(0.03, 0.0, 0.0), g, tol, wmax));
        w.add_mag(&V3::new(1.0, 2.0, 3.0));
        assert_eq!((w.imu_count(), w.mag_count()), (2, 1));
        assert!((w.mean_gyro().unwrap_or_default() - V3::new(0.02, 0.0, 0.0)).amax() < 1e-15, "mean gyro");
        assert!(!w.add_imu(&V3::new(0.0, 0.0, g), &V3::new(0.2, 0.0, 0.0), g, tol, wmax), "spinning");
        assert_eq!((w.imu_count(), w.mag_count()), (0, 0), "cleared");
        assert!(w.mean_accel().is_none() && w.mean_mag().is_none());
        assert!(!mp::is_still(&V3::new(0.0, 0.0, g + 1.0), &V3::zeros(), g, tol, wmax), "accelerating");
    });
    test("start-up: Waiting -> Aligned -> OnPad -> Flying", || {
        let pad = Pad::new();
        let mut nav = mp::Navigator::new(mp::navigator_config());
        let out = drive(&mut nav, &pad, 0, 399, false);
        assert_eq!(out.status.startup, mp::StartupState::Waiting, "399 still samples: not yet");
        assert!(out.estimate.is_none() && nav.position_f32().is_none(), "no estimate before alignment");
        let out = nav.step(&pad.inputs(399, false));
        assert_eq!(out.status.startup, mp::StartupState::Aligned, "aligned on the 400th still sample");
        assert!(!out.status.lidar_ground_known);
        assert!(out.estimate.is_some(), "estimate after alignment");
        let Some(e) = out.estimate else { return };
        assert!(e.position.norm() < 1e-9 && boxminus(&e.attitude, &Quat::identity()).norm() < 1e-9, "level, at home");
        let out = drive(&mut nav, &pad, 400, 401, false); // loop 400 carries a lidar reading
        assert_eq!(out.status.startup, mp::StartupState::OnPad);
        assert!(out.status.lidar_ground_known);
        let out = drive(&mut nav, &pad, 401, 402, true);
        assert_eq!(out.status.startup, mp::StartupState::Flying, "armed");
        let out = drive(&mut nav, &pad, 402, 403, false);
        assert_eq!(out.status.startup, mp::StartupState::Flying, "one-way");
    });
    test("a moving sample restarts the window, and armed prevents alignment", || {
        let pad = Pad::new();
        let mut nav = mp::Navigator::new(mp::navigator_config());
        drive(&mut nav, &pad, 0, 200, false);
        let t = 1_000_000 + 200 * PERIOD_US;
        let spinning = mp::ImuSample::new(t, V3::new(0.0, 0.0, GRAVITY), V3::new(0.0, 0.0, 0.5)); // 0.5 rad/s: moving
        let bump = mp::NavInputs { imu_a: spinning, imu_b: spinning, ..pad.inputs(200, false) };
        nav.step(&bump);
        let out = drive(&mut nav, &pad, 201, 600, false);
        assert_eq!(out.status.startup, mp::StartupState::Waiting, "window restarted at 200: 399 still samples since");
        let out = drive(&mut nav, &pad, 600, 601, false);
        assert!(out.estimate.is_some(), "aligned on the 400th still sample after the bump (loop 600)");
        let mut armed_early = mp::Navigator::new(mp::navigator_config());
        let out = drive(&mut armed_early, &pad, 0, 1000, true);
        assert_eq!(out.status.startup, mp::StartupState::Waiting, "never aligns while armed");
        assert!(out.estimate.is_none());
    });
    test("a failed alignment stays Waiting and retries", || {
        let mut pad = Pad { mag: V3::new(0.0, 0.0, 40.0), ..Pad::new() }; // parallel to gravity: no heading
        let mut nav = mp::Navigator::new(mp::navigator_config());
        let out = drive(&mut nav, &pad, 0, 800, false);
        assert_eq!(out.status.startup, mp::StartupState::Waiting, "TRIAD can't use a vertical field");
        pad.mag = mag_reference() * 50.0;
        let out = drive(&mut nav, &pad, 800, 1200, false);
        assert_eq!(out.status.startup, mp::StartupState::Aligned, "aligned once the readings are usable");
    });
    test("IMU timing: bad timestamps rejected, long gaps not integrated, no sample no prediction", || {
        let pad = Pad::new();
        let (mut nav, k) = on_pad(&pad);
        let before = nav.estimate();
        let s0 = nav.status().counts;
        let quiet = nav.step(&mp::NavInputs::default());
        assert_eq!(quiet.estimate, before, "no IMU sample: nothing changes");
        let same_t = mp::NavInputs { lidar: None, mag: None, ..pad.inputs(k - 1, false) };
        let out = nav.step(&same_t);
        assert_eq!(out.status.counts.imu.rejected, s0.imu.rejected + 1, "repeated timestamp rejected");
        assert_eq!(out.estimate, before, "and changes nothing");
        let late = mp::NavInputs { lidar: None, mag: None, ..pad.inputs(k + 20, false) }; // 50 ms gap
        let out = nav.step(&late);
        assert_eq!(out.status.counts.imu_gaps, s0.imu_gaps + 1, "gap counted");
        assert_eq!(out.estimate.map(|e| e.position), before.map(|e| e.position), "not integrated");
        let out = nav.step(&pad.inputs(k + 21, false));
        assert_eq!(out.status.counts.imu.accepted, s0.imu.accepted + 1, "normal again");
    });
    test("a sample is used once: repeats are skipped", || {
        let pad = Pad::new();
        let (mut nav, k) = on_pad(&pad);
        let s0 = nav.status().counts;
        let t = 1_000_000 + k * PERIOD_US;
        let mag = mp::MagSample::new(t, pad.mag);
        for j in 0..5 {
            nav.step(&mp::NavInputs { mag, ..pad.inputs(k + j, false) });
        }
        let s = nav.status().counts;
        assert_eq!(s.mag.accepted + s.mag.rejected - s0.mag.accepted - s0.mag.rejected, 1, "fused once");
        assert!(s.mag.skipped >= s0.mag.skipped + 4, "then skipped");
    });
    test("stationary updates run at the reduced rate and stop when armed", || {
        let pad = Pad::new();
        let (mut nav, k) = on_pad(&pad);
        let s0 = nav.status().counts;
        drive(&mut nav, &pad, k, k + 400, false); // 1 s
        let s1 = nav.status().counts;
        let zv = s1.zero_velocity.accepted + s1.zero_velocity.rejected - s0.zero_velocity.accepted - s0.zero_velocity.rejected;
        let zr = s1.zero_rate.accepted + s1.zero_rate.rejected - s0.zero_rate.accepted - s0.zero_rate.rejected;
        assert_eq!((zv, zr), (10, 10), "10 Hz, not 400 Hz");
        assert_eq!(s1.zero_velocity.rejected, 0, "still vehicle: all accepted");
        drive(&mut nav, &pad, k + 400, k + 800, true); // armed, still on the ground
        let s2 = nav.status().counts;
        assert_eq!(s2.zero_velocity, s1.zero_velocity, "none after armed");
        assert_eq!(s2.zero_rate, s1.zero_rate, "none after armed");
    });
    test("on the pad the gyro bias is learned", || {
        let pad = Pad { gyro: V3::new(0.004, -0.002, 0.003), ..Pad::new() };
        let (mut nav, k) = on_pad(&pad);
        drive(&mut nav, &pad, k, k + 2000, false); // 5 s
        assert!(nav.angular_velocity_f32().is_some(), "aligned");
        let Some(rate) = nav.angular_velocity_f32() else { return };
        let residual = V3::new(rate[0] as f64, rate[1] as f64, rate[2] as f64);
        assert!(residual.amax() < 2e-4, "bias-corrected rate {:?} should be about zero", rate);
    });
    test("IMU mode switches are counted", || {
        let pad = Pad::new();
        let (mut nav, k) = on_pad(&pad);
        let s0 = nav.status().counts.imu_mode_switches;
        for j in 0..10 {
            let mut i = pad.inputs(k + j, false);
            if j % 2 == 0 {
                i.imu_b = None;
            }
            let out = nav.step(&i);
            let expected = if j % 2 == 0 {
                mp::ImuPairStatus::Single { used: mp::ImuId::A, reason: mp::SingleReason::OtherMissing }
            } else {
                mp::ImuPairStatus::Both
            };
            assert_eq!(out.status.imu_pair, Some(expected));
        }
        assert_eq!(nav.status().counts.imu_mode_switches - s0, 10, "every loop switches");
    });
    test("lidar out of range is skipped, a GNSS outlier is rejected", || {
        let pad = Pad::new();
        let (mut nav, k) = on_pad(&pad);
        let s0 = nav.status().counts;
        let t = 1_000_000 + k * PERIOD_US;
        let far = mp::NavInputs { lidar: mp::LidarSample::new(t, 5.0), ..pad.inputs(k, false) };
        let s = nav.step(&far).status.counts;
        assert_eq!(s.lidar.skipped, s0.lidar.skipped + 1, "5 m is beyond the lidar's range");
        assert_eq!(s.lidar.rejected, s0.lidar.rejected);
        let fix = mp::GnssSample::new(t + PERIOD_US, V3::new(100.0, 0.0, 0.0), V3::new(0.5, 0.5, 1.0));
        let s = nav.step(&mp::NavInputs { gnss: fix, ..pad.inputs(k + 1, false) }).status.counts;
        assert_eq!(s.gnss.rejected, s0.gnss.rejected + 1, "a 100 m jump is an outlier");
        let good = mp::GnssSample::new(t + 2 * PERIOD_US, V3::new(0.1, 0.0, 0.0), V3::new(0.5, 0.5, 1.0));
        let s = nav.step(&mp::NavInputs { gnss: good, ..pad.inputs(k + 2, false) }).status.counts;
        assert_eq!(s.gnss.accepted, s0.gnss.accepted + 1, "a sensible fix is used");
    });
    test("telemetry getters: None before alignment, then the estimate in f32", || {
        let pad = Pad::new();
        let nav = mp::Navigator::new(mp::navigator_config());
        assert!(nav.position_f32().is_none() && nav.velocity_f32().is_none() && nav.orientation_xyzw_f32().is_none());
        assert!(nav.angular_velocity_f32().is_none() && nav.sigmas_f32().is_none());
        let pad_tilted = Pad { gyro: V3::new(0.001, 0.0, 0.0), ..pad };
        let (mut nav, k) = on_pad(&pad_tilted);
        let out = drive(&mut nav, &pad_tilted, k, k + 10, false);
        assert!(out.estimate.is_some(), "aligned");
        let Some(e) = out.estimate else { return };
        assert_eq!(nav.position_f32(), Some(v3_to_f32(&e.position)));
        assert_eq!(nav.velocity_f32(), Some(v3_to_f32(&e.velocity)));
        let q = e.attitude.quaternion();
        assert_eq!(nav.orientation_xyzw_f32(), Some([q.i as f32, q.j as f32, q.k as f32, q.w as f32]), "x, y, z, w");
        assert_eq!(nav.angular_velocity_f32(), Some(v3_to_f32(&e.angular_rate)));
        assert!(nav.sigmas_f32().is_some_and(|s| s.iter().all(|v| v.is_finite() && *v > 0.0)));
    });
    test("navigator configuration from the Monoprop parameters", || {
        let c = mp::navigator_config();
        for (name, v) in [
            ("gravity", c.gravity),
            ("sigma_pos", c.sigma_pos),
            ("sigma_mag", c.sigma_mag),
            ("sigma_lidar_range", c.sigma_lidar_range),
            ("sigma_zero_velocity", c.sigma_zero_velocity),
            ("sigma_zero_rate", c.sigma_zero_rate),
            ("still_accel_tol", c.still_accel_tol),
            ("still_gyro_max", c.still_gyro_max),
        ] {
            assert!(v.is_finite() && v > 0.0, "{} = {}", name, v);
        }
        assert!(c.stationary_period_us > 0 && c.max_imu_gap_us > 0 && c.still_window_samples > 0 && c.still_min_mag_samples > 0);
        assert_eq!(c.stationary_period_us, 100_000, "10 Hz");
        assert_eq!(c.max_imu_gap_us, 25_000, "10 periods at 400 Hz");
        assert_eq!(c.still_window_samples, 400, "1 s at 400 Hz");
        let sg = per_sample_sigma(mp::bmi323::GYRO_NOISE_DENSITY, mp::IMU_RATE_HZ);
        let sa = per_sample_sigma(mp::bmi323::ACCEL_NOISE_DENSITY, mp::IMU_RATE_HZ);
        assert_close(c.still_gyro_max, 3f64.sqrt() * mp::bmi323::GYRO_ZERO_RATE_OFFSET + 5.0 * sg, 1e-15, "gyro threshold");
        assert_close(c.still_accel_tol, 3f64.sqrt() * mp::bmi323::ACCEL_OFFSET + 5.0 * sa, 1e-15, "accel threshold");
    });
}
