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
    test("start-up: Waiting -> OnPad -> Flying", || {
        let pad = Pad::new();
        let mut nav = mp::Navigator::new(mp::navigator_config());
        let out = drive(&mut nav, &pad, 0, 399, false);
        assert_eq!(out.status.startup, mp::StartupState::Waiting, "399 still samples: not yet");
        assert!(out.estimate.is_none() && nav.position_f32().is_none(), "no estimate before alignment");
        let out = nav.step(&pad.inputs(399, false));
        assert_eq!(out.status.startup, mp::StartupState::OnPad, "aligned, on the pad, on the 400th still sample");
        assert!(!out.status.lidar_ground_measured, "ground from the fallback");
        assert_eq!(nav.ground_z(), Some(-mp::IMU_HEIGHT_ABOVE_PAD));
        assert!(out.estimate.is_some(), "estimate after alignment");
        let Some(e) = out.estimate else { return };
        assert!(e.position.norm() < 1e-9 && boxminus(&e.attitude, &Quat::identity()).norm() < 1e-9, "level, at home");
        let out = drive(&mut nav, &pad, 400, 800, false); // 50 lidar readings
        assert_eq!(out.status.startup, mp::StartupState::OnPad);
        assert!(out.status.lidar_ground_measured, "ground from the pad readings");
        let out = drive(&mut nav, &pad, 800, 801, true);
        assert_eq!(out.status.startup, mp::StartupState::Flying, "armed");
        let out = drive(&mut nav, &pad, 801, 802, false);
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
        assert_eq!(out.status.startup, mp::StartupState::OnPad, "aligned once the readings are usable");
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
    test("lidar too close on the pad: the fallback ground height is used and the lidar works in flight", || {
        let pad = Pad { pad_range: 0.03, ..Pad::new() }; // below the 4 cm minimum
        let cfg = mp::NavConfig { imu_height_above_pad: 0.05, ..mp::navigator_config() };
        let mut nav = mp::Navigator::new(cfg);
        let out = drive(&mut nav, &pad, 0, 600, false);
        assert_eq!(out.status.startup, mp::StartupState::OnPad, "launch is not blocked by the lidar");
        assert!(!out.status.lidar_ground_measured && out.status.counts.lidar.skipped > 0, "readings skipped");
        assert_eq!(nav.ground_z(), Some(-0.05), "fallback: pad 5 cm below the IMU");
        drive(&mut nav, &pad, 600, 602, true);
        let s0 = nav.status().counts.lidar;
        let z0 = nav.estimate().map(|e| e.position.z).unwrap_or(f64::NAN);
        let t = 1_000_000 + 602 * PERIOD_US;
        let climb = mp::NavInputs { lidar: mp::LidarSample::new(t, 0.15), ..pad.inputs(602, true) };
        let out = nav.step(&climb);
        assert_eq!(out.status.counts.lidar.accepted, s0.accepted + 1, "fused in flight");
        let z1 = out.estimate.map(|e| e.position.z).unwrap_or(f64::NAN);
        assert!(z1 > z0 + 0.01, "height moved up towards 0.10 m: {} -> {}", z0, z1);
    });
    test("pad readings refine the fallback once LIDAR_GROUND_SAMPLES (50) have been collected", || {
        let pad = Pad { pad_range: 0.30, ..Pad::new() };
        let cfg = mp::NavConfig { imu_height_above_pad: 0.05, ..mp::navigator_config() };
        let mut nav = mp::Navigator::new(cfg);
        drive(&mut nav, &pad, 0, 400, false);
        let mut readings = 0;
        for k in 400..820 {
            let out = nav.step(&pad.inputs(k, false));
            if k.is_multiple_of(8) {
                readings += 1;
                assert_eq!(out.status.lidar_ground_measured, readings >= mp::LIDAR_GROUND_SAMPLES, "after {} readings", readings);
            }
        }
        let ground = nav.ground_z().unwrap_or(f64::NAN);
        let z = nav.estimate().map(|e| e.position.z).unwrap_or(f64::NAN);
        assert_close(z - ground, 0.30, 1e-3, "predicted range on the pad");
        assert!(!nav.status().lidar_ground_truncated, "well above the floor: no truncation");
    });
    test("averaging pad readings beats a single reading", || {
        let pad = Pad::new();
        let mut nav = mp::Navigator::new(mp::navigator_config());
        drive(&mut nav, &pad, 0, 400, false);
        let (truth, sigma) = (0.30, 0.01);
        let mut rng = Rng::new(41);
        let mut first = None;
        for k in 400..820 {
            let mut i = pad.inputs(k, false);
            if i.lidar.is_some() {
                let r = truth + rng.normal(sigma);
                first.get_or_insert(r);
                i.lidar = mp::LidarSample::new(1_000_000 + k * PERIOD_US, r);
            }
            nav.step(&i);
        }
        assert!(nav.status().lidar_ground_measured);
        let z = nav.estimate().map(|e| e.position.z).unwrap_or(f64::NAN);
        let averaged_error = ((z - nav.ground_z().unwrap_or(f64::NAN)) - truth).abs();
        let single_error = (first.unwrap_or(f64::NAN) - truth).abs();
        assert!(averaged_error < sigma / 2.0, "averaged error {:.4} m", averaged_error);
        assert!(averaged_error < single_error, "averaged {:.4} m, first reading alone {:.4} m", averaged_error, single_error);
    });
    test("switching A -> B -> A keeps the clock even", || {
        for off in [1000i64, -1000] {
            let pad = Pad::new();
            let (mut nav, k0) = on_pad(&pad);
            let s0 = nav.status().counts;
            let mut last_t = nav.estimate().map(|e| e.t_us).unwrap_or(0);
            // a few combined loops (learn the offset), then A only, B only, A only
            let pattern = [(true, true); 5].iter().chain([(true, false); 3].iter()).chain([(false, true); 3].iter()).chain([(true, false); 3].iter());
            for (j, (use_a, use_b)) in pattern.enumerate() {
                let t_a = 1_000_000 + (k0 + j as u64) * PERIOD_US;
                let t_b = t_a.saturating_add_signed(off);
                let i = mp::NavInputs {
                    imu_a: if *use_a { mp::ImuSample::new(t_a, V3::new(0.0, 0.0, GRAVITY), V3::zeros()) } else { None },
                    imu_b: if *use_b { mp::ImuSample::new(t_b, V3::new(0.0, 0.0, GRAVITY), V3::zeros()) } else { None },
                    ..mp::NavInputs::default()
                };
                let t = nav.step(&i).estimate.map(|e| e.t_us).unwrap_or(0);
                assert_eq!(t - last_t, PERIOD_US, "offset {} us, loop {}: dt must be one period", off, j);
                last_t = t;
            }
            let s = nav.status().counts;
            assert_eq!((s.imu.rejected, s.imu_gaps), (s0.imu.rejected, s0.imu_gaps), "nothing rejected");
        }
    });
    test("switching with a jittering clock offset: every dt within the skew limit of one period", || {
        let skew = mp::IMU_PAIR_MAX_SKEW_US;
        for base in [600i64, -600] {
            let pad = Pad::new();
            let (mut nav, k0) = on_pad(&pad);
            let s0 = nav.status().counts;
            let mut last_t = nav.estimate().map(|e| e.t_us).unwrap_or(0);
            let mut rng = Rng::new(51);
            let pattern = [(true, true), (true, true), (true, false), (false, true), (false, true), (true, false), (true, true), (false, true), (true, false)];
            for j in 0..200u64 {
                let (use_a, use_b) = pattern[(j % pattern.len() as u64) as usize];
                let jitter = ((rng.uniform() * 2.0 - 1.0) * 500.0) as i64;
                let t_a = 1_000_000 + (k0 + j) * PERIOD_US;
                let t_b = t_a.saturating_add_signed(base + jitter);
                let i = mp::NavInputs {
                    imu_a: if use_a { mp::ImuSample::new(t_a, V3::new(0.0, 0.0, GRAVITY), V3::zeros()) } else { None },
                    imu_b: if use_b { mp::ImuSample::new(t_b, V3::new(0.0, 0.0, GRAVITY), V3::zeros()) } else { None },
                    ..mp::NavInputs::default()
                };
                let t = nav.step(&i).estimate.map(|e| e.t_us).unwrap_or(0);
                let dt = t as i64 - last_t as i64;
                assert!((dt - PERIOD_US as i64).abs() <= skew as i64, "offset {} us, loop {}: dt {} us", base, j, dt);
                last_t = t;
            }
            let s = nav.status().counts;
            assert_eq!((s.imu.rejected, s.imu_gaps), (s0.imu.rejected, s0.imu_gaps), "nothing rejected");
        }
    });
    test("GNSS moving the pad height doesn't move the ground (the ground is taken at home)", || {
        // regression: the ground used to be computed from the filter's own height, which a GNSS fix on the
        // pad can move by tenths of a metre
        let pad = Pad { pad_range: 0.30, ..Pad::new() };
        let mut nav = mp::Navigator::new(mp::navigator_config());
        drive(&mut nav, &pad, 0, 400, false);
        for k in 400..820 {
            let mut i = pad.inputs(k, false);
            if k.is_multiple_of(4) {
                // fixes 0.4 m too high, arriving while the ground average is collected
                i.gnss = mp::GnssSample::new(1_000_000 + k * PERIOD_US, V3::new(0.0, 0.0, 0.4), V3::new(0.5, 0.5, 1.0));
            }
            nav.step(&i);
            if k == 470 {
                let z = nav.estimate().map(|e| e.position.z).unwrap_or(f64::NAN);
                assert!(z > 0.1, "the fixes moved the filter's pad height by a few tenths: {:.3} m", z);
            }
        }
        assert!(nav.status().lidar_ground_measured);
        assert_close(nav.ground_z().unwrap_or(f64::NAN), -0.30, 0.05, "ground height (within the lidar noise)");
    });
    test("arming before 50 pad readings keeps the fallback ground for the flight", || {
        let pad = Pad { pad_range: 0.30, ..Pad::new() };
        let cfg = mp::NavConfig { imu_height_above_pad: 0.05, ..mp::navigator_config() };
        let mut nav = mp::Navigator::new(cfg);
        drive(&mut nav, &pad, 0, 600, false); // aligned at 400, then 25 lidar readings
        assert!(!nav.status().lidar_ground_measured, "25 of 50 readings");
        let out = drive(&mut nav, &pad, 600, 1200, true); // armed: 75 more readings, in flight
        assert_eq!(out.status.startup, mp::StartupState::Flying);
        assert!(!out.status.lidar_ground_measured, "the partial average is dropped at arming");
        assert_eq!(nav.ground_z(), Some(-0.05), "fallback kept for the flight");
    });
    test("pad readings near the lidar's floor: the average is not used and the fallback is kept", || {
        // readings alternate between just below the 4 cm minimum and just above it: half are cut off
        let cfg = mp::NavConfig { imu_height_above_pad: 0.05, ..mp::navigator_config() };
        let mut nav = mp::Navigator::new(cfg);
        let pad = Pad { pad_range: 0.05, ..Pad::new() };
        drive(&mut nav, &pad, 0, 400, false);
        for k in 400..1400 {
            let mut i = pad.inputs(k, false);
            if i.lidar.is_some() && (k / 8).is_multiple_of(2) {
                i.lidar = mp::LidarSample::new(1_000_000 + k * PERIOD_US, 0.035);
            }
            nav.step(&i);
        }
        let st = nav.status();
        assert!(!st.lidar_ground_measured && st.lidar_ground_truncated, "fallback kept, flagged as truncation");
        assert_eq!(nav.ground_z(), Some(-0.05), "the fallback");
    });
    test("a few readings below the floor (under 5 %) still give a measured ground", || {
        let mut nav = mp::Navigator::new(mp::navigator_config());
        let pad = Pad { pad_range: 0.30, ..Pad::new() };
        drive(&mut nav, &pad, 0, 400, false);
        let mut lidar_loops = 0;
        for k in 400..1000 {
            let mut i = pad.inputs(k, false);
            if i.lidar.is_some() {
                lidar_loops += 1;
                if lidar_loops <= 2 {
                    i.lidar = mp::LidarSample::new(1_000_000 + k * PERIOD_US, 0.02); // 2 below, then 50 valid: 3.8 %
                }
            }
            nav.step(&i);
        }
        let st = nav.status();
        assert!(st.lidar_ground_measured && !st.lidar_ground_truncated, "measured, not flagged");
        let z = nav.estimate().map(|e| e.position.z).unwrap_or(f64::NAN);
        assert_close(z - nav.ground_z().unwrap_or(f64::NAN), 0.30, 1e-3, "predicted range on the pad");
    });
}
