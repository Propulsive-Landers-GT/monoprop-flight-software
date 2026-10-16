// Monoprop flights through the Navigator: the full sensor set (two IMUs, magnetometer, lidar, optional
// GNSS) on simulated hovers below the lidar's range.
//
// Every sensor value is a PLACEHOLDER from params.rs (navigator_config()), and the bounds below hold for
// those placeholders, not for the real hardware. Re-check them once the parameters are measured.
// Every assertion message names its seed, so a failing flight can be re-run alone.
#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::monoprop as mp;
use Estimation::*;

const SEED_PAD: u64 = 701;
const SEEDS_HOVER: [u64; 3] = [711, 712, 713];
const SEED_NO_GNSS: u64 = 721;
const SEED_LIDAR_CLOSE: u64 = 731;
const SEEDS_DROPOUT: [u64; 5] = [741, 742, 743, 744, 745];
const SEED_JITTER: u64 = 751;
const SEED_OFFSETS: u64 = 761;

/// Largest horizontal position error rms (m) with GNSS.
const HORIZ_RMS: f64 = 0.35;
/// Largest vertical error (m) while the lidar can read (measured worst 0.056 m, with mounting offsets).
const VERT_MAX: f64 = 0.10;
/// Largest vertical error (m) when the lidar can't read on the pad (scenario 4; measured 0.108 m, 0.4 s
/// after arming, still on the ground: no zero-velocity updates and no lidar, so GNSS noise moves the height
/// until the climb brings the lidar into range).
const VERT_MAX_LIDAR_CLOSE: f64 = 0.15;
const VEL_MAX: f64 = 0.3;
/// Largest attitude error (deg). Measured up to 2.5 deg, nearly all of it yaw (roll/pitch at most 1.15 deg),
/// a single peak 5 to 7 s after arming that does not grow, and the same without the IMU dropout: consistent
/// with the magnetometer noise (sigma 0.02 on a field inclined 64 deg is about 2.6 deg of yaw per sample).
/// SIGMA_MAG is a placeholder and the main driver of this yaw error: re-check this bound once it has been
/// measured on the real magnetometer with the motor running.
const ATT_MAX_DEG: f64 = 3.0;
/// Average NEES band (15 states: about 15 if the filter's covariance describes its real error).
const NEES_MIN: f64 = 5.0;
const NEES_MAX: f64 = 45.0;

/// The Monoprop navigator configuration with the fallback pad height set to the simulated pad (as it
/// would be set from the vehicle drawings).
fn cfg_for(opt: &MonopropOptions) -> mp::NavConfig {
    mp::NavConfig { imu_height_above_pad: opt.pad_height, ..mp::navigator_config() }
}

fn print_report(name: &str, seed: u64, r: &NavReport) {
    println!(
        "          {} (seed {}): horizontal rms {:.3} m max {:.3} m, vertical max {:.3} m at t+{:.1} s, velocity max {:.3} m/s, attitude max {:.2} deg at t+{:.1} s (yaw {:.2}, tilt {:.2}), NEES {:.1} (horizontal {:.1}, vertical {:.1}), ground {}",
        name, seed, r.horiz_rms, r.horiz_max, r.vert_max, r.vert_max_t, r.vel_max, r.att_max_deg, r.att_max_t, r.att_peak_yaw_deg, r.att_peak_tilt_deg, r.nees_avg, r.nees_horiz, r.nees_vert,
        match r.status { Some(s) if s.lidar_ground_measured => "measured", Some(s) if s.lidar_ground_truncated => "fallback (truncation)", _ => "fallback" }
    );
}

/// The accuracy and consistency checks every hover with GNSS must pass.
fn check_hover(name: &str, seed: u64, r: &NavReport, horiz_rms: f64, vert_max: f64) {
    assert!(r.finite, "{} (seed {}): a NaN or infinity appeared", name, seed);
    assert!(r.horiz_rms < horiz_rms, "{} (seed {}): horizontal rms {:.3} m", name, seed, r.horiz_rms);
    assert!(r.vert_max < vert_max, "{} (seed {}): vertical max {:.3} m", name, seed, r.vert_max);
    assert!(r.vel_max < VEL_MAX, "{} (seed {}): velocity max {:.3} m/s", name, seed, r.vel_max);
    assert!(r.att_max_deg < ATT_MAX_DEG, "{} (seed {}): attitude max {:.2} deg", name, seed, r.att_max_deg);
    assert!(r.nees_avg > NEES_MIN && r.nees_avg < NEES_MAX, "{} (seed {}): average NEES {:.1}", name, seed, r.nees_avg);
}

pub fn run() {
    test("1. pad sequence: Waiting -> OnPad -> Flying at the right times", || {
        let opt = MonopropOptions::hover(SEED_PAD);
        let r = fly_navigator(cfg_for(&opt), opt);
        let s = SEED_PAD;
        assert!(!r.estimate_before_align, "seed {}: an estimate before alignment", s);
        let onpad = r.t_onpad.unwrap_or(f64::NAN);
        assert!((onpad - 1.0).abs() < 0.01, "seed {}: OnPad at {:.3} s, expected after 1 s of still samples", s, onpad);
        // 50 valid lidar readings at 50 Hz after alignment: at least 1 s (the pad is 20 cm below the lidar,
        // well above its floor, so hardly any reading is skipped).
        let ground = r.t_ground_measured.unwrap_or(f64::NAN);
        assert!(ground >= onpad + 0.98 && ground < onpad + 1.5, "seed {}: ground measured at {:.3} s", s, ground);
        assert!(r.status.is_some_and(|st| !st.lidar_ground_truncated), "seed {}: no truncation at 20 cm", s);
        let flying = r.t_flying.unwrap_or(f64::NAN);
        assert!((flying - opt.t_arm).abs() < 1e-9, "seed {}: Flying at {:.4} s, armed at {} s", s, flying, opt.t_arm);
    });
    test("2. hover with GNSS (3 seeds): accurate and consistent", || {
        for seed in SEEDS_HOVER {
            let opt = MonopropOptions::hover(seed);
            let r = fly_navigator(cfg_for(&opt), opt);
            print_report("hover", seed, &r);
            check_hover("hover", seed, &r, HORIZ_RMS, VERT_MAX);
        }
    });
    test("3. hover without GNSS: vertical held by the lidar, horizontal drift bounded and known", || {
        // Horizontal drift (seed 721): 0.03 m after 5 s, 0.36 m after 10 s, 0.71 m after 15 s, against the
        // filter's own 1-sigma of 0.74, 1.05 and 1.91 m. Theory as an upper bound: an uncorrected 0.05 m/s^2
        // accelerometer bias (the simulator's turn-on scale) drifts 0.5 * b * t^2 = 0.6, 2.5 and 5.6 m; the
        // pad's zero-velocity updates learn most of the bias/tilt combination that causes drift, hence less.
        let s = SEED_NO_GNSS;
        let opt = MonopropOptions { gnss: false, ..MonopropOptions::hover(s) };
        let r = fly_navigator(cfg_for(&opt), opt);
        print_report("no GNSS", s, &r);
        for (t, h, sigma) in &r.drift {
            println!("            t+{:4.1} s: horizontal {:.3} m, filter 1-sigma {:.3} m, theory bound 0.5*0.05*t^2 = {:.2} m", t, h, sigma, 0.5 * 0.05 * t * t);
            assert!(*h <= 3.0 * sigma, "seed {}: at t+{:.1} s the horizontal error {:.3} m exceeds 3 sigma ({:.3} m)", s, t, h, 3.0 * sigma);
        }
        assert!(r.finite, "seed {}: a NaN or infinity appeared", s);
        assert!(r.vert_max < VERT_MAX, "seed {}: vertical max {:.3} m", s, r.vert_max);
        assert!(r.horiz_max < 2.5, "seed {}: horizontal max {:.3} m (absolute bound 2.5 m)", s, r.horiz_max);
    });
    test("4. lidar can't read on the pad: the fallback ground is used and the lidar works in flight", || {
        let s = SEED_LIDAR_CLOSE;
        let opt = MonopropOptions { pad_height: 0.03, ..MonopropOptions::hover(s) }; // under the 4 cm minimum
        let r = fly_navigator(cfg_for(&opt), opt);
        print_report("lidar close", s, &r);
        assert!(r.t_onpad.is_some_and(|t| (t - 1.0).abs() < 0.01), "seed {}: OnPad on time, without the lidar", s);
        assert!(r.t_ground_measured.is_none(), "seed {}: no pad reading is usable, so the fallback stays", s);
        assert!(r.lidar_accepted_in_flight > 0, "seed {}: the lidar is fused once in range", s);
        check_hover("lidar close", s, &r, HORIZ_RMS, VERT_MAX_LIDAR_CLOSE);
    });
    test("4b. the same with the fallback height 5 cm wrong: bounded (vertical error reported)", || {
        let s = SEED_LIDAR_CLOSE;
        let opt = MonopropOptions { pad_height: 0.03, ..MonopropOptions::hover(s) };
        let cfg = mp::NavConfig { imu_height_above_pad: 0.08, ..mp::navigator_config() };
        let r = fly_navigator(cfg, opt);
        print_report("lidar close, fallback 5 cm wrong", s, &r);
        assert!(r.finite && r.vert_max < 1.0, "seed {}: vertical max {:.3} m", s, r.vert_max);
    });
    test("5. IMU B drops out for 3 s mid-hover (5 seeds): single mode, error bounded", || {
        // every seed flown and reported first, then checked, so one failure doesn't hide the others
        let runs: Vec<(u64, NavReport)> = SEEDS_DROPOUT
            .iter()
            .map(|&s| {
                let opt = MonopropOptions { b_dropout: Some((9.0, 12.0)), ..MonopropOptions::hover(s) };
                (s, fly_navigator(cfg_for(&opt), opt))
            })
            .collect();
        for (s, r) in &runs {
            print_report("IMU B dropout", *s, r);
        }
        for (s, r) in &runs {
            let c = r.status.map(|st| st.counts).unwrap_or_default();
            assert!(r.saw_single, "seed {}: the navigator never went to single mode", s);
            assert!(c.imu_mode_switches >= 2, "seed {}: {} mode switches", s, c.imu_mode_switches);
            assert_eq!(c.imu.rejected, 0, "seed {}: IMU samples rejected", s);
            check_hover("IMU B dropout", *s, r, 0.5, VERT_MAX);
        }
    });
    test("6. jittery IMU B timestamps (+600 us +- 500 us): nothing rejected, error bounded", || {
        let s = SEED_JITTER;
        let opt = MonopropOptions { b_clock_offset_us: 600, b_clock_jitter_us: 500, ..MonopropOptions::hover(s) };
        let r = fly_navigator(cfg_for(&opt), opt);
        print_report("jitter", s, &r);
        // like-for-like: the jitter has its own random stream, so the same seed without it has the same noise
        let calm = MonopropOptions::hover(s);
        print_report("same seed, no jitter", s, &fly_navigator(cfg_for(&calm), calm));
        let c = r.status.map(|st| st.counts).unwrap_or_default();
        assert_eq!((c.imu.rejected, c.imu_gaps), (0, 0), "seed {}: IMU samples rejected or gaps", s);
        check_hover("jitter", s, &r, HORIZ_RMS, VERT_MAX);
    });
    test("7. hover with mounting offsets (IMU B 5 cm, lidar 5 cm, GNSS antenna 10 cm)", || {
        let s = SEED_OFFSETS;
        // params.rs untouched: offsets through the config only. With the lidar 4 cm lower it sees the pad at
        // 16 cm, above its floor even with the 5 cm noise placeholder, so the measured ground is used and
        // the offset corrections are tested without truncation.
        let with_offsets = |opt: &MonopropOptions| {
            let mut cfg = cfg_for(opt);
            cfg.imu_pair.b_offset = V3::new(0.0, 0.05, 0.0);
            cfg.lidar.offset = V3::new(0.03, 0.0, -0.04);
            cfg.antenna_offset = V3::new(0.0, 0.0, 0.10);
            cfg
        };
        let opt = MonopropOptions::hover(s);
        let r = fly_navigator(with_offsets(&opt), opt);
        print_report("offsets", s, &r);
        assert!(r.status.is_some_and(|st| st.lidar_ground_measured), "seed {}: measured ground (no truncation)", s);
        check_hover("offsets", s, &r, HORIZ_RMS, VERT_MAX);
        // reported only: the same with the pad 10 cm below the lidar (about 11 % of readings under the floor
        // with the 5 cm placeholder, so the truncation check keeps the matching fallback)
        let near = MonopropOptions { pad_height: 0.14, ..MonopropOptions::hover(s) };
        print_report("offsets, pad 10 cm below the lidar", s, &fly_navigator(with_offsets(&near), near));
    });
}
