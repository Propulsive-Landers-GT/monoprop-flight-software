//! Dry run of the navigator on simulated sensor data: still on the pad, climb to 1 m, hover, land. Two IMUs
//! (each with its own bias and noise), magnetometer, lidar and GPS with the noise in config/monoprop.toml
//! (placeholders); also with either IMU missing for 3 s mid-hover.
//!
//! A minimal simulator lives here because the existing data doesn't fit: the ControlLoop/Data CSVs disagree
//! with each other (a stationary IMU while the lidar and GPS climb), their magnetometer is all zeros and the
//! GPS file has no time column; David's simulator flies a figure-eight at 3.5-6.5 m, moving from t = 0, so
//! it never passes the stillness check and is beyond the lidar's range.
use Drone::config::CONFIG;
use Drone::imu_pair::ImuId;
use Drone::navigator::Navigator;
use Drone::sensors::{GpsSample, ImuSample, LidarSample, MagSample, SensorData};
use Estimation::{boxminus, g_inertial, mag_reference, quat_exp, Quat, PI, V3};

#[path = "../../Estimation/tests/support/rng.rs"]
mod rng;
use rng::Rng;

const T_ARM: f64 = 3.0;
const PAD_HEIGHT: f64 = 0.20; // true height of the IMU above the pad, m
const HEIGHT: f64 = 1.0;
const RAMP: f64 = 2.0;
const END: f64 = T_ARM + 1.0 + 2.0 * RAMP + 8.0 + 1.0;
const SEEDS: [u64; 3] = [801, 802, 803];
const SEED_DROPOUT: u64 = 804;
const DROPOUT: (f64, f64) = (9.0, 12.0); // s, mid-hover

// Bounds for the placeholder sensor values in config/monoprop.toml, about 1.5x the worst measured run.
const HORIZ_RMS: f64 = 0.45; // m; worst 0.300 (seed 802)
const VERT_MAX: f64 = 0.07; // m; worst 0.044 (seed 802)
const VEL_MAX: f64 = 0.85; // m/s; worst 0.562 (seed 802)
// Mostly yaw, driven by the mag.sigma placeholder: re-check once mag.sigma is measured.
const ATT_MAX_DEG: f64 = 7.5; // deg; worst 5.13 (seed 802)
// While one IMU is missing (9-13 s), on the dropout runs only
const GAP_POS_MAX: f64 = 0.7; // m; worst 0.447 (IMU A missing)
const GAP_VEL_MAX: f64 = 0.35; // m/s; worst 0.236
const GAP_ATT_MAX_DEG: f64 = 2.0; // deg; worst 1.37

/// Height, vertical speed and acceleration of the hover: climb 1 s after T_ARM, hover 8 s, descend.
fn profile(t: f64) -> (f64, f64, f64) {
    let (up, hover, down) = (T_ARM + 1.0, T_ARM + 1.0 + RAMP, T_ARM + 1.0 + RAMP + 8.0);
    let k = PI / RAMP;
    let h = 0.5 * HEIGHT;
    let ramp = |u: f64, s: f64| (h * (1.0 - s * (k * u).cos()), s * h * k * (k * u).sin(), s * h * k * k * (k * u).cos());
    if t < up {
        (0.0, 0.0, 0.0)
    } else if t < hover {
        ramp(t - up, 1.0)
    } else if t < down {
        (HEIGHT, 0.0, 0.0)
    } else if t < down + RAMP {
        ramp(t - down, -1.0)
    } else {
        (0.0, 0.0, 0.0)
    }
}

/// A gentle wobble scaled by height (zero on the pad).
fn body_rate(t: f64) -> V3 {
    let e = profile(t).0 / HEIGHT;
    V3::new(0.05 * (1.3 * t).sin(), 0.04 * (0.9 * t).cos(), 0.1 * (0.5 * t).sin()) * e
}

struct Report {
    horiz_rms: f64,
    horiz_max: f64,
    vert_max: f64,
    vel_max: f64,
    att_max_deg: f64,
    aligned_at: Option<f64>,
    finite: bool,
    /// (velocity error m/s, attitude error deg) at 2, 4 and 6 s after alignment
    after: [Option<(f64, f64)>; 3],
    /// Navigator updates given IMU A only, IMU B only, both.
    imu_counts: [u32; 3],
    /// Time (s) of the vertical maximum.
    vert_max_t: f64,
    /// Largest position, velocity and attitude errors from the start of the dropout window to 1 s after it.
    window_max: (f64, f64, f64),
}

/// One IMU's turn-on biases (accel, gyro) in its own axes.
fn biases(rng: &mut Rng) -> (V3, V3) {
    (rng.normal_vec::<3>(0.05), rng.normal_vec::<3>(0.005))
}

fn fly(seed: u64, missing: Option<ImuId>) -> Report {
    let c = CONFIG;
    let mut rng = Rng::new(seed);
    let dt = 1.0 / c.imu_rate_hz;
    let period_us = (1e6 / c.imu_rate_hz) as u64;
    let (sa, sg) = (c.imu_accel_noise_density * c.imu_rate_hz.sqrt(), c.imu_gyro_noise_density * c.imu_rate_hz.sqrt());
    let (mut bias_a, mut bias_b) = (biases(&mut rng), biases(&mut rng));
    let m = |r: [[f64; 3]; 3]| Estimation::M3::new(r[0][0], r[0][1], r[0][2], r[1][0], r[1][1], r[1][2], r[2][0], r[2][1], r[2][2]);
    let (rot_a, rot_b) = (m(c.imu_pair_a_rotation), m(c.imu_pair_b_rotation));
    let (off_a, off_b) = (V3::from(c.imu_pair_a_offset), V3::from(c.imu_pair_b_offset));
    let ref_off = if c.imu_pair_reference == ImuId::A { off_a } else { off_b };
    let mut q = quat_exp(&V3::new(0.0, 0.0, rng.uniform() * 6.0 - 3.0));
    let mut nav = Navigator::new(&c);
    let mut r = Report { horiz_rms: 0.0, horiz_max: 0.0, vert_max: 0.0, vel_max: 0.0, att_max_deg: 0.0, aligned_at: None, finite: true, after: [None; 3], imu_counts: [0; 3], vert_max_t: 0.0, window_max: (0.0, 0.0, 0.0) };
    let (mut sum_sq, mut n) = (0.0, 0usize);
    let mut k: u64 = 0;
    while (k as f64) * dt < END {
        let t = k as f64 * dt;
        let (_, _, az) = profile(t);
        let w = body_rate(t);
        let alpha = (body_rate(t + 1e-4) - body_rate(t - 1e-4)) / 2e-4;
        let f_ref = q.inverse() * (V3::new(0.0, 0.0, az) - g_inertial());
        let mut read = |rot: &Estimation::M3, off: V3, bias: (V3, V3)| {
            let d = off - ref_off;
            let f = f_ref + alpha.cross(&d) + w.cross(&w.cross(&d));
            (rot.transpose() * f + bias.0 + rng.normal_vec::<3>(sa), rot.transpose() * w + bias.1 + rng.normal_vec::<3>(sg))
        };
        let (accel_a, gyro_a) = read(&rot_a, off_a, bias_a);
        let (accel_b, gyro_b) = read(&rot_b, off_b, bias_b);
        // advance the truth over the interval, then stamp the readings at its end
        q *= quat_exp(&(w * dt));
        q.renormalize();
        k += 1;
        let t = k as f64 * dt;
        for b in [&mut bias_a, &mut bias_b] {
            b.0 += rng.normal_vec::<3>(c.imu_accel_bias_random_walk * dt.sqrt());
            b.1 += rng.normal_vec::<3>(c.imu_gyro_bias_random_walk * dt.sqrt());
        }
        let (z, vz, _) = profile(t);
        let p = V3::new(0.0, 0.0, z);
        let t_us = 1_000_000 + k * period_us;
        let mag = k.is_multiple_of(4).then(|| MagSample { t_us, field: ((q.inverse() * mag_reference() + rng.normal_vec::<3>(c.mag_sigma)) * 50.0).into() });
        let lidar = k.is_multiple_of(8).then(|| {
            let ps = p + q * V3::from(c.lidar_offset);
            let d = q * V3::from(c.lidar_beam).normalize();
            let true_range = (-PAD_HEIGHT - ps.z) / d.z;
            // modelling choice: below the floor the reading stays below it (the navigator then ignores it)
            let range = if true_range < c.lidar_min_range { true_range } else { true_range + rng.normal(c.lidar_sigma) };
            LidarSample { t_us, range }
        });
        let gps = k.is_multiple_of(80).then(|| {
            let sigma = [0.5, 0.5, 1.0];
            let noise = V3::new(rng.normal(sigma[0]), rng.normal(sigma[1]), rng.normal(sigma[2]));
            GpsSample { t_us, position: (p + q * V3::from(c.gps_antenna_offset) + noise).into(), sigma }
        });
        let gone = |id: ImuId| missing == Some(id) && t >= DROPOUT.0 && t < DROPOUT.1;
        let imu_a = (!gone(ImuId::A)).then(|| ImuSample { t_us, accel: accel_a.into(), gyro: gyro_a.into() });
        let imu_b = (!gone(ImuId::B)).then(|| ImuSample { t_us, accel: accel_b.into(), gyro: gyro_b.into() });
        match (imu_a.is_some(), imu_b.is_some()) {
            (true, false) => r.imu_counts[0] += 1,
            (false, true) => r.imu_counts[1] += 1,
            (true, true) => r.imu_counts[2] += 1,
            (false, false) => {}
        }
        let data = SensorData { imu_a, imu_b, mag, lidar, gps };
        let Some(s) = nav.update(&data) else { continue };
        r.aligned_at.get_or_insert(t);
        let est_p = V3::from(s.position);
        let dp = p - est_p;
        let horiz = (dp.x * dp.x + dp.y * dp.y).sqrt();
        sum_sq += horiz * horiz;
        n += 1;
        r.horiz_max = r.horiz_max.max(horiz);
        if dp.z.abs() > r.vert_max {
            r.vert_max = dp.z.abs();
            r.vert_max_t = t;
        }
        let vel_err = (V3::new(0.0, 0.0, vz) - V3::from(s.velocity)).norm();
        r.vel_max = r.vel_max.max(vel_err);
        let est_q = Quat::from_quaternion(Estimation::Quaternion::new(s.attitude[0], s.attitude[1], s.attitude[2], s.attitude[3]));
        let att_err = boxminus(&q, &est_q).norm().to_degrees();
        r.att_max_deg = r.att_max_deg.max(att_err);
        if t >= DROPOUT.0 && t < DROPOUT.1 + 1.0 {
            let (p0, v0, a0) = r.window_max;
            r.window_max = (p0.max(dp.norm()), v0.max(vel_err), a0.max(att_err));
        }
        let since = t - r.aligned_at.unwrap_or(t);
        for (slot, at) in r.after.iter_mut().zip([2.0, 4.0, 6.0]) {
            if slot.is_none() && since >= at {
                *slot = Some((vel_err, att_err));
            }
        }
        r.finite &= s.position.iter().chain(s.velocity.iter()).chain(s.attitude.iter()).all(|x| x.is_finite());
    }
    r.horiz_rms = (sum_sq / n.max(1) as f64).sqrt();
    r
}

#[test]
fn navigator_follows_a_simulated_hover() {
    let runs = SEEDS.iter().map(|&s| (s, None)).chain([(SEED_DROPOUT, Some(ImuId::B)), (SEED_DROPOUT, Some(ImuId::A))]);
    for (seed, missing) in runs {
        let r = fly(seed, missing);
        let fmt = |x: Option<(f64, f64)>| x.map(|(v, a)| format!("{:.3} m/s {:.2} deg", v, a)).unwrap_or_default();
        println!(
            "seed {} {}: aligned at {:?} s; horizontal rms {:.3} m max {:.3} m, vertical max {:.3} m, velocity max {:.3} m/s, attitude max {:.2} deg | +2 s {} | +4 s {} | +6 s {}",
            seed,
            missing.map(|m| format!("(IMU {:?} missing 9-12 s)", m)).unwrap_or_default(),
            r.aligned_at, r.horiz_rms, r.horiz_max, r.vert_max, r.vel_max, r.att_max_deg,
            fmt(r.after[0]), fmt(r.after[1]), fmt(r.after[2])
        );
        println!(
            "    IMU updates: A only {}, B only {}, both {}; vertical max at t = {:.3} s; 9-13 s max: position {:.3} m, velocity {:.3} m/s, attitude {:.2} deg",
            r.imu_counts[0], r.imu_counts[1], r.imu_counts[2], r.vert_max_t, r.window_max.0, r.window_max.1, r.window_max.2
        );
        assert!(r.aligned_at.is_some(), "seed {}: never aligned", seed);
        assert!(r.finite, "seed {}: a NaN or infinity in the state", seed);
        let run = format!("seed {}{}", seed, missing.map(|m| format!(", IMU {:?} missing", m)).unwrap_or_default());
        assert!(r.horiz_rms < HORIZ_RMS, "{}: horizontal rms {:.3} m", run, r.horiz_rms);
        assert!(r.vert_max < VERT_MAX, "{}: vertical max {:.3} m", run, r.vert_max);
        assert!(r.vel_max < VEL_MAX, "{}: velocity max {:.3} m/s", run, r.vel_max);
        assert!(r.att_max_deg < ATT_MAX_DEG, "{}: attitude max {:.2} deg", run, r.att_max_deg);
        if let Some(gone) = missing {
            let only = if gone == ImuId::A { r.imu_counts[1] } else { r.imu_counts[0] };
            assert_eq!(only, ((DROPOUT.1 - DROPOUT.0) * CONFIG.imu_rate_hz) as u32, "{}: single-IMU updates", run);
            let (p, v, a) = r.window_max;
            assert!(p < GAP_POS_MAX && v < GAP_VEL_MAX && a < GAP_ATT_MAX_DEG, "{}: in the gap {:.3} m, {:.3} m/s, {:.2} deg", run, p, v, a);
        }
    }
}
