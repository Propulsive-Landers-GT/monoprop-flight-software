// Tests for src/monoprop/inputs.rs and src/monoprop/params.rs.
#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::monoprop as mp; // by path: the simulator in support/ has its own ImuSample
use Estimation::*;

const BAD: [f64; 3] = [f64::NAN, f64::INFINITY, f64::NEG_INFINITY];

/// `good` with component `axis` replaced by `value`.
fn with(good: &V3, axis: usize, value: f64) -> V3 {
    let mut v = *good;
    if let Some(c) = v.iter_mut().nth(axis) {
        *c = value;
    }
    v
}

pub fn run() {
    let accel = V3::new(0.1, -0.2, 9.81);
    let gyro = V3::new(0.01, 0.02, -0.03);
    let field = V3::new(-2.0, 22.0, -44.3);
    let position = V3::new(1.0, -2.0, 3.0);
    let sigma = V3::new(0.5, 0.6, 1.2);

    test("valid samples construct and return what went in", || {
        let imu = mp::ImuSample::new(7, accel, gyro);
        assert!(imu.is_some(), "IMU sample");
        if let Some(s) = imu {
            assert_eq!(s.t_us(), 7);
            assert_eq!(s.accel(), accel);
            assert_eq!(s.gyro(), gyro);
        }
        let mag = mp::MagSample::new(8, field);
        assert!(mag.is_some(), "mag sample");
        if let Some(s) = mag {
            assert_eq!(s.t_us(), 8);
            assert_eq!(s.field(), field);
        }
        let lidar = mp::LidarSample::new(9, 1.25);
        assert!(lidar.is_some(), "lidar sample");
        if let Some(s) = lidar {
            assert_eq!(s.t_us(), 9);
            assert_eq!(s.range(), 1.25);
        }
        let gnss = mp::GnssSample::new(10, position, sigma);
        assert!(gnss.is_some(), "GNSS sample");
        if let Some(s) = gnss {
            assert_eq!(s.t_us(), 10);
            assert_eq!(s.position(), position);
            assert_eq!(s.sigma(), sigma);
        }
    });
    test("NaN and infinity are rejected in every component of every sample", || {
        for bad in BAD {
            for axis in 0..3 {
                assert!(mp::ImuSample::new(0, with(&accel, axis, bad), gyro).is_none(), "accel {} axis {}", bad, axis);
                assert!(mp::ImuSample::new(0, accel, with(&gyro, axis, bad)).is_none(), "gyro {} axis {}", bad, axis);
                assert!(mp::MagSample::new(0, with(&field, axis, bad)).is_none(), "field {} axis {}", bad, axis);
                assert!(mp::GnssSample::new(0, with(&position, axis, bad), sigma).is_none(), "position {} axis {}", bad, axis);
                assert!(mp::GnssSample::new(0, position, with(&sigma, axis, bad)).is_none(), "sigma {} axis {}", bad, axis);
            }
            assert!(mp::LidarSample::new(0, bad).is_none(), "range {}", bad);
        }
    });
    test("a zero or negative lidar range is rejected", || {
        assert!(mp::LidarSample::new(0, 0.0).is_none(), "zero");
        assert!(mp::LidarSample::new(0, -0.5).is_none(), "negative");
    });
    test("a zero-length magnetometer field is rejected", || {
        assert!(mp::MagSample::new(0, V3::zeros()).is_none(), "zero field");
        assert!(mp::MagSample::new(0, V3::new(0.0, 0.0, 1e-9)).is_some(), "a tiny field still has a direction");
    });
    test("a zero or negative GNSS sigma is rejected on each axis", || {
        for axis in 0..3 {
            assert!(mp::GnssSample::new(0, position, with(&sigma, axis, 0.0)).is_none(), "zero, axis {}", axis);
            assert!(mp::GnssSample::new(0, position, with(&sigma, axis, -0.5)).is_none(), "negative, axis {}", axis);
        }
    });
    test("every parameter is finite and every noise value is positive", || {
        let values = [
            ("STANDARD_GRAVITY", mp::STANDARD_GRAVITY),
            ("GYRO_NOISE_DENSITY", mp::bmi323::GYRO_NOISE_DENSITY),
            ("ACCEL_NOISE_DENSITY", mp::bmi323::ACCEL_NOISE_DENSITY),
            ("GYRO_ZERO_RATE_OFFSET", mp::bmi323::GYRO_ZERO_RATE_OFFSET),
            ("ACCEL_OFFSET", mp::bmi323::ACCEL_OFFSET),
            ("ACCEL_BIAS_RANDOM_WALK", mp::bmi323::ACCEL_BIAS_RANDOM_WALK),
            ("GYRO_BIAS_RANDOM_WALK", mp::bmi323::GYRO_BIAS_RANDOM_WALK),
            ("IMU_RATE_HZ", mp::IMU_RATE_HZ),
            ("SIGMA_POS", mp::SIGMA_POS),
            ("SIGMA_MAG", mp::SIGMA_MAG),
        ];
        for (name, v) in values {
            assert!(v.is_finite() && v > 0.0, "{} = {}", name, v);
        }
    });
    test("the datasheet values convert to SI as documented", || {
        assert_close(mp::bmi323::GYRO_NOISE_DENSITY, 0.007 * PI / 180.0, 1e-18, "0.007 deg/s/sqrt(Hz)");
        assert_close(mp::bmi323::ACCEL_NOISE_DENSITY, 180.0e-6 * 9.80665, 1e-18, "180 ug/sqrt(Hz)");
        assert_close(mp::bmi323::GYRO_ZERO_RATE_OFFSET, PI / 180.0, 1e-18, "1 deg/s");
        assert_close(mp::bmi323::ACCEL_OFFSET, 50.0e-3 * 9.80665, 1e-18, "50 mg");
    });
    test("bmi323_spec(400.0) has finite, positive sigmas built from the densities", || {
        let spec = mp::bmi323_spec(400.0);
        for (name, v) in [("sigma_a", spec.sigma_a), ("sigma_g", spec.sigma_g), ("sigma_aw", spec.sigma_aw), ("sigma_gw", spec.sigma_gw)] {
            assert!(v.is_finite() && v > 0.0, "{} = {}", name, v);
        }
        assert_close(spec.sigma_a, per_sample_sigma(mp::bmi323::ACCEL_NOISE_DENSITY, 400.0), 1e-18, "sigma_a");
        assert_close(spec.sigma_g, per_sample_sigma(mp::bmi323::GYRO_NOISE_DENSITY, 400.0), 1e-18, "sigma_g");
        assert_close(spec.sigma_aw, mp::bmi323::ACCEL_BIAS_RANDOM_WALK, 0.0, "sigma_aw");
        assert_close(spec.sigma_gw, mp::bmi323::GYRO_BIAS_RANDOM_WALK, 0.0, "sigma_gw");
    });
    test("mounting placeholders are identity rotations and zero offsets", || {
        for r in [mp::imu_mount_rotation(), mp::mag_mount_rotation(), mp::lidar_mount_rotation(), mp::gnss_mount_rotation()] {
            assert_eq!(r, M3::identity());
        }
        for p in [mp::imu_mount_offset(), mp::mag_mount_offset(), mp::lidar_mount_offset(), mp::gnss_mount_offset()] {
            assert_eq!(p, V3::zeros());
        }
    });
}
