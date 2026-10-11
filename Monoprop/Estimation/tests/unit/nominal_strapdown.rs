#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

fn perfect() -> ImuSpec {
    ImuSpec { sigma_a: 0.0, sigma_g: 0.0, sigma_aw: 0.0, sigma_gw: 0.0 }
}

pub fn run() {
    test("level and still: nothing moves", || {
        let q = quat_exp(&V3::new(0.1, 0.2, 0.3));
        let x0 = Nominal { p: V3::new(1.0, 2.0, 3.0), v: V3::zeros(), q, ab: V3::zeros(), wb: V3::zeros() };
        let mut x = x0;
        for _ in 0..1000 {
            x = propagate_nominal(&x, &specific_force(&q, &V3::zeros()), &V3::zeros(), 0.005);
        }
        assert!((x.p - x0.p).amax() < 1e-9 && x.v.amax() < 1e-9, "drifted to {}", x.p.transpose());
    });
    test("biases are subtracted before use (the worked example)", || {
        let x0 = Nominal { p: V3::zeros(), v: V3::zeros(), q: Quat::identity(), ab: V3::new(0.1, 0.0, 0.0), wb: V3::new(0.0, 0.0, 0.2) };
        let x = propagate_nominal(&x0, &V3::new(0.1, 0.0, 9.81), &V3::new(0.0, 0.0, 0.2), 0.01);
        assert!(x.v.amax() < 1e-12, "velocity {}: did you subtract the accelerometer bias?", x.v.transpose());
        assert!(x.q.angle() < 1e-12, "attitude turned: did you subtract the gyro bias?");
    });
    test("free fall: velocity grows by g dt", || {
        let x0 = Nominal { p: V3::zeros(), v: V3::zeros(), q: Quat::identity(), ab: V3::zeros(), wb: V3::zeros() };
        let x = propagate_nominal(&x0, &V3::zeros(), &V3::zeros(), 0.1);
        assert!((x.v - V3::new(0.0, 0.0, -0.981)).amax() < 1e-12, "v {}", x.v.transpose());
        assert!((x.p - V3::new(0.0, 0.0, -0.04905)).amax() < 1e-12, "p {} (position uses the old velocity plus a dt^2 / 2)", x.p.transpose());
    });
    test("perfect IMU, known biases: follows the true flight for 5 s", || {
        let (ba, bg) = (V3::new(0.08, -0.05, 0.12), V3::new(0.01, -0.02, 0.015));
        let q0 = quat_exp(&V3::new(0.05, -0.1, 0.3));
        let mut sim = Simulator::new(0.005, perfect(), ba, bg, q0, 1);
        let t0 = sim.truth;
        let mut x = Nominal { p: t0.p, v: t0.v, q: t0.q, ab: ba, wb: bg };
        for _ in 0..1000 {
            let imu = sim.step();
            x = propagate_nominal(&x, &imu.accel, &imu.gyro, sim.dt);
        }
        let ep = (x.p - sim.truth.p).norm();
        let eq = quat_log(&(x.q.inverse() * sim.truth.q)).norm();
        assert!(ep < 0.02, "position error after 5 s: {:.4} m", ep);
        assert!(eq < 1e-9, "attitude error: {:e} rad", eq);
    });
    test("ignoring a 0.05 m/s^2 accelerometer bias drifts about 2.5 m in 10 s", || {
        let ba = V3::new(0.05, 0.0, 0.0);
        let mut sim = Simulator::new(0.005, perfect(), ba, V3::zeros(), Quat::identity(), 1);
        let t0 = sim.truth;
        let mut x = Nominal { p: t0.p, v: t0.v, q: t0.q, ab: V3::zeros(), wb: V3::zeros() };
        for _ in 0..2000 {
            let imu = sim.step();
            x = propagate_nominal(&x, &imu.accel, &imu.gyro, sim.dt);
        }
        let drift = (x.p - sim.truth.p).norm();
        assert!(drift > 1.5 && drift < 3.5, "drift {:.2} m, expected about 2.5 m", drift);
    });
}
