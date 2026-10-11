#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

fn spec() -> ImuSpec {
    ImuSpec { sigma_a: 0.03, sigma_g: 0.003, sigma_aw: 1e-4, sigma_gw: 1e-5 }
}

fn p0() -> Mat<15, 15> {
    diag([1.0, 1.0, 1.0, 0.25, 0.25, 0.25, 0.01, 0.01, 0.03, 0.04, 0.04, 0.04, 1e-3, 1e-3, 1e-3])
}

fn truth_nominal(t: &TruthState) -> Nominal {
    Nominal { p: t.p, v: t.v, q: t.q, ab: t.accel_bias, wb: t.gyro_bias }
}

pub fn run() {
    test("update_gps moves the position towards the fix", || {
        let x = Nominal { p: V3::zeros(), v: V3::zeros(), q: Quat::identity(), ab: V3::zeros(), wb: V3::zeros() };
        let mut f = Esekf::new(x, p0(), spec());
        assert!(f.update_gps(&V3::new(1.0, 0.0, 0.0), 1.0), "accepted");
        assert_close(f.x.p.x, 0.5, 1e-9, "half way (equal variances)");
        assert!(!f.update_gps(&V3::new(100.0, 0.0, 0.0), 0.1) && f.rejected == 1, "a 100 m jump must be gated");
    });
    test("update_mag corrects an attitude error the field can see", || {
        let truth = quat_exp(&V3::new(0.0, 0.0, 0.3));
        let mb = truth.inverse() * mag_reference();
        let axis = mb.cross(&V3::z()); // perpendicular to the field: rotation about the field itself is invisible
        let x = Nominal { p: V3::zeros(), v: V3::zeros(), q: truth * quat_exp(&(axis * (0.03 / axis.norm()))), ab: V3::zeros(), wb: V3::zeros() };
        let mut f = Esekf::new(x, p0(), spec());
        for _ in 0..10 {
            f.update_mag(&mb, &mag_reference(), 0.01);
        }
        assert!(boxminus(&truth, &f.x.q).norm() < 3e-3, "attitude error still {:.4} rad (started at 0.03)", boxminus(&truth, &f.x.q).norm());
    });
    test("30 s flight with GPS and magnetometer: accurate and consistent", || {
        let s = spec();
        let q0 = quat_exp(&V3::new(0.05, -0.1, 0.3));
        let mut sim = Simulator::new(0.005, s, V3::new(0.08, -0.05, 0.12), V3::new(0.01, -0.02, 0.015), q0, 5);
        let t = sim.truth;
        let x0 = Nominal { p: t.p + V3::new(0.5, -0.5, 0.3), v: t.v, q: t.q * quat_exp(&V3::new(0.03, -0.03, 0.08)), ab: V3::zeros(), wb: V3::zeros() };
        let mut f = Esekf::new(x0, p0(), s);
        let (mut se, mut amax, mut nees, mut n) = (0.0, 0.0f64, 0.0, 0);
        for k in 0..6000 {
            let imu = sim.step();
            f.predict(&imu.accel, &imu.gyro, sim.dt);
            if k % 20 == 19 {
                let z = sim.gps(0.5);
                f.update_gps(&z, 0.5);
            }
            if k % 4 == 3 {
                let m = sim.mag(0.02);
                f.update_mag(&m, &mag_reference(), 0.02);
            }
            if sim.truth.t > 15.0 {
                let e = difference(&truth_nominal(&sim.truth), &f.x);
                se += sub3(&e, 0).norm_squared();
                amax = amax.max(sub3(&e, 6).norm());
                if k % 20 == 0 {
                    nees += e.dot(&solve_spd(&f.p, &e).expect("P must stay positive definite"));
                    n += 1;
                }
            }
        }
        let rms = (se / 3000.0).sqrt();
        let nees = nees / n as f64;
        println!("position rms {:.3} m, max attitude error {:.2} deg, average NEES {:.1} (15 is ideal)", rms, amax.to_degrees(), nees);
        assert!(rms < 0.4, "position rms {:.3} m over the last 15 s", rms);
        assert!(amax.to_degrees() < 2.0, "attitude error up to {:.2} deg", amax.to_degrees());
        assert!(nees > 5.0 && nees < 40.0, "average NEES {:.1}: the filter's P doesn't describe its own errors", nees);
    });
    test("update_mag uses only the direction: a reading in microtesla is accepted", || {
        let truth = quat_exp(&V3::new(0.1, -0.05, 0.4));
        let mb = truth.inverse() * mag_reference();
        let x = Nominal { p: V3::zeros(), v: V3::zeros(), q: truth, ab: V3::zeros(), wb: V3::zeros() };
        let mut f = Esekf::new(x, p0(), spec());
        assert!(f.update_mag(&(mb * 49.4), &mag_reference(), 0.02), "a 49.4 uT reading of the right direction must be accepted");
        assert!(f.update_mag(&(mb * 49.4e-6), &mag_reference(), 0.02), "and so must the same reading in tesla");
        assert_eq!(f.rejected, 0);
    });
    test("update_mag refuses a zero or NaN reading and changes nothing", || {
        let x = Nominal { p: V3::zeros(), v: V3::zeros(), q: Quat::identity(), ab: V3::zeros(), wb: V3::zeros() };
        let mut f = Esekf::new(x, p0(), spec());
        let (x0, p_before) = (f.x, f.p);
        assert!(!f.update_mag(&V3::zeros(), &mag_reference(), 0.02), "zero reading");
        assert!(!f.update_mag(&V3::new(f64::NAN, 0.0, 0.0), &mag_reference(), 0.02), "NaN reading");
        assert_eq!(f.rejected, 2, "both counted as rejected");
        assert!(f.x == x0 && f.p == p_before, "state untouched");
    });
}
