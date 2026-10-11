// Tests for src/interop.rs.
#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

pub fn run() {
    let spec = ImuSpec { sigma_a: 0.01, sigma_g: 0.001, sigma_aw: 1e-4, sigma_gw: 1e-5 };
    let q = Quat::from_scaled_axis(V3::new(0.1, -0.2, 0.3));
    let x = Nominal { p: V3::new(1.0, 2.0, 3.0), v: V3::new(-1.0, 0.5, 0.25), q, ab: V3::zeros(), wb: V3::new(0.01, 0.02, 0.03) };
    let f = Esekf::new(x, Mat::<15, 15>::identity() * 4.0, spec);

    test("orientation is [qx, qy, qz, qw] (DroneTelemetry order)", || {
        let o = f.orientation_xyzw_f32();
        let c = q.quaternion();
        assert_close(o[0] as f64, c.i, 1e-6, "qx");
        assert_close(o[1] as f64, c.j, 1e-6, "qy");
        assert_close(o[2] as f64, c.k, 1e-6, "qz");
        assert_close(o[3] as f64, c.w, 1e-6, "qw");
    });
    test("position, velocity and bias-corrected rate convert to f32", || {
        assert_eq!(f.position_f32(), [1.0, 2.0, 3.0]);
        assert_eq!(f.velocity_f32(), [-1.0, 0.5, 0.25]);
        let w = f.angular_velocity_f32(&V3::new(0.11, 0.22, 0.33));
        assert_close(w[0] as f64, 0.10, 1e-6, "wx");
        assert_close(w[2] as f64, 0.30, 1e-6, "wz");
        assert_eq!(v3_from_f32([1.5, -2.0, 0.25]), V3::new(1.5, -2.0, 0.25));
    });
    test("sigmas are the square roots of P's diagonal", || {
        assert!(f.sigmas_f32().iter().all(|s| (*s - 2.0).abs() < 1e-6));
    });
}
