#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

pub fn run() {
    let rs = [V3::new(0.3, -0.5, 1.1), V3::new(-2.0, 0.4, 0.9), V3::new(1e-7, -2e-7, 5e-8), V3::new(0.0, 0.0, 3.0)];
    test("i (x) j = k", || {
        let k = hamilton(&Quaternion::new(0.0, 1.0, 0.0, 0.0), &Quaternion::new(0.0, 0.0, 1.0, 0.0));
        assert_eq!(k, Quaternion::new(0.0, 0.0, 0.0, 1.0));
    });
    test("j (x) i = -k: the order matters", || {
        let k = hamilton(&Quaternion::new(0.0, 0.0, 1.0, 0.0), &Quaternion::new(0.0, 1.0, 0.0, 0.0));
        assert_eq!(k, Quaternion::new(0.0, 0.0, 0.0, -1.0));
    });
    test("matches nalgebra's p * q", || {
        let (p, q) = (Quaternion::new(0.3, -1.0, 2.0, 0.5), Quaternion::new(-0.7, 0.2, 0.1, 1.5));
        assert!((hamilton(&p, &q) - p * q).norm() < 1e-14, "got {}, nalgebra {}", hamilton(&p, &q), p * q);
    });
    test("the worked example: 90 degrees about z", || {
        let q = Quat::from_scaled_axis(V3::new(0.0, 0.0, FRAC_PI_2));
        let r = quat_to_rot(&q);
        assert!((r - M3::new(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)).amax() < 1e-12, "got\n{}", r);
    });
    test("quat_to_rot matches nalgebra and is a proper rotation", || {
        for v in rs.iter() {
            let q = Quat::from_scaled_axis(*v);
            let r = quat_to_rot(&q);
            assert!((r - q.to_rotation_matrix().into_inner()).amax() < 1e-12, "for rotation vector {}", v.transpose());
            assert!((r.transpose() * r - M3::identity()).amax() < 1e-12, "R^T R must be I");
            assert_close(r.determinant(), 1.0, 1e-12, "det R");
        }
    });
    test("R(p (x) q) = R(p) R(q)", || {
        let p = Quat::from_scaled_axis(V3::new(0.1, 0.2, -0.3));
        let q = Quat::from_scaled_axis(V3::new(-0.5, 0.4, 1.0));
        let pq = Quat::new_normalize(hamilton(&p, &q));
        assert!((quat_to_rot(&pq) - quat_to_rot(&p) * quat_to_rot(&q)).amax() < 1e-12);
    });
}
