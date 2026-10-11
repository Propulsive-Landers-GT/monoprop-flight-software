#![allow(unused_imports, dead_code)]
use crate::support::*;
use Estimation::*;

pub fn run() {
    test("the worked example: v = (1, 2, 3)", || {
        let m = skew(&V3::new(1.0, 2.0, 3.0));
        assert_eq!(m, M3::new(0.0, -3.0, 2.0, 3.0, 0.0, -1.0, -2.0, 1.0, 0.0), "got\n{}", m);
    });
    test("skew(v) * w equals v x w", || {
        let vs = [V3::new(1.0, -2.0, 0.5), V3::new(0.0, 0.0, -9.81), V3::new(0.3, 0.7, -1.1)];
        for v in vs.iter() {
            for w in vs.iter().chain([V3::x(), V3::y(), V3::z()].iter()) {
                let (got, want) = (skew(v) * w, v.cross(w));
                assert!((got - want).amax() < 1e-14, "v = {}, w = {}: skew(v) w = {}, v x w = {}", v.transpose(), w.transpose(), got.transpose(), want.transpose());
            }
        }
    });
    test("skew(v) is skew-symmetric with a zero diagonal", || {
        let m = skew(&V3::new(1.0, -2.0, 0.5));
        assert_eq!(m.transpose(), -m, "transpose should equal the negative");
    });
    test("matches nalgebra's cross_matrix()", || {
        let v = V3::new(0.4, -1.3, 2.2);
        assert_eq!(skew(&v), v.cross_matrix());
    });
}
