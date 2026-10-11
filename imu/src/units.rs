//! Conversions from common vendor units into the SI units of [`ImuSample`](crate::ImuSample).

/// Standard gravity [m/s^2].
pub const G0: f64 = 9.806_65;

pub const fn g_to_mps2(g: f64) -> f64 {
    g * G0
}

pub const fn deg_to_rad(deg: f64) -> f64 {
    deg * (core::f64::consts::PI / 180.0)
}

pub const fn gauss_to_tesla(gauss: f64) -> f64 {
    gauss * 1e-4
}

pub const fn microtesla_to_tesla(ut: f64) -> f64 {
    ut * 1e-6
}

pub const fn kpa_to_pa(kpa: f64) -> f64 {
    kpa * 1e3
}

pub const fn hpa_to_pa(hpa: f64) -> f64 {
    hpa * 1e2
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-12 * b.abs().max(1.0)
    }

    #[test]
    fn conversions() {
        assert!(close(g_to_mps2(1.0), 9.80665));
        assert!(close(deg_to_rad(180.0), core::f64::consts::PI));
        assert!(close(gauss_to_tesla(0.5), 50e-6));
        assert!(close(microtesla_to_tesla(50.0), 50e-6));
        assert!(close(kpa_to_pa(101.325), 101_325.0));
        assert!(close(hpa_to_pa(1013.25), 101_325.0));
    }
}
