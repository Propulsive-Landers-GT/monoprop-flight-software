//! Reads config/monoprop.toml at compile time and writes OUT_DIR/config.rs: a `Config` struct of plain
//! numbers and arrays and the `CONFIG` constant. Any problem fails the build with the message.
use std::fmt::Write as _;
use std::{env, fs, path::Path};

#[derive(Clone, Copy, PartialEq)]
enum Kind {
    Num,   // finite f64
    Noise, // finite f64 > 0
    Count, // integer > 0
    Vec3,  // three finite f64
    Rot,   // 3x3 rotation (rows), R R^T = I and det = 1
    Imu,   // "A" or "B"
}

const KEYS: &[(&str, &str, Kind, &str)] = &[
    ("imu", "rate_hz", Kind::Noise, "f64"),
    ("imu", "accel_noise_density", Kind::Noise, "f64"),
    ("imu", "gyro_noise_density", Kind::Noise, "f64"),
    ("imu", "accel_bias_random_walk", Kind::Noise, "f64"),
    ("imu", "gyro_bias_random_walk", Kind::Noise, "f64"),
    ("imu", "max_gap_us", Kind::Count, "u64"),
    ("imu_pair", "reference", Kind::Imu, "crate::imu_pair::ImuId"),
    ("imu_pair", "a_rotation", Kind::Rot, "[[f64; 3]; 3]"),
    ("imu_pair", "b_rotation", Kind::Rot, "[[f64; 3]; 3]"),
    ("imu_pair", "a_offset", Kind::Vec3, "[f64; 3]"),
    ("imu_pair", "b_offset", Kind::Vec3, "[f64; 3]"),
    ("imu_pair", "max_skew_us", Kind::Count, "u64"),
    ("imu_pair", "diff_window", Kind::Count, "u32"),
    ("imu_pair", "diff_min_pairs", Kind::Count, "u32"),
    ("mag", "sigma", Kind::Noise, "f64"),
    ("lidar", "sigma", Kind::Noise, "f64"),
    ("lidar", "min_range", Kind::Num, "f64"),
    ("lidar", "max_range", Kind::Num, "f64"),
    ("lidar", "max_tilt", Kind::Num, "f64"),
    ("lidar", "offset", Kind::Vec3, "[f64; 3]"),
    ("lidar", "beam", Kind::Vec3, "[f64; 3]"),
    ("gps", "antenna_offset", Kind::Vec3, "[f64; 3]"),
    ("pad", "imu_height_above_pad", Kind::Num, "f64"),
    ("pad", "ground_samples", Kind::Count, "u32"),
    ("align", "window_samples", Kind::Count, "u32"),
    ("align", "accel_tol", Kind::Noise, "f64"),
    ("align", "gyro_max", Kind::Noise, "f64"),
    ("align", "sigma_pos", Kind::Noise, "f64"),
    ("align", "velocity_sigma", Kind::Noise, "f64"),
    ("telemetry", "period_us", Kind::Count, "u64"),
];

fn number(v: &toml::Value) -> Option<f64> {
    v.as_float().or_else(|| v.as_integer().map(|i| i as f64))
}

fn vec3(v: &toml::Value, name: &str) -> Result<[f64; 3], String> {
    let a = v.as_array().filter(|a| a.len() == 3).ok_or(format!("{} must be 3 numbers", name))?;
    let mut out = [0.0; 3];
    for (o, x) in out.iter_mut().zip(a) {
        *o = number(x).filter(|x| x.is_finite()).ok_or(format!("{} must be 3 finite numbers", name))?;
    }
    Ok(out)
}

/// A proper rotation (rows): R R^T = I and det = 1 within 1e-6.
fn rotation(v: &toml::Value, name: &str) -> Result<[[f64; 3]; 3], String> {
    let rows = v.as_array().filter(|a| a.len() == 3).ok_or(format!("{} must be 3 rows", name))?;
    let mut r = [[0.0; 3]; 3];
    for (out, row) in r.iter_mut().zip(rows) {
        *out = vec3(row, name)?;
    }
    for i in 0..3 {
        for j in 0..3 {
            let dot: f64 = (0..3).map(|k| r[i][k] * r[j][k]).sum();
            let want = if i == j { 1.0 } else { 0.0 };
            if (dot - want).abs() > 1e-6 {
                return Err(format!("{} is not a rotation (R R^T != I)", name));
            }
        }
    }
    let det = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
        + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
    if (det - 1.0).abs() > 1e-6 {
        return Err(format!("{} is not a rotation (det = {})", name, det));
    }
    Ok(r)
}

fn main() -> Result<(), String> {
    let path = Path::new("config/monoprop.toml");
    println!("cargo:rerun-if-changed={}", path.display());
    println!("cargo:rerun-if-changed=build.rs");
    let text = fs::read_to_string(path).map_err(|e| format!("{}: {}", path.display(), e))?;
    let table: toml::Table = text.parse().map_err(|e| format!("{}: {}", path.display(), e))?;

    for (section, value) in &table {
        let Some(keys) = value.as_table() else { return Err(format!("[{}] is not a section", section)) };
        for key in keys.keys() {
            if !KEYS.iter().any(|(s, k, _, _)| s == section && k == key) {
                return Err(format!("unknown key {}.{}", section, key));
            }
        }
    }

    let mut fields = String::new();
    let mut values = String::new();
    let get = |section: &str, key: &str| -> Result<f64, String> {
        let v = table.get(section).and_then(|s| s.get(key)).ok_or(format!("missing {}.{}", section, key))?;
        number(v).ok_or(format!("{}.{} is not a number", section, key))
    };
    for &(section, key, kind, ty) in KEYS {
        let name = format!("{}_{}", section, key);
        let full = format!("{}.{}", section, key);
        let raw = table.get(section).and_then(|s| s.get(key));
        let literal = match kind {
            Kind::Vec3 => format!("{:?}", vec3(raw.ok_or(format!("missing {}", full))?, &full)?),
            Kind::Rot => format!("{:?}", rotation(raw.ok_or(format!("missing {}", full))?, &full)?),
            Kind::Imu => match raw.and_then(|v| v.as_str()) {
                Some("A") => "crate::imu_pair::ImuId::A".to_string(),
                Some("B") => "crate::imu_pair::ImuId::B".to_string(),
                _ => return Err(format!("{} must be \"A\" or \"B\"", full)),
            },
            Kind::Count => {
                let v = table.get(section).and_then(|s| s.get(key)).and_then(|v| v.as_integer());
                let v = v.ok_or(format!("{}.{} must be an integer", section, key))?;
                if v <= 0 {
                    return Err(format!("{}.{} must be positive, is {}", section, key, v));
                }
                v.to_string()
            }
            Kind::Num | Kind::Noise => {
                let x = get(section, key)?;
                if !x.is_finite() {
                    return Err(format!("{}.{} must be finite, is {}", section, key, x));
                }
                if kind == Kind::Noise && x <= 0.0 {
                    return Err(format!("{}.{} must be positive, is {}", section, key, x));
                }
                format!("{:?}", x)
            }
        };
        let _ = writeln!(fields, "    pub {}: {},", name, ty);
        let _ = writeln!(values, "    {}: {},", name, literal);
    }
    let (min, max, tilt) = (get("lidar", "min_range")?, get("lidar", "max_range")?, get("lidar", "max_tilt")?);
    if !(0.0..max).contains(&min) {
        return Err(format!("lidar ranges must satisfy 0 <= min_range < max_range, are {} and {}", min, max));
    }
    if tilt <= 0.0 || tilt >= std::f64::consts::FRAC_PI_2 {
        return Err(format!("lidar.max_tilt must be between 0 and pi/2 rad, is {}", tilt));
    }

    let out = format!(
        "/// Vehicle parameters from config/monoprop.toml (generated by build.rs).\n\
         #[derive(Clone, Copy, Debug)]\npub struct Config {{\n{}}}\n\n\
         pub const CONFIG: Config = Config {{\n{}}};\n",
        fields, values
    );
    let dir = env::var("OUT_DIR").map_err(|e| format!("OUT_DIR: {}", e))?;
    fs::write(Path::new(&dir).join("config.rs"), out).map_err(|e| format!("writing config.rs: {}", e))
}
