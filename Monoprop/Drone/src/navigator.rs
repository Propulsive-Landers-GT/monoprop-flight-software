use Common::DroneTelemetry;
use Estimation::{align, lidar_ground_z, mag_reference, Esekf, ImuSpec, LidarConfig, Nominal, GRAVITY, V3};

use crate::config::Config;
use crate::imu_pair::ImuPair;
use crate::sensors::{self, ImuSample, SensorData};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VehicleState {
    pub t_us: u64,
    pub position: [f64; 3],         // m, east, north, up from home
    pub velocity: [f64; 3],         // m/s
    pub attitude: [f64; 4],         // w, x, y, z; body -> navigation
    pub angular_velocity: [f64; 3], // rad/s, body, gyro bias removed
}

/// Sums of the readings collected while waiting to align.
#[derive(Clone, Copy, Debug, Default)]
struct Window {
    accel: V3,
    imu: u32,
    mag: V3,
    mags: u32,
    range: f64,
    ranges: u32,
}

pub struct Navigator {
    cfg: Config,
    lidar: LidarConfig,
    pair: ImuPair,
    filter: Option<Esekf>,
    window: Window,
    gyro: V3,
    last_imu: Option<u64>,
    last_mag: Option<u64>,
    last_lidar: Option<u64>,
    last_gps: Option<u64>,
}

fn v3(a: [f64; 3]) -> V3 {
    V3::new(a[0], a[1], a[2])
}

/// True, remembering `t`, if a sample at `t` is newer than the last one used.
fn newer(last: &mut Option<u64>, t: u64) -> bool {
    let is_newer = last.is_none_or(|l| t > l);
    if is_newer {
        *last = Some(t);
    }
    is_newer
}

impl Navigator {
    pub fn new(cfg: &Config) -> Self {
        sensors::init();
        Navigator {
            cfg: *cfg,
            lidar: LidarConfig {
                offset: v3(cfg.lidar_offset),
                beam: v3(cfg.lidar_beam),
                ground_z: -cfg.pad_imu_height_above_pad,
                min_range: cfg.lidar_min_range,
                max_range: cfg.lidar_max_range,
                max_tilt: cfg.lidar_max_tilt,
            },
            pair: ImuPair::new(cfg),
            filter: None,
            window: Window::default(),
            gyro: V3::zeros(),
            last_imu: None,
            last_mag: None,
            last_lidar: None,
            last_gps: None,
        }
    }

    /// Use the new samples and return the vehicle state, None until aligned.
    pub fn update(&mut self, data: &SensorData) -> Option<VehicleState> {
        // first, on every call: the pair learns its A-B difference and clock offset from the first pair on
        if let Some(s) = self.pair.combine(data.imu_a.as_ref(), data.imu_b.as_ref()) {
            self.imu(&s);
        }
        if let Some(s) = data.mag.filter(|s| newer(&mut self.last_mag, s.t_us)) {
            match self.filter.as_mut() {
                Some(f) => {
                    f.update_mag(&v3(s.field), &mag_reference(), self.cfg.mag_sigma);
                }
                None => {
                    self.window.mag += v3(s.field);
                    self.window.mags += 1;
                }
            }
        }
        if let Some(s) = data.lidar.filter(|s| newer(&mut self.last_lidar, s.t_us)) {
            match self.filter.as_mut() {
                Some(f) => {
                    f.update_lidar(s.range, self.cfg.lidar_sigma, &self.lidar);
                }
                None if s.range >= self.lidar.min_range && s.range <= self.lidar.max_range => {
                    self.window.range += s.range;
                    self.window.ranges += 1;
                }
                None => {}
            }
        }
        if let Some(s) = data.gps.filter(|s| newer(&mut self.last_gps, s.t_us)) {
            if let Some(f) = self.filter.as_mut() {
                f.update_gps_offset(&v3(s.position), &v3(s.sigma), &v3(self.cfg.gps_antenna_offset));
            }
        }
        self.state()
    }

    pub fn state(&self) -> Option<VehicleState> {
        let f = self.filter.as_ref()?;
        let q = f.x.q.quaternion();
        Some(VehicleState {
            t_us: self.last_imu.unwrap_or(0),
            position: f.x.p.into(),
            velocity: f.x.v.into(),
            attitude: [q.w, q.i, q.j, q.k],
            angular_velocity: (self.gyro - f.x.wb).into(),
        })
    }

    fn imu(&mut self, s: &ImuSample) {
        let dt_us = match self.last_imu {
            Some(last) if s.t_us <= last => return,
            Some(last) => Some(s.t_us - last),
            None => None,
        };
        self.last_imu = Some(s.t_us);
        let (accel, gyro) = (v3(s.accel), v3(s.gyro));
        match self.filter.as_mut() {
            Some(f) => {
                // a gap longer than max_gap_us isn't integrated: the clock resyncs on this sample
                let dt = dt_us.filter(|&d| d <= self.cfg.imu_max_gap_us);
                if dt.is_some_and(|d| f.predict(&accel, &gyro, d as f64 * 1e-6)) {
                    self.gyro = gyro;
                }
            }
            None => self.collect(accel, gyro),
        }
    }

    /// Add a still IMU reading to the alignment window (motion restarts it) and align once it is full.
    fn collect(&mut self, accel: V3, gyro: V3) {
        let c = &self.cfg;
        if (accel.norm() - GRAVITY).abs() > c.align_accel_tol || gyro.norm() > c.align_gyro_max {
            self.window = Window::default();
            return;
        }
        let w = &mut self.window;
        w.accel += accel;
        w.imu += 1;
        if w.imu < c.align_window_samples || w.mags == 0 {
            return;
        }
        let spec = ImuSpec::from_densities(
            c.imu_accel_noise_density,
            c.imu_gyro_noise_density,
            c.imu_accel_bias_random_walk,
            c.imu_gyro_bias_random_walk,
            c.imu_rate_hz,
        );
        let mag = w.mag / f64::from(w.mags);
        if let Some(mut f) = align(&(w.accel / f64::from(w.imu)), &mag, &V3::zeros(), spec, c.align_sigma_pos) {
            // the stillness check has just shown the vehicle at rest: velocity known to velocity_sigma
            let var = c.align_velocity_sigma * c.align_velocity_sigma;
            f.p.fixed_view_mut::<3, 15>(3, 0).fill(0.0);
            f.p.fixed_view_mut::<15, 3>(0, 3).fill(0.0);
            f.p.fixed_view_mut::<3, 3>(3, 3).copy_from(&(Estimation::M3::identity() * var));
            // The pad ground is taken at home (height 0); the filter's own height isn't used.
            // TODO: add a truncation check back if pad readings near the lidar's 4 cm floor bias the ground
            // height (it mattered only with the 5 cm noise placeholder).
            if w.ranges >= c.pad_ground_samples {
                let at_home = Nominal { p: V3::zeros(), ..f.x };
                if let Some(g) = lidar_ground_z(&at_home, w.range / f64::from(w.ranges), &self.lidar) {
                    self.lidar.ground_z = g;
                }
            }
            self.gyro = gyro;
            self.filter = Some(f);
        }
        self.window = Window::default();
    }
}

/// The telemetry packet for a state, encoded into `buf`; the number of bytes used.
pub fn encode_telemetry(s: &VehicleState, buf: &mut [u8; 64]) -> Option<usize> {
    let f32s = |a: [f64; 3]| a.map(|x| x as f32);
    let t = DroneTelemetry {
        timestamp_us: s.t_us,
        orientation: [s.attitude[1] as f32, s.attitude[2] as f32, s.attitude[3] as f32, s.attitude[0] as f32],
        position: f32s(s.position),
        velocity: f32s(s.velocity),
        angular_vel: f32s(s.angular_velocity),
    };
    postcard::to_slice(&t, buf).ok().map(|b| b.len())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn worst_case_packet_fits_the_buffer() {
        let s = VehicleState { t_us: u64::MAX, position: [f64::MAX; 3], velocity: [-1e30; 3], attitude: [1.0, 0.0, 0.0, 0.0], angular_velocity: [f64::NAN; 3] };
        let mut buf = [0u8; 64];
        assert_eq!(encode_telemetry(&s, &mut buf), Some(62), "10-byte varint + 13 f32");
    }
}
