//! The Navigator: the one object the flight software talks to. Each loop it takes
//! any new sensor samples plus the "armed" flag, and returns the vehicle state and status.
//!
//! Start-up: Waiting (collect still IMU and mag readings) -> Aligned (filter started;
//! LiDAR ground unknown) -> OnPad (ground known) -> Flying (first loop with `armed`).
//! On the pad, zero-velocity and zero-rate updates run at a reduced rate, and stop
//! as soon as `armed` is set, not at lift-off.
//!
//! Rules: no IMU sample means no prediction; each sensor sample is used only if newer
//! than the last one used; every outcome is counted as accepted (fused), rejected
//! (refused by the filter, or a bad timestamp) or skipped (couldn't apply).
use crate::*;
use super::{combine_imus, GnssSample, ImuId, ImuPairConfig, ImuPairStatus, ImuSample, LidarSample, MagSample, StillnessWindow};

/// Everything the navigator needs to know: built from the Monoprop parameters by `navigator_config()`.
#[derive(Clone, Copy, Debug)]
pub struct NavConfig {
    pub imu_pair: ImuPairConfig,
    pub imu_spec: ImuSpec,
    pub lidar: LidarConfig,         // ground_z is set at start-up
    pub antenna_offset: V3,         // GNSS antenna position in the body frame (m)
    pub gravity: f64,               // m/s^2, for the stillness check
    pub sigma_pos: f64,             // m: uncertainty of the home position at alignment
    pub sigma_mag: f64,             // unit-direction sigma of update_mag
    pub sigma_lidar_range: f64,     // m
    pub sigma_zero_velocity: f64,   // m/s
    pub sigma_zero_rate: f64,       // rad/s, of ONE gyro sample (divided by sqrt(n) for a mean of n)
    pub stationary_period_us: u64,  // time between stationary updates on the pad
    pub max_imu_gap_us: u64,        // a longer gap between IMU samples is not integrated
    pub still_window_samples: u32,  // still IMU samples needed before aligning
    pub still_min_mag_samples: u32, // magnetometer samples needed before aligning
    pub still_accel_tol: f64,       // m/s^2
    pub still_gyro_max: f64,        // rad/s
}

/// One loop's new samples (None: nothing new from that sensor) and the main loop's armed flag.
#[derive(Clone, Copy, Debug, Default)]
pub struct NavInputs {
    pub imu_a: Option<ImuSample>,
    pub imu_b: Option<ImuSample>,
    pub mag: Option<MagSample>,
    pub lidar: Option<LidarSample>,
    pub gnss: Option<GnssSample>,
    /// Motor armed, spinning or launched: ends the pad phase.
    pub armed: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StartupState {
    Waiting,
    Aligned,
    OnPad,
    Flying,
}

/// Outcome counts for one sensor or update (saturating).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SensorCounts {
    pub accepted: u32,
    pub rejected: u32,
    pub skipped: u32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NavCounts {
    pub imu: SensorCounts,
    pub mag: SensorCounts,
    pub lidar: SensorCounts,
    pub gnss: SensorCounts,
    pub zero_velocity: SensorCounts,
    pub zero_rate: SensorCounts,
    pub imu_gaps: u32,          // IMU gaps too long to integrate
    pub imu_mode_switches: u32, // changes of which IMUs are used (both, A only, B only)
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NavStatus {
    pub startup: StartupState,
    pub imu_pair: Option<ImuPairStatus>, // of the last IMU sample used
    pub lidar_ground_known: bool,
    pub counts: NavCounts,
}

/// The estimate at the time of the last IMU sample used.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VehicleEstimate {
    pub t_us: u64,
    pub position: V3,     // m, navigation frame (ENU from home)
    pub velocity: V3,     // m/s, navigation frame
    pub attitude: Quat,   // body -> navigation
    pub angular_rate: V3, // rad/s, body frame, gyro bias removed
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NavOutput {
    pub estimate: Option<VehicleEstimate>, // None until aligned
    pub status: NavStatus,
}

pub struct Navigator {
    cfg: NavConfig,
    startup: StartupState,
    filter: Option<Esekf>,
    lidar: LidarConfig, // cfg.lidar with the ground height found at start-up
    lidar_ground_known: bool,
    window: StillnessWindow,
    last_imu_t: Option<u64>,
    last_gyro: V3, // last combined gyro reading used (body frame)
    imu_pair: Option<ImuPairStatus>,
    last_mag_t: Option<u64>,
    last_lidar_t: Option<u64>,
    last_gnss_t: Option<u64>,
    last_stationary_t: u64,
    pad_gyro_sum: V3,
    pad_gyro_count: u32,
    counts: NavCounts,
}

/// What the IMU timestamp check decided.
enum Timing {
    First,
    Step(f64), // dt (s)
    Gap,
    Bad,
}

fn bump(c: &mut u32) {
    *c = c.saturating_add(1);
}

fn record(c: &mut SensorCounts, used: bool) {
    if used { bump(&mut c.accepted) } else { bump(&mut c.rejected) }
}

/// Which IMUs a combined sample came from: for counting mode switches.
fn imu_set(status: &ImuPairStatus) -> Option<ImuId> {
    match status {
        ImuPairStatus::Both => None,
        ImuPairStatus::Single { used, .. } => Some(*used),
    }
}

/// True (and remembers t) if a sample at t is newer than the last one used.
fn newer(last: &mut Option<u64>, t: u64) -> bool {
    match *last {
        Some(prev) if t <= prev => false,
        _ => {
            *last = Some(t);
            true
        }
    }
}

impl Navigator {
    pub fn new(cfg: NavConfig) -> Self {
        Navigator {
            cfg,
            startup: StartupState::Waiting,
            filter: None,
            lidar: cfg.lidar,
            lidar_ground_known: false,
            window: StillnessWindow::new(),
            last_imu_t: None,
            last_gyro: V3::zeros(),
            imu_pair: None,
            last_mag_t: None,
            last_lidar_t: None,
            last_gnss_t: None,
            last_stationary_t: 0,
            pad_gyro_sum: V3::zeros(),
            pad_gyro_count: 0,
            counts: NavCounts::default(),
        }
    }

    /// One loop: IMU (start-up or prediction, then the pad updates), then the other sensors.
    pub fn step(&mut self, inputs: &NavInputs) -> NavOutput {
        if inputs.armed {
            match self.startup {
                StartupState::Aligned | StartupState::OnPad => self.startup = StartupState::Flying,
                StartupState::Waiting => self.window.clear(), // a running motor is not "still"
                StartupState::Flying => {}
            }
        }
        self.step_imu(inputs);
        self.step_mag(inputs.mag.as_ref());
        self.step_lidar(inputs.lidar.as_ref());
        self.step_gnss(inputs.gnss.as_ref());
        NavOutput { estimate: self.estimate(), status: self.status() }
    }

    fn timing(&mut self, t: u64) -> Timing {
        let Some(last) = self.last_imu_t else {
            self.last_imu_t = Some(t);
            return Timing::First;
        };
        match t.checked_sub(last) {
            None | Some(0) => Timing::Bad,
            Some(gap) => {
                self.last_imu_t = Some(t);
                if gap > self.cfg.max_imu_gap_us { Timing::Gap } else { Timing::Step(gap as f64 * 1e-6) }
            }
        }
    }

    fn step_imu(&mut self, inputs: &NavInputs) {
        if inputs.imu_a.is_none() && inputs.imu_b.is_none() {
            return; // nothing new: no prediction
        }
        let Some(c) = combine_imus(inputs.imu_a.as_ref(), inputs.imu_b.as_ref(), &self.cfg.imu_pair) else {
            bump(&mut self.counts.imu.rejected);
            return;
        };
        let t = c.sample.t_us();
        let timing = self.timing(t);
        if let Timing::Bad = timing {
            bump(&mut self.counts.imu.rejected);
            return;
        }
        if let Some(prev) = self.imu_pair {
            if imu_set(&prev) != imu_set(&c.status) {
                bump(&mut self.counts.imu_mode_switches);
            }
        }
        self.imu_pair = Some(c.status);
        let (accel, gyro) = (c.sample.accel(), c.sample.gyro());
        match self.startup {
            StartupState::Waiting => self.step_waiting(&accel, &gyro, t, &timing, inputs.armed),
            _ => {
                let Timing::Step(dt) = timing else {
                    // first sample or a gap: nothing to integrate over
                    if let Timing::Gap = timing {
                        bump(&mut self.counts.imu_gaps);
                    }
                    bump(&mut self.counts.imu.skipped);
                    return;
                };
                let Some(f) = self.filter.as_mut() else { return };
                let used = f.predict(&accel, &gyro, dt);
                record(&mut self.counts.imu, used);
                if used {
                    self.last_gyro = gyro;
                    self.step_pad(&gyro, t);
                }
            }
        }
    }

    fn step_waiting(&mut self, accel: &V3, gyro: &V3, t: u64, timing: &Timing, armed: bool) {
        if armed {
            bump(&mut self.counts.imu.skipped);
            return;
        }
        if let Timing::Gap = timing {
            bump(&mut self.counts.imu_gaps);
            self.window.clear(); // readings were lost: stillness can't be assumed across the gap
        }
        bump(&mut self.counts.imu.accepted);
        let c = &self.cfg;
        self.window.add_imu(accel, gyro, c.gravity, c.still_accel_tol, c.still_gyro_max);
        if self.window.imu_count() < c.still_window_samples || self.window.mag_count() < c.still_min_mag_samples {
            return;
        }
        let (Some(a), Some(m)) = (self.window.mean_accel(), self.window.mean_mag()) else { return };
        if let Some(f) = align(&a, &m, &V3::zeros(), c.imu_spec, c.sigma_pos) {
            self.filter = Some(f);
            self.startup = StartupState::Aligned;
            self.last_gyro = *gyro;
            self.last_stationary_t = t;
            self.pad_gyro_sum = V3::zeros();
            self.pad_gyro_count = 0;
        }
        // aligned, or TRIAD couldn't use these readings: either way the next window starts empty
        self.window.clear();
    }

    /// Stationary updates while on the pad and not armed, at the reduced rate, using the mean gyro reading.
    fn step_pad(&mut self, gyro: &V3, t: u64) {
        if !matches!(self.startup, StartupState::Aligned | StartupState::OnPad) {
            return;
        }
        self.pad_gyro_sum += gyro;
        self.pad_gyro_count = self.pad_gyro_count.saturating_add(1);
        if t.saturating_sub(self.last_stationary_t) < self.cfg.stationary_period_us || self.pad_gyro_count == 0 {
            return;
        }
        let n = f64::from(self.pad_gyro_count);
        let mean_gyro = self.pad_gyro_sum / n;
        let (sigma_v, sigma_w) = (self.cfg.sigma_zero_velocity, self.cfg.sigma_zero_rate / n.sqrt());
        if let Some(f) = self.filter.as_mut() {
            record(&mut self.counts.zero_velocity, f.update_zero_velocity(sigma_v));
            record(&mut self.counts.zero_rate, f.update_zero_rate(&mean_gyro, sigma_w));
        }
        self.last_stationary_t = t;
        self.pad_gyro_sum = V3::zeros();
        self.pad_gyro_count = 0;
    }

    fn step_mag(&mut self, mag: Option<&MagSample>) {
        let Some(s) = mag else { return };
        if !newer(&mut self.last_mag_t, s.t_us()) {
            bump(&mut self.counts.mag.skipped);
            return;
        }
        match self.filter.as_mut() {
            // Waiting: collected for the alignment, not fused
            None => {
                self.window.add_mag(&s.field());
                bump(&mut self.counts.mag.skipped);
            }
            Some(f) => record(&mut self.counts.mag, f.update_mag(&s.field(), &mag_reference(), self.cfg.sigma_mag)),
        }
    }

    fn step_lidar(&mut self, lidar: Option<&LidarSample>) {
        let Some(s) = lidar else { return };
        if !newer(&mut self.last_lidar_t, s.t_us()) {
            bump(&mut self.counts.lidar.skipped);
            return;
        }
        let Some(f) = self.filter.as_mut() else {
            bump(&mut self.counts.lidar.skipped);
            return;
        };
        let range = s.range();
        if !self.lidar_ground_known {
            // only on the pad: the first usable reading defines the ground
            let ground = if self.startup == StartupState::Aligned { lidar_ground_z(&f.x, range, &self.lidar) } else { None };
            match ground {
                Some(g) => {
                    self.lidar.ground_z = g;
                    self.lidar_ground_known = true;
                    self.startup = StartupState::OnPad;
                    bump(&mut self.counts.lidar.accepted);
                }
                None => bump(&mut self.counts.lidar.skipped),
            }
            return;
        }
        let in_range = range >= self.lidar.min_range && range <= self.lidar.max_range;
        if !in_range || lidar_prediction(&f.x, &self.lidar).is_none() {
            bump(&mut self.counts.lidar.skipped);
            return;
        }
        record(&mut self.counts.lidar, f.update_lidar(range, self.cfg.sigma_lidar_range, &self.lidar));
    }

    fn step_gnss(&mut self, gnss: Option<&GnssSample>) {
        let Some(s) = gnss else { return };
        if !newer(&mut self.last_gnss_t, s.t_us()) {
            bump(&mut self.counts.gnss.skipped);
            return;
        }
        match self.filter.as_mut() {
            None => bump(&mut self.counts.gnss.skipped),
            Some(f) => record(&mut self.counts.gnss, f.update_gps_offset(&s.position(), &s.sigma(), &self.cfg.antenna_offset)),
        }
    }

    /// The current estimate; None until aligned.
    pub fn estimate(&self) -> Option<VehicleEstimate> {
        let f = self.filter.as_ref()?;
        Some(VehicleEstimate {
            t_us: self.last_imu_t.unwrap_or(0),
            position: f.x.p,
            velocity: f.x.v,
            attitude: f.x.q,
            angular_rate: self.last_gyro - f.x.wb,
        })
    }

    pub fn status(&self) -> NavStatus {
        NavStatus { startup: self.startup, imu_pair: self.imu_pair, lidar_ground_known: self.lidar_ground_known, counts: self.counts }
    }

    /// Position (m, navigation frame) for telemetry; None until aligned.
    pub fn position_f32(&self) -> Option<[f32; 3]> {
        self.filter.as_ref().map(|f| f.position_f32())
    }

    /// Velocity (m/s, navigation frame) for telemetry; None until aligned.
    pub fn velocity_f32(&self) -> Option<[f32; 3]> {
        self.filter.as_ref().map(|f| f.velocity_f32())
    }

    /// Attitude as [qx, qy, qz, qw] for telemetry; None until aligned.
    pub fn orientation_xyzw_f32(&self) -> Option<[f32; 4]> {
        self.filter.as_ref().map(|f| f.orientation_xyzw_f32())
    }

    /// Body angular rate (rad/s) with the estimated gyro bias removed, for telemetry; None until aligned.
    pub fn angular_velocity_f32(&self) -> Option<[f32; 3]> {
        self.filter.as_ref().map(|f| f.angular_velocity_f32(&self.last_gyro))
    }

    /// One standard deviation of each error-state component, for telemetry; None until aligned.
    pub fn sigmas_f32(&self) -> Option<[f32; 15]> {
        self.filter.as_ref().map(|f| f.sigmas_f32())
    }
}
