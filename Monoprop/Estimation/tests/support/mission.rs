//! The flight loop used by the scenario tests (simulation only).
#![allow(unused_imports)]
use Estimation::*;
use super::sim::*;
use super::helpers::*;

pub struct Mission {
    pub steps: usize,
    pub gps_every: usize,
    pub mag_every: usize,
    pub sigma_gps: f64,
    pub sigma_mag: f64,
}

#[derive(Debug, Default)]
pub struct Report {
    pub pos_rms: f64,     // position error rms over t > SETTLE_TIME
    pub pos_max: f64,     // largest position error over t > SETTLE_TIME
    pub att_max_deg: f64, // largest attitude error over t > SETTLE_TIME, degrees
    pub gps_used: u32,
    pub gps_rejected: u32,
    pub mag_rejected: u32,
}

pub const SETTLE_TIME: f64 = 20.0;

/// Fly the mission. `gps_fault(k, fix)` may corrupt a fix (return Some(bad)) or drop it (None).
pub fn fly(sim: &mut Simulator, m: &Mission, gps_fault: &dyn Fn(usize, V3) -> Option<V3>) -> (Esekf, Report) {
    let mag0 = sim.mag(m.sigma_mag);
    let gps0 = sim.gps(m.sigma_gps);
    let mut filter: Option<Esekf> = None;
    let mut rep = Report::default();
    let (mut sum_sq, mut n) = (0.0, 0usize);
    for k in 0..m.steps {
        let imu = sim.step();
        if filter.is_none() {
            filter = align(&imu.accel, &mag0, &gps0, sim.spec, m.sigma_gps);
        }
        let Some(f) = filter.as_mut() else { continue }; // readings TRIAD can't use: try the next sample
        f.predict(&imu.accel, &imu.gyro, sim.dt);
        if k % m.gps_every == m.gps_every - 1 {
            let fix = sim.gps(m.sigma_gps);
            if let Some(z) = gps_fault(k, fix) {
                if f.update_gps(&z, m.sigma_gps) {
                    rep.gps_used += 1;
                } else {
                    rep.gps_rejected += 1;
                }
            }
        }
        if k % m.mag_every == m.mag_every - 1 {
            let mm = sim.mag(m.sigma_mag);
            if !f.update_mag(&mm, &mag_reference(), m.sigma_mag) {
                rep.mag_rejected += 1;
            }
        }
        if sim.truth.t > SETTLE_TIME {
            let ep = (sim.truth.p - f.x.p).norm();
            sum_sq += ep * ep;
            n += 1;
            rep.pos_max = rep.pos_max.max(ep);
            rep.att_max_deg = rep.att_max_deg.max(boxminus(&sim.truth.q, &f.x.q).norm().to_degrees());
        }
    }
    rep.pos_rms = if n > 0 { (sum_sq / n as f64).sqrt() } else { 0.0 };
    (filter.expect("the filter aligned during the mission"), rep)
}

// ---------------------------------------------------------------------------
// Monoprop flight through the Navigator (sensor values from the NavConfig: PLACEHOLDERS).
// ---------------------------------------------------------------------------

use Estimation::monoprop as mp;

/// What a navigator flight did. Errors are measured from arming to the end of the flight.
#[derive(Debug, Default)]
pub struct NavReport {
    pub horiz_rms: f64,
    pub horiz_max: f64,
    pub vert_max: f64,
    pub vel_max: f64,
    pub att_max_deg: f64,
    /// Seconds since arming at which the vertical and the attitude errors peaked.
    pub vert_max_t: f64,
    pub att_max_t: f64,
    /// The attitude error split, in the navigation frame: about the vertical (yaw) and horizontal (roll/pitch).
    pub yaw_max_deg: f64,
    pub tilt_max_deg: f64,
    /// Yaw and tilt parts of the error at the attitude peak.
    pub att_peak_yaw_deg: f64,
    pub att_peak_tilt_deg: f64,
    pub nees_avg: f64,
    /// Average NEES of each 3-state block on its own: position, velocity, attitude, accel bias, gyro bias
    /// (about 3 each for a consistent filter).
    pub nees_blocks: [f64; 5],
    /// Position NEES split: horizontal (2 states, about 2 if consistent) and vertical (1 state, about 1).
    pub nees_horiz: f64,
    pub nees_vert: f64,
    pub finite: bool,
    pub estimate_before_align: bool,
    pub t_onpad: Option<f64>,
    pub t_ground_measured: Option<f64>,
    pub t_flying: Option<f64>,
    pub saw_single: bool,
    pub lidar_accepted_in_flight: u32,
    /// (seconds since arming, horizontal error m, the filter's own horizontal 1-sigma m), every 5 s.
    pub drift: Vec<(f64, f64, f64)>,
    pub status: Option<mp::NavStatus>,
}

/// Fly the hover in `opt` with the navigator set up by `cfg`.
pub fn fly_navigator(cfg: mp::NavConfig, opt: MonopropOptions) -> NavReport {
    let mut sim = MonopropSim::new(cfg, opt);
    let mut nav = mp::Navigator::new(cfg);
    let mut rep = NavReport { finite: true, ..NavReport::default() };
    let (mut horiz_sq, mut nees_sum, mut n, mut nees_n) = (0.0, 0.0, 0usize, 0usize);
    let mut lidar_at_arm = None;
    let mut next_drift = 0.0;
    let end = hover_end(opt.t_arm);
    while sim.t < end {
        let inputs = sim.step();
        let out = nav.step(&inputs);
        let t = sim.t;
        match out.status.startup {
            mp::StartupState::Waiting => rep.estimate_before_align |= out.estimate.is_some(),
            mp::StartupState::OnPad => {
                rep.t_onpad.get_or_insert(t);
            }
            mp::StartupState::Flying => {
                rep.t_flying.get_or_insert(t);
            }
        }
        if out.status.lidar_ground_measured {
            rep.t_ground_measured.get_or_insert(t);
        }
        rep.saw_single |= matches!(out.status.imu_pair, Some(mp::ImuPairStatus::Single { .. }));
        if t < opt.t_arm {
            continue;
        }
        let lidar_now = out.status.counts.lidar.accepted;
        rep.lidar_accepted_in_flight = lidar_now - *lidar_at_arm.get_or_insert(lidar_now);
        let (Some(e), Some(f)) = (out.estimate, nav.filter()) else { continue };
        let truth = sim.truth_nominal();
        let dp = truth.p - e.position;
        let horiz = (dp.x * dp.x + dp.y * dp.y).sqrt();
        horiz_sq += horiz * horiz;
        n += 1;
        rep.horiz_max = rep.horiz_max.max(horiz);
        if dp.z.abs() > rep.vert_max {
            rep.vert_max = dp.z.abs();
            rep.vert_max_t = t - opt.t_arm;
        }
        rep.vel_max = rep.vel_max.max((truth.v - e.velocity).norm());
        let dtheta = e.attitude * boxminus(&truth.q, &e.attitude); // attitude error, navigation axes
        let (yaw, tilt) = (dtheta.z.abs().to_degrees(), (dtheta.x * dtheta.x + dtheta.y * dtheta.y).sqrt().to_degrees());
        rep.yaw_max_deg = rep.yaw_max_deg.max(yaw);
        rep.tilt_max_deg = rep.tilt_max_deg.max(tilt);
        let att = dtheta.norm().to_degrees();
        if att > rep.att_max_deg {
            rep.att_max_deg = att;
            rep.att_max_t = t - opt.t_arm;
            rep.att_peak_yaw_deg = yaw;
            rep.att_peak_tilt_deg = tilt;
        }
        rep.finite &= f.x.p.iter().chain(f.p.iter()).all(|v| v.is_finite());
        let err = difference(&truth, &f.x);
        if let Some(w) = solve_spd(&f.p, &err) {
            nees_sum += err.dot(&w);
            nees_n += 1;
            let ph: Mat<2, 2> = f.p.fixed_view::<2, 2>(0, 0).into_owned();
            if let Some(wh) = solve_spd(&ph, &Vector::<2>::new(err[0], err[1])) {
                rep.nees_horiz += err[0] * wh[0] + err[1] * wh[1];
            }
            if f.p[(2, 2)] > 0.0 {
                rep.nees_vert += err[2] * err[2] / f.p[(2, 2)];
            }
            for (b, slot) in rep.nees_blocks.iter_mut().enumerate() {
                let pb: Mat<3, 3> = f.p.fixed_view::<3, 3>(3 * b, 3 * b).into_owned();
                let eb = sub3(&err, 3 * b);
                if let Some(wb) = solve_spd(&pb, &eb) {
                    *slot += eb.dot(&wb);
                }
            }
        }
        if t - opt.t_arm >= next_drift {
            rep.drift.push((t - opt.t_arm, horiz, (f.p[(0, 0)] + f.p[(1, 1)]).max(0.0).sqrt()));
            next_drift += 5.0;
        }
    }
    rep.horiz_rms = if n > 0 { (horiz_sq / n as f64).sqrt() } else { f64::NAN };
    rep.nees_avg = if nees_n > 0 { nees_sum / nees_n as f64 } else { f64::NAN };
    for slot in rep.nees_blocks.iter_mut().chain([&mut rep.nees_horiz, &mut rep.nees_vert]) {
        *slot = if nees_n > 0 { *slot / nees_n as f64 } else { f64::NAN };
    }
    rep.status = Some(nav.status());
    rep
}
