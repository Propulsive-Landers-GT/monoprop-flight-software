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
