use crate::{config::Model, mpc_crate};
use anyhow::{Result, anyhow, ensure};
use monoprop_common::{ControlCommand, FlightPhase, MPC_HORIZON_STEPS, TelemetryFrame};
use ndarray::{Array1, Array2, Axis};
use optimization_engine::panoc::PANOCCache;

pub struct Controller {
    model: Model,
    warm: Vec<Array1<f64>>,
    cache: PANOCCache,
    reference: Array1<f64>,
    q: Array2<f64>,
    qn: Array2<f64>,
    descent: Option<(u64, crate::trajectory::ReturnReference)>,
}
impl Controller {
    pub fn new(model: Model) -> Self {
        let warm = vec![Array1::from_vec(vec![0.0, 0.0, model.mass_kg * 9.81]); MPC_HORIZON_STEPS];
        let mut reference = Array1::zeros(13);
        reference[2] = model.hover_height_m;
        reference[6] = 1.0;
        Self {
            model,
            descent: None,
            warm,
            reference,
            cache: PANOCCache::new(3 * MPC_HORIZON_STEPS, 1e-4, 20),
            q: Array2::from_diag(&Array1::from_vec(vec![
                20., 20., 200., 10., 10., 0., 10., 3., 3., 1., 1., 1., 1.,
            ])),
            qn: Array2::from_diag(&Array1::from_vec(vec![
                200., 200., 5000., 10., 10., 0., 10., 10., 10., 10., 5., 5., 5.,
            ])),
        }
    }
    pub fn solve(&mut self, t: &TelemetryFrame, sequence: u32) -> Result<ControlCommand> {
        let refs = if t.phase == FlightPhase::Descent {
            let stamp = t.sensor_data.timestamp_us;
            if self
                .descent
                .as_ref()
                .is_none_or(|(start, _)| stamp < *start)
            {
                self.descent = Some((stamp, crate::trajectory::ReturnReference::new(t)));
            }
            let (start, trajectory) = self.descent.as_ref().unwrap();
            trajectory.horizon((stamp - start) as f64 * 1e-6)
        } else {
            self.descent = None;
            vec![self.reference.clone(); MPC_HORIZON_STEPS + 1]
        };
        self.solve_tracking(t, sequence, &refs)
    }

    /// Track a supplied navigation-frame trajectory using the same production MPC.
    pub fn solve_tracking(
        &mut self,
        t: &TelemetryFrame,
        sequence: u32,
        refs: &Vec<Array1<f64>>,
    ) -> Result<ControlCommand> {
        ensure!(
            refs.len() == MPC_HORIZON_STEPS + 1
                && refs
                    .iter()
                    .all(|r| r.len() == 13 && r.iter().all(|v| v.is_finite())),
            "invalid reference trajectory"
        );
        let v = &t.vehicle_state;
        ensure!(
            (v.mass_kg as f64 - self.model.mass_kg).abs() < 0.01,
            "ground/Teensy mass mismatch"
        );
        let legacy = t.as_legacy();
        let mut x = Array1::zeros(13);
        for i in 0..3 {
            x[i] = legacy.position[i] as f64;
            x[7 + i] = legacy.velocity[i] as f64;
            x[10 + i] = legacy.angular_vel[i] as f64;
        }
        for i in 0..4 {
            x[3 + i] = legacy.orientation[i] as f64;
        }
        ensure!(
            x.iter().all(|v| v.is_finite()),
            "nonfinite navigation state"
        );
        let norm = (3..7).map(|i| x[i] * x[i]).sum::<f64>().sqrt();
        ensure!((norm - 1.0).abs() < 0.05, "invalid quaternion");
        // q and -q are the same rotation; keep the cost on the identity hemisphere.
        let sign = if x[6] < 0.0 { -1.0 } else { 1.0 };
        for i in 3..7 {
            x[i] *= sign / norm;
        }
        let r = Array2::zeros((3, 3));
        let smoothing = Array1::from_vec(vec![1500., 1500., 0.02]);
        let (first, horizon) = mpc_crate::OpEnSolve(
            &self.model,
            &x,
            &self.warm,
            refs,
            &self.q,
            &r,
            &self.qn,
            &smoothing,
            &mut self.cache,
        )
        .map_err(|e| anyhow!(e))?;
        self.warm = horizon
            .axis_iter(Axis(0))
            .skip(1)
            .map(|row| row.to_owned())
            .collect();
        self.warm
            .push(horizon.row(MPC_HORIZON_STEPS - 1).to_owned());
        Ok(ControlCommand {
            sequence,
            target_phase: t.phase,
            actuator_command: monoprop_common::ActuatorCommand {
                thrust_norm: self.model.throttle(first[2])?,
                gimbal_rad: [
                    (first[0] * self.model.gimbal_sign[0]) as f32,
                    (first[1] * self.model.gimbal_sign[1]) as f32,
                ],
                body_rate_cmd_rps: [0.0; 3],
            },
        })
    }
}
