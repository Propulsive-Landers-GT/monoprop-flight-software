use ndarray::{Array1, Array2};
use nalgebra::{Vector3, UnitQuaternion, Quaternion};
use rust_ekf::es_ekf::filter::ErrorStateKalmanFilter;
use rust_ekf::models::full_state_esekf::RocketState;
use crate::state::{SensorData, VehicleState};

pub struct Navigator {
    pub ekf_es: ErrorStateKalmanFilter<RocketState>,
    last_update: Option<f64>,
    last_uwb: Option<f64>,
}

impl Navigator {
    pub fn new() -> Self {
        // Nominal state: [px, py, pz, vx, vy, vz, qw, qx, qy, qz, abx, aby, abz, wbx, wby, wbz] (16D)
        let initial_nominal = Array1::from(vec![
            0.0, 0.0, 0.0, // pos
            0.0, 0.0, 0.0, // vel
            1.0, 0.0, 0.0, 0.0, // quat (w, x, y, z)
            0.0, 0.0, 0.0, // ab
            0.0, 0.0, 0.0, // wb
        ]);
        
        let initial_p = RocketState::initial_covariance();
        
        let mut q = Array2::eye(15);
        for i in 0..3 { q[(i, i)] = 0.01; } // pos
        for i in 3..6 { q[(i, i)] = 0.05; } // vel
        for i in 6..9 { q[(i, i)] = 0.01; } // att
        for i in 9..12 { q[(i, i)] = 0.001; } // ab
        for i in 12..15 { q[(i, i)] = 0.001; } // wb
        q = q * 0.1;
        
        let ekf_es = ErrorStateKalmanFilter::new(
            initial_nominal,
            initial_p,
            q,
            RocketState,
        );

        Self {
            ekf_es,
            last_update: None,
            last_uwb: None,
        }
    }
    
    pub fn update(&mut self, sensor_data: &SensorData, dt: f64, _in_prelaunch: bool) -> Option<VehicleState> {
        let now = sensor_data.timestamp;
        if let Some(last) = self.last_update {
            if now - last < dt - 1e-6 {
                return None;
            }
        }
        self.last_update = Some(now);

        let gyro = sensor_data.imu_data.as_ref().map(|d| d.gyro).unwrap_or([0.0, 0.0, 0.0]);
        let accel = sensor_data.imu_data.as_ref().map(|d| d.accel).unwrap_or([0.0, 0.0, 9.81]);
        
        // 1. Prediction step using high-frequency IMU
        let imu_input = [accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]];
        self.ekf_es.predict(&imu_input, dt);

        // 2. Correction step if GPS/UWB is available
        // A held sample is not an independent observation. Unknown timestamps fail closed.
        let fresh = sensor_data.uwb_acquired_at.filter(|t|
            t.is_finite() && now.is_finite() && *t <= now && now - *t <= 0.25
            && self.last_uwb.map_or(true, |last| *t > last));
        let uwb = if fresh.is_some() { sensor_data.uwb_data.as_ref() } else { None };
        if uwb.is_some() { self.last_uwb = fresh; }
        let (pos, pos_noise) = match (uwb, &sensor_data.gps_data) {
            (Some(uwb), _) => {
                (Some([uwb[0], uwb[1], uwb[2]]), 0.01)
            }
            (None, Some(gps)) => {
                (Some([gps[0], gps[1], gps[2]]), 0.2)
            }
            (None, None) => {
                (None, 1e10)
            }
        };

        if let Some(pos_val) = pos {
            let measurement = Array1::from(vec![pos_val[0], pos_val[1], pos_val[2]]);
            let r_matrix = Array2::eye(3) * pos_noise;
            self.ekf_es.update(&measurement, &r_matrix);
        }

        // 3. Magnetometer update step to resolve yaw/attitude observability
        // Skipped for IMUs without a magnetometer.
        if let Some(mag) = sensor_data.imu_data.as_ref().and_then(|imu| imu.mag) {
            let measurement = Array1::from(vec![mag[0], mag[1], mag[2]]);
            let mag_world = RocketState::mag_world();
            let prediction = RocketState::mag_prediction(&self.ekf_es.nominal_state, &mag_world);
            let h = RocketState::mag_jacobian(&self.ekf_es.nominal_state, &mag_world);
            let r_mag = Array2::eye(3) * (14.0e-9_f64).powi(2);
            self.ekf_es.update_with(&measurement, &prediction, &h, &r_mag);
        }

        // 3. Extract and construct VehicleState
        let state_vec = &self.ekf_es.nominal_state;
        
        let position = Vector3::new(state_vec[0], state_vec[1], state_vec[2]);
        let velocity = Vector3::new(state_vec[3], state_vec[4], state_vec[5]);
        let attitude = UnitQuaternion::new_normalize(Quaternion::new(state_vec[6], state_vec[7], state_vec[8], state_vec[9]));
        let wb = [state_vec[13], state_vec[14], state_vec[15]];
        let angular_velocity = Vector3::new(gyro[0] - wb[0], gyro[1] - wb[1], gyro[2] - wb[2]);

        Some(VehicleState {
            position,
            velocity,
            attitude,
            angular_velocity,
            mass: 80.0,
            dry_mass: 50.0,
        })
    }

    fn inject_state(&mut self, initial_nominal: Array1<f64>, initial_p: Array2<f64>, q: Array2<f64>) {
        // 🚀 MAP TO THE CORRECT STRUCT FIELDS 🚀
        self.ekf_es.nominal_state = initial_nominal;
        self.ekf_es.error_covariance = initial_p;
        self.ekf_es.process_noise = q;
        
        // Reset the update timer so it processes the first IMU tick correctly
        self.last_update = None;
    }
}

impl super::Navigator for Navigator {
    fn update(&mut self, sensor_data: &SensorData, dt: f64, in_prelaunch: bool) -> Option<VehicleState> {
        self.update(sensor_data, dt, in_prelaunch)
    }

    fn get_state_vector(&self) -> Option<Array1<f64>> {
        Some(self.ekf_es.nominal_state.clone())
    }
    fn inject_state(&mut self, initial_nominal: Array1<f64>, initial_p: Array2<f64>, q: Array2<f64>) {
        // Dynamic dispatch must initialize the same filter as the concrete API.
        Navigator::inject_state(self, initial_nominal, initial_p, q);
    }
}

#[cfg(test)]
mod freshness_tests {
 use super::*;
 #[test]
 fn stale_fresh_hold_dropout_and_phase_transition() {
  let mut n = Navigator::new();
  let mut d = SensorData { timestamp: 24.0, imu_data: None, gps_data: Some([0.0;3]), uwb_data: Some([5.0;3]), uwb_acquired_at: Some(3.0), chamber_pressure: None, tank_pressure: None };
  n.update(&d,0.01,false); assert_eq!(n.last_uwb,None);
  d.timestamp=24.01; d.uwb_acquired_at=Some(24.0);
  n.update(&d,0.01,false); assert_eq!(n.last_uwb,Some(24.0));
  d.timestamp=24.02; n.update(&d,0.01,true); assert_eq!(n.last_uwb,Some(24.0));
  d.timestamp=24.03; d.uwb_data=None; n.update(&d,0.01,false);
  d.timestamp=24.04; d.uwb_data=Some([0.0;3]); d.uwb_acquired_at=Some(24.04);
  n.update(&d,0.01,false); assert_eq!(n.last_uwb,Some(24.04));
 }
}

#[cfg(test)]
mod fallback_tests {
 use super::*;
 #[test]
 fn stale_uwb_is_exactly_gps_only() {
  let mut a=Navigator::new(); let mut b=Navigator::new();
  let mut d=SensorData { timestamp:24.0, imu_data:None, gps_data:Some([1.0,2.0,3.0]), uwb_data:Some([8.0;3]), uwb_acquired_at:Some(3.0), chamber_pressure:None,tank_pressure:None };
  a.update(&d,0.01,false); d.uwb_data=None; b.update(&d,0.01,false);
  assert_eq!(a.ekf_es.nominal_state,b.ekf_es.nominal_state);
  assert_eq!(a.ekf_es.error_covariance,b.ekf_es.error_covariance);
 }
}

#[cfg(test)]
mod held_tests {
 use super::*;
 #[test]
 fn held_sample_does_not_fuse_again_and_unknown_future_fall_back() {
  for stamp in [Some(1.0),None,Some(10.0)] {
   let mut a=Navigator::new();let mut b=Navigator::new();
   let mut d=SensorData {timestamp:1.0,imu_data:None,gps_data:Some([0.0;3]),uwb_data:Some([1.0;3]),uwb_acquired_at:Some(1.0),chamber_pressure:None,tank_pressure:None};
   a.update(&d,0.01,false);b.update(&d,0.01,false);
   d.timestamp=1.01;d.uwb_acquired_at=stamp;
   a.update(&d,0.01,true);d.uwb_data=None;b.update(&d,0.01,true);
   assert_eq!(a.ekf_es.nominal_state,b.ekf_es.nominal_state);
   assert_eq!(a.ekf_es.error_covariance,b.ekf_es.error_covariance);
  }
 }
}

#[cfg(test)]
mod injection_tests {
    use super::*;

    #[test]
    fn trait_object_injection_updates_filter_and_resets_clock() {
        let mut navigator = Navigator::new();
        navigator.last_update = Some(100.0);
        let nominal = Array1::from(vec![
            -3.5, 2.0, 50.0, 0.2, -0.1, -2.0,
            0.6, 0.8, 0.0, 0.0, // Unit quaternion, stored w first.
            0.01, -0.02, 0.03, 0.001, -0.002, 0.003,
        ]);
        let covariance = Array2::eye(15) * 0.002;
        let process_noise = Array2::eye(15) * 0.00003;
        {
            // The simulator receives this trait object from navigator_mut().
            let erased: &mut dyn crate::algorithms::Navigator = &mut navigator;
            erased.inject_state(nominal.clone(), covariance.clone(), process_noise.clone());
            assert_eq!(erased.get_state_vector().unwrap(), nominal);
        }
        assert_eq!(navigator.ekf_es.error_covariance, covariance);
        assert_eq!(navigator.ekf_es.process_noise, process_noise);
        assert_eq!(navigator.last_update, None);
    }

    #[test]
    fn autopilot_navigator_injection_reaches_concrete_filter() {
        let mut autopilot = crate::fsm::Autopilot::new();
        let mut nominal = Array1::zeros(16);
        nominal[2] = 37.0;
        nominal[6] = 1.0;
        let navigator = autopilot.navigator_mut();
        navigator.inject_state(nominal.clone(), Array2::eye(15), Array2::eye(15) * 1e-5);
        assert_eq!(navigator.get_state_vector().unwrap(), nominal);
    }
}
