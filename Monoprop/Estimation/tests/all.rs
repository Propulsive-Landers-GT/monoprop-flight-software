//! All tests of this crate: `cargo test -p Estimation`. Host only (uses std).
#![allow(non_snake_case)]
mod support;
#[path = "unit/linalg_basics.rs"]
mod t_linalg_basics;
#[path = "unit/linalg_skew.rs"]
mod t_linalg_skew;
#[path = "unit/linalg_blocks.rs"]
mod t_linalg_blocks;
#[path = "unit/linalg_spd.rs"]
mod t_linalg_spd;
#[path = "unit/linalg_cholesky.rs"]
mod t_linalg_cholesky;
#[path = "unit/linalg_solve.rs"]
mod t_linalg_solve;
#[path = "unit/quat_product.rs"]
mod t_quat_product;
#[path = "unit/quat_quat_exp_log.rs"]
mod t_quat_quat_exp_log;
#[path = "unit/quat_so3_exp_log.rs"]
mod t_quat_so3_exp_log;
#[path = "unit/rot_perturbation.rs"]
mod t_rot_perturbation;
#[path = "unit/nominal_strapdown.rs"]
mod t_nominal_strapdown;
#[path = "unit/noise_units.rs"]
mod t_noise_units;
#[path = "unit/nominal_error_state.rs"]
mod t_nominal_error_state;
#[path = "unit/errprop_transition.rs"]
mod t_errprop_transition;
#[path = "unit/errprop_process_noise.rs"]
mod t_errprop_process_noise;
#[path = "unit/esekf_filter.rs"]
mod t_esekf_filter;
#[path = "unit/esekf_models.rs"]
mod t_esekf_models;
#[path = "unit/esekf_update.rs"]
mod t_esekf_update;
#[path = "unit/esekf_sensors.rs"]
mod t_esekf_sensors;
#[path = "unit/align_triad.rs"]
mod t_align_triad;
#[path = "unit/align_start.rs"]
mod t_align_start;
#[path = "unit/scenario_when_sensors_fail.rs"]
mod t_scenario_when_sensors_fail;
#[path = "unit/interop.rs"]
mod t_interop;
#[path = "unit/esekf_height.rs"]
mod t_esekf_height;
#[path = "unit/esekf_lidar.rs"]
mod t_esekf_lidar;
#[path = "unit/esekf_gps_offset.rs"]
mod t_esekf_gps_offset;
#[path = "unit/monoprop_inputs.rs"]
mod t_monoprop_inputs;
#[path = "unit/monoprop_imu_pair.rs"]
mod t_monoprop_imu_pair;
#[path = "unit/system.rs"]
mod t_system;

fn main() {
    support::section("linalg/basics");
    t_linalg_basics::run();
    support::section("linalg/skew");
    t_linalg_skew::run();
    support::section("linalg/blocks");
    t_linalg_blocks::run();
    support::section("linalg/spd");
    t_linalg_spd::run();
    support::section("linalg/cholesky");
    t_linalg_cholesky::run();
    support::section("linalg/solve");
    t_linalg_solve::run();
    support::section("quat/product");
    t_quat_product::run();
    support::section("quat/quat_exp_log");
    t_quat_quat_exp_log::run();
    support::section("quat/so3_exp_log");
    t_quat_so3_exp_log::run();
    support::section("rot/perturbation");
    t_rot_perturbation::run();
    support::section("nominal/strapdown");
    t_nominal_strapdown::run();
    support::section("noise/units");
    t_noise_units::run();
    support::section("nominal/error_state");
    t_nominal_error_state::run();
    support::section("errprop/transition");
    t_errprop_transition::run();
    support::section("errprop/process_noise");
    t_errprop_process_noise::run();
    support::section("esekf/filter");
    t_esekf_filter::run();
    support::section("esekf/models");
    t_esekf_models::run();
    support::section("esekf/update");
    t_esekf_update::run();
    support::section("esekf/sensors");
    t_esekf_sensors::run();
    support::section("align/triad");
    t_align_triad::run();
    support::section("align/start");
    t_align_start::run();
    support::section("scenario/when_sensors_fail");
    t_scenario_when_sensors_fail::run();
    support::section("interop");
    t_interop::run();
    support::section("esekf/height");
    t_esekf_height::run();
    support::section("esekf/lidar");
    t_esekf_lidar::run();
    support::section("esekf/gps_offset");
    t_esekf_gps_offset::run();
    support::section("monoprop/inputs");
    t_monoprop_inputs::run();
    support::section("monoprop/imu_pair");
    t_monoprop_imu_pair::run();
    support::section("system: the whole filter on simulated flights");
    t_system::run();
    support::finish();
}
