//! Bare-metal error-state Kalman filter (ES-EKF): a template for new estimators.
//!
//! Fixed-size nalgebra matrices only: no heap, no `std` (only `core` and nalgebra), so the same code
//! runs on a PC and on a microcontroller (e.g. the Teensy 4.1). f64 throughout.
//!
//! - `math.rs`: linear algebra and rotation helpers.
//! - `model.rs`: the `BareESEKFModel` trait: what a vehicle/sensor model must provide.
//! - `filter.rs`: the generic filter `BareESEKF<Model, E>`: predict, gated update, injection, reset.
//! - `full_state.rs`: the reference model, `FullState` (15-state INS: IMU, position, magnetometer,
//!   height), with alignment (TRIAD) and the VN-200 noise model.
//!
//! A new estimator: write a struct implementing `BareESEKFModel<E>` (copy `full_state.rs` as a
//! starting point), add its measurements as `impl BareESEKF<YourModel, E>` blocks that call
//! `update`, and use `BareESEKF<YourModel, E>`.
//!
//! In this crate the modules sit at the crate root; `interop.rs` adds f32 conversions for the Drone.
#![cfg_attr(not(test), no_std)]
#![allow(non_snake_case)] // the crate is named like the workspace's other members (Common, Drone)

pub mod filter;
pub mod full_state;
pub mod math;
pub mod model;

pub use filter::{BareESEKF, GATE_1DOF, GATE_2DOF, GATE_3DOF, UpdateOutcome};
pub use full_state::{FullState, FullStateESEKF, ImuSample, ImuSpec, Nominal};
pub use math::{M3, Mat, Quat, V3, Vector};
pub use model::BareESEKFModel;

#[cfg(test)]
mod system_test;

pub mod interop;
pub use interop::*;
