//! Estimation: state estimation for the monoprop vehicle.
//!
//! `#![no_std]`, no heap, f64, nalgebra fixed-size matrices; builds for the Teensy 4.1
//! (thumbv7em-none-eabihf) and for the host (tests). See README.md for the module map and how to use it.
#![no_std]
#![allow(dead_code, unused_imports, unused_variables, unused_mut, unused_assignments, non_snake_case)]

// ---- prelude ----
pub use core::f64::consts::{FRAC_PI_2, PI, TAU};
pub use nalgebra as na;
pub use nalgebra::{
    ComplexField, Matrix2, Matrix3, Quaternion, RealField, SMatrix, SVector, Unit, UnitQuaternion, Vector2, Vector3,
};
/// f64 3-vector and 3x3 matrix, the workhorses of the ESEKF.
pub type V3 = Vector3<f64>;
pub type M3 = Matrix3<f64>;
pub type Mat<const R: usize, const C: usize> = SMatrix<f64, R, C>;
pub type Vector<const N: usize> = SVector<f64, N>;
/// Attitude quaternion (unit, Hamilton convention). `q * v` rotates a body-frame vector into the inertial frame.
pub type Quat = UnitQuaternion<f64>;

pub mod env;
pub use env::*;
pub mod linalg;
pub use linalg::*;
pub mod quat;
pub use quat::*;
pub mod rot;
pub use rot::*;
pub mod nominal;
pub use nominal::*;
pub mod noise;
pub use noise::*;
pub mod errprop;
pub use errprop::*;
pub mod esekf;
pub use esekf::*;
pub mod align;
pub use align::*;
pub mod interop;
pub use interop::*;
pub mod monoprop; // not re-exported at the root: see monoprop/mod.rs
