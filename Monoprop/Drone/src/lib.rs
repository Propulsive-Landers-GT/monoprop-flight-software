//! Monoprop drone firmware, the parts that don't touch the hardware (host-testable). src/main.rs is the
//! Teensy binary.
#![cfg_attr(not(test), no_std)]
#![allow(non_snake_case)] // the crate is named Drone

pub mod clock;
pub mod config;
pub mod imu_pair;
pub mod navigator;
pub mod sensors;
