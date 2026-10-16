//! Drone flight software: the parts that don't touch the hardware, so they can be tested on a laptop
//! (`cargo test -p Drone --lib --no-default-features`, run from Monoprop/). The firmware binary is
//! src/main.rs.
//!
//! - `link`: the microsecond clock's wraparound, the telemetry rate limiter and the non-blocking UART queue
//! - `nav`: the sensor interface, the Navigator, telemetry, the arming gate and the LED patterns
//! - `stack` (feature `stack-check`, Teensy only): the stack high-water check for the first hardware boot
#![cfg_attr(not(test), no_std)]
#![allow(non_snake_case)] // the crate is named Drone

pub mod link;
pub mod nav;
#[cfg(all(feature = "stack-check", target_arch = "arm"))]
pub mod stack;
#[cfg(test)]
mod stack_budget;
