//! Stack high-water check, for the first hardware boot (feature `stack-check`, off in flight builds).
//!
//! `paint` fills the unused part of the stack with a pattern. The fill stops `MARGIN_BYTES` below the
//! current stack pointer, so it never touches the frame it runs in, its caller's, or anything an interrupt
//! might push meanwhile. Later, `high_water_bytes` finds the deepest word the stack has overwritten. The
//! result includes everything above the fill's end (the boot frames plus the margin), so it is an upper
//! bound on what was used. NOT verified until it has run on the Teensy.
//!
//! The stack is the linker's `.stack` section (imxrt-rt): `__estack` is its LOW end and `__sstack` its top
//! (the initial stack pointer); it grows down from `__sstack`.
use core::sync::atomic::{AtomicU32, Ordering};

// PLACEHOLDER(choice): room left untouched below the stack pointer while painting.
/// Bytes below the stack pointer that the fill leaves alone.
pub const MARGIN_BYTES: usize = 256;

const PATTERN: u32 = 0xC0FF_EE11;

/// The last measured high-water mark (bytes), readable with a debugger.
pub static STACK_HIGH_WATER_BYTES: AtomicU32 = AtomicU32::new(0);

unsafe extern "C" {
    static __estack: u32;
    static __sstack: u32;
}

/// (low end, top) of the stack region.
fn bounds() -> (usize, usize) {
    (&raw const __estack as usize, &raw const __sstack as usize)
}

/// Fill the unused stack (from its low end up to MARGIN_BYTES below the stack pointer) with the pattern.
pub fn paint() {
    let (low, top) = bounds();
    let sp = cortex_m::register::msp::read() as usize;
    let end = sp.saturating_sub(MARGIN_BYTES).min(top);
    let mut p = low;
    while p + 4 <= end {
        // SAFETY (the only `unsafe` in the Drone's own code): `p` lies in [__estack, sp - MARGIN_BYTES), inside
        // the linker's stack region and below everything the stack holds now, so nothing live is there;
        // the region is 8-byte aligned (imxrt-rt asserts it), so these are aligned u32 writes.
        unsafe { (p as *mut u32).write_volatile(PATTERN) };
        p += 4;
    }
}

/// How deep the stack has reached since `paint` (bytes from the top), also stored in STACK_HIGH_WATER_BYTES.
pub fn high_water_bytes() -> u32 {
    let (low, top) = bounds();
    let mut p = low;
    while p + 4 <= top {
        // SAFETY: inside the linker's stack region, aligned u32 reads of memory we painted.
        let v = unsafe { (p as *const u32).read_volatile() };
        if v != PATTERN {
            break;
        }
        p += 4;
    }
    let used = u32::try_from(top - p).unwrap_or(u32::MAX);
    STACK_HIGH_WATER_BYTES.store(used, Ordering::Relaxed);
    used
}
