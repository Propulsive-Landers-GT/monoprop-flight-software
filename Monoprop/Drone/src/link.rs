//! Timing and the telemetry link: no hardware access, so testable on the host.
use Common::DroneTelemetry;

/// Extends a free-running 32-bit microsecond counter (GPT2 at 1 MHz) to 64 bits.
///
/// Each time the raw count is lower than the previous one, the counter has wrapped and 2^32 us is added.
/// Correct as long as it is read at least once per wrap (2^32 us = 71.6 min); the main loop reads it every
/// millisecond.
#[derive(Clone, Copy, Debug, Default)]
pub struct WrapExtender {
    last: Option<u32>,
    high: u64,
}

impl WrapExtender {
    pub const fn new() -> Self {
        WrapExtender { last: None, high: 0 }
    }

    /// The 64-bit time (us) for a new raw counter reading.
    pub fn extend(&mut self, raw: u32) -> u64 {
        if let Some(last) = self.last {
            if raw < last {
                self.high = self.high.saturating_add(1 << 32);
            }
        }
        self.last = Some(raw);
        self.high.saturating_add(u64::from(raw))
    }
}

/// Decides when the next telemetry packet is due, by timestamp rather than by loop count.
#[derive(Clone, Copy, Debug)]
pub struct RateLimiter {
    period_us: u64,
    last: Option<u64>,
}

impl RateLimiter {
    pub const fn new(period_us: u64) -> Self {
        RateLimiter { period_us, last: None }
    }

    /// True (and remembers `now_us`) if no packet was sent yet, or at least one period has passed since the
    /// last one. A clock that goes backwards just delays the next packet.
    pub fn due(&mut self, now_us: u64) -> bool {
        let due = match self.last {
            None => true,
            Some(last) => now_us.saturating_sub(last) >= self.period_us,
        };
        if due {
            self.last = Some(now_us);
        }
        due
    }

    /// Forget the last packet: the next call is due.
    pub fn reset(&mut self) {
        self.last = None;
    }
}

/// Bytes the transmit queue holds: one encoded `DroneTelemetry` (at most 62 bytes, see the tests).
pub const TX_CAPACITY: usize = 64;

/// A one-packet transmit queue, emptied a few bytes at a time without ever waiting for the UART.
#[derive(Clone, Copy, Debug)]
pub struct TxQueue {
    buf: [u8; TX_CAPACITY],
    len: usize,
    pos: usize,
}

impl Default for TxQueue {
    fn default() -> Self {
        TxQueue::new()
    }
}

impl TxQueue {
    pub const fn new() -> Self {
        TxQueue { buf: [0; TX_CAPACITY], len: 0, pos: 0 }
    }

    /// True while a packet is still being sent.
    pub fn is_busy(&self) -> bool {
        self.pos < self.len
    }

    /// Encode a telemetry packet into the queue. False, changing nothing, if a packet is still being sent
    /// or it doesn't encode into the buffer.
    pub fn load(&mut self, t: &DroneTelemetry) -> bool {
        if self.is_busy() {
            return false;
        }
        match postcard::to_slice(t, &mut self.buf) {
            Ok(used) => {
                self.len = used.len();
                self.pos = 0;
                true
            }
            Err(_) => false,
        }
    }

    /// Hand bytes to `try_write` until it reports the UART full or the packet is done.
    pub fn pump(&mut self, mut try_write: impl FnMut(u8) -> bool) {
        while let Some(&b) = self.buf.get(self.pos).filter(|_| self.pos < self.len) {
            if !try_write(b) {
                return;
            }
            self.pos += 1;
        }
        self.clear();
    }

    /// Drop whatever is queued.
    pub fn clear(&mut self) {
        self.len = 0;
        self.pos = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap_extender_is_monotonic_across_wraps() {
        let mut w = WrapExtender::new();
        assert_eq!(w.extend(10), 10);
        assert_eq!(w.extend(10), 10, "equal reads");
        assert_eq!(w.extend(u32::MAX), u64::from(u32::MAX));
        assert_eq!(w.extend(5), (1u64 << 32) + 5, "one wrap");
        assert_eq!(w.extend(4), (2u64 << 32) + 4, "a second wrap");
        assert_eq!(w.extend(100), (2u64 << 32) + 100);
    }

    #[test]
    fn rate_limiter_by_timestamp() {
        let mut r = RateLimiter::new(20_000);
        assert!(r.due(1_000), "first call");
        assert!(!r.due(1_000) && !r.due(20_999), "not before 20 ms");
        assert!(r.due(21_000), "exactly one period later");
        assert!(r.due(5_000_000), "after a long pause");
        assert!(!r.due(4_000_000), "clock going backwards: not due");
        assert!(r.due(u64::MAX), "no overflow at the end of time");
        r.reset();
        assert!(r.due(0), "due again after reset");
    }

    fn sample(t: u64) -> DroneTelemetry {
        DroneTelemetry {
            timestamp_us: t,
            orientation: [0.1, -0.2, 0.3, 0.9],
            position: [1.5, -2.5, 3.5],
            velocity: [f32::MAX, f32::MIN, -0.0],
            angular_vel: [1e-30, 1e30, f32::NAN],
        }
    }

    #[test]
    fn worst_case_packet_fits_the_queue() {
        let mut buf = [0u8; 128];
        let used = postcard::to_slice(&sample(u64::MAX), &mut buf).map(|s| s.len()).unwrap_or(usize::MAX);
        println!("worst-case DroneTelemetry: {} bytes (queue holds {})", used, TX_CAPACITY);
        assert!(used <= TX_CAPACITY, "{} bytes", used);
        assert_eq!(used, 10 + 13 * 4, "10-byte varint + 13 f32");
        let mut q = TxQueue::new();
        assert!(q.load(&sample(u64::MAX)));
    }

    #[test]
    fn tx_queue_sends_in_pieces_and_never_waits() {
        let mut q = TxQueue::new();
        assert!(q.load(&sample(7)));
        assert!(!q.load(&sample(8)), "a second packet while busy is refused");
        let mut sent = Vec::new();
        let mut budget = 5;
        q.pump(|b| {
            if budget == 0 {
                return false;
            }
            budget -= 1;
            sent.push(b);
            true
        });
        assert_eq!(sent.len(), 5, "stopped when the UART was full");
        assert!(q.is_busy());
        q.pump(|b| {
            sent.push(b);
            true
        });
        assert!(!q.is_busy(), "empty after the rest was sent");
        let mut expected = [0u8; 64];
        let enc = postcard::to_slice(&sample(7), &mut expected).map(|s| s.to_vec()).unwrap_or_default();
        assert_eq!(sent, enc, "bytes in order, nothing lost or repeated");
        assert!(q.load(&sample(9)), "free again");
    }
}
