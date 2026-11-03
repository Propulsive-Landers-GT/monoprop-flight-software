/// Extends GPT2's free-running 32-bit microsecond count to 64 bits. It must be read at least once per wrap
/// (2^32 us, 71.6 min).
#[derive(Clone, Copy, Debug, Default)]
pub struct WrapExtender {
    last: u32,
    high: u64,
}

impl WrapExtender {
    pub fn extend(&mut self, raw: u32) -> u64 {
        if raw < self.last {
            self.high = self.high.saturating_add(1 << 32);
        }
        self.last = raw;
        self.high.saturating_add(u64::from(raw))
    }
}
