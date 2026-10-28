use crate::{config::Config, protocol::Sample};
use monoprop_common::FlightPhase;
use std::time::{Duration, Instant};

/// Small, testable state machine shared by the operator loop and result checks.
#[derive(Default)]
pub struct Station {
    pub latest: Option<Sample>,
    pub mpc_enabled: bool,
    pub lossless_enabled: bool,
    pub awaiting_phase: bool,
    pub generation: u64,
}

impl Station {
    pub fn invalidate(&mut self) {
        self.generation = self.generation.wrapping_add(1);
    }

    pub fn stop(&mut self) {
        self.mpc_enabled = false;
        self.awaiting_phase = false;
        self.invalidate();
    }

    pub fn reset(&mut self) {
        self.stop();
        self.latest = None;
    }

    pub fn accept(&mut self, sample: Sample) -> bool {
        if let Some(previous) = &self.latest {
            // Duplicates never refresh freshness; an estimator frozen on the
            // Teensy is stale even if the bridge keeps forwarding packets.
            if sample.nav.timestamp_us <= previous.nav.timestamp_us {
                return false;
            }
            if sample.phase() != previous.phase() {
                if previous.phase().is_some_and(|p| p.is_flying())
                    && !sample.phase().is_some_and(|p| p.is_flying())
                {
                    self.mpc_enabled = false;
                }
                self.invalidate();
            }
        }
        self.latest = Some(sample);
        true
    }

    /// Missing navigation must not prevent recovery of an unaligned estimator.
    /// Reject a known fresh non-Standby phase; Teensy always enforces the gate.
    pub fn prepare_ekf_reset(&mut self, now: Instant, config: &Config) -> bool {
        if self.fresh(now, config)
            && self
                .latest
                .as_ref()
                .and_then(Sample::phase)
                .is_some_and(|phase| phase != FlightPhase::Standby)
        {
            return false;
        }
        self.lossless_enabled = false;
        self.reset();
        true
    }

    pub fn fresh(&self, now: Instant, config: &Config) -> bool {
        self.latest.as_ref().is_some_and(|s| {
            now.saturating_duration_since(s.received)
                < Duration::from_millis(config.telemetry_timeout_ms)
        })
    }

    pub fn target(&self, config: &Config) -> [f64; 3] {
        if self.latest.as_ref().and_then(Sample::phase) == Some(FlightPhase::Descent) {
            config.landing_position_m
        } else {
            let mut target = config.hover_position_m;
            if self.latest.as_ref().and_then(Sample::phase) == Some(FlightPhase::Ascent) {
                target[2] += config.ascent_margin_m;
            }
            target
        }
    }

    pub fn can_transmit(
        &self,
        generation: u64,
        source: &Sample,
        now: Instant,
        config: &Config,
    ) -> bool {
        self.mpc_enabled
            && !self.awaiting_phase
            && generation == self.generation
            && self.fresh(now, config)
            && source.phase().is_some_and(|p| p.is_flying())
            && self
                .latest
                .as_ref()
                .is_some_and(|s| s.phase() == source.phase())
            && now.saturating_duration_since(source.received)
                < Duration::from_millis(config.result_max_age_ms)
    }
}
