//! The Navigator on the Drone: sensor interface, one step per loop, telemetry, the arming gate and the LED.
//! No hardware access, so all of it is testable on the host.
//!
//! NOTE: until sensor drivers exist, every sensor is the `NoSamples` stub, the Navigator never leaves
//! Waiting, and so `go` is always refused. That is intended: the vehicle can't be armed without an estimate.
use Common::DroneTelemetry;
use Estimation::monoprop::{self as mp, GnssSample, ImuSample, LidarSample, MagSample, NavInputs, NavOutput, Navigator, StartupState};

use crate::link::RateLimiter;

// PLACEHOLDER(choice: about 50 Hz, within what the 115200 baud UART link can carry).
/// Time between telemetry packets, us.
pub const TELEMETRY_PERIOD_US: u64 = 20_000;

// PLACEHOLDER(choice): long enough to be seen from the ground station.
/// How long the LED shows the "go refused" pattern after a refusal, us.
pub const REFUSAL_LED_US: u64 = 2_000_000;

// ---- sensors ----

/// A source of one sensor's samples. A driver implements this; `poll` returns a sample only when a new one
/// has arrived since the last call (stamped on the same microsecond clock as `now_us`), else None.
pub trait SampleSource<T> {
    fn poll(&mut self, now_us: u64) -> Option<T>;
}

/// Stand-in for a sensor that has no driver yet: never has a sample.
#[derive(Clone, Copy, Debug, Default)]
pub struct NoSamples;

impl<T> SampleSource<T> for NoSamples {
    fn poll(&mut self, _now_us: u64) -> Option<T> {
        None
    }
}

/// The Monoprop's sensors, one source each.
pub struct Sensors<A, B, M, L, G> {
    pub imu_a: A,
    pub imu_b: B,
    pub mag: M,
    pub lidar: L,
    pub gnss: G,
}

impl Sensors<NoSamples, NoSamples, NoSamples, NoSamples, NoSamples> {
    /// Every sensor a stub (no drivers yet).
    pub fn stubs() -> Self {
        Sensors { imu_a: NoSamples, imu_b: NoSamples, mag: NoSamples, lidar: NoSamples, gnss: NoSamples }
    }
}

impl<A, B, M, L, G> Sensors<A, B, M, L, G>
where
    A: SampleSource<ImuSample>,
    B: SampleSource<ImuSample>,
    M: SampleSource<MagSample>,
    L: SampleSource<LidarSample>,
    G: SampleSource<GnssSample>,
{
    /// This loop's new samples, as the Navigator's input.
    pub fn poll(&mut self, now_us: u64, armed: bool) -> NavInputs {
        NavInputs {
            imu_a: self.imu_a.poll(now_us),
            imu_b: self.imu_b.poll(now_us),
            mag: self.mag.poll(now_us),
            lidar: self.lidar.poll(now_us),
            gnss: self.gnss.poll(now_us),
            armed,
        }
    }
}

// ---- navigator ----

/// Owns the Navigator. Lives in main's stack frame for the whole run (see `size_of` in the tests).
pub struct NavLink {
    nav: Navigator,
}

impl Default for NavLink {
    fn default() -> Self {
        NavLink::new()
    }
}

impl NavLink {
    pub fn new() -> Self {
        NavLink { nav: Navigator::new(mp::navigator_config()) }
    }

    /// Start again from scratch (after a kill): the new Navigator re-aligns on the pad.
    pub fn reset(&mut self) {
        self.nav = Navigator::new(mp::navigator_config());
    }

    pub fn step(&mut self, inputs: &NavInputs) -> NavOutput {
        self.nav.step(inputs)
    }

    pub fn startup(&self) -> StartupState {
        self.nav.status().startup
    }

    pub fn navigator(&self) -> &Navigator {
        &self.nav
    }

    /// The telemetry packet for the current estimate; None until the Navigator has aligned. Sent from
    /// OnPad on, not only in flight: intended, so the ground station sees the aligned state before launch.
    pub fn telemetry(&self) -> Option<DroneTelemetry> {
        telemetry(&self.nav)
    }
}

/// The telemetry packet for the Navigator's current estimate (its timestamp is the estimate's own time on
/// the microsecond clock); None until aligned.
pub fn telemetry(nav: &Navigator) -> Option<DroneTelemetry> {
    let e = nav.estimate()?;
    Some(DroneTelemetry {
        timestamp_us: e.t_us,
        orientation: nav.orientation_xyzw_f32()?, // (qx, qy, qz, qw), the order DroneTelemetry uses
        position: nav.position_f32()?,
        velocity: nav.velocity_f32()?,
        angular_vel: nav.angular_velocity_f32()?,
    })
}

// ---- arming gate and commands ----

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DroneState {
    Idle,
    Flying,
    Landing,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    Go,
    Land,
    Kill,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GoDecision {
    Accept,
    /// The Navigator isn't OnPad (not aligned yet, or already past the pad).
    RefuseNotOnPad,
    /// Already Flying or Landing.
    RefuseAlreadyActive,
}

/// Whether `go` may launch: only from Idle, and only with the Navigator OnPad.
pub fn decide_go(state: DroneState, nav: StartupState) -> GoDecision {
    match (state, nav) {
        (DroneState::Idle, StartupState::OnPad) => GoDecision::Accept,
        (DroneState::Idle, _) => GoDecision::RefuseNotOnPad,
        _ => GoDecision::RefuseAlreadyActive,
    }
}

/// The Navigator's `armed` input: true once launched (Flying or Landing).
pub fn armed(state: DroneState) -> bool {
    matches!(state, DroneState::Flying | DroneState::Landing)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandOutcome {
    Launched,
    Refused(GoDecision),
    Landing,
    Ignored,
    /// The caller must also reset the Navigator (`NavLink::reset`).
    Killed,
}

/// The flight state machine, without the Navigator (it only reads its startup state), so `kill` can never
/// depend on it.
#[derive(Clone, Copy, Debug)]
pub struct FlightControl {
    pub state: DroneState,
    /// `go` commands refused since boot (diagnostic; kept across kills).
    pub refusals: u32,
    /// Telemetry packets dropped because the previous one was still being sent.
    pub telemetry_drops: u32,
    pub telemetry_rate: RateLimiter,
    refused_at: Option<u64>,
}

impl Default for FlightControl {
    fn default() -> Self {
        FlightControl::new()
    }
}

impl FlightControl {
    pub const fn new() -> Self {
        FlightControl {
            state: DroneState::Idle,
            refusals: 0,
            telemetry_drops: 0,
            telemetry_rate: RateLimiter::new(TELEMETRY_PERIOD_US),
            refused_at: None,
        }
    }

    /// Apply a command. A refused `go` changes nothing and is not remembered (reaching OnPad later does not
    /// launch); it is counted and shown on the LED.
    pub fn command(&mut self, cmd: Command, nav: StartupState, now_us: u64) -> CommandOutcome {
        match cmd {
            Command::Go => match decide_go(self.state, nav) {
                GoDecision::Accept => {
                    self.state = DroneState::Flying;
                    CommandOutcome::Launched
                }
                refused => {
                    self.refusals = self.refusals.saturating_add(1);
                    self.refused_at = Some(now_us);
                    CommandOutcome::Refused(refused)
                }
            },
            Command::Land if self.state == DroneState::Flying => {
                self.state = DroneState::Landing;
                CommandOutcome::Landing
            }
            Command::Land => CommandOutcome::Ignored,
            Command::Kill => {
                self.kill();
                CommandOutcome::Killed
            }
        }
    }

    /// Back to Idle from any state; clears the telemetry rate limiter and the refusal LED.
    pub fn kill(&mut self) {
        self.state = DroneState::Idle;
        self.telemetry_rate.reset();
        self.refused_at = None;
    }

    /// Whether the refusal pattern is showing at `now_us`.
    pub fn refusal_showing(&self, now_us: u64) -> bool {
        self.refused_at.is_some_and(|t| now_us.saturating_sub(t) < REFUSAL_LED_US)
    }

    /// The LED at `now_us`: the refusal pattern while it is showing, else the heartbeat.
    pub fn led_on(&self, now_us: u64) -> bool {
        match self.refused_at {
            Some(t) if self.refusal_showing(now_us) => refusal_led(now_us.saturating_sub(t)),
            _ => heartbeat_led(now_us),
        }
    }
}

// ---- LED patterns ----

/// 1 Hz heartbeat: on for the first half of every second.
pub fn heartbeat_led(now_us: u64) -> bool {
    now_us % 1_000_000 < 500_000
}

/// "go refused": three fast blinks (100 ms on, 100 ms off) then a 400 ms pause, every second, `since_us`
/// after the refusal.
pub fn refusal_led(since_us: u64) -> bool {
    let phase = since_us % 1_000_000;
    phase < 600_000 && phase % 200_000 < 100_000
}

/// The stack check's result on the LED (feature `stack-check`): one long blink (400 ms on, 400 ms off) per
/// KiB used, then a 2 s pause, repeated.
pub fn stack_code_led(since_us: u64, kib: u64) -> bool {
    let blinks_us = kib.saturating_mul(800_000);
    let t = since_us % (blinks_us + 2_000_000);
    t < blinks_us && t % 800_000 < 400_000
}

#[cfg(test)]
mod tests {
    use super::*;
    use Estimation::V3;

    #[test]
    fn sizes() {
        let (nav, link) = (core::mem::size_of::<Navigator>(), core::mem::size_of::<NavLink>());
        println!("size_of Navigator = {} bytes, NavLink = {} bytes", nav, link);
        assert!(link < 16 * 1024 / 2, "the Navigator must fit well inside the 16 KiB stack");
    }

    #[test]
    fn go_is_accepted_only_from_idle_on_the_pad() {
        use StartupState::*;
        for nav in [Waiting, OnPad, Flying] {
            for state in [DroneState::Idle, DroneState::Flying, DroneState::Landing] {
                let expected = match (state, nav) {
                    (DroneState::Idle, OnPad) => GoDecision::Accept,
                    (DroneState::Idle, _) => GoDecision::RefuseNotOnPad,
                    _ => GoDecision::RefuseAlreadyActive,
                };
                assert_eq!(decide_go(state, nav), expected, "{:?} / {:?}", state, nav);
            }
        }
        assert!(!armed(DroneState::Idle) && armed(DroneState::Flying) && armed(DroneState::Landing));
    }

    #[test]
    fn kill_works_from_every_state_without_the_navigator() {
        for start in [DroneState::Idle, DroneState::Flying, DroneState::Landing] {
            let mut fc = FlightControl::new();
            fc.state = start;
            assert_eq!(fc.command(Command::Kill, StartupState::Flying, 0), CommandOutcome::Killed);
            assert_eq!(fc.state, DroneState::Idle, "from {:?}", start);
        }
    }

    #[test]
    fn kill_then_go_is_refused_by_the_fresh_navigator() {
        let mut fc = FlightControl::new();
        let mut link = NavLink::new();
        // pretend we had launched
        fc.state = DroneState::Flying;
        if fc.command(Command::Kill, link.startup(), 10) == CommandOutcome::Killed {
            link.reset();
        }
        assert_eq!(link.startup(), StartupState::Waiting, "the fresh navigator has to re-align");
        let out = fc.command(Command::Go, link.startup(), 20);
        assert_eq!(out, CommandOutcome::Refused(GoDecision::RefuseNotOnPad));
        assert_eq!(fc.state, DroneState::Idle);
    }

    #[test]
    fn a_refused_go_is_not_remembered() {
        let mut fc = FlightControl::new();
        assert_eq!(fc.command(Command::Go, StartupState::Waiting, 0), CommandOutcome::Refused(GoDecision::RefuseNotOnPad));
        // the navigator reaches OnPad later: nothing happens without a new go
        assert_eq!(fc.state, DroneState::Idle);
        assert!(!armed(fc.state));
        assert_eq!(fc.refusals, 1);
        assert_eq!(fc.command(Command::Go, StartupState::OnPad, 5_000_000), CommandOutcome::Launched, "a new go launches");
    }

    #[test]
    fn kill_clears_the_rate_limiter_and_the_refusal() {
        let mut fc = FlightControl::new();
        assert!(fc.telemetry_rate.due(0));
        fc.command(Command::Go, StartupState::Waiting, 100);
        assert!(fc.refusal_showing(200));
        fc.command(Command::Kill, StartupState::Waiting, 300);
        assert!(!fc.refusal_showing(400), "refusal cleared");
        assert!(fc.telemetry_rate.due(400), "rate limiter cleared: due straight away");
        assert_eq!(fc.refusals, 1, "the diagnostic count is kept");
    }

    #[test]
    fn led_patterns() {
        assert!(heartbeat_led(0) && heartbeat_led(499_999) && !heartbeat_led(500_000) && heartbeat_led(1_000_000));
        let on: Vec<u64> = (0..10).map(|i| i * 100_000).filter(|&t| refusal_led(t)).collect();
        assert_eq!(on, [0, 200_000, 400_000], "three blinks, then the pause");
        assert!(!refusal_led(650_000) && !refusal_led(950_000), "pause");
        assert!(refusal_led(1_000_000), "repeats");
        let mut fc = FlightControl::new();
        fc.command(Command::Go, StartupState::Waiting, 1_000_000);
        assert_eq!(fc.led_on(1_100_000), refusal_led(100_000), "refusal pattern while showing");
        assert_eq!(fc.led_on(1_000_000 + REFUSAL_LED_US + 10), heartbeat_led(1_000_000 + REFUSAL_LED_US + 10), "then the heartbeat");
    }

    #[test]
    fn stack_code_blinks_once_per_kib() {
        let on = |kib| (0..40).filter(|i| stack_code_led(i * 100_000 + 50_000, kib)).count();
        assert_eq!(on(3), 3 * 4, "3 blinks of 400 ms in the first 4 s");
        assert!(!stack_code_led(2_400_000 + 100_000, 3), "pause after the blinks");
        assert!(stack_code_led(4_400_000 + 100_000, 3), "repeats");
    }

    #[test]
    fn stubs_never_give_a_sample_and_go_stays_refused() {
        let mut s = Sensors::stubs();
        let mut link = NavLink::new();
        for k in 0..2000u64 {
            let i = s.poll(k * 1000, false);
            assert!(i.imu_a.is_none() && i.imu_b.is_none() && i.mag.is_none() && i.lidar.is_none() && i.gnss.is_none());
            link.step(&i);
        }
        assert_eq!(link.startup(), StartupState::Waiting);
        assert!(link.telemetry().is_none());
        assert_eq!(decide_go(DroneState::Idle, link.startup()), GoDecision::RefuseNotOnPad);
    }

    /// A still, level pad: the same IMU reading from both IMUs, the reference field on the magnetometer.
    struct StillImu {
        period_us: u64,
        next_us: u64,
    }
    impl SampleSource<ImuSample> for StillImu {
        fn poll(&mut self, now_us: u64) -> Option<ImuSample> {
            if now_us < self.next_us {
                return None;
            }
            self.next_us = now_us + self.period_us;
            ImuSample::new(now_us, V3::new(0.0, 0.0, Estimation::GRAVITY), V3::zeros())
        }
    }
    struct StillMag;
    impl SampleSource<MagSample> for StillMag {
        fn poll(&mut self, now_us: u64) -> Option<MagSample> {
            MagSample::new(now_us, Estimation::mag_reference() * 50.0)
        }
    }

    #[test]
    fn telemetry_after_alignment_matches_the_navigator() {
        let mut s = Sensors {
            imu_a: StillImu { period_us: 2500, next_us: 0 },
            imu_b: StillImu { period_us: 2500, next_us: 0 },
            mag: StillMag,
            lidar: NoSamples,
            gnss: NoSamples,
        };
        let mut link = NavLink::new();
        assert!(link.telemetry().is_none(), "None before alignment");
        let mut now = 1_000_000;
        while link.startup() == StartupState::Waiting && now < 5_000_000 {
            let i = s.poll(now, false);
            link.step(&i);
            now += 2500;
        }
        assert_eq!(link.startup(), StartupState::OnPad, "aligned from the test doubles");
        let t = link.telemetry();
        assert!(t.is_some());
        let Some(t) = t else { return };
        let n = link.navigator();
        assert_eq!(Some(t.position), n.position_f32());
        assert_eq!(Some(t.velocity), n.velocity_f32());
        assert_eq!(Some(t.angular_vel), n.angular_velocity_f32());
        let q = n.estimate().map(|e| e.attitude.quaternion().coords).unwrap_or_default();
        assert_eq!(t.orientation, [q.x as f32, q.y as f32, q.z as f32, q.w as f32], "x, y, z, w");
        assert_eq!(Some(t.timestamp_us), n.estimate().map(|e| e.t_us));
    }
}
