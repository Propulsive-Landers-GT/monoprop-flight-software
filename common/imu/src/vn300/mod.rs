//! VectorNav VN-300 driver.
//!
//! The VN-300 is the VN-200 with a second GNSS antenna for heading. Its IMU (accelerometer,
//! gyroscope, magnetometer, barometer, temperature) and its binary output protocol are the same,
//! so this driver also reads a VN-200 or VN-100 configured with the same binary output.
//!
//! # What the driver does
//!
//! - Reads whatever bytes the serial port has, without blocking, through [`ByteSource`].
//! - Finds binary packets, checks their CRC, and skips anything else: ASCII output, noise, partial
//!   packets (kept until the rest arrives).
//! - Converts the newest packet into an [`ImuSample`]: SI units, rotated into the vehicle body frame
//!   by the [`Mounting`] it was given.
//!
//! # What vehicle firmware supplies
//!
//! - A [`ByteSource`] for the serial port (Teensy UART, Linux serial device, ...).
//! - The sensor configuration at start-up, using [`config`]: binary output register, baud rate,
//!   ASCII output off. Without it the sensor sends ASCII at 115200 baud and the driver returns no samples.
//! - The [`Mounting`] for how the unit is installed on that vehicle.
//!
//! Without hardware, [`emulator`] turns simulated readings into VN-300 packets for this driver.
//!
//! # Example
//!
//! ```
//! use imu::{Imu, Mounting};
//! use imu::vn300::{ByteSource, Vn300};
//!
//! struct Port; // firmware's serial port
//! impl ByteSource for Port {
//!     type Error = ();
//!     fn read_available(&mut self, _buf: &mut [u8]) -> Result<usize, ()> { Ok(0) }
//! }
//!
//! let mut vn300 = Vn300::new(Port, Mounting::Z_DOWN_TO_Z_UP);
//! // In the control loop: None until a packet has arrived.
//! assert_eq!(vn300.read(0.0), Ok(None));
//! ```

pub mod config;
pub mod emulator;
pub mod protocol;

use crate::units::{deg_to_rad, gauss_to_tesla, g_to_mps2, kpa_to_pa};
use crate::{Capabilities, DeltaIntegrals, Imu, ImuInfo, ImuSample, ImuStatus, Mounting, NoiseSpec};
use protocol::{Frame, FrameError, Packet, SYNC};

/// Non-blocking byte input, implemented by vehicle firmware for its serial port.
pub trait ByteSource {
    type Error;

    /// Copy bytes that have already arrived into `buf` and return how many.
    /// Must not wait: return `Ok(0)` when nothing is waiting.
    fn read_available(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error>;
}

/// Receive buffer size: more than one maximum-size packet, so one always fits.
const RX_LEN: usize = 512;
const _: () = assert!(RX_LEN > protocol::MAX_PACKET_LEN);

/// Counters for link health. They only increase.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Stats {
    /// Valid packets received.
    pub packets: u32,
    /// Valid packets dropped because a newer one arrived in the same `read`.
    pub superseded: u32,
    /// Packets that failed the CRC check (or stray 0xFA bytes).
    pub crc_errors: u32,
    /// Packets with groups or fields this driver does not decode. Usually a configuration mismatch.
    pub unsupported: u32,
    /// Valid packets with no accelerometer or no gyroscope data, so no sample could be made.
    pub incomplete: u32,
    /// Bytes skipped while looking for a packet start (ASCII output, noise).
    pub bytes_skipped: u32,
}

/// VN-300 on a serial port. See the module docs.
pub struct Vn300<S> {
    source: S,
    mounting: Mounting,
    info: ImuInfo,
    rx: [u8; RX_LEN],
    rx_len: usize,
    next_seq: u32,
    last_packet: Option<Packet>,
    stats: Stats,
}

impl<S: ByteSource> Vn300<S> {
    /// Datasheet figures for the VN-300's IMU, the same IMU as the VN-200. Rates and ranges are the
    /// factory defaults; confirm them against the datasheet revision for the unit being flown.
    pub const INFO: ImuInfo = ImuInfo {
        model: "VectorNav VN-300",
        capabilities: Capabilities {
            sensor_time: true,
            mag: true,
            temperature: true,
            pressure: true,
            delta: true,
            // The onboard quaternion is decoded (see `last_packet`) but not converted into the
            // body-to-world convention of `ImuSample::attitude` yet: that needs the mounting and
            // the NED-to-world rotation checked on hardware.
            attitude: false,
        },
        max_rate_hz: 400.0,
        accel_range_mps2: g_to_mps2(16.0),
        gyro_range_radps: deg_to_rad(2000.0),
        noise: NoiseSpec {
            accel_noise_density: g_to_mps2(0.14e-3),
            gyro_noise_density: deg_to_rad(0.0035),
            accel_bias_instability: g_to_mps2(0.04e-3),
            gyro_bias_instability: deg_to_rad(10.0) / 3600.0,
            // 140 uGauss/sqrt(Hz) noise density, i.e. 1 sigma at 1 Hz bandwidth.
            mag_noise: Some(gauss_to_tesla(140.0e-6)),
        },
    };

    pub fn new(source: S, mounting: Mounting) -> Self {
        Self {
            source,
            mounting,
            info: Self::INFO,
            rx: [0; RX_LEN],
            rx_len: 0,
            next_seq: 0,
            last_packet: None,
            stats: Stats::default(),
        }
    }

    pub fn stats(&self) -> Stats {
        self.stats
    }

    /// The newest valid packet in the sensor's own units and axes, including fields that
    /// `ImuSample` does not carry (onboard attitude, INS status, GPS time).
    pub fn last_packet(&self) -> Option<&Packet> {
        self.last_packet.as_ref()
    }

    /// The serial port, for sending [`config`] commands.
    pub fn source_mut(&mut self) -> &mut S {
        &mut self.source
    }

    /// Scan the receive buffer for packets. Returns the newest valid one and its sequence number.
    fn parse_buffered(&mut self) -> Option<(Packet, u32)> {
        let mut newest = None;
        let mut i = 0;
        while i < self.rx_len {
            if self.rx[i] != SYNC {
                i += 1;
                self.stats.bytes_skipped += 1;
                continue;
            }
            match protocol::parse_frame(&self.rx[i..self.rx_len]) {
                Frame::Incomplete => break,
                Frame::Invalid(err) => {
                    match err {
                        FrameError::BadCrc => self.stats.crc_errors += 1,
                        FrameError::UnsupportedGroup(_) | FrameError::UnsupportedField { .. } => {
                            self.stats.unsupported += 1
                        }
                    }
                    // Not a packet start after all: resume scanning at the next byte.
                    i += 1;
                    self.stats.bytes_skipped += 1;
                }
                Frame::Packet { packet, len } => {
                    if newest.is_some() {
                        self.stats.superseded += 1;
                    }
                    newest = Some((packet, self.next_seq));
                    self.next_seq = self.next_seq.wrapping_add(1);
                    self.stats.packets += 1;
                    i += len;
                }
            }
        }
        // Keep the unparsed tail (a packet still arriving) at the front.
        self.rx.copy_within(i..self.rx_len, 0);
        self.rx_len -= i;
        newest
    }
}

impl<S: ByteSource> Imu for Vn300<S> {
    type Error = S::Error;

    fn info(&self) -> &ImuInfo {
        &self.info
    }

    /// Drain the port, parse, and return the newest packet as a sample.
    ///
    /// Only the newest packet since the last call is returned; older ones are counted in
    /// [`Stats::superseded`], and `seq` jumps by the number dropped.
    fn read(&mut self, now_s: f64) -> Result<Option<ImuSample>, Self::Error> {
        let mut newest = None;
        // Alternate reading and parsing so a backlog larger than the buffer is not lost.
        // Bounded so a port that never runs dry cannot stall the control loop.
        for _ in 0..8 {
            let n = self.source.read_available(&mut self.rx[self.rx_len..])?;
            self.rx_len += n;
            if let Some(found) = self.parse_buffered() {
                if newest.is_some() {
                    self.stats.superseded += 1;
                }
                newest = Some(found);
            }
            if self.rx_len == RX_LEN {
                // Cannot happen with valid framing (RX_LEN > MAX_PACKET_LEN); drop a byte to make room.
                self.rx.copy_within(1..RX_LEN, 0);
                self.rx_len -= 1;
                self.stats.bytes_skipped += 1;
            }
            if n == 0 {
                break;
            }
        }

        let Some((packet, seq)) = newest else {
            return Ok(None);
        };
        self.last_packet = Some(packet);
        match to_sample(&packet, now_s, seq, &self.mounting) {
            Some(sample) => Ok(Some(sample)),
            None => {
                self.stats.incomplete += 1;
                Ok(None)
            }
        }
    }
}

fn v3(a: [f32; 3]) -> [f64; 3] {
    [a[0] as f64, a[1] as f64, a[2] as f64]
}

/// Packet (VN units, sensor axes) to sample (SI units, body frame).
/// None without both an accelerometer and a gyroscope reading.
pub fn to_sample(p: &Packet, now_s: f64, seq: u32, mounting: &Mounting) -> Option<ImuSample> {
    // Prefer measurements without the VN-300's own bias correction: the navigation filter
    // estimates bias itself. Fall back to the compensated ones if that is all the packet has.
    let (accel, accel_compensated) = match (p.uncomp_accel, p.accel) {
        (Some(a), _) => (a, false),
        (None, Some(a)) => (a, true),
        (None, None) => return None,
    };
    let (gyro, gyro_compensated) = match (p.uncomp_gyro, p.angular_rate) {
        (Some(g), _) => (g, false),
        (None, Some(g)) => (g, true),
        (None, None) => return None,
    };

    let mut s = ImuSample::new(now_s, v3(accel), v3(gyro));
    s.seq = seq;
    s.sensor_time_ns = p.time_startup_ns;
    s.mag = p.mag_gauss.or(p.uncomp_mag_gauss).map(|m| v3(m).map(gauss_to_tesla));
    s.temperature_c = p.temperature_c.map(f64::from);
    s.pressure_pa = p.pressure_kpa.map(|k| kpa_to_pa(k as f64));
    if let (Some(dt), Some(dtheta), Some(dvel)) = (p.delta_time_s, p.delta_theta_deg, p.delta_velocity) {
        s.delta = Some(DeltaIntegrals {
            delta_theta: v3(dtheta).map(deg_to_rad),
            delta_velocity: v3(dvel),
            dt_s: dt as f64,
        });
    }
    s.status = ImuStatus {
        bias_compensated: accel_compensated || gyro_compensated,
        // Saturation, magnetic disturbance and calibration state live in the INS status word,
        // kept raw in `Packet::ins_status` until its bits are mapped and checked on hardware.
        ..ImuStatus::default()
    };
    mounting.apply(&mut s);
    Some(s)
}

#[cfg(test)]
mod tests;
