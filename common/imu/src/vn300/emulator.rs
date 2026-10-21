//! A stand-in for VN-300 hardware, so a simulator can run the real driver.
//!
//! The simulator knows the true accelerometer, gyroscope and magnetometer readings in the vehicle
//! body frame. This module turns them into exactly the bytes a VN-300 would send:
//!
//! 1. [`packet_from_sample`] converts a body-frame, SI-unit [`ImuSample`] back into what the sensor
//!    reports: its own axes (the inverse of the [`Mounting`]) and its own units (gauss, kPa, degrees).
//! 2. [`encode`] writes that [`Packet`] in the binary output format, CRC included.
//! 3. [`LoopbackPort`] holds the bytes and gives them to the driver as if a serial port had received them.
//!
//! The driver then parses, checks and converts them as it would on the vehicle, so the simulator
//! exercises the same code path as a flight with real hardware.
//!
//! ```
//! use imu::{Imu, ImuSample, Mounting};
//! use imu::vn300::{emulator, Vn300};
//!
//! let mounting = Mounting::Z_DOWN_TO_Z_UP;
//! let mut vn300 = Vn300::new(emulator::LoopbackPort::<1024>::new(), mounting);
//!
//! // Simulator truth: level and at rest, body frame.
//! let truth = ImuSample::new(0.01, [0.0, 0.0, 9.81], [0.0; 3]);
//! let packet = emulator::packet_from_sample(&truth, &mounting);
//! vn300.source_mut().push(emulator::encode(&packet).as_bytes());
//!
//! let sample = vn300.read(0.01).unwrap().expect("the driver decodes the emulated packet");
//! assert!((sample.accel[2] - 9.81).abs() < 1e-5);
//! ```

use super::protocol::{common, crc16, group, imu, Packet, MAX_PACKET_LEN, SYNC};
use super::ByteSource;
use crate::{ImuSample, Mounting};

/// A body-frame, SI-unit sample as the VN-300 would report it: sensor axes, VectorNav units.
///
/// Accelerometer and gyroscope go out as the uncompensated IMU-group fields (what the
/// recommended configuration sends), the magnetometer as the compensated one. The sensor's time
/// is the sample's `sensor_time_ns`, or its `timestamp_s` in nanoseconds when that is not set.
pub fn packet_from_sample(sample: &ImuSample, mounting: &Mounting) -> Packet {
    let to_sensor = mounting.inverse();
    let v = |a: [f64; 3]| {
        let r = to_sensor.rotate(a);
        [r[0] as f32, r[1] as f32, r[2] as f32]
    };
    let rad_to_deg = 180.0 / core::f64::consts::PI;
    Packet {
        time_startup_ns: Some(sample.sensor_time_ns.unwrap_or((sample.timestamp_s.max(0.0) * 1e9) as u64)),
        uncomp_accel: Some(v(sample.accel)),
        uncomp_gyro: Some(v(sample.gyro)),
        mag_gauss: sample.mag.map(|m| v(m.map(|t| t * 1e4))),
        temperature_c: sample.temperature_c.map(|t| t as f32),
        pressure_kpa: sample.pressure_pa.map(|p| (p / 1e3) as f32),
        delta_time_s: sample.delta.map(|d| d.dt_s as f32),
        delta_theta_deg: sample.delta.map(|d| v(d.delta_theta.map(|x| x * rad_to_deg))),
        delta_velocity: sample.delta.map(|d| v(d.delta_velocity)),
        ..Packet::default()
    }
}

/// One encoded packet.
#[derive(Clone, Copy)]
pub struct Encoded {
    buf: [u8; MAX_PACKET_LEN],
    len: usize,
}

impl Encoded {
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }

    fn put(&mut self, bytes: &[u8]) {
        self.buf[self.len..self.len + bytes.len()].copy_from_slice(bytes);
        self.len += bytes.len();
    }

    fn f32s(&mut self, values: &[f32]) {
        for v in values {
            self.put(&v.to_le_bytes());
        }
    }
}

/// Write `p` in the VN-300 binary output format. Each field present in `p` goes in one fixed place:
///
/// - Common group: start-up time, GPS time, yaw/pitch/roll, quaternion, compensated angular rate
///   and acceleration, INS status.
/// - IMU group: uncompensated magnetometer, accelerometer and gyroscope, temperature, pressure,
///   delta-theta (needs `delta_time_s` too), delta-velocity, compensated magnetometer.
///
/// Decoding the result gives back `p`. A packet with no fields at all is not valid VectorNav output.
pub fn encode(p: &Packet) -> Encoded {
    let mut common_mask = 0u16;
    let mut imu_mask = 0u16;
    let set = |present: bool, mask: &mut u16, bit: u16| {
        if present {
            *mask |= bit;
        }
    };
    set(p.time_startup_ns.is_some(), &mut common_mask, common::TIME_STARTUP);
    set(p.time_gps_ns.is_some(), &mut common_mask, common::TIME_GPS);
    set(p.yaw_pitch_roll_deg.is_some(), &mut common_mask, common::YAW_PITCH_ROLL);
    set(p.quaternion_xyzw.is_some(), &mut common_mask, common::QUATERNION);
    set(p.angular_rate.is_some(), &mut common_mask, common::ANGULAR_RATE);
    set(p.accel.is_some(), &mut common_mask, common::ACCEL);
    set(p.ins_status.is_some(), &mut common_mask, common::INS_STATUS);
    set(p.uncomp_mag_gauss.is_some(), &mut imu_mask, imu::UNCOMP_MAG);
    set(p.uncomp_accel.is_some(), &mut imu_mask, imu::UNCOMP_ACCEL);
    set(p.uncomp_gyro.is_some(), &mut imu_mask, imu::UNCOMP_GYRO);
    set(p.temperature_c.is_some(), &mut imu_mask, imu::TEMP);
    set(p.pressure_kpa.is_some(), &mut imu_mask, imu::PRES);
    let delta_theta = p.delta_time_s.zip(p.delta_theta_deg);
    set(delta_theta.is_some(), &mut imu_mask, imu::DELTA_THETA);
    set(p.delta_velocity.is_some(), &mut imu_mask, imu::DELTA_VEL);
    set(p.mag_gauss.is_some(), &mut imu_mask, imu::MAG);

    let mut groups = 0u8;
    if common_mask != 0 {
        groups |= group::COMMON;
    }
    if imu_mask != 0 {
        groups |= group::IMU;
    }

    let mut out = Encoded { buf: [0; MAX_PACKET_LEN], len: 0 };
    out.put(&[SYNC, groups]);
    if common_mask != 0 {
        out.put(&common_mask.to_le_bytes());
    }
    if imu_mask != 0 {
        out.put(&imu_mask.to_le_bytes());
    }

    // Payload: Common fields, then IMU fields, each in ascending bit order.
    if let Some(t) = p.time_startup_ns {
        out.put(&t.to_le_bytes());
    }
    if let Some(t) = p.time_gps_ns {
        out.put(&t.to_le_bytes());
    }
    if let Some(a) = p.yaw_pitch_roll_deg {
        out.f32s(&a);
    }
    if let Some(q) = p.quaternion_xyzw {
        out.f32s(&q);
    }
    if let Some(a) = p.angular_rate {
        out.f32s(&a);
    }
    if let Some(a) = p.accel {
        out.f32s(&a);
    }
    if let Some(s) = p.ins_status {
        out.put(&s.to_le_bytes());
    }
    if let Some(m) = p.uncomp_mag_gauss {
        out.f32s(&m);
    }
    if let Some(a) = p.uncomp_accel {
        out.f32s(&a);
    }
    if let Some(g) = p.uncomp_gyro {
        out.f32s(&g);
    }
    if let Some(t) = p.temperature_c {
        out.f32s(&[t]);
    }
    if let Some(pr) = p.pressure_kpa {
        out.f32s(&[pr]);
    }
    if let Some((dt, dtheta)) = delta_theta {
        out.f32s(&[dt]);
        out.f32s(&dtheta);
    }
    if let Some(dv) = p.delta_velocity {
        out.f32s(&dv);
    }
    if let Some(m) = p.mag_gauss {
        out.f32s(&m);
    }

    let crc = crc16(&out.buf[1..out.len]);
    out.put(&crc.to_be_bytes());
    out
}

/// In-memory serial port: bytes pushed in come out of [`ByteSource::read_available`], in order.
/// Holds up to `N` bytes; pushing more than fits drops the excess, like a full UART buffer.
pub struct LoopbackPort<const N: usize> {
    buf: [u8; N],
    len: usize,
    /// Bytes dropped because the buffer was full.
    pub overflowed: usize,
}

impl<const N: usize> LoopbackPort<N> {
    pub const fn new() -> Self {
        Self { buf: [0; N], len: 0, overflowed: 0 }
    }

    /// Queue bytes for the driver. Returns how many fit.
    pub fn push(&mut self, bytes: &[u8]) -> usize {
        let n = bytes.len().min(N - self.len);
        self.buf[self.len..self.len + n].copy_from_slice(&bytes[..n]);
        self.len += n;
        self.overflowed += bytes.len() - n;
        n
    }

    /// Bytes waiting to be read.
    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

impl<const N: usize> Default for LoopbackPort<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> ByteSource for LoopbackPort<N> {
    type Error = core::convert::Infallible;

    fn read_available(&mut self, out: &mut [u8]) -> Result<usize, Self::Error> {
        let n = out.len().min(self.len);
        out[..n].copy_from_slice(&self.buf[..n]);
        self.buf.copy_within(n..self.len, 0);
        self.len -= n;
        Ok(n)
    }
}
