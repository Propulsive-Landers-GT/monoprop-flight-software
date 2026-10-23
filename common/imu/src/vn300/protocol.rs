//! VectorNav binary output protocol, as emitted by the VN-300 (shared with the VN-100 and VN-200).
//!
//! A packet is
//!
//! ```text
//! 0xFA | groups: u8 | one u16 field mask per group present | payload | CRC-16, big-endian
//! ```
//!
//! - `groups` has one bit per output group (Common, Time, IMU, GNSS1, Attitude, INS, GNSS2).
//! - Each mask has one bit per field of that group. Fields appear in the payload in group order,
//!   then bit order, little-endian, with a fixed size per field.
//! - The CRC is CRC-16-CCITT (polynomial 0x1021, initial value 0) over everything after the sync
//!   byte. Running it over the payload *and* the appended CRC yields 0.
//!
//! This parser decodes the Common, Time and IMU groups, which hold everything an [`ImuSample`]
//! needs. The GNSS, Attitude and INS groups are not decoded: a packet that contains them is
//! reported as [`FrameError::UnsupportedGroup`] rather than misparsed. Supporting them means
//! adding their field-size tables here.
//!
//! [`ImuSample`]: crate::ImuSample

/// First byte of every binary packet.
pub const SYNC: u8 = 0xFA;

/// Bits of the `groups` byte.
pub mod group {
    pub const COMMON: u8 = 1 << 0;
    pub const TIME: u8 = 1 << 1;
    pub const IMU: u8 = 1 << 2;
    pub const GNSS1: u8 = 1 << 3;
    pub const ATTITUDE: u8 = 1 << 4;
    pub const INS: u8 = 1 << 5;
    pub const GNSS2: u8 = 1 << 6;
    /// The groups this parser decodes.
    pub const SUPPORTED: u8 = COMMON | TIME | IMU;
}

/// Field bits of the Common group (group 1).
pub mod common {
    pub const TIME_STARTUP: u16 = 1 << 0;
    pub const TIME_GPS: u16 = 1 << 1;
    pub const TIME_SYNC_IN: u16 = 1 << 2;
    pub const YAW_PITCH_ROLL: u16 = 1 << 3;
    pub const QUATERNION: u16 = 1 << 4;
    pub const ANGULAR_RATE: u16 = 1 << 5;
    pub const POSITION: u16 = 1 << 6;
    pub const VELOCITY: u16 = 1 << 7;
    pub const ACCEL: u16 = 1 << 8;
    /// Uncompensated accel + gyro.
    pub const IMU: u16 = 1 << 9;
    /// Magnetometer, temperature, pressure.
    pub const MAG_PRES: u16 = 1 << 10;
    pub const DELTA_THETA_VEL: u16 = 1 << 11;
    pub const INS_STATUS: u16 = 1 << 12;
    pub const SYNC_IN_CNT: u16 = 1 << 13;
    pub const TIME_GPS_PPS: u16 = 1 << 14;
}

/// Field bits of the Time group (group 2).
pub mod time {
    pub const TIME_STARTUP: u16 = 1 << 0;
    pub const TIME_GPS: u16 = 1 << 1;
    pub const GPS_TOW: u16 = 1 << 2;
    pub const GPS_WEEK: u16 = 1 << 3;
    pub const TIME_SYNC_IN: u16 = 1 << 4;
    pub const TIME_GPS_PPS: u16 = 1 << 5;
    pub const TIME_UTC: u16 = 1 << 6;
    pub const SYNC_IN_CNT: u16 = 1 << 7;
    pub const SYNC_OUT_CNT: u16 = 1 << 8;
    pub const TIME_STATUS: u16 = 1 << 9;
}

/// Field bits of the IMU group (group 3).
pub mod imu {
    pub const IMU_STATUS: u16 = 1 << 0;
    pub const UNCOMP_MAG: u16 = 1 << 1;
    pub const UNCOMP_ACCEL: u16 = 1 << 2;
    pub const UNCOMP_GYRO: u16 = 1 << 3;
    pub const TEMP: u16 = 1 << 4;
    pub const PRES: u16 = 1 << 5;
    pub const DELTA_THETA: u16 = 1 << 6;
    pub const DELTA_VEL: u16 = 1 << 7;
    pub const MAG: u16 = 1 << 8;
    pub const ACCEL: u16 = 1 << 9;
    pub const ANGULAR_RATE: u16 = 1 << 10;
}

/// Payload bytes per field bit. 0 marks a reserved bit, which this parser rejects.
const COMMON_SIZES: [u8; 16] = [8, 8, 8, 12, 16, 12, 24, 12, 12, 24, 20, 28, 2, 4, 8, 0];
const TIME_SIZES: [u8; 16] = [8, 8, 8, 2, 8, 8, 8, 4, 4, 1, 0, 0, 0, 0, 0, 0];
const IMU_SIZES: [u8; 16] = [2, 12, 12, 12, 4, 4, 16, 12, 12, 12, 12, 0, 0, 0, 0, 0];

/// Largest packet the supported groups can produce: header, every field of all three groups, CRC.
pub const MAX_PACKET_LEN: usize = 2 + 2 * 3 + 198 + 59 + 110 + 2;

/// One decoded packet, in the VN-300's own units and sensor axes. `None` = field not in the packet.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Packet {
    /// Time since the sensor powered up [ns].
    pub time_startup_ns: Option<u64>,
    /// GPS time [ns].
    pub time_gps_ns: Option<u64>,
    /// Onboard estimate [deg].
    pub yaw_pitch_roll_deg: Option<[f32; 3]>,
    /// Onboard estimate, scalar last: [x, y, z, w].
    pub quaternion_xyzw: Option<[f32; 4]>,
    /// Bias-compensated by the onboard filter [rad/s].
    pub angular_rate: Option<[f32; 3]>,
    /// Bias-compensated by the onboard filter, includes gravity [m/s^2].
    pub accel: Option<[f32; 3]>,
    /// Factory-calibrated, not bias-compensated [m/s^2].
    pub uncomp_accel: Option<[f32; 3]>,
    /// Factory-calibrated, not bias-compensated [rad/s].
    pub uncomp_gyro: Option<[f32; 3]>,
    /// Compensated magnetic field [gauss].
    pub mag_gauss: Option<[f32; 3]>,
    /// Uncompensated magnetic field [gauss].
    pub uncomp_mag_gauss: Option<[f32; 3]>,
    /// [degC]
    pub temperature_c: Option<f32>,
    /// [kPa]
    pub pressure_kpa: Option<f32>,
    /// Integration interval of the delta fields [s].
    pub delta_time_s: Option<f32>,
    /// [deg]
    pub delta_theta_deg: Option<[f32; 3]>,
    /// [m/s]
    pub delta_velocity: Option<[f32; 3]>,
    /// Raw INS status word.
    pub ins_status: Option<u16>,
}

/// Why bytes starting at a sync byte are not a usable packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// The packet uses a group this parser does not decode (or a reserved group bit).
    UnsupportedGroup(u8),
    /// A reserved field bit is set in a supported group.
    UnsupportedField { group: u8, bit: u8 },
    /// The checksum did not match: corruption, or a stray 0xFA that is not a packet start.
    BadCrc,
}

/// Result of looking for a packet at the start of a buffer.
// Returned by value and consumed immediately, never stored; boxing needs an allocator (no_std).
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Frame {
    /// A valid packet that occupies the first `len` bytes.
    Packet { packet: Packet, len: usize },
    /// Starts like a packet, but not all of it has arrived yet.
    Incomplete,
    /// Not a usable packet. Skip the sync byte and keep scanning.
    Invalid(FrameError),
}

/// CRC-16-CCITT as VectorNav specifies it (polynomial 0x1021, initial value 0; also called XMODEM).
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        crc = crc.rotate_left(8);
        crc ^= byte as u16;
        crc ^= (crc & 0x00ff) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0x00ff) << 5;
    }
    crc
}

fn sizes_for(group_bit: u8) -> Option<&'static [u8; 16]> {
    match 1u8 << group_bit {
        group::COMMON => Some(&COMMON_SIZES),
        group::TIME => Some(&TIME_SIZES),
        group::IMU => Some(&IMU_SIZES),
        _ => None,
    }
}

/// Payload length of a packet with this `groups` byte and these field masks (one per group, in order).
pub fn payload_len(groups: u8, masks: &[u16]) -> Result<usize, FrameError> {
    if groups & !group::SUPPORTED != 0 {
        return Err(FrameError::UnsupportedGroup(groups & !group::SUPPORTED));
    }
    let mut len = 0;
    let mut masks = masks.iter();
    for g in 0..8u8 {
        if groups & (1 << g) == 0 {
            continue;
        }
        let sizes = sizes_for(g).ok_or(FrameError::UnsupportedGroup(1 << g))?;
        let mask = *masks.next().unwrap_or(&0);
        for bit in 0..16u8 {
            if mask & (1 << bit) != 0 {
                match sizes[bit as usize] {
                    0 => return Err(FrameError::UnsupportedField { group: 1 << g, bit }),
                    n => len += n as usize,
                }
            }
        }
    }
    Ok(len)
}

/// Look for a packet at the start of `buf`, which must begin with [`SYNC`].
pub fn parse_frame(buf: &[u8]) -> Frame {
    debug_assert!(buf.first() == Some(&SYNC));
    if buf.len() < 2 {
        return Frame::Incomplete;
    }
    let groups = buf[1];
    if groups == 0 || groups & !group::SUPPORTED != 0 {
        return Frame::Invalid(FrameError::UnsupportedGroup(groups & !group::SUPPORTED));
    }
    let n_groups = groups.count_ones() as usize;
    let header = 2 + 2 * n_groups;
    if buf.len() < header {
        return Frame::Incomplete;
    }
    let mut masks = [0u16; 3];
    for (i, mask) in masks.iter_mut().enumerate().take(n_groups) {
        *mask = u16::from_le_bytes([buf[2 + 2 * i], buf[3 + 2 * i]]);
    }
    let payload = match payload_len(groups, &masks[..n_groups]) {
        Ok(n) => n,
        Err(e) => return Frame::Invalid(e),
    };
    let len = header + payload + 2;
    if buf.len() < len {
        return Frame::Incomplete;
    }
    if crc16(&buf[1..len]) != 0 {
        return Frame::Invalid(FrameError::BadCrc);
    }
    let packet = decode(groups, &masks[..n_groups], &buf[header..header + payload]);
    Frame::Packet { packet, len }
}

/// Little-endian reader over a payload whose length has already been checked.
struct Reader<'a> {
    bytes: &'a [u8],
    at: usize,
}

impl Reader<'_> {
    fn take<const N: usize>(&mut self) -> [u8; N] {
        let mut out = [0u8; N];
        out.copy_from_slice(&self.bytes[self.at..self.at + N]);
        self.at += N;
        out
    }
    fn skip(&mut self, n: usize) {
        self.at += n;
    }
    fn u16(&mut self) -> u16 {
        u16::from_le_bytes(self.take())
    }
    fn u64(&mut self) -> u64 {
        u64::from_le_bytes(self.take())
    }
    fn f32(&mut self) -> f32 {
        f32::from_le_bytes(self.take())
    }
    fn f32x3(&mut self) -> [f32; 3] {
        [self.f32(), self.f32(), self.f32()]
    }
}

fn decode(groups: u8, masks: &[u16], payload: &[u8]) -> Packet {
    let mut p = Packet::default();
    let mut r = Reader { bytes: payload, at: 0 };
    let mut masks = masks.iter();
    for g in 0..8u8 {
        if groups & (1 << g) == 0 {
            continue;
        }
        let group_bit = 1u8 << g;
        let sizes = sizes_for(g).expect("checked by payload_len");
        let mask = *masks.next().unwrap_or(&0);
        for bit in 0..16u8 {
            let field = 1u16 << bit;
            if mask & field == 0 {
                continue;
            }
            match (group_bit, field) {
                (group::COMMON, common::TIME_STARTUP) | (group::TIME, time::TIME_STARTUP) => {
                    p.time_startup_ns = Some(r.u64())
                }
                (group::COMMON, common::TIME_GPS) | (group::TIME, time::TIME_GPS) => p.time_gps_ns = Some(r.u64()),
                (group::COMMON, common::YAW_PITCH_ROLL) => p.yaw_pitch_roll_deg = Some(r.f32x3()),
                (group::COMMON, common::QUATERNION) => {
                    p.quaternion_xyzw = Some([r.f32(), r.f32(), r.f32(), r.f32()])
                }
                (group::COMMON, common::ANGULAR_RATE) | (group::IMU, imu::ANGULAR_RATE) => {
                    p.angular_rate = Some(r.f32x3())
                }
                (group::COMMON, common::ACCEL) | (group::IMU, imu::ACCEL) => p.accel = Some(r.f32x3()),
                (group::COMMON, common::IMU) => {
                    p.uncomp_accel = Some(r.f32x3());
                    p.uncomp_gyro = Some(r.f32x3());
                }
                (group::COMMON, common::MAG_PRES) => {
                    p.mag_gauss = Some(r.f32x3());
                    p.temperature_c = Some(r.f32());
                    p.pressure_kpa = Some(r.f32());
                }
                (group::COMMON, common::DELTA_THETA_VEL) => {
                    p.delta_time_s = Some(r.f32());
                    p.delta_theta_deg = Some(r.f32x3());
                    p.delta_velocity = Some(r.f32x3());
                }
                (group::COMMON, common::INS_STATUS) => p.ins_status = Some(r.u16()),
                (group::IMU, imu::UNCOMP_MAG) => p.uncomp_mag_gauss = Some(r.f32x3()),
                (group::IMU, imu::UNCOMP_ACCEL) => p.uncomp_accel = Some(r.f32x3()),
                (group::IMU, imu::UNCOMP_GYRO) => p.uncomp_gyro = Some(r.f32x3()),
                (group::IMU, imu::TEMP) => p.temperature_c = Some(r.f32()),
                (group::IMU, imu::PRES) => p.pressure_kpa = Some(r.f32()),
                (group::IMU, imu::DELTA_THETA) => {
                    p.delta_time_s = Some(r.f32());
                    p.delta_theta_deg = Some(r.f32x3());
                }
                (group::IMU, imu::DELTA_VEL) => p.delta_velocity = Some(r.f32x3()),
                (group::IMU, imu::MAG) => p.mag_gauss = Some(r.f32x3()),
                // Present but not needed for an ImuSample: step over it.
                _ => r.skip(sizes[bit as usize] as usize),
            }
        }
    }
    p
}
