//! ASCII configuration commands for the VN-300, built without allocation.
//!
//! VectorNav commands look like `$VNWRG,75,1,4,05,1C11,000C*hh\r\n`: a body between `$` and `*`,
//! then `hh`, a two-digit hex checksum that is the XOR of every body byte. Firmware sends these once at start-up,
//! then reads binary packets (see [`Vn300`](super::Vn300)).
//!
//! Electrical and start-up details (which port, baud rate change, waiting for the sensor to boot,
//! reading the replies) are left to vehicle firmware.

use core::fmt::Write;

use super::protocol::{common, group, imu};

/// Longest command this module builds, in bytes.
pub const MAX_COMMAND_LEN: usize = 96;

/// A finished command, ready to write to the serial port.
#[derive(Clone, Copy)]
pub struct Command {
    buf: [u8; MAX_COMMAND_LEN],
    len: usize,
}

impl Command {
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }

    pub fn as_str(&self) -> &str {
        // Only ASCII is ever written.
        core::str::from_utf8(self.as_bytes()).unwrap_or("")
    }
}

impl core::fmt::Debug for Command {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.as_str().trim_end())
    }
}

impl Write for Command {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        if self.len + bytes.len() > MAX_COMMAND_LEN {
            return Err(core::fmt::Error);
        }
        self.buf[self.len..self.len + bytes.len()].copy_from_slice(bytes);
        self.len += bytes.len();
        Ok(())
    }
}

/// XOR of every byte: the 8-bit checksum VectorNav uses for ASCII commands.
pub fn checksum(body: &[u8]) -> u8 {
    body.iter().fold(0, |acc, b| acc ^ b)
}

/// Wrap a command body (`VNWRG,06,0`) as `$<body>*<checksum>\r\n`. None if it does not fit.
pub fn command(body: &str) -> Option<Command> {
    let mut c = Command { buf: [0; MAX_COMMAND_LEN], len: 0 };
    write!(c, "${}*{:02X}\r\n", body, checksum(body.as_bytes())).ok()?;
    Some(c)
}

/// Turn off the default ASCII output (register 6) so only binary packets arrive.
/// The parser skips ASCII anyway, but it costs link bandwidth.
pub fn disable_ascii_output() -> Command {
    command("VNWRG,06,0").expect("fits")
}

/// Change the serial baud rate (register 5). The default 115200 is too slow for the
/// [`BinaryOutput::recommended`] packet much above 100 Hz; 921600 carries it at full rate.
/// The host port must switch to the new rate after this is sent.
pub fn set_baud_rate(baud: u32) -> Option<Command> {
    let mut body = Command { buf: [0; MAX_COMMAND_LEN], len: 0 };
    write!(body, "VNWRG,05,{}", baud).ok()?;
    command(body.as_str())
}

/// Store the current configuration so it survives a power cycle.
pub fn write_settings() -> Command {
    command("VNWNV").expect("fits")
}

/// Contents of a binary output register (75, 76 or 77: binary outputs 1 to 3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BinaryOutput {
    /// Which serial port(s) send it: 0 off, 1 port 1, 2 port 2, 3 both.
    pub async_mode: u8,
    /// Output rate = the sensor's IMU rate / `rate_divisor`. Check the IMU rate in the datasheet.
    pub rate_divisor: u16,
    /// Common group field mask (0 = group off).
    pub common: u16,
    /// Time group field mask (0 = group off).
    pub time: u16,
    /// IMU group field mask (0 = group off).
    pub imu: u16,
}

impl BinaryOutput {
    /// What the navigation filter needs, and nothing else:
    /// - Common: start-up time, onboard quaternion (for reference), magnetometer + temperature +
    ///   pressure, delta-theta / delta-velocity, INS status.
    /// - IMU: uncompensated accel and gyro. The Lander filter estimates bias itself, so it wants
    ///   measurements without the sensor's own bias correction.
    ///
    /// 106 bytes per packet.
    pub const fn recommended(async_mode: u8, rate_divisor: u16) -> Self {
        Self {
            async_mode,
            rate_divisor,
            common: common::TIME_STARTUP
                | common::QUATERNION
                | common::MAG_PRES
                | common::DELTA_THETA_VEL
                | common::INS_STATUS,
            time: 0,
            imu: imu::UNCOMP_ACCEL | imu::UNCOMP_GYRO,
        }
    }

    /// `groups` byte implied by the non-zero masks.
    pub fn groups(&self) -> u8 {
        let mut g = 0;
        if self.common != 0 {
            g |= group::COMMON;
        }
        if self.time != 0 {
            g |= group::TIME;
        }
        if self.imu != 0 {
            g |= group::IMU;
        }
        g
    }

    /// Bytes per packet, header and CRC included.
    pub fn packet_len(&self) -> usize {
        let masks = [self.common, self.time, self.imu];
        let present: [u16; 3] = {
            let mut out = [0u16; 3];
            let mut n = 0;
            for m in masks {
                if m != 0 {
                    out[n] = m;
                    n += 1;
                }
            }
            out
        };
        let n = self.groups().count_ones() as usize;
        let payload = super::protocol::payload_len(self.groups(), &present[..n]).unwrap_or(0);
        2 + 2 * n + payload + 2
    }

    /// The register write. `register` is 75, 76 or 77. Group and field masks go out in hex.
    pub fn command(&self, register: u8) -> Option<Command> {
        let mut body = Command { buf: [0; MAX_COMMAND_LEN], len: 0 };
        write!(body, "VNWRG,{},{},{},{:02X}", register, self.async_mode, self.rate_divisor, self.groups()).ok()?;
        for mask in [self.common, self.time, self.imu] {
            if mask != 0 {
                write!(body, ",{:04X}", mask).ok()?;
            }
        }
        command(body.as_str())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_wraps_body_with_checksum() {
        let c = command("VNRRG,01").unwrap();
        let expected = checksum(b"VNRRG,01");
        assert_eq!(c.as_str(), std::format!("$VNRRG,01*{:02X}\r\n", expected));
        // XOR checksum: a body with its own checksum byte appended folds to zero.
        let mut with_cs = std::vec::Vec::from(&b"VNRRG,01"[..]);
        with_cs.push(expected);
        assert_eq!(checksum(&with_cs), 0);
    }

    #[test]
    fn recommended_output_register_write() {
        let out = BinaryOutput::recommended(1, 4);
        let c = out.command(75).unwrap();
        let body = "VNWRG,75,1,4,05,1C11,000C";
        assert_eq!(c.as_str(), std::format!("${}*{:02X}\r\n", body, checksum(body.as_bytes())));
        assert_eq!(out.packet_len(), 106);
    }

    #[test]
    fn baud_and_ascii_commands() {
        assert!(set_baud_rate(921_600).unwrap().as_str().starts_with("$VNWRG,05,921600*"));
        assert!(disable_ascii_output().as_str().starts_with("$VNWRG,06,0*"));
        assert!(write_settings().as_str().starts_with("$VNWNV*"));
    }
}
