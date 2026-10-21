//! The VN-300 on the flight computer's serial port.
//!
//! The `imu` crate's VN-300 driver is hardware-independent: it only needs something that hands it
//! the bytes that have arrived. This module provides that for a serial device (a USB adapter or a
//! board UART), sends the start-up configuration, and builds the driver.
//!
//! Settings come from environment variables so the same binary runs on any computer:
//!
//! | Variable | Default | Meaning |
//! |---|---|---|
//! | `VN300_PORT` | `/dev/ttyUSB0` | Serial device (`COM3` on Windows) |
//! | `VN300_BAUD` | `115200` | Baud rate the sensor is currently set to (factory default 115200) |
//! | `VN300_RATE_DIVISOR` | `4` | Binary output rate = the sensor's IMU rate / this |
//! | `VN300_CONFIGURE` | `1` | `0` skips the start-up configuration (sensor already set up and saved) |
//!
//! At 115200 baud the recommended 106-byte packet fits about 100 times a second. For faster output,
//! set the sensor to a higher baud once (see `imu::vn300::config::set_baud_rate`), save it, and set
//! `VN300_BAUD` to match.

use imu::vn300::config::{self, BinaryOutput};
use imu::vn300::{ByteSource, Vn300};
use imu::Mounting;
use std::io::{ErrorKind, Read, Write};
use std::time::Duration;

/// A serial port as the VN-300 driver's byte source. Never blocks.
pub struct SerialByteSource {
    port: Box<dyn serialport::SerialPort>,
}

impl ByteSource for SerialByteSource {
    type Error = serialport::Error;

    fn read_available(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let waiting = self.port.bytes_to_read()? as usize;
        if waiting == 0 || buf.is_empty() {
            return Ok(0);
        }
        let n = waiting.min(buf.len());
        match self.port.read(&mut buf[..n]) {
            Ok(n) => Ok(n),
            Err(e) if matches!(e.kind(), ErrorKind::TimedOut | ErrorKind::WouldBlock | ErrorKind::Interrupted) => Ok(0),
            Err(e) => Err(e.into()),
        }
    }
}

/// Where the VN-300 is and how to set it up. See the module docs for the environment variables.
#[derive(Debug, Clone, PartialEq)]
pub struct Vn300Config {
    pub port: String,
    pub baud: u32,
    pub rate_divisor: u16,
    /// Send the binary output configuration at start-up.
    pub configure: bool,
    /// How the unit is mounted. Per-vehicle mounting configuration is a later integration step;
    /// this assumes the sensor's X axis is the vehicle's X axis.
    pub mounting: Mounting,
}

impl Default for Vn300Config {
    fn default() -> Self {
        Self {
            port: "/dev/ttyUSB0".to_string(),
            baud: 115_200,
            rate_divisor: 4,
            configure: true,
            mounting: Mounting::Z_DOWN_TO_Z_UP,
        }
    }
}

impl Vn300Config {
    /// Defaults, overridden by `VN300_PORT`, `VN300_BAUD`, `VN300_RATE_DIVISOR` and `VN300_CONFIGURE`.
    pub fn from_env() -> Self {
        Self::from_lookup(|name| std::env::var(name).ok())
    }

    fn from_lookup(get: impl Fn(&str) -> Option<String>) -> Self {
        let mut c = Self::default();
        let get = |name: &str| get(name).map(|v| v.trim().to_string()).filter(|v| !v.is_empty());
        if let Some(port) = get("VN300_PORT") {
            c.port = port;
        }
        if let Some(baud) = get("VN300_BAUD").and_then(|v| v.parse().ok()) {
            c.baud = baud;
        }
        if let Some(div) = get("VN300_RATE_DIVISOR").and_then(|v| v.parse().ok()).filter(|d: &u16| *d > 0) {
            c.rate_divisor = div;
        }
        if let Some(flag) = get("VN300_CONFIGURE") {
            c.configure = flag != "0";
        }
        c
    }
}

/// Open the port, configure the sensor (unless `configure` is off) and return the driver.
pub fn open_vn300(cfg: &Vn300Config) -> Result<Vn300<SerialByteSource>, String> {
    let mut port = serialport::new(&cfg.port, cfg.baud)
        .timeout(Duration::from_millis(1))
        .open()
        .map_err(|e| format!("could not open {} at {} baud: {}", cfg.port, cfg.baud, e.to_string().trim_end_matches('.')))?;

    if cfg.configure {
        // Volatile register writes: they last until the sensor power-cycles. Binary output 1 is
        // sent on both of the sensor's serial ports so it reaches whichever one is wired.
        let output = BinaryOutput::recommended(3, cfg.rate_divisor);
        let binary = output.command(75).ok_or("binary output command too long")?;
        for cmd in [config::disable_ascii_output(), binary] {
            port.write_all(cmd.as_bytes()).map_err(|e| format!("could not write to {}: {}", cfg.port, e))?;
            std::thread::sleep(Duration::from_millis(20));
        }
        // Drop the sensor's replies and any ASCII output still in flight.
        let _ = port.clear(serialport::ClearBuffer::Input);
    }

    Ok(Vn300::new(SerialByteSource { port }, cfg.mounting))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_without_environment() {
        assert_eq!(Vn300Config::from_lookup(|_| None), Vn300Config::default());
    }

    #[test]
    fn environment_overrides() {
        let c = Vn300Config::from_lookup(|name| {
            match name {
                "VN300_PORT" => Some("COM5"),
                "VN300_BAUD" => Some(" 921600 "),
                "VN300_RATE_DIVISOR" => Some("2"),
                "VN300_CONFIGURE" => Some("0"),
                _ => None,
            }
            .map(str::to_string)
        });
        assert_eq!(c.port, "COM5");
        assert_eq!(c.baud, 921_600);
        assert_eq!(c.rate_divisor, 2);
        assert!(!c.configure);
    }

    #[test]
    fn bad_values_keep_defaults() {
        let c = Vn300Config::from_lookup(|name| match name {
            "VN300_BAUD" => Some("fast".to_string()),
            "VN300_RATE_DIVISOR" => Some("0".to_string()),
            "VN300_PORT" => Some("  ".to_string()),
            _ => None,
        });
        assert_eq!(c, Vn300Config::default());
    }

    #[test]
    fn missing_port_is_a_clear_error() {
        let cfg = Vn300Config { port: "/dev/no-such-vn300".to_string(), ..Vn300Config::default() };
        let err = open_vn300(&cfg).err().expect("opening a missing port fails");
        assert!(err.contains("/dev/no-such-vn300"), "{}", err);
    }
}