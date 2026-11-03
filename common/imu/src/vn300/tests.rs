use super::protocol::{common, crc16, group, imu, SYNC};
use super::*;
use std::vec;
use std::vec::Vec;

/// Fake serial port: tests push bytes, the driver drains them.
#[derive(Default)]
struct Port {
    pending: Vec<u8>,
}

impl ByteSource for Port {
    type Error = ();
    fn read_available(&mut self, buf: &mut [u8]) -> Result<usize, ()> {
        let n = buf.len().min(self.pending.len());
        buf[..n].copy_from_slice(&self.pending[..n]);
        self.pending.drain(..n);
        Ok(n)
    }
}

fn driver(mounting: Mounting) -> Vn300<Port> {
    Vn300::new(Port::default(), mounting)
}

fn feed(vn: &mut Vn300<Port>, bytes: &[u8]) {
    vn.source_mut().pending.extend_from_slice(bytes);
}

/// Encode a packet exactly as the sensor would: sync, groups, masks, payload, big-endian CRC.
fn packet(groups: u8, masks: &[u16], payload: &[u8]) -> Vec<u8> {
    let mut body = vec![groups];
    for m in masks {
        body.extend_from_slice(&m.to_le_bytes());
    }
    body.extend_from_slice(payload);
    let crc = crc16(&body);
    let mut out = vec![SYNC];
    out.extend(body);
    out.extend_from_slice(&crc.to_be_bytes());
    out
}

fn f32s(values: &[f32]) -> Vec<u8> {
    values.iter().flat_map(|v| v.to_le_bytes()).collect()
}

/// Common group: start-up time, angular rate, accel, mag + temperature + pressure.
fn common_packet(time_ns: u64, accel: [f32; 3], gyro: [f32; 3]) -> Vec<u8> {
    let mask = common::TIME_STARTUP | common::ANGULAR_RATE | common::ACCEL | common::MAG_PRES;
    let mut payload = time_ns.to_le_bytes().to_vec();
    payload.extend(f32s(&gyro));
    payload.extend(f32s(&accel));
    payload.extend(f32s(&[0.2, 0.0, -0.4, 25.0, 101.325])); // mag [G], temp [C], pressure [kPa]
    packet(group::COMMON, &[mask], &payload)
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-5 * b.abs().max(1.0)
}

fn close3(a: [f64; 3], b: [f64; 3]) -> bool {
    (0..3).all(|i| close(a[i], b[i]))
}

#[test]
fn crc_matches_the_ccitt_xmodem_check_value() {
    assert_eq!(crc16(b"123456789"), 0x31C3);
    // Over a whole packet after the sync byte, CRC included, it comes out zero.
    let p = common_packet(1, [0.0; 3], [0.0; 3]);
    assert_eq!(crc16(&p[1..]), 0);
}

#[test]
fn common_group_becomes_an_si_body_frame_sample() {
    let mut vn = driver(Mounting::Z_DOWN_TO_Z_UP);
    // Level and at rest, VectorNav forward-right-down axes: -g on Z.
    feed(&mut vn, &common_packet(123_456_789, [0.0, 0.0, -9.81], [0.1, 0.2, 0.3]));
    let s = vn.read(5.0).unwrap().expect("one sample");

    assert_eq!(s.timestamp_s, 5.0);
    assert_eq!(s.sensor_time_ns, Some(123_456_789));
    assert!(close3(s.accel, [0.0, 0.0, 9.81]), "accel {:?}", s.accel);
    assert!(close3(s.gyro, [0.1, -0.2, -0.3]), "gyro {:?}", s.gyro);
    assert!(close3(s.mag.unwrap(), [0.2e-4, 0.0, 0.4e-4]), "mag {:?}", s.mag); // gauss -> tesla, Z flipped
    assert!(close(s.temperature_c.unwrap(), 25.0));
    assert!(close(s.pressure_pa.unwrap(), 101_325.0));
    assert!(s.status.bias_compensated, "Common-group accel/rate are bias-compensated");
    assert!(s.is_finite());
    assert_eq!(vn.stats().packets, 1);
}

#[test]
fn uncompensated_imu_group_is_preferred() {
    let mut vn = driver(Mounting::IDENTITY);
    let mut payload = f32s(&[0.0, 0.0, -9.0, 0.5, 0.5, 0.5]); // Common: compensated rate, accel
    payload.extend(f32s(&[0.0, 0.0, -9.5, 0.1, 0.2, 0.3])); // IMU: uncompensated accel, gyro
    feed(
        &mut vn,
        &packet(
            group::COMMON | group::IMU,
            &[common::ANGULAR_RATE | common::ACCEL, imu::UNCOMP_ACCEL | imu::UNCOMP_GYRO],
            &payload,
        ),
    );
    let s = vn.read(0.0).unwrap().unwrap();
    assert!(close3(s.accel, [0.0, 0.0, -9.5]));
    assert!(close3(s.gyro, [0.1, 0.2, 0.3]));
    assert!(!s.status.bias_compensated);
}

#[test]
fn skips_ascii_and_waits_for_a_split_packet() {
    let mut vn = driver(Mounting::IDENTITY);
    let p = common_packet(7, [0.0, 0.0, -9.81], [0.0; 3]);
    let ascii = b"$VNYMR,+010.071,-000.278,-000.926,+1.0640,-0.2263,+3.0878*64\r\n";
    feed(&mut vn, ascii);
    feed(&mut vn, &p[..10]);
    assert_eq!(vn.read(0.0), Ok(None), "half a packet is not a sample");
    feed(&mut vn, &p[10..]);
    let s = vn.read(0.002).unwrap().expect("completed packet");
    assert_eq!(s.sensor_time_ns, Some(7));
    assert_eq!(vn.stats().bytes_skipped as usize, ascii.len());
    assert_eq!(vn.stats().crc_errors, 0);
}

#[test]
fn corrupted_packet_is_skipped_and_counted() {
    let mut vn = driver(Mounting::IDENTITY);
    let mut bad = common_packet(1, [0.0, 0.0, -9.81], [0.0; 3]);
    bad[12] ^= 0xFF;
    feed(&mut vn, &bad);
    feed(&mut vn, &common_packet(2, [0.0, 0.0, -9.81], [0.0; 3]));
    let s = vn.read(0.0).unwrap().expect("the good packet");
    assert_eq!(s.sensor_time_ns, Some(2));
    assert_eq!(vn.stats().crc_errors, 1);
}

#[test]
fn newest_packet_wins_and_seq_shows_the_dropped_one() {
    let mut vn = driver(Mounting::IDENTITY);
    feed(&mut vn, &common_packet(1, [0.0, 0.0, -9.81], [0.0; 3]));
    let first = vn.read(0.0).unwrap().unwrap();
    feed(&mut vn, &common_packet(2, [0.0, 0.0, -9.81], [0.0; 3]));
    feed(&mut vn, &common_packet(3, [0.0, 0.0, -9.81], [0.0; 3]));
    let s = vn.read(0.002).unwrap().unwrap();
    assert_eq!(s.sensor_time_ns, Some(3));
    assert_eq!(s.seq, first.seq + 2, "seq skips the superseded packet");
    assert_eq!(vn.stats().superseded, 1);
    assert_eq!(vn.read(0.004), Ok(None), "nothing new: no sample");
}

#[test]
fn gnss_groups_are_reported_unsupported_not_misparsed() {
    let mut vn = driver(Mounting::IDENTITY);
    feed(&mut vn, &packet(group::GNSS1, &[0x0001], &[0u8; 8]));
    assert_eq!(vn.read(0.0), Ok(None));
    assert!(vn.stats().unsupported >= 1);
    assert_eq!(vn.stats().packets, 0);
}

#[test]
fn packet_without_accel_or_gyro_gives_no_sample() {
    let mut vn = driver(Mounting::IDENTITY);
    feed(&mut vn, &packet(group::COMMON, &[common::TIME_STARTUP], &42u64.to_le_bytes()));
    assert_eq!(vn.read(0.0), Ok(None));
    assert_eq!(vn.stats().incomplete, 1);
    assert_eq!(vn.last_packet().unwrap().time_startup_ns, Some(42));
}

#[test]
fn delta_theta_is_converted_to_radians() {
    let mut vn = driver(Mounting::IDENTITY);
    let mut payload = f32s(&[0.0, 0.0, -9.81, 0.0, 0.0, 0.0]); // Common Imu: uncompensated accel, gyro
    payload.extend(f32s(&[0.0025, 1.0, 0.0, 0.0, 0.0, 0.0, -0.0245])); // dt, dtheta [deg], dvel
    feed(&mut vn, &packet(group::COMMON, &[common::IMU | common::DELTA_THETA_VEL], &payload));
    let d = vn.read(0.0).unwrap().unwrap().delta.expect("delta integrals");
    assert!(close(d.dt_s, 0.0025));
    assert!(close3(d.delta_theta, [1.0_f64.to_radians(), 0.0, 0.0]));
    assert!(close3(d.delta_velocity, [0.0, 0.0, -0.0245]));
}

#[test]
fn recommended_configuration_round_trips_through_the_parser() {
    let out = config::BinaryOutput::recommended(1, 4);
    let mut payload = 9u64.to_le_bytes().to_vec();
    payload.extend(f32s(&[0.0, 0.0, 0.0, 1.0])); // quaternion x y z w
    payload.extend(f32s(&[0.2, 0.0, -0.4, 25.0, 101.325])); // mag, temp, pressure
    payload.extend(f32s(&[0.0025, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0245])); // deltas
    payload.extend(0u16.to_le_bytes()); // INS status
    payload.extend(f32s(&[0.0, 0.0, -9.81, 0.0, 0.0, 0.0])); // IMU group: uncomp accel, gyro
    let p = packet(out.groups(), &[out.common, out.imu], &payload);
    assert_eq!(p.len(), out.packet_len());

    let mut vn = driver(Mounting::Z_DOWN_TO_Z_UP);
    feed(&mut vn, &p);
    let s = vn.read(0.0).unwrap().unwrap();
    assert!(close3(s.accel, [0.0, 0.0, 9.81]));
    assert!(s.mag.is_some() && s.delta.is_some() && s.pressure_pa.is_some());
    assert!(!s.status.bias_compensated);
    assert_eq!(vn.last_packet().unwrap().quaternion_xyzw, Some([0.0, 0.0, 0.0, 1.0]));
}
