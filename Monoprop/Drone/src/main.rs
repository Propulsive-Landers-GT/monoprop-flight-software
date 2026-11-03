#![no_std]
#![no_main]

use teensy4_bsp as bsp;
use bsp::board;
use bsp::hal::gpt;
use Common::GroundCommand;
use Drone::clock::WrapExtender;
use Drone::config::CONFIG;
use Drone::navigator::{Navigator, encode_telemetry};
use Drone::sensors;

// State machine for monocopter
#[derive(Copy, Clone, PartialEq)]
enum DroneState {
    Idle,
    Flying,
    Landing,
}

#[bsp::rt::entry]
fn main() -> ! {
    let board::Resources {
        mut gpio2,
        pins,
        lpuart6,
        mut gpt1,
        mut gpt2,
        ..
    } = board::t41(board::instances());

    // Create LPUART peripheral for communication with ESP32
    // where pins.p1 = LPUART6_TX, pins.p0 = LPUART6_RX
    let mut lpuart6 = board::lpuart(lpuart6, pins.p1, pins.p0, 115200);

    // Create the LED peripheral. There is only one LED on the Teensy 4.1, connected to pin 13.
    let led = board::led(&mut gpio2, pins.p13);

    gpt1.set_clock_source(bsp::hal::gpt::ClockSource::PeripheralClock);
    let mut delay = bsp::hal::timer::Blocking::<_, 1_000_000>::from_gpt(gpt1);

    // GPT2 free-running at PERCLK (1 MHz): the microsecond clock
    gpt2.disable();
    gpt2.set_clock_source(gpt::ClockSource::PeripheralClock);
    gpt2.set_divider(1);
    gpt2.set_mode(gpt::Mode::FreeRunning);
    gpt2.set_reset_on_enable(true);
    gpt2.enable();
    let mut clock = WrapExtender::default();

    let mut buf = [0u8; 256];
    let mut idx = 0;
    let mut state = DroneState::Idle;
    let mut latest_cmd: Option<GroundCommand> = None; // Store latest control command
    const GROUND_COMMAND_SIZE: usize = core::mem::size_of::<GroundCommand>();

    let mut nav = Navigator::new(&CONFIG); // also initialises the sensors
    let mut tx = [0u8; 64];
    let (mut tx_len, mut tx_pos) = (0usize, 0usize);
    let mut next_tx_us = 0u64;

    loop {
        let now = clock.extend(gpt2.count());

        // Read commands from ESP32
        match lpuart6.try_read() {
            Ok(Some(byte)) => {
                if idx < buf.len() {
                    buf[idx] = byte;
                    idx += 1;
                }

                // Check for binary GroundCommand packet
                if idx >= GROUND_COMMAND_SIZE {
                    if let Ok(cmd) = postcard::from_bytes::<GroundCommand>(&buf[..GROUND_COMMAND_SIZE]) {
                        latest_cmd = Some(cmd); // stored for continuous control
                    }
                    idx = 0; // Reset buffer after processing
                }

                // Check for text commands (e.g., "go\n" for state changes)
                if byte == b'\n' || byte == b'\r' {
                    if let Some(cmd) = recognize_command(&buf[..idx.saturating_sub(1)]) {
                        state = handle_command(cmd, state);
                    }
                    idx = 0; // Reset buffer
                }
            }
            Ok(None) => {}
            Err(_) => {}
        }

        let vehicle = nav.update(&sensors::read(now));

        // Telemetry, never waiting for the UART: a packet is started at most every period_us, only once the
        // previous one has gone out; each loop sends what the UART accepts.
        if let Some(v) = vehicle {
            if tx_pos >= tx_len && now >= next_tx_us {
                if let Some(n) = encode_telemetry(&v, &mut tx) {
                    (tx_len, tx_pos) = (n, 0);
                }
                next_tx_us = now.saturating_add(CONFIG.telemetry_period_us);
            }
        }
        while let Some(&b) = tx.get(tx_pos).filter(|_| tx_pos < tx_len) {
            if !lpuart6.try_write(b) {
                break;
            }
            tx_pos += 1;
        }

        // Control loop - only when flying or landing
        if state == DroneState::Flying || state == DroneState::Landing {
            if let Some(cmd) = &latest_cmd {
                // Apply current control: thrust, gimbal angles, body rates
                let _thrust = cmd.thrust_seq[0];
                let _theta = cmd.gimbal_theta_seq[0];
                let _phi = cmd.gimbal_phi_seq[0];

                // TODO: Send to motors/servos
                // e.g., set_motor_thrust(thrust);
                // set_gimbal_angles(theta, phi);
                // apply_body_rates(rates);
            }
        }

        led.toggle();
        delay.block_ms(1);  // 1ms loop
    }
}

enum Command {
    Go,
    Land,
    Kill,
}

fn recognize_command(data: &[u8]) -> Option<Command> {
    match data {
        b"go" => Some(Command::Go),
        b"land" => Some(Command::Land),
        b"kill" => Some(Command::Kill),
        _ => None,
    }
}

fn handle_command(cmd: Command, current_state: DroneState) -> DroneState {
    match cmd {
        Command::Go if current_state == DroneState::Idle => DroneState::Flying,
        Command::Land if current_state == DroneState::Flying => DroneState::Landing,
        Command::Kill => DroneState::Idle,
        _ => current_state,
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
