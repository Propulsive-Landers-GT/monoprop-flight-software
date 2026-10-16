//! Monoprop drone firmware for the Teensy 4.1.
//!
//! Build and flash (from Monoprop/, so .cargo/config.toml adds the t4link.x linker script and the 64 KiB
//! stack). Always with --release: a debug build does not fit the Teensy's 192 KiB of ITCM and fails to link.
//!   cargo build -p Drone --release --target thumbv7em-none-eabihf
//!   rust-objcopy -O ihex target/thumbv7em-none-eabihf/release/Drone Drone.hex   (cargo-binutils / llvm-tools)
//!   teensy_loader_cli --mcu=TEENSY41 -w -v Drone.hex                           (or the Teensy Loader app)
//! For the first boot, add `--features stack-check` to see the stack high-water mark on the LED.
//!
//! Stack: 64 KiB (TEENSY4_STACK_SIZE in .cargo/config.toml). The Navigator's update chain needs about 28 KB
//! (checked by the stack_budget host test). NOTE for the team: trimming the 15x15 temporaries in
//! Esekf::update (David's code, later) would cut that a lot.
//!
//! Each 1 ms loop: read the clock, handle ground commands, step the Navigator, queue telemetry at about
//! 50 Hz, push queued bytes to the UART without waiting, drive the LED.
//!
//! NOTE: the sensors are stubs until drivers exist, so the Navigator never aligns and `go` is always
//! refused (three fast LED blinks).
#![no_std]
#![no_main]

#[allow(dead_code)] // kept until retired: the Navigator now provides the telemetry
mod sensor_fusion_crate;

use teensy4_bsp as bsp;
use bsp::board;
use bsp::hal::gpt;
use Common::GroundCommand;
use Drone::link::{TxQueue, WrapExtender};
use Drone::nav::{CommandOutcome, Command, FlightControl, NavLink, Sensors, armed};

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

    #[cfg(feature = "stack-check")]
    Drone::stack::paint();

    // Create LPUART peripheral for communication with ESP32
    // where pins.p1 = LPUART6_TX, pins.p0 = LPUART6_RX
    let mut lpuart6 = board::lpuart(lpuart6, pins.p1, pins.p0, 115200);

    // Create the LED peripheral. There is only one LED on the Teensy 4.1, connected to pin 13.
    let led = board::led(&mut gpio2, pins.p13);

    // GPT1: the 1 ms loop delay
    gpt1.set_clock_source(bsp::hal::gpt::ClockSource::PeripheralClock);
    let mut delay = bsp::hal::timer::Blocking::<_, 1_000_000>::from_gpt(gpt1);

    // GPT2: the free-running microsecond clock (PERCLK is 1 MHz, see board::PERCLK_FREQUENCY)
    gpt2.disable();
    gpt2.set_clock_source(gpt::ClockSource::PeripheralClock);
    gpt2.set_divider(1);
    gpt2.set_mode(gpt::Mode::FreeRunning);
    gpt2.set_reset_on_enable(true);
    gpt2.enable();
    let mut clock = WrapExtender::new();

    let mut buf = [0u8; 256];
    let mut idx = 0;
    let mut latest_cmd: Option<GroundCommand> = None; // Store latest control command
    const GROUND_COMMAND_SIZE: usize = core::mem::size_of::<GroundCommand>();

    // initialize sensors (if that's smth we need to do idk
    sensor_fusion_crate::uwb_init();
    sensor_fusion_crate::imu_init();

    let mut sensors = Sensors::stubs(); // TODO: drivers for the BMI323s, AK09918, VL53L1X and GNSS
    let mut nav = NavLink::new(); // in main's frame for the whole run (about 3 KB)
    let mut fc = FlightControl::new();
    let mut tx = TxQueue::new();
    #[cfg(feature = "stack-check")]
    let mut stack_kib: Option<(u64, u64)> = None; // (measured at, KiB used)

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
                        // go is accepted only with the Navigator OnPad; kill also restarts the Navigator
                        if fc.command(cmd, nav.startup(), now) == CommandOutcome::Killed {
                            nav.reset();
                        }
                    }
                    idx = 0; // Reset buffer
                }
            }
            Ok(None) => {}
            Err(_) => {}
        }

        // Navigator: this loop's new samples (none from the stubs), armed once launched
        let inputs = sensors.poll(now, armed(fc.state));
        nav.step(&inputs);

        // Telemetry at about 50 Hz once the Navigator has an estimate (from OnPad on, intended), never
        // waiting for the UART: a packet due while the previous one is still being sent is dropped
        if let Some(t) = nav.telemetry() {
            if fc.telemetry_rate.due(now) && !tx.load(&t) {
                fc.telemetry_drops = fc.telemetry_drops.saturating_add(1);
            }
        }
        tx.pump(|b| lpuart6.try_write(b));

        // Control loop - only when flying or landing
        if armed(fc.state) {
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

        // LED: heartbeat, or three fast blinks after a refused go
        #[allow(unused_mut)]
        let mut led_on = fc.led_on(now);
        #[cfg(feature = "stack-check")]
        {
            // PLACEHOLDER(choice): measure once, 10 s after boot, then show it on the LED instead
            if stack_kib.is_none() && now >= 10_000_000 {
                let used = u64::from(Drone::stack::high_water_bytes());
                stack_kib = Some((now, used.div_ceil(1024)));
            }
            if let Some((at, kib)) = stack_kib {
                led_on = Drone::nav::stack_code_led(now.saturating_sub(at), kib);
            }
        }
        if led_on { led.set() } else { led.clear() }

        delay.block_ms(1);  // 1ms loop
    }
}

fn recognize_command(data: &[u8]) -> Option<Command> {
    match data {
        b"go" => Some(Command::Go),
        b"land" => Some(Command::Land),
        b"kill" => Some(Command::Kill),
        _ => None,
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
