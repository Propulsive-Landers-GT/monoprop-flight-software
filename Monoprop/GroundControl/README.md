# Monoprop ground station

This is the ground half of the prototype flight loop:

BMI323 + VL53L1 -> Teensy navigation/FSM -> UART -> ESP32 -> UDP ->
ground Lossless reference planning -> nonlinear MPC -> 10-step actuator horizon ->
UDP -> ESP32 -> UART -> Teensy phase/interlock/actuator processing.

The navigation algorithm remains onboard. Ground uses its position, velocity,
quaternion and body rates directly. Lossless generates position/velocity
references; MPC tracks those references and sends thrust/thrust-vector commands.
There is no separate Lossless trajectory packet understood by this firmware.

## Build and start

Keep these repositories adjacent:

- monoprop-flight-software/
- monoprop-prototype-software/

The ground package imports the prototype's actual packet definitions through a
relative Cargo dependency. Tests also exercise its real command parser, navigator
and runtime. No Intel MKL, BLAS or Python dependency is needed by the ground app.

From monoprop-flight-software/Monoprop:

    cargo test -p GroundControl --release
    cargo run -p GroundControl --release

The default configuration is GroundControl/ground.toml (resolved relative to the
package, independent of the process's working directory). Override it or the UDP
addresses:

    cargo run -p GroundControl --release -- --config GroundControl/ground.toml --bind 0.0.0.0:9999 --bridge 192.168.1.50:8888

Use a release build for MPC. Runtime speed is enabled for GroundControl while
embedded packages retain the workspace's size optimization. Logs append to
ground-session.jsonl in the current directory. Each accepted telemetry sample,
operator command, trajectory, solver status and transmitted horizon is recorded.
Console status updates once a second. A busy UDP port is reported without killing
the process that owns it.

## ESP32 and wiring

Verified against monoprop-firmware revision
f3336ef4e977ad26b2dcb28cd0a847a896823c15,
[esp32Firmware/src/main.cpp](https://github.com/Propulsive-Landers-GT/monoprop-firmware/blob/f3336ef4e977ad26b2dcb28cd0a847a896823c15/esp32Firmware/src/main.cpp):

- ESP32 address 192.168.1.50, listening UDP port 8888.
- Ground destination 192.168.1.100, UDP port 9999 by default.
- The ESP32 has fixed ground addressing by default. Give the laptop that address
  on the field network, or rebuild the ESP32 with its actual address. Changing
  only the ground bind address cannot change the ESP32 destination.
- Configure the ESP32's field-router SSID/password before use.
- UART 115200 baud, 8N1, 3.3 V logic and common ground.
- Prototype Teensy TX **pin 14** -> ESP32 RX GPIO16.
- ESP32 TX GPIO17 -> prototype Teensy RX **pin 15**.
- The ESP32 source comment mentioning Teensy pins 0/1 describes an older wiring
  layout; those pins conflict with the prototype BMI323 SPI connection.
- Use the default MODE_DELIMITER packaging (zero-terminated postcard COBS).
  Idle-timeout/fixed-size packaging is not supported by this ground decoder.
- Allow inbound UDP 9999 in the ground computer's firewall for the field network.

The bridge is transparent: every command datagram contains one complete COBS
frame including its final zero. Telemetry can contain multiple complete frames;
truncated, malformed, oversized, nonfinite and invalid-quaternion frames are
rejected. Packets from other IP/port pairs are ignored. A lone zero sent on
startup and once per second lets dynamic-ground-IP builds learn our address; the
Teensy ignores it and it does not refresh its flight-command timeout.

## Prototype firmware

Build the actual flight binary, not an IMU/LiDAR smoke-test binary:

    cd monoprop-prototype-software
    cargo build-teensy

A built HEX is available at
monoprop-prototype-software/target/thumbv7em-none-eabihf/release/monoprop-teensy41-firmware.hex.
Regenerate it after any rebuild with llvm-objcopy -O ihex (see the prototype
README's loader instructions). Flash through your existing Teensy loader procedure. The integration selects
modern telemetry in teensy41-firmware/src/config.rs. It includes flight phase,
real sensor readings, navigation state and applied actuator commands. UART
transmission is incremental so it does not wait for an entire telemetry frame
before returning to navigation. Both directions remain at the ESP32's 115200 baud.

Legacy telemetry is still decoded for monitoring and solver previews. Live
streaming requires modern telemetry to observe the onboard flight phase.

Keep the vehicle stationary during initial alignment: the prototype requires
200 fresh stationary BMI323 samples and valid LiDAR ground samples (approximately
two seconds at 100 Hz). No navigation telemetry is fabricated before alignment.
If no packets arrive, check sensor readiness, the UART wiring, ESP32 diagnostic
counters, ground IP and firewall. The current driver supplies one BMI323 and a
VL53L1; it does not provide GPS or a magnetometer. Horizontal position and yaw
therefore drift without additional aiding. Ground does not invent missing sensors.

## Operator sequence

The program begins in telemetry-only mode. For the prototype tethered-hover
mission, enter each phase command and wait for the printed confirmation:

    mission hover
    ready
    arm
    solve mpc
    solve lossless
    lossless on
    mpc on
    launch

Both solve commands are previews and never transmit actuator horizons. Lossless
preview is optional. Live Lossless replanning runs during Ascent/Descent; Hover
uses the configured fixed hover reference, matching Lander's integration.
Use mission standard before ready for Ascent -> Hover -> Descent. The standard
mission's automatic timing/altitudes are still the prototype FlightLimits
(default 1 m hover, 10 s hover duration, 0.07 m landing threshold). Ascent aims
5 cm above the hover reference so it crosses the exact onboard transition
threshold; once Hover is observed the reference returns to 1 m. Descent targets
the home/pad altitude (0 m), crossing the 0.07 m touchdown threshold at low speed.
These margins avoid asymptotically hovering just short of a phase transition.
Keep ground targets consistent with onboard limits; the protocol cannot change limits.

Commands understood by the vehicle: ready, arm, disarm, launch (alias go), hover,
land (alias descent), abort, kill, mission standard and mission hover.
The ground never sends startup commands automatically. Launch requires fresh
Armed telemetry and enabled MPC. Land/hover/launch suspend horizon transmission
until the new phase is observed, preventing an old solve from following a phase
command. If confirmation does not arrive, inspect status and retry the command.

- land requests onboard Descent; ground follows the landing target and optional
  Lossless plan. The onboard FSM detects Landed and gates thrust to zero.
- kill/abort send the actual vehicle command and immediately disable ground
  streaming; the program stays open to monitor. Termination is latched onboard.
- mpc off stops future horizons; the Teensy may finish its existing buffered
  horizon until its command timeout. This is not an immediate motor-stop command.
- lossless off returns MPC to a fixed target reference.
- quit exits ground control; it does not issue a vehicle kill.
- reset clears timestamp ordering after a vehicle reboot and disables streaming.
- status and help print local information.

UDP send success means sent, not acknowledged. Modern phase telemetry confirms
phase transitions. The existing protocol has no explicit command acknowledgement,
mission-mode report or termination-reason field; the program does not claim those
are confirmed. Reissue a lost command if needed.

## Solvers, units and freshness

MPC uses a 13-state rigid-body model: [x,y,z,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz].
Modern vehicle quaternions [qw,qx,qy,qz] are reordered using the prototype's own
as_legacy conversion. Coordinates are launch-relative z-up, positions in meters,
rates in radians/second, gimbal angles in radians. The 10-step horizon uses the
prototype's fixed 0.1 s interval. Ground defaults to 20 solves/second.

PANOC solves a nonlinear box-constrained problem with RK4 propagation, warm
starting, a time budget and finite/feasibility/cost checks. The best feasible
iterate is retained because PANOC is nonmonotone; choosing it over the final
iterate is reported as BestFeasibleIterate. A time- or iteration-
limited feasible iterate is used only when its cost is no worse than the warm
start; its nonconverged status is logged explicitly. The packet carries the
current first action, followed by the remaining nine actions.

Lossless uses the same pinned rust_lossless solver as Lander, with a fixed
configurable duration and 30 intervals. This avoids the library's fuel-based
time search dividing by zero for electric propulsion (fuel_burn_kg_per_ns=0).
Its specific-force output is multiplied by trajectory mass to get newtons for
MPC's force reference. Position and velocity are interpolated on the vehicle's
clock, accounting for solve latency. Failure falls back to setpoint tracking
when no still-valid plan exists. Guidance and MPC run on separate workers so a
slow plan does not block operator commands or telemetry reception.

Only the packet boundary converts newtons into normalized ESC command through
the configurable thrust curve. The initial curve is linear and **assumed**.
Do not send newtons directly as normalized throttle.

Telemetry age uses the ground monotonic clock and requires advancing vehicle
timestamps: repeated cached estimates do not refresh it. Ground stops streaming
after 250 ms without new navigation, drops MPC results older than 100 ms, and
discards pending results after stop/kill/phase/solver-mode changes. Streaming
requires explicit re-enabling after a telemetry timeout. The onboard 1 s
horizon timeout and navigation/tilt interlocks remain authoritative.

## Provisional model and validation limits

Per the requested lightweight-UAV assumptions, ground.toml starts with 0.7 kg,
12 N combined maximum thrust, inertia [0.008,0.008,0.0035] kg*m^2 and a thrust
pivot 0.12 m below COM. It uses 15 degree per-axis gimbal limits and constant mass.
These values and controller weights are not measured or flight tuned.
The linear curve predicts roughly 57% hover throttle; actual propeller thrust
versus ESC command is not generally linear. Replace the curve and model from
bench measurements before relying on free-flight performance.

Automated tests cover real parser/codec compatibility, navigation alignment,
startup/interlocks, stale/reordered data, solver physics and responses, Lossless
ascent/descent, and the real ground executable over localhost UDP with the
prototype runtime. They do not establish Wi-Fi reliability, hardware timing,
sensor mounting signs, propulsion calibration, or closed-loop flight stability.
