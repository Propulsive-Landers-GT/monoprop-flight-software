# Monoprop ground station

This is the ground half of the prototype flight loop:

BMI323 + M10 GPS (+ enabled optional sensors) -> Teensy navigation/FSM ->
UART -> ESP32 -> UDP ->
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
- monoprop-firmware/ (ESP32 bridge firmware)

The ground package imports the prototype's actual packet definitions through a
relative Cargo dependency. Tests also exercise its real command parser, navigator
and runtime. No Intel MKL, BLAS or Python dependency is needed by the ground app.

From monoprop-flight-software/Monoprop:

    cargo test -p GroundControl --release
    $espAddress = Read-Host "ESP32 IPv4 address printed on USB serial"
    cargo run -p GroundControl --release -- --bridge "${espAddress}:8888"

The default configuration is GroundControl/ground.toml (resolved relative to the
package, independent of the process's working directory). The saved bridge
address must match the ESP32's current DHCP address. Update `ground.toml` or
override it with `--bridge` if that address changes. To also override the bind address:

    cargo run -p GroundControl --release -- --config GroundControl/ground.toml --bind 0.0.0.0:9999 --bridge "${espAddress}:8888"

Use a release build for MPC. Runtime speed is enabled for GroundControl while
embedded packages retain the workspace's size optimization. Logs append to
ground-session.jsonl in the current directory. Each accepted telemetry sample,
operator command, trajectory, solver status and transmitted horizon is recorded.
Console status updates once a second. A busy UDP port is reported without killing
the process that owns it.

## ESP32 and wiring

The local [ESP32 bridge firmware](../../../monoprop-firmware/esp32Firmware/src/main.cpp)
and prototype Teensy firmware use the following configuration:

- ESP32 uses DHCP on the connected router and listens on UDP 8888. Read its
  numeric address from `[WiFi] connected IP=...` on USB serial at 115200 baud.
- Ground stays on its normal LAN address and listens on UDP 9999. The ESP32
  learns that address from the source IP of incoming ground packets.
- Connect both devices to the same LAN. GroundControl's startup and periodic
  delimiter heartbeat establishes the return address without arming. No
  telemetry is forwarded until the ESP32 has received a ground packet.
- Pass the assigned ESP32 address with `--bridge IP:8888`. If its DHCP address
  changes, restart GroundControl with the new address. Run only the intended
  ground sender because the latest accepted sender sets the return address.
- Configure the ESP32's field-router SSID/password before use.
- UART 115200 baud, 8N1, 3.3 V logic and common ground.
- Prototype Teensy TX **pin 20** -> ESP32 header **RX (GPIO16)**.
- ESP32 header **TX (GPIO17)** -> prototype Teensy RX **pin 22**.
- The full Teensy flight binary receives on pin 22 and transmits on pin 20 using
  separate FlexIO3 shifters/timers; `LPUART8` and pin 21 are unused.
  LiDAR must remain disabled because its former XSHUT pin is now ESP32 RX.
- Use the full Teensy `monoprop-flight-rx22-tx20.hex` image. An existing RX22/TX20
  Teensy build needs no reflash for the ESP32 header change; its startup message
  may still name the old ESP GPIO14/15 wiring. Build and flash the ESP32 configured
  for RX16/TX17 using environment `featheresp32`; `featheresp32-echo` disables
  Teensy UART forwarding.
- Use the default MODE_DELIMITER packaging (zero-terminated postcard COBS).
  Idle-timeout/fixed-size packaging is not supported by this ground decoder.
- Allow inbound UDP 9999 in the ground computer's firewall for the field network.

The bridge is transparent: every command datagram contains one complete COBS
frame including its final zero. Telemetry can contain multiple complete frames;
truncated, malformed, oversized, nonfinite and invalid-quaternion frames are
rejected. Packets from other IP/port pairs are ignored. A lone zero sent on
startup and once per second lets the bridge learn our address; the
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

Keep the vehicle stationary during initial alignment: the current prototype
requires 200 fresh stationary BMI323 samples (approximately two seconds at 100 Hz).
Its compiled configuration enables the Holybro M10 GPS and disables LiDAR and
magnetometer updates. If LiDAR is enabled, alignment also requires valid ground
range samples. No navigation telemetry is fabricated before alignment.

With GPS enabled, launch additionally requires a valid GPS origin and a fresh
accepted fix; IMU alignment alone does not authorize launch. GPS TX connects to
Teensy p28 and GPS RX to p29 (`LPUART7`), independently of the ESP32 link on
p20 TX / p22 RX (FlexIO3). Navigation and telemetry use GPS position and velocity in
the launch-relative frame, not raw latitude/longitude. Heading starts at the
configured launch yaw while magnetometer aiding is disabled.

The Teensy's USB `ESP link` diagnostic reports received bytes and decoded commands
separately. The delimiter heartbeat should increment received bytes without
decoding a command. This verifies the ESP32-to-Teensy direction; telemetry at
the ground station verifies the reverse direction. The diagnostic should show
`rx_pin=22 tx_pin=20`; the corrected mapping still needs an on-board test after
installing the new Teensy build.

If no packets arrive, check sensor readiness, UART wiring, ESP32 diagnostic
counters, ground IP and firewall. If telemetry arrives but launch remains Armed,
check GPS readiness and IMU freshness in the Teensy diagnostics. Ground does
not fabricate missing sensors or bypass the onboard launch checks.

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
land (alias descent), abort, kill, mission standard, mission hover and ekf reset.
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
- ekf reset requests onboard navigation reinitialization. GroundControl disables
  MPC/Lossless, discards cached telemetry/plans and invalidates outstanding solver
  results. Teensy accepts only in Standby/Auto with no latched termination; it
  clears the EKF, learned biases/noise, GPS origin and queued control horizons.
  Mission selection and the hardware clock are preserved. Keep the vehicle
  stationary while fresh alignment and GPS-origin initialization run. Telemetry
  pauses until alignment completes. This command does not change noise or
  stationary thresholds, so it does not fix a failed stationary check by itself.
  Teensy USB prints `EKF reset accepted` or `EKF reset rejected`; GroundControl
  reports only that it sent the request, with no phase-based acknowledgement.
  Missing/stale telemetry permits sending a recovery request; the onboard gate
  remains authoritative. Both GroundControl and Teensy need this update; the
  transparent ESP32 bridge needs no reflash.
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

ground.toml uses the reported approximate flight mass of 1.05 kg,
16 N user-estimated combined maximum thrust and a thrust
pivot 0.12 m below COM. It uses 15 degree per-axis gimbal limits and constant mass.
Dry mass also equals 1.05 kg, matching the onboard electric-vehicle configuration.
For this Monoprop, inertia assumes a uniform solid cylinder of height 22.5 inches
(0.5715 m), diameter 6 inches (radius 0.0762 m), and mass 1.05 kg. About its center
of mass, with body Z along the cylinder axis:

- Ixx = Iyy = m*(3*r^2 + h^2)/12 = 0.030102762375 kg*m^2.
- Izz = m*r^2/2 = 0.003048381 kg*m^2.
- Products of inertia are zero by symmetry.

These are the diagonal body-frame moments used by MPC, not moments about the TVC
pivot. They are calculated from the requested uniform-cylinder assumption; other
model parameters and controller weights remain provisional.
The linear curve predicts about 10.3 N hover thrust and roughly 64% hover throttle; actual propeller thrust
versus ESC command is not generally linear. Replace the curve and model from
bench measurements before relying on free-flight performance.

Automated tests cover real parser/codec compatibility, navigation alignment,
startup/interlocks, stale/reordered data, solver physics and responses, Lossless
ascent/descent, and the real ground executable over localhost UDP with the
prototype runtime. They do not establish Wi-Fi reliability, hardware timing,
sensor mounting signs, propulsion calibration, or closed-loop flight stability.
