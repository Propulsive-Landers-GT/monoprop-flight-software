# Monoprop ground MPC

This executable runs the existing GroundControl PANOC MPC against modern Teensy
`TelemetryFrame` messages and sends shared `ControlCommand` packets through the
ESP32. Both directions use Postcard + COBS. The wire uses normalized ESC throttle;
the solver uses newtons. Conversion uses the configured combined thrust curve (currently the user-assumed linear ramp).

The previous program never activated in response to a Teensy Launch: it required
its own `go` input. It also expected raw legacy telemetry, sent raw legacy commands
to a hardcoded IP, and modeled an 80 kg rocket with 300–1000 N control bounds.
Those values must never be sent as normalized throttle. The ESP32 is a relay;
Launch does not start a process on the PC.

## Build and observe

From this directory:

```sh
cargo build --release -p GroundControl
../target/release/GroundControl --help
../target/release/GroundControl --target <ESP32-IP>:8888 --observe
```

Use the ESP32's current IP, not the computer IP. This program owns UDP 9999;
close the existing telemetry receiver before starting it. It never kills another
process to acquire a port. Run it on the machine/interface receiving ESP32 UDP;
WSL requires the same UDP routing/firewall arrangement as the telemetry monitor.
USB on the Teensy is not used. The console reports `udp_received`, `rejected`,
and `wrong_peer` counters even before a valid telemetry frame arrives. Zero
received packets means there is no UDP input at this listener; increasing rejected
packets indicate framing/schema errors. On the ESP32 USB diagnostic console,
`uartRx=0` means no Teensy UART bytes have reached the bridge, while `tx=0` means
no telemetry datagrams have been sent. The source firmware publishes navigation
telemetry only after IMU alignment; absence of telemetry alone therefore cannot
distinguish missing UART wiring from incomplete sensor initialization/alignment. `--observe` emits no network packets, does not
run the solver, and allows observation without validating the model. It prints phase, link and
telemetry age. `quit` exits.

## Required calibration

`ground.toml` now contains the user-specified provisional cylinder and linear-thrust
model: mass 1.05 kg, height 0.5715 m, diameter 0.1524 m, lever arm 0.18415 m,
Ixx=Iyy=0.030102762375 kg m² and Izz=0.003048381 kg m². The central moments
of a uniform solid cylinder are used as a rough estimate about the actual CoM.
The stated CoM is 0.1016 m below the uniform cylinder center; dimensions and CoM
alone cannot determine the nonuniform vehicle inertia. A parallel-axis shift of
the uniform cylinder would describe a non-CoM reference axis, not resolve this
mass-distribution uncertainty.

The assumed running thrust curve is F=16*(u-0.15)/0.85 N, so 16 N maps to full
normalized throttle and 10.3005 N hover force maps to approximately 69.72% throttle.
The 15% endpoint assumes zero thrust even though the motors spin there. This is
a user-requested approximation, not a measured calibration. Stopped/disarmed output
is still normalized command zero through the firmware actuator gate.

Replace these estimates with validated mass, diagonal principal inertia in kg m², thrust lever arm
in meters, and measured pairs of `[normalized_ESC_command, combined_thrust_N]`.
The model assumes body z up, thrust applied below CoM on body -z, and a diagonal
inertia tensor in the body frame. If CAD does not satisfy these assumptions,
change the model before flight. Verify the correspondence between solver theta/phi
and the firmware's X/Y TVC commands; `gimbal_sign` handles signs only, not axis swaps.
Minimum running throttle is currently 0.15. The curve must span hover force and
must be measured with both motors and the intended propellers/supply. Its upper
point is the maximum controller throttle; choose only a validated operating range.
The current linear ramp is an explicit assumption; maximum thrust alone does not
validate that relationship.

The supplied model contains provisional estimates. The manual
`hardware_validated` flag has been removed at the operator's request; `--send`
uses the configured model without that attestation. Numerical parameter checks,
fresh telemetry, solver convergence/deadlines,
phase gates and actuator limits remain enforced. The configured estimates and
successful software checks do not establish real flight stability. Existing MPC cost weights are
starting values, not flight-validated monoprop tuning. The simplified model has no
motor lag, yaw torque allocation, or aerodynamic model; those limitations need
simulation/bench evaluation. MPC always uses the position and velocity received in telemetry. There is no
GPS prerequisite or position-source setting. GPS still corrects the onboard EKF
when available; without those corrections inertial position can drift.

## Compute without transmitting

With the provisional parameters supplied, omit `--send` and `--observe`:

```sh
../target/release/GroundControl --target <ESP32-IP>:8888
```

Type `go` to queue computation, or start with `--compute` to queue it automatically.
It waits for a valid fresh telemetry packet before starting. `--compute` cannot
be combined with `--send` or `--observe`; `stop` cancels a queued start. Staleness,
a drone restart, or a solver failure still deactivates computation and requires
`go` again; this is not automatic flight recovery. With fresh telemetry the solver prints predicted normalized throttle,
gimbal commands and runtime; nothing is transmitted. Confirm performance against
the 150 ms receive-to-command deadline and 250 ms Teensy single-command timeout.
No fixed hover throttle is guessed. No automatic commands are generated at startup.

## Live operation after validation

```sh
../target/release/GroundControl --target <ESP32-IP>:8888 --send
```

Use one operator console. `ready`, `arm`, then `launch`, waiting for each requested
phase in telemetry. `launch` checks a solve, selects MissionTetheredHover, enables
the ground MPC, and sends Launch. Control starts only after Hover telemetry. For
an existing flight, `go` enables streaming; `hover` requests Hover and `land`
requests Descent. Phase changes suspend output until telemetry confirms the phase;
missing confirmation deactivates the controller. Submission is not an actuator ACK.
Monitor `link=true` and actual actuator telemetry as well as computed outputs.

The command cycle is 20 ms with the latest received state, using the existing
10-step, 0.1 s prediction horizon. The first optimized action is sent, then the
horizon is shifted for the next solve. Both optimization and firmware enforce the
shared 15 degree polar TVC limit. Solver errors, nonconvergence, stale telemetry,
drone restart, or missed deadlines stop transmission; the existing
Teensy timeout then clears control and requests Descent. That fallback is **not**
an autonomous controlled landing: no external stream means zero actuator demand.
`stop`/`quit` likewise leave the firmware timeout in effect. `kill` submits the
shared Kill command and stops the stream. Restarting requires explicit operator
activation. The current telemetry schema has no flight-termination reason or
command acknowledgement field, which limits diagnosis from ground alone.

## Software checks

```sh
cargo test --release -p GroundControl
```

Tests use synthetic model values, never hardware. They cover modern framing,
quaternion conversion, calibration rejection, force-to-throttle conversion,
hover equilibrium/vertical response, prediction-matrix propagation and the polar
TVC constraint. They do not establish real flight stability.

## Live TVC checkout without motor thrust

Start with `--send`, wait for fresh Standby telemetry, then enter `ready`. Once
telemetry confirms Ready, `tvc 1 0` sends a one-degree X command at 50 Hz;
`tvc 0 1` checks Y and `tvc 0 0` commands calibrated neutral. Angles are degrees
relative to the compiled trim. All checkout packets have zero thrust and zero
body-rate command. The shared 15-degree polar limit applies. GPS and a calibrated
thrust model are not required for this Ready-only test; the firmware also gates
motor output to zero in Ready. Live feedback prints the Teensy's reported
actuator command, which confirms firmware application, not measured servo motion.
`stop` stops the stream and the firmware timeout returns TVC to neutral;
`disarm` returns Ready to Standby. Stale telemetry or leaving Ready cancels checkout.
This does not enable the autonomous MPC: `arm`, `launch`, and live `go` retain the
fresh-navigation, solver and phase gates.

## Solver convergence and an origin check

`NotConvergedIterations` means the solver exhausted its iteration allowance without
satisfying its existing 1e-4 fixed-point-residual tolerance. Candidate numerical
controls are not accepted as successful solutions. The limit is now 1000 iterations
instead of 150; the 100 ms solver deadline and 150 ms receive-to-send deadline remain.
Failures include iteration count, residual and solve time in the diagnostic message.

The offline example reuses the production controller and plant model; it opens no
network sockets and does not command the vehicle:

```sh
cargo run --release --example check_origin
# Optionally replay one captured modern Postcard/COBS telemetry datagram:
cargo run --release --example check_origin -- /path/to/telemetry.bin
```

It compares a held origin assumption with feedback from the ideal plant at 50 Hz,
with and without initial tilt, and optionally repeats those checks from captured
telemetry. A held starting-position assumption commands full thrust indefinitely
for the 1 m target. The ideal closed loop reaching the target is not hardware flight
validation: live navigation must track actual position after initialization. Do
not overwrite live position with zero to hide an origin/alignment error.

## Offline return trajectory from incoming telemetry

`plan_return` accepts a captured modern Postcard/COBS telemetry packet, keeps its
reported position, velocity and attitude, and treats the phase as Descent only in
the calculation. A smooth quintic reference returns to local [0,0,0], preserves
initial velocity, ends at zero velocity/acceleration, and uses initial yaw as its
heading reference. Nominal duration is at least 20 s and scales with distance for
about 0.5 m/s peak reference speed from rest. Actual initial velocity can change
that speed; the rollout reports the resulting motion. The production MPC tracks
the full reference horizon through `Controller::solve`. Live Descent uses this
same reference generator, initialized from the first Descent telemetry packet.
The reference advances on the Teensy timestamp; each solve uses the newest received
state. It is not restarted on each packet. Leaving Descent clears the reference;
explicit `go` restarts it from the current state.

```sh
cargo run --release --example plan_return -- /path/to/telemetry.bin return.csv
python3 tools/plot-return.py return.csv return.png
```

The rollout uses the same assumed model as the optimizer at 50 Hz. It stops on
solver/deadline failure, the firmware's 30-degree tilt limit, the firmware's landing
predicate, or the duration limit. CSV rows contain predicted state, ESC throttle,
TVC angles, modeled force and solve timing. Terminal position and outcome print to
the console. The landing predicate includes a 7 cm height threshold, so the final
position is near rather than exactly at [0,0,0]. All results are predictions from
a snapshot, not continued live feedback or proof of real-flight accuracy. These
examples never create a socket, transmit a command, or alter the hardware phase.

## Resetting on the pad

`kill` stops streaming, cuts thrust, clears the termination latch and returns to
Standby. `disarm` returns Ready/Armed to Standby without resetting navigation.
In Standby, `reset-ekf` requests fresh alignment and a new GPS origin; wait for
new navigation telemetry before `ready`, `arm`, `launch`. Resetting the EKF is
not permitted during flight. None of these commands is an automatic landing.

## GPS events across telemetry decimation

The Teensy keeps the latest prepared local GPS fix until a telemetry frame is
successfully encoded and queued. It emits the fix only once and only within
100 ms of the receiver receipt timestamp, discarding expired/future-dated fixes.
Cached fixes are never fed back into the EKF. This preserves the existing Postcard
schema and ESP32 byte relay; no ESP32 reflash is needed. The packet has no precise
GPS-age field: this is a bounded-age event, not a repeatedly refreshed cache.

## Separate incoming-telemetry terminal

GroundControl can copy accepted Postcard/COBS telemetry to a loopback receiver:
`--telemetry-mirror 127.0.0.1:10000`. The primary receiver remains on Wi-Fi port
9999; do not bind a second viewer there. Copies are nonblocking and best effort,
so closing the viewer does not stop MPC. Only validated packets from the configured
ESP32 peer are copied. No new ESP32 or Teensy firmware is needed.

On Windows, launch `tools/view-telemetry.ps1` for a five-updates-per-second view of
incoming position, velocity, attitude, phase and reported actuator commands.
Pass `-TelemetryMirror 127.0.0.1:10000` to `tools/run-live.ps1` when starting the
controller. The viewer alone opens port 10000 but waits until a controller with
mirroring enabled is running. Source addresses in mirrored packets identify the
local forwarding socket; the configured ESP32 is validated before copying.
