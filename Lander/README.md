# Lander Flight Software

This directory contains the flight control loop, navigation estimation, and guidance tracking algorithms for the Lander. The software is written in Rust and is designed to run in real-time on the onboard flight computer.

---

## Software Architecture

The flight software is structured into four distinct, modular components designed to run within a high-frequency real-time loop:

```
+-------------------------------------------------------------------------+
|                                                                         |
|                             Flight Computer                             |
|                                                                         |
|     +------------------+                   +------------------+         |
|     |                  |                   |                  |         |
|     |   Sensors (IMU)  |                   |   Operator (CLI) |         |
|     |                  |                   |                  |         |
|     +--------+---------+                   +--------+---------+         |
|              |                                      |                   |
|              | 500 Hz Telemetry                     | CLI Commands      |
|              v                                      v                   |
|     +--------+--------------------------------------+---------+         |
|     |                                                         |         |
|     |                 Flight State Machine                    |         |
|     |           (Path determination & transitions)            |         |
|     |                                                         |         |
|     +---+----------------------+--------------------------+---+         |
|         |                      |                          |             |
|         | Rates                | Fused State / Traj       | Step        |
|         v                      v                          v             |
|     +---+------+       +-------+--------+         +-------+-------+     |
|     |          |       |                |         |               |     |
|     |          |       |   Autopilot    |         |   Actuator    |     |
|     |          |       |  - Navigator   |         |  Controller   |     |
|     |Scheduler |       |  - Guidance    |         | - RCS Control |     |
|     |          |       |  - MPC         |         | - Slew/Clamp  |     |
|     |          |       |                |         |               |     |
|     +----------+       +-------+--------+         +-------+-------+     |
|                                |                          |             |
|                                | Path Reference           | TVC / RCS   |
|                                v                          v             |
|                        +-------+--------------------------+-------+     |
|                        |                MCAP Logger               |     |
|                        +------------------------------------------+     |
+-------------------------------------------------------------------------+
```

### 1. Flight State Manager (FSM Coordinator)
*   **Orchestration**: Directs the main loop execution, handles checking safety boundaries, and manages state transitions.
*   **Decoupled State Transitions**: Implements a pure path determination function (`next_phase`) to return the target phase, and a unified state modification hook (`on_transition`) to execute transitions safely.

### 2. Scheduler
*   **Rate Management**: Tracks monotonic mission time and decides when tasks are due to run:
    *   **Navigation Updates (500 Hz)**: Triggers sensor estimation updates.
    *   **Guidance Planner (1 Hz)**: Triggers trajectory re-generation.
    *   **MPC Tracking Control (50 Hz)**: Triggers optimization controller updates.

### 3. Autopilot
*   **Navigator (EKF)**: Fuses VN-200 IMU, GPS, and UWB telemetry inside a 15-state Error-State Kalman Filter (ES-EKF) to estimate vehicle position, velocity, attitude (quaternion), and sensor biases.
*   **Guidance (Lossless)**: Dynamically plans fuel-optimal reference trajectories using lossless convexification algorithms.
*   **MPC Controller (PANOC)**: Tracks the reference trajectory using a Non-linear Model Predictive Control optimization solver.

### 4. Actuator Controller
*   **Roll Control**: Runs the reaction control system (RCS) controller to regulate vehicle roll.
*   **Slew-Rate Clamping**: Limits maximum change rate on gimbal and thrust command signals to prevent actuator slamming and protect flight hardware.

---

## Flight Phases & Transitions

The Lander transitions through the following discrete flight phases:
1.  **`Standby`**: Resting on the launch pad. Trajectory generation is pre-calculated. Thrust is locked to zero. Awaiting manual operator arm command.
2.  **`Armed`**: Safety interlocks removed. Actuators are ready. Awaiting manual operator launch command.
3.  **`Ascent`**: Thrust is active. The vehicle climbs to the target altitude (50m) tracking the lossless convex trajectory.
4.  **`Hover`**: Holds target hover position for 20 seconds.
5.  **`Descent`**: Triggers a guided descent trajectory to perform a controlled decelerating landing back at the pad.
6.  **`Landed`**: Touchdown detected (altitude z <= 0.1m and estimated speed ||v|| < 0.2m/s). Thrust is disabled, controls zeroed, and logging is finalized.

---

## Actuator Safety & Fallbacks

*   **NaN / Infinity Check**: Instantly detects optimization failures or numerical errors in the MPC output and falls back to the last valid control commands.
*   **Actuator Clamping Limits**:
    *   **Gimbal Servos**: TVC angles are rate-limited to $100^\circ/\text{s}$ (max $2^\circ$ change per 20ms step).
    *   **Throttle Valve**: Thrust change rates are limited to $2000\,\text{N}/\text{s}$ (max $40\,\text{N}$ change per 20ms step).
*   **Thrust Cost Penalty**: The MPC cost formulation penalizes high-frequency thrust changes to prevent actuator oscillations.

---

## Real-Time Execution & Safety Contingencies

*   **Real-Time Priority**: Thread scheduling on Linux targets is configured to `SCHED_FIFO` with a priority of `80` to minimize loop execution jitter.
*   **GPS/UWB Denial Emergency Landing**: If absolute position data (GPS/UWB) is lost for more than 5 seconds during active flight, the FSM transitions directly to `Descent` to attempt an emergency soft landing using EKF dead reckoning. If absolute position denial exceeds 15 seconds, a hard safety flight termination is triggered.
*   **Decoupled Master Timing**: Governed by a dedicated monotonic `Clock` struct owned by the FSM loop, ensuring timing references are independent of logging or OS scheduling anomalies.

---

## Interactive Command Console

The flight computer binary runs a non-blocking console interface. Operators can type commands directly into the terminal to transition the flight phase:
*   `arm`: Arms the vehicle (Standby $\rightarrow$ Armed).
*   `disarm`: Disarms the vehicle back to Standby (Armed $\rightarrow$ Standby).
*   `launch`: Triggers liftoff (Armed $\rightarrow$ Ascent).
*   `abort`: Terminates the flight in any phase (controls zeroed, loop stops). In flight this cuts thrust.

---

## Ground Station Link

The flight binary also talks to the GTPL ground station (`ground-station` repo) over UDP using the shared `gs-protocol` crate. The code lives in `src/telemetry.rs` (`GroundLink`) and is reused unchanged by the simulator's `--ground-station` mode, so the GUI exercises the same code against the sim and the vehicle.

*   **Ports**: the vehicle binds UDP `8888` (override with the first CLI argument or the `GS_PORT` environment variable); the ground bridge binds `9999`. Telemetry is sent to whoever last sent a valid packet (the bridge heartbeats at 2 Hz), so nothing is configured on the vehicle. If the port cannot be bound, a warning is printed and the vehicle runs without telemetry.
*   **Downlink**: `Flight` at 50 Hz (state estimate, actuation, tilt / trajectory-deviation margins, sensor health, link age), `Stand` at 20 Hz (chamber pressure as E-PT, tank pressure as O-PT, valve states), `Trajectory` whenever guidance replans, `Event` for every FSM diagnostic, `Ack` / `Params` on demand.
*   **Commands** (all acked `Accepted` / `Rejected(reason)` except `Heartbeat` and `Jog`; the vehicle is the authority on interlocks):

    | Command | Accepted when |
    |---|---|
    | `Arm` | Standby, control mode Auto |
    | `Disarm`, `Launch` | Armed |
    | `Abort` | always (flight terminated, reason "Operator abort") |
    | `SetPhase(Hover \| Descent)` | Ascent, Hover or Descent. `Hover` re-enters Hover for one hover duration |
    | `SetFlightParams` | always; range-checked (hover altitude 1-200 m, hover duration 0-120 s, max tilt 5-60 deg, max trajectory deviation 1-50 m) |
    | `SetMpcWeights` | always; diagonals of Q/R/QN, finite and non-negative. `None` restores the built-in per-phase weights |
    | `SetControlMode(Jog)` | Standby only; any phase change forces Auto |
    | `Jog` | control mode Jog; clamped to gimbal +/-15 deg, thrust 0-1200 N; expires after 0.5 s without refresh |
    | `SetValve` | Standby only. Stored in `valve_overrides` and echoed in `Stand` telemetry; nothing actuates valves yet |

*   **Running with the GUI**: start the bridge from the `ground-system` repo (`cargo run --release --bin gs-bridge`, UI on http://localhost:8080, see its README), then `cargo run` here (or `cargo run -- 8890` for another port). The GUI connects as soon as the bridge's first heartbeat arrives.

---

## Telemetry Logging (MCAP)

During flight, the computer writes real-time telemetry into `flight_log_*.mcap` files using the standard MCAP robotics container format.
*   **Asynchronous Logging**: Disk writes and JSON serialization run on a dedicated worker thread decoupled from the 500 Hz control loop via a lock-free channel.
*   **Visualizing**: Log files can be directly dragged and dropped into visualization platforms like **Foxglove Studio**.
*   **Logged Channels**:
    *   `telemetry/sensor_data` (500 Hz): Raw IMU, GPS, UWB, and pressure readings.
    *   `telemetry/vehicle_state` (500 Hz): Fused position, velocity, attitude (Euler & quaternion), angular velocity, and mass.
    *   `telemetry/control_output` (50 Hz / Event-driven): TVC gimbal angles, thrust, and RCS command inputs.
    *   `telemetry/flight_phase` (On-transition): Recorded only on flight phase transition updates to save disk space.
    *   `telemetry/diagnostics` (Event-driven): Phase transitions, solver alerts, fallbacks, and flight termination causes.

---

## How to Build and Run

1.  **Compile the code**:
    ```bash
    cargo build
    ```
2.  **Run the FSM console wrapper**:
    ```bash
    cargo run
    ```
3.  **Commanding**: Type `arm` followed by `launch` in the terminal to takeoff.
