# monoprop-flight-software

The **Monoprop-specific integrated flight software** — the real-time stack that
wires the vehicle-agnostic GN&C libraries into a running vehicle. Unlike the
[`navigation`](../navigation) / [`control`](../control) / [`guidance`](../guidance) /
[`simulations`](../simulations) libraries, this repo is *not* plug-and-play: it is the
Monoprop vehicle integration.

## Contents

| Path | What it is |
|------|------------|
| `ControlLoop/` | The integrated real-time control loop (C++): `Loop/`, `LQR/`, `Filters/` (EKFs, Madgwick), `CustomLinear/`, `IntegratedDynModel/`, bundled `lapack/`, `Data/` + `data_creators/`, CMake/Make build |
| `Lander/` | Rust flight software: `src/algorithms/` (control, guidance, navigator, rcs) + `src/fsm/` (flight state machine) |
| `Monoprop/` | Rust Cargo workspace: `Common/`, `Drone/` (onboard), `GroundControl/` (ground station + MPC) |
| `ControlLoopMPC/` | MPC control-loop design notes |
| `docs/` | Project docs, including the team Google Drive links |

> **Note:** `ControlLoop/` contains integrated *runtime* copies of LQR, EKF, and the
> `CustomLinear` math that also appear as standalone designs in the library repos.
> They are kept here as part of this build unit; deduplicating them into shared
> dependencies is a good future refactor.

## Related repos

Libraries this integrates: [`navigation`](../navigation), [`control`](../control), [`guidance`](../guidance), [`simulations`](../simulations).
Hardware it runs on: [`monoprop-firmware`](../monoprop-firmware), [`monoprop-electronics`](../monoprop-electronics).

## Prototype ground station

See [Monoprop/GroundControl/README.md](Monoprop/GroundControl/README.md) for the
ESP32/Teensy connection, startup commands, provisional vehicle configuration,
Lossless/MPC integration and tests. Keep monoprop-prototype-software adjacent
to this repository; ground imports its exact shared packet definitions.
