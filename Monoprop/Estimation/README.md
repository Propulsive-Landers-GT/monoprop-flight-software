# Estimation

The bare-metal error-state Kalman filter (ES-EKF) for the monoprop vehicle: it fuses the IMU with position
fixes (UWB or GPS), a magnetometer and height (barometer or lidar), and estimates position, velocity,
attitude, accelerometer bias and gyro bias. `#![no_std]`, no heap, f64, nalgebra fixed-size matrices
(same version as `Drone`).

The filter code is the same as `AttitudeEstimation/rust-ekf/src/bare_es_ekf/` in the navigation repository,
where it serves as the template for new estimators; here its modules sit at the crate root.

| File | What it is |
|---|---|
| `src/math.rs` | Linear algebra and rotation helpers. |
| `src/model.rs` | `BareESEKFModel`: what a vehicle/sensor model provides. |
| `src/filter.rs` | `BareESEKF<Model, E>`: predict, gated update, injection, covariance reset. |
| `src/full_state.rs` | `FullState`: the 15-state INS model (IMU, position, magnetometer, height), TRIAD alignment, VN-200 noise, environment constants. |
| `src/interop.rs` | f32 conversions for the Drone software and `Common::DroneTelemetry`. |
| `src/system_test.rs` | The system test: simulated flights with faults (`cargo test -p Estimation`). |

## Build and test (from `Monoprop/`)

```text
cargo test -p Estimation -- --nocapture                    # the system test, on the PC (about a minute)
cargo build -p Estimation --lib --target thumbv7em-none-eabihf
```

## Using it

```rust
use Estimation::*;

// IMU noise: the VN-200 datasheet model at its sample rate (replace with Allan-deviation values later)
let model = FullState::new(ImuSpec::vn200(200.0));

// start-up, from readings taken with the vehicle still. None: the readings can't define an attitude
// (e.g. magnetometer parallel to gravity, a dead sensor): try again with the next ones.
let Some(mut filter) = FullStateESEKF::align(model, &accel0, &mag0, &pos0, SIGMA_POS) else { return };

// every IMU sample; dt from the IMU timestamps. false = refused (NaN reading or bad dt), nothing changed.
let ok = filter.predict(&ImuSample { accel, gyro }, dt);

// when a measurement arrives: UpdateOutcome::Fused { nis } or a Rejected... variant
let outcome = filter.update_position(&position, SIGMA_POS); // GPS or UWB, navigation frame
let outcome = filter.update_mag(&mag, SIGMA_MAG);           // any unit: only the direction is used
let outcome = filter.update_height(height, SIGMA_HEIGHT);   // barometer or downward lidar

// the estimate
let x = filter.get_state(); // x.p, x.v, x.q (body -> navigation), x.ab, x.wb; covariance: filter.get_covariance()
```

Before flight: set gravity, the magnetic field (navigation frame) and the IMU noise in `src/full_state.rs`.
