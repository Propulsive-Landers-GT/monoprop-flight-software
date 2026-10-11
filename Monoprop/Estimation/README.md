# Estimation

A 15-state error-state Kalman filter (ESEKF) for the monoprop vehicle: it fuses the IMU with position fixes (UWB or GPS), a magnetometer and height (barometer or lidar), and estimates position, velocity, attitude, accelerometer bias and gyro bias.
`#![no_std]`, no heap, f64 throughout (the Teensy 4.1's Cortex-M7 has a double-precision FPU), built on
nalgebra fixed-size matrices (same version as `Drone`).

## Layout (dependency order)

| Module | What it does |
|---|---|
| `src/env.rs` | Gravity, magnetic field reference, IMU noise model (VN-200 datasheet values). **Set these for the vehicle.** |
| `src/linalg/` | Matrix helpers on nalgebra fixed-size types. `basics`: diag and outer products; `skew`: the cross-product matrix skew(v); `blocks`: reading and writing 3x3 blocks of larger matrices; `spd`: symmetrising a matrix and checking positive definiteness; `cholesky`: Cholesky factorisation L L^T; `solve`: solving S x = b with Cholesky (no explicit inverse). |
| `src/quat/` | Quaternions and rotation matrices. `product`: Hamilton product and the rotation matrix R(q); `quat_exp_log`: Exp and Log maps for quaternions; `so3_exp_log`: Exp and Log maps for rotation matrices. |
| `src/rot/` | Small rotations (perturbations) and their Jacobians. `perturbation`: boxplus / boxminus and gyro integration. |
| `src/nominal/` | The nominal state, strapdown propagation, and the error state. `strapdown`: the nominal state and strapdown propagation; `error_state`: inject / difference between the nominal state and the 15-number error state. |
| `src/noise/` | IMU noise models. `units`: IMU noise units: density to per-sample sigma, random-walk steps. |
| `src/errprop/` | Error-state propagation: F and Q. `transition`: the error-state transition matrix F; `process_noise`: the process noise Q. |
| `src/esekf/` | The 15-state error-state Kalman filter. `filter`: the Esekf struct and predict; `models`: position and magnetometer measurement models and Jacobians; `update`: the generic gated update with injection and covariance reset; `sensors`: position (GPS / UWB) and magnetometer updates; `height`: height (barometer / downward lidar) updates. |
| `src/align/` | Start-up: TRIAD alignment and the initial covariance. `triad`: TRIAD: attitude from one accelerometer and one magnetometer reading; `start`: initial covariance and align (building the filter from the first readings). |
| `src/interop.rs` | f32 conversions for the Drone software and `Common::DroneTelemetry`. |
| `tests/` | Host-only: unit tests for every file, a flight simulator, fault scenarios and whole-filter flights checked for consistency (NEES). |

Everything is re-exported from the crate root (`use Estimation::*;`).

## Build and test (from `Monoprop/`)

```text
cargo test -p Estimation                                        # every test, on the PC
cargo build -p Estimation --lib --target thumbv7em-none-eabihf  # proves it still builds for the Teensy
```

Run both before every push that touches this crate.

## Using it

```rust
use Estimation::*;

// IMU noise: the VN-200 datasheet model at its sample rate (replace with Allan-deviation values later)
let imu_spec = ImuSpec::vn200(200.0);

// start-up: from averaged readings with the vehicle still (None: readings TRIAD can't use, e.g. a
// magnetometer parallel to gravity or a dead sensor; try again with the next readings)
let Some(mut filter) = align(&accel0, &mag0, &pos0, imu_spec, SIGMA_POS) else { return };

// every IMU sample, dt from timestamps (false = refused: a NaN reading or a bad dt; nothing changed)
let ok = filter.predict(&accel, &gyro, dt);

// when a measurement arrives (false = rejected: NaN, or an outlier by the gate)
let used = filter.update_gps(&position, SIGMA_POS);         // any 3-D position fix (UWB or GPS)
let used = filter.update_mag(&mag, &mag_reference(), SIGMA_MAG); // raw reading, any unit: only its direction is used
let used = filter.update_height(height, SIGMA_HEIGHT);      // barometer or downward lidar

// estimate: filter.x.p, filter.x.v, filter.x.q, filter.x.ab, filter.x.wb; covariance filter.p
// for telemetry: filter.position_f32(), velocity_f32(), orientation_xyzw_f32(), angular_velocity_f32(&gyro)
```

`update_gps` takes any 3-D position fix in the navigation frame, so it serves UWB position as well.
New sensors use the generic `update::<M>(&innovation, &h, &r, gate)`; `src/esekf/height.rs` is the smallest
complete example.

## Conventions

- Navigation frame: z up (gravity `(0, 0, -g)`), x/y horizontal (ENU in `env.rs`). Attitude q rotates body vectors into it.
- Error state order: position, velocity, attitude (local, radians), accel bias, gyro bias (15 numbers).
- Units: SI (m, m/s, m/s^2, rad, rad/s); the magnetometer only needs a consistent unit (only its direction is used).
