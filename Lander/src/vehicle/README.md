# Vehicle parameters and modules

This is the starting guide for configuring a vehicle and connecting it to the
Lander control loop. For the implementation history, design rationale and complete
method reference, see [UPDATE.md](UPDATE.md).

Define a vehicle with `VehicleParameters`. Put its measured masses, engine feed
connections, actuator limits and principal moments of inertia in one Rust function.
`profiles::monarch()` contains Monarch's defaults. Start from
that profile or use a struct literal with `..VehicleParameters::default()`.

## Start here

Read [First run](#first-run) to create and step a vehicle, then
[Configure a vehicle](#configure-a-vehicle) to change its physical parameters.
Use [Per-feed throttles](#per-feed-throttles-and-servo-valves) for valve commands,
[Custom engine dynamics](#replace-engine-or-tvc-behavior) for coupled propulsion
calculations, and [Troubleshooting](#troubleshooting) when a change does not appear.

| Term | Meaning in this code |
| --- | --- |
| FSM | `FlightStateMachine`, the object that owns the control loop and flight phase. |
| Navigator / EKF | Estimates position, velocity and attitude from sensors; EKF means extended Kalman filter. |
| Guidance | Plans a trajectory to the target. The built-in implementation is `Lossless`. |
| MPC | Model predictive controller; computes thrust and gimbal commands from state and trajectory. |
| TVC | Thrust vector control; maps the requested gimbal/thrust command to realized control. |
| Mass component | One physical contribution to mass, such as a tank, fuel grain, nozzle or structure. |
| Feed | A connection from a named mass component to an engine, with its own throttle. |
| CoM / inertia | Center of mass and resistance to angular acceleration. |
| Model / trait | A Rust implementation and the interface it implements, e.g. `ServoValve` implements `ThrottleModel`. |

There are three different quantities: **net thrust** is a force in newtons,
**feed opening** is a normalized valve position, and **propellant flow** is mass
per second. A custom engine model defines the relationship between them.

## First run

The crate is named `Lander`, including the capital L. Within this repository, code
can import `crate::vehicle`; an external Rust caller imports `Lander::vehicle`.
An external caller can add this path dependency to its `Cargo.toml`, replacing the
path with its own relative path to the Lander directory:

```toml
[dependencies]
Lander = { path = "../monoprop-flight-software/Lander" }
```

The following complete `main` can be placed in that caller's `src/main.rs` and run
with `cargo run`. It starts Monarch at an altitude of 30 m and advances three
Standby ticks using illustrative stationary sensor samples. It does not launch or
integrate vehicle motion. In your application, replace the samples with readings
from your simulator or sensor drivers.

```rust
use Lander::fsm::FlightStateMachine;
use Lander::state::{ImuSample, SensorData, VehicleState};
use Lander::vehicle::{profiles, FeedThrottle, ServoValve};

fn main() {
    let mut parameters = profiles::monarch();
    parameters.mass_components[0].dry_mass_kg = 50.0;
    parameters.mass_components[0].propellant_mass_kg = 20.0;
    // Example actuator settings, not a calibration for Monarch's hardware.
    parameters.engines[0].feeds[0].throttle = FeedThrottle::new(
        0.0,
        ServoValve {
            closed_angle_rad: 0.0,
            open_angle_rad: 90.0_f64.to_radians(),
            max_speed_rad_s: 90.0_f64.to_radians(),
        },
    );

    let mut fsm = FlightStateMachine::new_with_vehicle(parameters);
    let mut initial = VehicleState::default();
    initial.position.z = 30.0;
    fsm.initialize_with_state(initial, 0.0);

    // Set runtime commands AFTER initialization, which restores initial commands.
    if let Some(throttle) = fsm.feed_throttle_mut(0, 0) {
        throttle.set_command(0.6);
    }
    for tick in 1..=3 {
        let now = tick as f64 * 0.002;
        let sensors = SensorData {
            timestamp: now,
            imu_data: Some(ImuSample::new(now, [0.0, 0.0, 9.81], [0.0; 3])),
            gps_data: None,
            uwb_data: Some([0.0, 0.0, 30.0]),
            chamber_pressure: Some(100_000.0),
            tank_pressure: Some(300_000.0),
        };
        match fsm.step(&sensors) {
            Some(control) => println!("control [theta, phi, thrust, RCS]: {control:?}"),
            None => {
                println!("terminated: {:?}", fsm.get_state().termination_reason);
                break;
            }
        }
    }
    if let Some(throttle) = fsm.feed_throttle(0, 0) {
        println!("command={}, opening={}", throttle.commanded_opening, throttle.opening);
    }
    println!("mass={} kg", fsm.mass_properties().mass_kg);
}
```

The expected valve opening after these ticks is about 0.006: travel is limited to
one full opening per second, and 0.006 s has elapsed. Total mass stays at 70 kg
because the default engine does not consume propellant in Standby. The control
output's main thrust remains zero even though the separately modeled feed moves.

`step()` returns `[gimbal theta rad, gimbal phi rad, thrust N, RCS command]` inside
`Some`. `None` means flight termination, not simply that the next update is not
due. Existing sensor checks still apply in Standby: provide both pressure fields,
regular IMU samples and position updates. They are independent of the removed
vehicle-parameter validation.

Use one monotonic seconds clock for initialization and sensor timestamps. The FSM
updates navigation, feed actuators, engine dynamics and component callbacks when
a fusion tick is due (500 Hz by default); it is not a background task. Calling
`step()` faster does not advance those models until the next due tick. The first
sensor interval after a zero-time start uses the scheduler's nominal 0.002 s;
later intervals use elapsed sensor time. Guidance is scheduled at 1 Hz and MPC at
50 Hz. Standby can estimate state and generate guidance, but does not run the
in-flight MPC path. No separate startup warm-up procedure is implemented.

For a normal mission the existing `arm(now)` and `launch(now)` methods transition
from Standby through Armed into Ascent. A simulator can use `set_flight_phase()`
after initialization to choose a starting phase. These are existing lifecycle
methods; defining a vehicle does not launch it automatically.

## Where to put your code

- Put a reusable parameter function in [profiles.rs](profiles.rs), or in your
  caller's own Rust module. Return `VehicleParameters` from it.
- Put coupled thermal, pressure, fuel-grain, nozzle and consumption calculations
  in an `EngineModel` implementation owned by your caller or vehicle module.
- Put valve/servo position dynamics in a `ThrottleModel`; its output feeds the
  engine model's flow calculation.
- Put a local mass rule such as independent venting in a `MassUpdateFn` callback.
  Use the engine model for calculations involving several parts together.
- Put gimbal/linkage behavior in `TvcModel`. Hardware I/O remains in your driver.

A custom file inside `vehicle/` must be declared with `pub mod your_module;` in
[mod.rs](mod.rs). A module in your caller needs no change to Lander's module list.
The `Box::new(model)` syntax transfers ownership of a model to the FSM. `impl Trait`
defines the methods the runtime calls. `if let Some(...)` handles a lookup that
may not find the requested model type or feed index.

## Configure a vehicle

This illustrative vehicle has two tanks and two engines. The numbers are examples,
not a measured vehicle definition. Each engine's hardware mass is counted once in
`mass_components`; `engines` describes its thrust and propellant connections.

```rust
use Lander::fsm::FlightStateMachine;
use Lander::state::{PropulsionParameters, VehicleState};
use Lander::vehicle::{EngineParameters, MassComponent, PropellantFeed, VehicleParameters};

let engine_a = EngineParameters {
    name: "engine A".into(),
    min_thrust_n: 100.0,
    max_thrust_n: 600.0,
    propulsion: PropulsionParameters { specific_impulse_s: 180.0 },
    feeds: vec![PropellantFeed::new("tank A", 1.0)],
};
let engine_b = EngineParameters {
    name: "engine B".into(),
    feeds: vec![PropellantFeed::new("tank B", 1.0)],
    ..engine_a.clone()
};

let mut parameters = VehicleParameters {
    mass_components: vec![
        //                       name       dry kg  propellant kg
        MassComponent::new("structure",        40.0,  0.0),
        MassComponent::new("tank A",            2.0, 10.0),
        MassComponent::new("tank B",            2.0, 10.0),
        MassComponent::new("engine A hardware", 3.0,  0.0),
        MassComponent::new("engine B hardware", 3.0,  0.0),
    ],
    engines: vec![engine_a, engine_b],
    inertia_kg_m2: [7.0, 7.0, 2.5],
    ..VehicleParameters::default()
};
parameters.tvc.max_angle_rad = 10.0_f64.to_radians();
parameters.tvc.max_angle_step_rad = 1.0_f64.to_radians();
parameters.tvc.max_thrust_step_n = 25.0;
parameters.tvc.jog_max_thrust_n = 1200.0;

let mut fsm = FlightStateMachine::new_with_vehicle(parameters);
let mut initial = VehicleState::default();
initial.position.z = 30.0;
fsm.initialize_with_state(initial, 0.0);
```

All quantities use SI units. Component and engine names must be unique within their
respective lists. For the default engine model, each feed names a component, and
each engine's feed mass fractions sum to one. For a fuel/oxidizer engine, list both tanks with the appropriate
fractions of total consumed propellant mass. Several engines can share a tank.

Mass components and engines are different lists: the engine entry describes
performance and connections, while the engine's hardware mass is counted once
in a mass component. Component names are matched exactly by feeds. Engine/feed
indices are zero-based positions in `parameters.engines` and each engine's
`feeds`; component indices refer to `parameters.mass_components` instead.

| Parameter | Unit / interpretation |
| --- | --- |
| Component masses | kg; dry mass, consumable propellant, and named additional terms. |
| `position_m` | m from a common body-fixed reference point to a component's CoM. |
| Component `inertia_kg_m2` | 3×3 tensor in kg m² about that component's CoM. |
| Vehicle `inertia_kg_m2` | Three constant principal moments in kg m² used by the existing MPC. |
| Engine min/max thrust | N; used for nominal aggregate controller limits. |
| `specific_impulse_s` | s; default mass flow is `thrust / (specific_impulse_s * 9.80665)` in kg/s. |
| `mass_fraction` | Dimensionless nominal fraction of an engine's total draw before feed opening is applied. |
| Throttle command/opening | Dimensionless position, conventionally 0–1. |
| TVC `max_angle_rad` | rad; convert degrees with `.to_radians()`. |
| TVC `max_angle_step_rad`, `max_thrust_step_n` | rad/update and N/update, respectively, not rates per second. |
| Servo `max_speed_rad_s` | rad/s; unlike TVC steps, it uses elapsed time. |
| Ascent/descent maximum velocity | m/s; supplied during guidance planning. |

For the default shared-throttle engine bank, maximum thrust is the sum of engine
maxima. Minimum thrust is that sum multiplied by the largest individual
minimum/maximum ratio. A custom engine model does not automatically replace these
controller bounds. Vehicle parameters are used directly without validation.

The FSM copies the configuration into its built-in
MPC and guidance adapters: thrust bounds, TVC angle, principal moments of inertia,
guidance velocity limits, and nominal consumption coefficient. The MPC now uses
its configured minimum thrust instead of a literal value. Mission targets, flight
limits and algorithm tuning retain their existing accessors.

## Initial and live masses

| What you want to change | Where to change it |
| --- | --- |
| Starting masses, geometry, limits, actuator model | Edit `VehicleParameters` before installation. |
| Starting pose and velocity | Pass a `VehicleState` to `initialize_with_state()`. |
| One feed's target during a run | `fsm.feed_throttle_mut(engine, feed)` then `set_command()`. |
| A custom engine's inputs/history | `fsm.engine_model_mut::<YourEngine>()`. |
| A custom actuator's parameters/inputs | Access its live feed throttle, then `model_mut::<YourValve>()`. |
| Measured dry/propellant mass during a run | `fsm.set_component_mass(name, dry, propellant)`. |
| Current mass distribution | Read `fsm.mass_components()` or `fsm.mass_properties()`. |

`vehicle_parameters()` returns the stored configuration, not live state.
Installing another vehicle with `set_vehicle()` restarts the run in Standby at the
current pose and sensor timestamp. For routine commands or input changes, use the
live accessors instead of reinstalling the vehicle every tick.

With `new_with_vehicle()` or `set_vehicle()`, component masses are authoritative.
`initialize_with_state()` supplies the pose, velocity and angular velocity; its
aggregate `mass` and `dry_mass` fields are replaced by the component totals.
Initialization refills components to their configured initial values and resets
engine/TVC internal history and each feed throttle's initial command, opening and
model state. Propulsion parameters and other tuning are retained.

`fsm.mass_components()` exposes the live masses. For externally measured changes,
use `fsm.set_component_mass(name, dry_mass_kg, propellant_mass_kg)`. This updates the
aggregate vehicle mass immediately, for example after venting or releasing hardware.
It does not modify the stored initial configuration or automatically change a
component's intrinsic inertia. Aggregate properties use the current component data.

## Component geometry and custom mass updates

`MassComponent` is defined in `mass.rs`. Each component stores:

- `position_m`: a `Vector3<f64>` from a common **body-fixed reference point** to
  the component's current center of mass, in metres. Choose the same reference
  point and body axes for every component; these are not world coordinates.
- `inertia_kg_m2`: a full `Matrix3<f64>` about the **component's own CoM**, expressed
  in those same body axes, including all its current mass. Enter a diagonal matrix
  when the component's principal axes align with the body axes; otherwise rotate
  the full tensor into the body axes before supplying it.
- `dry_mass_kg`, `propellant_mass_kg`, and `additional_masses_kg`: non-negative
  mass contributions. The additional map holds named non-engine-propellant terms
  such as pressurant, coolant or payload. They contribute to both total mass and
  the retained/dry mass reported to guidance, so they are not mistaken for engine
  propellant. Model separately located parts as separate components.
- `update_mass`: a function selected per component. A dry-only component simply
  has zero propellant and can use the default updater or its own function.

The existing `MassComponent::new(name, dry_mass_kg, propellant_mass_kg)` constructor
still works. It initializes geometry as a point mass at the reference point.
Populate geometry before using the computed CoM/inertia for a real vehicle. The
original lander profile puts its existing aggregate inertia on its sole component.

Here is an illustrative update function that combines engine consumption with a
separate vented mass term:

```rust
use Lander::vehicle::{MassComponent, MassUpdateContext};
use nalgebra::{Matrix3, Vector3};

fn update_tank(component: &mut MassComponent, context: &MassUpdateContext<'_>) {
    // Apply the already allocated engine draw exactly once.
    component.deplete_propellant(context);
    let pressurant = component.additional_masses_kg.entry("pressurant".into()).or_default();
    *pressurant = (*pressurant - 0.01 * context.dt_s).max(0.0);

    // Illustrative uniform-scaling assumption, not a tank/slosh model.
    // Replace this with the measured geometry/filled-tank model for your vehicle.
    component.inertia_kg_m2 = context.initial.inertia_kg_m2
        * (component.total_mass_kg() / context.initial.total_mass_kg());
}

let mut tank = MassComponent::new("tank A", 2.0, 10.0);
tank.position_m = Vector3::new(0.2, 0.0, 0.8);
tank.inertia_kg_m2 = Matrix3::from_diagonal(&Vector3::new(0.4, 0.4, 0.15));
tank.additional_masses_kg.insert("pressurant".into(), 0.5);
tank.update_mass = update_tank;
```

The context supplies elapsed seconds, the sensor sample (including timestamp),
flight phase, the allocated propellant draw in kg, and the original component
configuration. The callback is responsible for applying the draw; call
`deplete_propellant()` to retain the standard behavior. It can also change dry mass,
additional terms, position and inertia, for example when fuel drains or a payload
is released. Engine feeds refer to components by name.

Callbacks run after feed-actuator and engine dynamics on sensor-fusion ticks in every phase, including
Standby. The default engine queues zero draw outside Ascent/Hover/Descent. A callback can use `context.flight_phase` to
choose when other losses occur. Functions must be stateless (a function pointer
or non-capturing closure); store changing quantities in the component. Each callback
updates its live component directly.
Reinitialization restores configured masses, geometry and update functions.

For a hybrid engine, the consumable grain belongs in `propellant_mass_kg` even
though it is solid. Keep its nonconsumable casing in dry mass. A liquid engine can
use separate fuel and oxidizer tank components. Neither layout supplies a detailed
engine simulation on its own; the custom engine implements that vehicle's physics.

`additional_masses_kg` contributes to the retained/dry aggregate even if a callback
later vents or releases it. It is not drawn by the default engine feed model.
The component callback is a function pointer, so it cannot capture a closure's
local variables. Put arbitrary typed history in the engine or throttle model when
the mass fields alone are insufficient.

Read the initial aggregate properties with `parameters.mass_properties()`, or the
current properties with `fsm.mass_properties()`. Both return total mass, retained
mass, CoM relative to the shared reference point, and a full inertia tensor **about
the vehicle CoM**. With component masses `m_i`, CoM positions `r_i`, and intrinsic
inertia tensors `I_i`, the calculation is:

```text
M = sum(m_i)
r_com = sum(m_i * r_i) / M
d_i = r_i - r_com
I_vehicle = sum(I_i + m_i * ((d_i dot d_i) * Identity - d_i * transpose(d_i)))
```

This uses the [parallel-axis theorem for inertia tensors](https://ocw.mit.edu/courses/16-07-dynamics-fall-2009/dd277ec654440f4c2b5b07d6c286c3fd_MIT16_07F09_Lec26.pdf).
The default updater changes propellant mass only; it cannot infer how a component's
internal mass distribution changes. Supply that geometry model in the callback.
`set_component_mass()` likewise leaves the component's intrinsic inertia and position
unchanged, though aggregate CoM and parallel-axis terms reflect the new mass.

These properties are available for callers and future dynamics models. The existing
MPC still uses the separately configured `VehicleParameters.inertia_kg_m2`; it does
not automatically consume the derived tensor or a changing CoM/thrust lever arm.

The existing `new()`/`new_with_algorithms()` plus `initialize_with_state()` path still
accepts aggregate masses. Its default vehicle has one aggregate component, so older
simulators can keep setting total and dry mass. For configured vehicles, change live
mass through `set_component_mass()` rather than writing `get_state_mut().mass`.

`set_propulsion_parameters()` remains available and applies one specific impulse to
all engines. Set individual engine values in the vehicle description when they differ.
`propulsion_parameters()` reports the equivalent specific impulse for the nominal
engine-bank allocation; the individual inputs are in `vehicle_parameters().engines`.

## Per-feed throttles and servo valves

Every `PropellantFeed` has a `FeedThrottle`. Its `commanded_opening` is the target
and its `opening` is the realized actuator position, both normalized from 0 (closed)
to 1 (fully open). Opening is a valve position, not necessarily a flow fraction.
`PropellantFeed::new(component, mass_fraction)` installs a fully open
`DirectThrottle`, preserving the original default consumption.

For an oxidizer servo valve, configure a `ServoValve` on that feed. These travel
angles and speed are illustrative; supply your actuator's values:

```rust
use Lander::fsm::FlightStateMachine;
use Lander::vehicle::{FeedThrottle, ServoValve, VehicleParameters};

let mut parameters = VehicleParameters::default();
parameters.engines[0].feeds[0].throttle = FeedThrottle::new(
    0.0, // Initial command and realized opening: closed.
    ServoValve {
        closed_angle_rad: 10.0_f64.to_radians(),
        open_angle_rad: 100.0_f64.to_radians(),
        max_speed_rad_s: 60.0_f64.to_radians(),
    },
);
let mut fsm = FlightStateMachine::new_with_vehicle(parameters);

// Zero-based engine/feed indices, matching vehicle_parameters().engines.
if let Some(throttle) = fsm.feed_throttle_mut(0, 0) {
    throttle.set_command(0.6);
}
// fsm.step(&sensor_sample) advances the actuator when a fusion update is due.
if let Some(throttle) = fsm.feed_throttle(0, 0) {
    let realized_opening = throttle.opening;
    if let Some(servo) = throttle.model::<ServoValve>() {
        let servo_angle_rad = servo.angle_rad(realized_opening);
    }
}
```

`ServoValve` maps opening to shaft angle and limits angular movement using elapsed
time. Its travel endpoints also limit the commanded target. It is a simple speed
model; your calibrated linkage, acceleration, deadband or load response can use a
custom implementation. It does not send hardware servo commands.

Implement `ThrottleModel` and derive `Clone` to provide another method. For example,
this actuator uses a configurable first-order response instead of a speed limit:

```rust
use Lander::vehicle::{FeedThrottle, ThrottleModel, ThrottleUpdateContext};

#[derive(Clone)]
struct LaggedValve {
    response_time_s: f64,
}

impl ThrottleModel for LaggedValve {
    fn update(&mut self, target: f64, previous: f64, context: &ThrottleUpdateContext<'_>) -> f64 {
        target + (previous - target) * (-context.dt_s / self.response_time_s).exp()
    }
}

let mut throttle = FeedThrottle::new(0.0, LaggedValve { response_time_s: 0.1 });
throttle.set_command(0.5);
// Change model-specific parameters or inputs between updates:
if let Some(valve) = throttle.model_mut::<LaggedValve>() {
    valve.response_time_s = 0.2;
}
// Replace the update method without changing the current opening or command:
throttle.set_model(LaggedValve { response_time_s: 0.3 });
```

The throttle context includes elapsed time, sensors, phase, net thrust command,
the engine/feed configuration and current mass components. Additional typed inputs
and history can live in your model and be changed through `model_mut::<T>()`.

Each feed owns its live model. Configuration clones are kept separately, and
reinitialization restores their commands, openings and model history. Custom
`Clone` implementations should copy actuator history independently; any shared
handles are intentionally shared. Changing a live command or replacing a live
model does not modify the initial configuration.

All feed throttles update before engine dynamics on every fusion tick, including
Standby. The engine reads the results from
`context.feed_throttles[engine_index][feed_index].opening`. Read live state there,
not from `context.engines[..].feeds[..].throttle`, which holds initial settings.
Custom engines choose how that opening affects consumption and predicted thrust.
The built-in controller still outputs one net thrust command; it does not set
independent feed commands automatically. Valve commands persist until changed
or reinitialized; the custom throttle model can use flight phase for its policy.

## Replace engine or TVC behavior

Implement `EngineModel::update_dynamics()` for vehicle-specific propulsion dynamics.
One call covers the entire propulsion system, so coupled engines and shared parts
can be updated together. `EngineUpdateContext` supplies:

- Elapsed seconds, sensor data, flight phase and the last applied net thrust command.
- All engine parameters, live `feed_throttles[engine_index][feed_index]`, and the
  original mass-component configuration.
- Mutable access to all live mass components, including positions and inertia.
- A zeroed `propellant_consumed_kg` slice, indexed like `components`, for optional
  consumption to apply through the component callbacks.

Store architecture-specific configuration, inputs, state and outputs in your own
engine struct. These can be ordinary Rust structs, enums, vectors or maps: multiple
throttle channels, chamber pressure, engine/propellant temperatures, fuel-grain
geometry, nozzle erosion, and cooling circuits do not require new FSM fields.
The model can own separate submodels for different engines and components.
Thermal and other non-mass state lives in those typed fields; `MassComponent`
continues to hold the mass and geometry used for vehicle CoM/inertia.

Use `fsm.engine_model_mut::<YourEngine>()` to supply custom inputs before `step()`
and `fsm.engine_model::<YourEngine>()` to read its state or outputs afterwards.
These return `None` if the installed model is a different type. The FSM owns the
model, so it must own its data or use shared handles rather than borrowed local data.

The update order is feed-actuator dynamics, engine dynamics, component callbacks, then aggregate mass
synchronization. A model may directly change any component's mass and geometry,
or queue propellant draws for the callbacks. For a given mass change, use one path
so it is applied once. Callbacks see any direct engine changes and may further
change them; assign each calculation to the appropriate model or callback.
Custom dynamics choose their own mixture ratios, depletion limits and phase policy.
Calls also occur in Standby, allowing cooling or pressure evolution without burning.

`reset()` is called on installation and reinitialization. Stateful implementations
restore their configured initial history there; the default method does nothing.
`guidance_alpha()` supplies the nominal constant consumption approximation required
by the built-in Lossless planner. Detailed dynamics do not automatically change
that planner or its thrust commands; a different planner can use `GuidancePlanner`.

Implement the engine in this order:

1. Define configuration and initial state, then live typed state for your parts.
2. Read live valve openings and sensors in `update_dynamics()`; compute coupled
   changes using `context.dt_s`.
3. Queue propellant draw in kg, or directly change mass/geometry. Assign each
   calculation to one place so component callbacks do not repeat it.
4. Implement `reset()` to restore configured history while retaining calibration.
5. Return a nominal guidance coefficient from `guidance_alpha()`. Installation
   evaluates this before resetting the engine, so it should depend on configured
   nominal performance rather than uninitialized live state.
6. Install with `set_vehicle()`, initialize the desired pose, then supply live
   custom inputs before subsequent `step()` calls.

The engine is called once for the whole vehicle, not once per engine entry. You
can own multiple engine submodels inside it. Its context allows mass mutations,
but exposes feed actuator states read-only: it consumes the openings calculated
earlier that tick. Set feed commands from the caller before stepping, or let a
custom throttle model compute its response using its own inputs and context.

This small example illustrates separate feed rates and direct updates to another
part. Its rates are supplied by the caller; it is not a liquid/hybrid physics model.
An actual implementation places its coupled dynamics equations in `update_dynamics()`.

```rust
use std::collections::BTreeMap;
use Lander::fsm::FlightStateMachine;
use Lander::state::FlightPhase;
use Lander::vehicle::{
    EngineModel, EngineParameters, EngineUpdateContext, MassComponent, PropellantFeed,
    SlewLimitedTvc, VehicleModules, VehicleParameters,
};

struct ExampleEngine {
    full_open_flow_kg_s: Vec<f64>, // one value per feed of this example's single engine
    nozzle_loss_kg_s: f64,
    temperatures_k: BTreeMap<String, f64>,
    initial_temperatures_k: BTreeMap<String, f64>,
}

impl EngineModel for ExampleEngine {
    fn reset(&mut self) {
        self.temperatures_k = self.initial_temperatures_k.clone();
    }

    fn update_dynamics(&mut self, context: &mut EngineUpdateContext<'_>) {
        // Custom thermal/pressure models can run here in every phase.
        if matches!(context.flight_phase,
            FlightPhase::Ascent | FlightPhase::Hover | FlightPhase::Descent)
        {
            let feeds = &context.engines[0].feeds;
            for (feed_index, feed) in feeds.iter().enumerate() {
                let opening = context.feed_throttles[0][feed_index].opening;
                // Illustrative linear flow map; replace with your valve/engine model.
                let flow = self.full_open_flow_kg_s[feed_index] * opening;
                for (index, component) in context.components.iter().enumerate() {
                    if component.name == feed.component {
                        context.propellant_consumed_kg[index] += flow * context.dt_s;
                    }
                }
            }
            for component in context.components.iter_mut() {
                if component.name == "nozzle" {
                    component.dry_mass_kg -= self.nozzle_loss_kg_s * context.dt_s;
                    // Update position/inertia here using this part's geometry model.
                }
            }
        }
    }

    fn guidance_alpha(&self, engine: &EngineParameters) -> f64 {
        engine.propulsion.alpha()
    }
}

let mut parameters = VehicleParameters {
    mass_components: vec![
        MassComponent::new("oxidizer", 2.0, 15.0),
        MassComponent::new("fuel", 2.0, 5.0),
        MassComponent::new("nozzle", 1.0, 0.0),
    ],
    ..VehicleParameters::default()
};
parameters.engines[0].feeds = vec![
    PropellantFeed::new("oxidizer", 0.8),
    PropellantFeed::new("fuel", 0.2),
];
let engine = ExampleEngine {
    full_open_flow_kg_s: vec![0.2, 0.05],
    nozzle_loss_kg_s: 0.0001,
    temperatures_k: BTreeMap::new(),
    initial_temperatures_k: BTreeMap::from([("nozzle".into(), 293.15)]),
};
let mut fsm = FlightStateMachine::new();
fsm.set_vehicle(parameters, VehicleModules {
    engine: Box::new(engine),
    tvc: Box::new(SlewLimitedTvc),
});
if let Some(engine) = fsm.engine_model_mut::<ExampleEngine>() {
    engine.full_open_flow_kg_s[0] = 0.25;
    engine.temperatures_k.insert("nozzle".into(), 300.0);
}
// Call fsm.step(&sensor_sample), then read updated model state:
if let Some(engine) = fsm.engine_model::<ExampleEngine>() {
    let nozzle_temperature = engine.temperatures_k.get("nozzle");
}
```

Implement `TvcModel` for different actuator dynamics or linkage calibration. It
receives requested and previous `[theta, phi, thrust]`, vehicle state, TVC parameters
and time. It returns realized control in radians/radians/newtons. Its `reset()` runs
on initialization. Ground jog uses the same module via `jog()`; an expired jog still
forces zero commands. Keep hardware-specific servo/valve commands in the module or
your hardware driver, rather than replacing physical units with servo counts.

For example, an ideal TVC can pass commands through directly. This is a wiring
example; the built-in `SlewLimitedTvc` includes movement limits:

```rust
use Lander::state::VehicleState;
use Lander::vehicle::{TvcModel, TvcParameters};

struct IdealTvc;

impl TvcModel for IdealTvc {
    fn reset(&mut self, _now: f64) {}

    fn update(
        &mut self,
        requested: [f64; 3],
        _previous: [f64; 3],
        _state: &VehicleState,
        _parameters: &TvcParameters,
        _now: f64,
    ) -> [f64; 3] {
        requested
    }
    // The default jog() calls update(); override it for different ground behavior.
}
```

Install both modules together:

```rust
use Lander::fsm::FlightStateMachine;
use Lander::vehicle::{ConstantSpecificImpulse, SlewLimitedTvc, VehicleModules, VehicleParameters};

let mut fsm = FlightStateMachine::new();
fsm.set_vehicle(VehicleParameters::default(), VehicleModules {
    engine: Box::new(ConstantSpecificImpulse), // substitute your EngineModel
    tvc: Box::new(SlewLimitedTvc),             // substitute your TvcModel
});
```

`set_vehicle()` starts a fresh Standby run at the current pose
and last sensor timestamp. Use it after `new_with_algorithms()` to combine custom
algorithms with custom vehicle modules. Custom algorithms remain caller-configured;
the FSM only applies built-in-specific settings to its built-in adapters.

## Algorithm tuning and initialization

Vehicle configuration describes the physical vehicle. Estimator uncertainty,
controller weights and RCS gains belong to their algorithms. The earlier
accessibility methods remain available:

```rust
use Lander::fsm::FlightStateMachine;
use Lander::state::VehicleState;

let mut fsm = FlightStateMachine::new();
fsm.initialize_with_state(VehicleState::default(), 0.0);
let mass = fsm.get_state().mass;
if let Some(mpc) = fsm.autopilot_mut().mpc_mut() {
    mpc.reset(mass, 0.0); // Reset solver history after changing controller setup.
}
if let Some(guidance) = fsm.autopilot_mut().lossless_mut() {
    guidance.coarse_delta_t = 0.5;
}
fsm.rcs_mut().kp = 15.0;
```

For EKF biases, covariance and process noise, call
`fsm.autopilot_mut().navigator_mut().inject_state(nominal, covariance, noise)`
after the FSM initializer. The built-in nominal state has 16 entries:
position (3), velocity (3), quaternion **w,x,y,z** (4), accelerometer bias (3),
gyro bias (3). Error covariance and process noise are 15×15 matrices. FSM
initialization seeds defaults; a direct built-in navigator `initialize()` retains
that navigator's process noise. Custom controllers may need their own history
reset through the controller accessor; the automatic solver reset targets MPC.

`set_vehicle()` applies physical settings to built-in MPC/Lossless adapters.
Custom algorithms still receive their trait calls, including guidance's
`configure(max_velocity, lower_thrust_bound, dry_mass)` before a solve, but their
additional configuration remains the caller's responsibility. See the
[accessibility reference](UPDATE.md#algorithm-accessibility-and-initialization-foundation)
for the available interfaces and initialization order.

## Scope of the default models

- The existing loop still uses one virtual control output:
  `[gimbal theta, gimbal phi, net thrust, RCS command]`. The default engine model shares a normalized
  throttle, with thrust allocated in proportion to engine maximum thrusts. The bank's
  minimum throttle is the largest per-engine minimum/maximum ratio. Independent
  engine throttles can be supplied as custom model inputs, but automatic independent
  engine control and per-engine physical actuator outputs need a different
  controller/allocation interface. Custom predicted thrust or other outputs can be
  read from the model; they do not replace the control loop's actuator output.
- `ConstantSpecificImpulse` calculates each engine's flow from its own specific
  impulse, then multiplies each nominal feed draw by its realized throttle opening.
  This is a simple linear flow approximation; custom engines can use a calibrated
  valve/flow relationship. Component draws are accumulated before consumption, so shared tanks are
  counted correctly. When a feed runs out, all bank draws are scaled together to
  preserve the requested draw ratios and prevent negative contents. This is mass accounting;
  it does not simulate flameout or cut commanded thrust when propellant is exhausted.
- `SlewLimitedTvc` preserves the existing per-control-update slew limits and direct
  ground jog behavior. A custom module can use timestamps for time-based dynamics.
- The MPC's inertia remains configured as three constant principal moments.
  Component geometry and update functions supply the separately derived live CoM
  and full inertia tensor described above.
- The external MPC dynamics library still fixes gravity at 9.81 m/s² and its thrust
  lever arm at 0.286 m. The external navigator and guidance libraries also contain
  their own model assumptions. Replace those algorithms through `new_with_algorithms()`
  when the vehicle needs different dynamics; the changes here do not rewrite those
  dependencies. RCS tuning remains accessible through `rcs_mut()`.

## Troubleshooting

| Observation | Explanation / next step |
| --- | --- |
| A changed throttle command does not immediately change opening | `set_command()` changes the target. Advance to a due fusion tick with `step()`; a servo may need several ticks. |
| Values in `vehicle_parameters()` never move | They are the stored starting configuration. Read `feed_throttle()` and `mass_components()` for live values. |
| A feed lookup returns `None` | Check zero-based engine and feed indices. The feed index is not a mass-component index. |
| `engine_model::<T>()` or `model::<T>()` returns `None` | `T` must be the exact concrete type installed, not a different implementation of the same trait. |
| A command disappears on initialization | Initialization restores feed prototypes. Apply live commands afterwards. |
| Default mass does not decrease in Standby | Default engine consumption is enabled only in Ascent/Hover/Descent. Feed actuators can still move. |
| A custom engine ignores throttle commands | Read `context.feed_throttles` and use its openings in your own flow equations. This is not inserted into a custom engine automatically. |
| No propellant is drawn from a named component | Match feed/component names exactly. Custom mass callbacks must apply queued consumption; engine/callback changes should not cancel or duplicate each other. |
| `step()` returns `None` | Inspect `get_state().termination_reason`. Existing sensor/flight checks remain active; adding a vehicle does not disable them. |
| CoM changes but MPC behavior does not reflect it | Computed mass properties are separate from the MPC's fixed inertia and external dynamics model. |
| Geometry stays constant while mass decreases | Default depletion changes mass only. Supply a geometry update in your engine or component callback. |
| A new field cannot be edited through `VehicleParameters` | Architecture-specific state belongs in the installed engine or actuator's typed struct. Use its mutable accessor. |

## Reading the source and verifying examples

| File | Start here when changing… |
| --- | --- |
| [profiles.rs](profiles.rs), [parameters.rs](parameters.rs) | Vehicle defaults and editable configuration. |
| [mass.rs](mass.rs) | Component mass, geometry and callbacks. |
| [throttle.rs](throttle.rs) | Feed actuator behavior and custom-model cloning. |
| [engine.rs](engine.rs) | Propulsion dynamics and default consumption. |
| [tvc.rs](tvc.rs) | Gimbal/thrust actuator behavior. |
| [mod.rs](mod.rs) | Runtime ownership, reset, update ordering and public exports. |
| [fsm/mod.rs](../fsm/mod.rs) | Installation, initialization, scheduling integration and caller access. |

The README is included in Rust module documentation, so its Rust examples are
compiled and executed by documentation checks. From `monoprop-flight-software/`:

```sh
cargo test --offline --manifest-path Lander/Cargo.toml --doc
cargo doc --offline --manifest-path Lander/Cargo.toml --no-deps
```

Offline commands assume the existing Rust dependencies have been fetched. The
generated API documentation is under `Lander/target/doc/Lander/`. The complete
change rationale and method inventory are in [UPDATE.md](UPDATE.md).

