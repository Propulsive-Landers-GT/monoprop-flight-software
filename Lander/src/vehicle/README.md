# Vehicle configuration

Define masses, engine feeds and actuator settings with `VehicleParameters`.
Start from `profiles::monarch()` and replace the engine, feed-throttle or TVC
models when your vehicle needs different behavior. Values use SI units.

## Configure and initialize

Each mass component stores its own CoM position and inertia tensor in common
body axes. Feeds refer to their source component by name. These values are examples.

```rust
use Lander::fsm::FlightStateMachine;
use Lander::state::VehicleState;
use Lander::vehicle::{profiles, FeedThrottle, MassComponent, PropellantFeed, ServoValve};
use nalgebra::{Matrix3, Vector3};

let mut body = MassComponent::new("body", 40.0, 0.0);
body.inertia_kg_m2 = Matrix3::from_diagonal(&Vector3::new(6.0, 6.0, 2.0));
let mut tank = MassComponent::new("tank", 10.0, 20.0);
tank.position_m = Vector3::new(0.0, 0.0, 0.8);
tank.inertia_kg_m2 = Matrix3::from_diagonal(&Vector3::new(0.5, 0.5, 0.2));

let mut parameters = profiles::monarch();
parameters.mass_components = vec![body, tank];
// Find a specific component by name and edit its initial mass and position.
if let Some(tank) = parameters.mass_components.iter_mut().find(|c| c.name == "tank") {
    tank.propellant_mass_kg = 18.0;
    tank.position_m.z = 0.75;
}
parameters.engines[0].feeds = vec![PropellantFeed::new("tank", 1.0)];
parameters.engines[0].feeds[0].throttle = FeedThrottle::new(
    0.0, // Start closed.
    ServoValve {
        closed_angle_rad: 0.0,
        open_angle_rad: 90.0_f64.to_radians(),
        max_speed_rad_s: 60.0_f64.to_radians(),
    },
);

let mut fsm = FlightStateMachine::new_with_vehicle(parameters);
let mut initial = VehicleState::default();
initial.position.z = 30.0;
fsm.initialize_with_state(initial, 0.0);

// Set live commands after initialization.
if let Some(throttle) = fsm.feed_throttle_mut(0, 0) {
    throttle.set_command(0.6);
}

let properties = fsm.mass_properties();
println!("CoM: {:?}", properties.center_of_mass_m);
println!("Component offsets: {:?}", properties.component_offsets_m);
println!("Vehicle inertia: {:?}", properties.inertia_kg_m2);
```

Call `fsm.step(&sensor_data)` with increasing timestamps to advance the loop.
It returns `Some([theta, phi, thrust, rcs])`, or `None` after termination; inspect
`fsm.get_state().termination_reason` for the reason. Existing sensor checks apply.
Feed actuators, engine dynamics and mass callbacks run on due fusion ticks,
including Standby. The default engine consumes propellant only in flight.

Indices are zero-based vector positions: `feed_throttle_mut(engine, feed)` selects
an engine's feed, while `component_offsets_m[i]` matches `mass_components()[i]`.
Order stays fixed during a run; engine/feed indices are separate from component indices.
Use `iter_mut().find(...)` to edit a component by name without relying on its index.
It selects the first matching name; if none matches, the `if let` body is skipped.

## Live state and mass updates

- `vehicle_parameters()` reads initial configuration; `mass_components()` and
  `feed_throttle()` read live state.
- `set_component_mass(name, dry_kg, propellant_kg)` changes a component's live mass.
- `MassComponent.update_mass` accepts a custom function. Call
  `component.deplete_propellant(context)` inside it to apply queued engine draw.
- CoM, component offsets and full inertia are recalculated after mass/geometry
  updates using the parallel-axis theorem. The built-in MPC receives that tensor.
- Component intrinsic inertia must be updated by your model when its internal
  mass distribution changes. Additional named masses count toward retained/dry mass.
- Reinitialization restores configured component and throttle state and resets
  engine/TVC history. Configured component totals determine initial vehicle mass.

## Custom engine dynamics

Implement `EngineModel` to calculate coupled consumption, thermal or degradation
behavior. The context exposes sensors, elapsed time, live feed openings and mutable
mass components. Store additional inputs/history on your own struct.

This single-feed example converts valve opening into a simple flow rate:

```rust
use Lander::fsm::FlightStateMachine;
use Lander::state::FlightPhase;
use Lander::vehicle::{
    EngineModel, EngineParameters, EngineUpdateContext,
    VehicleParameters, VehicleModules,
};

struct MyEngine { full_open_flow_kg_s: f64 }

impl EngineModel for MyEngine {
    fn update_dynamics(&mut self, context: &mut EngineUpdateContext<'_>) {
        if matches!(context.flight_phase,
            FlightPhase::Ascent | FlightPhase::Hover | FlightPhase::Descent)
        {
            let feed = &context.engines[0].feeds[0];
            let opening = context.feed_throttles[0][0].opening;
            let consumed = self.full_open_flow_kg_s * opening * context.dt_s;
            // Find the live component named by this feed and update it directly.
            if let Some(component) = context.components.iter_mut()
                .find(|c| c.name == feed.component)
            {
                component.propellant_mass_kg =
                    (component.propellant_mass_kg - consumed).max(0.0);
                // Update position/inertia here if your geometry model requires it.
            }
        }
    }

    fn guidance_alpha(&self, engine: &EngineParameters) -> f64 {
        engine.propulsion.alpha() // Nominal approximation for guidance.
    }
}

let mut fsm = FlightStateMachine::new();
fsm.set_vehicle(VehicleParameters::default(), VehicleModules {
    engine: Box::new(MyEngine { full_open_flow_kg_s: 0.2 }),
    ..VehicleModules::default()
});
if let Some(engine) = fsm.engine_model_mut::<MyEngine>() {
    engine.full_open_flow_kg_s = 0.25;
}
```

Override `reset()` when your engine retains history. Engine dynamics run before
component callbacks: either queue propellant draw or subtract it directly, without
applying the same consumption twice. This example is not a calibrated engine model.

For custom valve dynamics, implement `ThrottleModel`, derive `Clone`, and assign
`FeedThrottle::new(initial_opening, model)` to the feed. `set_command()` changes its
target; `opening` tracks realized position. Use `set_model()` to replace behavior
and `model_mut::<YourValve>()` to edit model-specific inputs. For custom gimbal
behavior, implement `TvcModel` and supply it through `VehicleModules.tvc`.

See [mass.rs](mass.rs), [engine.rs](engine.rs), [throttle.rs](throttle.rs) and
[tvc.rs](tvc.rs) for the interfaces. The controller still outputs one net thrust
command; separate feed commands and hardware servo I/O belong to your caller.
