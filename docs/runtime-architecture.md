# Runtime Architecture

This repository uses a single runtime model in `pendulum-lib` and adapts it to firmware and simulation with platform-specific ECS systems.

## Decision Record

The runtime is split into four layers.

`src/runtime/model.rs` owns the in-memory `DeviceModel` and only the state that must persist while the device is alive.

`src/runtime/lifecycle.rs` owns boot, request planning, and request finalization. This layer is pure domain logic. It decides which state transitions are allowed and which side effects are required, but it does not talk to flash, Wi-Fi, motors, or sensors directly.

`src/runtime/effects.rs` owns effect descriptions and effect execution traits. The domain requests actions such as saving config, validating Wi-Fi, or calibrating the motor through `DeviceAction`, and platform adapters implement `DeviceServices` to carry them out.

`src/runtime/ecs.rs` owns ECS resources and domain-level ECS systems. It provides the shared runtime resource model used by both firmware and simulation, including request resources, control inputs, control outputs, motor telemetry, and the fixed-step control clock.

## Entrypoints

Firmware starts in `penfw/src/main.rs`. Its job is only board bring-up: initialize peripherals, load the boot snapshot, create the platform adapter, and hand control to `FirmwareRuntime`.

Simulation starts in `pensim/src/main.rs`. Its job is only process setup: create the telemetry stream, spawn the command server, and hand control to `SimulationRuntime`.

After entry, both platforms run the same domain runtime shape:

1. Boot a `DeviceModel` from persisted records and an explicit controller config.
2. Run a command pipeline that polls transport, plans requests, executes effects, finalizes replies, and handles reboot.
3. Run a control pipeline that samples sensors, steps the controller, applies the actuator command, advances the clock, and publishes telemetry where supported.

## Boot And Config

Boot status is derived from persisted records by `src/device.rs` plus `src/runtime/lifecycle.rs`.

The important rule is that the raw records survive until boot. Firmware no longer flattens calibration to `Some` or `None` before the state machine sees it, so the runtime can distinguish `Missing` from `Invalid` calibration and enter the correct fault state.

Controller tuning is now explicit at boot. `RuntimeConfig` produces a `ControllerConfig`, and that exact config is threaded into `boot_device_model`. The runtime no longer creates a hidden `PendulumController::new(Default::default())` behind the caller's back.

## ECS Data Flow

The shared ECS control model is:

`ControlInputs`
Current sensor inputs and estimator outputs that the controller consumes. Today this includes wheel angle, IMU estimate, and phase current.

`ControlOutputs`
The controller-facing actuator intent. This includes the raw `ControllerOutput` plus the normalized `MotorCommand` passed downstream.

`MotorTelemetryResource`
The measured or simulated actuator response. Simulation fills this from the motor model, while firmware currently uses it for measured current and leaves the unavailable values at defaults.

The fixed-step control path is:

1. Platform systems sample hardware or the simulated plant into `ControlInputs`.
2. `control_system` turns `ControlInputs` plus `DeviceModel` state into `ControlOutputs`.
3. Platform systems apply `ControlOutputs` to hardware or the simulated motor.
4. Platform systems update `MotorTelemetryResource`.
5. `advance_clock_system` increments the shared clock.
6. `publish_telemetry_system` reads the shared resources and emits runtime telemetry on host builds.

## Platform Adapters

Firmware-specific ECS systems live in `penfw/src/runtime.rs`.

They own serial polling, flash-backed effects, sensor sampling, and PWM output application. Hardware details stay in firmware modules such as `settings`, `hall`, `imu`, and `motor_drive`.

Simulation-specific ECS systems live in `pensim/src/runtime.rs`.

They own TCP command intake, simulated IMU sampling, motor model stepping, plant stepping, and world reset after reboot.

## Boundary Rules

When extending the runtime, prefer these rules.

1. If the code changes the allowed device states or request transitions, it belongs in `lifecycle.rs`.
2. If the code stores mutable runtime state, it belongs in `model.rs`.
3. If the code touches flash, Wi-Fi, calibration routines, transports, or hardware drivers, it belongs in a platform adapter or `effects.rs`.
4. If the code is about moving data through a fixed-step loop, it belongs in ECS systems and resources.
