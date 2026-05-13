# Architecture Overview

## Core components

- `ArchNav`: top-level entry point (`create`, `api`).
- `ArchNavApi`: user-facing non-blocking navigation API.
- `OperationalController`: state machine and operation orchestration.
- `VehicleContext`: live vehicle state storage and subscriptions.
- `DriverRegistry` + `DriverPluginLoader`: runtime plugin discovery and instantiation.

## Runtime flow

1. `ArchNav::create()` loads driver plugins.
2. A driver is selected (`ARCH_NAV_DRIVER` or automatic selection if unique).
3. The driver is created with optional config (`ARCH_NAV_DRIVER_CONFIG`).
4. Driver telemetry updates `VehicleContext`.
5. API calls route to `OperationalController`.
6. Controller dispatches operations to the driver through `ICommandDispatcher`.

## State model

The operation state exposed by the controller/API:

- `HANDOVER`
- `DISARMED`
- `IDLE`
- `RUNNING`

### Command vs NavigationTask

The controller distinguishes two kinds of actions:

- **NavigationTask** (`takeoff`, `waypoint_following`, `trajectory_execution`): moves the state to `RUNNING` and fires `on_operation_complete` when finished.
- **Command** (`arm`, `disarm`, `set_roi`, `clear_roi`): executes immediately, returns a `CommandResponse`, and does not change the FSM state. Commands are only accepted in `DISARMED` and `IDLE`; they are `DENIED` in `RUNNING` and `HANDOVER`.

### ROI (Region of Interest)

`VehicleContext` stores an optional `GlobalPosition` as the active ROI. The driver writes to it via `update_roi()` / `clear_roi()` after forwarding the command to the autopilot. The API exposes `get_roi()` for consumers that need to read current ROI state.
