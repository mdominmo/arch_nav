# Architecture Overview

## Core components

- `ArchNav`: top-level entry point (`create`, `api`).
- `ArchNavApi`: user-facing non-blocking navigation API.
- `OperationalController`: state machine and operation orchestration.
- `VehicleContext`: live vehicle state (telemetry) written by the driver, read by the kernel.
- `OperationContext`: operational intent and situational awareness written by the API/kernel, read by the driver.
- `DriverRegistry` + `DriverPluginLoader`: runtime plugin discovery and instantiation.

## Runtime flow

1. `ArchNav::create()` loads driver plugins.
2. A driver is selected (`ARCH_NAV_DRIVER` or automatic selection if unique).
3. The driver is created with optional config (`ARCH_NAV_DRIVER_CONFIG`).
4. Driver telemetry updates `VehicleContext`.
5. API calls route to `OperationalController`.
6. Controller dispatches operations to the driver through `ICommandDispatcher`.
7. Context updates (obstacles, etc.) are written to `OperationContext`; the driver can subscribe to react.

## State model

The operation state exposed by the controller/API:

- `HANDOVER`
- `DISARMED`
- `IDLE`
- `RUNNING`

## Action categories

The kernel classifies API actions into three categories. See [Operations Model](operations.md) for a detailed description.

- **Navigation Tasks** (`takeoff`, `waypoint_following`, `trajectory_execution`, ...): long-running operations that transition the controller to `RUNNING`.
- **Imperative Commands** (`arm`, `disarm`, `set_roi`, `clear_roi`): instantaneous actions that require driver confirmation. Accepted in `DISARMED` and `IDLE`.
- **Context Updates** (`set_obstacle_info`, `remove_obstacle`, ...): declarative writes to `OperationContext` that do not go through the driver. Accepted in any state.
