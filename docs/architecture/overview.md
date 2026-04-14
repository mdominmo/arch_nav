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
