# arch_nav

`arch_nav` is a platform-agnostic UAV navigation kernel written in C++17.

It provides:

- A non-blocking navigation API for takeoff, landing, waypoint following, and trajectory execution.
- Runtime driver loading through shared-library plugins.
- A control-layer state machine (`HANDOVER`, `DISARMED`, `IDLE`, `RUNNING`) that is independent from any autopilot stack.

## Who this is for

This documentation is intended for:

- Integrators embedding `arch_nav` in applications.
- Driver authors implementing `IPlatformDriver` and `ICommandDispatcher`.
- Maintainers evolving the core architecture safely.

## Documentation map

- **Getting Started**: install, build, and run a minimal mission flow.
- **Architecture**: kernel components and driver lifecycle.
- **API Reference**: public headers and integration contracts.
- **Development**: build/test workflows and concurrency constraints.
