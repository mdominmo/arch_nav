# arch-nav

![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg) ![Version](https://img.shields.io/badge/version-0.0.1-green.svg)

Platform-agnostic UAV navigation kernel. Handles trajectory planning, vehicle state management, and operation control with zero dependency on any specific autopilot or middleware.

The kernel exposes a single entry point (`ArchNav`) and a single API object (`NavigationApi`). All platform-specific knowledge — ROS 2, PX4, MAVLink, or anything else — lives entirely inside external drivers that self-register at startup via `DriverRegistry`.

## Usage

```cpp
#include "arch_nav.hpp"

// 1. Create the system — loads config, instantiates and starts the driver
auto arch_nav = arch_nav::ArchNav::create("/path/to/navigation_config.yaml");

// 2. Use the API
arch_nav::NavigationApi& api = arch_nav->api();

api.arm();
api.takeoff(5.0);                          // metres AGL
api.waypoint_following(waypoints);
api.land();
api.disarm();
```

`ArchNav::create()` blocks until the driver reports ready. The destructor stops it cleanly.

## Navigation API

All operations are **non-blocking**. Use `operation_status()` and `last_operation_report()` to track progress.

| Method | Description |
|--------|-------------|
| `arm()` | Arms the vehicle |
| `disarm()` | Disarms the vehicle |
| `takeoff(double height)` | Takes off to `height` metres above ground |
| `waypoint_following(vector<GeoPose>)` | Navigates through a sequence of geographic waypoints |
| `land()` | Descends at constant velocity until the autopilot exits offboard mode |
| `cancel_operation()` | Aborts the current operation |
| `operation_status()` | Returns the current `OperationStatus` |
| `last_operation_report()` | Returns the report for the last completed operation |

### Operation reports

- **`OperationReport`** — base report with `result()` (`IN_PROGRESS`, `COMPLETED`, `ABORTED`).
- **`WaypointReport`** — extends `OperationReport` with `total_waypoints()` and `completed_waypoints()`.

## Configuration

The kernel is configured via a YAML file passed to `ArchNav::create()`. A default is shipped at `config/navigation_config.yaml`.

```yaml
# navigation_config.yaml

driver: "px4"                  # driver name as registered in DriverRegistry
driver_config_path: ""         # optional — empty means the driver resolves its own default

local_planner:
  max_linear_velocity:      2.0   # m/s
  max_linear_acceleration:  1.0   # m/s²
  max_angular_velocity:     1.5   # rad/s
  max_angular_acceleration: 0.8   # rad/s²
  max_vertical_velocity:    0.5   # m/s
  max_vertical_acceleration: 0.2  # m/s²
  time_step:                0.05  # s
  land_descent_velocity:    0.4   # m/s — constant descent rate during landing
```