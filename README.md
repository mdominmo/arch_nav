# arch-nav

![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg) ![Version](https://img.shields.io/badge/version-0.0.1-green.svg)

Platform-agnostic UAV navigation kernel written in pure C++17. Handles operation control, vehicle state management, and trajectory planning with zero dependency on any specific autopilot, middleware, or framework (no ROS, no MAVLink, no PX4 — nothing).

Drivers are loaded as shared library plugins at runtime via `dlopen`. Each driver can use whatever technology it needs internally (ROS 1, ROS 2, MAVSDK, serial, etc.) — the kernel doesn't care.

## Quick start

```cpp
#include "arch_nav.hpp"

auto nav = arch_nav::ArchNav::create("/path/to/arch_nav_config.yaml");
arch_nav::ArchNavApi& api = nav->api();

api.arm();
api.takeoff(10.0);
api.waypoint_following(waypoints);
api.land();
```

## Build and install

### Prerequisites

- CMake >= 3.16
- C++17 compiler
- yaml-cpp
- Driver-specific dependencies (e.g. MAVSDK for the MAVSDK driver)

### Compile kernel + drivers

The kernel `CMakeLists.txt` compiles the kernel library and any drivers found in `drivers/` or pointed to by the `ARCH_NAV_DRIVERS` environment variable.

```bash
cd arch_nav_ws

# Tell CMake where the driver source code is
export ARCH_NAV_DRIVERS=$(pwd)/src/arch-nav-mavsdk-driver

# Build
cmake -B src/arch-nav/build -S src/arch-nav
cmake --build src/arch-nav/build -j$(nproc)

# Install system-wide
sudo cmake --install src/arch-nav/build
sudo ldconfig
```

After this, `libarch_nav.so` is installed to `/usr/local/lib/` and driver `.so` files go to `/usr/local/lib/arch_nav/drivers/`. No `LD_LIBRARY_PATH` needed.

Alternatively, drivers can be placed directly inside `src/arch-nav/drivers/` and they will be discovered automatically without the environment variable.

### Example with the MAVSDK driver

```bash
cd arch_nav_ws

# MAVSDK must be installed (see https://mavsdk.mavlink.io/main/en/cpp/guide/installation.html)
export ARCH_NAV_DRIVERS=$(pwd)/src/arch-nav-mavsdk-driver

cmake -B src/arch-nav/build -S src/arch-nav
cmake --build src/arch-nav/build -j$(nproc)
sudo cmake --install src/arch-nav/build
sudo ldconfig
```

### Compile external modules (e.g. ROS 2 nodes)

External modules that use ROS 2 (like `arch_nav_json_flight_plan`) are compiled separately with colcon:

```bash
source /opt/ros/humble/setup.bash
colcon build --paths src/arch_nav_json_flight_plan
```

### Run

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run arch_nav_json_flight_plan arch_nav_json_flight_plan \
  --ros-args \
  -p mission_file:=/path/to/mission.json \
  -p config:=/path/to/arch_nav_config.yaml
```

## Navigation API

All operations are **non-blocking**. Use `operation_status()` to track progress.

| Method | Description |
|--------|-------------|
| `arm()` | Arms the vehicle |
| `disarm()` | Disarms the vehicle |
| `takeoff(double height)` | Takes off to the specified height in metres AGL |
| `waypoint_following(vector<GeoWaypoint>)` | Navigates through a sequence of geographic waypoints |
| `land()` | Lands the vehicle |
| `cancel_operation()` | Aborts the current operation |
| `operation_status()` | Returns the current `OperationStatus` |
| `last_operation_report()` | Returns the report for the last completed operation |

### Operation status

| Status | Meaning |
|--------|---------|
| `HANDOVER` | Kernel does not have control of the vehicle |
| `DISARMED` | Vehicle is disarmed, kernel has control |
| `IDLE` | Vehicle is armed and ready, no operation running |
| `RUNNING` | An operation (takeoff, waypoints, land) is in progress |
| `FAILED` | Last operation failed |

### Operation reports

- **`OperationReport`** -- base report with `result()` (`IN_PROGRESS`, `COMPLETED`, `ABORTED`).
- **`WaypointReport`** -- extends `OperationReport` with `total_waypoints()` and `completed_waypoints()`.

## Configuration

```yaml
# arch_nav_config.yaml

driver: "mavsdk"               # driver name as registered in DriverRegistry
driver_config_path: ""          # optional path to driver-specific config
```

## Writing a driver

A driver is a shared library that implements `IPlatformDriver` and registers itself in the `DriverRegistry`. The kernel loads it at runtime via `dlopen`.

### 1. Implement the interfaces

```cpp
#include "platform/i_platform_driver.hpp"
#include "dispatchers/i_command_dispatcher.hpp"

class MyDispatcher : public arch_nav::dispatchers::ICommandDispatcher {
  void execute_takeoff(double height, std::function<void()> on_complete) override;
  void execute_land(std::function<void()> on_complete) override;
  void execute_waypoint_following(
      std::vector<arch_nav::vehicle::GeoWaypoint> waypoints,
      std::function<void()> on_complete) override;
  void execute_trajectory(
      std::vector<arch_nav::vehicle::TrajectoryPoint> trajectory,
      std::function<void()> on_complete) override;
  void execute_arm() override;
  void execute_disarm() override;
  void stop() override;
};

class MyDriver : public arch_nav::platform::IPlatformDriver {
  arch_nav::dispatchers::ICommandDispatcher& dispatcher() override;
  void start(arch_nav::context::VehicleContext& context) override;
  void stop() override;
};
```

The `start()` method receives a `VehicleContext` that the driver must feed with position, kinematics, and status updates.

### 2. Register the driver

Create a source file with a static registration object:

```cpp
#include "platform/driver_registry.hpp"

namespace {
struct Registration {
  Registration() {
    arch_nav::platform::DriverRegistry::instance().register_driver(
        "my_driver",
        [](const std::string& config_path) {
          return std::make_unique<MyDriver>(config_path);
        });
  }
};
static Registration reg;
}
```

### 3. Build as a shared library

The driver `CMakeLists.txt` must produce a `.so` named `libarch_nav_<name>.so`:

```cmake
project(arch_nav_my_driver)

find_package(arch_nav REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/my_driver.cpp
  src/my_dispatcher.cpp
  src/registration.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC arch_nav::arch_nav)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/arch_nav/drivers
)
```

### 4. Set the driver name in config

```yaml
driver: "my_driver"
```

