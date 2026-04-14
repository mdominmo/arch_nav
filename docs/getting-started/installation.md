# Installation

## Prerequisites

- CMake >= 3.16
- C++17 compiler
- `yaml-cpp`
- Driver-specific dependencies (for example MAVSDK if using the MAVSDK driver)

## Build kernel + external drivers

From the workspace root:

```bash
cd arch_nav_ws
export ARCH_NAV_DRIVERS=$(pwd)/src/arch_nav_mavsdk_driver

cmake -B src/arch_nav/build -S src/arch_nav
cmake --build src/arch_nav/build -j$(nproc)
```

Install:

```bash
sudo cmake --install src/arch_nav/build
sudo ldconfig
```

This installs:

- `libarch_nav.so`
- driver plugins under `.../lib/arch_nav/drivers/`

## Build ROS 2 external modules (optional)

If using `arch_nav_json_flight_plan`:

```bash
source /opt/ros/humble/setup.bash
colcon build --paths src/arch_nav_json_flight_plan
```
