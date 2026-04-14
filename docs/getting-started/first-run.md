# First Run

This page shows a minimal run using the JSON mission executor module.

## 1. Build required packages

```bash
source /opt/ros/humble/setup.bash
cd arch_nav_ws
colcon build --packages-select arch_nav arch_nav_mavsdk arch_nav_json_flight_plan
source install/setup.bash
```

## 2. Run mission executor

```bash
ros2 run arch_nav_json_flight_plan arch_nav_json_flight_plan \
  --ros-args \
  -p mission_file:=/path/to/mission.json \
  -p config:=/path/to/mavsdk_config.yaml \
  -p driver:=mavsdk
```

## 3. Expected flow

Typical log sequence:

1. mission loaded
2. vehicle connected / arming
3. takeoff progress
4. waypoint progress
5. optional landing

If startup fails, the node should print a fatal message with the exception reason.
