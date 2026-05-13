# `arch_nav::ArchNavApi`

Header: `include/arch_nav/arch_nav_api.hpp`

## Purpose

User-facing API for commanding operations and querying state.

All operation methods are asynchronous.

## Operations

```cpp
CommandResponse takeoff(double height, ReferenceFrame frame = LOCAL_NED);
CommandResponse land();
CommandResponse waypoint_following(std::vector<Waypoint>, ReferenceFrame frame = GLOBAL_WGS84);
CommandResponse trajectory_execution(std::vector<TrajectoryPoint>, ReferenceFrame frame = LOCAL_NED);
void cancel_operation();
```

## Vehicle commands

```cpp
CommandResponse arm();
CommandResponse disarm();
CommandResponse set_roi(GlobalPosition position,
                        ReferenceFrame frame = GLOBAL_WGS84);
CommandResponse clear_roi();
```

Commands return immediately. `set_roi` and `clear_roi` are only accepted in `IDLE` or `DISARMED` states; they return `DENIED` if the controller is `RUNNING` or `HANDOVER`. If the active driver does not support the requested frame, `NOT_SUPPORTED` is returned.

## Read-only state

```cpp
OperationStatus operation_status() const;
const OperationReport* last_operation_report() const;
GlobalPosition global_position() const;
Kinematics kinematics() const;
VehicleStatus vehicle_status() const;
std::optional<GlobalPosition> get_roi() const;
```

## Event callbacks

```cpp
void on_operation_complete(std::function<void(const OperationReport&)>);
void on_operation_progress(std::function<void(const OperationReport&)>);
```
