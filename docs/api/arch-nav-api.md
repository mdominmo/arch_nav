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

## Imperative commands

```cpp
CommandResponse arm();
CommandResponse disarm();
CommandResponse set_roi(GlobalPosition position,
                        ReferenceFrame frame = GLOBAL_WGS84);
CommandResponse clear_roi();
```

Commands return immediately. `set_roi` and `clear_roi` are only accepted in `IDLE` or `DISARMED` states; they return `DENIED` if the controller is `RUNNING` or `HANDOVER`. If the active driver does not support the requested frame, `NOT_SUPPORTED` is returned. On `ACCEPTED`, the confirmed ROI is written to `OperationContext`.

## Context updates

```cpp
void set_obstacle_info(std::vector<Obstacle> obstacles);
void remove_obstacle(const std::string& id);
void clear_obstacles();
```

Context updates write directly to `OperationContext` without going through the platform driver. They are accepted in any controller state. See [Operations Model](../architecture/operations.md) for the rationale behind context updates vs imperative commands.

Obstacles are modeled as identifiable cylinders (`Obstacle{id, position, radius, height}`). The driver can subscribe to obstacle changes via `OperationContext::subscribe_obstacles`.

## Read-only state

```cpp
OperationStatus operation_status() const;
const OperationReport* last_operation_report() const;
GlobalPosition global_position() const;
Kinematics kinematics() const;
VehicleStatus vehicle_status() const;
std::optional<GlobalPosition> get_roi() const;
std::vector<Obstacle> get_obstacles() const;
```

## Event callbacks

```cpp
void on_operation_complete(std::function<void(const OperationReport&)>);
void on_operation_progress(std::function<void(const OperationReport&)>);
```
