# Operations Model

The kernel classifies every action that the API exposes into one of three categories depending on its semantics, its relationship with the platform driver, and the guarantees it provides to the caller.

## Navigation Tasks

Navigation tasks are long-running operations that move the vehicle through space.

**Examples:** `takeoff`, `land`, `waypoint_following`, `trajectory_execution`, `change_yaw`.

### Characteristics

- The controller transitions to `RUNNING` when the driver accepts the task.
- Only one navigation task can be active at a time. A new task is `DENIED` while another is in progress.
- Completion is asynchronous: the driver invokes a callback when the operation finishes, and the controller transitions back to `IDLE`.
- Progress is reported periodically through `on_operation_progress`.
- The task can be interrupted externally (`cancel_operation`) or by a state change (disarm, control loss), which aborts the task and notifies via `on_operation_complete` with status `ABORTED`.

### Flow

```
API ──▸ OperationalController ──▸ NavigationTask::start()
           │                            │
           │  state → RUNNING           ▼
           │                     ICommandDispatcher::execute_*()
           │                            │
           │                            ▼
           │                      Platform Driver ──▸ FMU
           │                            │
           │  ◂── on_complete() ────────┘
           │
           │  state → IDLE
           ▼
    on_operation_complete(report)
```

## Imperative Commands

Commands are point-in-time actions that require explicit confirmation from the platform driver.

**Examples:** `arm`, `disarm`, `set_roi`, `clear_roi`.

### Characteristics

- The driver must accept or reject the command synchronously via `CommandResponse` (`ACCEPTED`, `DENIED`, `NOT_SUPPORTED`).
- Commands do not change the controller state machine — the controller remains in its current state after execution.
- Only accepted in `IDLE` and `DISARMED`; they are `DENIED` in `RUNNING` and `HANDOVER`.
- There is no completion callback — the response itself is the confirmation.

### When to use a command

A command is appropriate when the caller needs **a guarantee that the platform processed the request**. This typically applies to:

- Safety-critical operations: arming, disarming.
- Actions that the FMU must acknowledge: setting a ROI that drives gimbal control, establishing a geofence imposed by regulation.
- Any action where a `DENIED` response must prevent the operation from proceeding.

### Flow

```
API ──▸ OperationalController ──▸ VehicleCommand::execute()
                                        │
                                        ▼
                                 ICommandDispatcher::execute_*()
                                        │
                                        ▼
                                  Platform Driver ──▸ FMU
                                        │
                              CommandResponse ◂────┘
```

### ROI and OperationContext

`set_roi` and `clear_roi` are commands because they require FMU confirmation. However, the ROI value itself is operational state — it describes what the operator wants, not what the vehicle reports about itself.

For this reason, the confirmed ROI is stored in `OperationContext` (not in `VehicleContext`). The command confirms with the driver first, then writes to the operation context on success. This preserves the command's confirmation guarantee while keeping operational state in the right place.

## Context Updates

Context updates are declarative, non-imperative actions that inform the kernel about the state of the world without requiring confirmation from the platform.

**Examples:** `set_obstacle_info`, `remove_obstacle`, `clear_obstacles`.

### Characteristics

- The caller writes information to `OperationContext`. There is no round-trip to the driver and no `CommandResponse`.
- The platform driver can **react** to context changes by subscribing to `OperationContext` (e.g. `subscribe_obstacles`), but this is at its discretion — the kernel does not enforce it.
- Context updates are accepted regardless of the controller state. Informing the system about an obstacle is valid whether the vehicle is idle, running a mission, or disarmed.
- The data model supports identification (`Obstacle::id`), which allows updating the position of a known obstacle without creating duplicates.

### When to use a context update

A context update is appropriate when:

- The information describes the **environment**, not a command to the vehicle.
- There is no meaningful concept of the platform "rejecting" the information.
- The data may be updated at high frequency (sensor-rate obstacle feeds).
- The driver should decide autonomously how and when to act on the information.

### Flow

```
API ──▸ OperationalController ──▸ ContextUpdate::apply()
                                        │
                                        ▼
                                  OperationContext
                                        │
                              subscribe ─┤
                                        ▼
                                  Platform Driver
                                  (reacts at its discretion)
```

## Comparison

| | Navigation Task | Imperative Command | Context Update |
|---|---|---|---|
| **Semantics** | "Do this movement" | "Execute this action" | "This is what I know" |
| **Duration** | Long-running | Instantaneous | Instantaneous |
| **Confirmation** | Async callback | Synchronous response | None |
| **Controller state** | Transitions to `RUNNING` | No transition | No transition |
| **State restriction** | `IDLE` only | `IDLE` / `DISARMED` | Any state |
| **Driver role** | Executes and reports | Executes and confirms | Reads and reacts (optional) |
| **Examples** | takeoff, waypoint following | arm, set_roi, geofence | obstacle info, weather data |

## Data flow direction

The two context objects reflect opposite data flows:

- **VehicleContext** (driver → kernel): the driver writes telemetry from the FMU, the kernel and API read it. Represents what the vehicle reports about itself.
- **OperationContext** (API/kernel → driver): the API or kernel writes operational intent and situational awareness, the driver reads it. Represents what the operator knows or wants.

This separation ensures that each context has a single authoritative writer direction and a clear semantic boundary.
