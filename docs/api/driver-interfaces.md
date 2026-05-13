# Driver Interfaces

Headers:

- `include/arch_nav/driver/i_platform_driver.hpp`
- `include/arch_nav/driver/i_command_dispatcher.hpp`

## `IPlatformDriver`

```cpp
class IPlatformDriver {
public:
  virtual ICommandDispatcher& dispatcher() = 0;
  virtual void start(context::VehicleContext& context,
                     std::chrono::milliseconds update_period) = 0;
  virtual void stop() = 0;
  virtual ~IPlatformDriver() = default;
};
```

## `ICommandDispatcher`

`execute_*` methods return a `CommandResponse` immediately and complete asynchronously.

Supported hooks include:

| Method | Default | Description |
|--------|---------|-------------|
| `execute_arm` | `NOT_SUPPORTED` | Arm the vehicle |
| `execute_disarm` | `NOT_SUPPORTED` | Disarm the vehicle |
| `execute_takeoff` | `NOT_SUPPORTED` | Takeoff to the given height |
| `execute_land` | `NOT_SUPPORTED` | Land the vehicle |
| `execute_waypoint_following` | `NOT_SUPPORTED` | Follow a sequence of global waypoints |
| `execute_trajectory` | `NOT_SUPPORTED` | Execute a time-parametrised trajectory |
| `execute_set_roi` | `NOT_SUPPORTED` | Point the vehicle toward a geographic location |
| `execute_clear_roi` | `NOT_SUPPORTED` | Cancel an active ROI |
| `stop` | — | Cancel any running asynchronous operation |

All `execute_*` methods return a `CommandResponse` immediately. `ACCEPTED` means the driver has started the operation; `DENIED` means the autopilot rejected it; `NOT_SUPPORTED` means the driver or the requested frame is not handled.

`execute_set_roi` and `execute_clear_roi` are point-in-time commands — the controller calls them and does not wait for a completion callback. If the driver stores ROI state it should write it to `VehicleContext` via `update_roi()` / `clear_roi()`.
