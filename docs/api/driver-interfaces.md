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

- `execute_takeoff`
- `execute_land`
- `execute_waypoint_following`
- `execute_trajectory`
- `execute_arm`
- `execute_disarm`
- `stop`

Each method can remain `NOT_SUPPORTED` by default if a driver does not implement it.
