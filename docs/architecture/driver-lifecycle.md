# Driver Lifecycle

`arch_nav` separates responsibilities between two driver interfaces:

- `IPlatformDriver`: telemetry/startup/shutdown boundary.
- `ICommandDispatcher`: command execution boundary.

## Startup

1. Plugin `.so` is loaded.
2. Driver factory registers under a unique name.
3. `IPlatformDriver::start(context, update_period)` begins telemetry feed.
4. `dispatcher()` exposes the command interface used by the kernel.

## Operation execution

1. Controller requests `execute_takeoff/land/waypoint_following/...`.
2. Driver returns quickly with `CommandResponse` (`ACCEPTED`, `DENIED`, etc.).
3. Operation completion is signaled asynchronously via callback.

## Shutdown

1. Kernel calls `IPlatformDriver::stop()`.
2. Driver stops command execution and telemetry threads safely.
3. Resources and subscriptions are released deterministically.

## Design constraints

- Operations must be non-blocking from the kernel perspective.
- Completion callbacks must be safe under reentrancy.
- `stop()` must be idempotent and thread-safe.
