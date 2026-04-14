# Writing Drivers

## 1. Implement the interfaces

Implement:

- `arch_nav::platform::IPlatformDriver`
- `arch_nav::platform::ICommandDispatcher`

The platform driver owns telemetry ingestion and lifecycle.
The dispatcher owns command execution.

## 2. Register the driver

Use `DriverRegistry::instance().register_driver("name", factory)`.

Registration is typically done in a translation unit with a static registration object.

## 3. Build as plugin

Build your driver as shared library:

- naming convention: `libarch_nav_<driver>.so`
- install destination: `.../lib/arch_nav/drivers`

## 4. Select at runtime

Set:

- `ARCH_NAV_DRIVER=<driver_name>`
- `ARCH_NAV_DRIVER_CONFIG=<path_to_driver_config>` (optional)

If only one driver is loaded, explicit selection is optional.
