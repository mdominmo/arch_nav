# `arch_nav::ArchNav`

Header: `include/arch_nav/arch_nav.hpp`

## Purpose

Top-level object that owns:

- plugin loader
- selected platform driver
- kernel core (`ArchNavCore`)

## Public API

```cpp
static std::unique_ptr<ArchNav> create(
    std::chrono::milliseconds context_update_period = std::chrono::milliseconds(20));

ArchNavApi& api();
~ArchNav();
```

## Notes

- Driver selection is resolved at runtime.
- Driver configuration is passed through environment variables:
  - `ARCH_NAV_DRIVER`
  - `ARCH_NAV_DRIVER_CONFIG`
