# Build & Test

## Configure and build

```bash
cd arch_nav_ws
cmake -B src/arch_nav/build -S src/arch_nav
cmake --build src/arch_nav/build -j$(nproc)
```

If using external drivers:

```bash
export ARCH_NAV_DRIVERS=$(pwd)/src/arch_nav_mavsdk_driver
```

## Run tests

```bash
cd src/arch_nav/build
ctest --output-on-failure
```

or:

```bash
cmake --build src/arch_nav/build --target test
```

## Documentation preview

From `src/arch_nav`:

```bash
mkdocs serve
```
