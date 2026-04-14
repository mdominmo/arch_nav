#include "arch_nav/arch_nav.hpp"

#include <cstdlib>
#include <stdexcept>
#include <string>

#include "arch_nav_core.hpp"
#include "arch_nav/driver/driver_plugin_loader.hpp"
#include "arch_nav/driver/driver_registry.hpp"

namespace arch_nav {

// TODO: make configurable (env var or config file) instead of a hardcoded default
static constexpr std::chrono::milliseconds kDefaultContextUpdatePeriod{20};

namespace {

std::string resolve_driver_name(const platform::DriverRegistry& registry) {
  const char* env = std::getenv("ARCH_NAV_DRIVER");
  if (env != nullptr && env[0] != '\0') {
    return std::string(env);
  }

  if (registry.size() == 0) {
    throw std::runtime_error(
        "No drivers registered. Install a driver plugin or set ARCH_NAV_DRIVERS at build time.");
  }

  if (registry.size() == 1) {
    return registry.registered_names().front();
  }

  auto names = registry.registered_names();
  std::string list;
  for (const auto& n : names) {
    if (!list.empty()) list += ", ";
    list += n;
  }
  throw std::runtime_error(
      "Multiple drivers registered (" + list +
      "). Set ARCH_NAV_DRIVER to select one.");
}

std::string resolve_driver_config() {
  const char* env = std::getenv("ARCH_NAV_DRIVER_CONFIG");
  if (env != nullptr && env[0] != '\0') {
    return std::string(env);
  }
  return {};
}

}  // namespace

struct ArchNav::Impl {
  platform::DriverPluginLoader           plugin_loader;
  std::unique_ptr<platform::IPlatformDriver> driver;
  std::unique_ptr<ArchNavCore>                core;
};

ArchNav::ArchNav(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}

std::unique_ptr<ArchNav> ArchNav::create(std::chrono::milliseconds context_update_period) {
  auto impl = std::make_unique<Impl>();
  impl->plugin_loader.load_all();

  auto& registry = platform::DriverRegistry::instance();
  const auto driver_name   = resolve_driver_name(registry);
  const auto driver_config = resolve_driver_config();

  impl->driver = registry.create(driver_name, driver_config);

  impl->core = std::make_unique<ArchNavCore>(
      impl->driver->dispatcher());

  impl->driver->start(impl->core->context(), context_update_period);

  return std::unique_ptr<ArchNav>(new ArchNav(std::move(impl)));
}

ArchNavApi& ArchNav::api() {
  return impl_->core->api();
}

ArchNav::~ArchNav() {
  if (impl_ && impl_->driver) {
    impl_->driver->stop();
  }
}

}  // namespace arch_nav
