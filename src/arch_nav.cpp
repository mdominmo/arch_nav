#include "arch_nav/arch_nav.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "arch_nav_core.hpp"
#include "arch_nav/driver/driver_plugin_loader.hpp"
#include "arch_nav/driver/driver_registry.hpp"
#include "arch_nav/supervisor/supervisor_registry.hpp"
#include "supervisor/supervisor_plugin_loader.hpp"

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

std::string resolve_supervisor_config(const std::string& name) {
  std::string env_name = "ARCH_NAV_SUPERVISOR_" + name + "_CONFIG";
  for (auto& c : env_name) c = static_cast<char>(std::toupper(c));
  const char* env = std::getenv(env_name.c_str());
  if (env != nullptr && env[0] != '\0') {
    return std::string(env);
  }
  return {};
}

}  // namespace

struct ArchNav::Impl {
  platform::DriverPluginLoader              plugin_loader;
  supervisor::SupervisorPluginLoader        supervisor_loader;
  std::unique_ptr<platform::IPlatformDriver> driver;
  std::unique_ptr<ArchNavCore>               core;
  std::vector<std::unique_ptr<supervisor::ISupervisor>> supervisors;
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

  impl->driver->start(impl->core->vehicle_context_writer(),
                      context_update_period);

  impl->supervisor_loader.load_all();
  auto& supervisor_registry = supervisor::SupervisorRegistry::instance();
  for (const auto& name : supervisor_registry.registered_names()) {
    auto config = resolve_supervisor_config(name);
    auto sv = supervisor_registry.create(name, config);
    sv->start(impl->core->vehicle_context_reader(),
              impl->core->operation_context_reader(),
              impl->core->supervisor_chain());
    impl->supervisors.push_back(std::move(sv));
  }

  return std::unique_ptr<ArchNav>(new ArchNav(std::move(impl)));
}

ArchNavApi& ArchNav::api() {
  return impl_->core->api();
}

ArchNav::~ArchNav() {
  if (impl_) {
    for (auto& sv : impl_->supervisors) {
      std::cerr << "[arch_nav] stopping supervisor..." << std::endl;
      sv->stop();
      std::cerr << "[arch_nav] supervisor stopped" << std::endl;
    }
    if (impl_->driver) {
      std::cerr << "[arch_nav] stopping driver..." << std::endl;
      impl_->driver->stop();
      std::cerr << "[arch_nav] driver stopped" << std::endl;
    }
  }
}

}  // namespace arch_nav
