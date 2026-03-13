#include "platform/driver_registry.hpp"

#include <stdexcept>

namespace arch_nav::platform {

DriverRegistry& DriverRegistry::instance() {
  static DriverRegistry registry;
  return registry;
}

void DriverRegistry::register_driver(const std::string& name, DriverFactory factory) {
  factories_[name] = std::move(factory);
}

std::unique_ptr<IPlatformDriver> DriverRegistry::create(
    const std::string& name,
    const std::string& config_path) const
{
  auto it = factories_.find(name);
  if (it == factories_.end()) {
    throw std::runtime_error("Driver not registered: '" + name + "'");
  }
  return it->second(config_path);
}

}  // namespace arch_nav::platform
