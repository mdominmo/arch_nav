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

std::vector<std::string> DriverRegistry::registered_names() const {
  std::vector<std::string> names;
  names.reserve(factories_.size());
  for (const auto& pair : factories_) {
    names.push_back(pair.first);
  }
  return names;
}

std::size_t DriverRegistry::size() const {
  return factories_.size();
}

}  // namespace arch_nav::platform
