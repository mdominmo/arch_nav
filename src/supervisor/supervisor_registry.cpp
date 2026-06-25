#include "arch_nav/supervisor/supervisor_registry.hpp"

#include <stdexcept>

namespace arch_nav::supervisor {

SupervisorRegistry& SupervisorRegistry::instance() {
  static SupervisorRegistry registry;
  return registry;
}

void SupervisorRegistry::register_supervisor(const std::string& name,
                                             SupervisorFactory factory) {
  factories_[name] = std::move(factory);
}

std::unique_ptr<ISupervisor> SupervisorRegistry::create(
    const std::string& name,
    const std::string& config_path) const {
  auto it = factories_.find(name);
  if (it == factories_.end()) {
    throw std::runtime_error("Supervisor not registered: '" + name + "'");
  }
  return it->second(config_path);
}

std::vector<std::string> SupervisorRegistry::registered_names() const {
  std::vector<std::string> names;
  names.reserve(factories_.size());
  for (const auto& pair : factories_) {
    names.push_back(pair.first);
  }
  return names;
}

std::size_t SupervisorRegistry::size() const {
  return factories_.size();
}

}  // namespace arch_nav::supervisor
