#ifndef ARCH_NAV_SUPERVISOR_SUPERVISOR_REGISTRY_HPP_
#define ARCH_NAV_SUPERVISOR_SUPERVISOR_REGISTRY_HPP_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "arch_nav/supervisor/i_supervisor.hpp"

namespace arch_nav::supervisor {

using SupervisorFactory =
    std::function<std::unique_ptr<ISupervisor>(const std::string& config_path)>;

class SupervisorRegistry {
 public:
  static SupervisorRegistry& instance();

  void register_supervisor(const std::string& name, SupervisorFactory factory);

  std::unique_ptr<ISupervisor> create(
      const std::string& name,
      const std::string& config_path) const;

  std::vector<std::string> registered_names() const;
  std::size_t size() const;

 private:
  SupervisorRegistry() = default;
  std::unordered_map<std::string, SupervisorFactory> factories_;
};

}  // namespace arch_nav::supervisor

#endif  // ARCH_NAV_SUPERVISOR_SUPERVISOR_REGISTRY_HPP_
