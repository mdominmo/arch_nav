#ifndef ARCH_NAV_DRIVER_DRIVER_REGISTRY_HPP_
#define ARCH_NAV_DRIVER_DRIVER_REGISTRY_HPP_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "arch_nav/driver/i_platform_driver.hpp"

namespace arch_nav::platform {

using DriverFactory =
    std::function<std::unique_ptr<IPlatformDriver>(const std::string& config_path)>;

class DriverRegistry {
 public:
  static DriverRegistry& instance();

  void register_driver(const std::string& name, DriverFactory factory);

  std::unique_ptr<IPlatformDriver> create(
      const std::string& name,
      const std::string& config_path) const;

  std::vector<std::string> registered_names() const;
  std::size_t size() const;

 private:
  DriverRegistry() = default;
  std::unordered_map<std::string, DriverFactory> factories_;
};

}  // namespace arch_nav::platform

#endif  // ARCH_NAV_DRIVER_DRIVER_REGISTRY_HPP_
