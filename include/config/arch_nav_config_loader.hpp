#ifndef ARCH_NAV__CONFIG__ARCH_NAV_CONFIG_LOADER_HPP_
#define ARCH_NAV__CONFIG__ARCH_NAV_CONFIG_LOADER_HPP_

#include <string>

#include "config/arch_nav_config.hpp"

namespace arch_nav::config {

class ArchNavConfigLoader {
 public:
  static ArchNavConfig load_from(const std::string& path);
};

}  // namespace arch_nav::config

#endif  // ARCH_NAV__CONFIG__ARCH_NAV_CONFIG_LOADER_HPP_
