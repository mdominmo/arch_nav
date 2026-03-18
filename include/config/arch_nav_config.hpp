#ifndef ARCH_NAV__CONFIG__ARCH_NAV_CONFIG_HPP_
#define ARCH_NAV__CONFIG__ARCH_NAV_CONFIG_HPP_

#include <string>

#include "config/local_planner_config.hpp"

namespace arch_nav::config {

struct ArchNavConfig {
  std::string        driver;
  std::string        driver_config_path;
  LocalPlannerConfig local_planner;
};

}  // namespace arch_nav::config

#endif  // ARCH_NAV__CONFIG__ARCH_NAV_CONFIG_HPP_
