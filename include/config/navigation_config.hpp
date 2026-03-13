#ifndef NAVIGATION_STATE_REGISTER__CONFIG__NAVIGATION_CONFIG_HPP_
#define NAVIGATION_STATE_REGISTER__CONFIG__NAVIGATION_CONFIG_HPP_

#include <string>

#include "config/local_planner_config.hpp"

namespace arch_nav::config {

struct NavigationConfig {
  std::string        driver;
  std::string        driver_config_path;
  LocalPlannerConfig local_planner;
};

}  // namespace arch_nav::config

#endif  // NAVIGATION_STATE_REGISTER__CONFIG__NAVIGATION_CONFIG_HPP_
