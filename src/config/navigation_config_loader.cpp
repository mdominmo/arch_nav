#include "config/navigation_config_loader.hpp"

#include <yaml-cpp/yaml.h>

namespace arch_nav::config {

NavigationConfig NavigationConfigLoader::load_from(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  NavigationConfig config;

  config.driver = root["driver"].as<std::string>();
  if (root["driver_config_path"]) {
    config.driver_config_path = root["driver_config_path"].as<std::string>();
  }

  const YAML::Node planner = root["local_planner"];
  config.local_planner.max_linear_velocity       = planner["max_linear_velocity"].as<double>();
  config.local_planner.max_linear_acceleration   = planner["max_linear_acceleration"].as<double>();
  config.local_planner.max_angular_velocity      = planner["max_angular_velocity"].as<double>();
  config.local_planner.max_angular_acceleration  = planner["max_angular_acceleration"].as<double>();
  config.local_planner.max_vertical_velocity     = planner["max_vertical_velocity"].as<double>();
  config.local_planner.max_vertical_acceleration = planner["max_vertical_acceleration"].as<double>();
  config.local_planner.time_step                 = planner["time_step"].as<double>();
  config.local_planner.land_descent_velocity     = planner["land_descent_velocity"].as<double>(0.4);

  return config;
}

}  // namespace arch_nav::config
