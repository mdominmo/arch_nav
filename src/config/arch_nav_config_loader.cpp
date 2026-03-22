#include "config/arch_nav_config_loader.hpp"

#include <yaml-cpp/yaml.h>

namespace arch_nav::config {

ArchNavConfig ArchNavConfigLoader::load_from(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  ArchNavConfig config;

  config.driver = root["driver"].as<std::string>();
  if (root["driver_config_path"]) {
    config.driver_config_path = root["driver_config_path"].as<std::string>();
  }

  return config;
}

}  // namespace arch_nav::config
