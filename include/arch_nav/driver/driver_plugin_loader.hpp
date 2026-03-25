#ifndef ARCH_NAV_DRIVER_DRIVER_PLUGIN_LOADER_HPP_
#define ARCH_NAV_DRIVER_DRIVER_PLUGIN_LOADER_HPP_

#include <string>
#include <vector>

namespace arch_nav::platform {

class DriverPluginLoader {
 public:
  DriverPluginLoader() = default;
  ~DriverPluginLoader();

  DriverPluginLoader(const DriverPluginLoader&) = delete;
  DriverPluginLoader& operator=(const DriverPluginLoader&) = delete;

  void load_all();

  const std::vector<std::string>& loaded_plugins() const;

 private:
  void load_library(const std::string& path);

  std::vector<void*>       handles_;
  std::vector<std::string> loaded_plugins_;
};

}  // namespace arch_nav::platform

#endif  // ARCH_NAV_DRIVER_DRIVER_PLUGIN_LOADER_HPP_
