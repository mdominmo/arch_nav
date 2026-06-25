#ifndef ARCH_NAV_SUPERVISOR_SUPERVISOR_PLUGIN_LOADER_HPP_
#define ARCH_NAV_SUPERVISOR_SUPERVISOR_PLUGIN_LOADER_HPP_

#include <string>
#include <vector>

namespace arch_nav::supervisor {

class SupervisorPluginLoader {
 public:
  SupervisorPluginLoader() = default;
  ~SupervisorPluginLoader();

  SupervisorPluginLoader(const SupervisorPluginLoader&) = delete;
  SupervisorPluginLoader& operator=(const SupervisorPluginLoader&) = delete;

  void load_all();

  const std::vector<std::string>& loaded_plugins() const;

 private:
  void load_library(const std::string& path);

  std::vector<void*>       handles_;
  std::vector<std::string> loaded_plugins_;
};

}  // namespace arch_nav::supervisor

#endif  // ARCH_NAV_SUPERVISOR_SUPERVISOR_PLUGIN_LOADER_HPP_
