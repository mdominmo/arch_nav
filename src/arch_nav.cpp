#include "arch_nav.hpp"

#include <cstdlib>
#include <stdexcept>
#include <string>

#include <unistd.h>

#include "arch_nav_core.hpp"
#include "config/arch_nav_config_loader.hpp"
#include "platform/driver_plugin_loader.hpp"
#include "platform/driver_registry.hpp"

namespace arch_nav {
namespace {

bool is_readable_file(const std::string& path) {
  return path.empty() == false && access(path.c_str(), R_OK) == 0;
}

std::string resolve_default_config_path() {
  const char* env_path = std::getenv("ARCH_NAV_CONFIG_PATH");
  if (env_path != nullptr && is_readable_file(env_path)) {
    return std::string(env_path);
  }

#ifdef ARCH_NAV_INSTALL_CONFIG_PATH
  if (is_readable_file(ARCH_NAV_INSTALL_CONFIG_PATH)) {
    return ARCH_NAV_INSTALL_CONFIG_PATH;
  }
#endif

#ifdef ARCH_NAV_SOURCE_CONFIG_PATH
  if (is_readable_file(ARCH_NAV_SOURCE_CONFIG_PATH)) {
    return ARCH_NAV_SOURCE_CONFIG_PATH;
  }
#endif

  static const char* kRelativeDefault = "config/arch_nav_config.yaml";
  if (is_readable_file(kRelativeDefault)) {
    return kRelativeDefault;
  }

  throw std::runtime_error(
      "No default navigation config found. Set ARCH_NAV_CONFIG_PATH or call ArchNav::create(path).");
}

}  // namespace

struct ArchNav::Impl {
  platform::DriverPluginLoader           plugin_loader;
  std::unique_ptr<platform::IPlatformDriver> driver;
  std::unique_ptr<ArchNavCore>                core;
};

ArchNav::ArchNav(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}

std::unique_ptr<ArchNav> ArchNav::create() {
  return create(resolve_default_config_path());
}

std::unique_ptr<ArchNav> ArchNav::create(const std::string& config_path) {
  auto config = config::ArchNavConfigLoader::load_from(config_path);

  auto impl = std::make_unique<Impl>();
  impl->plugin_loader.load_all();

  impl->driver = platform::DriverRegistry::instance().create(
      config.driver,
      config.driver_config_path);

  impl->core = std::make_unique<ArchNavCore>(
      impl->driver->dispatcher());

  impl->driver->start(impl->core->context());

  return std::unique_ptr<ArchNav>(new ArchNav(std::move(impl)));
}

ArchNavApi& ArchNav::api() {
  return impl_->core->api();
}

ArchNav::~ArchNav() {
  if (impl_ && impl_->driver) {
    impl_->driver->stop();
  }
}

}  // namespace arch_nav
