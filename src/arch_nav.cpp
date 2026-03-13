#include "arch_nav.hpp"

#include "config/navigation_config_loader.hpp"
#include "navigation_system_core.hpp"
#include "platform/driver_registry.hpp"

namespace arch_nav {

struct ArchNav::Impl {
  std::unique_ptr<platform::IPlatformDriver> driver;
  std::unique_ptr<NavigationSystemCore>      core;
};

ArchNav::ArchNav(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}

std::unique_ptr<ArchNav> ArchNav::create(const std::string& config_path) {
  auto config = config::NavigationConfigLoader::load_from(config_path);

  auto driver = platform::DriverRegistry::instance().create(
      config.driver,
      config.driver_config_path);

  auto core = std::make_unique<NavigationSystemCore>(
      driver->dispatcher(),
      config.local_planner);

  driver->start(core->context());

  auto impl = std::make_unique<Impl>(Impl{std::move(driver), std::move(core)});
  return std::unique_ptr<ArchNav>(new ArchNav(std::move(impl)));
}

NavigationApi& ArchNav::api() {
  return impl_->core->api();
}

ArchNav::~ArchNav() {
  impl_->driver->stop();
}

}  // namespace arch_nav
