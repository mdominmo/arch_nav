#include "arch_nav.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "config/arch_nav_config_loader.hpp"
#include "arch_nav_core.hpp"
#include "platform/driver_registry.hpp"

namespace arch_nav {

struct ArchNav::Impl {
  std::unique_ptr<platform::IPlatformDriver> driver;
  std::unique_ptr<ArchNavCore>      core;
};

ArchNav::ArchNav(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}

std::unique_ptr<ArchNav> ArchNav::create() {
  const auto config_path =
      ament_index_cpp::get_package_share_directory("arch-nav") + "/config/navigation_config.yaml";
  return create(config_path);
}

std::unique_ptr<ArchNav> ArchNav::create(const std::string& config_path) {
  auto config = config::ArchNavConfigLoader::load_from(config_path);

  auto driver = platform::DriverRegistry::instance().create(
      config.driver,
      config.driver_config_path);

  auto core = std::make_unique<ArchNavCore>(
      driver->dispatcher(),
      config.local_planner);

  driver->start(core->context());

  auto impl = std::make_unique<Impl>(Impl{std::move(driver), std::move(core)});
  return std::unique_ptr<ArchNav>(new ArchNav(std::move(impl)));
}

ArchNavApi& ArchNav::api() {
  return impl_->core->api();
}

ArchNav::~ArchNav() {
  impl_->driver->stop();
}

}  // namespace arch_nav
