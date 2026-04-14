#ifndef ARCH_NAV_ARCH_NAV_CORE_HPP_
#define ARCH_NAV_ARCH_NAV_CORE_HPP_

#include "arch_nav/context/vehicle_context.hpp"
#include "controller/operational_controller.hpp"
#include "arch_nav/driver/i_command_dispatcher.hpp"
#include "arch_nav/arch_nav_api.hpp"

namespace arch_nav {

class ArchNavCore {
 public:
  explicit ArchNavCore(platform::ICommandDispatcher& dispatcher);

  ArchNavApi& api();
  context::VehicleContext& context();

 private:
  context::VehicleContext           context_;
  controller::OperationalController controller_;
  ArchNavApi                        api_;
};

}  // namespace arch_nav

#endif  // ARCH_NAV_ARCH_NAV_CORE_HPP_
