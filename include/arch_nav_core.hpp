#ifndef ARCH_NAV__ARCH_NAV_CORE_HPP_
#define ARCH_NAV__ARCH_NAV_CORE_HPP_

#include "core/context/vehicle_context.hpp"
#include "core/controller/operational_controller.hpp"
#include "dispatchers/i_command_dispatcher.hpp"
#include "arch_nav_api.hpp"

namespace arch_nav {

class ArchNavCore {
 public:
  explicit ArchNavCore(dispatchers::ICommandDispatcher& dispatcher);

  ArchNavApi& api();
  context::VehicleContext& context();

 private:
  context::VehicleContext           context_;
  controller::OperationalController controller_;
  ArchNavApi                        api_;
};

}  // namespace arch_nav

#endif  // ARCH_NAV__ARCH_NAV_CORE_HPP_
