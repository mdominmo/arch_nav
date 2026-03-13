#ifndef ARCH_NAV__NAVIGATION_SYSTEM_CORE_HPP_
#define ARCH_NAV__NAVIGATION_SYSTEM_CORE_HPP_

#include "config/local_planner_config.hpp"
#include "core/context/vehicle_context.hpp"
#include "core/controller/operational_controller.hpp"
#include "core/planner/kinematic_point_to_point_local_planner.hpp"
#include "dispatchers/i_command_dispatcher.hpp"
#include "navigation_api.hpp"

namespace arch_nav {

class NavigationSystemCore {
 public:
  explicit NavigationSystemCore(
      dispatchers::ICommandDispatcher& dispatcher,
      const config::LocalPlannerConfig& config);

  NavigationApi& api();
  context::VehicleContext& context();

 private:
  context::VehicleContext                    context_;
  planner::KinematicPointToPointLocalPlanner planner_;
  controller::OperationalController          controller_;
  NavigationApi                              api_;
};

}  // namespace arch_nav

#endif  // ARCH_NAV__NAVIGATION_SYSTEM_CORE_HPP_
