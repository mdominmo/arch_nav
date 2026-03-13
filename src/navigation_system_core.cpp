#include "navigation_system_core.hpp"

namespace arch_nav {

NavigationSystemCore::NavigationSystemCore(
    dispatchers::ICommandDispatcher& dispatcher,
    const config::LocalPlannerConfig& config)
    : context_()
    , planner_(
          config.max_linear_velocity,
          config.max_linear_acceleration,
          config.max_angular_velocity,
          config.max_angular_acceleration,
          config.max_vertical_velocity,
          config.max_vertical_acceleration,
          config.time_step,
          config.land_descent_velocity)
    , controller_(context_, planner_, dispatcher)
    , api_(controller_) {}

NavigationApi& NavigationSystemCore::api() {
  return api_;
}

context::VehicleContext& NavigationSystemCore::context() {
  return context_;
}

}  // namespace arch_nav
