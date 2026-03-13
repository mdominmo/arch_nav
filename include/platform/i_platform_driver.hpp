#ifndef ARCH_NAV__PLATFORM__I_PLATFORM_DRIVER_HPP_
#define ARCH_NAV__PLATFORM__I_PLATFORM_DRIVER_HPP_

#include "core/context/vehicle_context.hpp"
#include "dispatchers/i_command_dispatcher.hpp"

namespace arch_nav::platform {

class IPlatformDriver {
 public:
  virtual ~IPlatformDriver() = default;

  virtual dispatchers::ICommandDispatcher& dispatcher() = 0;

  virtual void start(context::VehicleContext& context) = 0;

  virtual void stop() = 0;
};

}  // namespace arch_nav::platform

#endif  // ARCH_NAV__PLATFORM__I_PLATFORM_DRIVER_HPP_
