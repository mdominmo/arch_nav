#ifndef ARCH_NAV_DRIVER_I_PLATFORM_DRIVER_HPP_
#define ARCH_NAV_DRIVER_I_PLATFORM_DRIVER_HPP_

#include <chrono>

#include "arch_nav/driver/i_command_dispatcher.hpp"

namespace arch_nav::context {
class VehicleContext;
}

namespace arch_nav::platform {

class IPlatformDriver {
 public:
  virtual ~IPlatformDriver() = default;

  virtual ICommandDispatcher& dispatcher() = 0;

  virtual void start(context::VehicleContext& context,
                     std::chrono::milliseconds update_period) = 0;

  virtual void stop() = 0;
};

}  // namespace arch_nav::platform

#endif  // ARCH_NAV_DRIVER_I_PLATFORM_DRIVER_HPP_
