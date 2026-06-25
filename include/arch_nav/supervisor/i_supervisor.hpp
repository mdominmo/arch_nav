#ifndef ARCH_NAV_SUPERVISOR_I_SUPERVISOR_HPP_
#define ARCH_NAV_SUPERVISOR_I_SUPERVISOR_HPP_

#include "arch_nav/context/i_vehicle_context_reader.hpp"
#include "arch_nav/controller/i_operational_controller.hpp"

namespace arch_nav::supervisor {

class SupervisorChain;

class ISupervisor {
 public:
  virtual ~ISupervisor() = default;

  virtual void start(
      context::IVehicleContextReader& vehicle_reader,
      SupervisorChain& chain) = 0;

  virtual void stop() = 0;

  virtual void execute(controller::IOperationalController& controller) = 0;
};

}  // namespace arch_nav::supervisor

#endif  // ARCH_NAV_SUPERVISOR_I_SUPERVISOR_HPP_
