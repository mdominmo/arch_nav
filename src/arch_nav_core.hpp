#ifndef ARCH_NAV_ARCH_NAV_CORE_HPP_
#define ARCH_NAV_ARCH_NAV_CORE_HPP_

#include "arch_nav/context/vehicle_context.hpp"
#include "arch_nav/context/operation_context.hpp"
#include "controller/operational_controller.hpp"
#include "arch_nav/driver/i_command_dispatcher.hpp"
#include "arch_nav/arch_nav_api.hpp"
#include "arch_nav/supervisor/supervisor_chain.hpp"
#include "protection_proxy/vehicle_context_writer_proxy.hpp"
#include "protection_proxy/vehicle_context_reader_proxy.hpp"
#include "protection_proxy/operation_context_reader_proxy.hpp"
#include "protection_proxy/operation_context_writer_proxy.hpp"

namespace arch_nav {

class ArchNavCore {
 public:
  explicit ArchNavCore(platform::ICommandDispatcher& dispatcher);

  ArchNavApi& api();
  context::VehicleContext& vehicle_context();
  context::OperationContext& operation_context();
  context::IVehicleContextWriter& vehicle_context_writer();
  context::IVehicleContextReader& vehicle_context_reader();
  context::IOperationContextReader& operation_context_reader();
  controller::IOperationalController& operational_controller();
  supervisor::SupervisorChain& supervisor_chain();

 private:
  context::VehicleContext              vehicle_context_;
  context::OperationContext            operation_context_;
  context::VehicleContextWriterProxy     vehicle_context_writer_proxy_;
  context::VehicleContextReaderProxy     vehicle_context_reader_proxy_;
  context::OperationContextReaderProxy   operation_context_reader_proxy_;
  context::OperationContextWriterProxy   operation_context_writer_proxy_;
  controller::OperationalController      controller_;
  supervisor::SupervisorChain            supervisor_chain_;
  ArchNavApi                           api_;
};

}  // namespace arch_nav

#endif  // ARCH_NAV_ARCH_NAV_CORE_HPP_
