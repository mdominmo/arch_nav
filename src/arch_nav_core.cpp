#include "arch_nav_core.hpp"

namespace arch_nav {

ArchNavCore::ArchNavCore(platform::ICommandDispatcher& dispatcher)
    : vehicle_context_()
    , operation_context_()
    , vehicle_context_writer_proxy_(vehicle_context_)
    , vehicle_context_reader_proxy_(vehicle_context_)
    , operation_context_reader_proxy_(operation_context_)
    , operation_context_writer_proxy_(operation_context_)
    , controller_(operation_context_, dispatcher)
    , supervisor_chain_(controller_, operation_context_writer_proxy_)
    , api_(controller_, vehicle_context_) {
  vehicle_context_.subscribe_vehicle_status(
      [this](const vehicle::VehicleStatus& status) {
        controller_.on_vehicle_status_update(status);
      });
}

ArchNavApi& ArchNavCore::api() {
  return api_;
}

context::VehicleContext& ArchNavCore::vehicle_context() {
  return vehicle_context_;
}

context::OperationContext& ArchNavCore::operation_context() {
  return operation_context_;
}

context::IVehicleContextWriter& ArchNavCore::vehicle_context_writer() {
  return vehicle_context_writer_proxy_;
}

context::IVehicleContextReader& ArchNavCore::vehicle_context_reader() {
  return vehicle_context_reader_proxy_;
}

context::IOperationContextReader& ArchNavCore::operation_context_reader() {
  return operation_context_reader_proxy_;
}

controller::IOperationalController& ArchNavCore::operational_controller() {
  return controller_;
}

supervisor::SupervisorChain& ArchNavCore::supervisor_chain() {
  return supervisor_chain_;
}

}  // namespace arch_nav
