#include "arch_nav_core.hpp"

namespace arch_nav {

ArchNavCore::ArchNavCore(platform::ICommandDispatcher& dispatcher)
    : vehicle_context_()
    , operation_context_()
    , controller_(vehicle_context_, operation_context_, dispatcher)
    , api_(controller_, vehicle_context_, operation_context_) {}

ArchNavApi& ArchNavCore::api() {
  return api_;
}

context::VehicleContext& ArchNavCore::vehicle_context() {
  return vehicle_context_;
}

context::OperationContext& ArchNavCore::operation_context() {
  return operation_context_;
}

}  // namespace arch_nav
