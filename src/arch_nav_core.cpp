#include "arch_nav_core.hpp"

namespace arch_nav {

ArchNavCore::ArchNavCore(platform::ICommandDispatcher& dispatcher)
    : context_()
    , controller_(context_, dispatcher)
    , api_(controller_, context_) {}

ArchNavApi& ArchNavCore::api() {
  return api_;
}

context::VehicleContext& ArchNavCore::context() {
  return context_;
}

}  // namespace arch_nav
