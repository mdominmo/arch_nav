#ifndef ARCH_NAV_CONTROLLER_CONTEXT_UPDATE_HPP_
#define ARCH_NAV_CONTROLLER_CONTEXT_UPDATE_HPP_

#include "arch_nav/context/operation_context.hpp"

namespace arch_nav::controller {

class ContextUpdate {
 public:
  virtual void apply(context::OperationContext& context) = 0;
  virtual ~ContextUpdate() = default;
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV_CONTROLLER_CONTEXT_UPDATE_HPP_
