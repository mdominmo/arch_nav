#ifndef ARCH_NAV_CONTROLLER_NAVIGATION_TASK_FACTORY_HPP_
#define ARCH_NAV_CONTROLLER_NAVIGATION_TASK_FACTORY_HPP_

#include <memory>

#include "controller/navigation_task.hpp"
#include "arch_nav/descriptor/operation_descriptor.hpp"

namespace arch_nav::controller {

class NavigationTaskFactory {
 public:
  static std::unique_ptr<NavigationTask> create_from_descriptor(
      descriptor::OperationDescriptor& desc);
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV_CONTROLLER_NAVIGATION_TASK_FACTORY_HPP_
