#ifndef ARCH_NAV_CONTROLLER_NAVIGATION_TASK_MEMENTO_HPP_
#define ARCH_NAV_CONTROLLER_NAVIGATION_TASK_MEMENTO_HPP_

#include <memory>

namespace arch_nav::controller {

class NavigationTask;

class NavigationTaskMemento {
 public:
  virtual ~NavigationTaskMemento() = default;
  virtual std::unique_ptr<NavigationTask> reconstruct() const = 0;
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV_CONTROLLER_NAVIGATION_TASK_MEMENTO_HPP_
