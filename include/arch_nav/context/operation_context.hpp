#ifndef ARCH_NAV_CONTEXT_OPERATION_CONTEXT_HPP_
#define ARCH_NAV_CONTEXT_OPERATION_CONTEXT_HPP_

#include <memory>
#include <mutex>

#include "arch_nav/descriptor/operation_descriptor.hpp"

namespace arch_nav::context {

class OperationContext {
 public:
  OperationContext();

  std::shared_ptr<descriptor::OperationDescriptor> current_descriptor() const;
  void set_current_descriptor(
      std::shared_ptr<descriptor::OperationDescriptor> desc);
  void clear_current_descriptor();
  std::unique_ptr<descriptor::OperationDescriptor> snapshot_descriptor() const;

 private:
  mutable std::mutex descriptor_mutex_;
  std::shared_ptr<descriptor::OperationDescriptor> current_descriptor_;
};

}  // namespace arch_nav::context

#endif  // ARCH_NAV_CONTEXT_OPERATION_CONTEXT_HPP_
