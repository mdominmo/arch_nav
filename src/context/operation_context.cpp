#include "arch_nav/context/operation_context.hpp"

namespace arch_nav::context {

OperationContext::OperationContext() = default;

std::shared_ptr<descriptor::OperationDescriptor>
OperationContext::current_descriptor() const {
  std::lock_guard<std::mutex> lock(descriptor_mutex_);
  return current_descriptor_;
}

void OperationContext::set_current_descriptor(
    std::shared_ptr<descriptor::OperationDescriptor> desc) {
  std::lock_guard<std::mutex> lock(descriptor_mutex_);
  current_descriptor_ = std::move(desc);
}

void OperationContext::clear_current_descriptor() {
  std::lock_guard<std::mutex> lock(descriptor_mutex_);
  current_descriptor_.reset();
}

std::unique_ptr<descriptor::OperationDescriptor>
OperationContext::snapshot_descriptor() const {
  std::lock_guard<std::mutex> lock(descriptor_mutex_);
  if (!current_descriptor_) return nullptr;
  return current_descriptor_->snapshot();
}

}  // namespace arch_nav::context
