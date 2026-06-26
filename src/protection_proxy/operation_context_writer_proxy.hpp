#ifndef ARCH_NAV_PROTECTION_PROXY_OPERATION_CONTEXT_WRITER_PROXY_HPP_
#define ARCH_NAV_PROTECTION_PROXY_OPERATION_CONTEXT_WRITER_PROXY_HPP_

#include "arch_nav/context/i_operation_context_writer.hpp"
#include "arch_nav/context/operation_context.hpp"

namespace arch_nav::context {

class OperationContextWriterProxy final : public IOperationContextWriter {
 public:
  explicit OperationContextWriterProxy(OperationContext& context)
      : context_(context) {}

  void set_current_descriptor(
      std::shared_ptr<descriptor::OperationDescriptor> desc) override {
    context_.set_current_descriptor(std::move(desc));
  }

  void clear_current_descriptor() override {
    context_.clear_current_descriptor();
  }

 private:
  OperationContext& context_;
};

}  // namespace arch_nav::context

#endif  // ARCH_NAV_PROTECTION_PROXY_OPERATION_CONTEXT_WRITER_PROXY_HPP_
