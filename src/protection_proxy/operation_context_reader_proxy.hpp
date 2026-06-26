#ifndef ARCH_NAV_PROTECTION_PROXY_OPERATION_CONTEXT_READER_PROXY_HPP_
#define ARCH_NAV_PROTECTION_PROXY_OPERATION_CONTEXT_READER_PROXY_HPP_

#include "arch_nav/context/i_operation_context_reader.hpp"
#include "arch_nav/context/operation_context.hpp"

namespace arch_nav::context {

class OperationContextReaderProxy final : public IOperationContextReader {
 public:
  explicit OperationContextReaderProxy(OperationContext& context)
      : context_(context) {}

  std::shared_ptr<const descriptor::OperationDescriptor>
      current_descriptor() const override {
    return context_.current_descriptor();
  }

 private:
  OperationContext& context_;
};

}  // namespace arch_nav::context

#endif  // ARCH_NAV_PROTECTION_PROXY_OPERATION_CONTEXT_READER_PROXY_HPP_
