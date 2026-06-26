#ifndef ARCH_NAV_CONTEXT_I_OPERATION_CONTEXT_READER_HPP_
#define ARCH_NAV_CONTEXT_I_OPERATION_CONTEXT_READER_HPP_

#include <memory>

#include "arch_nav/context/i_context_reader.hpp"
#include "arch_nav/descriptor/operation_descriptor.hpp"

namespace arch_nav::context {

class IOperationContextReader : public IContextReader {
 public:
  virtual std::shared_ptr<const descriptor::OperationDescriptor>
      current_descriptor() const = 0;
};

}  // namespace arch_nav::context

#endif  // ARCH_NAV_CONTEXT_I_OPERATION_CONTEXT_READER_HPP_
