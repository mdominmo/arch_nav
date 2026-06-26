#ifndef ARCH_NAV_CONTEXT_I_OPERATION_CONTEXT_WRITER_HPP_
#define ARCH_NAV_CONTEXT_I_OPERATION_CONTEXT_WRITER_HPP_

#include <memory>

#include "arch_nav/context/i_context_writer.hpp"
#include "arch_nav/descriptor/operation_descriptor.hpp"

namespace arch_nav::context {

class IOperationContextWriter : public IContextWriter {
 public:
  virtual void set_current_descriptor(
      std::shared_ptr<descriptor::OperationDescriptor> desc) = 0;
  virtual void clear_current_descriptor() = 0;
};

}  // namespace arch_nav::context

#endif  // ARCH_NAV_CONTEXT_I_OPERATION_CONTEXT_WRITER_HPP_
