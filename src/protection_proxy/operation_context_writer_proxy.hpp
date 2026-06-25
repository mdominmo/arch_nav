#ifndef ARCH_NAV_PROTECTION_PROXY_OPERATION_CONTEXT_WRITER_PROXY_HPP_
#define ARCH_NAV_PROTECTION_PROXY_OPERATION_CONTEXT_WRITER_PROXY_HPP_

#include "arch_nav/context/i_operation_context_writer.hpp"
#include "arch_nav/context/operation_context.hpp"

namespace arch_nav::context {

class OperationContextWriterProxy final : public IOperationContextWriter {
 public:
  explicit OperationContextWriterProxy(OperationContext& context)
      : context_(context) {}

  void set_obstacle(const operation::Obstacle& obstacle) override {
    context_.set_obstacle(obstacle);
  }

  void remove_obstacle(const std::string& id) override {
    context_.remove_obstacle(id);
  }

  void clear_obstacles() override {
    context_.clear_obstacles();
  }

 private:
  OperationContext& context_;
};

}  // namespace arch_nav::context

#endif  // ARCH_NAV_PROTECTION_PROXY_OPERATION_CONTEXT_WRITER_PROXY_HPP_
