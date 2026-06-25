#ifndef ARCH_NAV_CONTEXT_I_OPERATION_CONTEXT_WRITER_HPP_
#define ARCH_NAV_CONTEXT_I_OPERATION_CONTEXT_WRITER_HPP_

#include <string>

#include "arch_nav/context/i_context_writer.hpp"
#include "arch_nav/model/operation/obstacle.hpp"

namespace arch_nav::context {

class IOperationContextWriter : public IContextWriter {
 public:
  virtual void set_obstacle(const operation::Obstacle& obstacle) = 0;
  virtual void remove_obstacle(const std::string& id) = 0;
  virtual void clear_obstacles() = 0;
};

}  // namespace arch_nav::context

#endif  // ARCH_NAV_CONTEXT_I_OPERATION_CONTEXT_WRITER_HPP_
