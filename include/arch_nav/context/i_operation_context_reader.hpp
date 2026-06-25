#ifndef ARCH_NAV_CONTEXT_I_OPERATION_CONTEXT_READER_HPP_
#define ARCH_NAV_CONTEXT_I_OPERATION_CONTEXT_READER_HPP_

#include <functional>
#include <vector>

#include "arch_nav/context/i_context_reader.hpp"
#include "arch_nav/model/operation/obstacle.hpp"

namespace arch_nav::context {

class IOperationContextReader : public IContextReader {
 public:
  virtual std::vector<operation::Obstacle> get_obstacles() const = 0;
  virtual void subscribe_obstacles(
      std::function<void(const std::vector<operation::Obstacle>&)> callback) = 0;
};

}  // namespace arch_nav::context

#endif  // ARCH_NAV_CONTEXT_I_OPERATION_CONTEXT_READER_HPP_
