#ifndef ARCH_NAV_CONTEXT_OPERATION_CONTEXT_HPP_
#define ARCH_NAV_CONTEXT_OPERATION_CONTEXT_HPP_

#include <functional>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "arch_nav/model/vehicle/global_position.hpp"
#include "arch_nav/model/vehicle/behavior_subject.hpp"
#include "arch_nav/model/operation/obstacle.hpp"

namespace arch_nav::context {

class OperationContext {
 public:
  OperationContext();

  std::optional<vehicle::GlobalPosition> get_roi() const;
  void update_roi(const vehicle::GlobalPosition& roi);
  void clear_roi();

  std::vector<operation::Obstacle> get_obstacles() const;
  void set_obstacle(const operation::Obstacle& obstacle);
  void remove_obstacle(const std::string& id);
  void clear_obstacles();

  void subscribe_obstacles(
      std::function<void(const std::vector<operation::Obstacle>&)> callback);

 private:
  void publish_obstacles();

  mutable std::shared_mutex roi_mutex_;
  std::optional<vehicle::GlobalPosition> roi_;

  mutable std::mutex obstacles_mutex_;
  std::unordered_map<std::string, operation::Obstacle> obstacles_;
  std::vector<std::function<void(const std::vector<operation::Obstacle>&)>>
      obstacles_subscribers_;
};

}  // namespace arch_nav::context

#endif  // ARCH_NAV_CONTEXT_OPERATION_CONTEXT_HPP_
