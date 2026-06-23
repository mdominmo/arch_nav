#ifndef ARCH_NAV__CORE__CONTROLLER__CONTEXT_UPDATES__SET_OBSTACLE_INFO_HPP_
#define ARCH_NAV__CORE__CONTROLLER__CONTEXT_UPDATES__SET_OBSTACLE_INFO_HPP_

#include <vector>

#include "arch_nav/model/operation/obstacle.hpp"
#include "controller/context_update.hpp"

namespace arch_nav::controller {

class SetObstacleInfo : public ContextUpdate {
 public:
  explicit SetObstacleInfo(std::vector<operation::Obstacle> obstacles)
      : obstacles_(std::move(obstacles)) {}

  void apply(context::OperationContext& context) override {
    for (const auto& obstacle : obstacles_) {
      context.set_obstacle(obstacle);
    }
  }

 private:
  std::vector<operation::Obstacle> obstacles_;
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV__CORE__CONTROLLER__CONTEXT_UPDATES__SET_OBSTACLE_INFO_HPP_
