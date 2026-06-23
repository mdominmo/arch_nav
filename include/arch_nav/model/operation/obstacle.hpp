#ifndef ARCH_NAV_MODEL_OPERATION_OBSTACLE_HPP_
#define ARCH_NAV_MODEL_OPERATION_OBSTACLE_HPP_

#include <string>

#include "arch_nav/model/vehicle/global_position.hpp"

namespace arch_nav::operation {

struct Obstacle {
  std::string id;
  vehicle::GlobalPosition position;
  double radius;
  double height;

  Obstacle(std::string obstacle_id,
           vehicle::GlobalPosition center,
           double cylinder_radius,
           double cylinder_height)
      : id(std::move(obstacle_id)),
        position(std::move(center)),
        radius(cylinder_radius),
        height(cylinder_height) {}
};

}  // namespace arch_nav::operation

#endif  // ARCH_NAV_MODEL_OPERATION_OBSTACLE_HPP_
