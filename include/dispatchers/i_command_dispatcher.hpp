#ifndef NAVIGATION__DISPATCHERS__I_COMMAND_DISPATCHER_HPP_
#define NAVIGATION__DISPATCHERS__I_COMMAND_DISPATCHER_HPP_

#include <functional>
#include <vector>

#include "core/model/vehicle/trajectory_point.hpp"
#include "geographic_msgs/msg/geo_pose.hpp"

namespace arch_nav::dispatchers {

class ICommandDispatcher {
 public:
  virtual void execute_takeoff(double height, std::function<void()> on_complete) = 0;
  virtual void execute_land(std::function<void()> on_complete) = 0;
  virtual void execute_waypoint_following(
      std::vector<geographic_msgs::msg::GeoPose> waypoints,
      std::function<void()> on_complete) = 0;

  virtual void execute_trajectory(
      std::vector<vehicle::TrajectoryPoint> trajectory,
      std::function<void()> on_complete) = 0;

  virtual void execute_arm() = 0;
  virtual void execute_disarm() = 0;

  virtual void stop() = 0;

  virtual ~ICommandDispatcher() = default;
};

}  // namespace arch_nav::dispatchers

#endif  // NAVIGATION__DISPATCHERS__I_COMMAND_DISPATCHER_HPP_
