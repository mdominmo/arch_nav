#ifndef ARCH_NAV_UTILS_FRAME_CONVERTER_HPP_
#define ARCH_NAV_UTILS_FRAME_CONVERTER_HPP_

#include <vector>

#include "arch_nav/constants/reference_frame.hpp"
#include "arch_nav/model/vehicle/waypoint.hpp"
#include "arch_nav/model/vehicle/trajectory_point.hpp"
#include "arch_nav/model/vehicle/global_position.hpp"

namespace arch_nav::utils::frame_converter {

vehicle::Waypoint to_ned(
    const vehicle::Waypoint& wp,
    constants::ReferenceFrame frame,
    double heading = 0.0);

std::vector<vehicle::Waypoint> to_ned(
    const std::vector<vehicle::Waypoint>& waypoints,
    constants::ReferenceFrame frame,
    double heading = 0.0);

vehicle::TrajectoryPoint to_ned(
    const vehicle::TrajectoryPoint& pt,
    constants::ReferenceFrame frame,
    double heading = 0.0);

std::vector<vehicle::TrajectoryPoint> to_ned(
    const std::vector<vehicle::TrajectoryPoint>& trajectory,
    constants::ReferenceFrame frame,
    double heading = 0.0);

vehicle::Waypoint global_to_ned(
    const vehicle::Waypoint& wp,
    const vehicle::GlobalPosition& ref);

vehicle::Waypoint ned_to_global(
    const vehicle::Waypoint& wp,
    const vehicle::GlobalPosition& ref);

}  // namespace arch_nav::utils::frame_converter

#endif  // ARCH_NAV_UTILS_FRAME_CONVERTER_HPP_
