#ifndef ARCH_NAV_CONTEXT_I_VEHICLE_CONTEXT_READER_HPP_
#define ARCH_NAV_CONTEXT_I_VEHICLE_CONTEXT_READER_HPP_

#include <functional>

#include "arch_nav/context/i_context_reader.hpp"
#include "arch_nav/model/vehicle/global_position.hpp"
#include "arch_nav/model/vehicle/kinematics.hpp"
#include "arch_nav/model/vehicle/vehicle_status.hpp"

namespace arch_nav::context {

class IVehicleContextReader : public IContextReader {
 public:
  virtual vehicle::GlobalPosition get_global_position() const = 0;
  virtual vehicle::Kinematics get_kinematic() const = 0;
  virtual vehicle::VehicleStatus get_vehicle_status() const = 0;
  virtual void subscribe_vehicle_status(
      std::function<void(const vehicle::VehicleStatus&)> callback) = 0;
};

}  // namespace arch_nav::context

#endif  // ARCH_NAV_CONTEXT_I_VEHICLE_CONTEXT_READER_HPP_
