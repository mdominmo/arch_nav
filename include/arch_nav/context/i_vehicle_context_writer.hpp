#ifndef ARCH_NAV_CONTEXT_I_VEHICLE_CONTEXT_WRITER_HPP_
#define ARCH_NAV_CONTEXT_I_VEHICLE_CONTEXT_WRITER_HPP_

#include "arch_nav/context/i_context_writer.hpp"
#include "arch_nav/model/vehicle/global_position.hpp"
#include "arch_nav/model/vehicle/kinematics.hpp"
#include "arch_nav/model/vehicle/vehicle_status.hpp"

namespace arch_nav::context {

class IVehicleContextWriter : public IContextWriter {
 public:
  virtual void update(const vehicle::GlobalPosition& state) = 0;
  virtual void update(const vehicle::Kinematics& state) = 0;
  virtual void update(const vehicle::VehicleStatus& state) = 0;
};

}  // namespace arch_nav::context

#endif  // ARCH_NAV_CONTEXT_I_VEHICLE_CONTEXT_WRITER_HPP_
