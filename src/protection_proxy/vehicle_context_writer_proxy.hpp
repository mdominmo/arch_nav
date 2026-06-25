#ifndef ARCH_NAV_PROTECTION_PROXY_VEHICLE_CONTEXT_WRITER_PROXY_HPP_
#define ARCH_NAV_PROTECTION_PROXY_VEHICLE_CONTEXT_WRITER_PROXY_HPP_

#include "arch_nav/context/i_vehicle_context_writer.hpp"
#include "arch_nav/context/vehicle_context.hpp"

namespace arch_nav::context {

class VehicleContextWriterProxy final : public IVehicleContextWriter {
 public:
  explicit VehicleContextWriterProxy(VehicleContext& context)
      : context_(context) {}

  void update(const vehicle::GlobalPosition& state) override {
    context_.update(state);
  }

  void update(const vehicle::Kinematics& state) override {
    context_.update(state);
  }

  void update(const vehicle::VehicleStatus& state) override {
    context_.update(state);
  }

 private:
  VehicleContext& context_;
};

}  // namespace arch_nav::context

#endif  // ARCH_NAV_PROTECTION_PROXY_VEHICLE_CONTEXT_WRITER_PROXY_HPP_
