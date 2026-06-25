#ifndef ARCH_NAV_PROTECTION_PROXY_VEHICLE_CONTEXT_READER_PROXY_HPP_
#define ARCH_NAV_PROTECTION_PROXY_VEHICLE_CONTEXT_READER_PROXY_HPP_

#include "arch_nav/context/i_vehicle_context_reader.hpp"
#include "arch_nav/context/vehicle_context.hpp"

namespace arch_nav::context {

class VehicleContextReaderProxy final : public IVehicleContextReader {
 public:
  explicit VehicleContextReaderProxy(VehicleContext& context)
      : context_(context) {}

  vehicle::GlobalPosition get_global_position() const override {
    return context_.get_global_position();
  }

  vehicle::Kinematics get_kinematic() const override {
    return context_.get_kinematic();
  }

  vehicle::VehicleStatus get_vehicle_status() const override {
    return context_.get_vehicle_status();
  }

  void subscribe_vehicle_status(
      std::function<void(const vehicle::VehicleStatus&)> callback) override {
    context_.subscribe_vehicle_status(std::move(callback));
  }

 private:
  VehicleContext& context_;
};

}  // namespace arch_nav::context

#endif  // ARCH_NAV_PROTECTION_PROXY_VEHICLE_CONTEXT_READER_PROXY_HPP_
