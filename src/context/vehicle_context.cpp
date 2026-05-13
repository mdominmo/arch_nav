#include "arch_nav/context/vehicle_context.hpp"

#include <shared_mutex>
#include <mutex>

namespace arch_nav::context {

using std::shared_lock;
using std::shared_mutex;
using std::unique_lock;

VehicleContext::VehicleContext() = default;

GlobalPosition VehicleContext::get_global_position() const {
  shared_lock<shared_mutex> lock(global_position_mutex_);
  return global_position_;
}

Kinematics VehicleContext::get_kinematic() const {
  shared_lock<shared_mutex> lock(kinematic_mutex_);
  return kinematic_;
}

VehicleStatus VehicleContext::get_vehicle_status() const {
  return vehicle_status_.get();
}

void VehicleContext::update(
    const GlobalPosition& state) {
  unique_lock<shared_mutex> lock(global_position_mutex_);
  global_position_ = state;
}

void VehicleContext::update(const Kinematics& state) {
  unique_lock<shared_mutex> lock(kinematic_mutex_);
  kinematic_ = state;
}

void VehicleContext::update(const VehicleStatus& state) {
  vehicle_status_.set(state);
}

std::optional<GlobalPosition> VehicleContext::get_roi() const {
  shared_lock<shared_mutex> lock(roi_mutex_);
  return roi_;
}

void VehicleContext::update_roi(const GlobalPosition& roi) {
  unique_lock<shared_mutex> lock(roi_mutex_);
  roi_ = roi;
}

void VehicleContext::clear_roi() {
  unique_lock<shared_mutex> lock(roi_mutex_);
  roi_.reset();
}

void VehicleContext::subscribe_vehicle_status(
    std::function<void(const VehicleStatus&)> callback) {
  vehicle_status_.subscribe(std::move(callback));
}

}  // namespace arch_nav::context
