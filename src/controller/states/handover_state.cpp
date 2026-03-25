#include "handover_state.hpp"

#include <memory>

#include "disarmed_state.hpp"
#include "idle_state.hpp"

#include "arch_nav/constants/operation_status.hpp"
#include "arch_nav/constants/vehicle_status_states.hpp"

namespace arch_nav::controller {

void OperationalController::HandoverState::on_vehicle_status_update(
    OperationalController& ctx, const vehicle::VehicleStatus& status) {
  if (!status.is_valid() ||
      status.control_state != constants::ControlState::KERNEL_CONTROLLED) {
    return;
  }
  if (status.arm_state == constants::ArmState::ARMED) {
    ctx.change_state(
        std::make_unique<IdleState>(),
        constants::OperationStatus::IDLE);
  } else {
    ctx.change_state(
        std::make_unique<DisarmedState>(),
        constants::OperationStatus::DISARMED);
  }
}

}  // namespace arch_nav::controller
