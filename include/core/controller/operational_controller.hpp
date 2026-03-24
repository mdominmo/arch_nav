#ifndef ARCH_NAV__CORE__CONTROLLER__OPERATIONAL_CONTROLLER_HPP_
#define ARCH_NAV__CORE__CONTROLLER__OPERATIONAL_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <vector>

#include "core/constants/command_response.hpp"
#include "core/constants/operation_status.hpp"
#include "core/constants/reference_frame.hpp"
#include "core/controller/navigation_task.hpp"
#include "core/model/report/operation_report.hpp"
#include "core/controller/vehicle_command.hpp"
#include "core/context/vehicle_context.hpp"
#include "core/model/vehicle/waypoint.hpp"
#include "core/model/vehicle/trajectory_point.hpp"
#include "core/model/vehicle/vehicle_status.hpp"
#include "dispatchers/i_command_dispatcher.hpp"

namespace arch_nav::controller {

class OperationalController {
 public:
  explicit OperationalController(
      context::VehicleContext& vehicle_context,
      dispatchers::ICommandDispatcher& dispatcher);

  ~OperationalController();

  constants::CommandResponse waypoint_following(
      std::vector<vehicle::Waypoint> waypoints,
      constants::ReferenceFrame frame);
  constants::CommandResponse trajectory_execution(
      std::vector<vehicle::TrajectoryPoint> trajectory,
      constants::ReferenceFrame frame);
  constants::CommandResponse takeoff(double height, constants::ReferenceFrame frame);
  constants::CommandResponse land();
  void stop();
  constants::CommandResponse arm();
  constants::CommandResponse disarm();

  constants::OperationStatus       operation_status() const;
  const report::OperationReport*   last_operation_report() const;

 private:
  struct State {
    virtual void on_enter(OperationalController&) {}
    virtual void on_vehicle_status_update(
        OperationalController&, const vehicle::VehicleStatus&) {}
    virtual constants::CommandResponse try_execute(
        OperationalController&, std::unique_ptr<NavigationTask>) {
      return constants::CommandResponse::DENIED;
    }
    virtual void try_command(
        OperationalController&, std::unique_ptr<VehicleCommand>) {}
    virtual void try_stop(OperationalController&) {}
    virtual ~State() = default;
  };

  struct HandoverState;
  struct DisarmedState;
  struct IdleState;
  struct RunningState;

  void on_vehicle_status_update(const vehicle::VehicleStatus& status);
  void on_operation_complete();
  void change_state(std::unique_ptr<State> new_state, constants::OperationStatus status);

  mutable std::mutex                         mutex_;
  context::VehicleContext&                   vehicle_context_;
  dispatchers::ICommandDispatcher&           dispatcher_;
  std::unique_ptr<State>                     current_state_;
  constants::OperationStatus                 current_status_;
  std::shared_ptr<report::OperationReport>   last_report_;
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV__CORE__CONTROLLER__OPERATIONAL_CONTROLLER_HPP_
