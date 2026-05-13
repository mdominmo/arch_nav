#ifndef ARCH_NAV_CONTROLLER_OPERATIONAL_CONTROLLER_HPP_
#define ARCH_NAV_CONTROLLER_OPERATIONAL_CONTROLLER_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "arch_nav/constants/command_response.hpp"
#include "arch_nav/constants/operation_status.hpp"
#include "arch_nav/constants/reference_frame.hpp"
#include "controller/navigation_task.hpp"
#include "arch_nav/model/report/operation_report.hpp"
#include "controller/vehicle_command.hpp"
#include "arch_nav/context/vehicle_context.hpp"
#include "arch_nav/model/vehicle/waypoint.hpp"
#include "arch_nav/model/vehicle/trajectory_point.hpp"
#include "arch_nav/model/vehicle/vehicle_status.hpp"
#include "arch_nav/driver/i_command_dispatcher.hpp"

namespace arch_nav::controller {

class OperationalController {
 public:
  explicit OperationalController(
      context::VehicleContext& vehicle_context,
      platform::ICommandDispatcher& dispatcher);

  ~OperationalController();

  constants::CommandResponse waypoint_following(
      std::vector<vehicle::Waypoint> waypoints,
      constants::ReferenceFrame frame);
  constants::CommandResponse trajectory_execution(
      std::vector<vehicle::TrajectoryPoint> trajectory,
      constants::ReferenceFrame frame);
  constants::CommandResponse takeoff(double height, constants::ReferenceFrame frame);
  constants::CommandResponse land();
  constants::CommandResponse change_yaw(double new_yaw, constants::ReferenceFrame frame);
  void stop();
  constants::CommandResponse arm();
  constants::CommandResponse disarm();
  constants::CommandResponse set_roi(
      vehicle::GlobalPosition position,
      constants::ReferenceFrame frame);
  constants::CommandResponse clear_roi();

  constants::OperationStatus       operation_status() const;
  const report::OperationReport*   last_operation_report() const;

  void set_on_complete_listener(std::function<void(const report::OperationReport&)>);
  void set_on_progress_listener(std::function<void(const report::OperationReport&)>);

 private:
  struct State {
    virtual void on_enter(OperationalController&) {}
    virtual void on_vehicle_status_update(
        OperationalController&, const vehicle::VehicleStatus&) {}
    virtual constants::CommandResponse try_execute(
        OperationalController&, std::unique_ptr<NavigationTask>) {
      return constants::CommandResponse::DENIED;
    }
    virtual constants::CommandResponse try_command(
        OperationalController&, std::unique_ptr<VehicleCommand>) {
      return constants::CommandResponse::DENIED;
    }
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

  void start_progress_thread();
  void stop_progress_thread();

  mutable std::mutex                         mutex_;
  context::VehicleContext&                   vehicle_context_;
  platform::ICommandDispatcher&              dispatcher_;
  std::unique_ptr<State>                     current_state_;
  constants::OperationStatus                 current_status_;
  std::shared_ptr<report::OperationReport>   last_report_;

  std::function<void(const report::OperationReport&)> on_complete_listener_;
  std::function<void(const report::OperationReport&)> on_progress_listener_;
  std::thread                                progress_thread_;
  std::atomic<bool>                          progress_stop_{false};
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV_CONTROLLER_OPERATIONAL_CONTROLLER_HPP_
