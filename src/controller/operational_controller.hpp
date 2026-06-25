#ifndef ARCH_NAV_CONTROLLER_OPERATIONAL_CONTROLLER_HPP_
#define ARCH_NAV_CONTROLLER_OPERATIONAL_CONTROLLER_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "arch_nav/constants/command_response.hpp"
#include "arch_nav/constants/operation_status.hpp"
#include "arch_nav/constants/reference_frame.hpp"
#include "controller/navigation_task.hpp"
#include "controller/navigation_task_memento.hpp"
#include "arch_nav/model/report/operation_report.hpp"
#include "controller/vehicle_command.hpp"
#include "arch_nav/model/vehicle/waypoint.hpp"
#include "arch_nav/model/vehicle/trajectory_point.hpp"
#include "arch_nav/model/vehicle/vehicle_status.hpp"
#include "arch_nav/driver/i_command_dispatcher.hpp"
#include "arch_nav/controller/i_operational_controller.hpp"

namespace arch_nav::controller {

class OperationalController : public IOperationalController {
 public:
  explicit OperationalController(
      platform::ICommandDispatcher& dispatcher);

  ~OperationalController();

  constants::CommandResponse waypoint_following(
      std::vector<vehicle::Waypoint> waypoints,
      constants::ReferenceFrame frame) override;
  constants::CommandResponse trajectory_execution(
      std::vector<vehicle::TrajectoryPoint> trajectory,
      constants::ReferenceFrame frame) override;
  constants::CommandResponse takeoff(double height, constants::ReferenceFrame frame) override;
  constants::CommandResponse land() override;
  constants::CommandResponse change_yaw(double new_yaw, constants::ReferenceFrame frame) override;
  void stop() override;
  constants::CommandResponse arm() override;
  constants::CommandResponse disarm() override;
  constants::CommandResponse set_roi(
      vehicle::GlobalPosition position,
      constants::ReferenceFrame frame) override;
  constants::CommandResponse clear_roi() override;

  constants::OperationStatus       operation_status() const override;
  const report::OperationReport*   last_operation_report() const override;

  void set_on_complete_listener(std::function<void(const report::OperationReport&)>) override;
  void set_on_progress_listener(std::function<void(const report::OperationReport&)>) override;

  void set_on_preemption_event_listener(
      std::function<void(const PreemptionEvent&)> callback) override;

  void preempt(PreemptionType type,
               const PreemptionInfo& info) override;

  void on_vehicle_status_update(const vehicle::VehicleStatus& status);

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

    struct PreemptionResult {
      std::unique_ptr<NavigationTaskMemento> memento;
      std::shared_ptr<report::OperationReport> user_report;
      bool accepted{false};
      std::optional<PreemptionEvent> displaced_preemption_event;
    };
    virtual PreemptionResult try_preempt(OperationalController&) {
      return {};
    }

    virtual ~State() = default;
  };

  struct HandoverState;
  struct DisarmedState;
  struct IdleState;
  struct RunningState;
  struct PreemptedState;

  void on_operation_complete();
  void on_supervisor_task_complete();
  void change_state(std::unique_ptr<State> new_state, constants::OperationStatus status);

  void start_progress_thread();
  void stop_progress_thread();

  mutable std::mutex                         mutex_;
  platform::ICommandDispatcher&              dispatcher_;
  std::unique_ptr<State>                     current_state_;
  constants::OperationStatus                 current_status_;
  std::shared_ptr<report::OperationReport>   last_report_;

  std::function<void(const report::OperationReport&)> on_complete_listener_;
  std::function<void(const report::OperationReport&)> on_progress_listener_;
  std::function<void(const PreemptionEvent&)> on_preemption_event_listener_;
  std::thread                                progress_thread_;
  std::atomic<bool>                          progress_stop_{false};
};

}  // namespace arch_nav::controller

#endif  // ARCH_NAV_CONTROLLER_OPERATIONAL_CONTROLLER_HPP_
