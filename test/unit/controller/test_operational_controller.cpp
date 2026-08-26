#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "arch_nav/constants/command_response.hpp"
#include "arch_nav/constants/operation_status.hpp"
#include "arch_nav/constants/reference_frame.hpp"
#include "arch_nav/constants/vehicle_status_states.hpp"
#include "arch_nav/context/vehicle_context.hpp"
#include "arch_nav/context/operation_context.hpp"
#include "arch_nav/driver/i_command_dispatcher.hpp"
#include "arch_nav/execution/takeoff_execution_state.hpp"
#include "arch_nav/execution/waypoint_execution_state.hpp"
#include "arch_nav/execution/trajectory_execution_state.hpp"
#include "arch_nav/model/vehicle/global_position.hpp"
#include "arch_nav/model/report/operation_report.hpp"
#include "arch_nav/model/vehicle/trajectory_point.hpp"
#include "arch_nav/model/vehicle/vehicle_status.hpp"
#include "arch_nav/model/vehicle/waypoint.hpp"
#include "arch_nav/controller/preemption_event.hpp"
#include "arch_nav/controller/preemption_info.hpp"
#include "arch_nav/controller/preemption_type.hpp"
#include "controller/operational_controller.hpp"

using namespace arch_nav::constants;
using namespace arch_nav::context;
using namespace arch_nav::controller;
using namespace arch_nav::platform;
using namespace arch_nav::report;
using arch_nav::vehicle::GlobalPosition;
using arch_nav::vehicle::TrajectoryPoint;
using arch_nav::vehicle::VehicleStatus;
using arch_nav::vehicle::Waypoint;

// ─────────────────────────────────────────────────────────────────────────────
// Mock dispatcher
// ─────────────────────────────────────────────────────────────────────────────

struct MockDispatcher : public ICommandDispatcher {
  bool accept_takeoff     = false;
  bool accept_land        = false;
  bool accept_change_yaw  = false;
  bool accept_waypoints   = false;
  bool accept_trajectory  = false;
  bool accept_set_roi     = false;
  std::function<void()> stored_complete;

  arch_nav::execution::WaypointExecutionState*   last_waypoint_state{nullptr};
  arch_nav::execution::TrajectoryExecutionState* last_trajectory_state{nullptr};

  CommandResponse execute_arm()    override { return CommandResponse::ACCEPTED; }
  CommandResponse execute_disarm() override { return CommandResponse::ACCEPTED; }

  CommandResponse execute_set_roi(
      arch_nav::vehicle::GlobalPosition, ReferenceFrame) override {
    return accept_set_roi ? CommandResponse::ACCEPTED : CommandResponse::NOT_SUPPORTED;
  }

  CommandResponse execute_clear_roi() override { return CommandResponse::ACCEPTED; }

  CommandResponse execute_takeoff(
      double, ReferenceFrame,
      std::function<void()> on_complete,
      arch_nav::execution::TakeoffExecutionState&) override {
    if (!accept_takeoff) return CommandResponse::NOT_SUPPORTED;
    stored_complete = std::move(on_complete);
    return CommandResponse::ACCEPTED;
  }

  CommandResponse execute_land(std::function<void()> on_complete) override {
    if (!accept_land) return CommandResponse::NOT_SUPPORTED;
    stored_complete = std::move(on_complete);
    return CommandResponse::ACCEPTED;
  }

  CommandResponse execute_change_yaw(
      double, ReferenceFrame,
      std::function<void()> on_complete) override {
    if (!accept_change_yaw) return CommandResponse::NOT_SUPPORTED;
    stored_complete = std::move(on_complete);
    return CommandResponse::ACCEPTED;
  }

  CommandResponse execute_waypoint_following(
      std::vector<Waypoint>, ReferenceFrame,
      std::function<void()> on_complete,
      arch_nav::execution::WaypointExecutionState& state) override {
    if (!accept_waypoints) return CommandResponse::NOT_SUPPORTED;
    last_waypoint_state = &state;
    stored_complete = std::move(on_complete);
    return CommandResponse::ACCEPTED;
  }

  CommandResponse execute_trajectory(
      std::vector<TrajectoryPoint>, ReferenceFrame,
      std::function<void()> on_complete,
      arch_nav::execution::TrajectoryExecutionState& state) override {
    if (!accept_trajectory) return CommandResponse::NOT_SUPPORTED;
    last_trajectory_state = &state;
    stored_complete = std::move(on_complete);
    return CommandResponse::ACCEPTED;
  }

  void complete() { if (stored_complete) stored_complete(); }
};

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

static VehicleStatus kernel_armed() {
  return VehicleStatus{ControlState::KERNEL_CONTROLLED, ArmState::ARMED};
}
static VehicleStatus kernel_disarmed() {
  return VehicleStatus{ControlState::KERNEL_CONTROLLED, ArmState::DISARMED};
}
static VehicleStatus external_control() {
  return VehicleStatus{ControlState::EXTERNAL, ArmState::ARMED};
}
static VehicleStatus unknown_status() {
  return VehicleStatus{ControlState::UNKNOWN, ArmState::UNKNOWN};
}

static std::vector<Waypoint> sample_waypoints() {
  return {{40.0, -3.0, 10.0}, {40.1, -3.1, 15.0}, {40.2, -3.2, 20.0}};
}

static std::vector<TrajectoryPoint> sample_trajectory() {
  return {
    {0.0, 0,0,0, 0,0,0, 0,0,0, 0.0, 0.0},
    {1.0, 1,0,0, 1,0,0, 0,0,0, 0.0, 0.0},
    {2.0, 2,0,0, 0,0,0, 0,0,0, 0.0, 0.0},
  };
}

// ─────────────────────────────────────────────────────────────────────────────
// Fixture
// ─────────────────────────────────────────────────────────────────────────────

class OperationalControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    context_.subscribe_vehicle_status(
        [this](const VehicleStatus& s) { ctrl_.on_vehicle_status_update(s); });
  }

  VehicleContext        context_;
  OperationContext      operation_context_;
  MockDispatcher        dispatcher_;
  OperationalController ctrl_{operation_context_, dispatcher_};
};

// ─────────────────────────────────────────────────────────────────────────────
// HANDOVER state
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(OperationalControllerTest, Handover_InitialState) {
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::HANDOVER);
}

TEST_F(OperationalControllerTest, Handover_TasksDenied) {
  EXPECT_EQ(ctrl_.takeoff(10.0, ReferenceFrame::GLOBAL_WGS84), CommandResponse::DENIED);
  EXPECT_EQ(ctrl_.land(),                                        CommandResponse::DENIED);
  EXPECT_EQ(ctrl_.change_yaw(1.2, ReferenceFrame::LOCAL_NED),    CommandResponse::DENIED);
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::HANDOVER);
}

TEST_F(OperationalControllerTest, Handover_StaysOnExternalControl) {
  context_.update(external_control());
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::HANDOVER);
}

TEST_F(OperationalControllerTest, Handover_StaysOnUnknownStatus) {
  context_.update(unknown_status());
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::HANDOVER);
}

TEST_F(OperationalControllerTest, Handover_ToDisarmedOnKernelDisarmed) {
  context_.update(kernel_disarmed());
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::DISARMED);
}

TEST_F(OperationalControllerTest, Handover_ToIdleOnKernelArmed) {
  context_.update(kernel_armed());
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::IDLE);
}

// ─────────────────────────────────────────────────────────────────────────────
// DISARMED state
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(OperationalControllerTest, Disarmed_TasksDenied) {
  context_.update(kernel_disarmed());
  EXPECT_EQ(ctrl_.takeoff(10.0, ReferenceFrame::GLOBAL_WGS84), CommandResponse::DENIED);
  EXPECT_EQ(ctrl_.land(),                                        CommandResponse::DENIED);
  EXPECT_EQ(ctrl_.change_yaw(1.2, ReferenceFrame::LOCAL_NED),    CommandResponse::DENIED);
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::DISARMED);
}

TEST_F(OperationalControllerTest, Disarmed_ToIdleOnArmed) {
  context_.update(kernel_disarmed());
  context_.update(kernel_armed());
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::IDLE);
}

TEST_F(OperationalControllerTest, Disarmed_ToHandoverOnControlLost) {
  context_.update(kernel_disarmed());
  context_.update(external_control());
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::HANDOVER);
}

TEST_F(OperationalControllerTest, Disarmed_ToHandoverOnInvalidStatus) {
  context_.update(kernel_disarmed());
  context_.update(unknown_status());
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::HANDOVER);
}

// ─────────────────────────────────────────────────────────────────────────────
// IDLE state
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(OperationalControllerTest, Idle_ToHandoverOnControlLost) {
  context_.update(kernel_armed());
  context_.update(external_control());
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::HANDOVER);
}

TEST_F(OperationalControllerTest, Idle_ToDisarmedOnDisarm) {
  context_.update(kernel_armed());
  context_.update(kernel_disarmed());
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::DISARMED);
}

TEST_F(OperationalControllerTest, Idle_TaskNotSupportedStaysIdle) {
  context_.update(kernel_armed());
  dispatcher_.accept_takeoff = false;

  auto resp = ctrl_.takeoff(10.0, ReferenceFrame::GLOBAL_WGS84);

  EXPECT_EQ(resp, CommandResponse::NOT_SUPPORTED);
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::IDLE);
  ASSERT_NE(ctrl_.last_operation_report(), nullptr);
  EXPECT_EQ(ctrl_.last_operation_report()->status(), ReportStatus::FAILED);
}

TEST_F(OperationalControllerTest, Idle_ChangeYawNotSupportedStaysIdle) {
  context_.update(kernel_armed());
  dispatcher_.accept_change_yaw = false;

  auto resp = ctrl_.change_yaw(0.5, ReferenceFrame::LOCAL_NED);

  EXPECT_EQ(resp, CommandResponse::NOT_SUPPORTED);
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::IDLE);
  ASSERT_NE(ctrl_.last_operation_report(), nullptr);
  EXPECT_EQ(ctrl_.last_operation_report()->status(), ReportStatus::FAILED);
}

TEST_F(OperationalControllerTest, Idle_ToRunningOnAcceptedTakeoff) {
  context_.update(kernel_armed());
  dispatcher_.accept_takeoff = true;

  auto resp = ctrl_.takeoff(10.0, ReferenceFrame::GLOBAL_WGS84);

  EXPECT_EQ(resp, CommandResponse::ACCEPTED);
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::RUNNING);
  ASSERT_NE(ctrl_.last_operation_report(), nullptr);
  EXPECT_EQ(ctrl_.last_operation_report()->status(), ReportStatus::IN_PROGRESS);
}

TEST_F(OperationalControllerTest, Idle_ToRunningOnAcceptedLand) {
  context_.update(kernel_armed());
  dispatcher_.accept_land = true;

  EXPECT_EQ(ctrl_.land(), CommandResponse::ACCEPTED);
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::RUNNING);
}

TEST_F(OperationalControllerTest, Idle_ToRunningOnAcceptedChangeYaw) {
  context_.update(kernel_armed());
  dispatcher_.accept_change_yaw = true;

  EXPECT_EQ(ctrl_.change_yaw(0.7, ReferenceFrame::LOCAL_NED), CommandResponse::ACCEPTED);
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::RUNNING);
}

// ─────────────────────────────────────────────────────────────────────────────
// RUNNING state
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(OperationalControllerTest, Running_NewTaskDenied) {
  context_.update(kernel_armed());
  dispatcher_.accept_takeoff = true;
  ctrl_.takeoff(10.0, ReferenceFrame::GLOBAL_WGS84);

  EXPECT_EQ(ctrl_.land(), CommandResponse::DENIED);
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::RUNNING);
}

TEST_F(OperationalControllerTest, Running_ToIdleOnOperationComplete) {
  context_.update(kernel_armed());
  dispatcher_.accept_takeoff = true;
  ctrl_.takeoff(10.0, ReferenceFrame::GLOBAL_WGS84);

  dispatcher_.complete();

  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::IDLE);
  ASSERT_NE(ctrl_.last_operation_report(), nullptr);
  EXPECT_EQ(ctrl_.last_operation_report()->status(), ReportStatus::COMPLETED);
}

TEST_F(OperationalControllerTest, Running_ChangeYawToIdleOnOperationComplete) {
  context_.update(kernel_armed());
  dispatcher_.accept_change_yaw = true;
  ctrl_.change_yaw(1.0, ReferenceFrame::LOCAL_NED);

  dispatcher_.complete();

  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::IDLE);
  ASSERT_NE(ctrl_.last_operation_report(), nullptr);
  EXPECT_EQ(ctrl_.last_operation_report()->status(), ReportStatus::COMPLETED);
}

TEST_F(OperationalControllerTest, Running_StopAbortsAndGoesToIdle) {
  context_.update(kernel_armed());
  dispatcher_.accept_takeoff = true;
  ctrl_.takeoff(10.0, ReferenceFrame::GLOBAL_WGS84);

  ctrl_.stop();

  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::IDLE);
  ASSERT_NE(ctrl_.last_operation_report(), nullptr);
  EXPECT_EQ(ctrl_.last_operation_report()->status(), ReportStatus::ABORTED);
}

TEST_F(OperationalControllerTest, Running_ToHandoverOnControlLost) {
  context_.update(kernel_armed());
  dispatcher_.accept_takeoff = true;
  ctrl_.takeoff(10.0, ReferenceFrame::GLOBAL_WGS84);

  context_.update(external_control());

  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::HANDOVER);
  EXPECT_EQ(ctrl_.last_operation_report()->status(), ReportStatus::ABORTED);
}

TEST_F(OperationalControllerTest, Running_ToDisarmedOnDisarm) {
  context_.update(kernel_armed());
  dispatcher_.accept_takeoff = true;
  ctrl_.takeoff(10.0, ReferenceFrame::GLOBAL_WGS84);

  context_.update(kernel_disarmed());

  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::DISARMED);
  EXPECT_EQ(ctrl_.last_operation_report()->status(), ReportStatus::ABORTED);
}

// ─────────────────────────────────────────────────────────────────────────────
// Callbacks
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(OperationalControllerTest, Callback_OnCompleteCalledWithCompletedOnNormalFinish) {
  context_.update(kernel_armed());
  dispatcher_.accept_takeoff = true;

  bool called = false;
  ReportStatus received = ReportStatus::IN_PROGRESS;
  ctrl_.set_on_complete_listener([&](const OperationReport& r) {
    called = true;
    received = r.status();
  });

  ctrl_.takeoff(10.0, ReferenceFrame::GLOBAL_WGS84);
  dispatcher_.complete();

  EXPECT_TRUE(called);
  EXPECT_EQ(received, ReportStatus::COMPLETED);
}

TEST_F(OperationalControllerTest, Callback_OnCompleteCalledWithAbortedOnControlLost) {
  context_.update(kernel_armed());
  dispatcher_.accept_takeoff = true;

  bool called = false;
  ReportStatus received = ReportStatus::IN_PROGRESS;
  ctrl_.set_on_complete_listener([&](const OperationReport& r) {
    called = true;
    received = r.status();
  });

  ctrl_.takeoff(10.0, ReferenceFrame::GLOBAL_WGS84);
  context_.update(external_control());

  EXPECT_TRUE(called);
  EXPECT_EQ(received, ReportStatus::ABORTED);
}

TEST_F(OperationalControllerTest, Callback_OnCompleteCalledWithAbortedOnDisarm) {
  context_.update(kernel_armed());
  dispatcher_.accept_takeoff = true;

  bool called = false;
  ReportStatus received = ReportStatus::IN_PROGRESS;
  ctrl_.set_on_complete_listener([&](const OperationReport& r) {
    called = true;
    received = r.status();
  });

  ctrl_.takeoff(10.0, ReferenceFrame::GLOBAL_WGS84);
  context_.update(kernel_disarmed());

  EXPECT_TRUE(called);
  EXPECT_EQ(received, ReportStatus::ABORTED);
}

TEST_F(OperationalControllerTest, Callback_OnCompleteCalledWithAbortedOnStop) {
  context_.update(kernel_armed());
  dispatcher_.accept_takeoff = true;

  bool called = false;
  ReportStatus received = ReportStatus::IN_PROGRESS;
  ctrl_.set_on_complete_listener([&](const OperationReport& r) {
    called = true;
    received = r.status();
  });

  ctrl_.takeoff(10.0, ReferenceFrame::GLOBAL_WGS84);
  ctrl_.stop();

  EXPECT_TRUE(called);
  EXPECT_EQ(received, ReportStatus::ABORTED);
}

TEST_F(OperationalControllerTest, Callback_OnProgressCalledDuringOperation) {
  context_.update(kernel_armed());
  dispatcher_.accept_takeoff = true;

  std::atomic<int> call_count{0};
  ctrl_.set_on_progress_listener([&](const OperationReport&) {
    call_count++;
  });

  ctrl_.takeoff(10.0, ReferenceFrame::GLOBAL_WGS84);
  std::this_thread::sleep_for(std::chrono::milliseconds(350));
  ctrl_.stop();

  EXPECT_GE(call_count.load(), 2);
}

TEST_F(OperationalControllerTest, Callback_OnProgressStopsAfterOperationComplete) {
  context_.update(kernel_armed());
  dispatcher_.accept_takeoff = true;

  std::atomic<int> call_count{0};
  ctrl_.set_on_progress_listener([&](const OperationReport&) {
    call_count++;
  });

  ctrl_.takeoff(10.0, ReferenceFrame::GLOBAL_WGS84);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  dispatcher_.complete();

  const int count_at_complete = call_count.load();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  EXPECT_EQ(call_count.load(), count_at_complete);
}

// ─────────────────────────────────────────────────────────────────────────────
// ROI command
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(OperationalControllerTest, SetRoi_AcceptedInIdle) {
  context_.update(kernel_armed());
  dispatcher_.accept_set_roi = true;

  auto resp = ctrl_.set_roi(
      GlobalPosition{40.0, -3.0, 100.0}, ReferenceFrame::GLOBAL_WGS84);

  EXPECT_EQ(resp, CommandResponse::ACCEPTED);
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::IDLE);
}

TEST_F(OperationalControllerTest, SetRoi_NotSupportedPropagatedFromDriver) {
  context_.update(kernel_armed());
  dispatcher_.accept_set_roi = false;

  auto resp = ctrl_.set_roi(
      GlobalPosition{40.0, -3.0, 100.0}, ReferenceFrame::GLOBAL_WGS84);

  EXPECT_EQ(resp, CommandResponse::NOT_SUPPORTED);
}

TEST_F(OperationalControllerTest, SetRoi_AcceptedInDisarmed) {
  context_.update(kernel_disarmed());
  dispatcher_.accept_set_roi = true;

  auto resp = ctrl_.set_roi(
      GlobalPosition{40.0, -3.0, 100.0}, ReferenceFrame::GLOBAL_WGS84);

  EXPECT_EQ(resp, CommandResponse::ACCEPTED);
}

TEST_F(OperationalControllerTest, SetRoi_DeniedInRunning) {
  context_.update(kernel_armed());
  dispatcher_.accept_takeoff = true;
  ctrl_.takeoff(10.0, ReferenceFrame::GLOBAL_WGS84);
  dispatcher_.accept_set_roi = true;

  auto resp = ctrl_.set_roi(
      GlobalPosition{40.0, -3.0, 100.0}, ReferenceFrame::GLOBAL_WGS84);

  EXPECT_EQ(resp, CommandResponse::DENIED);
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::RUNNING);
}

TEST_F(OperationalControllerTest, SetRoi_DeniedInHandover) {
  dispatcher_.accept_set_roi = true;

  auto resp = ctrl_.set_roi(
      GlobalPosition{40.0, -3.0, 100.0}, ReferenceFrame::GLOBAL_WGS84);

  EXPECT_EQ(resp, CommandResponse::DENIED);
}

TEST_F(OperationalControllerTest, ClearRoi_AcceptedInIdle) {
  context_.update(kernel_armed());
  EXPECT_EQ(ctrl_.clear_roi(), CommandResponse::ACCEPTED);
}

TEST_F(OperationalControllerTest, ClearRoi_DeniedInRunning) {
  context_.update(kernel_armed());
  dispatcher_.accept_takeoff = true;
  ctrl_.takeoff(10.0, ReferenceFrame::GLOBAL_WGS84);

  EXPECT_EQ(ctrl_.clear_roi(), CommandResponse::DENIED);
}

// ─────────────────────────────────────────────────────────────────────────────
// Preemption / Memento — Waypoint
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(OperationalControllerTest, Preempt_WaypointTransientResumes) {
  context_.update(kernel_armed());
  dispatcher_.accept_waypoints = true;
  ctrl_.waypoint_following(sample_waypoints(), ReferenceFrame::GLOBAL_WGS84);
  ASSERT_EQ(ctrl_.operation_status(), OperationStatus::RUNNING);

  ctrl_.preempt(PreemptionType::TRANSIENT,
              {"test_supervisor", "test reason", ""});
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::SUPERVISED);

  dispatcher_.accept_land = true;
  ctrl_.land();

  dispatcher_.accept_waypoints = true;
  dispatcher_.complete();

  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::RUNNING);
}

TEST_F(OperationalControllerTest, Preempt_WaypointTerminalDoesNotResume) {
  context_.update(kernel_armed());
  dispatcher_.accept_waypoints = true;
  ctrl_.waypoint_following(sample_waypoints(), ReferenceFrame::GLOBAL_WGS84);

  ctrl_.preempt(PreemptionType::TERMINAL,
              {"test_supervisor", "test reason", ""});
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::SUPERVISED);

  dispatcher_.accept_land = true;
  ctrl_.land();
  dispatcher_.complete();

  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::IDLE);
}

// ─────────────────────────────────────────────────────────────────────────────
// Preemption / Memento — Trajectory
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(OperationalControllerTest, Preempt_TrajectoryTransientResumes) {
  context_.update(kernel_armed());
  dispatcher_.accept_trajectory = true;
  ctrl_.trajectory_execution(sample_trajectory(), ReferenceFrame::LOCAL_NED);
  ASSERT_EQ(ctrl_.operation_status(), OperationStatus::RUNNING);

  ctrl_.preempt(PreemptionType::TRANSIENT,
              {"test_supervisor", "test reason", ""});
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::SUPERVISED);

  dispatcher_.accept_land = true;
  ctrl_.land();

  dispatcher_.accept_trajectory = true;
  dispatcher_.complete();

  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::RUNNING);
}

TEST_F(OperationalControllerTest, Preempt_TrajectoryTerminalDoesNotResume) {
  context_.update(kernel_armed());
  dispatcher_.accept_trajectory = true;
  ctrl_.trajectory_execution(sample_trajectory(), ReferenceFrame::LOCAL_NED);

  ctrl_.preempt(PreemptionType::TERMINAL,
              {"test_supervisor", "test reason", ""});
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::SUPERVISED);

  dispatcher_.accept_land = true;
  ctrl_.land();
  dispatcher_.complete();

  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::IDLE);
}

// ─────────────────────────────────────────────────────────────────────────────
// Preemption / Memento — Takeoff
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(OperationalControllerTest, Preempt_TakeoffTransientResumes) {
  context_.update(kernel_armed());
  dispatcher_.accept_takeoff = true;
  ctrl_.takeoff(10.0, ReferenceFrame::LOCAL_NED);
  ASSERT_EQ(ctrl_.operation_status(), OperationStatus::RUNNING);

  ctrl_.preempt(PreemptionType::TRANSIENT,
              {"test_supervisor", "test reason", ""});
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::SUPERVISED);

  dispatcher_.accept_land = true;
  ctrl_.land();

  dispatcher_.accept_takeoff = true;
  dispatcher_.complete();

  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::RUNNING);
}

// ─────────────────────────────────────────────────────────────────────────────
// Preemption / Memento — Land
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(OperationalControllerTest, Preempt_LandTransientResumes) {
  context_.update(kernel_armed());
  dispatcher_.accept_land = true;
  ctrl_.land();
  ASSERT_EQ(ctrl_.operation_status(), OperationStatus::RUNNING);

  ctrl_.preempt(PreemptionType::TRANSIENT,
              {"test_supervisor", "test reason", ""});
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::SUPERVISED);

  dispatcher_.accept_takeoff = true;
  ctrl_.takeoff(5.0, ReferenceFrame::LOCAL_NED);

  dispatcher_.accept_land = true;
  dispatcher_.complete();

  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::RUNNING);
}

// ─────────────────────────────────────────────────────────────────────────────
// Preemption / Memento — ChangeYaw
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(OperationalControllerTest, Preempt_StopWithNoSupervisorTaskResumes) {
  context_.update(kernel_armed());
  dispatcher_.accept_waypoints = true;
  ctrl_.waypoint_following(sample_waypoints(), ReferenceFrame::GLOBAL_WGS84);
  ASSERT_EQ(ctrl_.operation_status(), OperationStatus::RUNNING);

  ctrl_.preempt(PreemptionType::TRANSIENT,
              {"test_supervisor", "test reason", ""});
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::SUPERVISED);

  // Supervisor calls stop() directly without ever dispatching a task of
  // its own (e.g. OAS's fallback_stop path) - must still resolve the
  // preemption instead of getting stuck in SUPERVISED forever. stop()
  // resumes the original waypoint_following synchronously, no further
  // dispatcher_.complete() needed.
  ctrl_.stop();

  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::RUNNING);
}

TEST_F(OperationalControllerTest, Preempt_StopWithNoSupervisorTaskTerminalGoesToIdle) {
  context_.update(kernel_armed());
  dispatcher_.accept_waypoints = true;
  ctrl_.waypoint_following(sample_waypoints(), ReferenceFrame::GLOBAL_WGS84);

  ctrl_.preempt(PreemptionType::TERMINAL,
              {"test_supervisor", "test reason", ""});
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::SUPERVISED);

  ctrl_.stop();

  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::IDLE);
  ASSERT_NE(ctrl_.last_operation_report(), nullptr);
  EXPECT_EQ(ctrl_.last_operation_report()->status(), ReportStatus::ABORTED);
}

TEST_F(OperationalControllerTest, Preempt_SupervisorTaskRejectedResumes) {
  context_.update(kernel_armed());
  dispatcher_.accept_waypoints = true;
  ctrl_.waypoint_following(sample_waypoints(), ReferenceFrame::GLOBAL_WGS84);
  ASSERT_EQ(ctrl_.operation_status(), OperationStatus::RUNNING);

  ctrl_.preempt(PreemptionType::TRANSIENT,
              {"test_supervisor", "test reason", ""});
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::SUPERVISED);

  // Supervisor's own dispatched task is rejected (e.g. OAS sending a
  // ReferenceFrame the driver doesn't support) - must still resolve the
  // preemption instead of getting stuck in SUPERVISED forever.
  dispatcher_.accept_land = false;
  auto response = ctrl_.land();
  EXPECT_EQ(response, CommandResponse::NOT_SUPPORTED);

  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::RUNNING);
}

TEST_F(OperationalControllerTest, Preempt_ChangeYawTransientResumes) {
  context_.update(kernel_armed());
  dispatcher_.accept_change_yaw = true;
  ctrl_.change_yaw(1.5, ReferenceFrame::LOCAL_NED);
  ASSERT_EQ(ctrl_.operation_status(), OperationStatus::RUNNING);

  ctrl_.preempt(PreemptionType::TRANSIENT,
              {"test_supervisor", "test reason", ""});
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::SUPERVISED);

  dispatcher_.accept_land = true;
  ctrl_.land();

  dispatcher_.accept_change_yaw = true;
  dispatcher_.complete();

  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::RUNNING);
}

// ─────────────────────────────────────────────────────────────────────────────
// Supervisor Events
// ─────────────────────────────────────────────────────────────────────────────

using arch_nav::controller::PreemptionEvent;
using arch_nav::controller::PreemptionEventType;

TEST_F(OperationalControllerTest, PreemptionEvent_ActivatedFiredOnPreempt) {
  context_.update(kernel_armed());
  dispatcher_.accept_takeoff = true;
  ctrl_.takeoff(10.0, ReferenceFrame::LOCAL_NED);

  std::vector<PreemptionEvent> events;
  ctrl_.set_on_preemption_event_listener([&](const PreemptionEvent& e) {
    events.push_back(e);
  });

  ctrl_.preempt(PreemptionType::TERMINAL,
                {"geofence", "boundary violation", "{\"zone\":1}"});

  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].event_type, PreemptionEventType::ACTIVATED);
  EXPECT_EQ(events[0].name, "geofence");
  EXPECT_EQ(events[0].reason, "boundary violation");
  EXPECT_EQ(events[0].preemption_type, PreemptionType::TERMINAL);
  EXPECT_EQ(events[0].details, "{\"zone\":1}");
}

TEST_F(OperationalControllerTest, PreemptionEvent_ResolvedFiredOnTransientComplete) {
  context_.update(kernel_armed());
  dispatcher_.accept_waypoints = true;
  ctrl_.waypoint_following(sample_waypoints(), ReferenceFrame::GLOBAL_WGS84);

  std::vector<PreemptionEvent> events;
  ctrl_.set_on_preemption_event_listener([&](const PreemptionEvent& e) {
    events.push_back(e);
  });

  ctrl_.preempt(PreemptionType::TRANSIENT,
                {"battery", "low battery", ""});

  dispatcher_.accept_land = true;
  ctrl_.land();

  dispatcher_.accept_waypoints = true;
  dispatcher_.complete();

  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].event_type, PreemptionEventType::ACTIVATED);
  EXPECT_EQ(events[1].event_type, PreemptionEventType::RESOLVED);
  EXPECT_EQ(events[1].name, "battery");
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::RUNNING);
}

TEST_F(OperationalControllerTest, PreemptionEvent_ResolvedFiredOnTerminalComplete) {
  context_.update(kernel_armed());
  dispatcher_.accept_takeoff = true;
  ctrl_.takeoff(10.0, ReferenceFrame::LOCAL_NED);

  std::vector<PreemptionEvent> events;
  ctrl_.set_on_preemption_event_listener([&](const PreemptionEvent& e) {
    events.push_back(e);
  });

  ctrl_.preempt(PreemptionType::TERMINAL,
                {"geofence", "boundary violation", ""});

  dispatcher_.accept_land = true;
  ctrl_.land();
  dispatcher_.complete();

  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].event_type, PreemptionEventType::ACTIVATED);
  EXPECT_EQ(events[1].event_type, PreemptionEventType::RESOLVED);
  EXPECT_EQ(events[1].name, "geofence");
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::IDLE);
}

TEST_F(OperationalControllerTest, PreemptionEvent_ResolvedOnVehicleStatusLoss) {
  context_.update(kernel_armed());
  dispatcher_.accept_takeoff = true;
  ctrl_.takeoff(10.0, ReferenceFrame::LOCAL_NED);

  std::vector<PreemptionEvent> events;
  ctrl_.set_on_preemption_event_listener([&](const PreemptionEvent& e) {
    events.push_back(e);
  });

  ctrl_.preempt(PreemptionType::TRANSIENT,
                {"battery", "low battery", ""});

  context_.update(external_control());

  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].event_type, PreemptionEventType::ACTIVATED);
  EXPECT_EQ(events[1].event_type, PreemptionEventType::RESOLVED);
  EXPECT_EQ(events[1].name, "battery");
  EXPECT_EQ(ctrl_.operation_status(), OperationStatus::HANDOVER);
}
