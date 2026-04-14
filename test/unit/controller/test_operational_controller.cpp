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
#include "arch_nav/driver/i_command_dispatcher.hpp"
#include "arch_nav/model/report/operation_report.hpp"
#include "arch_nav/model/report/takeoff_driver_operation_data.hpp"
#include "arch_nav/model/report/waypoint_driver_operation_data.hpp"
#include "arch_nav/model/vehicle/vehicle_status.hpp"
#include "arch_nav/model/vehicle/waypoint.hpp"
#include "controller/operational_controller.hpp"

using namespace arch_nav::constants;
using namespace arch_nav::context;
using namespace arch_nav::controller;
using namespace arch_nav::platform;
using namespace arch_nav::report;
using arch_nav::vehicle::VehicleStatus;
using arch_nav::vehicle::Waypoint;

// ─────────────────────────────────────────────────────────────────────────────
// Mock dispatcher
// ─────────────────────────────────────────────────────────────────────────────

struct MockDispatcher : public ICommandDispatcher {
  bool accept_takeoff  = false;
  bool accept_land     = false;
  std::function<void()> stored_complete;

  CommandResponse execute_arm()    override { return CommandResponse::ACCEPTED; }
  CommandResponse execute_disarm() override { return CommandResponse::ACCEPTED; }

  CommandResponse execute_takeoff(
      double, ReferenceFrame,
      std::function<void()> on_complete,
      TakeoffDriverOperationData&) override {
    if (!accept_takeoff) return CommandResponse::NOT_SUPPORTED;
    stored_complete = std::move(on_complete);
    return CommandResponse::ACCEPTED;
  }

  CommandResponse execute_land(std::function<void()> on_complete) override {
    if (!accept_land) return CommandResponse::NOT_SUPPORTED;
    stored_complete = std::move(on_complete);
    return CommandResponse::ACCEPTED;
  }

  // Simulates the driver signalling operation completion
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

// ─────────────────────────────────────────────────────────────────────────────
// Fixture
// ─────────────────────────────────────────────────────────────────────────────

class OperationalControllerTest : public ::testing::Test {
 protected:
  VehicleContext        context_;
  MockDispatcher        dispatcher_;
  OperationalController ctrl_{context_, dispatcher_};
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
