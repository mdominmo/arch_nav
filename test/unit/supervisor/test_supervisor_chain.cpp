#include <gtest/gtest.h>

#include "arch_nav/context/operation_context.hpp"
#include "arch_nav/context/vehicle_context.hpp"
#include "arch_nav/controller/preemption_info.hpp"
#include "arch_nav/controller/preemption_type.hpp"
#include "arch_nav/driver/i_command_dispatcher.hpp"
#include "arch_nav/supervisor/supervisor_chain.hpp"
#include "controller/operational_controller.hpp"
#include "protection_proxy/operation_context_reader_proxy.hpp"
#include "protection_proxy/operation_context_writer_proxy.hpp"
#include "protection_proxy/vehicle_context_reader_proxy.hpp"

using namespace arch_nav::constants;
using namespace arch_nav::context;
using namespace arch_nav::controller;
using namespace arch_nav::platform;
using namespace arch_nav::supervisor;

namespace {

// Every command is unsupported - request_control()/execute() only need to
// complete without deadlocking here, not actually move a vehicle.
struct NoopDispatcher : public ICommandDispatcher {
  CommandResponse execute_arm() override { return CommandResponse::ACCEPTED; }
  CommandResponse execute_disarm() override { return CommandResponse::ACCEPTED; }
  CommandResponse execute_set_roi(arch_nav::vehicle::GlobalPosition, ReferenceFrame) override {
    return CommandResponse::NOT_SUPPORTED;
  }
  CommandResponse execute_clear_roi() override { return CommandResponse::ACCEPTED; }
  CommandResponse execute_takeoff(double, ReferenceFrame, std::function<void()>,
                                   arch_nav::execution::TakeoffExecutionState&) override {
    return CommandResponse::NOT_SUPPORTED;
  }
  CommandResponse execute_land(std::function<void()>) override { return CommandResponse::NOT_SUPPORTED; }
  CommandResponse execute_change_yaw(double, ReferenceFrame, std::function<void()>) override {
    return CommandResponse::NOT_SUPPORTED;
  }
  CommandResponse execute_waypoint_following(
      std::vector<arch_nav::vehicle::Waypoint>, ReferenceFrame, std::function<void()>,
      arch_nav::execution::WaypointExecutionState&) override {
    return CommandResponse::NOT_SUPPORTED;
  }
  CommandResponse execute_trajectory(
      std::vector<arch_nav::vehicle::TrajectoryPoint>, ReferenceFrame, std::function<void()>,
      arch_nav::execution::TrajectoryExecutionState&) override {
    return CommandResponse::NOT_SUPPORTED;
  }
  void stop() override {}
};

// Mirrors OasSupervisor::execute()'s real shape: it releases control itself,
// synchronously, before request_control() (which invoked it) has returned -
// this is exactly what triggered the self-deadlock on SupervisorChain's own
// mutex being held across the whole request_control() call.
struct SelfReleasingSupervisor : public ISupervisor {
  SupervisorChain* chain{nullptr};
  int execute_count{0};

  void start(IVehicleContextReader&, IOperationContextReader&, SupervisorChain& c) override {
    chain = &c;
  }
  void stop() override {}
  void execute(IOperationalController&, IOperationContextWriter&) override {
    ++execute_count;
    chain->release_control(*this);
  }
};

}  // namespace

TEST(SupervisorChainTest, RequestControlDoesNotDeadlockWhenExecuteReleasesImmediately) {
  NoopDispatcher dispatcher;
  OperationContext operation_context;
  OperationalController controller(operation_context, dispatcher);

  OperationContextWriterProxy writer_proxy(operation_context);
  OperationContextReaderProxy reader_proxy(operation_context);
  VehicleContext vehicle_context;
  VehicleContextReaderProxy vehicle_reader_proxy(vehicle_context);

  SupervisorChain chain(controller, writer_proxy);

  SelfReleasingSupervisor supervisor;
  chain.register_supervisor(supervisor, /*priority=*/10, PreemptionType::TRANSIENT);
  supervisor.start(vehicle_reader_proxy, reader_proxy, chain);

  // The call below must return - if SupervisorChain::request_control() still
  // held its mutex across the execute() call, release_control() (called
  // synchronously from inside execute()) would deadlock the test here
  // forever.
  chain.request_control(supervisor, PreemptionInfo{"test", "reason", ""});

  EXPECT_EQ(supervisor.execute_count, 1);

  // A second request must be accepted too - proof release_control() actually
  // cleared active_supervisor_ rather than the test merely not hanging.
  chain.request_control(supervisor, PreemptionInfo{"test", "reason2", ""});
  EXPECT_EQ(supervisor.execute_count, 2);
}
