#include <gtest/gtest.h>

#include "arch_nav/context/vehicle_context.hpp"
#include "protection_proxy/vehicle_context_reader_proxy.hpp"

using arch_nav::constants::ArmState;
using arch_nav::constants::ControlState;
using arch_nav::context::VehicleContext;
using arch_nav::context::VehicleContextReaderProxy;
using arch_nav::vehicle::GlobalPosition;
using arch_nav::vehicle::Kinematics;
using arch_nav::vehicle::VehicleStatus;

TEST(VehicleContextReaderProxy, ReadsGlobalPosition) {
  VehicleContext ctx;
  ctx.update(GlobalPosition{40.0, -3.0, 100.0});
  VehicleContextReaderProxy proxy(ctx);

  auto gp = proxy.get_global_position();
  EXPECT_DOUBLE_EQ(gp.lat, 40.0);
  EXPECT_DOUBLE_EQ(gp.lon, -3.0);
  EXPECT_DOUBLE_EQ(gp.alt, 100.0);
}

TEST(VehicleContextReaderProxy, ReadsKinematics) {
  VehicleContext ctx;
  Kinematics kin;
  kin.vx = 2.0;
  kin.heading = 1.0;
  ctx.update(kin);
  VehicleContextReaderProxy proxy(ctx);

  auto stored = proxy.get_kinematic();
  EXPECT_DOUBLE_EQ(stored.vx, 2.0);
  EXPECT_DOUBLE_EQ(stored.heading, 1.0);
}

TEST(VehicleContextReaderProxy, ReadsVehicleStatus) {
  VehicleContext ctx;
  ctx.update(VehicleStatus(ControlState::KERNEL_CONTROLLED, ArmState::ARMED));
  VehicleContextReaderProxy proxy(ctx);

  auto vs = proxy.get_vehicle_status();
  EXPECT_EQ(vs.control_state, ControlState::KERNEL_CONTROLLED);
  EXPECT_EQ(vs.arm_state, ArmState::ARMED);
}

TEST(VehicleContextReaderProxy, SubscribeVehicleStatus) {
  VehicleContext ctx;
  VehicleContextReaderProxy proxy(ctx);

  VehicleStatus received;
  proxy.subscribe_vehicle_status([&received](const VehicleStatus& vs) {
    received = vs;
  });

  ctx.update(VehicleStatus(ControlState::EXTERNAL, ArmState::DISARMED));
  EXPECT_EQ(received.control_state, ControlState::EXTERNAL);
  EXPECT_EQ(received.arm_state, ArmState::DISARMED);
}
