#include <gtest/gtest.h>

#include "arch_nav/context/operation_context.hpp"
#include "protection_proxy/operation_context_writer_proxy.hpp"

using arch_nav::context::OperationContext;
using arch_nav::context::OperationContextWriterProxy;
using arch_nav::operation::Obstacle;
using arch_nav::vehicle::GlobalPosition;

TEST(OperationContextWriterProxy, SetObstacle) {
  OperationContext ctx;
  OperationContextWriterProxy proxy(ctx);

  proxy.set_obstacle(Obstacle{"obs1", GlobalPosition{40.0, -3.0, 50.0}, 5.0, 10.0});

  auto obstacles = ctx.get_obstacles();
  ASSERT_EQ(obstacles.size(), 1u);
  EXPECT_EQ(obstacles[0].id, "obs1");
}

TEST(OperationContextWriterProxy, RemoveObstacle) {
  OperationContext ctx;
  OperationContextWriterProxy proxy(ctx);

  proxy.set_obstacle(Obstacle{"obs1", GlobalPosition{40.0, -3.0, 50.0}, 5.0, 10.0});
  proxy.set_obstacle(Obstacle{"obs2", GlobalPosition{41.0, -2.0, 60.0}, 3.0, 8.0});
  proxy.remove_obstacle("obs1");

  auto obstacles = ctx.get_obstacles();
  ASSERT_EQ(obstacles.size(), 1u);
  EXPECT_EQ(obstacles[0].id, "obs2");
}

TEST(OperationContextWriterProxy, ClearObstacles) {
  OperationContext ctx;
  OperationContextWriterProxy proxy(ctx);

  proxy.set_obstacle(Obstacle{"obs1", GlobalPosition{40.0, -3.0, 50.0}, 5.0, 10.0});
  proxy.clear_obstacles();

  EXPECT_TRUE(ctx.get_obstacles().empty());
}
