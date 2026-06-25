#include <gtest/gtest.h>

#include "arch_nav/context/operation_context.hpp"
#include "protection_proxy/operation_context_reader_proxy.hpp"

using arch_nav::context::OperationContext;
using arch_nav::context::OperationContextReaderProxy;
using arch_nav::operation::Obstacle;
using arch_nav::vehicle::GlobalPosition;

TEST(OperationContextReaderProxy, ReadsObstacles) {
  OperationContext ctx;
  ctx.set_obstacle(Obstacle{"obs1", GlobalPosition{40.0, -3.0, 50.0}, 5.0, 10.0});
  OperationContextReaderProxy proxy(ctx);

  auto obstacles = proxy.get_obstacles();
  ASSERT_EQ(obstacles.size(), 1u);
  EXPECT_EQ(obstacles[0].id, "obs1");
}

TEST(OperationContextReaderProxy, SubscribeObstacles) {
  OperationContext ctx;
  OperationContextReaderProxy proxy(ctx);

  std::vector<Obstacle> received;
  proxy.subscribe_obstacles([&received](const std::vector<Obstacle>& obs) {
    received = obs;
  });

  ctx.set_obstacle(Obstacle{"obs1", GlobalPosition{40.0, -3.0, 50.0}, 5.0, 10.0});
  ASSERT_EQ(received.size(), 1u);
  EXPECT_EQ(received[0].id, "obs1");
}
