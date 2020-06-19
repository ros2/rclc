// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// testing default transition sequence.
// This test requires that the transitions are set
// as depicted in design.ros2.org

#include <gtest/gtest.h>

#include "rclc_lifecycle/rclc_lifecycle.h"

#include "rcl/error_handling.h"
#include "lifecycle_msgs/msg/transition_event.h"
#include "lifecycle_msgs/srv/change_state.h"
#include "lifecycle_msgs/srv/get_available_states.h"
#include "lifecycle_msgs/srv/get_available_transitions.h"
#include "lifecycle_msgs/srv/get_state.h"

TEST(TestRclcLifecycle, lifecycle_node) {
  rcl_context_t context = rcl_get_zero_initialized_context();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_allocator_t allocator = rcl_get_default_allocator();

  rcl_init_options_init(&init_options, allocator);
  rcl_init(0, nullptr, &init_options, &context);

  rcl_node_t my_node = rcl_get_zero_initialized_node();
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rcl_node_init(&my_node, "lifecycle_node", "rclc", &context, &node_ops);

  rcl_lifecycle_state_machine_t state_machine_ = rcl_lifecycle_get_zero_initialized_state_machine();

  rclc_lifecycle_node_t lifecycle_node = rclc_make_node_a_lifecycle_node(
    &my_node,
    &state_machine_,
    &node_ops);

  EXPECT_EQ(rcl_lifecycle_state_machine_is_initialized(lifecycle_node.state_machine), RCL_RET_OK);
}
