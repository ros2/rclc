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

extern "C"
{
#include "rclc_lifecycle/rclc_lifecycle.h"

#include <lifecycle_msgs/msg/state.h>
#include <lifecycle_msgs/msg/transition.h>
}

TEST(TestRclcLifecycle, lifecycle_node) {
  rcl_context_t context = rcl_get_zero_initialized_context();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_allocator_t allocator = rcl_get_default_allocator();

  rcl_init_options_init(&init_options, allocator);
  rcl_init(0, nullptr, &init_options, &context);

  rcl_node_t my_node = rcl_get_zero_initialized_node();
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rcl_node_init(&my_node, "lifecycle_node", "rclc", &context, &node_ops);

  rclc_lifecycle_node_t lifecycle_node;
  rcl_lifecycle_state_machine_t state_machine_ = rcl_lifecycle_get_zero_initialized_state_machine();

  rcl_ret_t res = rclc_make_node_a_lifecycle_node(
    &lifecycle_node,
    &my_node,
    &state_machine_,
    &node_ops);

  EXPECT_EQ(RCL_RET_OK, res);
  EXPECT_EQ(
    RCL_RET_OK,
    rcl_lifecycle_state_machine_is_initialized(lifecycle_node.state_machine));
}

TEST(TestRclcLifecycle, lifecycle_node_transitions) {
  rcl_context_t context = rcl_get_zero_initialized_context();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_allocator_t allocator = rcl_get_default_allocator();

  rcl_init_options_init(&init_options, allocator);
  rcl_init(0, nullptr, &init_options, &context);

  rcl_node_t my_node = rcl_get_zero_initialized_node();
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rcl_node_init(&my_node, "lifecycle_node", "rclc", &context, &node_ops);

  rclc_lifecycle_node_t lifecycle_node;
  rcl_lifecycle_state_machine_t state_machine_ = rcl_lifecycle_get_zero_initialized_state_machine();

  rcl_ret_t res = rclc_make_node_a_lifecycle_node(
    &lifecycle_node,
    &my_node,
    &state_machine_,
    &node_ops);

  // configure
  res = rclc_lifecycle_change_state(
    &lifecycle_node,
    lifecycle_msgs__msg__Transition__TRANSITION_CONFIGURE,
    true);
  EXPECT_EQ(RCL_RET_OK, res);
  EXPECT_EQ(
    lifecycle_msgs__msg__State__PRIMARY_STATE_INACTIVE,
    lifecycle_node.state_machine->current_state->id);

  // activate
  res = rclc_lifecycle_change_state(
    &lifecycle_node,
    lifecycle_msgs__msg__Transition__TRANSITION_ACTIVATE,
    true);
  EXPECT_EQ(RCL_RET_OK, res);
  EXPECT_EQ(
    lifecycle_msgs__msg__State__PRIMARY_STATE_ACTIVE,
    lifecycle_node.state_machine->current_state->id);

  // deactivate
  res = rclc_lifecycle_change_state(
    &lifecycle_node,
    lifecycle_msgs__msg__Transition__TRANSITION_DEACTIVATE,
    true);
  EXPECT_EQ(RCL_RET_OK, res);
  EXPECT_EQ(
    lifecycle_msgs__msg__State__PRIMARY_STATE_INACTIVE,
    lifecycle_node.state_machine->current_state->id);

  // cleanup
  res = rclc_lifecycle_change_state(
    &lifecycle_node,
    lifecycle_msgs__msg__Transition__TRANSITION_CLEANUP,
    true);
  EXPECT_EQ(RCL_RET_OK, res);
  EXPECT_EQ(
    lifecycle_msgs__msg__State__PRIMARY_STATE_UNCONFIGURED,
    lifecycle_node.state_machine->current_state->id);
}
