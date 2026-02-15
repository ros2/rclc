// Copyright (c) 2019 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/ros2/rclc.
// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>

extern "C"
{
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/action_client.h>
#include <example_interfaces/action/fibonacci.h>
}

#include <chrono>
#include <cmath>
#include <thread>
#include <memory>
#include <map>
#include <mutex>
#include <vector>
#include <utility>

#define RCLC_MAX_GOALS 10

using namespace std::chrono_literals;

#define GOAL_STATE_SUCCEEDED action_msgs__msg__GoalStatus__STATUS_SUCCEEDED
#define GOAL_STATE_ABORTED action_msgs__msg__GoalStatus__STATUS_ABORTED
#define GOAL_STATE_CANCELED action_msgs__msg__GoalStatus__STATUS_CANCELED

TEST(Test, rclc_action_server) {
  rclc_support_t support;
  rcl_node_t node;
  rcl_ret_t rc;

  // preliminary setup
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rc = rclc_support_init(&support, 0, nullptr, &allocator);
  rc = rclc_node_init_default(&node, "my_node", "my_namespace", &support);
  EXPECT_EQ(RCL_RET_OK, rc);

  // test action server with valid arguments
  rclc_action_server_t action_server;
  rc = rclc_action_server_init_default(
    &action_server,
    &node,
    &support,
    ROSIDL_GET_ACTION_TYPE_SUPPORT(example_interfaces, Fibonacci),
    "fibonacci"
  );
  EXPECT_EQ(RCL_RET_OK, rc);

  // test action server with invalid arguments
  rclc_action_server_t invalid_action_server;
  rc = rclc_action_server_init_default(
    nullptr,
    &node,
    &support,
    ROSIDL_GET_ACTION_TYPE_SUPPORT(example_interfaces, Fibonacci),
    "fibonacci"
  );
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, rc);
  rcutils_reset_error();

  rc = rclc_action_server_init_default(
    &invalid_action_server,
    nullptr,
    &support,
    ROSIDL_GET_ACTION_TYPE_SUPPORT(example_interfaces, Fibonacci),
    "fibonacci"
  );
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, rc);
  rcutils_reset_error();

  rc = rclc_action_server_init_default(
    &invalid_action_server,
    &node,
    nullptr,
    ROSIDL_GET_ACTION_TYPE_SUPPORT(example_interfaces, Fibonacci),
    "fibonacci"
  );
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, rc);
  rcutils_reset_error();

  rc = rclc_action_server_init_default(
    &invalid_action_server,
    &node,
    &support,
    nullptr,
    "fibonacci"
  );
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, rc);
  rcutils_reset_error();

  rc = rclc_action_server_init_default(
    &invalid_action_server,
    &node,
    &support,
    ROSIDL_GET_ACTION_TYPE_SUPPORT(example_interfaces, Fibonacci),
    nullptr
  );
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, rc);
  rcutils_reset_error();

  // Create executor
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 1, &allocator);

  example_interfaces__action__Fibonacci_SendGoal_Request ros_goal_request[RCLC_MAX_GOALS];

  rc = rclc_executor_add_action_server(
    &executor,
    &action_server,
    RCLC_MAX_GOALS,
    ros_goal_request,
    sizeof(example_interfaces__action__Fibonacci_SendGoal_Request),
    [](rclc_action_goal_handle_t * /* goal_handle */, void * /* context */) -> rcl_ret_t {
      return RCL_RET_ACTION_GOAL_REJECTED;
    },
    [](rclc_action_goal_handle_t * /* goal_handle */, void * /* context */) -> bool {
      return false;
    },
    &action_server);

  EXPECT_EQ(RCL_RET_OK, rc);

  rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  EXPECT_EQ(RCL_RET_OK, rc);

  // clean up
  rc = rclc_action_server_fini(&action_server, &node);
  EXPECT_EQ(RCL_RET_OK, rc);
  rc = rcl_node_fini(&node);
  EXPECT_EQ(RCL_RET_OK, rc);
  rc = rclc_support_fini(&support);
  EXPECT_EQ(RCL_RET_OK, rc);
}

class ActionServerTest : public ::testing::Test
{
public:
  ActionServerTest() {}

  ~ActionServerTest() {}

  void SetUp() override
  {
    rcl_ret_t rc;

    // Init support
    allocator = rcl_get_default_allocator();
    rc = rclc_support_init(&support, 0, nullptr, &allocator);
    EXPECT_EQ(RCL_RET_OK, rc);

    node = rcl_get_zero_initialized_node();
    rc = rclc_node_init_default(&node, "my_node", "", &support);
    EXPECT_EQ(RCL_RET_OK, rc);

    // Init action server
    rc = rclc_action_server_init_default(
      &action_server,
      &node,
      &support,
      ROSIDL_GET_ACTION_TYPE_SUPPORT(example_interfaces, Fibonacci),
      "fibonacci"
    );
    EXPECT_EQ(RCL_RET_OK, rc);

    // Init executor
    rclc_executor_init(&executor, &support.context, 1, &allocator);

    rc = rclc_executor_add_action_server(
      &executor,
      &action_server,
      RCLC_MAX_GOALS,
      ros_goal_request,
      sizeof(example_interfaces__action__Fibonacci_SendGoal_Request),
      handle_goal_dispatcher,
      handle_cancel_dispatcher,
      this);

    EXPECT_EQ(RCL_RET_OK, rc);

    run_server = true;
    server_thread = std::thread(
      [&]() {
        while (run_server) {
          rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }
      });

    // Init RCLC action client (test helper)
    client_node = rcl_get_zero_initialized_node();
    rc = rclc_node_init_default(&client_node, "action_client_node", "", &support);
    EXPECT_EQ(RCL_RET_OK, rc);

    rc = rclc_action_client_init_default(
      &action_client,
      &client_node,
      ROSIDL_GET_ACTION_TYPE_SUPPORT(example_interfaces, Fibonacci),
      "fibonacci"
    );
    EXPECT_EQ(RCL_RET_OK, rc);

    // Init client executor
    rclc_executor_init(&client_executor, &support.context, 1, &allocator);

    ros_feedback.feedback.sequence.capacity = RCLC_MAX_GOALS;
    ros_feedback.feedback.sequence.size = 0;
    ros_feedback.feedback.sequence.data = reinterpret_cast<int32_t *>(malloc(
        ros_feedback.feedback.sequence.capacity * sizeof(int32_t)));

    ros_result_response.result.sequence.capacity = RCLC_MAX_GOALS;
    ros_result_response.result.sequence.size = 0;
    ros_result_response.result.sequence.data = reinterpret_cast<int32_t *>(malloc(
        ros_result_response.result.sequence.capacity * sizeof(int32_t)));

    rc = rclc_executor_add_action_client(
      &client_executor,
      &action_client,
      RCLC_MAX_GOALS,
      &ros_result_response,
      &ros_feedback,
      client_handle_goal_dispatcher,
      client_feedback_dispatcher,
      client_result_dispatcher,
      client_handle_cancel_dispatcher,
      this);
    EXPECT_EQ(RCL_RET_OK, rc);

    // Set default callbacks
    client_handle_goal = [](rclc_action_goal_handle_t *, bool, void *) {};
    client_handle_feedback = [](rclc_action_goal_handle_t *, void *, void *) {};
    client_handle_result = [](rclc_action_goal_handle_t *, void *, void *) {};
    client_handle_cancel = [](rclc_action_goal_handle_t *, bool, void *) {};

    // Wait for action server match
    bool server_matched = false;
    for (size_t i = 0; i < 10; i++) {
      rc = rcl_action_server_is_available(
        &client_node, &action_client.rcl_handle, &server_matched);

      if (server_matched) {
        break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    EXPECT_EQ(RCL_RET_OK, rc);
    EXPECT_TRUE(server_matched);
  }

  void TearDown() override
  {
    rcl_ret_t rc;

    run_server = false;
    server_thread.join();

    rc = rclc_executor_fini(&client_executor);
    EXPECT_EQ(RCL_RET_OK, rc);
    rc = rclc_action_client_fini(&action_client, &client_node);
    EXPECT_EQ(RCL_RET_OK, rc);
    rc = rcl_node_fini(&client_node);
    EXPECT_EQ(RCL_RET_OK, rc);

    rc = rclc_action_server_fini(&action_server, &node);
    EXPECT_EQ(RCL_RET_OK, rc);
    rc = rcl_node_fini(&node);
    EXPECT_EQ(RCL_RET_OK, rc);
    rc = rclc_support_fini(&support);
    EXPECT_EQ(RCL_RET_OK, rc);

    free(ros_feedback.feedback.sequence.data);
    free(ros_result_response.result.sequence.data);
  }

  static rcl_ret_t handle_goal_dispatcher(rclc_action_goal_handle_t * goal_handle, void * context)
  {
    return static_cast<ActionServerTest *>(context)->handle_goal(goal_handle, context);
  }

  static bool handle_cancel_dispatcher(rclc_action_goal_handle_t * goal_handle, void * context)
  {
    return static_cast<ActionServerTest *>(context)->handle_cancel(goal_handle, context);
  }

  static void client_handle_goal_dispatcher(
    rclc_action_goal_handle_t * goal_handle, bool accepted, void * context)
  {
    static_cast<ActionServerTest *>(context)->client_handle_goal(goal_handle, accepted, context);
  }

  static void client_feedback_dispatcher(
    rclc_action_goal_handle_t * goal_handle, void * ros_feedback, void * context)
  {
    static_cast<ActionServerTest *>(context)->client_handle_feedback(
      goal_handle, ros_feedback, context);
  }

  static void client_result_dispatcher(
    rclc_action_goal_handle_t * goal_handle, void * ros_result_response, void * context)
  {
    static_cast<ActionServerTest *>(context)->client_handle_result(
      goal_handle, ros_result_response, context);
  }

  static void client_handle_cancel_dispatcher(
    rclc_action_goal_handle_t * goal_handle, bool cancelled, void * context)
  {
    static_cast<ActionServerTest *>(context)->client_handle_cancel(
      goal_handle, cancelled, context);
  }

protected:
  // RCLC members
  rclc_support_t support;
  rcl_allocator_t allocator;

  rcl_node_t node;
  rclc_action_server_t action_server;
  rclc_executor_t executor;

  example_interfaces__action__Fibonacci_SendGoal_Request ros_goal_request[RCLC_MAX_GOALS];

  std::function<rcl_ret_t(rclc_action_goal_handle_t *, void *)> handle_goal;
  std::function<rcl_ret_t(rclc_action_goal_handle_t *, void *)> handle_cancel;

  bool run_server;
  std::thread server_thread;

  // RCLC action client (test helper) members
  rcl_node_t client_node;
  rclc_action_client_t action_client;
  rclc_executor_t client_executor;

  example_interfaces__action__Fibonacci_FeedbackMessage ros_feedback;
  example_interfaces__action__Fibonacci_GetResult_Response ros_result_response;

  std::function<void(rclc_action_goal_handle_t *, bool, void *)> client_handle_goal;
  std::function<void(rclc_action_goal_handle_t *, void *, void *)> client_handle_feedback;
  std::function<void(rclc_action_goal_handle_t *, void *, void *)> client_handle_result;
  std::function<void(rclc_action_goal_handle_t *, bool, void *)> client_handle_cancel;
};

inline bool operator<(
  const unique_identifier_msgs__msg__UUID & lhs,
  const unique_identifier_msgs__msg__UUID & rhs)
{
  uint64_t lhs_high = *(reinterpret_cast<const uint64_t *>(&lhs.uuid[0]));
  uint64_t lhs_low = *(reinterpret_cast<const uint64_t *>(&lhs.uuid[8]));

  uint64_t rhs_high = *(reinterpret_cast<const uint64_t *>(&rhs.uuid[0]));
  uint64_t rhs_low = *(reinterpret_cast<const uint64_t *>(&rhs.uuid[8]));

  return (rhs_high == lhs_high) ? (rhs_low < lhs_low) : (rhs_high < lhs_high);
}

TEST_F(ActionServerTest, goal_accept) {
  example_interfaces__action__Fibonacci_SendGoal_Request goal_request;
  goal_request.goal.order = 10;

  // Prepare RCLC server
  handle_goal = [&](rclc_action_goal_handle_t * goal_handle, void * /* context */) -> rcl_ret_t {
      example_interfaces__action__Fibonacci_SendGoal_Request * req =
        reinterpret_cast<example_interfaces__action__Fibonacci_SendGoal_Request *>(
        goal_handle->ros_goal_request);
      EXPECT_EQ(req->goal.order, goal_request.goal.order);
      return RCL_RET_ACTION_GOAL_ACCEPTED;
    };

  // Run RCLC client
  bool goal_accepted = false;
  client_handle_goal = [&](rclc_action_goal_handle_t * /* goal_handle */, bool accepted,
    void * /* context */) {
      ASSERT_TRUE(accepted);
      goal_accepted = true;
    };

  rcl_ret_t rc = rclc_action_send_goal_request(&action_client, &goal_request, NULL);
  EXPECT_EQ(RCL_RET_OK, rc);

  while (!goal_accepted) {
    rclc_executor_spin_some(&client_executor, RCL_MS_TO_NS(100));
  }

  ASSERT_TRUE(goal_accepted);
}

TEST_F(ActionServerTest, goal_reject) {
  example_interfaces__action__Fibonacci_SendGoal_Request goal_request;
  goal_request.goal.order = 10;

  // Prepare RCLC server
  handle_goal =
    [](rclc_action_goal_handle_t * /* goal_handle */, void * /* context */) -> rcl_ret_t {
      return RCL_RET_ACTION_GOAL_REJECTED;
    };

  // Run RCLC client
  bool goal_rejected = false;
  client_handle_goal = [&](rclc_action_goal_handle_t * /* goal_handle */, bool accepted,
    void * /* context */) {
      ASSERT_FALSE(accepted);
      goal_rejected = true;
    };

  rcl_ret_t rc = rclc_action_send_goal_request(&action_client, &goal_request, NULL);
  EXPECT_EQ(RCL_RET_OK, rc);

  while (!goal_rejected) {
    rclc_executor_spin_some(&client_executor, RCL_MS_TO_NS(100));
  }

  ASSERT_TRUE(goal_rejected);
}

TEST_F(ActionServerTest, goal_accept_feedback_and_result) {
  example_interfaces__action__Fibonacci_SendGoal_Request goal_request;
  goal_request.goal.order = 10;

  // Prepare RCLC server
  std::thread feedback_thread;

  handle_goal = [&](rclc_action_goal_handle_t * goal_handle, void * /* context */) -> rcl_ret_t {
      feedback_thread = std::thread(
        [ = ]() {
          std::this_thread::sleep_for(100ms);

          int32_t data[] = {0, 1, 2};
          example_interfaces__action__Fibonacci_FeedbackMessage feedback;
          feedback.feedback.sequence.capacity = sizeof(data) / sizeof(data[0]);
          feedback.feedback.sequence.size = feedback.feedback.sequence.capacity;
          feedback.feedback.sequence.data = data;
          for (size_t i = 0; i < 10; i++) {
            rcl_ret_t rc = rclc_action_publish_feedback(goal_handle, &feedback);
            EXPECT_EQ(RCL_RET_OK, rc);
            std::this_thread::sleep_for(10ms);
          }

          example_interfaces__action__Fibonacci_GetResult_Response response;
          response.result.sequence.capacity = feedback.feedback.sequence.capacity;
          response.result.sequence.size = feedback.feedback.sequence.size;
          response.result.sequence.data = feedback.feedback.sequence.data;
          rcl_ret_t rc = rclc_action_send_result(goal_handle, GOAL_STATE_SUCCEEDED, &response);
          EXPECT_EQ(RCL_RET_OK, rc);
        });

      return RCL_RET_ACTION_GOAL_ACCEPTED;
    };

  // Run RCLC client
  bool goal_accepted = false;
  client_handle_goal = [&](rclc_action_goal_handle_t * /* goal_handle */, bool accepted,
    void * /* context */) {
      ASSERT_TRUE(accepted);
      goal_accepted = true;
    };

  size_t feedback_received = 0;
  client_handle_feedback = [&](rclc_action_goal_handle_t * /* goal_handle */, void * ros_feedback,
    void * /* context */) {
      feedback_received++;
      example_interfaces__action__Fibonacci_FeedbackMessage * feedback =
        reinterpret_cast<example_interfaces__action__Fibonacci_FeedbackMessage *>(ros_feedback);
      ASSERT_EQ(feedback->feedback.sequence.size, 3U);
    };

  bool result_received = false;
  client_handle_result = [&](rclc_action_goal_handle_t * /* goal_handle */,
    void * ros_result_response, void * /* context */) {
      result_received = true;
      example_interfaces__action__Fibonacci_GetResult_Response * result =
        reinterpret_cast<example_interfaces__action__Fibonacci_GetResult_Response *>(
        ros_result_response);
      ASSERT_EQ(result->status, GOAL_STATE_SUCCEEDED);
    };

  rcl_ret_t rc = rclc_action_send_goal_request(&action_client, &goal_request, NULL);
  EXPECT_EQ(RCL_RET_OK, rc);

  while (!goal_accepted || !result_received) {
    rclc_executor_spin_some(&client_executor, RCL_MS_TO_NS(100));
  }

  feedback_thread.join();

  ASSERT_TRUE(goal_accepted);
  ASSERT_TRUE(result_received);
  ASSERT_EQ(feedback_received, 10U);
}

TEST_F(ActionServerTest, goal_accept_feedback_and_abort) {
  example_interfaces__action__Fibonacci_SendGoal_Request goal_request;
  goal_request.goal.order = 10;

  // Prepare RCLC server
  std::thread feedback_thread;

  handle_goal = [&](rclc_action_goal_handle_t * goal_handle, void * /* context */) -> rcl_ret_t {
      feedback_thread = std::thread(
        [ = ]() {
          std::this_thread::sleep_for(100ms);

          int32_t data[] = {0, 1, 2};
          example_interfaces__action__Fibonacci_FeedbackMessage feedback;
          feedback.feedback.sequence.capacity = sizeof(data) / sizeof(data[0]);
          feedback.feedback.sequence.size = feedback.feedback.sequence.capacity;
          feedback.feedback.sequence.data = data;
          for (size_t i = 0; i < 5; i++) {
            rcl_ret_t rc = rclc_action_publish_feedback(goal_handle, &feedback);
            EXPECT_EQ(RCL_RET_OK, rc);
            std::this_thread::sleep_for(10ms);
          }

          example_interfaces__action__Fibonacci_GetResult_Response response = {};
          rcl_ret_t rc = rclc_action_send_result(goal_handle, GOAL_STATE_ABORTED, &response);
          EXPECT_EQ(RCL_RET_OK, rc);
        });

      return RCL_RET_ACTION_GOAL_ACCEPTED;
    };

  // Run RCLC client
  bool goal_accepted = false;
  client_handle_goal = [&](rclc_action_goal_handle_t * /* goal_handle */, bool accepted,
    void * /* context */) {
      ASSERT_TRUE(accepted);
      goal_accepted = true;
    };

  size_t feedback_received = 0;
  client_handle_feedback = [&](rclc_action_goal_handle_t * /* goal_handle */, void * ros_feedback,
    void * /* context */) {
      feedback_received++;
      example_interfaces__action__Fibonacci_FeedbackMessage * feedback =
        reinterpret_cast<example_interfaces__action__Fibonacci_FeedbackMessage *>(ros_feedback);
      ASSERT_EQ(feedback->feedback.sequence.size, 3U);
    };

  bool result_received = false;
  client_handle_result = [&](rclc_action_goal_handle_t * /* goal_handle */,
    void * ros_result_response, void * /* context */) {
      result_received = true;
      example_interfaces__action__Fibonacci_GetResult_Response * result =
        reinterpret_cast<example_interfaces__action__Fibonacci_GetResult_Response *>(
        ros_result_response);
      ASSERT_EQ(result->status, GOAL_STATE_ABORTED);
    };

  rcl_ret_t rc = rclc_action_send_goal_request(&action_client, &goal_request, NULL);
  EXPECT_EQ(RCL_RET_OK, rc);

  while (!goal_accepted || !result_received) {
    rclc_executor_spin_some(&client_executor, RCL_MS_TO_NS(100));
  }

  feedback_thread.join();

  ASSERT_TRUE(goal_accepted);
  ASSERT_TRUE(result_received);
  ASSERT_EQ(feedback_received, 5U);
}

TEST_F(ActionServerTest, goal_accept_cancel_success) {
  example_interfaces__action__Fibonacci_SendGoal_Request goal_request;
  goal_request.goal.order = 10;

  // Prepare RCLC server
  std::thread feedback_thread;
  handle_goal =
    [&](rclc_action_goal_handle_t * goal_handle, void * /* context */) -> rcl_ret_t {
      feedback_thread = std::thread(
        [ = ]() {
          std::this_thread::sleep_for(100ms);
          for (size_t i = 0; i < 10 && !goal_handle->goal_cancelled; i++) {
            std::this_thread::sleep_for(100ms);
          }

          example_interfaces__action__Fibonacci_GetResult_Response response = {};
          int8_t status;
          if (goal_handle->goal_cancelled) {
            status = GOAL_STATE_CANCELED;
          } else {
            status = GOAL_STATE_ABORTED;
          }
          rcl_ret_t rc = rclc_action_send_result(goal_handle, status, &response);
          EXPECT_EQ(RCL_RET_OK, rc);
        });

      return RCL_RET_ACTION_GOAL_ACCEPTED;
    };

  handle_cancel = [&](rclc_action_goal_handle_t * goal_handle, void * /* context */) -> bool {
      goal_handle->goal_cancelled = true;
      return true;
    };

  // Run RCLC client
  bool goal_accepted = false;
  client_handle_goal = [&](rclc_action_goal_handle_t * /* goal_handle */, bool accepted,
    void * /* context */) {
      ASSERT_TRUE(accepted);
      goal_accepted = true;
    };

  bool cancel_accepted = false;
  client_handle_cancel = [&](rclc_action_goal_handle_t * /* goal_handle */, bool cancelled,
    void * /* context */) {
      cancel_accepted = cancelled;
    };

  bool result_received = false;
  client_handle_result = [&](rclc_action_goal_handle_t * /* goal_handle */,
    void * ros_result_response, void * /* context */) {
      result_received = true;
      example_interfaces__action__Fibonacci_GetResult_Response * result =
        reinterpret_cast<example_interfaces__action__Fibonacci_GetResult_Response *>(
        ros_result_response);
      ASSERT_EQ(result->status, GOAL_STATE_CANCELED);
    };

  rclc_action_goal_handle_t * goal_handle;
  rcl_ret_t rc = rclc_action_send_goal_request(&action_client, &goal_request, &goal_handle);
  EXPECT_EQ(RCL_RET_OK, rc);

  while (!goal_accepted) {
    rclc_executor_spin_some(&client_executor, RCL_MS_TO_NS(100));
  }

  std::this_thread::sleep_for(100ms);

  rc = rclc_action_send_cancel_request(goal_handle);
  EXPECT_EQ(RCL_RET_OK, rc);

  size_t max_iterations = 100;
  size_t iterations = 0;
  while ((!cancel_accepted || !result_received) && iterations < max_iterations) {
    rclc_executor_spin_some(&client_executor, RCL_MS_TO_NS(100));
    iterations++;
  }

  feedback_thread.join();
  ASSERT_TRUE(goal_accepted);
  ASSERT_TRUE(cancel_accepted);
  ASSERT_TRUE(result_received);
}

TEST_F(ActionServerTest, goal_accept_cancel_reject) {
  example_interfaces__action__Fibonacci_SendGoal_Request goal_request;
  goal_request.goal.order = 10;

  // Prepare RCLC server
  handle_goal =
    [&](rclc_action_goal_handle_t * /* goal_handle */, void * /* context */) -> rcl_ret_t {
      return RCL_RET_ACTION_GOAL_ACCEPTED;
    };

  handle_cancel = [&](rclc_action_goal_handle_t * /* goal_handle */, void * /* context */) -> bool {
      return false;
    };

  // Run RCLC client
  bool goal_accepted = false;
  client_handle_goal = [&](rclc_action_goal_handle_t * /* goal_handle */, bool accepted,
    void * /* context */) {
      ASSERT_TRUE(accepted);
      goal_accepted = true;
    };

  bool cancel_rejected = false;
  client_handle_cancel = [&](rclc_action_goal_handle_t * /* goal_handle */, bool cancelled,
    void * /* context */) {
      ASSERT_FALSE(cancelled);
      cancel_rejected = true;
    };

  rclc_action_goal_handle_t * goal_handle;
  rcl_ret_t rc = rclc_action_send_goal_request(&action_client, &goal_request, &goal_handle);
  EXPECT_EQ(RCL_RET_OK, rc);

  while (!goal_accepted) {
    rclc_executor_spin_some(&client_executor, RCL_MS_TO_NS(100));
  }

  std::this_thread::sleep_for(100ms);

  rc = rclc_action_send_cancel_request(goal_handle);
  EXPECT_EQ(RCL_RET_OK, rc);

  while (!cancel_rejected) {
    rclc_executor_spin_some(&client_executor, RCL_MS_TO_NS(100));
  }

  ASSERT_TRUE(goal_accepted);
  ASSERT_TRUE(cancel_rejected);
}

TEST_F(ActionServerTest, multi_goal_accept_feedback_and_result) {
  // Prepare RCLC server
  std::vector<std::thread> feedback_thread_pool;
  std::mutex thread_pool_mutex;

  size_t feedback_per_goal = 10;

  handle_goal = [&](rclc_action_goal_handle_t * goal_handle, void * /* context */) -> rcl_ret_t {
      std::thread worker = std::thread(
        [ = ]() {
          std::this_thread::sleep_for(100ms);

          example_interfaces__action__Fibonacci_SendGoal_Request * req =
          reinterpret_cast<example_interfaces__action__Fibonacci_SendGoal_Request *>(goal_handle->
          ros_goal_request);

          std::vector<int32_t> data(req->goal.order);

          example_interfaces__action__Fibonacci_FeedbackMessage feedback;
          feedback.feedback.sequence.capacity = data.size();
          feedback.feedback.sequence.size = feedback.feedback.sequence.capacity;
          feedback.feedback.sequence.data = &data[0];
          for (size_t i = 0; i < feedback_per_goal; i++) {
            rcl_ret_t rc = rclc_action_publish_feedback(goal_handle, &feedback);
            EXPECT_EQ(RCL_RET_OK, rc);
            std::this_thread::sleep_for(10ms);
          }

          example_interfaces__action__Fibonacci_GetResult_Response response;
          response.result.sequence.capacity = feedback.feedback.sequence.capacity;
          response.result.sequence.size = feedback.feedback.sequence.size;
          response.result.sequence.data = feedback.feedback.sequence.data;
          rclc_action_send_result(goal_handle, GOAL_STATE_SUCCEEDED, &response);
        });

      std::lock_guard<std::mutex> lock(thread_pool_mutex);
      feedback_thread_pool.push_back(std::move(worker));

      return RCL_RET_ACTION_GOAL_ACCEPTED;
    };

  // Run RCLC client
  std::map<unique_identifier_msgs__msg__UUID, size_t> goals;
  std::map<unique_identifier_msgs__msg__UUID, bool> goals_accepted;

  client_handle_goal = [&](rclc_action_goal_handle_t * goal_handle, bool accepted,
    void * /* context */) {
      ASSERT_TRUE(accepted);
      goals_accepted[goal_handle->goal_id] = true;
    };

  size_t feedback_received = 0;
  client_handle_feedback = [&](rclc_action_goal_handle_t * goal_handle, void * ros_feedback,
    void * /* context */) {
      feedback_received++;
      example_interfaces__action__Fibonacci_FeedbackMessage * feedback =
        reinterpret_cast<example_interfaces__action__Fibonacci_FeedbackMessage *>(ros_feedback);
      size_t feedback_size = goals[goal_handle->goal_id];
      ASSERT_EQ(feedback->feedback.sequence.size, feedback_size);
    };

  client_handle_result = [&](rclc_action_goal_handle_t * goal_handle,
    void * ros_result_response, void * /* context */) {
      example_interfaces__action__Fibonacci_GetResult_Response * result =
        reinterpret_cast<example_interfaces__action__Fibonacci_GetResult_Response *>(
        ros_result_response);
      ASSERT_EQ(result->status, GOAL_STATE_SUCCEEDED);
      ASSERT_TRUE(goals_accepted[goal_handle->goal_id]);
      goals.erase(goal_handle->goal_id);
    };

  size_t num_goals = std::floor(RCLC_MAX_GOALS / 2);

  for (size_t i = 0; i < num_goals; i++) {
    example_interfaces__action__Fibonacci_SendGoal_Request goal_request;
    goal_request.goal.order = 10 * (i + 1);

    rclc_action_goal_handle_t * goal_handle;
    rcl_ret_t rc = rclc_action_send_goal_request(&action_client, &goal_request, &goal_handle);
    EXPECT_EQ(RCL_RET_OK, rc);

    goals.insert({goal_handle->goal_id, goal_request.goal.order});
    goals_accepted.insert({goal_handle->goal_id, false});
  }

  // Spin until all goals complete
  while (goals.size() > 0) {
    rclc_executor_spin_some(&client_executor, RCL_MS_TO_NS(100));
  }

  for (auto & thread : feedback_thread_pool) {
    thread.join();
  }

  ASSERT_EQ(goals_accepted.size(), num_goals);

  for (auto const & x : goals_accepted) {
    ASSERT_TRUE(x.second);
  }

  ASSERT_EQ(feedback_received, num_goals * feedback_per_goal);
  ASSERT_EQ(goals.size(), 0U);
}

TEST(Test, rclc_action_server_regression_1) {
  rclc_support_t support;
  rcl_node_t node;
  rcl_ret_t rc;

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rc = rclc_support_init(&support, 0, nullptr, &allocator);
  rc = rclc_node_init_default(&node, "my_node", "my_namespace", &support);
  EXPECT_EQ(RCL_RET_OK, rc);

  rclc_action_server_t action_server;
  rc = rclc_action_server_init_default(
    &action_server,
    &node,
    &support,
    ROSIDL_GET_ACTION_TYPE_SUPPORT(example_interfaces, Fibonacci),
    "fibonacci"
  );
  EXPECT_EQ(RCL_RET_OK, rc);

  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 1, &allocator);

  example_interfaces__action__Fibonacci_SendGoal_Request ros_goal_request[RCLC_MAX_GOALS];

  rc = rclc_executor_add_action_server(
    &executor,
    &action_server,
    RCLC_MAX_GOALS,
    ros_goal_request,
    sizeof(example_interfaces__action__Fibonacci_SendGoal_Request),
    [](rclc_action_goal_handle_t * /* goal_handle */, void * /* context */) -> rcl_ret_t {
      return RCL_RET_ACTION_GOAL_REJECTED;
    },
    [](rclc_action_goal_handle_t * /* goal_handle */, void * /* context */) -> bool {
      return false;
    },
    &action_server);

  EXPECT_EQ(RCL_RET_OK, rc);

  EXPECT_EQ(RCL_RET_OK, rclc_action_server_fini(&action_server, &node));

  // Second time should be safe
  EXPECT_EQ(RCL_RET_OK, rclc_action_server_fini(&action_server, &node));
}

int main(int args, char ** argv)
{
  ::testing::InitGoogleTest(&args, argv);
  return RUN_ALL_TESTS();
}
