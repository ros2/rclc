// Copyright (c) 2021 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/ros2/rclc.
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

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <example_interfaces/action/fibonacci.h>


bool cancel_goal = false;

pthread_mutex_t rclc_mutex;

void * fibonacci_worker(void * args)
{
  (void) args;
  rclc_action_goal_handle_t * goal_handle = (rclc_action_goal_handle_t *) args;

  example_interfaces__action__Fibonacci_SendGoal_Request * req =
    (example_interfaces__action__Fibonacci_SendGoal_Request *) goal_handle->ros_goal_request;

  if (req->goal.order < 2) {
    pthread_mutex_lock(&rclc_mutex);
    rclc_action_server_finish_goal_abort(goal_handle, NULL);
    pthread_mutex_unlock(&rclc_mutex);
  }

  example_interfaces__action__Fibonacci_FeedbackMessage feedback;
  feedback.feedback.sequence.capacity = req->goal.order;
  feedback.feedback.sequence.size = 0;
  feedback.feedback.sequence.data =
    (int32_t *) malloc(feedback.feedback.sequence.capacity * sizeof(int32_t));

  feedback.feedback.sequence.data[0] = 0;
  feedback.feedback.sequence.data[1] = 1;
  feedback.feedback.sequence.size = 2;

  for (size_t i = 2; i < (size_t) req->goal.order && !cancel_goal; i++) {
    feedback.feedback.sequence.data[i] = feedback.feedback.sequence.data[i - 1] +
      feedback.feedback.sequence.data[i - 2];
    feedback.feedback.sequence.size++;

    if (i > 30) {
      rclc_action_server_finish_goal_abort(goal_handle, NULL);
      return NULL;
    }

    pthread_mutex_lock(&rclc_mutex);
    rclc_action_publish_feedback(goal_handle, &feedback);
    pthread_mutex_unlock(&rclc_mutex);
    usleep(100000);
  }

  if (!cancel_goal) {
    example_interfaces__action__Fibonacci_GetResult_Response response;
    response.result.sequence.capacity = feedback.feedback.sequence.capacity;
    response.result.sequence.size = feedback.feedback.sequence.size;
    response.result.sequence.data = feedback.feedback.sequence.data;

    pthread_mutex_lock(&rclc_mutex);
    rclc_action_server_finish_goal_sucess(goal_handle, &response);
    pthread_mutex_unlock(&rclc_mutex);
  } else {
    rclc_action_server_finish_goal_abort(goal_handle, NULL);
  }

  // active_worker = false;
  cancel_goal = false;

  free(feedback.feedback.sequence.data);
  pthread_exit(NULL);
}

rcl_ret_t handle_goal(rclc_action_goal_handle_t * goal_handle, void * context)
{
  (void) context;

  example_interfaces__action__Fibonacci_SendGoal_Request * req =
    (example_interfaces__action__Fibonacci_SendGoal_Request *) goal_handle->ros_goal_request;

  // Too big, rejecting
  if (req->goal.order > 200) {
    return RCL_RET_ACTION_GOAL_REJECTED;
  }

  pthread_t * thread_id = malloc(sizeof(pthread_t));

  pthread_create(thread_id, NULL, fibonacci_worker, goal_handle);
  return RCL_RET_ACTION_GOAL_ACCEPTED;
}

bool handle_cancel(rclc_action_goal_handle_t * goal_handle, void * context)
{
  (void) context;
  (void) goal_handle;

  cancel_goal = true;

  return cancel_goal;
}

int main()
{
  pthread_mutex_init(&rclc_mutex, NULL);

  rcl_allocator_t allocator = rcl_get_default_allocator();

  // Create init_options
  rclc_support_t support;
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create node
  rcl_node_t node;
  rclc_node_init_default(&node, "demo_action_server_node", "", &support);

  // Create action service
  rclc_action_server_t action_server;
  rclc_action_server_init_default(
    &action_server,
    &node,
    &support.clock,
    ROSIDL_GET_ACTION_TYPE_SUPPORT(example_interfaces, Fibonacci),
    "fibonacci"
  );

  // Create executor
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 1, &allocator);

  example_interfaces__action__Fibonacci_SendGoal_Request ros_goal_request[10];

  rclc_executor_add_action_server(
    &executor,
    &action_server,
    10,
    ros_goal_request,
    sizeof(example_interfaces__action__Fibonacci_SendGoal_Request),
    handle_goal,
    handle_cancel,
    (void *) &action_server);

  while (1) {
    pthread_mutex_lock(&rclc_mutex);
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    pthread_mutex_unlock(&rclc_mutex);
    usleep(100000);
  }

  // clean up
  rclc_executor_fini(&executor);
  (void)!rcl_node_fini(&node);

  return 0;
}
