// Copyright (c) 2020 - for information on the respective copyright owner
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
#include <std_msgs/msg/int32.h>
#include "rclc/rclc.h"
#include "rclc/executor.h"
#include "rclc_dispatcher_executor/dispatcher_executor.h"

// these data structures for the publisher and subscriber are global, so that
// they can be configured in main() and can be used in the corresponding callback.
rcl_publisher_t pub_a;
rcl_publisher_t pub_b;

std_msgs__msg__Int32 pub_msg_a;
std_msgs__msg__Int32 pub_msg_b;

std_msgs__msg__Int32 sub_msg_a;
std_msgs__msg__Int32 sub_msg_b;

/***************************** CALLBACKS ***********************************/

void callback_a(const void * msgin)
{
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if (msg == NULL) {
    printf("Callback: msg NULL\n");
  } else {
    printf("Callback A: I heard: %d\n", msg->data);
  }
}

void callback_b(const void * msgin)
{
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if (msg == NULL) {
    printf("Callback: msg NULL\n");
  } else {
    printf("Callback B: I heard: %d\n", msg->data);
  }
}


void timer_callback_a(rcl_timer_t * timer, int64_t last_call_time)
{
  rcl_ret_t rc;
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    pub_msg_a.data++;
    rc = rcl_publish(&pub_a, &pub_msg_a, NULL);
    if (rc == RCL_RET_OK) {
      printf("Published message %d\n", pub_msg_a.data);
    } else {
      printf("timer_callback: Error publishing message %d\n", pub_msg_a.data);
    }
  } else {
    printf("timer_callback Error: timer parameter is NULL\n");
  }
}

void timer_callback_b(rcl_timer_t * timer, int64_t last_call_time)
{
  rcl_ret_t rc;
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    pub_msg_b.data++;
    rc = rcl_publish(&pub_b, &pub_msg_b, NULL);
    if (rc == RCL_RET_OK) {
      printf("Published message %d\n", pub_msg_b.data);
    } else {
      printf("timer_callback: Error publishing message %d\n", pub_msg_b.data);
    }
  } else {
    printf("timer_callback Error: timer parameter is NULL\n");
  }
}


/******************** MAIN PROGRAM ****************************************/
int main(int argc, const char * argv[])
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rcl_ret_t rc;

  // create init_options
  rc = rclc_support_init(&support, argc, argv, &allocator);
  if (rc != RCL_RET_OK) {
    printf("Error rclc_support_init.\n");
    return -1;
  }

  // create rcl_node
  rcl_node_t my_node = rcl_get_zero_initialized_node();
  rc = rclc_node_init_default(&my_node, "dispatcher_executor", "", &support);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_node_init_default\n");
    return -1;
  }

  const char * topic_name_a = "topic_a";
  const char * topic_name_b = "topic_b";
  const rosidl_message_type_support_t * my_type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

  rc = rclc_publisher_init_default(
    &pub_a,
    &my_node,
    my_type_support,
    topic_name_a);
  if (RCL_RET_OK != rc) {
    printf("Error in rclc_publisher_init_default %s.\n", topic_name_a);
    return -1;
  }

  rc = rclc_publisher_init_default(
    &pub_b,
    &my_node,
    my_type_support,
    topic_name_b);
  if (RCL_RET_OK != rc) {
    printf("Error in rclc_publisher_init_default %s.\n", topic_name_b);
    return -1;
  }

  // publisher will be called with period=`timer_timeout` ms in timer callback
  rcl_timer_t timer_a = rcl_get_zero_initialized_timer();
  const unsigned int timer_timeout_a = 1000;
  rc = rclc_timer_init_default(
    &timer_a,
    &support,
    RCL_MS_TO_NS(timer_timeout_a),
    timer_callback_a);
  if (rc != RCL_RET_OK) {
    printf("Error in rcl_timer_init_default.\n");
    return -1;
  } else {
    printf("Created timer_a with timeout %d ms.\n", timer_timeout_a);
  }

  // publisher will be called with period=`timer_timeout` ms in timer callback
  rcl_timer_t timer_b = rcl_get_zero_initialized_timer();
  const unsigned int timer_timeout_b = 1000;
  rc = rclc_timer_init_default(
    &timer_b,
    &support,
    RCL_MS_TO_NS(timer_timeout_b),
    timer_callback_b);
  if (rc != RCL_RET_OK) {
    printf("Error in rcl_timer_init_default.\n");
    return -1;
  } else {
    printf("Created timer_b with timeout %d ms.\n", timer_timeout_b);
  }

  // initialize publisher message
  std_msgs__msg__Int32__init(&pub_msg_a);
  std_msgs__msg__Int32__init(&pub_msg_b);
  pub_msg_a.data=0;
  pub_msg_b.data=0;

  // create subscription
  rcl_subscription_t sub_a = rcl_get_zero_initialized_subscription();
  rc = rclc_subscription_init_default(
    &sub_a,
    &my_node,
    my_type_support,
    topic_name_a);
  if (rc != RCL_RET_OK) {
    printf("Failed to create subscriber %s.\n", topic_name_a);
    return -1;
  } else {
    printf("Created subscriber %s:\n", topic_name_a);
  }
  
  rcl_subscription_t sub_b = rcl_get_zero_initialized_subscription();
  rc = rclc_subscription_init_default(
    &sub_b,
    &my_node,
    my_type_support,
    topic_name_b);
  if (rc != RCL_RET_OK) {
    printf("Failed to create subscriber %s.\n", topic_name_b);
    return -1;
  } else {
    printf("Created subscriber %s:\n", topic_name_b);
  }

  // initialize subscription message
  std_msgs__msg__Int32__init(&sub_msg_a);
  std_msgs__msg__Int32__init(&sub_msg_b);

  ////////////////////////////////////////////////////////////////////////////
  // Configuration of Dispatcher Executor
  ////////////////////////////////////////////////////////////////////////////
  rclc_executor_t executor;
  rclc_executor_t publishing_executor;
  executor = rclc_executor_get_zero_initialized_executor();
  publishing_executor = rclc_executor_get_zero_initialized_executor();
  // total number of handles = #subscriptions + #timers
  unsigned int num_handles = 1 + 1;
  printf("Debug: number of DDS handles: %u\n", num_handles);
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  unsigned int num_handles_pub = 1 + 1;
  rclc_executor_init(&publishing_executor, &support.context, num_handles_pub, &allocator);

  rclc_dispatcher_executor_init(&executor);

  rclc_executor_sched_parameter_t sched_p1;
  rclc_executor_sched_parameter_t sched_p2;
  sched_p1.policy = SCHED_FIFO;
  sched_p1.param.sched_priority = 10;
  sched_p2.policy = SCHED_FIFO;
  sched_p2.param.sched_priority = 20;

  rc = rclc_dispatcher_executor_add_subscription(
    &executor, &sub_a, &sub_msg_a, &callback_a,
    ON_NEW_DATA, &sched_p1);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_dispatcher_executor_add_subscription. \n");
  }

  rc = rclc_dispatcher_executor_add_subscription(
    &executor, &sub_b, &sub_msg_b, &callback_b,
    ON_NEW_DATA, &sched_p2);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_dispatcher_executor_add_subscription. \n");
  }

  rclc_executor_add_timer(&publishing_executor, &timer_a);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_executor_add_timer.\n");
  }
  rclc_executor_add_timer(&publishing_executor, &timer_b);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_executor_add_timer.\n");
  }


  for (unsigned int i = 0; i < 10; i++) {
    // timeout in nanoseconds (here: 1s)
    rclc_executor_spin_some(&publishing_executor, 1000 * (1000 * 1000));
    
  }

  // TODO(JanStaschulat) replace with loop of spin_some()
  rclc_dispatcher_executor_spin(&executor);

  // clean up
  rc = rclc_executor_fini(&executor);
  rc = rclc_executor_fini(&publishing_executor);
  rc += rcl_publisher_fini(&pub_a, &my_node);
  rc += rcl_publisher_fini(&pub_b, &my_node);
  rc += rcl_timer_fini(&timer_a);
  rc += rcl_timer_fini(&timer_b);
  rc += rcl_subscription_fini(&sub_a, &my_node);
  rc += rcl_subscription_fini(&sub_b, &my_node);
  rc += rcl_node_fini(&my_node);
  rc += rclc_support_fini(&support);

  if (rc != RCL_RET_OK) {
    printf("Error while cleaning up!\n");
    return -1;
  }
  return 0;
}
