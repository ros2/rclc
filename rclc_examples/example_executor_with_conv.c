// Copyright (c) 2018 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/micro-ROS/rcl_executor.
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

#include "rcl_executor/let_executor.h"
#include "rcl_ext/rcl_ext.h"
#include <std_msgs/msg/string.h>
// #include <unistd.h> // for usleep

// global data structures
rcl_publisher_t * pub;
std_msgs__msg__String pub_msg;    // string message for the publisher
std_msgs__msg__String * sub_msg;  // string message for the subscriber

/***************************** CALLBACKS ***********************************/


void subscriber_callback(const void * msgin)
{
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  if (msg == NULL) {
    printf("Callback: msg NULL\n");
  } else {
    printf("Callback: I heard: %s\n", msg->data.data);
  }
  // usleep(500000);   // sleep for 500ms
}

#define UNUSED(x) (void)x;


void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  rcl_ret_t rc;
  UNUSED(last_call_time);
  if (timer != NULL) {
    //printf("Timer: time since last call %d\n", (int) last_call_time);
    rc = rcl_publish(pub, &pub_msg, NULL);
    if (rc == RCL_RET_OK) {
      printf("Published message %s\n", pub_msg.data.data);
    } else {
      printf("timer_callback: Error publishing message %s\n", pub_msg.data.data);
    }
  } else {
    printf("timer_callback Error: timer parameter is NULL\n");
  }
}

/******************** MAIN PROGRAM *****************************************/
int main(int argc, const char * argv[])
{
  rcl_ret_t rc;

  ////////////////////////////////////////////////////////////////////////////
  // Initialize RCL
  ////////////////////////////////////////////////////////////////////////////
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ext_init_t init_obj;
  rc = rcl_ext_init(&init_obj, argc, argv, &allocator);
  if (rc != RCL_RET_OK) {
    printf("Error in rcl_ext_init.\n");
    return -1;
  }

  ////////////////////////////////////////////////////////////////////////////
  // Configure one node, one publisher, one timer, one subscriber
  ////////////////////////////////////////////////////////////////////////////

  rcl_node_t * rcl_node;
  rcl_subscription_t * sub;
  // publisher 'pub' is global, because it is used in the timer_callback
  rcl_timer_t * timer;

  // create a node 'node_0'
  rcl_node = rcl_ext_create_node("node_0", "", &init_obj);
  if (rcl_node == NULL) {
    printf("Error in rcl_ext_create_node.\n");
    return -1;
  }

  // create a publisher to publish topic 'topic_name' of type std_msg::msg::String
  const char * topic_name = "topic_0";
  pub = rcl_ext_create_publisher(rcl_node,
      init_obj.allocator, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      topic_name);
  if (pub == NULL) {
    printf("Error: Could not create publisher %s.\n", topic_name);
    return -1;
  }

  // message string the publisher
  std_msgs__msg__String__init(&pub_msg);
  const unsigned int msg_size = 10;
  char * pub_string = malloc(msg_size);
  for (unsigned int i = 0; i < msg_size; i++) {
    pub_string[i] = 'a';
  }
  rosidl_generator_c__String__assignn(&pub_msg, pub_string, msg_size);

  // create a timer, which will call the publisher every 'period' ms in the 'timer_callback'
  const unsigned int period = 500;   // in ms
  timer = rcl_ext_create_timer(&init_obj,
      RCL_MS_TO_NS(period), timer_callback);
  if (timer == NULL) {
    printf("Error: Could not create timer.\n");
    return -1;
  }
  // create subscription with topic 'topic_name' of type std_msg::msg::String
  sub = rcl_ext_create_subscription(rcl_node, init_obj.allocator,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic_name);
  if (sub == NULL) {
    printf("Error: Could not create subscriber %s.\n", topic_name);
    return -1;
  }

  // one string message for subscriber
  sub_msg = init_obj.allocator->allocate(
    sizeof(std_msgs__msg__String), init_obj.allocator->state);
  std_msgs__msg__String__init(sub_msg);

  ////////////////////////////////////////////////////////////////////////////
  // Configuration of RCL Executor
  ////////////////////////////////////////////////////////////////////////////
  rcle_let_executor_t exe;

  //compute total number of subsribers and timers
  unsigned int num_handles = 1 + 1;

  printf("Debug: number of DDS handles: %u\n", num_handles);
  exe = rcle_let_executor_get_zero_initialized_executor();
  rcle_let_executor_init(&exe, &init_obj.context, num_handles, init_obj.allocator);

  // set timeout for rcl_wait()
  unsigned int rcl_wait_timeout = 2000;   // in ms
  rc = rcle_let_executor_set_timeout(&exe, RCL_MS_TO_NS(rcl_wait_timeout));
  if (rc != RCL_RET_OK) {
    printf("Error in rcle_let_executor_set_timeout.");
  }

  // add subscription to executor
  rc = rcle_let_executor_add_subscription(&exe, sub, sub_msg, &subscriber_callback,
      ON_NEW_DATA);
  if (rc != RCL_RET_OK) {
    printf("Error in rcle_let_executor_add_subscription. \n");
  }

  // add the timer to executor
  if (timer != NULL) {
    rc = rcle_let_executor_add_timer(&exe, timer);
    if (rc != RCL_RET_OK) {
      printf("Error rcle_let_executor_add_timer: Could not add timer to executor\n");
    }
  }


  // spin forever
  //rcle_let_executor_spin(&exe);

  for (unsigned int i = 0; i < 10; i++) {
    // timeout in ns
    rcle_let_executor_spin_some(&exe, 2000 * (1000 * 1000));
  }

  // clean up
  rc = rcle_let_executor_fini(&exe);
  rc = rcl_ext_publisher_fini(&init_obj, pub, rcl_node);
  rc = rcl_ext_timer_fini(&init_obj, timer);
  rc = rcl_ext_subscription_fini(&init_obj, sub, rcl_node);
  init_obj.allocator->deallocate(sub_msg, init_obj.allocator->state);
  free(pub_string);
  rc = rcl_ext_node_fini(&init_obj, rcl_node);
  rc = rcl_ext_init_fini(&init_obj);

  if (rc != RCL_RET_OK) {
    printf("Error while de-allocating memory!\n");
    return -1;
  }

  return 0;
}
