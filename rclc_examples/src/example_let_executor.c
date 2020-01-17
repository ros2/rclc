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
#include <stdio.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>
#include "rclc/let_executor.h"
#include <unistd.h>

// these data structures for the publisher and subscriber are global, so that
// they can be configured in main() and can be used in the corresponding callback.
rcl_publisher_t my_pub;
rcl_publisher_t my_int_pub;

std_msgs__msg__String pub_msg;
std_msgs__msg__String sub_msg;

std_msgs__msg__Int32 pub_int_msg;
int pub_int_value;
std_msgs__msg__Int32 sub_int_msg;
int pub_string_value;

/***************************** CALLBACKS ***********************************/

typedef struct {
  rcl_timer_t * timer1;
  rcl_timer_t * timer2;
} pub_trigger_object_t;

typedef struct {
  rcl_subscription_t * sub1;
  rcl_subscription_t * sub2;
} sub_trigger_object_t;

bool pub_trigger(rclc_executor_handle_t * handles, unsigned int size, void * obj) {
  if (handles == NULL) {
    printf("Error in pub_trigger: 'handles' is a NULL pointer\n");
    return false;
  }
  if (obj == NULL) {
    printf("Error in pub_trigger: 'obj' is a NULL pointer\n");
    return false;
  }
  pub_trigger_object_t * comm_obj = (pub_trigger_object_t *) obj;
  bool timer1 = false;
  bool timer2 = false;
  //printf("pub_trigger ready set: ");
  for(unsigned int i=0; i<size; i++) {
    if (handles[i].data_available == true) {
      void * handle_ptr = rclc_executor_handle_get_ptr(&handles[i]);
      if ( handle_ptr == comm_obj->timer1) {
        timer1 = true;
        //printf("timer1 ");
      }
      if ( handle_ptr == comm_obj->timer2) {
        timer2 = true;
        //printf("timer2 ");
      }

    }
  }
  //printf("\n");
  if ( timer1 || timer2 )
   {
    return true;
  } else {
    return false;
  }
}


bool sub_trigger(rclc_executor_handle_t * handles, unsigned int size, void * obj) {
  if (handles == NULL) {
    printf("Error in sub_trigger: 'handles' is a NULL pointer\n");
    return false;
  }
  if (obj == NULL) {
    printf("Error in sub_trigger: 'obj' is a NULL pointer\n");
    return false;
  }
  sub_trigger_object_t * comm_obj = (sub_trigger_object_t *) obj;
  bool sub1 = false;
  bool sub2 = false;
  //printf("sub_trigger ready set: ");
  for(unsigned int i=0; i<size; i++) {
    if (handles[i].data_available == true) {
      void * handle_ptr = rclc_executor_handle_get_ptr(&handles[i]);

      if ( handle_ptr == comm_obj->sub1) {
        sub1 = true;
        //printf("sub1 ");
      }

      if ( handle_ptr == comm_obj->sub2) {
        sub2 = true;
        //printf("sub2 ");
      }
    }
  }
  //printf("\n");
  if ( sub1 && sub2 )
   {
    return true;
  } else {
    return false;
  }

}

// continue here: new test case where two input data is available
// but not in single spin_some call.
// if timer is not executed  - no data is received 

void my_string_subscriber_callback(const void * msgin)
{
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  if (msg == NULL) {
    printf("my_string_subscriber_callback: msgin is NULL\n");
  } else {
    printf("Callback 1: %s\n", msg->data.data);
  }
}


void my_int_subscriber_callback(const void * msgin)
{
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if (msg == NULL) {
    printf("my_int_subscriber_callback: msgin is NULL\n");
  } else {
    printf("Callback 2: %d\n", msg->data);
  }
}

#define UNUSED(x) (void)x;
void my_timer_string_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  rcl_ret_t rc;
  UNUSED(last_call_time);
  if (timer != NULL) {
    //printf("Timer: time since last call %d\n", (int) last_call_time);

    // create message
    std_msgs__msg__String__init(&pub_msg);
    const unsigned int PUB_MSG_SIZE = 20;
    char pub_string[PUB_MSG_SIZE];
    char num_string[10];
    snprintf(pub_string, 14, "%s", "Hello World! ");
    sprintf(num_string, "%d",pub_string_value++);
    strcat(pub_string, num_string);
    rosidl_generator_c__String__assignn(&pub_msg, pub_string, PUB_MSG_SIZE);

    rc = rcl_publish(&my_pub, &pub_msg, NULL);
    if (rc == RCL_RET_OK) {
      printf("Published: %s\n", pub_msg.data.data);
    } else {
      printf("Error in my_timer_string_callback: publishing message %s\n", pub_msg.data.data);
    }
  } else {
    printf("Error in my_timer_string_callback: timer parameter is NULL\n");
  }
}

void my_timer_int_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  rcl_ret_t rc;
  UNUSED(last_call_time);
  if (timer != NULL) {
    //printf("Timer: time since last call %d\n", (int) last_call_time);
    pub_int_msg.data = pub_int_value++;
    rc = rcl_publish(&my_int_pub, &pub_int_msg, NULL);
    if (rc == RCL_RET_OK) {
      printf("Published: %d\n", pub_int_msg.data);
    } else {
      printf("Error in my_timer_int_callback: publishing message %d\n", pub_int_msg.data);
    }
  } else {
    printf("Error in my_timer_int_callback: timer parameter is NULL\n");
  }
}

/******************** MAIN PROGRAM ****************************************/
int main(int argc, const char * argv[])
{
  rcl_context_t context = rcl_get_zero_initialized_context();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t rc;

  // create init_options
  rc = rcl_init_options_init(&init_options, allocator);
  if (rc != RCL_RET_OK) {
    printf("Error rcl_init_options_init.\n");
    return -1;
  }

  // create context
  rc = rcl_init(argc, argv, &init_options, &context);
  if (rc != RCL_RET_OK) {
    printf("Error in rcl_init.\n");
    return -1;
  }

  // create rcl_node
  rcl_node_t my_node = rcl_get_zero_initialized_node();
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rc = rcl_node_init(&my_node, "node_0", "let_executor_examples", &context, &node_ops);
  if (rc != RCL_RET_OK) {
    printf("Error in rcl_node_init\n");
    return -1;
  }

  // create a publisher to publish topic 'topic_0' with type std_msg::msg::String
  // my_pub is global, so that the timer callback can access this publisher.
  const char * topic_name = "topic_0";
  const rosidl_message_type_support_t * my_type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
  rcl_publisher_options_t pub_options = rcl_publisher_get_default_options();
  rc = rcl_publisher_init(
    &my_pub,
    &my_node,
    my_type_support,
    topic_name,
    &pub_options);
  if (RCL_RET_OK != rc) {
    printf("Error in rcl_publisher_init %s.\n", topic_name);
    return -1;
  }

  // create a timer, which will call the publisher every 'period' ms in the 'my_timer_string_callback'
  rcl_clock_t clock;
  rc = rcl_clock_init(RCL_STEADY_TIME, &clock, &allocator);
  if (rc != RCL_RET_OK) {
    printf("Error in rcl_clock_init.\n");
    return -1;
  }
  rcl_timer_t my_string_timer = rcl_get_zero_initialized_timer();
  const unsigned int timer_timeout = 100;
  rc = rcl_timer_init(
    &my_string_timer,
    &clock,
    &context,
    RCL_MS_TO_NS(timer_timeout),
    my_timer_string_callback,
    allocator);
  if (rc != RCL_RET_OK) {
    printf("Error in rcl_timer_init.\n");
    return -1;
  } else {
    printf("Created timer 'my_string_timer' with timeout %d ms.\n", timer_timeout);
  }

  // create a publisher to publish topic 'topic_1' with type std_msg::msg::Int
  // my_pub_int is global, so that the timer callback can access this publisher.
  const char * topic_name_1 = "topic_1";
  const rosidl_message_type_support_t * my_int_type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
  rcl_publisher_options_t my_int_pub_options = rcl_publisher_get_default_options();
  rc = rcl_publisher_init(
    &my_int_pub,
    &my_node,
    my_int_type_support,
    topic_name_1,
    &my_int_pub_options);
  if (RCL_RET_OK != rc) {
    printf("Error in rcl_publisher_init %s.\n", topic_name_1);
    return -1;
  }

  // create a timer, which will call my_int_pub every 'period' ms in the 'my_timer_string_callback'
  rcl_timer_t my_int_timer = rcl_get_zero_initialized_timer();
  const unsigned int timer_int_timeout = 10 * timer_timeout;
  rc = rcl_timer_init(
    &my_int_timer,
    &clock,
    &context,
    RCL_MS_TO_NS(timer_int_timeout),
    my_timer_int_callback,
    allocator);
  if (rc != RCL_RET_OK) {
    printf("Error in rcl_timer_init.\n");
    return -1;
  } else {
    printf("Created 'my_int_timer' with timeout %d ms.\n", timer_int_timeout);
  }

  // assign message to publisher
  std_msgs__msg__Int32__init(&pub_int_msg);
  pub_int_value = 0;
  pub_string_value = 0;

  // create subscription
  rcl_subscription_t my_string_sub = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t my_subscription_options = rcl_subscription_get_default_options();
  my_subscription_options.qos.depth = 0;
  rc = rcl_subscription_init(
    &my_string_sub,
    &my_node,
    my_type_support,
    topic_name,
    &my_subscription_options);

  if (rc != RCL_RET_OK) {
    printf("Failed to create subscriber %s.\n", topic_name);
    return -1;
  } else {
    printf("Created subscriber %s:\n", topic_name);
  }

  // one string message for subscriber
  std_msgs__msg__String__init(&sub_msg);


  // create int subscription
  rcl_subscription_t my_int_sub = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t my_int_subscription_options = rcl_subscription_get_default_options();
  rc = rcl_subscription_init(
    &my_int_sub,
    &my_node,
    my_int_type_support,
    topic_name_1,
    &my_int_subscription_options);

  if (rc != RCL_RET_OK) {
    printf("Failed to create subscriber %s.\n", topic_name_1);
    return -1;
  } else {
    printf("Created subscriber %s:\n", topic_name_1);
  }

  // one string message for subscriber
  std_msgs__msg__Int32__init(&sub_int_msg);

  ////////////////////////////////////////////////////////////////////////////
  // Configuration of RCL Executor
  ////////////////////////////////////////////////////////////////////////////
  rclc_let_executor_t executor_pub;
  rclc_let_executor_t executor_sub;


  // Executor for publishing messages
  unsigned int num_handles_pub = 2;
  printf("Executor_pub: number of DDS handles: %u\n", num_handles_pub);
  executor_pub = rclc_let_executor_get_zero_initialized_executor();
  rclc_let_executor_init(&executor_pub, &context, num_handles_pub, &allocator);
  unsigned int rcl_wait_timeout = 1000;   // rcl_wait timeout in ms
  rc = rclc_let_executor_set_timeout(&executor_pub, RCL_MS_TO_NS(rcl_wait_timeout));
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_let_executor_set_timeout.");
  }
  rc = rclc_let_executor_add_timer(&executor_pub, &my_string_timer);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_let_executor_add_timer 'my_string_timer'.\n");
  }

  rc = rclc_let_executor_add_timer(&executor_pub, &my_int_timer);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_let_executor_add_timer 'my_int_timer'.\n");
  }

  // Executor for subscribing messages
  unsigned int num_handles_sub = 2;
  printf("Executor_sub: number of DDS handles: %u\n", num_handles_sub);
  executor_sub = rclc_let_executor_get_zero_initialized_executor();
  rclc_let_executor_init(&executor_sub, &context, num_handles_sub, &allocator);
  rc = rclc_let_executor_set_timeout(&executor_sub, RCL_MS_TO_NS(rcl_wait_timeout));

  // add subscription to executor
  rc = rclc_let_executor_add_subscription(&executor_sub, &my_string_sub, &sub_msg, &my_string_subscriber_callback,
      ON_NEW_DATA);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_let_executor_add_subscription 'my_string_sub'. \n");
  }

  // add int subscription to executor
  rc = rclc_let_executor_add_subscription(&executor_sub, &my_int_sub, &sub_int_msg, &my_int_subscriber_callback,
      ON_NEW_DATA);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_let_executor_add_subscription 'my_int_sub'. \n");
  }

  pub_trigger_object_t comm_obj_pub;
  comm_obj_pub.timer1 = &my_string_timer;
  comm_obj_pub.timer2 = &my_int_timer;

  sub_trigger_object_t comm_obj_sub;
  comm_obj_sub.sub1 = &my_string_sub;
  comm_obj_sub.sub2 = &my_int_sub;

  //rc = rclc_let_executor_set_trigger(&executor_pub, pub_trigger, &comm_obj_pub);
  //rc = rclc_let_executor_set_trigger(&executor_sub, sub_trigger, &comm_obj_sub);
  rc = rclc_let_executor_set_trigger(&executor_pub, rclc_let_executor_trigger_any, NULL);
  rc = rclc_let_executor_set_trigger(&executor_sub, rclc_let_executor_trigger_all,NULL);

  for (unsigned int i = 0; i < 100; i++) {
    // timeout specified in ns                 (here: 1s)
    rclc_let_executor_spin_some(&executor_pub, 1000 * (1000 * 1000));
    usleep(1000); // 1ms
    rclc_let_executor_spin_some(&executor_sub, 1000 * (1000 * 1000));
  }


// example with two executors
// with one - publishing and subscribing i cannot 
// differentiate the case in which 
// two topics are received and I wait for the next round of spin_some
// because the publisher is always ready

// setup executor_1 
// publishes my_pub (1s rate) and my_int_pub (2s rate)
// trigger function ANY

// setup executor_2
// subscribes to my_string_sub and my_int_sub
// (1) trigger function AND 
// (2) trigger function OR
// expected output:
//   (1) see output at 2s rate and same number of callback calls
//   (2) see output at 1s rate and my_string_sub twice as many as my_sub_int
//       messages

  // clean up
  rc = rclc_let_executor_fini(&executor_pub);
  rc = rclc_let_executor_fini(&executor_sub);
  rc += rcl_publisher_fini(&my_pub, &my_node);
  rc += rcl_timer_fini(&my_string_timer);
  rc += rcl_publisher_fini(&my_int_pub, &my_node);
  rc += rcl_timer_fini(&my_int_timer);
  rc += rcl_subscription_fini(&my_string_sub, &my_node);
  rc += rcl_subscription_fini(&my_int_sub, &my_node);
  rc += rcl_node_fini(&my_node);
  rc += rcl_init_options_fini(&init_options);

  if (rc != RCL_RET_OK) {
    printf("Error while cleaning up!\n");
    return -1;
  }
  return 0;
}
