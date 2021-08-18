// Copyright (c) 2020 - for information on the respective copyright owner
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
#include <std_msgs/msg/int32.h>
#include <example_interfaces/srv/add_two_ints.h>
#include <gtest/gtest.h>

#include <chrono>
#include <thread>
#include <vector>

#include "rclc/executor.h"
#include "rclc/multi_threaded_executor.h"
#include "osrf_testing_tools_cpp/scope_exit.hpp"
#include "rcutils/time.h"

// Include backport of function 'rcl_wait_set_is_valid' introduced in Foxy
// in case of building for Dashing and Eloquent. This pre-processor macro
// is defined in CMakeLists.txt.
#if defined (USE_RCL_WAIT_SET_IS_VALID_BACKPORT)
#include "rclc/rcl_wait_set_is_valid_backport.h"
#endif

// timeout for rcl_wait() when calling spin_some API of executor
const uint64_t rclc_test_timeout_ns = 1000000000;  // 1s

// sleep time beween publish and receive in DDS middleware
#define RCLC_UNIT_TEST_SLEEP_TIME_MS 100
const std::chrono::milliseconds rclc_test_sleep_time =
  std::chrono::milliseconds(RCLC_UNIT_TEST_SLEEP_TIME_MS);

#define TC_SPIN_SOME_NUM_PUBLISHER 3
#define TC_SPIN_SOME_PUBLISHED_MSGS 3
#define TC_SPIN_SOME_MAX_MSGS \
  (TC_SPIN_SOME_NUM_PUBLISHER * TC_SPIN_SOME_PUBLISHED_MSGS)
static unsigned int _executor_results[TC_SPIN_SOME_MAX_MSGS];
static unsigned int _executor_results_i;

// callback for subscription 1
static unsigned int _cb1_cnt = 0;
static unsigned int _cb1_int_value = 0;
// callback for subscription 2
static unsigned int _cb2_cnt = 0;
static unsigned int _cb2_int_value = 0;
// callback for subscription 3
static unsigned int _cb3_cnt = 0;
static unsigned int _cb3_int_value = 0;

static
void
_results_callback_counters_init()
{
  _cb1_cnt = 0;
  _cb2_cnt = 0;
  _cb3_cnt = 0;
}

static
void
_results_callback_values_init()
{
  _cb1_int_value = 0;
  _cb2_int_value = 0;
  _cb3_int_value = 0;
}

static
void
_results_callback_init()
{
  _results_callback_counters_init();
  _results_callback_values_init();
}

static
void
_executor_results_init(void)
{
  for (unsigned int i = 0; i < TC_SPIN_SOME_MAX_MSGS; i++) {
    _executor_results[i] = 0;
  }
  _executor_results_i = 0;

  _results_callback_init();
}

/// preserves the order of received data
/// message values are stored in an array (left to right)
/// after the message value is stored, the array index (_executor_results_i)
/// is incremented.
/// assumption msg_id > 0
static
void
_executor_results_add(unsigned int msg_id)
{
  if (_executor_results_i < TC_SPIN_SOME_MAX_MSGS) {
    _executor_results[_executor_results_i] = msg_id;
    _executor_results_i++;
  } else {
    printf("_executor_results_add: buffer overflow!\n");
  }
}

static
void
_executor_results_print(void)
{
  printf("Results: ");
  for (unsigned int i = 0; i < TC_SPIN_SOME_MAX_MSGS; i++) {
    // if the value is zero, then it was not used => finished
    // assume positive message_id's
    if (_executor_results[i] > 0) {
      printf("%d ", _executor_results[i]);
    } else {
      break;
    }
  }
  printf("\n");
}

// definition of callbacks
#define INT_CALLBACK_DEFINITION(NUM) \
  static void int32_callback_ ## NUM(const void * msgin) \
  { \
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin; \
    if (msg == NULL) { \
      printf("Test CB " #NUM ": msg is NULL\n"); \
    } else { \
      _cb ## NUM ## _int_value = msg->data; \
    } \
    _cb ## NUM ## _cnt++; \
    _executor_results_add(NUM); \
  }

// definition of function pointer
#define INT_CALLBACK(NUM) \
  int32_callback_ ## NUM


// macro-definition               definition of callback function
INT_CALLBACK_DEFINITION(1)  // => int32_callback_1
INT_CALLBACK_DEFINITION(2)  // => int32_callback_2

#define CALLBACK_1 INT_CALLBACK(1)
#define CALLBACK_2 INT_CALLBACK(2)

#define CREATE_PUBLISHER(PUB, TOPIC_NAME) \
  this->PUB = rcl_get_zero_initialized_publisher(); \
  this->PUB ## _topic_name = #TOPIC_NAME; \
  this->PUB ## _type_support = \
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32); \
  this->PUB ## _options = rcl_publisher_get_default_options(); \
  ret = rcl_publisher_init( \
    &this->PUB, &this->node, this->PUB ## _type_support, \
    this->PUB ## _topic_name, &this->PUB ## _options); \
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str; \
  rcl_reset_error();

#define CREATE_SUBSCRIPTION(SUB, TOPIC_NAME) \
  this->SUB = rcl_get_zero_initialized_subscription(); \
  this->SUB ## _topic_name = #TOPIC_NAME; \
  this->SUB ## _type_support = \
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32); \
  this->SUB ## _options = rcl_subscription_get_default_options(); \
  ret = rcl_subscription_init( \
    &this->SUB, &this->node, this->SUB ## _type_support, \
    this->SUB ## _topic_name, &this->SUB ## _options); \
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str; \
  EXPECT_TRUE(rcl_subscription_is_valid(&this->SUB)); \
  rcl_reset_error();

class TestMultiThreadedExecutor : public ::testing::Test
{
public:
  rcl_context_t context;
  rcl_node_t node;

  // integer publisher 1
  rcl_publisher_t pub1;
  const char * pub1_topic_name;
  const rosidl_message_type_support_t * pub1_type_support;
  rcl_publisher_options_t pub1_options;
  std_msgs__msg__Int32 pub1_msg;

  // integer publisher 2
  rcl_publisher_t pub2;
  const char * pub2_topic_name;
  const rosidl_message_type_support_t * pub2_type_support;
  rcl_publisher_options_t pub2_options;
  std_msgs__msg__Int32 pub2_msg;

  // integer subscription 1
  rcl_subscription_t sub1;
  const char * sub1_topic_name;
  const rosidl_message_type_support_t * sub1_type_support;
  rcl_subscription_options_t sub1_options;
  std_msgs__msg__Int32 sub1_msg;

  // integer subscription 2
  rcl_subscription_t sub2;
  const char * sub2_topic_name;
  const rosidl_message_type_support_t * sub2_type_support;
  rcl_subscription_options_t sub2_options;
  std_msgs__msg__Int32 sub2_msg;

  // rcl allocator
  const rcl_allocator_t * allocator_ptr;

  void SetUp()
  {
    rcl_ret_t ret;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    ret = rcl_init_options_init(&init_options, rcl_get_default_allocator());
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
    {
      EXPECT_EQ(RCL_RET_OK, rcl_init_options_fini(&init_options)) << rcl_get_error_string().str;
    });
    this->context = rcl_get_zero_initialized_context();
    ret = rcl_init(0, nullptr, &init_options, &this->context);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

    this->node = rcl_get_zero_initialized_node();
    const char * name = "executor_test";
    rcl_node_options_t node_options = rcl_node_get_default_options();
    ret = rcl_node_init(&this->node, name, "", &this->context, &node_options);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    const rcl_node_options_t * node_ops = rcl_node_get_options(&this->node);
    this->allocator_ptr = &node_ops->allocator;

    // create publishers - correspond to member variables pub1, pub2, pub3
    // topic name is taken literally (must match with subscription)
    CREATE_PUBLISHER(pub1, data1_int)
    CREATE_PUBLISHER(pub2, data2_int)
    std_msgs__msg__Int32__init(&this->pub1_msg);
    std_msgs__msg__Int32__init(&this->pub2_msg);

    // create subscriptions - correspond to member variables sub1, sub2, sub3
    // topic name is taken literally (must match with publisher)
    CREATE_SUBSCRIPTION(sub1, data1_int)
    CREATE_SUBSCRIPTION(sub2, data2_int)
    std_msgs__msg__Int32__init(&this->sub1_msg);
    std_msgs__msg__Int32__init(&this->sub2_msg);
  }

  void TearDown()
  {
    rcl_ret_t ret;
    ret = rcl_subscription_fini(&this->sub1, &this->node);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ret = rcl_subscription_fini(&this->sub2, &this->node);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

    ret = rcl_publisher_fini(&this->pub1, &this->node);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ret = rcl_publisher_fini(&this->pub2, &this->node);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

    ret = rcl_node_fini(&this->node);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ret = rcl_shutdown(&this->context);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ret = rcl_context_fini(&this->context);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    std_msgs__msg__Int32__fini(&this->pub1_msg);
    std_msgs__msg__Int32__fini(&this->pub2_msg);

    std_msgs__msg__Int32__fini(&this->sub1_msg);
    std_msgs__msg__Int32__fini(&this->sub2_msg);
  }
};

TEST_F(TestMultiThreadedExecutor, base_line) {
  rcl_ret_t rc;
  rclc_executor_t executor;

  // initialize executor with 2 subscriptions
  int num_handles = 2;
  num_handles++;  // for one guard condition => move to executor itself!
  rc = rclc_executor_init(&executor, &this->context, num_handles, this->allocator_ptr);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcl_reset_error();


  rclc_executor_sched_parameter_t sched_p1;
  rclc_executor_sched_parameter_t sched_p2;
  sched_p1.policy = SCHED_FIFO;
  sched_p1.param.sched_priority = 10;
  sched_p1.policy = SCHED_FIFO;
  sched_p1.param.sched_priority = 20;


  // add subscriptions to executor
  rc = rclc_executor_add_subscription_multi_threaded(
    &executor, &this->sub1, &this->sub1_msg,
    &int32_callback_1, ON_NEW_DATA, &sched_p1);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcutils_reset_error();
  rc = rclc_executor_add_subscription_multi_threaded(
    &executor, &this->sub2, &this->sub2_msg,
    &int32_callback_2, ON_NEW_DATA, &sched_p2);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcutils_reset_error();

  // ------------------------- test case setup ------------------------
  // sending two messages - expecting to receive two messages
  rclc_executor_set_semantics(&executor, RCLCPP_EXECUTOR);
  _executor_results_init();
  _results_callback_init();
  this->pub1_msg.data = 991;
  this->pub2_msg.data = 772;

  rc = rcl_publish(&this->pub1, &this->pub1_msg, nullptr);
  EXPECT_EQ(RCL_RET_OK, rc) << " pub1 did not publish!";

  rc = rcl_publish(&this->pub2, &this->pub2_msg, nullptr);
  EXPECT_EQ(RCL_RET_OK, rc) << " pub1 did not publish!";

  std::this_thread::sleep_for(rclc_test_sleep_time);
  EXPECT_EQ(_cb1_cnt, (unsigned int) 0);
  EXPECT_EQ(_cb2_cnt, (unsigned int) 0);

  // define a function that is equivalent to spin_some
  // rclc_executor_spin_multi_threaded(&executor);

  // test result
  /*
  EXPECT_EQ(_cb1_cnt, (unsigned int) 1);
  EXPECT_EQ(_cb2_cnt, (unsigned int) 1);
  EXPECT_EQ(_cb1_int_value, (unsigned int) 991);
  EXPECT_EQ(_cb2_int_value, (unsigned int) 772);
  _executor_results_print();
  */
  RCLC_UNUSED(_executor_results_print);
  // clean
  rc = rclc_executor_fini(&executor);
}
