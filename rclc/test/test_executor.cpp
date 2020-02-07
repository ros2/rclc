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
#include <std_msgs/msg/int32.h>
#include <gtest/gtest.h>

#include <chrono>
#include <thread>
#include <vector>

#include "rclc/executor.h"
#include "osrf_testing_tools_cpp/scope_exit.hpp"
#include "rcutils/time.h"

// 27.06.2019, unit test adapted from ros2/rcl/rcl_lifecycle/test/test_default_state_machine.cpp
// by Jan Staschulat, under Apache 2.0 License

/******************************* CALLBACKS for subscriptions ************************/

// some variables for testing the LET-semantics
// array holds the msg_id in the order as they are received
// the callback calls the function _results_add(msg_id)
// at the end the array contains the order of received events.
static const unsigned int kMax = 3;
static const unsigned int MSG_MAX = 3 * kMax;  // (#publishers * #published messages)
static unsigned int _executor_results[MSG_MAX];
static unsigned int _executor_results_i;

// callback for topic "chatter1"
static unsigned int _cb1_cnt = 0;
static unsigned int _cb1_int_value = 0;
// callback for topic "chatter2"
static unsigned int _cb2_cnt = 0;
static unsigned int _cb2_int_value = 0;
// callback for topic "chatter3"
static unsigned int _cb3_cnt = 0;
static unsigned int _cb3_int_value = 0;
// callback for testing data communication semantics
static unsigned int _cb5_cnt = 0;
static unsigned int _cb5_int_value = 0;
rcl_publisher_t * _pub_int_ptr;
std_msgs__msg__Int32 * _pub_int_msg_ptr;
// test data for trigger conditions
static unsigned int _cb6_int_value = 0;
static unsigned int _cb7_int_value = 0;
static unsigned int _cb8_int_value = 0;
static unsigned int _cb6_cnt = 0;
static unsigned int _cb7_cnt = 0;
static unsigned int _cb8_cnt = 0;

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
_reset_result_values_for_trigger_test_case()
{
  _cb6_int_value = 0;
  _cb7_int_value = 0;
  _cb8_int_value = 0;
  _cb6_cnt = 0;
  _cb7_cnt = 0;
  _cb8_cnt = 0;
}

static
unsigned int
_results_callback_num_received()
{
  return _cb1_cnt + _cb2_cnt + _cb3_cnt;
}

static
void
_executor_results_init(void)
{
  for (unsigned int i = 0; i < MSG_MAX; i++) {
    _executor_results[i] = 0;
  }
  _executor_results_i = 0;

  _results_callback_counters_init();
}

/// preserves the order of received data
/// message values are stored in an array (left to right)
/// after the message value is stored, the array index (_executor_results_i)
/// is incremented.
/// assumption msg_id > 0
void
_executor_results_add(unsigned int msg_id)
{
  if (_executor_results_i < MSG_MAX) {
    _executor_results[_executor_results_i] = msg_id;
    _executor_results_i++;
  } else {
    printf("_executor_results_add: buffer overflow!\n");
  }
}

bool
_executor_results_all_msg_received()
{
  // total number of expected messages is MSG_MAX
  return _results_callback_num_received() == MSG_MAX;
}
void
_executor_results_print(void)
{
  printf("Results: ");
  for (unsigned int i = 0; i < MSG_MAX; i++) {
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

void
_executor_array_print(unsigned int * array, unsigned int max)
{
  printf("Results: ");
  for (unsigned int i = 0; i < max; i++) {
    // if the value is zero, then it was not used => finished
    // assume positive message_id's
    if (array[i] > 0) {
      printf("%d ", array[i]);
    } else {
      break;
    }
  }
  printf("\n");
}

bool
_executor_results_compare(unsigned int * array)
{
  // assume that array has same size as MSG_MAX
  for (unsigned int i = 0; i < MSG_MAX; i++) {
    if (_executor_results[i] != array[i]) {
      return false;
    }
  }
  return true;
}

// definition of callbacks
#define INT_CALLBACK_DEFINITION(NUM) \
  void int32_callback_ ## NUM(const void * msgin) \
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
INT_CALLBACK_DEFINITION(3)  // => int32_callback_3

#define CALLBACK_1 INT_CALLBACK(1)
#define CALLBACK_2 INT_CALLBACK(2)
#define CALLBACK_3 INT_CALLBACK(3)

// for test case semantics
void int32_callback4(const void * msgin)
{
  rcl_ret_t rc;
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if (msg == NULL) {
    printf("Test CB: msg NULL\n");
  } else {
    _pub_int_msg_ptr->data = 2;
    rc = rcl_publish(_pub_int_ptr, _pub_int_msg_ptr, NULL);
    if (rc != RCL_RET_OK) {
      printf("Error in int32_callback4: could not publish!\n");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

// for test case semantics
void int32_callback5(const void * msgin)
{
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if (msg == NULL) {
    printf("(int32_callback5): msg is NULL\n");
  } else {
    // printf("cb5 msg: %d\n", msg->data);
    _cb5_int_value = msg->data;
  }
}

// test case: trigger conditions
void int32_callback6(const void * msgin)
{
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if (msg == NULL) {
    printf("(int32_callback6): msg is NULL\n");
  } else {
    // printf("cb6 msg: %d\n", msg->data);
    _cb6_int_value = msg->data;
  }
  _cb6_cnt++;
}

// test case: trigger conditions
void int32_callback7(const void * msgin)
{
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if (msg == NULL) {
    printf("(int32_callback7): msg is NULL\n");
  } else {
    // printf("cb7 msg: %d\n", msg->data);
    _cb7_int_value = msg->data;
  }
  _cb7_cnt++;
}

// test case: trigger conditions
void int32_callback8(const void * msgin)
{
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if (msg == NULL) {
    printf("(int32_callback8): msg is NULL\n");
  } else {
    // printf("cb8 msg: %d\n", msg->data);
    _cb8_int_value = msg->data;
  }
  _cb8_cnt++;
}

// callback for unit test 'spin_period'
static const unsigned int MAX_SPIN_PERIOD_INVOCATIONS = 100;
static rcutils_duration_value_t callback_invocation_timepoints[MAX_SPIN_PERIOD_INVOCATIONS];
static unsigned int invocation_count = 0;  // array index

/*
void spin_period_callback(const void * msgin)
{
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  rcutils_time_point_value_t now;
  rcl_ret_t rc;
  RCL_UNUSED(msg);

  rc = rcutils_system_time_now(&now);
  if (rc != RCL_RET_OK) {
    PRINT_RCLC_ERROR(spin_period_callback, rcutils_system_time_now);
  }
  if (invocation_count < MAX_SPIN_PERIOD_INVOCATIONS) {
    callback_invocation_timepoints[invocation_count] = now;
  } else {
    printf("Error: spin_period_callback: Too many calls to the callback.\n");
  }
  invocation_count++;
}
*/
// returns average time in nanoseconds
uint64_t test_case_evaluate_spin_period()
{
  uint64_t sum;
  sum = 0;
  // i starts from 1 because the the first measurement starts in 1st iteration.
  for (unsigned int i = 1; i < MAX_SPIN_PERIOD_INVOCATIONS; i++) {
    sum += callback_invocation_timepoints[i] - callback_invocation_timepoints[i - 1];
    // overflow => use micro-seconds (divide by 1000)
  }
  return sum / (MAX_SPIN_PERIOD_INVOCATIONS - 1);
}

// timer callback
void my_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCL_UNUSED(last_call_time);
  // Do timer work...
  // Optionally reconfigure, cancel, or reset the timer...
  if (timer != NULL) {
    // printf("Timer: time since last call %d\n", static_cast<int>(last_call_time));
  }
}

#define CREATE_PUBLISHER(PUB, TOPIC_NAME) \
  this->PUB = rcl_get_zero_initialized_publisher(); \
  this->PUB ## _topic_name = #TOPIC_NAME; \
  this->PUB ## _type_support = \
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32); \
  this->PUB ## _options = rcl_publisher_get_default_options(); \
  ret = rcl_publisher_init(&this->PUB, &this->node, this->PUB ## _type_support, \
      this->PUB ## _topic_name, &this->PUB ## _options); \
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str; \
  rcl_reset_error();

#define CREATE_SUBSCRIPTION(SUB, TOPIC_NAME) \
  this->SUB = rcl_get_zero_initialized_subscription(); \
  this->SUB ## _topic_name = #TOPIC_NAME; \
  this->SUB ## _type_support = \
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32); \
  this->SUB ## _options = rcl_subscription_get_default_options(); \
  ret = rcl_subscription_init(&this->SUB, &this->node, this->SUB ## _type_support, \
      this->SUB ## _topic_name, &this->SUB ## _options); \
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str; \
  EXPECT_TRUE(rcl_subscription_is_valid(&this->SUB)); \
  rcl_reset_error();

class TestDefaultExecutor : public ::testing::Test
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

  // integer publisher 3
  rcl_publisher_t pub3;
  const char * pub3_topic_name;
  const rosidl_message_type_support_t * pub3_type_support;
  rcl_publisher_options_t pub3_options;
  std_msgs__msg__Int32 pub3_msg;

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

  // integer subscription 3
  rcl_subscription_t sub3;
  const char * sub3_topic_name;
  const rosidl_message_type_support_t * sub3_type_support;
  rcl_subscription_options_t sub3_options;
  std_msgs__msg__Int32 sub3_msg;

  // timer 1
  rcl_timer_t timer1;
  const unsigned int timer1_timeout = 100;
  rcl_clock_t clock;
  rcl_allocator_t clock_allocator;
  const rcl_allocator_t * allocator_ptr;

  void SetUp()
  {
    rcl_ret_t ret;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    ret = rcl_init_options_init(&init_options, rcl_get_default_allocator());
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    // TODO(jan) hmm das kann weg, oder?
    OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT({
      EXPECT_EQ(RCL_RET_OK, rcl_init_options_fini(&init_options)) << rcl_get_error_string().str;
    });
    // sollte normales objekt sein
    this->context = rcl_get_zero_initialized_context();
    ret = rcl_init(0, nullptr, &init_options, &this->context);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

    // create ROS node
    this->node = rcl_get_zero_initialized_node();
    const char * name = "executor_test";
    rcl_node_options_t node_options = rcl_node_get_default_options();
    ret = rcl_node_init(&this->node, name, "", &this->context, &node_options);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    const rcl_node_options_t * node_ops = rcl_node_get_options(&this->node);
    this->allocator_ptr = &node_ops->allocator;

    // create publishers - correspond to member variables pub1, pub2, pub3
    // topic name can be configured (should only match with subscription configuration)
    CREATE_PUBLISHER(pub1, data1_int)
    CREATE_PUBLISHER(pub2, data2_int)
    CREATE_PUBLISHER(pub3, data3_int)

    // create subscriptions - correspond to member variables sub1, sub2, sub3
    // topic name of subscriber i (subi) must be the same as publisher i (pubi)
    CREATE_SUBSCRIPTION(sub1, data1_int)
    CREATE_SUBSCRIPTION(sub2, data2_int)
    CREATE_SUBSCRIPTION(sub3, data3_int)

    // create timer with rcl
    this->clock_allocator = rcl_get_default_allocator();
    ret = rcl_clock_init(RCL_STEADY_TIME, &this->clock, &this->clock_allocator);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    this->timer1 = rcl_get_zero_initialized_timer();
    ret =
      rcl_timer_init(&this->timer1, &this->clock, &this->context, RCL_MS_TO_NS(
          this->timer1_timeout), my_timer_callback, this->clock_allocator);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  }

  void TearDown()
  {
    rcl_ret_t ret;
    ret = rcl_subscription_fini(&this->sub1, &this->node);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ret = rcl_subscription_fini(&this->sub2, &this->node);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ret = rcl_subscription_fini(&this->sub3, &this->node);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ret = rcl_timer_fini(&this->timer1);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ret = rcl_clock_fini(&this->clock);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ret = rcl_node_fini(&this->node);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ret = rcl_shutdown(&this->context);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ret = rcl_context_fini(&this->context);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  }
};

/*
 * Test suite
 */
TEST_F(TestDefaultExecutor, executor_init) {
  rcl_ret_t rc;

  rclc_executor_t executor;
  executor = rclc_executor_get_zero_initialized_executor();

  rc = rclc_executor_init(&executor, &this->context, 10, this->allocator_ptr);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;

  rc = rclc_executor_fini(&executor);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;

  // Executor: NULL executor
  rc = rclc_executor_init(NULL, &this->context, 10, this->allocator_ptr);
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, rc) << rcl_get_error_string().str;
  rcutils_reset_error();

  rc = rclc_executor_fini(&executor);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcutils_reset_error();

  // Error case: zero handles
  rc = rclc_executor_init(&executor, &this->context, 0, this->allocator_ptr);
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, rc) << rcl_get_error_string().str;
  rcutils_reset_error();

  rc = rclc_executor_fini(&executor);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcutils_reset_error();
}


TEST_F(TestDefaultExecutor, executor_fini) {
  rcl_ret_t rc;
  rclc_executor_t executor;

  executor = rclc_executor_get_zero_initialized_executor();
  rc = rclc_executor_init(&executor, &this->context, 10, this->allocator_ptr);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;

  // normal case
  rc = rclc_executor_fini(&executor);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;

  // its okay to call rclc_executor_fini twice
  rc = rclc_executor_fini(&executor);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcutils_reset_error();
}

TEST_F(TestDefaultExecutor, executor_add_subscription) {
  rcl_ret_t rc;
  rclc_executor_t executor;
  executor = rclc_executor_get_zero_initialized_executor();
  // test with normal arguemnt and NULL pointers as arguments
  rc = rclc_executor_init(&executor, &this->context, 10, this->allocator_ptr);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;

  // normal case
  rc = rclc_executor_add_subscription(&executor, &this->sub1, &this->sub1_msg,
      &INT_CALLBACK(1), ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcutils_reset_error();
  size_t num_subscriptions = 1;
  EXPECT_EQ(executor.info.number_of_subscriptions, num_subscriptions) <<
    "number of subscriptions is expected to be one";

  // test NULL pointer for executor
  rc = rclc_executor_add_subscription(NULL, &this->sub1, &this->sub1_msg, &CALLBACK_1,
      ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, rc) << rcl_get_error_string().str;
  rcutils_reset_error();
  EXPECT_EQ(executor.info.number_of_subscriptions, num_subscriptions) <<
    "number of subscriptions is expected to be one";

  // test NULL pointer for subscription
  rc = rclc_executor_add_subscription(&executor, NULL, &this->sub1_msg, &CALLBACK_1,
      ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, rc) << rcl_get_error_string().str;
  rcutils_reset_error();
  EXPECT_EQ(executor.info.number_of_subscriptions, num_subscriptions) <<
    "number of subscriptions is expected to be one";

  // test NULL pointer for message
  rc = rclc_executor_add_subscription(&executor, &this->sub1, NULL, &CALLBACK_1,
      ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, rc) << rcl_get_error_string().str;
  rcutils_reset_error();
  EXPECT_EQ(executor.info.number_of_subscriptions, num_subscriptions) <<
    "number of subscriptions is expected to be one";

  // test NULL pointer for callback
  rc = rclc_executor_add_subscription(&executor, &this->sub1, &this->sub1_msg, NULL,
      ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, rc) << rcl_get_error_string().str;
  rcutils_reset_error();
  EXPECT_EQ(executor.info.number_of_subscriptions, num_subscriptions) <<
    "number of subscriptions is expected to be one";

  // tear down
  rc = rclc_executor_fini(&executor);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
}

TEST_F(TestDefaultExecutor, executor_add_subscription_too_many) {
  rcl_ret_t rc;
  rclc_executor_t executor;
  executor = rclc_executor_get_zero_initialized_executor();

  // insert one handle, add two subscriptions
  rc = rclc_executor_init(&executor, &this->context, 1, this->allocator_ptr);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;

  // test 1: add subscription
  rc = rclc_executor_add_subscription(&executor, &this->sub1, &this->sub1_msg,
      &CALLBACK_1, ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  size_t num_subscriptions = 1;
  EXPECT_EQ(executor.info.number_of_subscriptions, num_subscriptions) <<
    "number of subscriptions is expected to be one";

  // test 2: add another subscription : failure (array full)
  rc = rclc_executor_add_subscription(&executor, &this->sub2, &this->sub2_msg,
      &CALLBACK_2, ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_ERROR, rc) << rcl_get_error_string().str;
  rcutils_reset_error();
  EXPECT_EQ(executor.info.number_of_subscriptions, num_subscriptions) <<
    "number of subscriptions is expected to be one";

  // tear down
  rc = rclc_executor_fini(&executor);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
}

TEST_F(TestDefaultExecutor, executor_add_timer) {
  rcl_ret_t rc;
  rclc_executor_t executor;
  executor = rclc_executor_get_zero_initialized_executor();
  // add a timer
  rc = rclc_executor_init(&executor, &this->context, 10, this->allocator_ptr);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;

  size_t exp_number_of_timers = 0;
  EXPECT_EQ(executor.info.number_of_timers, exp_number_of_timers) << "#times should be 0";
  rc = rclc_executor_add_timer(&executor, &this->timer1);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  exp_number_of_timers = 1;
  EXPECT_EQ(executor.info.number_of_timers, exp_number_of_timers) << "#timers should be 1";

  // tear down
  rc = rclc_executor_fini(&executor);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
}
/*
TEST_F(TestDefaultExecutor, executor_spin_some_API) {
  rcl_ret_t rc;
  rclc_executor_t executor;
  executor = rclc_executor_get_zero_initialized_executor();
  // add a timer
  rc = rclc_executor_init(&executor, &this->context, 10, this->allocator_ptr);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;

  rc = rclc_executor_add_timer(&executor, &this->timer1);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;

  const unsigned int timeout_ms = 100;
  rc = rclc_executor_spin_some(&executor, timeout_ms);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;

  // tear down
  rc = rclc_executor_fini(&executor);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
}


void
wait_for_subscription_to_be_ready(
  rcl_subscription_t * subscription,
  rcl_context_t * context_ptr,
  size_t max_tries,
  int64_t period_ms,
  bool & success)
{
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_ret_t ret = rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, context_ptr,
      rcl_get_default_allocator());
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT({
    rcl_ret_t ret = rcl_wait_set_fini(&wait_set);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });
  size_t iteration = 0;
  do {
    ++iteration;
    ret = rcl_wait_set_clear(&wait_set);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ret = rcl_wait_set_add_subscription(&wait_set, subscription, NULL);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ret = rcl_wait(&wait_set, RCL_MS_TO_NS(period_ms));
    if (ret == RCL_RET_TIMEOUT) {
      continue;
    }
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    for (size_t i = 0; i < wait_set.size_of_subscriptions; ++i) {
      if (wait_set.subscriptions[i] && wait_set.subscriptions[i] == subscription) {
        success = true;
        return;
      }
    }
  } while (iteration < max_tries);
  success = false;
}

TEST_F(TestDefaultExecutor, pub_sub_example) {
  // 27.06.2019, copied from ros2/rcl/rcl/test/rcl/test_subscriptions.cpp
  // by Jan Staschulat, under Apache 2.0 License
  rcl_ret_t ret;
  unsigned int expected_msg;

  // publisher
  rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
  const rosidl_message_type_support_t * ts =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
  const char * topic = "chatter";
  const char * expected_topic = "/chatter";
  rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();
  ret = rcl_publisher_init(&publisher, this->node_ptr, ts, topic, &publisher_options);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT({
    rcl_ret_t ret = rcl_publisher_fini(&publisher, this->node_ptr);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });

  // subscription
  rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t subscription_options = rcl_subscription_get_default_options();
  ret = rcl_subscription_init(&subscription, this->node_ptr, ts, topic, &subscription_options);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT({
    rcl_ret_t ret = rcl_subscription_fini(&subscription, this->node_ptr);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });
  EXPECT_EQ(strcmp(rcl_subscription_get_topic_name(&subscription), expected_topic), 0);
  EXPECT_TRUE(rcl_subscription_is_valid(&subscription));
  rcl_reset_error();

  // TODO(wjwwood): add logic to wait for the connection to be established
  //                probably using the count_subscriptions busy wait mechanism
  //                until then we will sleep for a short period of time
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  {
    std_msgs__msg__Int32 msg;
    std_msgs__msg__Int32__init(&msg);
    msg.data = 42;
    ret = rcl_publish(&publisher, &msg, nullptr);
    std_msgs__msg__Int32__fini(&msg);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  }
  bool success;
  wait_for_subscription_to_be_ready(&subscription, &this->context, 10, 100, success);
  ASSERT_TRUE(success);
  {
    std_msgs__msg__Int32 msg;
    std_msgs__msg__Int32__init(&msg);
    OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT({
      std_msgs__msg__Int32__fini(&msg);
    });
    ret = rcl_take(&subscription, &msg, nullptr, nullptr);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ASSERT_EQ(42, msg.data);

    // initialize all callback counters
    _results_callback_counters_init();
    // call callback with msg
    int32_callback1(&msg);
    // check result
    expected_msg = 1;
    ASSERT_EQ(_cb1_cnt, expected_msg) << "expect = 1";
  }
}


TEST_F(TestDefaultExecutor, spin_some_let_semantic) {
  // 27.06.2019, adopted from ros2/rcl/rcl/test/rcl/test_subscriptions.cpp
  // by Jan Staschulat, under Apache 2.0 License
  rcl_ret_t ret;
  rclc_executor_t executor;
  unsigned int expected_msg;
  executor = rclc_executor_get_zero_initialized_executor();
  // publisher 1
  rcl_publisher_t publisher1 = rcl_get_zero_initialized_publisher();
  const rosidl_message_type_support_t * ts1 =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
  const char * topic1 = "chatter1";
  rcl_publisher_options_t publisher_options1 = rcl_publisher_get_default_options();
  ret = rcl_publisher_init(&publisher1, this->node_ptr, ts1, topic1, &publisher_options1);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT({
    rcl_ret_t ret = rcl_publisher_fini(&publisher1, this->node_ptr);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });

  // publisher 2
  rcl_publisher_t publisher2 = rcl_get_zero_initialized_publisher();
  const rosidl_message_type_support_t * ts2 =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
  const char * topic2 = "chatter2";
  rcl_publisher_options_t publisher_options2 = rcl_publisher_get_default_options();
  ret = rcl_publisher_init(&publisher2, this->node_ptr, ts2, topic2, &publisher_options2);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT({
    rcl_ret_t ret = rcl_publisher_fini(&publisher2, this->node_ptr);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });

  // publisher 3
  rcl_publisher_t publisher3 = rcl_get_zero_initialized_publisher();
  const rosidl_message_type_support_t * ts3 =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
  const char * topic3 = "chatter3";
  rcl_publisher_options_t publisher_options3 = rcl_publisher_get_default_options();
  ret = rcl_publisher_init(&publisher3, this->node_ptr, ts3, topic3, &publisher_options3);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT({
    rcl_ret_t ret = rcl_publisher_fini(&publisher3, this->node_ptr);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });


  // subscription 1
  rcl_subscription_t subscription1 = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t subscription_options1 = rcl_subscription_get_default_options();
  ret = rcl_subscription_init(&subscription1, this->node_ptr, ts1, topic1, &subscription_options1);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT({
    rcl_ret_t ret = rcl_subscription_fini(&subscription1, this->node_ptr);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });
  EXPECT_TRUE(rcl_subscription_is_valid(&subscription1));
  rcl_reset_error();

  // subscription 2
  rcl_subscription_t subscription2 = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t subscription_options2 = rcl_subscription_get_default_options();
  ret = rcl_subscription_init(&subscription2, this->node_ptr, ts2, topic2, &subscription_options2);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT({
    rcl_ret_t ret = rcl_subscription_fini(&subscription2, this->node_ptr);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });
  EXPECT_TRUE(rcl_subscription_is_valid(&subscription2));
  rcl_reset_error();

  // subscription 3
  rcl_subscription_t subscription3 = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t subscription_options3 = rcl_subscription_get_default_options();
  ret = rcl_subscription_init(&subscription3, this->node_ptr, ts3, topic3, &subscription_options3);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT({
    rcl_ret_t ret = rcl_subscription_fini(&subscription3, this->node_ptr);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });
  EXPECT_TRUE(rcl_subscription_is_valid(&subscription3));
  rcl_reset_error();

  // initialize result variables
  _executor_results_init();
  // initialize executor with 3 handles
  ret = rclc_executor_init(&executor, &this->context, 3, this->allocator_ptr);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  // define subscription messages
  std_msgs__msg__Int32 sub_msg1;
  std_msgs__msg__Int32__init(&sub_msg1);

  std_msgs__msg__Int32 sub_msg2;
  std_msgs__msg__Int32__init(&sub_msg2);

  std_msgs__msg__Int32 sub_msg3;
  std_msgs__msg__Int32__init(&sub_msg3);

  // add subscription to the executor
  ret =
    rclc_executor_add_subscription(&executor, &subscription1, &sub_msg1, &int32_callback1,
      ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  rcutils_reset_error();

  ret =
    rclc_executor_add_subscription(&executor, &subscription2, &sub_msg2, &int32_callback2,
      ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  rcutils_reset_error();

  ret =
    rclc_executor_add_subscription(&executor, &subscription3, &sub_msg3, &int32_callback3,
      ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  rcutils_reset_error();

  // check, if all subscriptions were added
  size_t num_subscriptions = 3;
  EXPECT_EQ(executor.info.number_of_subscriptions, num_subscriptions) <<
    "number of subscriptions is expected 3";

  // publish message 1
  std_msgs__msg__Int32 pub_msg1;
  std_msgs__msg__Int32__init(&pub_msg1);
  pub_msg1.data = 1;

  // publish message 2
  std_msgs__msg__Int32 pub_msg2;
  std_msgs__msg__Int32__init(&pub_msg2);
  pub_msg2.data = 2;

  // publish message 3
  std_msgs__msg__Int32 pub_msg3;
  std_msgs__msg__Int32__init(&pub_msg3);
  pub_msg3.data = 3;

  ///////////////////////////////////////////////////////////////////////////////////
  /////////// test case 1 : sent in same order
  ///////////////////////////////////////////////////////////////////////////////////

  for (unsigned int i = 0; i < kMax; i++) {
    ret = rcl_publish(&publisher1, &pub_msg1, nullptr);
    EXPECT_EQ(RCL_RET_OK, ret) << " pub1 not published";

    ret = rcl_publish(&publisher2, &pub_msg2, nullptr);
    EXPECT_EQ(RCL_RET_OK, ret) << " pub2 not published";

    ret = rcl_publish(&publisher3, &pub_msg3, nullptr);
    EXPECT_EQ(RCL_RET_OK, ret) << " pub3 not published";
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  // running the executor
  for (unsigned int i = 0; i < 30; i++) {
    const unsigned int timeout_ms = 100;
    ret = rclc_executor_spin_some(&executor, timeout_ms);
    if ((ret == RCL_RET_OK) || (ret == RCL_RET_TIMEOUT)) {
      // valid return values
    } else {
      // any other error
      EXPECT_EQ(RCL_RET_OK, ret) << "spin_some error";
    }

    // finish - if all messages have been received
    if (_executor_results_all_msg_received()) {
      break;
    }
  }
  // check total number of received messages
  expected_msg = kMax;
  EXPECT_EQ(_cb1_cnt, expected_msg) << "cb1 msg does not match";
  EXPECT_EQ(_cb2_cnt, expected_msg) << "cb2 msg does not match";
  EXPECT_EQ(_cb3_cnt, expected_msg) << "cb3 msg does not match";
  // test order of received handles
  // _executor_results_print();

  // assumption !!! expecting kMax=3 => 9 messages
  unsigned int result_tc1[] = {1, 2, 3, 1, 2, 3, 1, 2, 3};
  EXPECT_EQ(_executor_results_compare(result_tc1), true) << "[1,2,3, ...] expected";

  ///////////////////////////////////////////////////////////////////////////////////
  /////////// test case 2 : sent in reverse order
  ///////////////////////////////////////////////////////////////////////////////////
  _executor_results_init();
  // now sent in different order
  for (unsigned int i = 0; i < kMax; i++) {
    ret = rcl_publish(&publisher3, &pub_msg3, nullptr);
    EXPECT_EQ(RCL_RET_OK, ret) << " pub3 not published";

    ret = rcl_publish(&publisher2, &pub_msg2, nullptr);
    EXPECT_EQ(RCL_RET_OK, ret) << " pub2 not published";

    ret = rcl_publish(&publisher1, &pub_msg1, nullptr);
    EXPECT_EQ(RCL_RET_OK, ret) << " pub1 not published";
  }
  // wait for some time until DDS queue is ready
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  // running the executor
  for (unsigned int i = 0; i < 30; i++) {
    const unsigned int timeout_ms = 100;
    ret = rclc_executor_spin_some(&executor, timeout_ms);
    if ((ret == RCL_RET_OK) || (ret == RCL_RET_TIMEOUT)) {
      // valid return values
    } else {
      // any other error
      EXPECT_EQ(RCL_RET_OK, ret) << "spin_some error";
    }

    if (_executor_results_all_msg_received()) {
      break;
    }

    // wait for some time
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  // check total number of received messages
  expected_msg = kMax;
  EXPECT_EQ(_cb1_cnt, expected_msg) << "cb1 msg does not match";
  EXPECT_EQ(_cb2_cnt, expected_msg) << "cb2 msg does not match";
  EXPECT_EQ(_cb3_cnt, expected_msg) << "cb3 msg does not match";
  // test order of received handles
  // _executor_results_print();

  // assumption !!! expecting kMax=3 => 9 messages
  unsigned int result_tc2[] = {1, 2, 3, 1, 2, 3, 1, 2, 3};
  EXPECT_EQ(_executor_results_compare(result_tc2), true) << "[1,2,3, ...] expected";

  // clean-up
  std_msgs__msg__Int32__init(&pub_msg1);
  std_msgs__msg__Int32__init(&pub_msg2);
  std_msgs__msg__Int32__init(&pub_msg3);

  std_msgs__msg__Int32__init(&sub_msg1);
  std_msgs__msg__Int32__init(&sub_msg2);
  std_msgs__msg__Int32__init(&sub_msg3);

  ret = rclc_executor_fini(&executor);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
}

TEST_F(TestDefaultExecutor, invocation_type) {
  // 27.06.2019, adopted from ros2/rcl/rcl/test/rcl/test_subscriptions.cpp
  // by Jan Staschulat, under Apache 2.0 License

  // test for invocation type ALWAYS ON_NEW_DATA
  //
  // publisher A
  // send 1 message
  // subscriber A' with invocation=ALWAYS

  // publisher B
  // send 1 message
  // subscriber B' with invocation=ON_NEW_DATA

  // executor setup()
  // executor_spin_some()
  // executor_spin_some()

  // expected result
  // number of invocations of callback A' = 2
  // number of invocations of callback B' = 1

// 27.06.2019, adopted from ros2/rcl/rcl/test/rcl/test_subscriptions.cpp
  // by Jan Staschulat, under Apache 2.0 License
  rcl_ret_t ret;
  rclc_executor_t executor;
  executor = rclc_executor_get_zero_initialized_executor();
  // publisher 1
  rcl_publisher_t publisher1 = rcl_get_zero_initialized_publisher();
  const rosidl_message_type_support_t * ts1 =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
  const char * topic1 = "chatter1";
  rcl_publisher_options_t publisher_options1 = rcl_publisher_get_default_options();
  ret = rcl_publisher_init(&publisher1, this->node_ptr, ts1, topic1, &publisher_options1);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT({
    rcl_ret_t ret = rcl_publisher_fini(&publisher1, this->node_ptr);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });

  // publisher 2
  rcl_publisher_t publisher2 = rcl_get_zero_initialized_publisher();
  const rosidl_message_type_support_t * ts2 =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
  const char * topic2 = "chatter2";
  rcl_publisher_options_t publisher_options2 = rcl_publisher_get_default_options();
  ret = rcl_publisher_init(&publisher2, this->node_ptr, ts2, topic2, &publisher_options2);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT({
    rcl_ret_t ret = rcl_publisher_fini(&publisher2, this->node_ptr);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });

  // subscription 1
  rcl_subscription_t subscription1 = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t subscription_options1 = rcl_subscription_get_default_options();
  ret = rcl_subscription_init(&subscription1, this->node_ptr, ts1, topic1, &subscription_options1);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT({
    rcl_ret_t ret = rcl_subscription_fini(&subscription1, this->node_ptr);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });
  EXPECT_TRUE(rcl_subscription_is_valid(&subscription1));
  rcl_reset_error();

  // subscription 2
  rcl_subscription_t subscription2 = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t subscription_options2 = rcl_subscription_get_default_options();
  ret = rcl_subscription_init(&subscription2, this->node_ptr, ts2, topic2, &subscription_options2);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT({
    rcl_ret_t ret = rcl_subscription_fini(&subscription2, this->node_ptr);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });
  EXPECT_TRUE(rcl_subscription_is_valid(&subscription2));
  rcl_reset_error();

  // initialize result variables
  _executor_results_init();
  // initialize executor with 2 handles
  ret = rclc_executor_init(&executor, &this->context, 2, this->allocator_ptr);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  ret = rclc_executor_set_trigger(&executor, rclc_executor_trigger_always, NULL);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  // define subscription messages
  std_msgs__msg__Int32 sub_msg1;
  std_msgs__msg__Int32__init(&sub_msg1);

  std_msgs__msg__Int32 sub_msg2;
  std_msgs__msg__Int32__init(&sub_msg2);

  // add subscription to the executor
  ret =
    rclc_executor_add_subscription(&executor, &subscription1, &sub_msg1, &int32_callback1,
      ALWAYS);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  rcutils_reset_error();

  ret =
    rclc_executor_add_subscription(&executor, &subscription2, &sub_msg2, &int32_callback2,
      ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  rcutils_reset_error();

  // check, if all subscriptions were added
  size_t num_subscriptions = 2;
  EXPECT_EQ(executor.info.number_of_subscriptions, num_subscriptions) <<
    "number of subscriptions should be = 2";

  // publish message 1
  std_msgs__msg__Int32 pub_msg1;
  std_msgs__msg__Int32__init(&pub_msg1);
  pub_msg1.data = 1;

  // publish message 2
  std_msgs__msg__Int32 pub_msg2;
  std_msgs__msg__Int32__init(&pub_msg2);
  pub_msg2.data = 2;


  ///////////////////////////////////////////////////////////////////////////////////
  /////////// test case 1 : publish one data for each publisher
  ///////////////////////////////////////////////////////////////////////////////////


  ret = rcl_publish(&publisher1, &pub_msg1, nullptr);
  EXPECT_EQ(RCL_RET_OK, ret) << " publisher1 did not publish!";
  ret = rcl_publish(&publisher2, &pub_msg2, nullptr);
  EXPECT_EQ(RCL_RET_OK, ret) << " publisher2 did not publish!";
  // give DDS some time to publish the message (without the test fails)
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // initialize result variables
  _cb1_cnt = 0;
  _cb2_cnt = 0;

  // running the executor
  unsigned int max_iterations = 2;
  for (unsigned int i = 0; i < max_iterations; i++) {
    const unsigned int timeout_ms = 100;
    ret = rclc_executor_spin_some(&executor, timeout_ms);
    if ((ret == RCL_RET_OK) || (ret == RCL_RET_TIMEOUT)) {
      // valid return values
    } else {
      // any other error
      EXPECT_EQ(RCL_RET_OK, ret) << "spin_some error";
    }
  }
  // check total number of received messages
  EXPECT_EQ(_cb1_cnt, (unsigned int) 2) << "cb1 msg does not match";
  EXPECT_EQ(_cb2_cnt, (unsigned int) 1) << "cb2 msg does not match";
}


// call executor_add_subscription() after executor_spin_some()
// this requires an update of the rcl wait_set
TEST_F(TestDefaultExecutor, update_wait_set) {
  rcl_ret_t ret;
  rclc_executor_t executor;
  const unsigned int timeout_ms = 100;

  executor = rclc_executor_get_zero_initialized_executor();

  // publisher 1
  rcl_publisher_t publisher1 = rcl_get_zero_initialized_publisher();
  const rosidl_message_type_support_t * ts1 =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
  const char * topic1 = "chatter1";
  rcl_publisher_options_t publisher_options1 = rcl_publisher_get_default_options();
  ret = rcl_publisher_init(&publisher1, this->node_ptr, ts1, topic1, &publisher_options1);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT({
    rcl_ret_t ret = rcl_publisher_fini(&publisher1, this->node_ptr);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });

  // subscription 1
  rcl_subscription_t subscription1 = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t subscription_options1 = rcl_subscription_get_default_options();
  ret = rcl_subscription_init(&subscription1, this->node_ptr, ts1, topic1, &subscription_options1);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT({
    rcl_ret_t ret = rcl_subscription_fini(&subscription1, this->node_ptr);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });
  EXPECT_TRUE(rcl_subscription_is_valid(&subscription1));
  rcl_reset_error();

  // subscription 2
  rcl_subscription_t subscription2 = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t subscription_options2 = rcl_subscription_get_default_options();
  ret = rcl_subscription_init(&subscription2, this->node_ptr, ts1, topic1, &subscription_options2);
  ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT({
    rcl_ret_t ret = rcl_subscription_fini(&subscription2, this->node_ptr);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  });
  EXPECT_TRUE(rcl_subscription_is_valid(&subscription2));
  rcl_reset_error();

  // initialize result variables
  _results_callback_counters_init();
  // initialize executor with 2 handles
  ret = rclc_executor_init(&executor, &this->context, 2, this->allocator_ptr);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  // define subscription messages
  std_msgs__msg__Int32 sub_msg1;
  std_msgs__msg__Int32__init(&sub_msg1);

  std_msgs__msg__Int32 sub_msg2;
  std_msgs__msg__Int32__init(&sub_msg2);

  // initially the wait_set is zero_initialized
  EXPECT_EQ(false, rcl_wait_set_is_valid(&executor.wait_set));
  // add subscription to the executor
  ret =
    rclc_executor_add_subscription(&executor, &subscription1, &sub_msg1, &int32_callback1,
      ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  rcutils_reset_error();

  // wait_set is not valid
  EXPECT_EQ(false, rcl_wait_set_is_valid(&executor.wait_set));

  // initialize pub message 1
  std_msgs__msg__Int32 pub_msg1;
  std_msgs__msg__Int32__init(&pub_msg1);
  pub_msg1.data = 1;

  ///////////////////////////////////////////////////////////////////////////////////
  /////////// test case 1 : spin_once
  ///////////////////////////////////////////////////////////////////////////////////

  // received one message
  EXPECT_EQ((unsigned int)0, _cb1_cnt);
  EXPECT_EQ((unsigned int)0, _cb2_cnt);

  ret = rcl_publish(&publisher1, &pub_msg1, nullptr);
  EXPECT_EQ(RCL_RET_OK, ret) << " publisher1 did not publish!";
  ret = rcl_publish(&publisher1, &pub_msg1, nullptr);
  EXPECT_EQ(RCL_RET_OK, ret) << " publisher1 did not publish!";

  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  ret = rclc_executor_spin_some(&executor, timeout_ms);
  if ((ret == RCL_RET_OK) || (ret == RCL_RET_TIMEOUT)) {
    // valid return values
  } else {
    // any other error
    EXPECT_EQ(RCL_RET_OK, ret) << "spin_some error";
  }

  // wait_set is valid
  EXPECT_EQ(true, rcl_wait_set_is_valid(&executor.wait_set));

  // received one message
  EXPECT_EQ((unsigned int)1, _cb1_cnt);
  EXPECT_EQ((unsigned int)0, _cb2_cnt);

  ///////////////////////////////////////////////////////////////////////////////////
  /////////// test case 2 : add another subscription
  ///////////////////////////////////////////////////////////////////////////////////

  // wait_set is valid
  EXPECT_EQ(true, rcl_wait_set_is_valid(&executor.wait_set));

  ret =
    rclc_executor_add_subscription(&executor, &subscription2, &sub_msg2, &int32_callback2,
      ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  rcutils_reset_error();

  // wait_set is not valid
  EXPECT_EQ(false, rcl_wait_set_is_valid(&executor.wait_set));

  ///////////////////////////////////////////////////////////////////////////////////
  /////////// test case 3 : spin_some again
  ///////////////////////////////////////////////////////////////////////////////////

  ret = rclc_executor_spin_some(&executor, timeout_ms);
  if ((ret == RCL_RET_OK) || (ret == RCL_RET_TIMEOUT)) {
    // valid return values
  } else {
    // any other error
    EXPECT_EQ(RCL_RET_OK, ret) << "spin_some error";
  }

  // wait_set is valid
  EXPECT_EQ(true, rcl_wait_set_is_valid(&executor.wait_set));

  // received two msg at callback 1 and one msg at callback 2:
  EXPECT_EQ((unsigned int)2, _cb1_cnt);
  EXPECT_EQ((unsigned int)1, _cb2_cnt);
}


TEST_F(TestDefaultExecutor, spin_period) {
  rcl_ret_t rc;
  rclc_executor_t executor;

  // initialize executor with 1 handle
  rc = rclc_executor_init(&executor, &this->context, 1, this->allocator_ptr);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;

  // set timeout to zero - so that rcl_wait() comes back immediately
  rc = rclc_executor_set_timeout(&executor, 0);

  // add dummy subscription (with string msg), which is always executed
  rc = rclc_executor_add_subscription(&executor, &this->sub2, &this->sub2_msg,
      &spin_period_callback, ALWAYS);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcutils_reset_error();

  rclc_executor_set_trigger(&executor, rclc_executor_trigger_always, NULL);

  // measure the timepoint, when spin_period_callback() is called
  uint64_t spin_period = 20000000;  // 20 ms
  for (unsigned int i = 0; i < MAX_SPIN_PERIOD_INVOCATIONS; i++) {
    rclc_executor_spin_one_period(&executor, spin_period);
  }
  // compute avarage time duration between calls to spin_period_callback
  uint64_t duration = test_case_evaluate_spin_period();
  printf("expected 'spin_period' : %ld\n", spin_period);
  printf("actual      'duration' : %ld\n", duration);

  uint64_t delta = 10000;  // 10 micro-seconds bound
  EXPECT_LE(duration, spin_period + delta);
  EXPECT_LE(spin_period - delta, duration);
}

TEST_F(TestDefaultExecutor, semantics_RCLCPP) {
  rcl_ret_t rc;
  rclc_executor_t executor;

  // configuration
  // - one publisher
  //    - publishes integer topic X
  // - subscriber A
  //    - subscribes to X
  //    - publishes also on topic X
  // - subscriber B
  //     - subscribes to X with DDS quality of service option: last-is-best
  //     - evaluates data of topic X and copies it to global variable result_X
  // - evaluation in test case, what the value of result_X is.
  //
  // test setup
  //  - publisher: publishes X=1
  //  - sleep(100ms)
  //  - spin_some()
  //  - subscriber A : publishes X=2
  //  - sleep(100ms)
  //  - subscriber B :
  // expected test result for semantics RCLCPP:
  //   => subscriber B receives X=2 (because it takes most rececent data)
  // expected test result for semantics LET:
  //   => subscriber B receives X=1 (because data is taken from DDS at the start of spin_some() )

  // implementation
  // use member variable this->pub1 as publisher
  // use member variable this->sub1 as subscription 1
  // create subscription 2 with last-is-best semantics
  rcl_subscription_t subscription2 = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t subscription_options2 = rcl_subscription_get_default_options();
  std_msgs__msg__Int32 subscription2_int_msg;
  subscription_options2.qos.depth = 0;  // qos: last is best
  rc = rcl_subscription_init(&subscription2, this->node_ptr, this->pub1_type_support,
      this->pub1_topic_name, &subscription_options2);
  ASSERT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  EXPECT_TRUE(rcl_subscription_is_valid(&subscription2));
  rcl_reset_error();

  // initialize executor with 2 handles
  rc = rclc_executor_init(&executor, &this->context, 2, this->allocator_ptr);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcl_reset_error();

  // add subscriptions to executor
  rc = rclc_executor_add_subscription(&executor, &this->sub1, &this->sub1_msg,
      &int32_callback4, ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcutils_reset_error();
  rc = rclc_executor_add_subscription(&executor, &subscription2, &subscription2_int_msg,
      &int32_callback5, ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcutils_reset_error();

  // create global pointer to this publisher to access it from callback 'int32_callback4'
  // of the subscriber 'this->sub_int'
  _pub_int_ptr = &this->pub1;
  _pub_int_msg_ptr = &this->pub1_msg;

  // ------------------------- test case setup ------------------------
  rclc_executor_set_semantics(&executor, RCLCPP_EXECUTOR);
  this->pub1_msg.data = 1;
  _cb5_int_value = 0;  // received value in subscription2
  rc = rcl_publish(&this->pub1, &this->pub1_msg, nullptr);
  EXPECT_EQ(RCL_RET_OK, rc) << " pub1(tc) did not publish!";

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  const uint64_t timeout_ns = 10000000;  // 10ms
  rclc_executor_spin_some(&executor, timeout_ns);
  // test result
  EXPECT_EQ(_cb5_int_value,
    (unsigned int) 2) <<
    " expect value 2: Value from callback of int32_callback4 should be received.";

  // clean-up
  rc = rcl_subscription_fini(&subscription2, this->node_ptr);
}


TEST_F(TestDefaultExecutor, semantics_LET) {
  rcl_ret_t rc;
  rclc_executor_t executor;

  // copy-ans-paste  from semantics_RCLCPP except:
  // - semantics = LET
  // - expected result = 1 (original value of publisher)

  // configuration
  // - one publisher
  //    - publishes integer topic X
  // - subscriber A
  //    - subscribes to X
  //    - publishes also on topic X
  // - subscriber B
  //     - subscribes to X with DDS quality of service option: last-is-best
  //     - evaluates data of topic X and copies it to global variable result_X
  // - evaluation in test case, what the value of result_X is.
  //
  // test setup
  //  - publisher: publishes X=1
  //  - sleep(100ms)
  //  - spin_some()
  //  - subscriber A : publishes X=2
  //  - sleep(100ms)
  //  - subscriber B :
  // expected test result for semantics RCLCPP:
  //   => subscriber B receives X=2 (because it takes most rececent data)
  // expected test result for semantics LET:
  //   => subscriber B receives X=1 (because data is taken from DDS at the start of spin_some() )

  // implementation
  // use member variable this->pub1 as publisher
  // use member variable this->sub1 as subscription 1
  // create subscription 2 with last-is-best semantics
  rcl_subscription_t subscription2 = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t subscription_options2 = rcl_subscription_get_default_options();
  std_msgs__msg__Int32 subscription2_int_msg;
  subscription_options2.qos.depth = 0;  // qos: last is best
  rc = rcl_subscription_init(&subscription2, this->node_ptr, this->pub1_type_support,
      this->pub1_topic_name, &subscription_options2);
  ASSERT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  EXPECT_TRUE(rcl_subscription_is_valid(&subscription2));
  rcl_reset_error();

  // initialize executor with 2 handles
  rc = rclc_executor_init(&executor, &this->context, 2, this->allocator_ptr);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcl_reset_error();

  // add subscriptions to executor
  rc = rclc_executor_add_subscription(&executor, &this->sub1, &this->sub1_msg,
      &int32_callback4, ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcutils_reset_error();
  rc = rclc_executor_add_subscription(&executor, &subscription2, &subscription2_int_msg,
      &int32_callback5, ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcutils_reset_error();

  // create global pointer to this publisher to access it from callback 'int32_callback4'
  // of the subscriber 'this->sub_int'
  _pub_int_ptr = &this->pub1;
  _pub_int_msg_ptr = &this->pub1_msg;

  // ------------------------- test case setup ------------------------
  rclc_executor_set_semantics(&executor, LET);
  this->pub1_msg.data = 1;
  _cb5_int_value = 0;  // received value in subscription2
  rc = rcl_publish(&this->pub1, &this->pub1_msg, nullptr);
  EXPECT_EQ(RCL_RET_OK, rc) << " pub1(tc) did not publish!";

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  const uint64_t timeout_ns = 10000000;  // 10ms
  rclc_executor_spin_some(&executor, timeout_ns);
  // test result
  EXPECT_EQ(_cb5_int_value,
    (unsigned int) 1) <<
    " expect value 1: first value of 'pub1' publisher should have been received.";

  // clean-up
  rc = rcl_subscription_fini(&subscription2, this->node_ptr);
}


TEST_F(TestDefaultExecutor, trigger_one) {
  // test specification
  // multiple subscriptions
  //  - two publishers for topic a and topic b
  //  - two subscriptions A, B
  //     - subscriber A subscribes to topic a
  //     - subscriber B subscribes to topic b
  //     - subscriber B sets result variable.
  // set trigger: trigger_one(A) => execute when A receives new data
  // setup:
  // - publish topic A
  // - spin_some()
  // - expected result:
  //   - subscriber A called
  //   - subscriber B not called
  // - publish topic B
  // - spin_some()
  // - expected result:
  //   - subscriber A not called
  //   - subscriber B not called
  // - publish topic A
  // - spin_some()
  // - expected result:
  //   - subscriber A called
  //   - subscriber B called
  // -------------- rcl objects ---------------------------------------------------------

  rcl_ret_t rc;
  rclc_executor_t executor;
  // initialize executor with 2 handles
  rc = rclc_executor_init(&executor, &this->context, 2, this->allocator_ptr);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcl_reset_error();
  rc = rclc_executor_set_trigger(&executor, rclc_executor_trigger_one, &this->sub1);

  // add subscriptions to executor
  rc = rclc_executor_add_subscription(&executor, &this->sub1, &this->sub1_msg,
      &int32_callback6, ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcutils_reset_error();
  rc = rclc_executor_add_subscription(&executor, &this->sub2, &this->sub2_msg,
      &int32_callback7, ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcutils_reset_error();
  const uint64_t timeout_ns = 10000000;  // 10ms
  // ------------------------- test case setup ---------------------------------------------

  // first round
  _reset_result_values_for_trigger_test_case();
  this->pub1_msg.data = 3;
  rc = rcl_publish(&this->pub1, &this->pub1_msg, nullptr);
  EXPECT_EQ(RCL_RET_OK, rc) << " pub1 did not publish!";
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclc_executor_spin_some(&executor, timeout_ns);
  EXPECT_EQ(_cb6_int_value, (unsigned int) 3) << " expected: A called";
  EXPECT_EQ(_cb7_int_value, (unsigned int) 0) << " expected: B not called";
  EXPECT_EQ(_cb6_cnt, (unsigned int) 1);
  EXPECT_EQ(_cb7_cnt, (unsigned int) 0);
  // second round
  this->pub2_msg.data = 7;
  rc = rcl_publish(&this->pub2, &this->pub2_msg, nullptr);
  EXPECT_EQ(RCL_RET_OK, rc) << " pub2 did not publish!";
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclc_executor_spin_some(&executor, timeout_ns);
  EXPECT_EQ(_cb6_int_value, (unsigned int) 3) << " expected: A not called";
  EXPECT_EQ(_cb7_int_value, (unsigned int) 0) << " expected: B not called";
  EXPECT_EQ(_cb6_cnt, (unsigned int) 1);
  EXPECT_EQ(_cb7_cnt, (unsigned int) 0);

  // third round
  this->pub1_msg.data = 11;
  rc = rcl_publish(&this->pub1, &this->pub1_msg, nullptr);
  EXPECT_EQ(RCL_RET_OK, rc) << " pub1 did not publish!";
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclc_executor_spin_some(&executor, timeout_ns);
  EXPECT_EQ(_cb6_int_value, (unsigned int) 11) << " expected: A called";
  EXPECT_EQ(_cb7_int_value, (unsigned int) 7) << " expected: B called";
  EXPECT_EQ(_cb6_cnt, (unsigned int) 2);
  EXPECT_EQ(_cb7_cnt, (unsigned int) 1);
}


TEST_F(TestDefaultExecutor, trigger_any) {
  // test specification
  // set trigger: trigger_any => execute when A or B received new data
  // setup:
  // - publish topic A
  // - spin_some()
  // - expected result:
  //   - subscriber A called
  //   - subscriber B not called
  // - publish topic B
  // - spin_some()
  // - expected result:
  //   - subscriber A not called
  //   - subscriber B called
  // - publish topic A
  // - spin_some()
  // - expected result:
  //   - subscriber A called
  //   - subscriber B not called

  // -------------- rcl objects ---------------------------------------------------------
  rcl_ret_t rc;
  rclc_executor_t executor;
  // initialize executor with 2 handles
  rc = rclc_executor_init(&executor, &this->context, 2, this->allocator_ptr);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcl_reset_error();
  rc = rclc_executor_set_trigger(&executor, rclc_executor_trigger_any, NULL);

  // add subscriptions to executor
  rc = rclc_executor_add_subscription(&executor, &this->sub1, &this->sub1_msg,
      &int32_callback6, ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcutils_reset_error();
  rc = rclc_executor_add_subscription(&executor, &this->sub2, &this->sub2_msg,
      &int32_callback7, ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcutils_reset_error();
  const uint64_t timeout_ns = 10000000;  // 10ms
  // ------------------------- test case setup --------------------------------------------

  // first round
  _reset_result_values_for_trigger_test_case();
  this->pub1_msg.data = 3;
  rc = rcl_publish(&this->pub1, &this->pub1_msg, nullptr);
  EXPECT_EQ(RCL_RET_OK, rc) << " pub1 did not publish!";
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclc_executor_spin_some(&executor, timeout_ns);
  EXPECT_EQ(_cb6_int_value, (unsigned int) 3) << " expected: A called";
  EXPECT_EQ(_cb7_int_value, (unsigned int) 0) << " expected: B not called";
  EXPECT_EQ(_cb6_cnt, (unsigned int) 1);
  EXPECT_EQ(_cb7_cnt, (unsigned int) 0);
  // second round
  this->pub2_msg.data = 7;
  rc = rcl_publish(&this->pub2, &this->pub2_msg, nullptr);
  EXPECT_EQ(RCL_RET_OK, rc) << " pub2 did not publish!";
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclc_executor_spin_some(&executor, timeout_ns);
  EXPECT_EQ(_cb6_int_value, (unsigned int) 3) << " expected: A not called";
  EXPECT_EQ(_cb7_int_value, (unsigned int) 7) << " expected: B called";
  EXPECT_EQ(_cb6_cnt, (unsigned int) 1);
  EXPECT_EQ(_cb7_cnt, (unsigned int) 1);

  // third round
  this->pub1_msg.data = 11;
  _cb6_int_value = 0;
  _cb7_int_value = 0;
  rc = rcl_publish(&this->pub1, &this->pub1_msg, nullptr);
  EXPECT_EQ(RCL_RET_OK, rc) << " pub1 did not publish!";
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclc_executor_spin_some(&executor, timeout_ns);
  EXPECT_EQ(_cb6_int_value, (unsigned int) 11) << " expected: A called";
  EXPECT_EQ(_cb7_int_value, (unsigned int) 0) << " expected: B not called";
  EXPECT_EQ(_cb6_cnt, (unsigned int) 2);
  EXPECT_EQ(_cb7_cnt, (unsigned int) 1);
}


TEST_F(TestDefaultExecutor, trigger_all) {
  // test specification
  // set trigger: trigger_all => execute when A and B received new data
  // setup:
  // - publish topic A
  // - spin_some()
  // - expected result:
  //   - subscriber A not called
  //   - subscriber B not called
  // - publish topic B
  // - spin_some()
  // - expected result:
  //   - subscriber A called
  //   - subscriber B called
  // - publish topic A
  // - spin_some()
  // - expected result:
  //   - subscriber A not called
  //   - subscriber B not called

  // -------------- rcl objects ---------------------------------------------------------
  rcl_ret_t rc;
  rclc_executor_t executor;
  // initialize executor with 2 handles
  rc = rclc_executor_init(&executor, &this->context, 2, this->allocator_ptr);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcl_reset_error();
  rc = rclc_executor_set_trigger(&executor, rclc_executor_trigger_all, NULL);

  // add subscriptions to executor
  rc = rclc_executor_add_subscription(&executor, &this->sub1, &this->sub1_msg,
      &int32_callback6, ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcutils_reset_error();
  rc = rclc_executor_add_subscription(&executor, &this->sub2, &this->sub2_msg,
      &int32_callback7, ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcutils_reset_error();
  const uint64_t timeout_ns = 10000000;  // 10ms
  // ------------------------- test case setup --------------------------------------------

  // first round
  _reset_result_values_for_trigger_test_case();
  this->pub1_msg.data = 3;
  rc = rcl_publish(&this->pub1, &this->pub1_msg, nullptr);
  EXPECT_EQ(RCL_RET_OK, rc) << " pub1 did not publish!";
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclc_executor_spin_some(&executor, timeout_ns);
  EXPECT_EQ(_cb6_int_value, (unsigned int) 0) << " expected: A not called";
  EXPECT_EQ(_cb7_int_value, (unsigned int) 0) << " expected: B not called";
  EXPECT_EQ(_cb6_cnt, (unsigned int) 0);
  EXPECT_EQ(_cb7_cnt, (unsigned int) 0);
  // second round
  this->pub2_msg.data = 7;
  rc = rcl_publish(&this->pub2, &this->pub2_msg, nullptr);
  EXPECT_EQ(RCL_RET_OK, rc) << " pub2 did not publish!";
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclc_executor_spin_some(&executor, timeout_ns);
  EXPECT_EQ(_cb6_int_value, (unsigned int) 3) << " expected: A called";
  EXPECT_EQ(_cb7_int_value, (unsigned int) 7) << " expected: B called";
  EXPECT_EQ(_cb6_cnt, (unsigned int) 1);
  EXPECT_EQ(_cb7_cnt, (unsigned int) 1);

  // third round
  this->pub1_msg.data = 11;
  _cb6_int_value = 0;
  _cb7_int_value = 0;
  rc = rcl_publish(&this->pub1, &this->pub1_msg, nullptr);
  EXPECT_EQ(RCL_RET_OK, rc) << " pub1 did not publish!";
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclc_executor_spin_some(&executor, timeout_ns);
  EXPECT_EQ(_cb6_int_value, (unsigned int) 0) << " expected: A not called";
  EXPECT_EQ(_cb7_int_value, (unsigned int) 0) << " expected: B not called";
  EXPECT_EQ(_cb6_cnt, (unsigned int) 1);
  EXPECT_EQ(_cb7_cnt, (unsigned int) 1);
}

TEST_F(TestDefaultExecutor, trigger_always) {
  // test specification
  // set trigger: trigger_always => execute always
  // additional subscriber with ALWAYS
  // test setup:
  // - subscriber A - ON_NEW_DATA
  // - subscriber B - ALWAYS

  // - spin_some()
  // - expected result:
  //   - subscriber A not called
  //   - subscriber B called
  // - publish topic A
  // - spin_some()
  //   - subscriber A called
  //   - subscriber B called

  // -------------- rcl objects ---------------------------------------------------------
  rcl_ret_t rc;
  rclc_executor_t executor;
  // initialize executor with 2 handles
  rc = rclc_executor_init(&executor, &this->context, 2, this->allocator_ptr);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcl_reset_error();
  rc = rclc_executor_set_trigger(&executor, rclc_executor_trigger_always, NULL);

  // add subscriptions to executor
  rc = rclc_executor_add_subscription(&executor, &this->sub1, &this->sub1_msg,
      &int32_callback6, ON_NEW_DATA);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcutils_reset_error();
  rc = rclc_executor_add_subscription(&executor, &this->sub2, &this->sub2_msg,
      &int32_callback7, ALWAYS);
  EXPECT_EQ(RCL_RET_OK, rc) << rcl_get_error_string().str;
  rcutils_reset_error();
  const uint64_t timeout_ns = 10000000;  // 10ms
  // ------------------------- test case setup --------------------------------------------

  // first round
  _reset_result_values_for_trigger_test_case();
  rclc_executor_spin_some(&executor, timeout_ns);
  EXPECT_EQ(_cb6_int_value, (unsigned int) 0);
  EXPECT_EQ(_cb7_int_value, (unsigned int) 0);
  EXPECT_EQ(_cb6_cnt, (unsigned int) 0) << " expected: A not called";
  EXPECT_EQ(_cb7_cnt, (unsigned int) 1) << " expected: B called";
  // second round
  this->pub1_msg.data = 3;
  rc = rcl_publish(&this->pub1, &this->pub1_msg, nullptr);
  EXPECT_EQ(RCL_RET_OK, rc) << " pub1 did not publish!";
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclc_executor_spin_some(&executor, timeout_ns);
  EXPECT_EQ(_cb6_int_value, (unsigned int) 3) << " expected: A called";
  EXPECT_EQ(_cb7_int_value, (unsigned int) 0) << " expected: B called";
  EXPECT_EQ(_cb6_cnt, (unsigned int) 1) << " expected: A called";
  EXPECT_EQ(_cb7_cnt, (unsigned int) 2) << " expected: B called";
  // third round
  this->pub2_msg.data = 7;
  _cb6_int_value = 0;
  _cb7_int_value = 0;
  rc = rcl_publish(&this->pub2, &this->pub2_msg, nullptr);
  EXPECT_EQ(RCL_RET_OK, rc) << " pub2 did not publish!";
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclc_executor_spin_some(&executor, timeout_ns);
  EXPECT_EQ(_cb6_int_value, (unsigned int) 0) << " expected: A not called";
  EXPECT_EQ(_cb7_int_value, (unsigned int) 7) << " expected: B called";
  EXPECT_EQ(_cb6_cnt, (unsigned int) 1);
  EXPECT_EQ(_cb7_cnt, (unsigned int) 3);
}
*/
