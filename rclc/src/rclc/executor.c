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

#include "rclc/executor.h"
#include <rcutils/time.h>

// Include backport of function 'rcl_wait_set_is_valid' introduced in Foxy
// in case of building for Dashing and Eloquent. This pre-processor macro
// is defined in CMakeLists.txt.
#if defined (USE_RCL_WAIT_SET_IS_VALID_BACKPORT)
#include "rclc/rcl_wait_set_is_valid_backport.h"
#endif


// default timeout for rcl_wait() is 1000ms
#define DEFAULT_WAIT_TIMEOUT_NS 1000000000

// declarations of helper functions
/*
/// get new data from DDS queue for handle i
static
rcl_ret_t
_rclc_check_for_new_data(rclc_executor_t * executor, rcl_wait_set_t * wait_set, size_t i);

/// get new data from DDS queue for handle i
static
rcl_ret_t
_rclc_take_new_data(rclc_executor_t * executor, rcl_wait_set_t * wait_set, size_t i);


/// execute callback of handle i
static
rcl_ret_t
_rclc_execute(rclc_executor_t * executor, rcl_wait_set_t * wait_set, size_t i);

static
rcl_ret_t
_rclc_let_scheduling(rclc_executor_t * executor, rcl_wait_set_t * wait_set);
*/

// rationale: user must create an executor with:
// executor = rclc_executor_get_zero_initialized_executor();
// then handles==NULL or not (e.g. properly initialized)
static
bool
_rclc_executor_is_valid(rclc_executor_t * executor)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(executor, "executor pointer is invalid", return false);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    executor->handles, "handle pointer is invalid", return false);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    executor->allocator, "allocator pointer is invalid", return false);
  if (executor->max_handles == 0) {
    return false;
  }

  return true;
}

// wait_set and rclc_executor_handle_size_t are structs and cannot be statically
// initialized here.
rclc_executor_t
rclc_executor_get_zero_initialized_executor()
{
  static rclc_executor_t null_executor = {
    .context = NULL,
    .handles = NULL,
    .max_handles = 0,
    .index = 0,
    .allocator = NULL,
    .timeout_ns = 0,
    .invocation_time = 0,
    .trigger_function = NULL,
    .trigger_object = NULL
  };
  return null_executor;
}

rcl_ret_t
rclc_executor_init(
  rclc_executor_t * executor,
  rcl_context_t * context,
  const size_t number_of_handles,
  const rcl_allocator_t * allocator)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(executor, "executor is NULL", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(context, "context is NULL", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ALLOCATOR_WITH_MSG(allocator, "allocator is NULL", return RCL_RET_INVALID_ARGUMENT);

  if (number_of_handles == 0) {
    RCL_SET_ERROR_MSG("number_of_handles is 0. Must be larger or equal to 1");
    return RCL_RET_INVALID_ARGUMENT;
  }

  rcl_ret_t ret = RCL_RET_OK;
  (*executor) = rclc_executor_get_zero_initialized_executor();
  executor->context = context;
  executor->max_handles = number_of_handles;
  executor->index = 0;
  executor->wait_set = rcl_get_zero_initialized_wait_set();
  executor->allocator = allocator;
  executor->timeout_ns = DEFAULT_WAIT_TIMEOUT_NS;
  // allocate memory for the array
  executor->handles =
    executor->allocator->allocate(
    (number_of_handles * sizeof(rclc_executor_handle_t)),
    executor->allocator->state);
  if (NULL == executor->handles) {
    RCL_SET_ERROR_MSG("Could not allocate memory for 'handles'.");
    return RCL_RET_BAD_ALLOC;
  }

  // initialize handle
  for (size_t i = 0; i < number_of_handles; i++) {
    rclc_executor_handle_init(&executor->handles[i], number_of_handles);
  }

  // initialize #counts for handle types
  rclc_executor_handle_counters_zero_init(&executor->info);

  // default: trigger_any which corresponds to the ROS2 rclcpp Executor semantics
  //          start processing if any handle has new data/or is ready
  rclc_executor_set_trigger(executor, rclc_executor_trigger_any, NULL);

  // default semantics
  rclc_executor_set_semantics(executor, RCLCPP_EXECUTOR);

  return ret;
}

rcl_ret_t
rclc_executor_set_timeout(rclc_executor_t * executor, const uint64_t timeout_ns)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    executor, "executor is null pointer", return RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;
  if (_rclc_executor_is_valid(executor)) {
    executor->timeout_ns = timeout_ns;
  } else {
    RCL_SET_ERROR_MSG("executor not initialized.");
    return RCL_RET_ERROR;
  }
  return ret;
}

rcl_ret_t
rclc_executor_set_semantics(rclc_executor_t * executor, rclc_executor_semantics_t semantics)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    executor, "executor is null pointer", return RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;
  if (_rclc_executor_is_valid(executor)) {
    executor->data_comm_semantics = semantics;
  } else {
    RCL_SET_ERROR_MSG("executor not initialized.");
    return RCL_RET_ERROR;
  }
  return ret;
}


rcl_ret_t
rclc_executor_fini(rclc_executor_t * executor)
{
  if (_rclc_executor_is_valid(executor)) {
    executor->allocator->deallocate(executor->handles, executor->allocator->state);
    executor->handles = NULL;
    executor->max_handles = 0;
    executor->index = 0;
    rclc_executor_handle_counters_zero_init(&executor->info);

    // free memory of wait_set if it has been initialized
    // calling it with un-initialized wait_set will fail.
    if (rcl_wait_set_is_valid(&executor->wait_set)) {
      rcl_ret_t rc = rcl_wait_set_fini(&executor->wait_set);
      if (rc != RCL_RET_OK) {
        PRINT_RCLC_ERROR(rclc_executor_fini, rcl_wait_set_fini);
      }
    }
    executor->timeout_ns = DEFAULT_WAIT_TIMEOUT_NS;
  } else {
    // Repeated calls to fini or calling fini on a zero initialized executor is ok
  }
  return RCL_RET_OK;
}


rcl_ret_t
rclc_executor_add_subscription(
  rclc_executor_t * executor,
  rcl_subscription_t * subscription,
  void * msg,
  rclc_subscription_callback_t callback,
  rclc_executor_handle_invocation_t invocation)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(subscription, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(msg, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(callback, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;
  // array bound check
  if (executor->index >= executor->max_handles) {
    RCL_SET_ERROR_MSG("Buffer overflow of 'executor->handles'. Increase 'max_handles'");
    return RCL_RET_ERROR;
  }

  // assign data fields
  executor->handles[executor->index].type = SUBSCRIPTION;
  executor->handles[executor->index].subscription = subscription;
  executor->handles[executor->index].data = msg;
  executor->handles[executor->index].subscription_callback = callback;
  executor->handles[executor->index].invocation = invocation;
  executor->handles[executor->index].initialized = true;
  executor->handles[executor->index].callback_context = NULL;

  // increase index of handle array
  executor->index++;

  // invalidate wait_set so that in next spin_some() call the
  // 'executor->wait_set' is updated accordingly
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in rclc_executor_add_subscription.");
      return ret;
    }
  }

  executor->info.number_of_subscriptions++;

  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Added a subscription.");
  return ret;
}


rcl_ret_t
rclc_executor_add_subscription_with_context(
  rclc_executor_t * executor,
  rcl_subscription_t * subscription,
  void * msg,
  rclc_subscription_callback_with_context_t callback,
  void * context,
  rclc_executor_handle_invocation_t invocation)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(subscription, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(msg, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(callback, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;
  // array bound check
  if (executor->index >= executor->max_handles) {
    RCL_SET_ERROR_MSG("Buffer overflow of 'executor->handles'. Increase 'max_handles'");
    return RCL_RET_ERROR;
  }

  // assign data fields
  executor->handles[executor->index].type = SUBSCRIPTION_WITH_CONTEXT;
  executor->handles[executor->index].subscription = subscription;
  executor->handles[executor->index].data = msg;
  executor->handles[executor->index].subscription_callback_with_context = callback;
  executor->handles[executor->index].invocation = invocation;
  executor->handles[executor->index].initialized = true;
  executor->handles[executor->index].callback_context = context;

  // increase index of handle array
  executor->index++;

  // invalidate wait_set so that in next spin_some() call the
  // 'executor->wait_set' is updated accordingly
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in rclc_executor_add_subscription_with_context.");
      return ret;
    }
  }

  executor->info.number_of_subscriptions++;

  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Added a subscription.");
  return ret;
}

rcl_ret_t
rclc_executor_add_timer(
  rclc_executor_t * executor,
  rcl_timer_t * timer)
{
  rcl_ret_t ret = RCL_RET_OK;

  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(timer, RCL_RET_INVALID_ARGUMENT);

  // array bound check
  if (executor->index >= executor->max_handles) {
    rcl_ret_t ret = RCL_RET_ERROR;     // TODO(jst3si) better name : rclc_RET_BUFFER_OVERFLOW
    RCL_SET_ERROR_MSG("Buffer overflow of 'executor->handles'. Increase 'max_handles'");
    return ret;
  }

  // assign data fields
  executor->handles[executor->index].type = TIMER;
  executor->handles[executor->index].timer = timer;
  executor->handles[executor->index].invocation = ON_NEW_DATA;  // i.e. when timer elapsed
  executor->handles[executor->index].initialized = true;
  executor->handles[executor->index].callback_context = NULL;

  // increase index of handle array
  executor->index++;

  // invalidate wait_set so that in next spin_some() call the
  // 'executor->wait_set' is updated accordingly
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in rclc_executor_add_timer function.");
      return ret;
    }
  }
  executor->info.number_of_timers++;
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Added a timer.");
  return ret;
}

rcl_ret_t
rclc_executor_add_client(
  rclc_executor_t * executor,
  rcl_client_t * client,
  void * response_msg,
  rclc_client_callback_t callback)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(client, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(response_msg, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(callback, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;
  // array bound check
  if (executor->index >= executor->max_handles) {
    rcl_ret_t ret = RCL_RET_ERROR;
    RCL_SET_ERROR_MSG("Buffer overflow of 'executor->handles'. Increase 'max_handles'");
    return ret;
  }

  // assign data fields
  executor->handles[executor->index].type = CLIENT;
  executor->handles[executor->index].client = client;
  executor->handles[executor->index].data = response_msg;
  executor->handles[executor->index].client_callback = callback;
  executor->handles[executor->index].invocation = ON_NEW_DATA;  // i.e. when request came in
  executor->handles[executor->index].initialized = true;
  executor->handles[executor->index].callback_context = NULL;

  // increase index of handle array
  executor->index++;

  // invalidate wait_set so that in next spin_some() call the
  // 'executor->wait_set' is updated accordingly
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in rclc_executor_add_client function.");
      return ret;
    }
  }

  executor->info.number_of_clients++;
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Added a client.");
  return ret;
}

rcl_ret_t
rclc_executor_add_client_with_request_id(
  rclc_executor_t * executor,
  rcl_client_t * client,
  void * response_msg,
  rclc_client_callback_with_request_id_t callback)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(client, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(response_msg, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(callback, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;
  // array bound check
  if (executor->index >= executor->max_handles) {
    rcl_ret_t ret = RCL_RET_ERROR;
    RCL_SET_ERROR_MSG("Buffer overflow of 'executor->handles'. Increase 'max_handles'");
    return ret;
  }

  // assign data fields
  executor->handles[executor->index].type = CLIENT_WITH_REQUEST_ID;
  executor->handles[executor->index].client = client;
  executor->handles[executor->index].data = response_msg;
  executor->handles[executor->index].client_callback_with_reqid = callback;
  executor->handles[executor->index].invocation = ON_NEW_DATA;  // i.e. when request came in
  executor->handles[executor->index].initialized = true;
  executor->handles[executor->index].callback_context = NULL;

  // increase index of handle array
  executor->index++;

  // invalidate wait_set so that in next spin_some() call the
  // 'executor->wait_set' is updated accordingly
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in rclc_executor_add_client function.");
      return ret;
    }
  }

  executor->info.number_of_clients++;
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Added a client.");
  return ret;
}

rcl_ret_t
rclc_executor_add_service(
  rclc_executor_t * executor,
  rcl_service_t * service,
  void * request_msg,
  void * response_msg,
  rclc_service_callback_t callback)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(service, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(request_msg, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(response_msg, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(callback, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;
  // array bound check
  if (executor->index >= executor->max_handles) {
    rcl_ret_t ret = RCL_RET_ERROR;
    RCL_SET_ERROR_MSG("Buffer overflow of 'executor->handles'. Increase 'max_handles'");
    return ret;
  }

  // assign data fields
  executor->handles[executor->index].type = SERVICE;
  executor->handles[executor->index].service = service;
  executor->handles[executor->index].data = request_msg;
  // TODO(jst3si) new type with req and resp message in data field.
  executor->handles[executor->index].data_response_msg = response_msg;
  executor->handles[executor->index].service_callback = callback;
  executor->handles[executor->index].invocation = ON_NEW_DATA;  // invoce when request came in
  executor->handles[executor->index].initialized = true;
  executor->handles[executor->index].callback_context = NULL;

  // increase index of handle array
  executor->index++;

  // invalidate wait_set so that in next spin_some() call the
  // 'executor->wait_set' is updated accordingly
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in rclc_executor_add_service function.");
      return ret;
    }
  }

  executor->info.number_of_services++;
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Added a service.");
  return ret;
}

rcl_ret_t
rclc_executor_add_service_with_context(
  rclc_executor_t * executor,
  rcl_service_t * service,
  void * request_msg,
  void * response_msg,
  rclc_service_callback_with_context_t callback,
  void * context)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(service, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(request_msg, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(response_msg, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(callback, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;
  // array bound check
  if (executor->index >= executor->max_handles) {
    rcl_ret_t ret = RCL_RET_ERROR;
    RCL_SET_ERROR_MSG("Buffer overflow of 'executor->handles'. Increase 'max_handles'");
    return ret;
  }

  // assign data fields
  executor->handles[executor->index].type = SERVICE_WITH_CONTEXT;
  executor->handles[executor->index].service = service;
  executor->handles[executor->index].data = request_msg;
  // TODO(jst3si) new type with req and resp message in data field.
  executor->handles[executor->index].data_response_msg = response_msg;
  executor->handles[executor->index].service_callback_with_context = callback;
  executor->handles[executor->index].invocation = ON_NEW_DATA;  // invoce when request came in
  executor->handles[executor->index].initialized = true;
  executor->handles[executor->index].callback_context = context;

  // increase index of handle array
  executor->index++;

  // invalidate wait_set so that in next spin_some() call the
  // 'executor->wait_set' is updated accordingly
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in rclc_executor_add_service function.");
      return ret;
    }
  }

  executor->info.number_of_services++;
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Added a service.");
  return ret;
}

rcl_ret_t
rclc_executor_add_service_with_request_id(
  rclc_executor_t * executor,
  rcl_service_t * service,
  void * request_msg,
  void * response_msg,
  rclc_service_callback_with_request_id_t callback)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(service, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(request_msg, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(response_msg, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(callback, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;
  // array bound check
  if (executor->index >= executor->max_handles) {
    rcl_ret_t ret = RCL_RET_ERROR;
    RCL_SET_ERROR_MSG("Buffer overflow of 'executor->handles'. Increase 'max_handles'");
    return ret;
  }

  // assign data fields
  executor->handles[executor->index].type = SERVICE_WITH_REQUEST_ID;
  executor->handles[executor->index].service = service;
  executor->handles[executor->index].data = request_msg;
  // TODO(jst3si) new type with req and resp message in data field.
  executor->handles[executor->index].data_response_msg = response_msg;
  executor->handles[executor->index].service_callback_with_reqid = callback;
  executor->handles[executor->index].invocation = ON_NEW_DATA;  // invoce when request came in
  executor->handles[executor->index].initialized = true;
  executor->handles[executor->index].callback_context = NULL;

  // increase index of handle array
  executor->index++;

  // invalidate wait_set so that in next spin_some() call the
  // 'executor->wait_set' is updated accordingly
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in rclc_executor_add_service function.");
      return ret;
    }
  }

  executor->info.number_of_services++;
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Added a service.");
  return ret;
}

rcl_ret_t
rclc_executor_add_guard_condition(
  rclc_executor_t * executor,
  rcl_guard_condition_t * gc,
  rclc_gc_callback_t callback)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(gc, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(callback, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;
  // array bound check
  if (executor->index >= executor->max_handles) {
    ret = RCL_RET_ERROR;
    RCL_SET_ERROR_MSG("Buffer overflow of 'executor->handles'. Increase 'max_handles'");
    return ret;
  }

  // assign data fields
  executor->handles[executor->index].type = GUARD_CONDITION;
  executor->handles[executor->index].gc = gc;
  executor->handles[executor->index].gc_callback = callback;
  executor->handles[executor->index].invocation = ON_NEW_DATA;  // invoce when request came in
  executor->handles[executor->index].initialized = true;
  executor->handles[executor->index].callback_context = NULL;

  // increase index of handle array
  executor->index++;

  // invalidate wait_set so that in next spin_some() call the
  // 'executor->wait_set' is updated accordingly
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in rclc_executor_add_guard_condition function.");
      return ret;
    }
  }

  executor->info.number_of_guard_conditions++;
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Added a guard_condition.");
  return ret;
}

static
rcl_ret_t
_rclc_executor_remove_handle(rclc_executor_t * executor, rclc_executor_handle_t * handle)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);

  rcl_ret_t ret = RCL_RET_OK;

  // NULL will be returned by _rclc_executor_find_handle if the handle is not found
  if (NULL == handle) {
    RCL_SET_ERROR_MSG("handle not found in rclc_executor_remove_handle");
    return RCL_RET_ERROR;
  }

  if ((handle >= &executor->handles[executor->index]) ||
    (handle < executor->handles))
  {
    RCL_SET_ERROR_MSG("Handle out of bounds");
    return RCL_RET_ERROR;
  }
  if (0 == executor->index) {
    RCL_SET_ERROR_MSG("No handles to remove");
    return RCL_RET_ERROR;
  }

  // shorten the list of handles without changing the order of remaining handles
  executor->index--;
  for (rclc_executor_handle_t * handle_dest = handle;
    handle_dest < &executor->handles[executor->index];
    handle_dest++)
  {
    *handle_dest = *(handle_dest + 1);
  }
  ret = rclc_executor_handle_init(&executor->handles[executor->index], executor->max_handles);

  // force a refresh of the wait set
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in _rclc_executor_remove_handle.");
      return ret;
    }
  }

  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Removed a handle.");
  return ret;
}

static
rclc_executor_handle_t *
_rclc_executor_find_handle(
  rclc_executor_t * executor,
  const void * rcl_handle)
{
  for (rclc_executor_handle_t * test_handle = executor->handles;
    test_handle < &executor->handles[executor->index];
    test_handle++)
  {
    if (rcl_handle == rclc_executor_handle_get_ptr(test_handle)) {
      return test_handle;
    }
  }
  return NULL;
}


rcl_ret_t
rclc_executor_remove_subscription(
  rclc_executor_t * executor,
  const rcl_subscription_t * subscription)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(subscription, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;

  rclc_executor_handle_t * handle = _rclc_executor_find_handle(executor, subscription);
  ret = _rclc_executor_remove_handle(executor, handle);
  if (RCL_RET_OK != ret) {
    RCL_SET_ERROR_MSG("Failed to remove handle in rclc_executor_remove_subscription.");
    return ret;
  }
  executor->info.number_of_subscriptions--;
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Removed a subscription.");
  return ret;
}

rcl_ret_t
rclc_executor_remove_timer(
  rclc_executor_t * executor,
  const rcl_timer_t * timer)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(timer, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;

  rclc_executor_handle_t * handle = _rclc_executor_find_handle(executor, timer);
  ret = _rclc_executor_remove_handle(executor, handle);
  if (RCL_RET_OK != ret) {
    RCL_SET_ERROR_MSG("Failed to remove handle in rclc_executor_remove_timer.");
    return ret;
  }
  executor->info.number_of_timers--;
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Removed a timer.");
  return ret;
}

rcl_ret_t
rclc_executor_remove_client(
  rclc_executor_t * executor,
  const rcl_client_t * client)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(client, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;

  rclc_executor_handle_t * handle = _rclc_executor_find_handle(executor, client);
  ret = _rclc_executor_remove_handle(executor, handle);
  if (RCL_RET_OK != ret) {
    RCL_SET_ERROR_MSG("Failed to remove handle in rclc_executor_remove_client.");
    return ret;
  }
  executor->info.number_of_clients--;
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Removed a client.");
  return ret;
}

rcl_ret_t
rclc_executor_remove_service(
  rclc_executor_t * executor,
  const rcl_service_t * service)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(service, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;

  rclc_executor_handle_t * handle = _rclc_executor_find_handle(executor, service);
  ret = _rclc_executor_remove_handle(executor, handle);
  if (RCL_RET_OK != ret) {
    RCL_SET_ERROR_MSG("Failed to remove handle in rclc_executor_remove_service.");
    return ret;
  }
  executor->info.number_of_services--;
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Removed a service.");
  return ret;
}


rcl_ret_t
rclc_executor_remove_guard_condition(
  rclc_executor_t * executor,
  const rcl_guard_condition_t * guard_condition)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(guard_condition, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;

  rclc_executor_handle_t * handle = _rclc_executor_find_handle(executor, guard_condition);
  ret = _rclc_executor_remove_handle(executor, handle);
  if (RCL_RET_OK != ret) {
    RCL_SET_ERROR_MSG("Failed to remove handle in rclc_executor_remove_guard_condition.");
    return ret;
  }
  executor->info.number_of_guard_conditions--;
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Removed a guard condition.");
  return ret;
}


/***
 * operates on handle executor->handles[i]
 * - evaluates the status bit in the wait_set for this handle
 * - if new message is available or timer is ready, assigns executor->handles[i].data_available = true
 */

// todo refactor parameters: (rclc_executor_handle_t *, rcl_wait_set_t * wait_set)

static
rcl_ret_t
_rclc_check_for_new_data(rclc_executor_handle_t * handle, rcl_wait_set_t * wait_set)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(handle, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(wait_set, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t rc = RCL_RET_OK;

  switch (handle->type) {
    case SUBSCRIPTION:
    case SUBSCRIPTION_WITH_CONTEXT:
      if (wait_set->subscriptions[handle->index]) {
        handle->data_available = true;
      }
      break;

    case TIMER:
      // case TIMER_WITH_CONTEXT:
      if (wait_set->timers[handle->index]) {
        bool timer_is_ready = false;
        rc = rcl_timer_is_ready(handle->timer, &timer_is_ready);
        if (rc != RCL_RET_OK) {
          PRINT_RCLC_ERROR(rclc_read_input_data, rcl_timer_is_ready);
          return rc;
        }
        // actually this is a unnecessary check: if wait_set.timers[i] is true, then also
        // rcl_timer_is_ready() should return true.
        if (timer_is_ready) {
          handle->data_available = true;
        } else {
          // this should never happen
          PRINT_RCLC_ERROR(rclc_read_input_data, rcl_timer_should_be_ready);
          return RCL_RET_ERROR;
        }
      }
      break;

    case SERVICE:
    case SERVICE_WITH_REQUEST_ID:
    case SERVICE_WITH_CONTEXT:
      if (wait_set->services[handle->index]) {
        handle->data_available = true;
      }
      break;

    case CLIENT:
    case CLIENT_WITH_REQUEST_ID:
      // case CLIENT_WITH_CONTEXT:
      if (wait_set->clients[handle->index]) {
        handle->data_available = true;
      }
      break;

    case GUARD_CONDITION:
      // case GUARD_CONDITION_WITH_CONTEXT:
      if (wait_set->guard_conditions[handle->index]) {
        handle->data_available = true;
      }
      break;

    default:
      RCUTILS_LOG_DEBUG_NAMED(
        ROS_PACKAGE_NAME, "Error in _rclc_check_for_new_data:wait_set unknwon handle type: %d",
        handle->type);
      return RCL_RET_ERROR;
  }    // switch-case
  return rc;
}

// call rcl_take for subscription
// todo change function signature (rclc_executor_handle_t * handle, rcl_wait_set_t * wait_set)

static
rcl_ret_t
_rclc_take_new_data(rclc_executor_handle_t * handle, rcl_wait_set_t * wait_set)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(handle, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(wait_set, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t rc = RCL_RET_OK;

  switch (handle->type) {
    case SUBSCRIPTION:
    case SUBSCRIPTION_WITH_CONTEXT:
      if (wait_set->subscriptions[handle->index]) {
        rmw_message_info_t messageInfo;
        rc = rcl_take(
          handle->subscription, handle->data, &messageInfo,
          NULL);
        if (rc != RCL_RET_OK) {
          // rcl_take might return this error even with successfull rcl_wait
          if (rc != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
            PRINT_RCLC_ERROR(rclc_take_new_data, rcl_take);
            RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME, "Error number: %d", rc);
          }
          // invalidate that data is available, because rcl_take failed
          if (rc == RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
            handle->data_available = false;
          }
          return rc;
        }
      }
      break;

    case TIMER:
      // case TIMER_WITH_CONTEXT:
      // nothing to do
      // notification, that timer is ready already done in _rclc_evaluate_data_availability()
      break;

    case SERVICE:
    case SERVICE_WITH_REQUEST_ID:
    case SERVICE_WITH_CONTEXT:
      if (wait_set->services[handle->index]) {
        rc = rcl_take_request(
          handle->service, &handle->req_id, handle->data);
        if (rc != RCL_RET_OK) {
          // rcl_take_request might return this error even with successfull rcl_wait
          if (rc != RCL_RET_SERVICE_TAKE_FAILED) {
            PRINT_RCLC_ERROR(rclc_take_new_data, rcl_take_request);
            RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME, "Error number: %d", rc);
          }
          // invalidate that data is available, because rcl_take failed
          if (rc == RCL_RET_SERVICE_TAKE_FAILED) {
            handle->data_available = false;
          }
          return rc;
        }
      }
      break;

    case CLIENT:
    case CLIENT_WITH_REQUEST_ID:
      // case CLIENT_WITH_CONTEXT:
      if (wait_set->clients[handle->index]) {
        rc = rcl_take_response(
          handle->client, &handle->req_id, handle->data);
        if (rc != RCL_RET_OK) {
          // rcl_take_response might return this error even with successfull rcl_wait
          if (rc != RCL_RET_CLIENT_TAKE_FAILED) {
            PRINT_RCLC_ERROR(rclc_take_new_data, rcl_take_response);
            RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME, "Error number: %d", rc);
          }
          return rc;
        }
      }
      break;

    case GUARD_CONDITION:
      // case GUARD_CONDITION_WITH_CONTEXT:
      // nothing to do
      break;

    default:
      RCUTILS_LOG_DEBUG_NAMED(
        ROS_PACKAGE_NAME, "Error in _rclc_take_new_data:wait_set unknwon handle type: %d",
        handle->type);
      return RCL_RET_ERROR;
  }    // switch-case
  return rc;
}

/***
 * operates on executor->handles[i] object
 * - calls every callback of each object depending on its type
 */

// todo change parametes (rclc_executor_handle_t * handle)

static
rcl_ret_t
_rclc_execute(rclc_executor_handle_t * handle)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(handle, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t rc = RCL_RET_OK;
  bool invoke_callback = false;

  // determine, if callback shall be called
  if (handle->invocation == ON_NEW_DATA &&
    handle->data_available == true)
  {
    invoke_callback = true;
  }

  if (handle->invocation == ALWAYS) {
    invoke_callback = true;
  }

  // execute callback
  if (invoke_callback) {
    switch (handle->type) {
      case SUBSCRIPTION:
        if (handle->data_available) {
          handle->subscription_callback(handle->data);
        } else {
          handle->subscription_callback(NULL);
        }
        break;

      case SUBSCRIPTION_WITH_CONTEXT:
        if (handle->data_available) {
          handle->subscription_callback_with_context(
            handle->data,
            handle->callback_context);
        } else {
          handle->subscription_callback_with_context(
            NULL,
            handle->callback_context);
        }
        break;

      case TIMER:
        // case TIMER_WITH_CONTEXT:
        rc = rcl_timer_call(handle->timer);
        if (rc != RCL_RET_OK) {
          PRINT_RCLC_ERROR(rclc_execute, rcl_timer_call);
          return rc;
        }
        break;

      case SERVICE:
      case SERVICE_WITH_REQUEST_ID:
      case SERVICE_WITH_CONTEXT:
        // differentiate user-side service types
        switch (handle->type) {
          case SERVICE:
            handle->service_callback(
              handle->data,
              handle->data_response_msg);
            break;
          case SERVICE_WITH_REQUEST_ID:
            handle->service_callback_with_reqid(
              handle->data,
              &handle->req_id,
              handle->data_response_msg);
            break;
          case SERVICE_WITH_CONTEXT:
            handle->service_callback_with_context(
              handle->data,
              handle->data_response_msg,
              handle->callback_context);
            break;
          default:
            break;  // flow can't reach here
        }
        // handle rcl-side services
        rc = rcl_send_response(handle->service, &handle->req_id, handle->data_response_msg);
        if (rc != RCL_RET_OK) {
          PRINT_RCLC_ERROR(rclc_execute, rcl_send_response);
          return rc;
        }
        break;

      case CLIENT:
        handle->client_callback(handle->data);
        break;

      case CLIENT_WITH_REQUEST_ID:
        handle->client_callback_with_reqid(handle->data, &handle->req_id);
        break;

      // case CLIENT_WITH_CONTEXT:   //TODO
      //   break;

      case GUARD_CONDITION:
        handle->gc_callback();
        break;

      // case GUARD_CONDITION_WITH_CONTEXT:  //TODO
      //   break;

      default:
        RCUTILS_LOG_DEBUG_NAMED(
          ROS_PACKAGE_NAME, "Error in _rclc_execute: unknwon handle type: %d",
          handle->type);
        return RCL_RET_ERROR;
    }   // switch-case
  }

  // corresponding callback of this handle has been called => reset data_available flag here
  // (see also comment in _rclc_read_input_data() function)
  if (handle->data_available == true) {
    handle->data_available = false;
  }
  return rc;
}


static
rcl_ret_t
_rclc_default_scheduling(rclc_executor_t * executor)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t rc = RCL_RET_OK;

  for (size_t i = 0; (i < executor->max_handles && executor->handles[i].initialized); i++) {
    rc = _rclc_check_for_new_data(&executor->handles[i], &executor->wait_set);
    if ((rc != RCL_RET_OK) && (rc != RCL_RET_SUBSCRIPTION_TAKE_FAILED)) {
      return rc;
    }
  }
  // if the trigger condition is fullfilled, fetch data and execute
  if (executor->trigger_function(
      executor->handles, executor->max_handles,
      executor->trigger_object))
  {
    // take new input data from DDS-queue and execute the corresponding callback of the handle
    for (size_t i = 0; (i < executor->max_handles && executor->handles[i].initialized); i++) {
      rc = _rclc_take_new_data(&executor->handles[i], &executor->wait_set);
      if ((rc != RCL_RET_OK) && (rc != RCL_RET_SUBSCRIPTION_TAKE_FAILED) &&
        (rc != RCL_RET_SERVICE_TAKE_FAILED))
      {
        return rc;
      }
      rc = _rclc_execute(&executor->handles[i]);
      if (rc != RCL_RET_OK) {
        return rc;
      }
    }
  }
  return rc;
}

static
rcl_ret_t
_rclc_let_scheduling(rclc_executor_t * executor)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t rc = RCL_RET_OK;

  // logical execution time
  // 1. read all input
  // 2. process
  // 3. write data (*) data is written not at the end of all callbacks, but it will not be
  //    processed by the callbacks 'in this round' because all input data is read in the
  //    beginning and the incoming messages were copied.

  // step 0: check for available input data from DDS queue
  // complexity: O(n) where n denotes the number of handles
  for (size_t i = 0; (i < executor->max_handles && executor->handles[i].initialized); i++) {
    rc = _rclc_check_for_new_data(&executor->handles[i], &executor->wait_set);
    if ((rc != RCL_RET_OK) && (rc != RCL_RET_SUBSCRIPTION_TAKE_FAILED)) {
      return rc;
    }
  }

  // if the trigger condition is fullfilled, fetch data and execute
  // complexity: O(n) where n denotes the number of handles
  if (executor->trigger_function(
      executor->handles, executor->max_handles,
      executor->trigger_object))
  {
    // step 1: read input data
    for (size_t i = 0; (i < executor->max_handles && executor->handles[i].initialized); i++) {
      rc = _rclc_take_new_data(&executor->handles[i], &executor->wait_set);
      if ((rc != RCL_RET_OK) && (rc != RCL_RET_SUBSCRIPTION_TAKE_FAILED)) {
        return rc;
      }
    }

    // step 2:  process (execute)
    for (size_t i = 0; (i < executor->max_handles && executor->handles[i].initialized); i++) {
      rc = _rclc_execute(&executor->handles[i]);
      if (rc != RCL_RET_OK) {
        return rc;
      }
    }
  }
  return rc;
}

rcl_ret_t
rclc_executor_prepare(rclc_executor_t * executor)
{
  rcl_ret_t rc = RCL_RET_OK;
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "executor_prepare");

  // initialize wait_set if
  // (1) this is the first invocation of executor_spin_some()
  // (2) executor_add_timer() or executor_add_subscription() has been called.
  //     i.e. a new timer or subscription has been added to the Executor.
  if (!rcl_wait_set_is_valid(&executor->wait_set)) {
    // calling wait_set on zero_initialized wait_set multiple times is ok.
    rcl_ret_t rc = rcl_wait_set_fini(&executor->wait_set);
    if (rc != RCL_RET_OK) {
      PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_wait_set_fini);
    }
    // initialize wait_set
    executor->wait_set = rcl_get_zero_initialized_wait_set();
    // create sufficient memory space for all handles in the wait_set
    rc = rcl_wait_set_init(
      &executor->wait_set, executor->info.number_of_subscriptions,
      executor->info.number_of_guard_conditions, executor->info.number_of_timers,
      executor->info.number_of_clients, executor->info.number_of_services,
      executor->info.number_of_events,
      executor->context,
      *executor->allocator);

    if (rc != RCL_RET_OK) {
      PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_wait_set_init);
      return rc;
    }
  }

  return rc;
}

rcl_ret_t
rclc_executor_spin_some(rclc_executor_t * executor, const uint64_t timeout_ns)
{
  rcl_ret_t rc = RCL_RET_OK;
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "spin_some");

  if (!rcl_context_is_valid(executor->context)) {
    PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_context_not_valid);
    return RCL_RET_ERROR;
  }

  rclc_executor_prepare(executor);

  // set rmw fields to NULL
  rc = rcl_wait_set_clear(&executor->wait_set);
  if (rc != RCL_RET_OK) {
    PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_wait_set_clear);
    return rc;
  }

  // (jst3si) put in a sub-function - for improved readability
  // add handles to wait_set
  for (size_t i = 0; (i < executor->max_handles && executor->handles[i].initialized); i++) {
    RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "wait_set_add_* %d", executor->handles[i].type);
    switch (executor->handles[i].type) {
      case SUBSCRIPTION:
      case SUBSCRIPTION_WITH_CONTEXT:
        // add subscription to wait_set and save index
        rc = rcl_wait_set_add_subscription(
          &executor->wait_set, executor->handles[i].subscription,
          &executor->handles[i].index);
        if (rc == RCL_RET_OK) {
          RCUTILS_LOG_DEBUG_NAMED(
            ROS_PACKAGE_NAME,
            "Subscription added to wait_set_subscription[%ld]",
            executor->handles[i].index);
        } else {
          PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_wait_set_add_subscription);
          return rc;
        }
        break;

      case TIMER:
        // case TIMER_WITH_CONTEXT:
        // add timer to wait_set and save index
        rc = rcl_wait_set_add_timer(
          &executor->wait_set, executor->handles[i].timer,
          &executor->handles[i].index);
        if (rc == RCL_RET_OK) {
          RCUTILS_LOG_DEBUG_NAMED(
            ROS_PACKAGE_NAME, "Timer added to wait_set_timers[%ld]",
            executor->handles[i].index);
        } else {
          PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_wait_set_add_timer);
          return rc;
        }
        break;

      case SERVICE:
      case SERVICE_WITH_REQUEST_ID:
      case SERVICE_WITH_CONTEXT:
        // add service to wait_set and save index
        rc = rcl_wait_set_add_service(
          &executor->wait_set, executor->handles[i].service,
          &executor->handles[i].index);
        if (rc == RCL_RET_OK) {
          RCUTILS_LOG_DEBUG_NAMED(
            ROS_PACKAGE_NAME, "Service added to wait_set_service[%ld]",
            executor->handles[i].index);
        } else {
          PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_wait_set_add_service);
          return rc;
        }
        break;


      case CLIENT:
      case CLIENT_WITH_REQUEST_ID:
        // case CLIENT_WITH_CONTEXT:
        // add client to wait_set and save index
        rc = rcl_wait_set_add_client(
          &executor->wait_set, executor->handles[i].client,
          &executor->handles[i].index);
        if (rc == RCL_RET_OK) {
          RCUTILS_LOG_DEBUG_NAMED(
            ROS_PACKAGE_NAME, "Client added to wait_set_client[%ld]",
            executor->handles[i].index);
        } else {
          PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_wait_set_add_client);
          return rc;
        }
        break;

      case GUARD_CONDITION:
        // case GUARD_CONDITION_WITH_CONTEXT:
        // add guard_condition to wait_set and save index
        rc = rcl_wait_set_add_guard_condition(
          &executor->wait_set, executor->handles[i].gc,
          &executor->handles[i].index);
        if (rc == RCL_RET_OK) {
          RCUTILS_LOG_DEBUG_NAMED(
            ROS_PACKAGE_NAME, "Guard_condition added to wait_set_client[%ld]",
            executor->handles[i].index);
        } else {
          PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_wait_set_add_guard_condition);
          return rc;
        }
        break;


      default:
        RCUTILS_LOG_DEBUG_NAMED(
          ROS_PACKAGE_NAME, "Error: unknown handle type: %d",
          executor->handles[i].type);
        PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_wait_set_add_unknown_handle);
        return RCL_RET_ERROR;
    }
  }

  // wait up to 'timeout_ns' to receive notification about which handles reveived
  // new data from DDS queue.
  rc = rcl_wait(&executor->wait_set, timeout_ns);
  RCLC_UNUSED(rc);

  // based on semantics process input data
  switch (executor->data_comm_semantics) {
    case LET:
      rc = _rclc_let_scheduling(executor);
      break;
    case RCLCPP_EXECUTOR:
      rc = _rclc_default_scheduling(executor);
      break;
    default:
      PRINT_RCLC_ERROR(rclc_executor_spin_some, unknown_semantics);
      return RCL_RET_ERROR;
  }

  return rc;
}

rcl_ret_t
rclc_executor_spin(rclc_executor_t * executor)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;
  printf("INFO: rcl_wait timeout %ld ms\n", ((executor->timeout_ns / 1000) / 1000));
  while (true) {
    ret = rclc_executor_spin_some(executor, executor->timeout_ns);
    if (!((ret == RCL_RET_OK) || (ret == RCL_RET_TIMEOUT))) {
      RCL_SET_ERROR_MSG("rclc_executor_spin_some error");
      return ret;
    }
  }
  return ret;
}


/*
 The reason for splitting this function up, is to be able to write a unit test.
 The spin_period is an endless loop, therefore it is not possible to stop after x iterations.
 rclc_executor_spin_period_ implements one iteration and the function
 rclc_executor_spin_period implements the endless while-loop. The unit test covers only
 rclc_executor_spin_period_.
*/
rcl_ret_t
rclc_executor_spin_one_period(rclc_executor_t * executor, const uint64_t period)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;
  rcutils_time_point_value_t end_time_point;
  rcutils_duration_value_t sleep_time;

  if (executor->invocation_time == 0) {
    ret = rcutils_system_time_now(&executor->invocation_time);
    RCLC_UNUSED(ret);
  }
  ret = rclc_executor_spin_some(executor, executor->timeout_ns);
  if (!((ret == RCL_RET_OK) || (ret == RCL_RET_TIMEOUT))) {
    RCL_SET_ERROR_MSG("rclc_executor_spin_some error");
    return ret;
  }
  // sleep UNTIL next invocation time point = invocation_time + period
  ret = rcutils_system_time_now(&end_time_point);
  sleep_time = (executor->invocation_time + period) - end_time_point;
  if (sleep_time > 0) {
    rclc_sleep_ms(sleep_time / 1000000);
  }
  executor->invocation_time += period;
  return ret;
}

rcl_ret_t
rclc_executor_spin_period(rclc_executor_t * executor, const uint64_t period)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret;
  while (true) {
    ret = rclc_executor_spin_one_period(executor, period);
    if (!((ret == RCL_RET_OK) || (ret == RCL_RET_TIMEOUT))) {
      RCL_SET_ERROR_MSG("rclc_executor_spin_one_period error");
      return ret;
    }
  }
  // never get here
  return RCL_RET_OK;
}

rcl_ret_t
rclc_executor_set_trigger(
  rclc_executor_t * executor,
  rclc_executor_trigger_t trigger_function,
  void * trigger_object)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  executor->trigger_function = trigger_function;
  executor->trigger_object = trigger_object;
  return RCL_RET_OK;
}

bool rclc_executor_trigger_all(rclc_executor_handle_t * handles, unsigned int size, void * obj)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(handles, "handles is NULL", return false);
  // did not use (i<size && handles[i].initialized) as loop-condition
  // because for last index i==size this would result in out-of-bound access
  RCLC_UNUSED(obj);
  for (unsigned int i = 0; i < size; i++) {
    if (handles[i].initialized) {
      if (handles[i].data_available == false) {
        return false;
      }
    } else {
      break;
    }
  }
  return true;
}

bool rclc_executor_trigger_any(rclc_executor_handle_t * handles, unsigned int size, void * obj)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(handles, "handles is NULL", return false);
  RCLC_UNUSED(obj);
  // did not use (i<size && handles[i].initialized) as loop-condition
  // because for last index i==size this would result in out-of-bound access
  for (unsigned int i = 0; i < size; i++) {
    if (handles[i].initialized) {
      if (handles[i].data_available == true) {
        return true;
      }
    } else {
      break;
    }
  }
  return false;
}

bool rclc_executor_trigger_one(rclc_executor_handle_t * handles, unsigned int size, void * obj)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(handles, "handles is NULL", return false);
  // did not use (i<size && handles[i].initialized) as loop-condition
  // because for last index i==size this would result in out-of-bound access
  for (unsigned int i = 0; i < size; i++) {
    if (handles[i].initialized) {
      if (handles[i].data_available == true) {
        void * handle_obj_ptr = rclc_executor_handle_get_ptr(&handles[i]);
        if (NULL == handle_obj_ptr) {
          // rclc_executor_handle_get_ptr returns null for unsupported types
          return false;
        }
        if (obj == handle_obj_ptr) {
          return true;
        }
      }
    } else {
      break;
    }
  }
  return false;
}

bool rclc_executor_trigger_always(rclc_executor_handle_t * handles, unsigned int size, void * obj)
{
  RCLC_UNUSED(handles);
  RCLC_UNUSED(size);
  RCLC_UNUSED(obj);
  return true;
}
