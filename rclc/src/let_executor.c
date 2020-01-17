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

#include "rclc/let_executor.h"
#include <unistd.h>
#include "rcutils/time.h"

// default timeout for rcl_wait() is 100ms
#define DEFAULT_WAIT_TIMEOUT_MS 100000000

// declarations of helper functions
/*
/// get new data from DDS queue for handle i
static
rcl_ret_t
_rclc_check_for_new_data(rclc_let_executor_t * executor, rcl_wait_set_t * wait_set, size_t i);

/// get new data from DDS queue for handle i
static
rcl_ret_t
_rclc_take_new_data(rclc_let_executor_t * executor, rcl_wait_set_t * wait_set, size_t i);


/// execute callback of handle i
static
rcl_ret_t
_rclc_execute(rclc_let_executor_t * executor, rcl_wait_set_t * wait_set, size_t i);

static
rcl_ret_t
_rclc_let_scheduling(rclc_let_executor_t * executor, rcl_wait_set_t * wait_set);
*/

// rationale: user must create an executor with:
// executor = rclc_let_executor_get_zero_initialized_executor();
// then handles==NULL or not (e.g. properly initialized)
static
bool
_rclc_let_executor_is_valid(rclc_let_executor_t * executor)
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
rclc_let_executor_t
rclc_let_executor_get_zero_initialized_executor()
{
  static rclc_let_executor_t null_executor = {
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
rclc_let_executor_init(
  rclc_let_executor_t * executor,
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
  executor->context = context;
  executor->max_handles = number_of_handles;
  executor->index = 0;
  executor->wait_set = rcl_get_zero_initialized_wait_set();
  executor->allocator = allocator;
  executor->timeout_ns = DEFAULT_WAIT_TIMEOUT_MS;
  // allocate memory for the array
  executor->handles =
    executor->allocator->allocate( (number_of_handles * sizeof(rclc_executor_handle_t)),
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

  return ret;
}

rcl_ret_t
rclc_let_executor_set_timeout(rclc_let_executor_t * executor, const uint64_t timeout_ns)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    executor, "executor is null pointer", return RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;
  if (_rclc_let_executor_is_valid(executor)) {
    executor->timeout_ns = timeout_ns;
  } else {
    RCL_SET_ERROR_MSG("executor not initialized.");
    return RCL_RET_ERROR;
  }
  return ret;
}

rcl_ret_t
rclc_let_executor_fini(rclc_let_executor_t * executor)
{
  if (_rclc_let_executor_is_valid(executor)) {
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
        PRINT_RCLC_ERROR(rclc_let_executor_fini, rcl_wait_set_fini);
      }
    }
    executor->timeout_ns = DEFAULT_WAIT_TIMEOUT_MS;
  } else {
    // Repeated calls to fini or calling fini on a zero initialized executor is ok
  }
  return RCL_RET_OK;
}


rcl_ret_t
rclc_let_executor_add_subscription(
  rclc_let_executor_t * executor,
  rcl_subscription_t * subscription,
  void * msg,
  rclc_callback_t callback,
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
  executor->handles[executor->index].callback = callback;
  executor->handles[executor->index].invocation = invocation;
  executor->handles[executor->index].initialized = true;

  // increase index of handle array
  executor->index++;

  // invalidate wait_set so that in next spin_some() call the
  // 'executor->wait_set' is updated accordingly
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in rclc_let_executor_add_subscription.");
      return ret;
    }
  }

  executor->info.number_of_subscriptions++;

  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Added a subscription.");
  return ret;
}


rcl_ret_t
rclc_let_executor_add_timer(
  rclc_let_executor_t * executor,
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

  // increase index of handle array
  executor->index++;

  // invalidate wait_set so that in next spin_some() call the
  // 'executor->wait_set' is updated accordingly
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in rclc_let_executor_add_timer function.");
      return ret;
    }
  }

  executor->info.number_of_timers++;

  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Added a timer.");
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
      if (wait_set->subscriptions[handle->index]) {
        handle->data_available = true;
      }
      break;

    case TIMER:
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

    default:
      RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Error:wait_set unknwon handle type: %d",
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
      if (wait_set->subscriptions[handle->index]) {
        rmw_message_info_t messageInfo;
        rc = rcl_take(handle->subscription, handle->data, &messageInfo,
            NULL);
        if (rc != RCL_RET_OK) {
          // it is documented, that rcl_take might return this error with successfull rcl_wait
          if (rc != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
            PRINT_RCLC_ERROR(rclc_read_input_data, rcl_take);
            RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME, "Error number: %d", rc);
          }
          return rc;
        }
      }
      break;

    case TIMER:
      // nothing to do
      // notification, that timer is ready already done in _rclc_evaluate_data_availability()
      break;

    default:
      RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Error:wait_set unknwon handle type: %d",
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

  // printf("execute: handles[%d] :  %d\n", i, invoke_callback);  // debug(jst3si)
  // execute callback
  if (invoke_callback) {
    switch (handle->type) {
      case SUBSCRIPTION:
        handle->callback(handle->data);
        break;

      case TIMER:
        rc = rcl_timer_call(handle->timer);
        if (rc != RCL_RET_OK) {
          PRINT_RCLC_ERROR(rclc_execute, rcl_timer_call);
          return rc;
        }
        break;

      default:
        RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Execute callback: unknwon handle type: %d",
          handle->type);
        return RCL_RET_ERROR;
    }    // switch-case
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
_rclc_let_scheduling(rclc_let_executor_t * executor)
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
  if (executor->trigger_function(executor->handles, executor->max_handles, executor->trigger_object)) {

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
rclc_let_executor_spin_some(rclc_let_executor_t * executor, const uint64_t timeout_ns)
{
  rcl_ret_t rc = RCL_RET_OK;
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "spin_some");

  // initialize wait_set if
  // (1) this is the first invocation of let_executor_spin_some()
  // (2) let_executor_add_timer() or let_executor_add_subscription() has been called.
  //     i.e. a new timer or subscription has been added to the Executor.
  if (!rcl_wait_set_is_valid(&executor->wait_set)) {
    // calling wait_set on zero_initialized wait_set multiple times is ok.
    rcl_ret_t rc = rcl_wait_set_fini(&executor->wait_set);
    if (rc != RCL_RET_OK) {
      PRINT_RCLC_ERROR(rclc_let_executor_spin_some, rcl_wait_set_fini);
    }
    // initialize wait_set
    executor->wait_set = rcl_get_zero_initialized_wait_set();
    // create sufficient memory space for all handles in the wait_set
    rc = rcl_wait_set_init(&executor->wait_set, executor->info.number_of_subscriptions,
        executor->info.number_of_guard_conditions, executor->info.number_of_timers,
        executor->info.number_of_clients, executor->info.number_of_services,
        executor->info.number_of_events,
        executor->context, rcl_get_default_allocator());
    if (rc != RCL_RET_OK) {
      PRINT_RCLC_ERROR(rclc_let_executor_spin_some, rcl_wait_set_init);
      return rc;
    }
  }

  // set rmw fields to NULL
  rc = rcl_wait_set_clear(&executor->wait_set);
  if (rc != RCL_RET_OK) {
    PRINT_RCLC_ERROR(rclc_let_executor_spin_some, rcl_wait_set_clear);
    return rc;
  }

  // (jst3si) put in a sub-function - for improved readability
  // add handles to wait_set
  for (size_t i = 0; (i < executor->max_handles && executor->handles[i].initialized); i++) {
    RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "wait_set_add_* %d", executor->handles[i].type);
    switch (executor->handles[i].type) {
      case SUBSCRIPTION:
        // add subscription to wait_set and save index
        rc = rcl_wait_set_add_subscription(&executor->wait_set, executor->handles[i].subscription,
            &executor->handles[i].index);
        if (rc != RCL_RET_OK) {
          PRINT_RCLC_ERROR(rclc_let_executor_spin_some, rcl_wait_set_add_subscription);
          return rc;
        } else {
          RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME,
            "Subscription added to wait_set_subscription[%ld]",
            executor->handles[i].index);
        }
        break;

      case TIMER:
        // add timer to wait_set and save index
        rc = rcl_wait_set_add_timer(&executor->wait_set, executor->handles[i].timer,
            &executor->handles[i].index);
        if (rc != RCL_RET_OK) {
          PRINT_RCLC_ERROR(rclc_let_executor_spin_some, rcl_wait_set_add_timer);
          return rc;
        } else {
          RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Timer added to wait_set_timers[%ld]",
            executor->handles[i].index);
        }
        break;

      default:
        RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Error: unknown handle type: %d",
          executor->handles[i].type);
        PRINT_RCLC_ERROR(rclc_let_executor_spin_some, rcl_wait_set_unknown_handle);
        return RCL_RET_ERROR;
    }
  }

  // wait up to 'timeout_ns' to receive notification about which handles reveived
  // new data from DDS queue.
  rc = rcl_wait(&executor->wait_set, timeout_ns);

  rc = _rclc_let_scheduling(executor);

  if (rc != RCL_RET_OK) {
    // PRINT_RCLC_ERROR has already been called in _rclc_let_scheduling()
    return rc;
  }

  return rc;
}

rcl_ret_t
rclc_let_executor_spin(rclc_let_executor_t * executor)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;
  printf("INFO: rcl_wait timeout %ld ms\n", ((executor->timeout_ns / 1000) / 1000));
  while (rcl_context_is_valid(executor->context) ) {
    ret = rclc_let_executor_spin_some(executor, executor->timeout_ns);
    if (!((ret == RCL_RET_OK) || (ret == RCL_RET_TIMEOUT))) {
      RCL_SET_ERROR_MSG("rclc_let_executor_spin_some error");
      return ret;
    }
  }
  return ret;
}


/*
 The reason for splitting this function up, is to be able to write a unit test.
 The spin_period is an endless loop, therefore it is not possible to stop after x iterations.
 rclc_let_executor_spin_period_ implements one iteration and the function
 rclc_let_executor_spin_period implements the endless while-loop. The unit test covers only
 rclc_let_executor_spin_period_.
*/
rcl_ret_t
rclc_let_executor_spin_one_period(rclc_let_executor_t * executor, const uint64_t period)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;
  rcutils_time_point_value_t end_time_point;
  rcutils_duration_value_t sleep_time;

  if (executor->invocation_time == 0) {
    ret = rcutils_system_time_now(&executor->invocation_time);
  }
  ret = rclc_let_executor_spin_some(executor, executor->timeout_ns);
  if (!((ret == RCL_RET_OK) || (ret == RCL_RET_TIMEOUT))) {
    RCL_SET_ERROR_MSG("rclc_let_executor_spin_some error");
    return ret;
  }
  // sleep until invocation_time plus period
  ret = rcutils_system_time_now(&end_time_point);
  sleep_time = (executor->invocation_time + period) - end_time_point;
  if (sleep_time > 0) {
    usleep(sleep_time / 1000);
  }
  executor->invocation_time += period;
  return ret;
}

rcl_ret_t
rclc_let_executor_spin_period(rclc_let_executor_t * executor, const uint64_t period)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  while (rcl_context_is_valid(executor->context) ) {
    rclc_let_executor_spin_one_period(executor, period);
  }
  // never get here
  return RCL_RET_OK;
}

rcl_ret_t
rclc_let_executor_set_trigger(
  rclc_let_executor_t * executor,
  rclc_let_executor_trigger_t trigger_function,
  void * trigger_object) {
   RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
   executor->trigger_function = trigger_function;
   executor->trigger_object = trigger_object;
   return RCL_RET_OK;
}

bool rclc_let_executor_trigger_all(rclc_executor_handle_t * handles, unsigned int size, void * obj) {
  RCL_CHECK_FOR_NULL_WITH_MSG(handles, "handles is NULL", return false);
  // did not use (i<size && handles[i].initialized) as loop-condition
  // because for last index i==size this would result in out-of-bound access
  for(unsigned int i=0; i<size; i++) {
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

bool rclc_let_executor_trigger_any(rclc_executor_handle_t * handles, unsigned int size, void * obj) {
  RCL_CHECK_FOR_NULL_WITH_MSG(handles, "handles is NULL", return false);
  // did not use (i<size && handles[i].initialized) as loop-condition
  // because for last index i==size this would result in out-of-bound access
  for(unsigned int i=0; i<size; i++) {
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

bool rclc_let_executor_trigger_one(rclc_executor_handle_t * handles, unsigned int size, void * obj) {
  RCL_CHECK_FOR_NULL_WITH_MSG(handles, "handles is NULL", return false);
  // did not use (i<size && handles[i].initialized) as loop-condition
  // because for last index i==size this would result in out-of-bound access
  for(unsigned int i=0; i<size; i++) {
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
