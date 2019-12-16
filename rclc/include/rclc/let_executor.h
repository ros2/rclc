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
#ifndef RCLC__LET_EXECUTOR_H_
#define RCLC__LET_EXECUTOR_H_

/* TODO(jan):
- timeout for spinning - not for waiting at DDS
- trigger to execute all functions
- example with convenience functions
- update test cases

*/
#if __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdarg.h>

#include <rcl/error_handling.h>
#include <rcutils/logging_macros.h>

#include "rclc/executor_handle.h"

/*! \file let_executor.h
    \brief The LET-Executor provides an Executor based on RCL in which all callbacks are
    processed in a user-defined order.
*/

/// Container for executor
typedef struct
{
  /// Context (to get information if ROS is up-and-running)
  rcl_context_t * context;
  /// Container for dynamic array for DDS-handles
  rclc_executor_handle_t * handles;
  /// Maximum size of array 'handles'
  size_t max_handles;
  /// Index to the next free element in array handles
  size_t index;
  /// Container to memory allocator for array handles
  const rcl_allocator_t * allocator;
  /// Wait set (is initialized only in the first call of the rclc_let_executor_spin_some function)
  rcl_wait_set_t wait_set;
  /// Statistics objects about total number of subscriptions, timers, clients, services, etc.
  rclc_executor_handle_counters_t info;
  /// timeout in nanoseconds for rcl_wait() used in rclc_let_executor_spin_once(). Default 100ms
  uint64_t timeout_ns;
  /// timepoint used for spin_period()
  rcutils_time_point_value_t invocation_time;
} rclc_let_executor_t;


/**
 *  Return a rclc_let_executor_t struct with pointer members initialized to `NULL`
 *  and member variables to 0.
 */
rclc_let_executor_t
rclc_let_executor_get_zero_initialized_executor(void);
/**
 *  Initializes an executor.
 *  It creates a dynamic array with size \p number_of_handles using the
 *  \p allocator.
 *
 *
 *  * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param[inout] e preallocated rclc_let_executor_t
 * \param[in] context RCL context
 * \param[in] number_of_handles size of the handle array
 * \param[in] allocator allocator for allocating memory
 * \return `RCL_RET_OK` if the executor was initialized successfully
 * \return `RCL_RET_INVALID_ARGUMENT` if any null pointer as argument
 * \return `RCL_RET_ERROR` in case of failure
 */
rcl_ret_t
rclc_let_executor_init(
  rclc_let_executor_t * executor,
  rcl_context_t * context,
  const size_t number_of_handles,
  const rcl_allocator_t * allocator);

/**
 *  Set timeout in nanoseconds for rcl_wait (called during {@link rclc_let_executor_spin_once()}).
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param [inout] executor pointer to an initialized executor
 * \param [in] timeout_ns  timeout in nanoseconds for the rcl_wait (DDS middleware)
 * \return `RCL_RET_OK` if timeout was set successfully
 * \return `RCL_RET_INVALID_ARGUMENT` if \p executor is a null pointer
 * \return `RCL_RET_ERROR` in an error occured
 */
rcl_ret_t
rclc_let_executor_set_timeout(
  rclc_let_executor_t * executor,
  const uint64_t timeout_ns);


/**
 *  Cleans up executor.
 *  Deallocates dynamic memory of {@link rclc_let_executor_t.handles} and
 *  resets all other values of {@link rclc_let_executor_t}.
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param [inout] executor pointer to initialized executor
 * \return `RCL_RET_OK` if reset operation was successful
 * \return `RCL_RET_INVALID_ARGUMENT` if \p executor is a null pointer
 * \return `RCL_RET_INVALID_ARGUMENT` if \p executor.handles is a null pointer
 * \return `RCL_RET_ERROR` in an error occured (aka executor was not initialized)
 */
rcl_ret_t
rclc_let_executor_fini(rclc_let_executor_t * executor);

/**
 *  Adds a subscription to an executor.
 * * An error is returned, if {@link rclc_let_executor_t.handles} array is full.
 * * The total number_of_subscriptions field of {@link rclc_let_executor_t.info}
 *   is incremented by one.
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param [inout] executor pointer to initialized executor
 * \param [in] subscription pointer to an allocated subscription
 * \param [in] msg pointer to an allocated message
 * \param [in] callback    function pointer to a callback
 * \param [in] invocation  invocation type for the callback (ALWAYS or only ON_NEW_DATA)
 * \return `RCL_RET_OK` if add-operation was successful
 * \return `RCL_RET_INVALID_ARGUMENT` if any parameter is a null pointer
 * \return `RCL_RET_ERROR` if any other error occured
 */
rcl_ret_t
rclc_let_executor_add_subscription(
  rclc_let_executor_t * executor,
  rcl_subscription_t * subscription,
  void * msg,
  rcle_callback_t callback,
  rcle_invocation_t invocation);

/**
 *  Adds a timer to an executor.
 * * An error is returned, if {@link rclc_let_executor_t.handles} array is full.
 * * The total number_of_timers field of {@link rclc_let_executor_t.info} is
 *   incremented by one.
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param [inout] executor pointer to initialized executor
 * \param [in] timer pointer to an allocated timer
 * \return `RCL_RET_OK` if add-operation was successful
 * \return `RCL_RET_INVALID_ARGUMENT` if any parameter is a null pointer
 * \return `RCL_RET_ERROR` if any other error occured
 */
rcl_ret_t
rclc_let_executor_add_timer(
  rclc_let_executor_t * executor,
  rcl_timer_t * timer);

/**
 *  The spin-some function checks one-time for new data from the DDS-queue.
 * * the timeout is defined in {@link rclc_let_executor_t.timeout_ns} and can
 *   be set by calling {@link rclc_let_executor_set_timeout()} function (default value is 100ms)
 *
 * The static-LET executor performs the following actions:
 * * initializes the wait_set with all handle of the array executor->handles
 * * waits for new data from DDS queue with rcl_wait() with timeout executor->timeout_ns
 * * takes all ready handles from the wait_set with rcl_take()
 * * processes all handles in the order, how they were added to the executor with the respective add-functions
 *   by calling respective callback (thus implementing first-read, process, semantic of LET)
 *
 * Memory is dynamically allocated within rcl-layer, when DDS queue is accessed with rcl_wait_set_init()
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 *
 * \param [inout] executor pointer to initialized executor
 * \param[in] timeout_ns  timeout in millisonds
 * \return `RCL_RET_OK` if spin_once operation was successful
 * \return `RCL_RET_INVALID_ARGUMENT` if any parameter is a null pointer
 * \return `RCL_RET_TIMEOUT` if rcl_wait() returned timeout (aka no data is avaiable during until the timeout)
 * \return `RCL_RET_ERROR` if any other error occured
 */
rcl_ret_t
rclc_let_executor_spin_some(
  rclc_let_executor_t * executor,
  const uint64_t timeout_ns);

/**
 *  The spin function checks for new data at DDS queue as long as ros context is available.
 *  It calls {@link rclc_let_executor_spin_some()} as long as rcl_is_context_is_valid() returns true.
 *
 *  Memory is dynamically allocated within rcl-layer, when DDS queue is accessed with rcl_wait_set_init()
 *  (in spin_some function)
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 *
 * \param [inout] executor pointer to initialized executor
 * \return `RCL_RET_OK` if spin operation was successful
 * \return `RCL_RET_INVALID_ARGUMENT` if executor is a null pointer
 * \return `RCL_RET_ERROR` if any other error occured
 */
rcl_ret_t
rclc_let_executor_spin(rclc_let_executor_t * executor);


/**
 *  The spin_period function checks for new data at DDS queue as long as ros context is available.
 *  It is called every period nanoseconds.
 *  It calls {@link rclc_let_executor_spin_some()} as long as rcl_is_context_is_valid() returns true.
 *
 *  Memory is dynamically allocated within rcl-layer, when DDS queue is accessed with rcl_wait_set_init()
 *  (in spin_some function)
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 *
 * \param [inout] executor pointer to initialized executor
 * \param [in] period in nanoseconds
 * \return `RCL_RET_OK` if spin operation was successful
 * \return `RCL_RET_INVALID_ARGUMENT` if executor is a null pointer
 * \return `RCL_RET_ERROR` if any other error occured
 */
rcl_ret_t
rclc_let_executor_spin_period(
  rclc_let_executor_t * executor,
  const uint64_t period);


/*
 The reason for splitting this function up, is to be able to write a unit test.
 The spin_period is an endless loop, therefore it is not possible to stop after x iterations. The function
 rclc_let_executor_spin_period_ implements one iteration and the function
 rclc_let_executor_spin_period implements the endless while-loop. The unit test covers only
 rclc_let_executor_spin_period_.
*/
rcl_ret_t
rclc_let_executor_spin_one_period(
  rclc_let_executor_t * executor,
  const uint64_t period);

#if __cplusplus
}
#endif

#endif  // RCLC__LET_EXECUTOR_H_
