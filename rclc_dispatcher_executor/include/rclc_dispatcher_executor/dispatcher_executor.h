// Copyright (c) 2022 - for information on the respective copyright owner
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
#ifndef RCLC_DISPATCHER_EXECUTOR__DISPATCHER_EXECUTOR_H_
#define RCLC_DISPATCHER_EXECUTOR__DISPATCHER_EXECUTOR_H_

#if __cplusplus
extern "C"
{
#endif

#include <rclc/executor.h>
#include <pthread.h>
#include <sched.h>
#include "rclc_dispatcher_executor/visibility_control.h"


/// Implementation for sporadic server scheduler for NuttX
typedef enum
{
  RCLC_THREAD_NONE,
  RCLC_THREAD_READY,
  RCLC_THREAD_BUSY
} rclc_executor_thread_state_t;

/// Scheduling policy (SCHED_FIFO, SCHED_SPORADIC) and sched_param
typedef struct
{
  int policy;
  struct sched_param param;
} rclc_executor_sched_parameter_t;

typedef struct
{
  /// mutex for worker threads
  pthread_mutex_t thread_state_mutex;
  /// mutex for RCL layer
  pthread_mutex_t micro_ros_mutex;
} rclc_executor_multi_threaded_data_t;

typedef struct
{
  /// worker thread
  pthread_t worker_thread;
  /// worker thread state and its mutex
  rclc_executor_thread_state_t worker_thread_state;
  /// signaling condition variable and its mutex
  pthread_cond_t new_msg_cond;
  pthread_mutex_t new_msg_mutex;
  bool new_msg_avail;
  /// scheduling parameter
  rclc_executor_sched_parameter_t * sparam;
  pthread_attr_t tattr;
} rclc_handle_multi_threaded_data_t;

typedef struct
{
  pthread_mutex_t * thread_state_mutex;
  pthread_mutex_t * micro_ros_mutex;
  rclc_executor_handle_t * handle;
} rclc_executor_worker_thread_param_t;

/**
 * Initialization of multi-threaded Executor.
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param [inout] executor pointer to pre-allocated rclc_executor_t
 */
RCLC_DISPATCHER_EXECUTOR_PUBLIC
rcl_ret_t
rclc_dispatcher_executor_init(rclc_executor_t * executor);

/**
 *  Adds a subscription to an executor with scheduling policy.
 *  An error is returned, if {@link rclc_executor_t.handles} array is full.
 *  The total number_of_subscriptions field of {@link rclc_executor_t.info}
 *  is incremented by one.
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
 * \param [in] sparam thread scheduling parameter (e.g. thread priority)
 * \return `RCL_RET_OK` if add-operation was successful
 * \return `RCL_RET_INVALID_ARGUMENT` if any parameter is a null pointer
 * \return `RCL_RET_ERROR` if any other error occured
 */
RCLC_DISPATCHER_EXECUTOR_PUBLIC
rcl_ret_t
rclc_dispatcher_executor_add_subscription(
  rclc_executor_t * executor,
  rcl_subscription_t * subscription,
  void * msg,
  rclc_callback_t callback,
  rclc_executor_handle_invocation_t invocation,
  rclc_executor_sched_parameter_t * sparam);


/**
 * Spin once with multi-threaded Executor.
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param [inout] executor pointer to pre-allocated rclc_executor_t
 */
RCLC_DISPATCHER_EXECUTOR_PUBLIC
rcl_ret_t
rclc_dispatcher_executor_spin_once(rclc_executor_t * executor);

/**
 * Starts the multi-threaded Executor.
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param [inout] executor pointer to pre-allocated rclc_executor_t
 */
RCLC_DISPATCHER_EXECUTOR_PUBLIC
rcl_ret_t
rclc_dispatcher_executor_spin(rclc_executor_t * executor);

/**
 * Publish a ROS message on a topic using a publisher.
 * It is thread-safe, in the sense, that multiple simultaneous calls
 * (from the worker-threads) are processed sequentially. So that only one
 * thread accesses the RCL layer (rcl_wait, rcl_take, rcl_publish)
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param [inout] publisher pointer to pre-allocated rcl_publisher_t
 * \param [inout] ros_message pointer to ROS message
 * \param [inout] allocation pointer to pre-allocated rmw_publisher_allocation_t
 * \return `RCL_RET_OK` if add-operation was successful
 * \return `RCL_RET_INVALID_ARGUMENT` if any parameter is a null pointer
 * \return `RCL_RET_ERROR` if any other error occured
 */
RCLC_DISPATCHER_EXECUTOR_PUBLIC
rcl_ret_t rclc_dispatcher_executor_publish(
  const rcl_publisher_t * publisher,
  const void * ros_message,
  rmw_publisher_allocation_t * allocation);


/**
 * Deallocation of multi-threaded Executor.
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes (deallocates)
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param [inout] executor pointer to pre-allocated rclc_executor_t
 */
RCLC_DISPATCHER_EXECUTOR_PUBLIC
rcl_ret_t
rclc_dispatcher_executor_init(rclc_executor_t * executor);

#if __cplusplus
}
#endif

#endif  // RCLC_DISPATCHER_EXECUTOR__DISPATCHER_EXECUTOR_H_
