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
#ifndef RCLC__MULTI_THREADED_EXECUTOR_H_
#define RCLC__MULTI_THREADED_EXECUTOR_H_

#if __cplusplus
extern "C"
{
#endif

#include <rclc/executor.h>


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
 * \param [in] param scheduling parameters for the thread that is executing the callback
 * \return `RCL_RET_OK` if add-operation was successful
 * \return `RCL_RET_INVALID_ARGUMENT` if any parameter is a null pointer
 * \return `RCL_RET_ERROR` if any other error occured
 */
rcl_ret_t
rclc_executor_add_subscription_multi_threaded(
  rclc_executor_t * executor,
  rcl_subscription_t * subscription,
  void * msg,
  rclc_callback_t callback,
  rclc_executor_handle_invocation_t invocation,
  rclc_executor_sched_parameter_t * param);

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
void
rclc_executor_init_multi_threaded(rclc_executor_t * e);

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
rcl_ret_t
rclc_executor_spin_multi_threaded(rclc_executor_t * e);

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
rcl_ret_t rclc_executor_publish(
  const rcl_publisher_t * publisher,
  const void * ros_message,
  rmw_publisher_allocation_t * allocation);

#if __cplusplus
}
#endif

#endif  // RCLC__MULTI_THREADED_EXECUTOR_H_
