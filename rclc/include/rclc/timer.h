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
#ifndef RCLC__TIMER_H_
#define RCLC__TIMER_H_

#if __cplusplus
extern "C"
{
#endif

#include <rcl/timer.h>
#include <rclc/types.h>
#include "rclc/visibility_control.h"

/**
 *  Creates an rcl timer.
 *
 *  * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes (in RCL)
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param[inout] timer a zero-initialized rcl_timer_t
 * \param[in] support the rclc_support_t object
 * \param[in] timeout_ns the time out in nanoseconds of the timer
 * \param[in] callback the callback of the timer
 * \return `RCL_RET_OK` if successful
 * \return `RCL_ERROR` (or other error code) if an error occurred
 */
RCLC_PUBLIC
rcl_ret_t
rclc_timer_init_default(
  rcl_timer_t * timer,
  rclc_support_t * support,
  const uint64_t timeout_ns,
  const rcl_timer_callback_t callback);

/**
 *  Allocates an rcl_timer_t object on the heap and sets its values to zero.
 *  Can be used as an alternative to rclc_timer_init_default() if no
 *  stack allocation can or should be used.
 *
 *  * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | No
 *
 * \param[in] allocator allocator for allocating memory
 * \return pointer to the timer (rcl_timer_t)
 * \return NULL, if no memory could be allocated.
 */
RCLC_PUBLIC
rcl_timer_t *
rclc_alloc_zero_initialized_timer(
  const rcl_allocator_t * const allocator
);

/**
 *  De-allocates an rcl_timer_t object.
 *
 *  * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | No
 *
 * \param[inout] timer a heap-allocated rcl_timer_t
 * \param[in] allocator the rcl_allocator_t to be used
 * \return `RCL_RET_OK` if operation was successful
 * \return `RCL_RET_INVALID_ARGUMENT` if any null pointer as argument
 */
RCLC_PUBLIC
rcl_ret_t
rclc_timer_free(
  rcl_timer_t * timer,
  const rcl_allocator_t * const allocator);

#if __cplusplus
}
#endif

#endif  // RCLC__TIMER_H_
