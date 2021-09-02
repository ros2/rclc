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
#ifndef RCLC__ACTION_SERVER_H_
#define RCLC__ACTION_SERVER_H_

#if __cplusplus
extern "C"
{
#endif

#include <rclc/visibility_control.h>

#include <rcl_action/rcl_action.h>
#include <rcl/allocator.h>
#include <rclc/types.h>
#include <rclc/action_goal_handle.h>

struct rclc_action_goal_handle_t;

typedef rcl_ret_t (* rclc_action_server_handle_goal_callback_t)(
  rclc_action_goal_handle_t * goal_handle,
  void * args);

typedef bool (* rclc_action_server_handle_cancel_callback_t)(
  rclc_action_goal_handle_t * ros_cancel_request,
  void * args);

typedef struct rclc_action_server_t
{
  DECLARE_GOAL_HANDLE_POOL

  rcl_action_server_t rcl_handle;

  // Callbacks
  rclc_action_server_handle_goal_callback_t goal_callback;
  rclc_action_server_handle_cancel_callback_t cancel_callback;

  // Available flags
  bool goal_request_available;
  bool cancel_request_available;
  bool result_request_available;
  bool goal_expired_available;
} rclc_action_server_t;

/**
 *  Creates an rcl action server.
 *
 *  * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes (in RCL)
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param[inout] client a zero_initialized rcl_action_server_t
 * \param[in] node the rcl node
 * \param[in] type_support the message data type
 * \param[in] action_name the name of the action
 * \return `RCL_RET_OK` if successful
 * \return `RCL_ERROR` (or other error code) if an error has occurred
 */
RCLC_PUBLIC
rcl_ret_t
rclc_action_server_init_default(
  rclc_action_server_t * action_server,
  rcl_node_t * node,
  rcl_clock_t * clock,
  const rosidl_action_type_support_t * type_support,
  const char * action_name);

RCLC_PUBLIC
rcl_ret_t
rclc_action_server_accept_goal_request(
  rclc_action_goal_handle_t * goal_handle);

RCLC_PUBLIC
rcl_ret_t
rclc_action_server_reject_goal_request(
  rclc_action_goal_handle_t * goal_handle);

RCLC_PUBLIC
rcl_ret_t
rclc_action_server_finish_goal_sucess(
  rclc_action_goal_handle_t * goal_handle,
  void * ros_response);

RCLC_PUBLIC
rcl_ret_t
rclc_action_server_finish_goal_abort(
  rclc_action_goal_handle_t * goal_handle,
  void * ros_response);

RCLC_PUBLIC
rcl_ret_t
rclc_action_server_finish_goal_cancel(
  rclc_action_goal_handle_t * goal_handle);

RCLC_PUBLIC
rcl_ret_t
rclc_action_server_goal_cancel_reject(
  rclc_action_goal_handle_t * goal_handle);

RCLC_PUBLIC
rcl_ret_t
rclc_action_publish_feedback(
  rclc_action_goal_handle_t * goal_handle,
  void * ros_feedback);

RCLC_PUBLIC
rcl_ret_t
rclc_action_server_fini(
  rclc_action_server_t * action_server,
  rcl_node_t * node);

#if __cplusplus
}
#endif

#endif  // RCLC__ACTION_SERVER_H_
