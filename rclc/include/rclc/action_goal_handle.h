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

#ifndef RCLC__ACTION_GOAL_HANDLE_H_
#define RCLC__ACTION_GOAL_HANDLE_H_

#if __cplusplus
extern "C"
{
#endif

#include <rcl_action/rcl_action.h>
#include <rcl_action/rcl_action.h>

struct rclc_generic_entity_t;
struct rclc_action_client_t;

typedef struct rclc_action_goal_handle_t
{
  struct rclc_action_goal_handle_t * next;

  union {
    struct rclc_action_server_t * action_server;
    struct rclc_action_client_t * action_client;
  };

  rcl_action_goal_handle_t * rcl_handle;
  rcl_action_goal_state_t status;

  unique_identifier_msgs__msg__UUID goal_id;
  struct Generic_SendGoal_Request * ros_goal_request;

  bool available_goal_response;
  bool goal_accepted;
  bool available_feedback;
  bool available_result_response;
  bool available_cancel_response;
  bool goal_cancelled;

  // Goal requests header
  union {
    rmw_request_id_t goal_request_header;
    int64_t goal_request_sequence_number;
  };

  union {
    rmw_request_id_t result_request_header;
    int64_t result_request_sequence_number;
  };

  union {
    rmw_request_id_t cancel_request_header;
    int64_t cancel_request_sequence_number;
  };

} rclc_action_goal_handle_t;

#define DECLARE_GOAL_HANDLE_POOL \
  rclc_action_goal_handle_t * goal_handles_memory; \
  size_t goal_handles_memory_size; \
  rclc_action_goal_handle_t * free_goal_handles; \
  rclc_action_goal_handle_t * used_goal_handles;

void rclc_action_put_goal_handle_in_list(
  rclc_action_goal_handle_t ** list,
  rclc_action_goal_handle_t * goal_handle);

bool rclc_action_check_handle_in_list(
  rclc_action_goal_handle_t ** list,
  rclc_action_goal_handle_t * goal_handle);

rclc_action_goal_handle_t * rclc_action_pop_goal_handle_from_list(
  rclc_action_goal_handle_t ** list);

rclc_action_goal_handle_t * rclc_action_take_goal_handle_from_list(
  rclc_action_goal_handle_t ** list,
  rclc_action_goal_handle_t * goal_handle);

rclc_action_goal_handle_t * rclc_action_take_goal_handle(
  void * untyped_entity);

void rclc_action_init_goal_handle_memory(
  void * untyped_entity);

void rclc_action_put_goal_handle(
  void * untyped_entity,
  rclc_action_goal_handle_t * goal_handle);

rclc_action_goal_handle_t * rclc_action_get_handle_by_uuid(
  void * untyped_entity,
  const unique_identifier_msgs__msg__UUID * uuid);

rclc_action_goal_handle_t * rclc_action_get_handle_by_uuid(
  void * untyped_entity,
  const unique_identifier_msgs__msg__UUID * uuid);

rclc_action_goal_handle_t * rclc_action_get_first_handle_by_status(
  void * untyped_entity,
  rcl_action_goal_state_t status);

rclc_action_goal_handle_t * rclc_action_get_handle_by_goal_request_sequence_number(
  void * untyped_entity,
  const int64_t goal_request_sequence_number);

rclc_action_goal_handle_t * rclc_action_get_handle_by_cancel_request_sequence_number(
  void * untyped_entity,
  const int64_t cancel_request_sequence_number);

rclc_action_goal_handle_t * rclc_action_get_first_handle_by_status(
  void * untyped_entity,
  rcl_action_goal_state_t status);

rclc_action_goal_handle_t * rclc_action_get_first_handle_with_goal_response(
  void * untyped_entity);

rclc_action_goal_handle_t * rclc_action_get_first_handle_with_feedback(
  void * untyped_entity);

rclc_action_goal_handle_t * rclc_action_get_first_handle_with_result_response(
  void * untyped_entity);

rclc_action_goal_handle_t * rclc_action_get_first_handle_with_cancel_response(
  void * untyped_entity);

void rclc_action_swap_messages(
  void ** ptr1,
  void ** ptr2);


#if __cplusplus
}
#endif

#endif  // RCLC__ACTION_GOAL_HANDLE_H_
