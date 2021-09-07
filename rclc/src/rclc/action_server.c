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

#include <rclc/action_server.h>

#include <rcl/error_handling.h>
#include <rcutils/logging_macros.h>

#include "./action_generic_types.h"
#include "./action_goal_handle_internal.h"

rcl_ret_t
rclc_action_server_init_default(
  rclc_action_server_t * action_server,
  rcl_node_t * node,
  rcl_clock_t * clock,
  const rosidl_action_type_support_t * type_support,
  const char * action_name)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    action_server, "action_server is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    node, "node is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    clock, "clock is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    type_support, "type_support is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    action_name, "action_name is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  *action_server = (rclc_action_server_t) {0};

  action_server->rcl_handle = rcl_action_get_zero_initialized_server();
  rcl_action_server_options_t action_server_opt = rcl_action_server_get_default_options();
  rcl_ret_t rc = rcl_action_server_init(
    &action_server->rcl_handle,
    node,
    clock,
    type_support,
    action_name,
    &action_server_opt);
  if (rc != RCL_RET_OK) {
    PRINT_RCLC_ERROR(rclc_action_server_init_default, rcl_action_server_init);
  }

  return rc;
}

static bool rclc_action_server_is_valid_handle(
  rclc_action_goal_handle_t * goal_handle)
{
  return rclc_action_check_handle_in_list(
    &goal_handle->action_server->used_goal_handles,
    goal_handle);
}

rcl_ret_t
rclc_action_server_accept_goal_request(
  rclc_action_goal_handle_t * goal_handle)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    goal_handle, "goal handle is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  rclc_action_server_t * action_server = goal_handle->action_server;

  Generic_SendGoal_Response res = {0};
  res.accepted = true;

  rcl_ret_t rc = rcl_action_send_goal_response(
    &action_server->rcl_handle,
    &goal_handle->goal_request_header, &res);
  if (rc != RCL_RET_OK) {
    PRINT_RCLC_ERROR(rclc_action_server_accept_request, rcl_action_send_goal_response);
    return rc;
  }

  rcl_action_goal_info_t goal_info = rcl_action_get_zero_initialized_goal_info();
  goal_info.goal_id = goal_handle->goal_id;

  goal_handle->rcl_handle = rcl_action_accept_new_goal(&action_server->rcl_handle, &goal_info);
  rc = rcl_action_update_goal_state(goal_handle->rcl_handle, GOAL_EVENT_EXECUTE);
  if (rc != RCL_RET_OK) {
    PRINT_RCLC_ERROR(rclc_action_server_accept_request, rcl_action_update_goal_state);
    return rc;
  }

  rcl_action_goal_status_array_t c_status_array =
    rcl_action_get_zero_initialized_goal_status_array();
  rc = rcl_action_get_goal_status_array(&action_server->rcl_handle, &c_status_array);
  if (rc != RCL_RET_OK) {
    PRINT_RCLC_ERROR(rclc_action_server_accept_request, rcl_action_get_goal_status_array);
    return rc;
  }

  rc = rcl_action_publish_status(&action_server->rcl_handle, &c_status_array.msg);
  if (rc != RCL_RET_OK) {
    PRINT_RCLC_ERROR(rclc_action_server_accept_request, rcl_action_publish_status);
    return rc;
  }

  return RCL_RET_OK;
}

rcl_ret_t
rclc_action_server_reject_goal_request(
  rclc_action_goal_handle_t * goal_handle)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    goal_handle, "goal handle is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  rclc_action_server_t * action_server = goal_handle->action_server;

  Generic_SendGoal_Response res;
  res.accepted = false;

  rcl_ret_t rc = rcl_action_send_goal_response(
    &action_server->rcl_handle,
    &goal_handle->goal_request_header, &res);
  if (rc != RCL_RET_OK) {
    PRINT_RCLC_ERROR(rclc_action_server_accept_request, rcl_action_send_goal_response);
    return rc;
  }

  return RCL_RET_OK;
}

rcl_ret_t
rclc_action_server_finish_goal_sucess(
  rclc_action_goal_handle_t * goal_handle,
  void * ros_response)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    goal_handle, "goal handle is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    ros_response, "ros_response handle is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  if (!rclc_action_server_is_valid_handle(goal_handle)) {
    return RCL_RET_INVALID_ARGUMENT;
  }

  Generic_GetResult_Response * response = (Generic_GetResult_Response *)ros_response;

  rcl_ret_t rc = rcl_action_update_goal_state(goal_handle->rcl_handle, GOAL_EVENT_SUCCEED);
  if (rc != RCL_RET_OK) {
    PRINT_RCLC_ERROR(rclc_action_server_finish_goal_sucess, rcl_action_update_goal_state);
    return rc;
  }

  rc = rcl_action_notify_goal_done(&goal_handle->action_server->rcl_handle);
  if (rc != RCL_RET_OK) {
    PRINT_RCLC_ERROR(rclc_action_server_finish_goal_sucess, rcl_action_notify_goal_done);
    return rc;
  }

  response->status = GOAL_STATE_SUCCEEDED;

  rc = rcl_action_send_result_response(
    &goal_handle->action_server->rcl_handle,
    &goal_handle->result_request_header, response);

  rclc_action_put_goal_handle(goal_handle->action_server, goal_handle);

  return rc;
}

rcl_ret_t
rclc_action_server_finish_goal_abort(
  rclc_action_goal_handle_t * goal_handle,
  void * ros_response)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    goal_handle, "goal handle is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    ros_response, "ros_response is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  if (!rclc_action_server_is_valid_handle(goal_handle)) {
    return RCL_RET_INVALID_ARGUMENT;
  }

  rcl_ret_t rc = rcl_action_update_goal_state(goal_handle->rcl_handle, GOAL_EVENT_ABORT);
  if (rc != RCL_RET_OK) {
    PRINT_RCLC_ERROR(rclc_action_server_finish_goal_abort, rcl_action_update_goal_state);
    return rc;
  }

  rc = rcl_action_notify_goal_done(&goal_handle->action_server->rcl_handle);
  if (rc != RCL_RET_OK) {
    PRINT_RCLC_ERROR(rclc_action_server_finish_goal_abort, rcl_action_notify_goal_done);
    return rc;
  }

  Generic_GetResult_Response * response = ros_response;

  response->status = GOAL_STATE_ABORTED;

  rc = rcl_action_send_result_response(
    &goal_handle->action_server->rcl_handle,
    &goal_handle->result_request_header, response);

  rclc_action_put_goal_handle(goal_handle->action_server, goal_handle);

  return rc;
}

rcl_ret_t
rclc_action_server_finish_goal_cancel(
  rclc_action_goal_handle_t * goal_handle)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    goal_handle, "goal handle is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  if (!rclc_action_server_is_valid_handle(goal_handle)) {
    return RCL_RET_INVALID_ARGUMENT;
  }

  rcl_action_cancel_request_t cancel_request = rcl_action_get_zero_initialized_cancel_request();
  cancel_request.goal_info.goal_id = goal_handle->goal_id;

  rcl_action_cancel_response_t cancel_response =
    rcl_action_get_zero_initialized_cancel_response();

  rcl_ret_t rc = rcl_action_process_cancel_request(
    &goal_handle->action_server->rcl_handle,
    &cancel_request,
    &cancel_response);

  if (RCL_RET_OK != rc) {
    return rc;
  }

  const bool is_cancelable = rcl_action_goal_handle_is_cancelable(goal_handle->rcl_handle);
  if (is_cancelable) {
    rcl_ret_t rc = rcl_action_update_goal_state(goal_handle->rcl_handle, GOAL_EVENT_CANCEL_GOAL);
    if (RCL_RET_OK != rc) {
      return rc;
    }
  }

  rcl_action_goal_state_t state = GOAL_STATE_UNKNOWN;
  rc = rcl_action_goal_handle_get_status(goal_handle->rcl_handle, &state);
  if (RCL_RET_OK != rc) {
    return rc;
  }

  if (GOAL_STATE_CANCELING == state) {
    rc = rcl_action_update_goal_state(goal_handle->rcl_handle, GOAL_EVENT_CANCELED);
  } else {
    return RCL_RET_ERROR;
  }

  rc = rcl_action_send_cancel_response(
    &goal_handle->action_server->rcl_handle,
    &goal_handle->cancel_request_header, &cancel_response.msg);
  if (RCL_RET_OK != rc) {
    return rc;
  }

  Generic_GetResult_Response response = {0};

  response.status = GOAL_STATE_CANCELED;

  rc = rcl_action_send_result_response(
    &goal_handle->action_server->rcl_handle,
    &goal_handle->result_request_header, &response);

  return rc;
}

rcl_ret_t
rclc_action_server_goal_cancel_reject(
  rclc_action_goal_handle_t * goal_handle)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    goal_handle, "goal handle is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  rcl_action_cancel_response_t cancel_response =
    rcl_action_get_zero_initialized_cancel_response();
  cancel_response.msg.return_code = action_msgs__srv__CancelGoal_Response__ERROR_REJECTED;
  return rcl_action_send_cancel_response(
    &goal_handle->action_server->rcl_handle,
    &goal_handle->cancel_request_header, &cancel_response.msg);
}

rcl_ret_t rclc_action_publish_feedback(
  rclc_action_goal_handle_t * goal_handle,
  void * ros_feedback)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    goal_handle, "goal handle is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    ros_feedback, "feedback is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  if (!rclc_action_server_is_valid_handle(goal_handle)) {
    return RCL_RET_INVALID_ARGUMENT;
  }

  Generic_FeedbackMessage * feedback = (Generic_FeedbackMessage *) ros_feedback;

  memcpy(
    feedback->goal_id.uuid, goal_handle->goal_id.uuid,
    sizeof(feedback->goal_id.uuid));
  rcl_ret_t rc = rcl_action_publish_feedback(&goal_handle->action_server->rcl_handle, feedback);
  return rc;
}


rcl_ret_t
rclc_action_server_fini(
  rclc_action_server_t * action_server,
  rcl_node_t * node)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    action_server, "action_server is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    node, "node is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  rcl_ret_t rc;

  if (NULL != action_server->goal_handles_memory) {
    action_server->allocator->deallocate(
      action_server->goal_handles_memory,
      action_server->allocator->state);
  }

  rc = rcl_action_server_fini(&action_server->rcl_handle, node);

  return rc;
}
