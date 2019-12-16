// Copyright (c) 2019 - for handle_countersrmation on the respective copyright owner
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
#include <rclc/executor_handle.h>
// #include<stdio.h>
#include <rcl/error_handling.h>
#include <rcutils/logging_macros.h>


// initialization of handle_counters object
rcl_ret_t
rclc_executor_handle_counters_zero_init(rclc_executor_handle_counters_t * handle_counters)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(handle_counters, RCL_RET_INVALID_ARGUMENT);
  memset(handle_counters, 0, sizeof(rclc_executor_handle_counters_t));
  return RCL_RET_OK;
}

// initialization of handle object
rcl_ret_t
rclc_executor_handle_init(
  rclc_executor_handle_t * handle,
  size_t max_handles)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(handle, RCL_RET_INVALID_ARGUMENT);
  handle->type = NONE;
  handle->invocation = ON_NEW_DATA;
  handle->subscription = NULL;
  handle->timer = NULL;
  handle->data = NULL;
  handle->callback = NULL;
  handle->index = max_handles;
  handle->initialized = false;
  handle->data_available = false;
  return RCL_RET_OK;
}

rcl_ret_t
rclc_executor_handle_clear(
  rclc_executor_handle_t * handle,
  size_t max_handles)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(handle, RCL_RET_INVALID_ARGUMENT);

  handle->index = max_handles;
  handle->initialized = false;

  return RCL_RET_OK;
}

rcl_ret_t
rclc_executor_handle_print(rclc_executor_handle_t * handle)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(handle, RCL_RET_INVALID_ARGUMENT);

  char * typeName;

  switch (handle->type) {
    case NONE:
      typeName = "None";
      break;
    case SUBSCRIPTION:
      typeName = "Sub";
      break;
    case TIMER:
      typeName = "Timer";
      break;
    default:
      typeName = "Unknown";
  }
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "  %s\n", typeName);
  return RCL_RET_OK;
}
