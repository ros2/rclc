// Copyright (c) 2020 - for handle_countersrmation on the respective copyright owner
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

#include "rclc/executor_handle.h"

#include <rcl/error_handling.h>
#include <rcutils/logging_macros.h>


// convenience function to deal with fluctuating number of handle types
rclc_executor_handle_type_t
rclc_executor_handle_reduced_type(const rclc_executor_handle_type_t full_type)
{
  rclc_executor_handle_type_t reduced_type;
  switch (full_type) {
    case NONE:
      reduced_type = NONE;
      break;
    case SUBSCRIPTION:
    case SUBSCRIPTION_WITH_CONTEXT:
      reduced_type = SUBSCRIPTION;
      break;
    case TIMER:
      // case TIMER_WITH_CONTEXT:
      reduced_type = TIMER;
      break;
    case CLIENT:
    case CLIENT_WITH_REQUEST_ID:
      // case CLIENT_WITH_CONTEXT:
      reduced_type = CLIENT;
      break;
    case SERVICE:
    case SERVICE_WITH_REQUEST_ID:
    case SERVICE_WITH_CONTEXT:
      reduced_type = SERVICE;
      break;
    case GUARD_CONDITION:
      // case GUARD_CONDITION_WITH_CONTEXT:
      reduced_type = GUARD_CONDITION;
      break;
    default:
      reduced_type = full_type;
      break;
  }
  return reduced_type;
}

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
  // Note, the pointer to subscription, timer, service, client, gc is a union
  // and a single NULL assignment should be sufficient.
  handle->subscription = NULL;
  handle->timer = NULL;
  handle->service = NULL;
  handle->client = NULL;
  handle->gc = NULL;

  handle->data = NULL;
  handle->data_response_msg = NULL;
  handle->callback_context = NULL;

  handle->subscription_callback = NULL;
  // because of union structure:
  //   handle->service_callback == NULL;
  //   handle->client_callback == NULL;
  //   handle->gc_callback == NULL
  //   ...

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

  switch (rclc_executor_handle_reduced_type(handle->type)) {
    case NONE:
      typeName = "None";
      break;
    case SUBSCRIPTION:
      typeName = "Sub";
      break;
    case TIMER:
      typeName = "Timer";
      break;
    case CLIENT:
      typeName = "Client";
      break;
    case SERVICE:
      typeName = "Service";
      break;
    case GUARD_CONDITION:
      typeName = "GuardCondition";
      break;
    default:
      typeName = "Unknown";
  }
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "  %s\n", typeName);
  return RCL_RET_OK;
}

void *
rclc_executor_handle_get_ptr(rclc_executor_handle_t * handle)
{
  // RCL_CHECK_ARGUMENT_FOR_NULL(handle, RCL_RET_INVALID_ARGUMENT);
  // cannot be used because it creates a "return" statement and
  // here the return type is (void *)
  if (handle == NULL) {
    return NULL;
  }

  void * ptr;
  switch (rclc_executor_handle_reduced_type(handle->type)) {
    case SUBSCRIPTION:
      ptr = handle->subscription;
      break;
    case TIMER:
      ptr = handle->timer;
      break;
    case CLIENT:
      ptr = handle->client;
      break;
    case SERVICE:
      ptr = handle->service;
      break;
    case GUARD_CONDITION:
      ptr = handle->gc;
      break;
    case NONE:
    default:
      ptr = NULL;
  }

  return ptr;
}
