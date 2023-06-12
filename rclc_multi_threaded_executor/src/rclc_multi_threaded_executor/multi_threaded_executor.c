// Copyright (c) 2020 - for information on the respective copyright owner
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

#include "rclc_multi_threaded_executor/multi_threaded_executor.h"


// note: check if necessary: publish and wait/take can be done
// in parallel, so this mutex should not be necessary any more.
static pthread_mutex_t * rclc_micro_ros_mutex;

rcl_ret_t
rclc_multi_threaded_executor_configure(rclc_executor_t * executor)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  executor->type = MULTI_THREADED;
  executor->init = rclc_multi_threaded_init;
  executor->spin_init = rclc_multi_threaded_executor_spin_init;
  return RCL_RET_OK;
}

rclc_handle_multi_threaded_data_t *
get_multi_threaded_handle(rclc_executor_handle_t * handle)
{
  return (rclc_handle_multi_threaded_data_t *) handle->custom;
}

rclc_executor_multi_threaded_data_t *
get_multi_threaded_executor(rclc_executor_t * executor)
{
  return (rclc_executor_multi_threaded_data_t *) executor->custom;
}

rcl_ret_t
rclc_multi_threaded_executor_spin_init(rclc_executor_t * executor)
{
  RCLC_UNUSED(executor);
  return RCL_RET_OK;
}




/// initialization of handle object for multi-threaded executor
rcl_ret_t
rclc_multi_threaded_executor_handle_init(
  rclc_executor_t * executor,
  rclc_executor_handle_t * handle)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(handle, RCL_RET_INVALID_ARGUMENT);

  get_multi_threaded_handle(handle) = executor->allocator->allocate(
    sizeof(rclc_handle_multi_threaded_data_t), executor->allocator->state);
  if (NULL == handle->multi_threaded) {
    RCL_SET_ERROR_MSG("Could not allocate memory for 'handle->custom'.");
    return RCL_RET_BAD_ALLOC;
  }
  // note: worker_thread not initialized => okay
  (get_multi_threaded_handle(handle))->worker_thread_state = RCLC_THREAD_READY;
  pthread_cond_init(&(get_multi_threaded_handle(handle))->new_msg_cond, NULL);
  pthread_mutex_init(&(get_multi_threaded_handle(handle))->new_msg_mutex, NULL);
  (get_multi_threaded_handle(handle))->new_msg_avail = false;
  (get_multi_threaded_handle(handle))->sparam = NULL;
  // note: tattr not initialized => okay
  return RCL_RET_OK;
}

// note: add multi_threaded_initialized flag
//       otherwise the functions to add subscription will fail.
rcl_ret_t rclc_multi_threaded_executor_init(rclc_executor_t * executor)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);

  // memory allocation and initialization of multi-threaded handle
  for (size_t i = 0; i < executor->max_handles; i++) {
    if (rclc_multi_threaded_executor_handle_init(
        executor, &executor->handles[i]) != RCL_RET_OK) {
          RCL_SET_ERROR_MSG("rclc_multi_threaded_executor_init: could not allocate memory for custom handle.");
          return RCL_RET_BAD_ALLOC;
        }
  }

  // memory allocation and initialization of multi-threaded executor
  get_multi_threaded_executor(executor) = executor->allocator->allocate(
    sizeof(rclc_executor_multi_threaded_data_t), executor->allocator->state);
  if (NULL == get_multi_threaded_handle(executor)) {
    RCL_SET_ERROR_MSG("rclc_multi_threaded_executor_init: could not allocate memory for custom executor.");
    return RCL_RET_BAD_ALLOC;
  }

  // initialization of mutexes and condition variables
  pthread_mutex_init(&(get_multi_threaded_executor(executor)->thread_state_mutex), NULL);
  pthread_mutex_init(&(get_multi_threaded_executor(executor)->micro_ros_mutex), NULL);

  rclc_micro_ros_mutex = &(get_multi_threaded_executor(executor)->micro_ros_mutex);
  return RCL_RET_OK;
}

rcl_ret_t
rclc_multi_threaded_executor_subscription_set_sched_param(
  rclc_executor_t * executor,
  rcl_subscription_t * subscription,
  rclc_executor_sched_parameter_t * sparam)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(subscription, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(sparam, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;

  for(unsinged int i = 0; i< executor->max_handles; i++)
  if (executor->handles[[executor->index].type == RCLC_SUBSCRIPTION]) &&
  (executor->handles[executor->index].subscription == subscription)
  {
    (get_multi_threaded_handle(
      &executor->handles[executor->index]))->sparam = sparam;
  }
  // if sched_param is changed at runtime, what else needs to be done?
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Multi-threaded-executor: assigned sched_param to subscription.");
  return ret;
}
