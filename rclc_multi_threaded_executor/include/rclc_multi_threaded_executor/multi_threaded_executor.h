// Copyright (c) 2023 - for information on the respective copyright owner
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
#ifndef RCLC_MULTI_THREADED_EXECUTOR__MULTI_THREADED_EXECUTOR_H_
#define RCLC_MULTI_THREADED_EXECUTOR__MULTI_THREADED_EXECUTOR_H_

#if __cplusplus
extern "C"
{
#endif

#include <rcl/rcl.h>
#include <pthread.h>
#include <sched.h>
#include <rclc/executor.h>
#include <rclc/visibility_control.h>


/// Thread state: READY, BUSY
typedef enum
{
  RCLC_THREAD_NONE,
  RCLC_THREAD_READY,
  RCLC_THREAD_BUSY
} rclc_executor_thread_state_t;

/// Scheduling policy (SCHED_FIFO, SCHED_SPORADIC) and POSIX parameter sched_param
typedef struct
{
  int policy;
  struct sched_param param;
} rclc_executor_sched_parameter_t;

typedef struct
{
  /// mutex for worker threads
  pthread_mutex_t thread_state_mutex;
  /// mutex for RCL layer access
  pthread_mutex_t micro_ros_mutex;
} rclc_executor_multi_threaded_data_t;

typedef struct
{
  /// worker thread
  pthread_t worker_thread;
  /// thread scheduling parameters
  rclc_executor_sched_parameter_t * sparam;
  pthread_attr_t tattr;
  /// worker thread state
  rclc_executor_thread_state_t worker_thread_state;
  /// condition variable to signal worker threads
  pthread_cond_t new_msg_cond;
  pthread_mutex_t new_msg_mutex;
  bool new_msg_avail;
} rclc_handle_multi_threaded_data_t;

typedef struct
{
  pthread_mutex_t * thread_state_mutex;
  pthread_mutex_t * micro_ros_mutex;
  rclc_executor_handle_t * handle;
} rclc_executor_worker_thread_param_t;

RCLC_PUBLIC
rcl_ret_t
rclc_multi_threaded_executor_configure(rclc_executor_t * executor);

RCLC_PUBLIC
rcl_ret_t
rclc_multi_threaded_executor_init(rclc_executor_t * executor);

RCLC_PUBLIC
rcl_ret_t
rclc_multi_threaded_executor_spin_init(rclc_executor_t * executor);

RCLC_PUBLIC
rcl_ret_t
rclc_multi_threaded_executor_subscription_set_sched_param(
  rclc_executor_t * executor,
  rcl_subscription_t * subscription,
  rclc_executor_sched_parameter_t * sparam);

#if __cplusplus
}
#endif

#endif  // RCLC_MULTI_THREADED_EXECUTOR__MULTI_THREADED_EXECUTOR_H_
