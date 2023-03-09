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

rcl_ret_t
rclc_multi_threaded_executor_spin_init(rclc_executor_t * executor)
{
  RCLC_UNUSED(executor);
  return RCL_RET_OK;
}

rcl_ret_t
rclc_multi_threaded_executor_configure(rclc_executor_t * executor)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  executor->type = MULTI_THREADED;
  executor->spin_init = rclc_multi_threaded_executor_spin_init;
  return RCL_RET_OK;
}
