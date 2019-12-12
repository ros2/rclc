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

#ifndef RCLC__TYPES_H_
#define RCLC__TYPES_H_

#if __cplusplus
extern "C"
{
#endif

#include "rcl/rcl.h"

typedef rcl_ret_t rclc_ret_t;

typedef rcl_node_t rclc_node_t;

typedef rcl_publisher_t rclc_publisher_t;

typedef rcl_subscription_t rclc_subscription_t;

// TODO(Jan): Add rcl_ext_init_t type and rename it to rclc_init_t.

#if __cplusplus
}
#endif

#endif  // RCLC__TYPES_H_
