// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef RCLC__TYPE_SUPPORT_H_
#define RCLC__TYPE_SUPPORT_H_

#include <stddef.h>

#include "rosidl_generator_c/message_type_support_struct.h"

typedef struct rclc_type_support_t rclc_type_support_t;
struct rclc_type_support_t
{
  const rosidl_message_type_support_t * rosidl_message_type_support;
  const size_t size_of;
};

#define RCLC_GET_MSG_TYPE_SUPPORT(pkg, dir, msg) \
  ((const rclc_type_support_t) { \
    ROSIDL_GET_MSG_TYPE_SUPPORT(pkg, dir, msg), \
    sizeof(pkg ## __ ## dir ## __ ## msg) \
  })

#endif  // RCLC__TYPE_SUPPORT_H_
