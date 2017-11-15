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

#include "rcl/rcl.h"

#include "rclc/type_support.h"

typedef rcl_ret_t rclc_ret_t;

typedef struct rclc_node_t rclc_node_t;

typedef struct rclc_publisher_t rclc_publisher_t;

typedef struct rclc_subscription_t rclc_subscription_t;

typedef struct rclc_executor_t rclc_executor_t;

typedef void (* rclc_callback_t)(const void *);

// impl
struct rclc_subscription_t
{
  rcl_subscription_t rcl_subscription;
  rclc_callback_t user_callback;

  rclc_type_support_t type_support;

  rclc_node_t * node;
};

struct rclc_publisher_t
{
  rcl_publisher_t rcl_publisher;

  rclc_node_t * node;
};

struct rclc_node_t
{
  rcl_node_t rcl_node;

  rclc_subscription_t ** subs;
  size_t subs_s;
};

#endif  // RCLC__TYPES_H_
