// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef RCL__PARAMETER_SERVICE_H_
#define RCL__PARAMETER_SERVICE_H_

#if __cplusplus
extern "C"
{
#endif

#include <rosidl_runtime_c/string_functions.h>
#include <rcl_interfaces/msg/detail/list_parameters_result__functions.h>
#include <rcl_interfaces/msg/detail/parameter_value__functions.h>
#include <rcl_interfaces/msg/detail/parameter__struct.h>
#include <rcl_interfaces/msg/detail/parameter_event__struct.h>
#include <rcl_interfaces/msg/detail/set_parameters_result__functions.h>

#include "rcl/node.h"
#include "rclc_parameter/parameter.h"
#include "rcl/service.h"
#include "rcl/wait.h"

struct rclc_parameter_service_impl_t;

typedef struct rclc_parameter_service_t
{
  struct rclc_parameter_service_impl_t * impl;
} rclc_parameter_service_t;

typedef struct rclc_parameter_service_options_t
{
  // quality of service settings for all services
  rmw_qos_profile_t qos;
  rmw_qos_profile_t parameter_event_qos;
  rcl_allocator_t allocator;
  char * remote_node_name;
} rclc_parameter_service_options_t;

RCL_PUBLIC
RCL_WARN_UNUSED
rclc_parameter_service_options_t
rclc_parameter_service_get_default_options(void);

RCL_PUBLIC
RCL_WARN_UNUSED
rclc_parameter_service_t
rclc_get_zero_initialized_parameter_service(void);


RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_service_init(
  rclc_parameter_service_t * parameter_service,
  rcl_node_t * node,
  const rclc_parameter_service_options_t * options
);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_service_init_default(
  rclc_parameter_service_t * parameter_service,
  rcl_node_t * node);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_service_fini(rclc_parameter_service_t * parameter_service);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_interfaces__msg__Parameter__Sequence *
rclc_parameter_service_take_set_request(
  const rclc_parameter_service_t * service,
  rmw_request_id_t * request_header);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_service_send_set_response(
  const rclc_parameter_service_t * service,
  rmw_request_id_t * request_header,
  const rcl_interfaces__msg__SetParametersResult__Sequence * set_parameter_results);

RCL_PUBLIC
RCL_WARN_UNUSED
rosidl_runtime_c__String__Sequence *
rclc_parameter_service_take_get_request(
  const rclc_parameter_service_t * service,
  rmw_request_id_t * request_header);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_service_send_get_response(
  const rclc_parameter_service_t * service,
  rmw_request_id_t * request_header,
  const rcl_interfaces__msg__ParameterValue__Sequence * parameters);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_service_send_get_types_response(
  const rclc_parameter_service_t * service,
  rmw_request_id_t * request_header,
  const rosidl_runtime_c__uint8__Sequence * parameter_types);

RCL_PUBLIC
RCL_WARN_UNUSED
rosidl_runtime_c__String__Sequence *
rclc_parameter_service_take_get_types_request(
  const rclc_parameter_service_t * service,
  rmw_request_id_t * request_header);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_interfaces__msg__Parameter__Sequence *
rclc_parameter_service_take_set_atomically_request(
  const rclc_parameter_service_t * service,
  rmw_request_id_t * request_header);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_service_send_set_atomically_response(
  const rclc_parameter_service_t * service,
  rmw_request_id_t * request_header,
  const rcl_interfaces__msg__SetParametersResult * set_parameters_result);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_service_take_list_request(
  const rclc_parameter_service_t * service,
  rmw_request_id_t * request_header,
  rosidl_runtime_c__String__Sequence * prefixes,
  uint64_t * depth);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_service_send_list_response(
  const rclc_parameter_service_t * service,
  rmw_request_id_t * request_header,
  const rcl_interfaces__msg__ListParametersResult * set_parameters_result);


RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_service_publish_event(
  const rclc_parameter_service_t * service,
  const rcl_interfaces__msg__ParameterEvent * event);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_wait_set_add_parameter_service(
  rcl_wait_set_t * wait_set,
  const rclc_parameter_service_t * parameter_service);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rcl_parameter_service_get_pending_action(
  const rcl_wait_set_t * wait_set,
  const rclc_parameter_service_t * parameter_service,
  rclc_param_action_t * action
);

#if __cplusplus
}
#endif

#endif  // RCL__PARAMETER_SERVICE_H_
