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

#ifndef RCL__PARAMETER_H_
#define RCL__PARAMETER_H_

#if __cplusplus
extern "C"
{
#endif

#include "rcl/types.h"
#include "rcl/visibility_control.h"
#include "rcl/macros.h"
#include "rcl_interfaces/msg/parameter.h"
#include "rcl_interfaces/msg/parameter_event.h"
#include <rcl_interfaces/msg/detail/parameter_type__functions.h>

#define RCLC_PARAMETER_NUMBER_OF_SERVICES 6
#define RCLC_PARAMETER_NUMBER_OF_SUBSCRIPTIONS 1

typedef int rclc_param_action_t;
#define RCLC_GET_PARAMETERS 0
#define RCLC_GET_PARAMETER_TYPES 1
#define RCLC_SET_PARAMETERS 2
#define RCLC_SET_PARAMETERS_ATOMICALLY 3
#define RCLC_LIST_PARAMETERS 4
#define RCLC_DESCRIBE_PARAMETERS 5
#define RCLC_PARAMETER_ACTION_UNKNOWN 7

/// rcl_parameter_client_set_<TYPE> adds the parameter key, value pair to the
/// pending set_parameters_request

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_set_bool(
  rcl_interfaces__msg__Parameter * parameter, const char * parameter_name, bool value);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_set_integer(
  rcl_interfaces__msg__Parameter * parameter, const char * parameter_name, int64_t value);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_set_double(
  rcl_interfaces__msg__Parameter * parameter, const char * parameter_name, double value);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_set_string(
  rcl_interfaces__msg__Parameter * parameter,
  const char * parameter_name,
  const char * value);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_set_bytes(
  rcl_interfaces__msg__Parameter * parameter,
  const char * parameter_name,
  const uint8_t * value);

// etc. for all types

/// Get the boolean value of the parameter, or return error if the parameter is not a boolean.
RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_get_bool(const rcl_interfaces__msg__Parameter * parameter, bool * output);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_get_int(const rcl_interfaces__msg__Parameter * parameter, int64_t * output);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_get_double(const rcl_interfaces__msg__Parameter * parameter, double * output);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_get_string(
  const rcl_interfaces__msg__Parameter * parameter, rosidl_runtime_c__String * output);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_set_value_bool(
  rcl_interfaces__msg__ParameterValue * parameter_value, bool value);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_set_value_integer(
  rcl_interfaces__msg__ParameterValue * parameter_value, int64_t value);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_set_value_double(
  rcl_interfaces__msg__ParameterValue * parameter_value, double value);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_set_value_string(
  rcl_interfaces__msg__ParameterValue * parameter_value,
  const char * value);

RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_set_value_bytes(
  rcl_interfaces__msg__ParameterValue * parameter_value,
  const uint8_t * value);

// etc. for all types

// Some other convenience function ideas:
// Get type enum from a ParameterValue
// rclc_parameter_get_type(rcl_interfaces__msg__ParameterValue * parameter_value, uint8 type)

// Get type enum from a Parameter
// rclc_parameter_get_type(rcl_interfaces__msg__Parameter * parameter, uint8 type)

bool rclc_parameter_value_compare(
  const rcl_interfaces__msg__ParameterValue * parameter1,
  const rcl_interfaces__msg__ParameterValue * parameter2);

// diff the new state and the old state of parameters to populate a ParameterEvent message struct
RCL_PUBLIC
RCL_WARN_UNUSED
rcl_ret_t
rclc_parameter_convert_changes_to_event(
  const rcl_interfaces__msg__Parameter__Sequence * prior_state,
  const rcl_interfaces__msg__Parameter__Sequence * new_state,
  rcl_interfaces__msg__ParameterEvent * parameter_event);

#if __cplusplus
}
#endif

#endif  // RCL__PARAMETER_H_
