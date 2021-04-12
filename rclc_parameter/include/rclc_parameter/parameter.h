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
#endif // if __cplusplus

#include <rcl/error_handling.h>

#include "rcl/types.h"
#include "rcl/visibility_control.h"
#include "rcl_interfaces/msg/parameter.h"
#include <rcl_interfaces/msg/detail/parameter_type__functions.h>

#include <rosidl_runtime_c/string_functions.h>
#include <rcl_interfaces/msg/detail/parameter_value__functions.h>

#define PARAMETER_NOT_SET rcl_interfaces__msg__ParameterType__PARAMETER_NOT_SET
#define PARAMETER_BOOL rcl_interfaces__msg__ParameterType__PARAMETER_BOOL
#define PARAMETER_INTEGER rcl_interfaces__msg__ParameterType__PARAMETER_INTEGER
#define PARAMETER_DOUBLE rcl_interfaces__msg__ParameterType__PARAMETER_DOUBLE
#define PARAMETER_STRING rcl_interfaces__msg__ParameterType__PARAMETER_STRING

RCL_PUBLIC
rcl_ret_t
rclc_parameter_set_bool(
        rcl_interfaces__msg__Parameter__Sequence* parameter_list,
        const char* parameter_name,
        bool value);

RCL_PUBLIC
rcl_ret_t
rclc_parameter_set_int(
        rcl_interfaces__msg__Parameter__Sequence* parameter_list,
        const char* parameter_name,
        int64_t value);

RCL_PUBLIC
rcl_ret_t
rclc_parameter_set_double(
        rcl_interfaces__msg__Parameter__Sequence* parameter_list,
        const char* parameter_name,
        double value);

RCL_PUBLIC
rcl_ret_t
rclc_parameter_set_string(
        rcl_interfaces__msg__Parameter__Sequence* parameter_list,
        const char* parameter_name,
        char* value);

RCL_PUBLIC
rcl_ret_t
rclc_parameter_get_bool(
        rcl_interfaces__msg__Parameter__Sequence* parameter_list,
        const char* parameter_name,
        bool* output);

RCL_PUBLIC
rcl_ret_t
rclc_parameter_get_int(
        rcl_interfaces__msg__Parameter__Sequence* parameter_list,
        const char* parameter_name,
        int64_t* output);

RCL_PUBLIC
rcl_ret_t
rclc_parameter_get_double(
        rcl_interfaces__msg__Parameter__Sequence* parameter_list,
        const char* parameter_name,
        double* output);

RCL_PUBLIC
rcl_ret_t
rclc_parameter_get_string(
        rcl_interfaces__msg__Parameter__Sequence* parameter_list,
        const char* parameter_name,
        char* output);

RCL_PUBLIC
rcl_ret_t
rclc_parameter_get_string_lenght(
        rcl_interfaces__msg__Parameter__Sequence* parameter_list,
        const char* parameter_name,
        size_t* output);

rcl_ret_t
rclc_parameter_value_copy(
        rcl_interfaces__msg__ParameterValue* dst,
        const rcl_interfaces__msg__ParameterValue* src);

int rclc_search_parameter_index(
        rcl_interfaces__msg__Parameter__Sequence* parameter_list,
        const char* param_name);

#if __cplusplus
}
#endif // if __cplusplus

#endif  // RCL__PARAMETER_H_
