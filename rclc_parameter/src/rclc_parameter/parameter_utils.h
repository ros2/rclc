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

#ifndef RCL__PARAMETER_UTILS_H_
#define RCL__PARAMETER_UTILS_H_

#if __cplusplus
extern "C"
{
#endif // if __cplusplus

#include <rcl/error_handling.h>
#include <rcl_interfaces/msg/parameter.h>

#include "rcl/types.h"
#include <rclc_parameter/rclc_parameter.h>

rcl_ret_t
rclc_parameter_set_value_bool(
        Parameter* parameter,
        bool value);

rcl_ret_t
rclc_parameter_set_value_int(
        Parameter* parameter,
        int64_t value);

rcl_ret_t
rclc_parameter_set_value_double(
        Parameter* parameter,
        double value);

rcl_ret_t
rclc_parameter_value_copy(
        ParameterValue* dst,
        const ParameterValue* src);

rcl_ret_t
rclc_parameter_copy(
        Parameter* dst,
        const Parameter* src);

Parameter* rclc_parameter_search(
        Parameter__Sequence* parameter_list,
        const char* param_name);

bool rclc_parameter_set_string(
        rosidl_runtime_c__String* string,
        const char* value);

#if __cplusplus
}
#endif // if __cplusplus

#endif  // RCL__PARAMETER_UTILS_H_
