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

//#include <rclc/visibility_control.h>
#include <rcl/error_handling.h>

#include "rcl/types.h"

#include "../rcl_interfaces/include/parameter.h"
#include "../rcl_interfaces/include/parameter_value.h"

#define PARAMETER_NOT_SET 0
#define PARAMETER_BOOL 1
#define PARAMETER_INTEGER 2
#define PARAMETER_DOUBLE 3

//RCLC_PUBLIC
rcl_ret_t
rclc_parameter_set_value_bool(
        parameter__Parameter* parameter,
        bool value);

//RCLC_PUBLIC
rcl_ret_t
rclc_parameter_set_value_int(
        parameter__Parameter* parameter,
        int64_t value);

//RCLC_PUBLIC
rcl_ret_t
rclc_parameter_set_value_double(
        parameter__Parameter* parameter,
        double value);

//RCLC_PUBLIC
rcl_ret_t
rclc_parameter_copy(
        parameter__Parameter* dst,
        const parameter__Parameter* src);

//RCLC_PUBLIC
rcl_ret_t
rclc_parameter_value_copy(
        parameter__ParameterValue* dst,
        const parameter__ParameterValue* src);

//RCLC_PUBLIC
parameter__Parameter* rclc_search_parameter(
        parameter__Parameter__Sequence* parameter_list,
        const char* param_name);

#if __cplusplus
}
#endif // if __cplusplus

#endif  // RCL__PARAMETER_UTILS_H_
