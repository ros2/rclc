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

#if __cplusplus
extern "C"
{
#include <string>
#endif /* if __cplusplus */

#include "rclc_parameter/parameter.h"

rcl_ret_t rclc_parameter_set_value_bool(
        rcl_interfaces__msg__Parameter* parameter,
        bool value)
{
    if (parameter->value.type != PARAMETER_BOOL)
    {
        return RCL_RET_INVALID_ARGUMENT;
    }

    parameter->value.bool_value = value;
    return RCL_RET_OK;
}

rcl_ret_t rclc_parameter_set_value_int(
        rcl_interfaces__msg__Parameter* parameter,
        int64_t value)
{
    if (parameter->value.type != PARAMETER_INTEGER)
    {
        return RCL_RET_INVALID_ARGUMENT;
    }

    parameter->value.integer_value = value;

    return RCL_RET_OK;
}

rcl_ret_t rclc_parameter_set_value_double(
        rcl_interfaces__msg__Parameter* parameter,
        double value)
{
    if (parameter->value.type != PARAMETER_DOUBLE)
    {
        return RCL_RET_INVALID_ARGUMENT;
    }

    parameter->value.double_value = value;

    return RCL_RET_OK;
}

rcl_ret_t rclc_parameter_set_value_string(
        rcl_interfaces__msg__Parameter* parameter,
        char* value)
{
    if (parameter->value.type != PARAMETER_STRING)
    {
        return RCL_RET_INVALID_ARGUMENT;
    }
    else if (rosidl_runtime_c__String__assign(&parameter->value.string_value, value))
    {
        return RCL_RET_OK;
    }

    return RCL_RET_ERROR;
}

rcl_ret_t
rclc_parameter_value_copy(
        rcl_interfaces__msg__ParameterValue* dst,
        const rcl_interfaces__msg__ParameterValue* src)
{
    RCL_CHECK_ARGUMENT_FOR_NULL(dst, RCL_RET_INVALID_ARGUMENT);
    RCL_CHECK_ARGUMENT_FOR_NULL(src, RCL_RET_INVALID_ARGUMENT);

    dst->type = src->type;

    switch (src->type){
        case PARAMETER_BOOL:
            dst->bool_value = src->bool_value;
            return RCL_RET_OK;
        case PARAMETER_INTEGER:
            dst->integer_value = src->integer_value;
            return RCL_RET_OK;
        case PARAMETER_DOUBLE:
            dst->double_value = src->double_value;
            return RCL_RET_OK;
        case PARAMETER_STRING:
            return rosidl_runtime_c__String__assign(
                &dst->string_value, src->string_value.data) ? RCL_RET_OK : RCL_RET_ERROR;
        case PARAMETER_NOT_SET:
        default:
            return RCL_RET_ERROR;
    }
    return RCL_RET_ERROR;
}

rcl_ret_t
rclc_parameter_copy(
        rcl_interfaces__msg__Parameter* dst,
        const rcl_interfaces__msg__Parameter* src)
{
    RCL_CHECK_ARGUMENT_FOR_NULL(dst, RCL_RET_INVALID_ARGUMENT);
    RCL_CHECK_ARGUMENT_FOR_NULL(src, RCL_RET_INVALID_ARGUMENT);
    if (!rosidl_runtime_c__String__assign(&dst->name, src->name.data))
    {
        return RCL_RET_ERROR;
    }
    return rclc_parameter_value_copy(&dst->value, &src->value);
}

rcl_interfaces__msg__Parameter* rclc_search_parameter(
        rcl_interfaces__msg__Parameter__Sequence* parameter_list,
        const char* param_name)
{
    for (size_t i = 0; i < parameter_list->size; i++)
    {
        if (!strcmp(param_name, parameter_list->data[i].name.data))
        {
            return &parameter_list->data[i];
        }
    }

    return NULL;
}

#if __cplusplus
}
#endif /* if __cplusplus */
