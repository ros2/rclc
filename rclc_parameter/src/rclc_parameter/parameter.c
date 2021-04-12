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

rcl_ret_t rclc_parameter_set_bool(
        rcl_interfaces__msg__Parameter__Sequence* parameter_list,
        const char* parameter_name,
        bool value)
{
    int index = rclc_search_parameter_index(parameter_list, parameter_name);
    rcl_ret_t ret = RCL_RET_ERROR;

    if (parameter_list->data[index].value.type != PARAMETER_BOOL)
    {
        ret = RCL_RET_INVALID_ARGUMENT;
    }
    else if (index != -1)
    {
        parameter_list->data[index].value.bool_value = value;
        ret = RCL_RET_OK;
    }

    return ret;
}

rcl_ret_t rclc_parameter_set_int(
        rcl_interfaces__msg__Parameter__Sequence* parameter_list,
        const char* parameter_name,
        int64_t value)
{
    int index = rclc_search_parameter_index(parameter_list, parameter_name);
    rcl_ret_t ret = RCL_RET_ERROR;

    if (parameter_list->data[index].value.type != PARAMETER_INTEGER)
    {
        ret = RCL_RET_INVALID_ARGUMENT;
    }
    else if (index != -1)
    {
        parameter_list->data[index].value.integer_value = value;
        ret = RCL_RET_OK;
    }

    return ret;
}

rcl_ret_t rclc_parameter_set_double(
        rcl_interfaces__msg__Parameter__Sequence* parameter_list,
        const char* parameter_name,
        double value)
{
    int index = rclc_search_parameter_index(parameter_list, parameter_name);
    rcl_ret_t ret = RCL_RET_ERROR;

    if (parameter_list->data[index].value.type != PARAMETER_DOUBLE)
    {
        ret = RCL_RET_INVALID_ARGUMENT;
    }
    else if (index != -1)
    {
        parameter_list->data[index].value.double_value = value;
        ret = RCL_RET_OK;
    }

    return ret;
}

rcl_ret_t rclc_parameter_set_string(
        rcl_interfaces__msg__Parameter__Sequence* parameter_list,
        const char* parameter_name,
        char* value)
{
    int index = rclc_search_parameter_index(parameter_list, parameter_name);
    rcl_ret_t ret = RCL_RET_ERROR;

    if (parameter_list->data[index].value.type != PARAMETER_STRING)
    {
        ret = RCL_RET_INVALID_ARGUMENT;
    }
    else if (index != -1 &&
            rosidl_runtime_c__String__assign(&parameter_list->data[index].value.string_value, value))
    {
        ret = RCL_RET_OK;
    }

    return ret;
}

rcl_ret_t rclc_parameter_get_bool(
        rcl_interfaces__msg__Parameter__Sequence* parameter_list,
        const char* parameter_name,
        bool* output)
{
    int index = rclc_search_parameter_index(parameter_list, parameter_name);
    rcl_ret_t ret = RCL_RET_ERROR;

    if (index != -1)
    {
        *output = parameter_list->data[index].value.bool_value;
        ret = RCL_RET_OK;
    }

    return ret;
}

rcl_ret_t rclc_parameter_get_int(
        rcl_interfaces__msg__Parameter__Sequence* parameter_list,
        const char* parameter_name,
        int64_t* output)
{
    int index = rclc_search_parameter_index(parameter_list, parameter_name);
    rcl_ret_t ret = RCL_RET_ERROR;

    if (index != -1)
    {
        *output = parameter_list->data[index].value.integer_value;
        ret = RCL_RET_OK;
    }

    return ret;
}

rcl_ret_t rclc_parameter_get_double(
        rcl_interfaces__msg__Parameter__Sequence* parameter_list,
        const char* parameter_name,
        double* output)
{
    int index = rclc_search_parameter_index(parameter_list, parameter_name);
    rcl_ret_t ret = RCL_RET_ERROR;

    if (index != -1)
    {
        *output = parameter_list->data[index].value.double_value;
        ret = RCL_RET_OK;
    }

    return ret;
}

// Add max_lenght for output?
rcl_ret_t rclc_parameter_get_string(
        rcl_interfaces__msg__Parameter__Sequence* parameter_list,
        const char* parameter_name,
        char* output)
{
    int index = rclc_search_parameter_index(parameter_list, parameter_name);
    rcl_ret_t ret = RCL_RET_ERROR;

    if (index != -1)
    {
        memcpy(output, parameter_list->data[index].value.string_value.data,
                parameter_list->data[index].value.string_value.capacity);
        ret = RCL_RET_OK;
    }

    return ret;
}

rcl_ret_t rclc_parameter_get_string_lenght(
        rcl_interfaces__msg__Parameter__Sequence* parameter_list,
        const char* parameter_name,
        size_t* output)
{
    int index = rclc_search_parameter_index(parameter_list, parameter_name);
    rcl_ret_t ret = RCL_RET_ERROR;

    if (index != -1)
    {
        *output = parameter_list->data[index].value.string_value.capacity;
        ret = RCL_RET_OK;
    }

    return ret;
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

int rclc_search_parameter_index(
        rcl_interfaces__msg__Parameter__Sequence* parameter_list,
        const char* param_name)
{
    for (size_t i = 0; i < parameter_list->size; i++)
    {
        if (!strcmp(param_name, parameter_list->data[i].name.data))
        {
            return i;
        }
    }

    return -1;
}

#if __cplusplus
}
#endif /* if __cplusplus */
