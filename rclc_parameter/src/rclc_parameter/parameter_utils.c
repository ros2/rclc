// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
#endif /* if __cplusplus */

#include <string.h>

#include "parameter_utils.h"
#include <rclc_parameter/rclc_parameter.h>

rcl_ret_t
rclc_parameter_value_copy(
        ParameterValue* dst,
        const ParameterValue* src)
{
    RCL_CHECK_ARGUMENT_FOR_NULL(dst, RCL_RET_INVALID_ARGUMENT);
    RCL_CHECK_ARGUMENT_FOR_NULL(src, RCL_RET_INVALID_ARGUMENT);

    dst->type = src->type;

    switch (src->type){
        case RCLC_PARAMETER_BOOL:
            dst->bool_value = src->bool_value;
            return RCL_RET_OK;
        case RCLC_PARAMETER_INT:
            dst->integer_value = src->integer_value;
            return RCL_RET_OK;
        case RCLC_PARAMETER_DOUBLE:
            dst->double_value = src->double_value;
            return RCL_RET_OK;
        case RCLC_PARAMETER_NOT_SET:
        default:
            dst->type = RCLC_PARAMETER_NOT_SET;
            return RCL_RET_ERROR;
    }
    return RCL_RET_ERROR;
}

rcl_ret_t
rclc_parameter_copy(
        Parameter* dst,
        const Parameter* src)
{
    RCL_CHECK_ARGUMENT_FOR_NULL(dst, RCL_RET_INVALID_ARGUMENT);
    RCL_CHECK_ARGUMENT_FOR_NULL(src, RCL_RET_INVALID_ARGUMENT);
    
    rclc_parameter_set_string(&dst->name, src->name.data);
    return rclc_parameter_value_copy(&dst->value, &src->value);
}

Parameter* rclc_parameter_search(
        Parameter__Sequence* parameter_list,
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

bool rclc_parameter_set_string(
        rosidl_runtime_c__String* string,
        const char* value)
{
    uint32_t len = strlen(value);
    if (string->capacity >= len)
    {
        memcpy(string->data, value, len);
        string->size = len;
        return true;
    }
    string->size = 0;
    return false;
}

void rclc_parameter_prepare_parameter_event(
        ParameterEvent* event,
        Parameter* parameter,
        bool new)
{   
    memset(&event->new_parameters, 0, sizeof(Parameter__Sequence));
    memset(&event->changed_parameters, 0, sizeof(Parameter__Sequence));

    Parameter__Sequence* seq = (new) ? 
        &event->new_parameters : 
        &event->changed_parameters;

    seq->data = parameter;
    seq->capacity = 1;
    seq->size = 1;
}

static char null_string = '\0';

void rclc_parameter_init_string(rosidl_runtime_c__String* dst, char* ptr, size_t capacity){
    dst->data = ptr;
    dst->capacity = capacity;
    dst->size = 0;
}

void rclc_parameter_init_parametervalue(rcl_interfaces__msg__ParameterValue* dst){
    rclc_parameter_init_string(&dst->string_value, &null_string, 1);
    rclc_parameter_init_empty_sequence(dst->byte_array_value);
    rclc_parameter_init_empty_sequence(dst->bool_array_value);
    rclc_parameter_init_empty_sequence(dst->integer_array_value);
    rclc_parameter_init_empty_sequence(dst->double_array_value);
    rclc_parameter_init_empty_sequence(dst->string_array_value);
}
        

#if __cplusplus
}
#endif /* if __cplusplus */