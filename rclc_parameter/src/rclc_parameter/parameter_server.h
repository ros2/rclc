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

#ifndef RCL__PARAMETER_SERVER_H_
#define RCL__PARAMETER_SERVER_H_

#if __cplusplus
extern "C"
{
#endif // if __cplusplus

#include "rclc_parameter/rclc_parameter.h"

#define RCLC_PARAMETER_MAX_STRING_LEN 30
#define RCLC_PARAMETER_MAX_PARAM 4

typedef struct rclc_parameter_static_memory_pool_t {
    Parameter parameter_list[RCLC_PARAMETER_MAX_PARAM];
    char parameter_list_names[RCLC_PARAMETER_MAX_PARAM][RCLC_PARAMETER_MAX_STRING_LEN];
    rosidl_runtime_c__String get_request_names[RCLC_PARAMETER_MAX_PARAM];
    char get_request_names_data[RCLC_PARAMETER_MAX_PARAM][RCLC_PARAMETER_MAX_STRING_LEN];
    ParameterValue get_response_values[RCLC_PARAMETER_MAX_PARAM];
    rosidl_runtime_c__String get_types_request_names[RCLC_PARAMETER_MAX_PARAM];
    char get_type_request_names_data[RCLC_PARAMETER_MAX_PARAM][RCLC_PARAMETER_MAX_STRING_LEN];
    uint8_t get_types_request_types[RCLC_PARAMETER_MAX_PARAM];
    Parameter set_request_parameters[RCLC_PARAMETER_MAX_PARAM];
    char set_request_parameters_name_data[RCLC_PARAMETER_MAX_PARAM][RCLC_PARAMETER_MAX_STRING_LEN];
    SetParameters_Result set_parameter_result[RCLC_PARAMETER_MAX_PARAM];
    char set_parameter_result_reason_data[RCLC_PARAMETER_MAX_PARAM][RCLC_PARAMETER_MAX_STRING_LEN];
    rosidl_runtime_c__String list_response_names[RCLC_PARAMETER_MAX_PARAM];
    char list_response_names_data[RCLC_PARAMETER_MAX_PARAM][RCLC_PARAMETER_MAX_STRING_LEN];
    Parameter event_list_new_parameters;
    char event_list_new_parameters_name_data[RCLC_PARAMETER_MAX_STRING_LEN];
    Parameter event_list_changed_parameters;
    char event_list_changed_parameters_name_data[RCLC_PARAMETER_MAX_STRING_LEN];
    char event_list_node_data[RCLC_PARAMETER_MAX_STRING_LEN];
} rclc_parameter_static_memory_pool_t;

rcl_ret_t rclc_parameter_server_init_service(
        rcl_service_t* service,
        rcl_node_t* node,
        char* service_name,
        const rosidl_service_type_support_t* srv_type);

rcl_ret_t rclc_parameter_service_publish_event(
        rclc_parameter_server_t* parameter_server);

#if __cplusplus
}
#endif // if __cplusplus

#endif  // RCL__PARAMETER_SERVER_H_
