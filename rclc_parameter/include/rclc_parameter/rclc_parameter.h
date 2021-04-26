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

#ifndef RCLC__PARAMETER_H_
#define RCLC__PARAMETER_H_

#if __cplusplus
extern "C"
{
#endif // if __cplusplus

#include <stdarg.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/types.h>

#include <rcl_interfaces/msg/parameter.h>
#include <rcl_interfaces/msg/parameter_event.h>
#include <rcl_interfaces/srv/get_parameter_types.h>
#include <rcl_interfaces/srv/get_parameters.h>
#include <rcl_interfaces/srv/list_parameters.h>
#include <rcl_interfaces/srv/set_parameters.h>

typedef struct rcl_interfaces__srv__GetParameters_Request GetParameters_Request;
typedef struct rcl_interfaces__srv__GetParameters_Response GetParameters_Response;

typedef struct rcl_interfaces__srv__GetParameterTypes_Request GetParameterTypes_Request;
typedef struct rcl_interfaces__srv__GetParameterTypes_Response GetParameterTypes_Response;

typedef struct rcl_interfaces__srv__SetParameters_Request SetParameters_Request;
typedef struct rcl_interfaces__srv__SetParameters_Response SetParameters_Response;
typedef struct rcl_interfaces__msg__SetParametersResult SetParameters_Result;

typedef struct rcl_interfaces__srv__ListParameters_Request ListParameters_Request;
typedef struct rcl_interfaces__srv__ListParameters_Response ListParameters_Response;

typedef struct rcl_interfaces__msg__Parameter Parameter;
typedef struct rcl_interfaces__msg__ParameterValue ParameterValue;
typedef struct rcl_interfaces__msg__Parameter__Sequence Parameter__Sequence;
typedef struct rcl_interfaces__msg__ParameterEvent ParameterEvent;

#define RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER 6

typedef void (* ModifiedParameter_Callback)(Parameter * param);

typedef enum rclc_parameter_type_t {
    RCLC_PARAMETER_NOT_SET = 0,
    RCLC_PARAMETER_BOOL,
    RCLC_PARAMETER_INT,
    RCLC_PARAMETER_DOUBLE
} rclc_parameter_type_t;

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

typedef struct rclc_parameter_server_t
{
    rcl_service_t get_service;
    rcl_service_t get_types_service;
    rcl_service_t set_service;
    rcl_service_t list_service;

    rcl_publisher_t event_publisher;

    rclc_parameter_static_memory_pool_t* static_pool;

    GetParameters_Request get_request;
    GetParameters_Response get_response;

    GetParameterTypes_Request get_types_request;
    GetParameterTypes_Response get_types_response;

    SetParameters_Request set_request;
    SetParameters_Response set_response;

    ListParameters_Request list_request;
    ListParameters_Response list_response;

    Parameter__Sequence parameter_list;

    ParameterEvent event_list;

    ModifiedParameter_Callback on_modification;
} rclc_parameter_server_t;

rcl_ret_t rclc_parameter_server_init_default(
        rclc_parameter_server_t* parameter_server,
        rcl_node_t* node);

rcl_ret_t rclc_parameter_server_fini(
        rclc_parameter_server_t* parameter_server,
        rcl_node_t* node);

rcl_ret_t rclc_executor_add_parameter_server(
        rclc_executor_t* executor,
        rclc_parameter_server_t* parameter_server,
        ModifiedParameter_Callback on_modification);

rcl_ret_t
rclc_add_parameter(
        rclc_parameter_server_t* parameter_server,
        const char* parameter_name,
        rclc_parameter_type_t type);

rcl_ret_t
rclc_parameter_set(
        rclc_parameter_server_t* parameter_server,
        const char* parameter_name,
        ...);

rcl_ret_t
rclc_parameter_set_bool(
        rclc_parameter_server_t* parameter_server,
        const char* parameter_name,
        bool value);

rcl_ret_t
rclc_parameter_set_int(
        rclc_parameter_server_t* parameter_server,
        const char* parameter_name,
        int64_t value);

rcl_ret_t
rclc_parameter_set_double(
        rclc_parameter_server_t* parameter_server,
        const char* parameter_name,
        double value);

rcl_ret_t
rclc_parameter_get(
        rclc_parameter_server_t* parameter_server,
        const char* parameter_name,
        void * value);

rcl_ret_t
rclc_parameter_get_bool(
        rclc_parameter_server_t* parameter_server,
        const char* parameter_name,
        bool* output);

rcl_ret_t
rclc_parameter_get_int(
        rclc_parameter_server_t* parameter_server,
        const char* parameter_name,
        int* output);

rcl_ret_t
rclc_parameter_get_double(
        rclc_parameter_server_t* parameter_server,
        const char* parameter_name,
        double* output);

#if __cplusplus
}
#endif // if __cplusplus

#endif  // RCL__PARAMETER_H_
