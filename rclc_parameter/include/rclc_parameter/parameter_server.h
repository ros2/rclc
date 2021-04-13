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

#ifndef RCL__PARAMETER_CLIENT_H_
#define RCL__PARAMETER_CLIENT_H_

#if __cplusplus
extern "C"
{
#endif // if __cplusplus

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "rcl/types.h"
#include "rcl/visibility_control.h"

#include <rcl_interfaces/msg/parameter_event.h>
#include <rcl_interfaces/msg/detail/set_parameters_result__functions.h>
#include <rcl_interfaces/srv/describe_parameters.h>
#include <rcl_interfaces/srv/get_parameters.h>
#include <rcl_interfaces/srv/list_parameters.h>
#include <rcl_interfaces/srv/set_parameters.h>

#include "rclc_parameter/parameter.h"
#include "rclc_parameter/config.h"
#include "rcl/publisher.h"

typedef struct rcl_parameter_server_t
{
    rcl_service_t get_service;
    //rcl_service_t get_types_service;
    rcl_service_t set_service;
    rcl_service_t list_service;

    rcl_publisher_t event_publisher;

    rcl_interfaces__srv__GetParameters_Request get_request;
    rcl_interfaces__srv__GetParameters_Response get_response;

    //rcl_interfaces__srv__GetParameterTypes_Request get_types_request;
    //rcl_interfaces__srv__GetParameterTypes_Response get_types_response;

    rcl_interfaces__srv__SetParameters_Request set_request;
    rcl_interfaces__srv__SetParameters_Response set_response;

    rcl_interfaces__srv__ListParameters_Request list_request;
    rcl_interfaces__srv__ListParameters_Response list_response;

    rcl_interfaces__srv__DescribeParameters_Request describe_request;
    rcl_interfaces__srv__DescribeParameters_Response describe_response;

    rcl_interfaces__msg__Parameter__Sequence parameter_list;

    rcl_interfaces__msg__ParameterEvent event_list;

    uint8_t param_number;
} rcl_parameter_server_t;

RCL_PUBLIC rcl_ret_t rclc_parameter_server_init_default(
        rcl_parameter_server_t* parameter_server,
        rcl_node_t* node);

RCL_PUBLIC rcl_ret_t rclc_parameter_server_fini(
        rcl_parameter_server_t* parameter_server,
        rcl_node_t* node);

RCL_PUBLIC rcl_ret_t rclc_executor_add_parameter_server(
        rclc_executor_t* executor,
        rcl_parameter_server_t* parameter_server);

RCL_PUBLIC
rcl_ret_t
rclc_add_parameter_bool(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        bool value);

RCL_PUBLIC
rcl_ret_t
rclc_add_parameter_int(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        int64_t value);

RCL_PUBLIC
rcl_ret_t
rclc_add_parameter_double(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        double value);

RCL_PUBLIC
rcl_ret_t
rclc_add_parameter_string(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        char* value);

RCL_PUBLIC
rcl_ret_t
rclc_parameter_set_bool(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        bool value);

RCL_PUBLIC
rcl_ret_t
rclc_parameter_set_int(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        int64_t value);

RCL_PUBLIC
rcl_ret_t
rclc_parameter_set_double(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        double value);

RCL_PUBLIC
rcl_ret_t
rclc_parameter_set_string(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        char* value);

RCL_PUBLIC
rcl_ret_t
rclc_parameter_get_bool(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        bool* output);

RCL_PUBLIC
rcl_ret_t
rclc_parameter_get_int(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        int64_t* output);

RCL_PUBLIC
rcl_ret_t
rclc_parameter_get_double(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        double* output);

RCL_PUBLIC
rcl_ret_t
rclc_parameter_get_string(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        char* output);

RCL_PUBLIC
rcl_ret_t
rclc_parameter_get_string_lenght(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        size_t* output);

rcl_ret_t rclc_parameter_server_init_service(
        rcl_service_t* service,
        rcl_node_t* node,
        char* service_name,
        const rosidl_service_type_support_t* srv_type);

rcl_ret_t rclc_parameter_service_publish_event(
        rcl_parameter_server_t* parameter_server);

#if __cplusplus
}
#endif // if __cplusplus

#endif  // RCL__PARAMETER_CLIENT_H_
