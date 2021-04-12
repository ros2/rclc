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

#include "rclc_parameter/parameter_server.h"

// TODO: add prefixes functionality
void rclc_parameter_server_list_service_callback(
        const void* req,
        void* res,
        void* param_server)
{
    //rcl_interfaces__srv__ListParameters_Request * request = (rcl_interfaces__srv__ListParameters_Request *) req;
    (void) req;

    rcl_interfaces__srv__ListParameters_Response* response = (rcl_interfaces__srv__ListParameters_Response*) res;
    rcl_parameter_server_t* parameters = (rcl_parameter_server_t*) param_server;

    // Check call to rosidl_runtime_c__String__Sequence__fini
    rosidl_runtime_c__String__Sequence__init(&response->result.names, (size_t)  parameters->param_number);
    rosidl_runtime_c__String__Sequence__init(&response->result.prefixes, (size_t) 1);

    for (size_t i = 0; i < parameters->param_number; i++)
    {
        rosidl_runtime_c__String__assign(&response->result.names.data[i], parameters->parameter_list.data[i].name.data);
    }
}

void rclc_parameter_server_get_service_callback(
        const void* req,
        void* res,
        void* parameter_list)
{
    rcl_interfaces__srv__GetParameters_Request* request = (rcl_interfaces__srv__GetParameters_Request*) req;
    rcl_interfaces__srv__GetParameters_Response* response = (rcl_interfaces__srv__GetParameters_Response*) res;
    rcl_interfaces__msg__Parameter__Sequence* parameters = (rcl_interfaces__msg__Parameter__Sequence*) parameter_list;

    rcl_interfaces__msg__ParameterValue__Sequence__init(&response->values, request->names.size);

    for (size_t i = 0; i < request->names.size; i++)
    {
        int index = rclc_search_parameter_index(parameter_list, request->names.data[i].data);

        if (index != -1)
        {
            rclc_parameter_value_copy(&response->values.data[i], &parameters->data[index].value);
        }
    }
}

void rclc_parameter_server_set_service_callback(
        const void* req,
        void* res,
        void* parameter_list)
{
    rcl_interfaces__srv__SetParameters_Request* request = (rcl_interfaces__srv__SetParameters_Request*) req;
    rcl_interfaces__srv__SetParameters_Response* response = (rcl_interfaces__srv__SetParameters_Response*) res;
    rcl_interfaces__msg__Parameter__Sequence* parameters = (rcl_interfaces__msg__Parameter__Sequence*) parameter_list;

    rcl_interfaces__msg__SetParametersResult__Sequence__init(&response->results, request->parameters.size);

    for (size_t i = 0; i < request->parameters.size; i++)
    {
        rosidl_runtime_c__String* message = (rosidl_runtime_c__String*) &response->results.data[i].reason.data;
        rcl_ret_t ret = RCL_RET_OK;

        switch (request->parameters.data[i].value.type)
        {
            case PARAMETER_NOT_SET:
                rosidl_runtime_c__String__assignn(message, "Parameter type NOT SET", 25);
                response->results.data[i].successful = false;
                break;

            case PARAMETER_BOOL:
                ret = rclc_parameter_set_bool(parameters, request->parameters.data[i].name.data,
                                request->parameters.data[i].value.bool_value);
                response->results.data[i].successful = true;
                break;

            case PARAMETER_INTEGER:
                ret = rclc_parameter_set_int(parameters, request->parameters.data[i].name.data,
                                request->parameters.data[i].value.integer_value);
                response->results.data[i].successful = true;
                break;

            case PARAMETER_DOUBLE:
                ret = rclc_parameter_set_double(parameters, request->parameters.data[i].name.data,
                                request->parameters.data[i].value.double_value);
                response->results.data[i].successful = true;
                break;

            case PARAMETER_STRING:
                ret = rclc_parameter_set_string(parameters, request->parameters.data[i].name.data,
                                request->parameters.data[i].value.string_value.data);
                response->results.data[i].successful = true;
                break;

            default:
                rosidl_runtime_c__String__assignn(message, "Parameter type not supported", 30);
                response->results.data[i].successful = false;
                break;
        }

        if (ret == RCL_RET_INVALID_ARGUMENT)
        {
            rosidl_runtime_c__String__assignn(message, "Parameter type mismatch", 25);
            response->results.data[i].successful = false;
        }
        else if (ret == RCL_RET_ERROR)
        {
            rosidl_runtime_c__String__assignn(message, "Parameter not found", 20);
            response->results.data[i].successful = false;
        }
        else
        {
            response->results.data[i].successful = true;
        }
    }
}

rcl_ret_t rclc_parameter_server_init_default(
        rcl_parameter_server_t* param_server,
        rcl_node_t* node)
{
    // TODO: Add ret check
    rcl_ret_t ret;

    param_server->param_number = 0;
    rcl_interfaces__msg__Parameter__Sequence__init(&param_server->parameter_list, RCLC_UXRCE_MAX_PARAMETERS);

    for (size_t i = 0; i < RCLC_UXRCE_MAX_PARAMETERS; i++)
    {
        rosidl_runtime_c__String__init(&param_server->parameter_list.data[i].name);
        param_server->parameter_list.data[i].value.type = PARAMETER_NOT_SET;
    }

    const rosidl_service_type_support_t* get_ts = ROSIDL_GET_SRV_TYPE_SUPPORT(rcl_interfaces, srv, GetParameters);
    ret = rclc_parameter_server_init_service(&param_server->get_service, node, "/get_parameters", get_ts);

    const rosidl_service_type_support_t* set_ts = ROSIDL_GET_SRV_TYPE_SUPPORT(rcl_interfaces, srv, SetParameters);
    ret = rclc_parameter_server_init_service(&param_server->set_service, node, "/set_parameters", set_ts);

    const rosidl_service_type_support_t* list_ts = ROSIDL_GET_SRV_TYPE_SUPPORT(rcl_interfaces, srv, ListParameters);
    ret = rclc_parameter_server_init_service(&param_server->list_service, node, "/list_parameters", list_ts);

    rcl_interfaces__srv__GetParameters_Request__init(&param_server->get_request);
    rcl_interfaces__srv__GetParameters_Response__init(&param_server->get_response);

    rcl_interfaces__srv__SetParameters_Request__init(&param_server->set_request);
    rcl_interfaces__srv__SetParameters_Response__init(&param_server->set_response);

    rcl_interfaces__srv__ListParameters_Request__init(&param_server->list_request);
    rcl_interfaces__srv__ListParameters_Response__init(&param_server->list_response);

    return ret;
}

rcl_ret_t rclc_parameter_server_fini(
        rcl_parameter_server_t* param_server,
        rcl_node_t* node)
{
    // TODO: Add ret check
    rcl_ret_t ret;
    rcl_interfaces__msg__Parameter__Sequence__fini(&param_server->parameter_list);
    param_server->param_number = 0;

    ret = rcl_service_fini(&param_server->list_service, node);
    ret = rcl_service_fini(&param_server->set_service, node);
    ret = rcl_service_fini(&param_server->get_service, node);

    rcl_interfaces__srv__GetParameters_Request__fini(&param_server->get_request);
    rcl_interfaces__srv__GetParameters_Response__fini(&param_server->get_response);

    rcl_interfaces__srv__SetParameters_Request__fini(&param_server->set_request);
    rcl_interfaces__srv__SetParameters_Response__fini(&param_server->set_response);

    rcl_interfaces__srv__ListParameters_Request__fini(&param_server->list_request);
    rcl_interfaces__srv__ListParameters_Response__fini(&param_server->list_response);

    return ret;
}

rcl_ret_t rclc_executor_add_parameter_server(
        rclc_executor_t* executor,
        rcl_parameter_server_t* param_server)
{
    // TODO: Add ret check
    rcl_ret_t ret;

    ret = rclc_executor_add_service_with_context(executor, &param_server->list_service,
                    &param_server->list_request, &param_server->list_response,
                    rclc_parameter_server_list_service_callback, param_server);

    ret = rclc_executor_add_service_with_context(executor, &param_server->set_service,
                    &param_server->set_request, &param_server->set_response, rclc_parameter_server_set_service_callback,
                    &param_server->parameter_list);

    ret = rclc_executor_add_service_with_context(executor, &param_server->get_service,
                    &param_server->get_request, &param_server->get_response, rclc_parameter_server_get_service_callback,
                    &param_server->parameter_list);

    return ret;
}

rcl_ret_t rclc_add_parameter_bool(
        rcl_parameter_server_t* param_server,
        const char* parameter_name,
        bool value)
{
    if (param_server->param_number >= RCLC_UXRCE_MAX_PARAMETERS)
    {
        return RCL_RET_ERROR;
    }

    if (!rosidl_runtime_c__String__assign(&param_server->parameter_list.data[param_server->param_number].name,
            parameter_name))
    {
        return RCL_RET_ERROR;
    }

    param_server->parameter_list.data[param_server->param_number].value.type = PARAMETER_BOOL;
    param_server->parameter_list.data[param_server->param_number].value.bool_value = value;
    param_server->param_number++;

    return RCL_RET_OK;
}

rcl_ret_t rclc_add_parameter_int(
        rcl_parameter_server_t* param_server,
        const char* parameter_name,
        int64_t value)
{
    if (param_server->param_number >= RCLC_UXRCE_MAX_PARAMETERS ||
            rclc_search_parameter_index(&param_server->parameter_list, parameter_name) != -1)
    {
        return RCL_RET_ERROR;
    }
    else if (!rosidl_runtime_c__String__assign(&param_server->parameter_list.data[param_server->param_number].name,
            parameter_name))
    {
        return RCL_RET_ERROR;
    }

    param_server->parameter_list.data[param_server->param_number].value.type = PARAMETER_INTEGER;
    param_server->parameter_list.data[param_server->param_number].value.integer_value = value;
    param_server->param_number++;

    return RCL_RET_OK;
}

rcl_ret_t rclc_add_parameter_double(
        rcl_parameter_server_t* param_server,
        const char* parameter_name,
        double value)
{
    if (param_server->param_number >= RCLC_UXRCE_MAX_PARAMETERS ||
            rclc_search_parameter_index(&param_server->parameter_list, parameter_name) != -1)
    {
        return RCL_RET_ERROR;
    }
    else if (!rosidl_runtime_c__String__assign(&param_server->parameter_list.data[param_server->param_number].name,
            parameter_name))
    {
        return RCL_RET_ERROR;
    }

    param_server->parameter_list.data[param_server->param_number].value.type = PARAMETER_DOUBLE;
    param_server->parameter_list.data[param_server->param_number].value.double_value = value;
    param_server->param_number++;

    return RCL_RET_OK;
}

rcl_ret_t rclc_add_parameter_string(
        rcl_parameter_server_t* param_server,
        const char* parameter_name,
        char* value)
{
    if (param_server->param_number >= RCLC_UXRCE_MAX_PARAMETERS ||
            rclc_search_parameter_index(&param_server->parameter_list, parameter_name) != -1)
    {
        return RCL_RET_ERROR;
    }
    else if (!rosidl_runtime_c__String__assign(&param_server->parameter_list.data[param_server->param_number].name,
            parameter_name))
    {
        return RCL_RET_ERROR;
    }

    param_server->parameter_list.data[param_server->param_number].value.type = PARAMETER_STRING;
    rosidl_runtime_c__String__assign(&param_server->parameter_list.data[param_server->param_number].value.string_value,
            value);
    param_server->param_number++;

    return RCL_RET_OK;
}

rcl_ret_t rclc_parameter_server_init_service(
        rcl_service_t* service,
        rcl_node_t* node,
        char* service_name,
        const rosidl_service_type_support_t* srv_type)
{
    const char* node_name = rcl_node_get_name(node);
    size_t getlen = strlen(node_name) + strlen(service_name) + 2;

    // Need to use allocator for get_service_name?
    char get_service_name[getlen];
    memset(get_service_name, 0, getlen);
    memcpy(get_service_name, node_name, strlen(node_name) + 1);
    memcpy((get_service_name + strlen(node_name)), service_name, strlen(service_name) + 1);

    return rclc_service_init_default(service, node, srv_type, get_service_name);
}

#if __cplusplus
}
#endif /* if __cplusplus */
