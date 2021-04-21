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

#include "../rcl_interfaces/include/parameter_event.h"
#include "../rcl_interfaces/include/set_parameters_result.h"
#include "../rcl_interfaces/include/get_parameters.h"
#include "../rcl_interfaces/include/get_parameter_types.h"
#include "../rcl_interfaces/include/list_parameters.h"
#include "../rcl_interfaces/include/set_parameters.h"

#include "parameter_server.h"
#include "parameter_utils.h"

#include "../rcl_interfaces/include/string_utils.h"
#include <time.h>

// TODO: add prefixes functionality
void rclc_parameter_server_list_service_callback(
        const void* req,
        void* res,
        void* parameter_server)
{
    //parameter__ListParameters_Request * request = (parameter__ListParameters_Request *) req;
    (void) req;

    parameter__ListParameters_Response* response = (parameter__ListParameters_Response*) res;
    rcl_parameter_server_t* param_server = (rcl_parameter_server_t*) parameter_server;

    response->result.names.size = param_server->parameter_list->size;

    for (size_t i = 0; i < response->result.names.size; i++)
    {
        parameter__String__assign(&response->result.names.data[i], param_server->parameter_list->data[i].name.data);
    }
}

void rclc_parameter_server_get_service_callback(
        const void* req,
        void* res,
        void* parameter_server)
{
    parameter__GetParameters_Request* request = (parameter__GetParameters_Request*) req;
    parameter__GetParameters_Response* response = (parameter__GetParameters_Response*) res;
    rcl_parameter_server_t* param_server = (rcl_parameter_server_t*) parameter_server;

    if(request->names.size > response->values.capacity)
    {
        response->values.size = 0;
        return;
    }

    response->values.size = request->names.size;

    for (size_t i = 0; i < response->values.size; i++)
    {
        // Clear previous values
        response->values.data[i].bool_value = false;
        response->values.data[i].integer_value = 0;
        response->values.data[i].double_value = 0;

        parameter__Parameter* parameter = rclc_search_parameter(param_server->parameter_list,
                        request->names.data[i].data);

        if (parameter != NULL)
        {
            rclc_parameter_value_copy(&response->values.data[i], &parameter->value);
        }
        else
        {
            response->values.data[i].type = PARAMETER_NOT_SET;
        }
    }
}

void rclc_parameter_server_get_types_service_callback(
        const void* req,
        void* res,
        void* parameter_server)
{
    parameter__GetParameterTypes_Request* request = (parameter__GetParameterTypes_Request *)  req;
    parameter__GetParameterTypes_Response* response = (parameter__GetParameterTypes_Response *) res;
    rcl_parameter_server_t* param_server = (rcl_parameter_server_t*) parameter_server;

    // TODO: fix request overflow on executor?
    if(request->names.size > response->types.capacity)
    {
        response->types.size = 0;
        return;
    }

    response->types.size = request->names.size;

    for (size_t i = 0; i < response->types.size; i++)
    {
        parameter__Parameter* parameter = rclc_search_parameter(param_server->parameter_list,
                        request->names.data[i].data);

        if (parameter != NULL)
        {
            response->types.data[i] = parameter->value.type;
        }
        else
        {
            response->types.data[i] = PARAMETER_NOT_SET;
        }
    }
}

void rclc_parameter_server_set_service_callback(
        const void* req,
        void* res,
        void* parameter_server)
{
    // Check if value is equal to stored and skip?

    parameter__SetParameters_Request* request = (parameter__SetParameters_Request*) req;
    parameter__SetParameters_Response* response = (parameter__SetParameters_Response*) res;
    rcl_parameter_server_t* param_server = (rcl_parameter_server_t*) parameter_server;

    parameter__Parameter__Sequence* changed_parameters =
            (parameter__Parameter__Sequence*) &param_server->event_list->changed_parameters;
    size_t index_params[request->parameters.size];
    size_t cont_changed = 0;

    if(request->parameters.size > response->results.capacity)
    {
        response->results.size = 0;
        return;
    }

    response->results.size = request->parameters.size;

    for (size_t i = 0; i < response->results.size; i++)
    {
        rosidl_runtime_c__String* message = (rosidl_runtime_c__String*) &response->results.data[i].reason.data;
        parameter__Parameter* parameter = rclc_search_parameter(param_server->parameter_list,
                        request->parameters.data[i].name.data);
        rcl_ret_t ret = RCL_RET_OK;

        if (parameter != NULL)
        {
            response->results.data[i].successful = true;
            message->data[0] = '\0';

            switch (request->parameters.data[i].value.type)
            {
                case PARAMETER_NOT_SET:
                    parameter__String__assign(message, "Type NOT SET");
                    response->results.data[i].successful = false;
                    break;

                case PARAMETER_BOOL:
                    ret = rclc_parameter_set_value_bool(parameter, request->parameters.data[i].value.bool_value);
                    break;

                case PARAMETER_INTEGER:
                    ret = rclc_parameter_set_value_int(parameter, request->parameters.data[i].value.integer_value);
                    break;

                case PARAMETER_DOUBLE:
                    ret = rclc_parameter_set_value_double(parameter, request->parameters.data[i].value.double_value);
                    break;

                default:
                    parameter__String__assign(message, "Type not supported");
                    response->results.data[i].successful = false;
                    break;
            }

            if (ret == RCL_RET_INVALID_ARGUMENT)
            {
                parameter__String__assign(message, "Type mismatch");
                response->results.data[i].successful = false;
            }
            else if (response->results.data[i].successful != false)
            {
                response->results.data[i].successful = true;
                index_params[cont_changed] = i;
                cont_changed++;
            }
        }
        else
        {
            parameter__String__assign(message, "Not found");
            response->results.data[i].successful = false;
        }
    }

    if (cont_changed > 0)
    {
        const char *parameters_changed[cont_changed];

        for (size_t j = 0; j < cont_changed; j++)
        {
            // TODO: add ret check
            parameters_changed[j] = request->parameters.data[index_params[j]].name.data;
            rclc_parameter_copy(&changed_parameters->data[j], &request->parameters.data[index_params[j]]);
            changed_parameters->size++;
        }
        
        rclc_parameter_service_publish_event(param_server);

        if (param_server->set_callback)
        {
            param_server->set_callback(param_server, parameters_changed, cont_changed);
        }
    }
}

rcl_ret_t rclc_parameter_server_init_default(
        rcl_parameter_server_t* parameter_server,
        size_t parameter_number,
        rcl_node_t* node)
{
    // TODO: Add ret check
    rcl_ret_t ret;

    const rosidl_service_type_support_t* get_ts = ROSIDL_GET_SRV_TYPE_SUPPORT(rcl_interfaces, srv, GetParameters);
    ret = rclc_parameter_server_init_service(&parameter_server->get_service, node, "/get_parameters", get_ts);

    const rosidl_service_type_support_t* get_types_ts = ROSIDL_GET_SRV_TYPE_SUPPORT(rcl_interfaces, srv, GetParameterTypes);
    ret = rclc_parameter_server_init_service(&parameter_server->get_types_service, node, "/get_parameter_types", get_types_ts);

    const rosidl_service_type_support_t* set_ts = ROSIDL_GET_SRV_TYPE_SUPPORT(rcl_interfaces, srv, SetParameters);
    ret = rclc_parameter_server_init_service(&parameter_server->set_service, node, "/set_parameters", set_ts);

    const rosidl_service_type_support_t* list_ts = ROSIDL_GET_SRV_TYPE_SUPPORT(rcl_interfaces, srv, ListParameters);
    ret = rclc_parameter_server_init_service(&parameter_server->list_service, node, "/list_parameters", list_ts);
    
    const rosidl_message_type_support_t* event_ts = ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, ParameterEvent);
    ret = rclc_publisher_init_default(&parameter_server->event_publisher, node, event_ts, "/parameter_events");

    parameter_server->parameter_list = parameter__Parameter__Sequence__create(parameter_number);

    // Request max size? User can ask for > parameter_number    
    parameter_server->get_request = parameter__GetParameters_Request__create(parameter_number);
    parameter_server->get_response = parameter__GetParameters_Response__create(parameter_number);

    parameter_server->get_types_request = parameter__GetParameterTypes_Request__create(parameter_number);
    parameter_server->get_types_response = parameter__GetParameterTypes_Response__create(parameter_number);

    parameter_server->set_request = parameter__SetParameters_Request__create(parameter_number);
    parameter_server->set_response = parameter__SetParameters_Response__create(parameter_number);

    parameter_server->list_request = parameter__ListParameters_Request__create(parameter_number);
    parameter_server->list_response = parameter__ListParameters_Response__create(parameter_number);

    const char* node_name = rcl_node_get_name(node);
    parameter_server->event_list = parameter__ParameterEvent__create(parameter_number);

    if (!parameter__String__assign(&parameter_server->event_list->node, node_name))
    {
        return RCL_RET_ERROR;
    }

    return ret;
}

rcl_ret_t rclc_parameter_server_fini(
        rcl_parameter_server_t* parameter_server,
        rcl_node_t* node)
{
    // TODO: Add ret check
    rcl_ret_t ret;

    ret = rcl_service_fini(&parameter_server->list_service, node);
    ret = rcl_service_fini(&parameter_server->set_service, node);
    ret = rcl_service_fini(&parameter_server->get_service, node);
    ret = rcl_service_fini(&parameter_server->get_types_service, node);
    ret = rcl_publisher_fini(&parameter_server->event_publisher, node);

    // Free memory first, in case service fini fails?
    parameter__Parameter__Sequence__destroy(parameter_server->parameter_list);

    parameter__GetParameters_Request__destroy(parameter_server->get_request);
    parameter__GetParameters_Response__destroy(parameter_server->get_response);

    parameter__GetParameterTypes_Request__destroy(parameter_server->get_types_request);
    parameter__GetParameterTypes_Response__destroy(parameter_server->get_types_response);

    parameter__SetParameters_Request__destroy(parameter_server->set_request);
    parameter__SetParameters_Response__destroy(parameter_server->set_response);

    parameter__ListParameters_Request__destroy(parameter_server->list_request);
    parameter__ListParameters_Response__destroy(parameter_server->list_response);

    parameter__ParameterEvent__destroy(parameter_server->event_list);

    return ret;
}

rcl_ret_t rclc_executor_add_parameter_server(
        rclc_executor_t* executor,
        rcl_parameter_server_t* parameter_server)
{
    // TODO: Add ret check
    rcl_ret_t ret;

    ret = rclc_executor_add_service_with_context(executor, &parameter_server->list_service,
                    parameter_server->list_request, parameter_server->list_response,
                    rclc_parameter_server_list_service_callback, parameter_server);

    ret = rclc_executor_add_service_with_context(executor, &parameter_server->get_types_service,
                    parameter_server->get_types_request, parameter_server->get_types_response,
                    rclc_parameter_server_get_types_service_callback, parameter_server);

    ret = rclc_executor_add_service_with_context(executor, &parameter_server->set_service,
                    parameter_server->set_request, parameter_server->set_response,
                    rclc_parameter_server_set_service_callback,
                    parameter_server);

    ret = rclc_executor_add_service_with_context(executor, &parameter_server->get_service,
                    parameter_server->get_request, parameter_server->get_response,
                    rclc_parameter_server_get_service_callback,
                    parameter_server);

    return ret;
}

rcl_ret_t rclc_parameter_server_add_callback(
        rcl_parameter_server_t* parameter_server,
        SetParameters_UserCallback callback)
{
    RCL_CHECK_ARGUMENT_FOR_NULL(parameter_server, RCL_RET_INVALID_ARGUMENT);
    RCL_CHECK_ARGUMENT_FOR_NULL(callback, RCL_RET_INVALID_ARGUMENT);

    parameter_server->set_callback = callback;
    return RCL_RET_OK;
}

rcl_ret_t rclc_add_parameter(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        int type)
{
    size_t index = parameter_server->parameter_list->size;

    if (index >= parameter_server->parameter_list->capacity ||
            rclc_search_parameter(parameter_server->parameter_list, parameter_name) != NULL)
    {
        return RCL_RET_ERROR;
    }
    else if (!parameter__String__assign(&parameter_server->parameter_list->data[index].name, parameter_name))
    {
        return RCL_RET_ERROR;
    }

    parameter_server->parameter_list->data[index].value.type = type;
    parameter_server->parameter_list->size++;
    
    if (rclc_parameter_copy(&parameter_server->event_list->new_parameters.data[0], &parameter_server->parameter_list->data[index]) == RCL_RET_ERROR)
    {
        return RCL_RET_ERROR;
    }

    parameter_server->event_list->new_parameters.size = 1;
    return rclc_parameter_service_publish_event(parameter_server);
}

rcl_ret_t rclc_parameter_set_bool(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        bool value)
{
    parameter__Parameter* parameter =
            rclc_search_parameter(parameter_server->parameter_list, parameter_name);

    if (parameter == NULL)
    {
        return RCL_RET_ERROR;
    }

    rcl_ret_t ret = rclc_parameter_set_value_bool(parameter, value);

    if (ret == RCL_RET_OK)
    {
        rclc_parameter_copy(&parameter_server->event_list->changed_parameters.data[0], parameter);
        parameter_server->event_list->changed_parameters.size = 1;
        rclc_parameter_service_publish_event(parameter_server);

        if (parameter_server->set_callback)
        {
            parameter_server->set_callback(parameter_server, &parameter_name, 1);
        }
    }

    return ret;
}

rcl_ret_t rclc_parameter_set_int(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        int64_t value)
{
    parameter__Parameter* parameter =
            rclc_search_parameter(parameter_server->parameter_list, parameter_name);

    if (parameter == NULL)
    {
        return RCL_RET_ERROR;
    }
    
    rcl_ret_t ret = rclc_parameter_set_value_int(parameter, value);

    if (ret == RCL_RET_OK)
    {
        rclc_parameter_copy(&parameter_server->event_list->changed_parameters.data[0], parameter);
        parameter_server->event_list->changed_parameters.size = 1;
        rclc_parameter_service_publish_event(parameter_server);

        if (parameter_server->set_callback)
        {
            parameter_server->set_callback(parameter_server, &parameter_name, 1);
        }
    }

    return ret;
}

rcl_ret_t rclc_parameter_set_double(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        double value)
{
    parameter__Parameter* parameter =
            rclc_search_parameter(parameter_server->parameter_list, parameter_name);

    if (parameter == NULL)
    {
        return RCL_RET_ERROR;
    }

    rcl_ret_t ret = rclc_parameter_set_value_double(parameter, value);

    if (ret == RCL_RET_OK)
    {
        rclc_parameter_copy(&parameter_server->event_list->changed_parameters.data[0], parameter);
        parameter_server->event_list->changed_parameters.size = 1;
        rclc_parameter_service_publish_event(parameter_server);
        
        if (parameter_server->set_callback)
        {
            parameter_server->set_callback(parameter_server, &parameter_name, 1);
        }
    }

    return ret;
}

rcl_ret_t rclc_parameter_get_bool(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        bool* output)
{
    parameter__Parameter* parameter =
            rclc_search_parameter(parameter_server->parameter_list, parameter_name);
    rcl_ret_t ret = RCL_RET_ERROR;

    if (parameter->value.type != PARAMETER_BOOL)
    {
        return RCL_RET_INVALID_ARGUMENT;
    }
    else if (parameter != NULL)
    {
        *output = parameter->value.bool_value;
        ret = RCL_RET_OK;
    }

    return ret;
}

rcl_ret_t rclc_parameter_get_int(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        int64_t* output)
{
    parameter__Parameter* parameter =
            rclc_search_parameter(parameter_server->parameter_list, parameter_name);
    rcl_ret_t ret = RCL_RET_ERROR;

    if (parameter->value.type != PARAMETER_INTEGER)
    {
        return RCL_RET_INVALID_ARGUMENT;
    }
    else if (parameter != NULL)
    {
        *output = parameter->value.integer_value;
        ret = RCL_RET_OK;
    }

    return ret;
}

rcl_ret_t rclc_parameter_get_double(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        double* output)
{
    parameter__Parameter* parameter =
            rclc_search_parameter(parameter_server->parameter_list, parameter_name);
    rcl_ret_t ret = RCL_RET_ERROR;

    if (parameter->value.type != PARAMETER_DOUBLE)
    {
        return RCL_RET_INVALID_ARGUMENT;
    }
    else if (parameter != NULL)
    {
        *output = parameter->value.double_value;
        ret = RCL_RET_OK;
    }

    return ret;
}

rcl_ret_t rclc_parameter_service_publish_event(
        rcl_parameter_server_t* parameter_server)
{
    RCL_CHECK_ARGUMENT_FOR_NULL(&parameter_server->event_publisher, RCL_RET_INVALID_ARGUMENT);
    RCL_CHECK_ARGUMENT_FOR_NULL(&parameter_server->event_list, RCL_RET_INVALID_ARGUMENT);

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    parameter_server->event_list->stamp.sec = ts.tv_sec;
    parameter_server->event_list->stamp.nanosec = ts.tv_nsec;

    rcl_ret_t ret = rcl_publish(&parameter_server->event_publisher, parameter_server->event_list, NULL);
    parameter_server->event_list->new_parameters.size = 0;
    parameter_server->event_list->changed_parameters.size = 0;

    return ret;
}

rcl_ret_t rclc_parameter_server_init_service(
        rcl_service_t* service,
        rcl_node_t* node,
        char* service_name,
        const rosidl_service_type_support_t* srv_type)
{
    const char* node_name = rcl_node_get_name(node);
    size_t getlen = strlen(node_name) + strlen(service_name) + 2;

    char get_service_name[getlen];
    memset(get_service_name, 0, getlen);
    memcpy(get_service_name, node_name, strlen(node_name) + 1);
    memcpy((get_service_name + strlen(node_name)), service_name, strlen(service_name) + 1);
    return rclc_service_init_default(service, node, srv_type, get_service_name);
}

#if __cplusplus
}
#endif /* if __cplusplus */
