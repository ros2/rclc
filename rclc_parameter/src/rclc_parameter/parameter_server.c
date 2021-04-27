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

#include <rclc_parameter/rclc_parameter.h>

#include "parameter_server.h"
#include "parameter_utils.h"
#include "microros_static_memory_manager.h"

#include <time.h>

#define RCLC_PARAMETER_SERVICE_MAX_LENGHT 50

void rclc_parameter_server_list_service_callback(
        const void* req,
        void* res,
        void* parameter_server)
{
    (void) req;

    rclc_parameter_server_t* param_server = (rclc_parameter_server_t*) parameter_server;
    ListParameters_Response* response = (ListParameters_Response*) res;

    response->result.names.size = param_server->parameter_list.size;

    for (size_t i = 0; i < response->result.names.size; i++)
    {
        rclc_parameter_set_string(&response->result.names.data[i], param_server->parameter_list.data[i].name.data);
    }
}

void rclc_parameter_server_get_service_callback(
        const void* req,
        void* res,
        void* parameter_server)
{
    GetParameters_Request* request = (GetParameters_Request*) req;
    GetParameters_Response* response = (GetParameters_Response*) res;
    rclc_parameter_server_t* param_server = (rclc_parameter_server_t*) parameter_server;

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

        Parameter* parameter = rclc_parameter_search(&param_server->parameter_list,
                        request->names.data[i].data);

        if (parameter != NULL)
        {
            rclc_parameter_value_copy(&response->values.data[i], &parameter->value);
        }
        else
        {
            response->values.data[i].type = RCLC_PARAMETER_NOT_SET;
        }
    }
}

void rclc_parameter_server_get_types_service_callback(
        const void* req,
        void* res,
        void* parameter_server)
{
    GetParameterTypes_Request* request = (GetParameterTypes_Request *)  req;
    GetParameterTypes_Response* response = (GetParameterTypes_Response *) res;
    rclc_parameter_server_t* param_server = (rclc_parameter_server_t*) parameter_server;

    if(request->names.size > response->types.capacity)
    {
        response->types.size = 0;
        return;
    }

    response->types.size = request->names.size;

    for (size_t i = 0; i < response->types.size; i++)
    {
        Parameter* parameter = rclc_parameter_search(&param_server->parameter_list,
                        request->names.data[i].data);

        if (parameter != NULL)
        {
            response->types.data[i] = parameter->value.type;
        }
        else
        {
            response->types.data[i] = RCLC_PARAMETER_NOT_SET;
        }
    }
}

void rclc_parameter_server_set_service_callback(
        const void* req,
        void* res,
        void* parameter_server)
{
    SetParameters_Request* request = (SetParameters_Request*) req;
    SetParameters_Response* response = (SetParameters_Response*) res;
    rclc_parameter_server_t* param_server = (rclc_parameter_server_t*) parameter_server;

    if(request->parameters.size > response->results.capacity)
    {
        response->results.size = 0;
        return;
    }

    response->results.size = request->parameters.size;

    for (size_t i = 0; i < response->results.size; i++)
    {
        rosidl_runtime_c__String* message = (rosidl_runtime_c__String*) &response->results.data[i].reason.data;
        Parameter* parameter = rclc_parameter_search(&param_server->parameter_list,
                        request->parameters.data[i].name.data);
        rcl_ret_t ret = RCL_RET_OK;

        if (parameter != NULL)
        {
            response->results.data[i].successful = true;
            message->data[0] = '\0';

            if (parameter->value.type != request->parameters.data[i].value.type)
            {
                strcpy(message->data, "Type mismatch");
                message->size = strlen("Type mismatch");
                response->results.data[i].successful = false;
                continue;
            }
            
            switch (request->parameters.data[i].value.type)
            {
                case RCLC_PARAMETER_NOT_SET:
                    strcpy(message->data, "Type not set");
                    message->size = strlen("Type not set");
                    response->results.data[i].successful = false;
                    break;

                case RCLC_PARAMETER_BOOL:
                    ret = rclc_parameter_set(parameter_server, parameter->name.data, request->parameters.data[i].value.bool_value);
                    break;
                case RCLC_PARAMETER_INT:
                    ret = rclc_parameter_set(parameter_server, parameter->name.data, request->parameters.data[i].value.integer_value);
                    break;
                case RCLC_PARAMETER_DOUBLE:
                    ret = rclc_parameter_set(parameter_server, parameter->name.data, request->parameters.data[i].value.double_value);
                    break;

                default:
                    strcpy(message->data, "Type not supported");
                    message->size = strlen("Type not supported");
                    response->results.data[i].successful = false;
                    break;
            }

            if (ret == RCL_RET_INVALID_ARGUMENT)
            {
                strcpy(message->data, "Set parameter error");
                message->size = strlen("Set parameter error");
                response->results.data[i].successful = false;
            }
        }
        else
        {
            strcpy(message->data, "Parameter not found");
            message->size = strlen("Parameter not found");
            response->results.data[i].successful = false;
        }
    }
}

rcl_ret_t rclc_parameter_server_init_default(
        rclc_parameter_server_t* parameter_server,
        rcl_node_t* node)
{
    rclc_parameter_options_t opts = {.notify_changed_over_dds = true};
    return rclc_parameter_server_init_with_option(parameter_server, node, &opts);
}

microros_sm_create_memory(rclc_parameter_static_pool, rclc_parameter_static_memory_pool_t, RCLC_PARAMETER_MAX_NODES, true)

rcl_ret_t rclc_parameter_server_init_with_option(
        rclc_parameter_server_t* parameter_server,
        rcl_node_t* node,
        rclc_parameter_options_t * options)
{
    RCL_CHECK_ARGUMENT_FOR_NULL(parameter_server, RCL_RET_INVALID_ARGUMENT);
    RCL_CHECK_ARGUMENT_FOR_NULL(node, RCL_RET_INVALID_ARGUMENT);

    rcl_ret_t ret;

    const rosidl_service_type_support_t* get_ts = ROSIDL_GET_SRV_TYPE_SUPPORT(rcl_interfaces, srv, GetParameters);
    ret = rclc_parameter_server_init_service(&parameter_server->get_service, node, "/get_parameters", get_ts);

    const rosidl_service_type_support_t* get_types_ts = ROSIDL_GET_SRV_TYPE_SUPPORT(rcl_interfaces, srv, GetParameterTypes);
    ret &= rclc_parameter_server_init_service(&parameter_server->get_types_service, node, "/get_parameter_types", get_types_ts);

    const rosidl_service_type_support_t* set_ts = ROSIDL_GET_SRV_TYPE_SUPPORT(rcl_interfaces, srv, SetParameters);
    ret &= rclc_parameter_server_init_service(&parameter_server->set_service, node, "/set_parameters", set_ts);

    const rosidl_service_type_support_t* list_ts = ROSIDL_GET_SRV_TYPE_SUPPORT(rcl_interfaces, srv, ListParameters);
    ret &= rclc_parameter_server_init_service(&parameter_server->list_service, node, "/list_parameters", list_ts);
    
    parameter_server->notify_changed_over_dds = options->notify_changed_over_dds;
    if (parameter_server->notify_changed_over_dds)
    {
        const rosidl_message_type_support_t* event_ts = ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, ParameterEvent);
        ret &= rclc_publisher_init_default(&parameter_server->event_publisher, node, event_ts, "/parameter_events");
    }

    // Init rclc_parameter static memory pools
    if (!microros_sm_is_init(rclc_parameter_static_pool))
    {
        microros_sm_init_memory(rclc_parameter_static_pool);
    }

    parameter_server->static_pool = microros_sm_get_memory(rclc_parameter_static_pool);

    // Set all memebers to zero
    memset(&parameter_server->get_request, 0, sizeof(GetParameters_Request));
    memset(&parameter_server->get_response, 0, sizeof(GetParameters_Response));
    memset(&parameter_server->get_types_request, 0, sizeof(GetParameterTypes_Request));
    memset(&parameter_server->get_types_response, 0, sizeof(GetParameterTypes_Response));

    memset(&parameter_server->set_request, 0, sizeof(SetParameters_Request));
    memset(&parameter_server->set_response, 0, sizeof(SetParameters_Response));

    memset(&parameter_server->list_request, 0, sizeof(ListParameters_Request));
    memset(&parameter_server->list_response, 0, sizeof(ListParameters_Response));

    memset(&parameter_server->list_request, 0, sizeof(ListParameters_Request));
    memset(&parameter_server->event_list, 0, sizeof(ParameterEvent));

    // Init parameter_server->parameter_list
    parameter_server->parameter_list.data = parameter_server->static_pool->parameter_list;
    parameter_server->parameter_list.capacity = RCLC_PARAMETER_MAX_PARAM;
    parameter_server->parameter_list.size = 0;
    
    for (size_t i = 0; i < RCLC_PARAMETER_MAX_PARAM; i++)
    {
        rclc_parameter_init_string(
            &parameter_server->parameter_list.data[i].name,
            parameter_server->static_pool->parameter_list_names[i],
            RCLC_PARAMETER_MAX_STRING_LEN
        );
        rclc_parameter_init_parametervalue(
            &parameter_server->parameter_list.data[i].value
        );
    }
    
    // Init parameter_server->get_request
    parameter_server->get_request.names.data = parameter_server->static_pool->get_request_names;
    parameter_server->get_request.names.capacity = RCLC_PARAMETER_MAX_PARAM;
    parameter_server->get_request.names.size = 0;

    for (size_t i = 0; i < RCLC_PARAMETER_MAX_PARAM; i++)
    {
        rclc_parameter_init_string(
            &parameter_server->get_request.names.data[i],
            parameter_server->static_pool->get_request_names_data[i],
            RCLC_PARAMETER_MAX_STRING_LEN
        );
    }
    
    // Init parameter_server->get_response
    parameter_server->get_response.values.data = parameter_server->static_pool->get_response_values;
    parameter_server->get_response.values.capacity = RCLC_PARAMETER_MAX_PARAM;
    parameter_server->get_response.values.size = 0;

    // Init parameter_server->get_types_request
    parameter_server->get_types_request.names.data = parameter_server->static_pool->get_types_request_names;
    parameter_server->get_types_request.names.capacity = RCLC_PARAMETER_MAX_PARAM;
    parameter_server->get_types_request.names.size = 0;

    for (size_t i = 0; i < RCLC_PARAMETER_MAX_PARAM; i++)
    {
        rclc_parameter_init_string(
            &parameter_server->get_types_request.names.data[i],
            parameter_server->static_pool->get_type_request_names_data[i],
            RCLC_PARAMETER_MAX_STRING_LEN
        );
    }

    // Init parameter_server->get_types_response
    parameter_server->get_types_response.types.data = parameter_server->static_pool->get_types_request_types;
    parameter_server->get_types_response.types.capacity = RCLC_PARAMETER_MAX_PARAM;
    parameter_server->get_types_response.types.size = 0;

    // Init parameter_server->set_request
    parameter_server->set_request.parameters.data = parameter_server->static_pool->set_request_parameters;
    parameter_server->set_request.parameters.capacity = RCLC_PARAMETER_MAX_PARAM;
    parameter_server->set_request.parameters.size = 0;

    for (size_t i = 0; i < RCLC_PARAMETER_MAX_PARAM; i++)
    {
        rclc_parameter_init_string(
            &parameter_server->set_request.parameters.data[i].name,
            parameter_server->static_pool->set_request_parameters_name_data[i],
            RCLC_PARAMETER_MAX_STRING_LEN
        );
        rclc_parameter_init_parametervalue(
            &parameter_server->set_request.parameters.data[i].value
        );
    }

    // Init parameter_server->set_response
    parameter_server->set_response.results.data = parameter_server->static_pool->set_parameter_result;
    parameter_server->set_response.results.capacity = RCLC_PARAMETER_MAX_PARAM;
    parameter_server->set_response.results.size = 0;
    
    for (size_t i = 0; i < RCLC_PARAMETER_MAX_PARAM; i++)
    {
        rclc_parameter_init_string(
            &parameter_server->set_response.results.data[i].reason,
            parameter_server->static_pool->set_parameter_result_reason_data[i],
            RCLC_PARAMETER_MAX_STRING_LEN
        );
    }

    // Init parameter_server->list_response
    parameter_server->list_response.result.names.data = parameter_server->static_pool->list_response_names;
    parameter_server->list_response.result.names.capacity = RCLC_PARAMETER_MAX_PARAM;
    parameter_server->list_response.result.names.size = 0;

    for (size_t i = 0; i < RCLC_PARAMETER_MAX_PARAM; i++)
    {
        rclc_parameter_init_string(
            &parameter_server->list_response.result.names.data[i],
            parameter_server->static_pool->list_response_names_data[i],
            RCLC_PARAMETER_MAX_STRING_LEN
        );
    }

    // Init parameter_server
    rclc_parameter_init_string(
            &parameter_server->event_list.node,
            parameter_server->static_pool->event_list_node_data,
            RCLC_PARAMETER_MAX_STRING_LEN
        );
    
    rclc_parameter_set_string(&parameter_server->event_list.node, rcl_node_get_name(node));

    return ret;
}

rcl_ret_t rclc_parameter_server_fini(
        rclc_parameter_server_t* parameter_server,
        rcl_node_t* node)
{
    RCL_CHECK_ARGUMENT_FOR_NULL(parameter_server, RCL_RET_INVALID_ARGUMENT);
    RCL_CHECK_ARGUMENT_FOR_NULL(node, RCL_RET_INVALID_ARGUMENT);

    rcl_ret_t ret;

    ret = rcl_service_fini(&parameter_server->list_service, node);
    ret &= rcl_service_fini(&parameter_server->set_service, node);
    ret &= rcl_service_fini(&parameter_server->get_service, node);
    ret &= rcl_service_fini(&parameter_server->get_types_service, node);
    
    if (parameter_server->notify_changed_over_dds)
    {
       ret &= rcl_publisher_fini(&parameter_server->event_publisher, node);
    }
    
    microros_sm_put_memory(rclc_parameter_static_pool, parameter_server->static_pool);

    return ret;
}

rcl_ret_t rclc_executor_add_parameter_server(
        rclc_executor_t* executor,
        rclc_parameter_server_t* parameter_server,
        ModifiedParameter_Callback on_modification)
{
    rcl_ret_t ret;

    RCL_CHECK_ARGUMENT_FOR_NULL(parameter_server, RCL_RET_INVALID_ARGUMENT);
    RCL_CHECK_ARGUMENT_FOR_NULL(on_modification, RCL_RET_INVALID_ARGUMENT);

    parameter_server->on_modification = on_modification;

    ret = rclc_executor_add_service_with_context(executor, &parameter_server->list_service,
                    &parameter_server->list_request, &parameter_server->list_response,
                    rclc_parameter_server_list_service_callback, parameter_server);

    ret &= rclc_executor_add_service_with_context(executor, &parameter_server->get_types_service,
                    &parameter_server->get_types_request, &parameter_server->get_types_response,
                    rclc_parameter_server_get_types_service_callback, parameter_server);

    ret &= rclc_executor_add_service_with_context(executor, &parameter_server->set_service,
                    &parameter_server->set_request, &parameter_server->set_response,
                    rclc_parameter_server_set_service_callback,
                    parameter_server);

    ret &= rclc_executor_add_service_with_context(executor, &parameter_server->get_service,
                    &parameter_server->get_request, &parameter_server->get_response,
                    rclc_parameter_server_get_service_callback,
                    parameter_server);

    return ret;
}

rcl_ret_t rclc_add_parameter(
        rclc_parameter_server_t* parameter_server,
        const char* parameter_name,
        rclc_parameter_type_t type)
{
    size_t index = parameter_server->parameter_list.size;

    if (index >= parameter_server->parameter_list.capacity ||
            rclc_parameter_search(&parameter_server->parameter_list, parameter_name) != NULL)
    {
        return RCL_RET_ERROR;
    }
    else if (!rclc_parameter_set_string(&parameter_server->parameter_list.data[index].name, parameter_name))
    {
        return RCL_RET_ERROR;
    }

    parameter_server->parameter_list.data[index].value.type = type;
    parameter_server->parameter_list.size++;

    if (parameter_server->notify_changed_over_dds)
    {
        rclc_parameter_prepare_parameter_event(
            &parameter_server->event_list, 
            &parameter_server->parameter_list.data[index],
            true);
        return rclc_parameter_service_publish_event(parameter_server);
    }

    return true;
}

rcl_ret_t
rclc_parameter_set(
        rclc_parameter_server_t* parameter_server,
        const char* parameter_name,
        ...)
{   
    rcl_ret_t ret = RCL_RET_OK;

    Parameter* parameter =
        rclc_parameter_search(&parameter_server->parameter_list, parameter_name);

    if (parameter == NULL)
    {
        return RCL_RET_ERROR;
    }

    va_list args;
    va_start(args, parameter_name);

    switch (parameter->value.type)
    {
        case RCLC_PARAMETER_NOT_SET:
            ret = RCL_RET_INVALID_ARGUMENT;
            break;
        case RCLC_PARAMETER_BOOL:
            parameter->value.bool_value = (bool) va_arg(args, int);
            break;
        case RCLC_PARAMETER_INT:
            parameter->value.integer_value = va_arg(args, int);
            break;
        case RCLC_PARAMETER_DOUBLE:
            parameter->value.double_value = va_arg(args, double);
            break;
        default:
            break;
    }

    va_end(args);

    if (ret == RCL_RET_OK)
    {
        if (parameter_server->notify_changed_over_dds)
        {
            rclc_parameter_prepare_parameter_event(&parameter_server->event_list, parameter, false);
            rclc_parameter_service_publish_event(parameter_server);
        }

        if (parameter_server->on_modification)
        {
            parameter_server->on_modification(parameter);
        }
    }

    return ret;
}

rcl_ret_t rclc_parameter_set_bool(
        rclc_parameter_server_t* parameter_server,
        const char* parameter_name,
        bool value)
{
    Parameter* parameter =
            rclc_parameter_search(&parameter_server->parameter_list, parameter_name);

    if (parameter == NULL)
    {
        return RCL_RET_ERROR;
    }

    if (parameter->value.type != RCLC_PARAMETER_BOOL)
    {
        return RCL_RET_INVALID_ARGUMENT;
    } 
    else 
    {
        parameter->value.bool_value = value;

        if (parameter_server->notify_changed_over_dds)
        {
            rclc_parameter_prepare_parameter_event(&parameter_server->event_list, parameter, false);
            rclc_parameter_service_publish_event(parameter_server);
        }

        if (parameter_server->on_modification)
        {
            parameter_server->on_modification(parameter);
        }

        return RCL_RET_OK;
    }
}

rcl_ret_t rclc_parameter_set_int(
        rclc_parameter_server_t* parameter_server,
        const char* parameter_name,
        int64_t value)
{
    Parameter* parameter =
            rclc_parameter_search(&parameter_server->parameter_list, parameter_name);

    if (parameter == NULL)
    {
        return RCL_RET_ERROR;
    }
    
    if (parameter->value.type != RCLC_PARAMETER_INT)
    {
        return RCL_RET_INVALID_ARGUMENT;
    }
    else
    {
        parameter->value.integer_value = value;

        if (parameter_server->notify_changed_over_dds)
        {
            rclc_parameter_prepare_parameter_event(&parameter_server->event_list, parameter, false);
            rclc_parameter_service_publish_event(parameter_server);
        }

        if (parameter_server->on_modification)
        {
            parameter_server->on_modification(parameter);
        }

        return RCL_RET_OK;
    }
}

rcl_ret_t rclc_parameter_set_double(
        rclc_parameter_server_t* parameter_server,
        const char* parameter_name,
        double value)
{
    Parameter* parameter =
            rclc_parameter_search(&parameter_server->parameter_list, parameter_name);

    if (parameter == NULL)
    {
        return RCL_RET_ERROR;
    }

    if (parameter->value.type != RCLC_PARAMETER_DOUBLE)
    {
        return RCL_RET_INVALID_ARGUMENT;
    }
    else
    {
        parameter->value.double_value = value;

        if (parameter_server->notify_changed_over_dds)
        {
            rclc_parameter_prepare_parameter_event(&parameter_server->event_list, parameter, false);
            rclc_parameter_service_publish_event(parameter_server);
        }
        
        if (parameter_server->on_modification)
        {
            parameter_server->on_modification(parameter);
        }
        
        return RCL_RET_OK;
    }
}

rcl_ret_t
rclc_parameter_get(
        rclc_parameter_server_t* parameter_server,
        const char* parameter_name,
        void * value)
{
    rcl_ret_t ret = RCL_RET_OK;

    Parameter* parameter =
        rclc_parameter_search(&parameter_server->parameter_list, parameter_name);

    if (parameter == NULL)
    {
        return RCL_RET_ERROR;
    }

    switch (parameter->value.type)
    {
        case RCLC_PARAMETER_NOT_SET:
            ret = RCL_RET_INVALID_ARGUMENT;
            break;
        case RCLC_PARAMETER_BOOL:
            {
                bool* aux = (bool*) value;
                *aux = parameter->value.bool_value;
            }
            break;
        case RCLC_PARAMETER_INT:
            { 
               int* aux = (int*) value;
                *aux = (int) parameter->value.integer_value;
            }
            break;
        case RCLC_PARAMETER_DOUBLE:
            {
                double* aux = (double*) value;
                *aux = parameter->value.double_value;
            }
            break;
        default:
            break;
    }

    return ret;
}

rcl_ret_t rclc_parameter_get_bool(
        rclc_parameter_server_t* parameter_server,
        const char* parameter_name,
        bool* output)
{
    Parameter* parameter =
            rclc_parameter_search(&parameter_server->parameter_list, parameter_name);
    
    rcl_ret_t ret = RCL_RET_OK;

    if (parameter == NULL)
    {
        ret = RCL_RET_ERROR;
    }
    else if (parameter->value.type != RCLC_PARAMETER_BOOL)
    {
        ret = RCL_RET_INVALID_ARGUMENT;
    }
    else if (parameter != NULL)
    {
        *output = parameter->value.bool_value;
    }

    return ret;
}

rcl_ret_t rclc_parameter_get_int(
        rclc_parameter_server_t* parameter_server,
        const char* parameter_name,
        int* output)
{
    Parameter* parameter =
            rclc_parameter_search(&parameter_server->parameter_list, parameter_name);
    
    rcl_ret_t ret = RCL_RET_OK;
    
    if (parameter == NULL)
    {
        ret = RCL_RET_ERROR;
    }
    else if (parameter->value.type != RCLC_PARAMETER_INT)
    {
        ret = RCL_RET_INVALID_ARGUMENT;
    }
    else if (parameter != NULL)
    {
        *output = parameter->value.integer_value;
    }

    return ret;
}

rcl_ret_t rclc_parameter_get_double(
        rclc_parameter_server_t* parameter_server,
        const char* parameter_name,
        double* output)
{
    Parameter* parameter =
            rclc_parameter_search(&parameter_server->parameter_list, parameter_name);
    
    rcl_ret_t ret = RCL_RET_OK;
    
    if (parameter == NULL)
    {
        ret = RCL_RET_ERROR;
    }
    else if (parameter->value.type != RCLC_PARAMETER_DOUBLE)
    {
        ret = RCL_RET_INVALID_ARGUMENT;
    }
    else if (parameter != NULL)
    {
        *output = parameter->value.double_value;
    }
    
    return ret;
}

rcl_ret_t rclc_parameter_service_publish_event(
        rclc_parameter_server_t* parameter_server)
{
    RCL_CHECK_ARGUMENT_FOR_NULL(&parameter_server->event_publisher, RCL_RET_INVALID_ARGUMENT);
    RCL_CHECK_ARGUMENT_FOR_NULL(&parameter_server->event_list, RCL_RET_INVALID_ARGUMENT);

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    parameter_server->event_list.stamp.sec = ts.tv_sec;
    parameter_server->event_list.stamp.nanosec = ts.tv_nsec;

    rcl_ret_t ret = rcl_publish(&parameter_server->event_publisher, &parameter_server->event_list, NULL);
    parameter_server->event_list.new_parameters.size = 0;
    parameter_server->event_list.changed_parameters.size = 0;

    return ret;
}

rcl_ret_t rclc_parameter_server_init_service(
        rcl_service_t* service,
        rcl_node_t* node,
        char* service_name,
        const rosidl_service_type_support_t* srv_type)
{
    const char* node_name = rcl_node_get_name(node);

    char get_service_name[RCLC_PARAMETER_SERVICE_MAX_LENGHT];
    memset(get_service_name, 0, RCLC_PARAMETER_SERVICE_MAX_LENGHT);
    memcpy(get_service_name, node_name, strlen(node_name) + 1);
    memcpy((get_service_name + strlen(node_name)), service_name, strlen(service_name) + 1);
    return rclc_service_init_default(service, node, srv_type, get_service_name);
}

#if __cplusplus
}
#endif /* if __cplusplus */
