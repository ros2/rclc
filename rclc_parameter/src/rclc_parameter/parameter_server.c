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
#include <time.h>

// TODO: add prefixes functionality
void rclc_parameter_server_list_service_callback(
        const void* req,
        void* res,
        void* parameter_server)
{
    //rcl_interfaces__srv__ListParameters_Request * request = (rcl_interfaces__srv__ListParameters_Request *) req;
    (void) req;

    rcl_interfaces__srv__ListParameters_Response* response = (rcl_interfaces__srv__ListParameters_Response*) res;
    rcl_parameter_server_t* param_server = (rcl_parameter_server_t*) parameter_server;

    // Check call to rosidl_runtime_c__String__Sequence__fini
    rosidl_runtime_c__String__Sequence__init(&response->result.names, (size_t)  param_server->param_number);
    rosidl_runtime_c__String__Sequence__init(&response->result.prefixes, (size_t) 1);

    for (size_t i = 0; i < param_server->param_number; i++)
    {
        rosidl_runtime_c__String__assign(&response->result.names.data[i],
                param_server->parameter_list.data[i].name.data);
    }
}

void rclc_parameter_server_get_service_callback(
        const void* req,
        void* res,
        void* parameter_server)
{

    rcl_interfaces__srv__GetParameters_Request* request = (rcl_interfaces__srv__GetParameters_Request*) req;
    rcl_interfaces__srv__GetParameters_Response* response = (rcl_interfaces__srv__GetParameters_Response*) res;
    rcl_parameter_server_t* param_server = (rcl_parameter_server_t*) parameter_server;

    rcl_interfaces__msg__ParameterValue__Sequence__init(&response->values, request->names.size);

    for (size_t i = 0; i < request->names.size; i++)
    {
        rcl_interfaces__msg__Parameter* parameter = rclc_search_parameter(&param_server->parameter_list,
                        request->names.data[i].data);

        if (parameter != NULL)
        {
            rclc_parameter_value_copy(&response->values.data[i], &parameter->value);
        }
    }
}

void rclc_parameter_server_set_service_callback(
        const void* req,
        void* res,
        void* parameter_server)
{
    // Check if value is equal to stored and skip?

    rcl_interfaces__srv__SetParameters_Request* request = (rcl_interfaces__srv__SetParameters_Request*) req;
    rcl_interfaces__srv__SetParameters_Response* response = (rcl_interfaces__srv__SetParameters_Response*) res;
    rcl_parameter_server_t* param_server = (rcl_parameter_server_t*) parameter_server;

    rcl_interfaces__msg__Parameter__Sequence* changed_parameters =
            (rcl_interfaces__msg__Parameter__Sequence*) &param_server->event_list.changed_parameters;
    size_t index_changed_params[request->parameters.size];
    size_t num_changed_params = 0;

    rcl_interfaces__msg__SetParametersResult__Sequence__init(&response->results, request->parameters.size);

    for (size_t i = 0; i < request->parameters.size; i++)
    {
        rosidl_runtime_c__String* message = (rosidl_runtime_c__String*) &response->results.data[i].reason.data;
        rcl_interfaces__msg__Parameter* parameter = rclc_search_parameter(&param_server->parameter_list,
                        request->parameters.data[i].name.data);
        rcl_ret_t ret = RCL_RET_OK;

        if (parameter != NULL)
        {
            response->results.data[i].successful = true;

            switch (request->parameters.data[i].value.type)
            {
                case PARAMETER_NOT_SET:
                    rosidl_runtime_c__String__assignn(message, "Parameter type NOT SET", 25);
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

                case PARAMETER_STRING:
                    ret =
                            rclc_parameter_set_value_string(parameter,
                                    request->parameters.data[i].value.string_value.data);
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
            else if (response->results.data[i].successful != false)
            {
                response->results.data[i].successful = true;
                index_changed_params[num_changed_params] = i;
                num_changed_params++;
            }
        }
        else
        {
            rosidl_runtime_c__String__assignn(message, "Parameter not found", 20);
            response->results.data[i].successful = false;
        }
    }

    if (num_changed_params > 0)
    {
        rcl_interfaces__msg__Parameter__Sequence__init(changed_parameters, num_changed_params);

        for (size_t j = 0; j < num_changed_params; j++)
        {
            rclc_parameter_copy(&changed_parameters->data[j], &request->parameters.data[index_changed_params[j]]);
        }

        rclc_parameter_service_publish_event(param_server);
        rcl_interfaces__msg__Parameter__Sequence__fini(changed_parameters);
    }
}

rcl_ret_t rclc_parameter_server_init_default(
        rcl_parameter_server_t* parameter_server,
        rcl_node_t* node)
{
    // TODO: Add ret check
    rcl_ret_t ret;

    parameter_server->param_number = 0;
    rcl_interfaces__msg__Parameter__Sequence__init(&parameter_server->parameter_list, RCLC_UXRCE_MAX_PARAMETERS);

    for (size_t i = 0; i < RCLC_UXRCE_MAX_PARAMETERS; i++)
    {
        rosidl_runtime_c__String__init(&parameter_server->parameter_list.data[i].name);
        parameter_server->parameter_list.data[i].value.type = PARAMETER_NOT_SET;
    }

    const rosidl_service_type_support_t* get_ts = ROSIDL_GET_SRV_TYPE_SUPPORT(rcl_interfaces, srv, GetParameters);
    ret = rclc_parameter_server_init_service(&parameter_server->get_service, node, "/get_parameters", get_ts);

    const rosidl_service_type_support_t* set_ts = ROSIDL_GET_SRV_TYPE_SUPPORT(rcl_interfaces, srv, SetParameters);
    ret = rclc_parameter_server_init_service(&parameter_server->set_service, node, "/set_parameters", set_ts);

    const rosidl_service_type_support_t* list_ts = ROSIDL_GET_SRV_TYPE_SUPPORT(rcl_interfaces, srv, ListParameters);
    ret = rclc_parameter_server_init_service(&parameter_server->list_service, node, "/list_parameters", list_ts);

    const rosidl_message_type_support_t* event_ts = ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, ParameterEvent);

    ret = rclc_publisher_init_default(&parameter_server->event_publisher, node, event_ts, "/parameter_events");

    const char* node_name = rcl_node_get_name(node);
    rcl_interfaces__msg__ParameterEvent__init(&parameter_server->event_list);
    if (!rosidl_runtime_c__String__assign(&parameter_server->event_list.node, node_name))
    {
        return RCL_RET_ERROR;
    }

    rcl_interfaces__srv__GetParameters_Request__init(&parameter_server->get_request);
    rcl_interfaces__srv__GetParameters_Response__init(&parameter_server->get_response);

    rcl_interfaces__srv__SetParameters_Request__init(&parameter_server->set_request);
    rcl_interfaces__srv__SetParameters_Response__init(&parameter_server->set_response);

    rcl_interfaces__srv__ListParameters_Request__init(&parameter_server->list_request);
    rcl_interfaces__srv__ListParameters_Response__init(&parameter_server->list_response);

    return ret;
}

rcl_ret_t rclc_parameter_server_fini(
        rcl_parameter_server_t* parameter_server,
        rcl_node_t* node)
{
    // TODO: Add ret check
    rcl_ret_t ret;
    rcl_interfaces__msg__Parameter__Sequence__fini(&parameter_server->parameter_list);
    parameter_server->param_number = 0;

    ret = rcl_service_fini(&parameter_server->list_service, node);
    ret = rcl_service_fini(&parameter_server->set_service, node);
    ret = rcl_service_fini(&parameter_server->get_service, node);
    ret = rcl_publisher_fini(&parameter_server->event_publisher, node);

    rcl_interfaces__msg__ParameterEvent__fini(&parameter_server->event_list);

    rcl_interfaces__srv__GetParameters_Request__fini(&parameter_server->get_request);
    rcl_interfaces__srv__GetParameters_Response__fini(&parameter_server->get_response);

    rcl_interfaces__srv__SetParameters_Request__fini(&parameter_server->set_request);
    rcl_interfaces__srv__SetParameters_Response__fini(&parameter_server->set_response);

    rcl_interfaces__srv__ListParameters_Request__fini(&parameter_server->list_request);
    rcl_interfaces__srv__ListParameters_Response__fini(&parameter_server->list_response);

    return ret;
}

rcl_ret_t rclc_executor_add_parameter_server(
        rclc_executor_t* executor,
        rcl_parameter_server_t* parameter_server)
{
    // TODO: Add ret check
    rcl_ret_t ret;

    ret = rclc_executor_add_service_with_context(executor, &parameter_server->list_service,
                    &parameter_server->list_request, &parameter_server->list_response,
                    rclc_parameter_server_list_service_callback, parameter_server);

    ret = rclc_executor_add_service_with_context(executor, &parameter_server->set_service,
                    &parameter_server->set_request, &parameter_server->set_response,
                    rclc_parameter_server_set_service_callback,
                    parameter_server);

    ret = rclc_executor_add_service_with_context(executor, &parameter_server->get_service,
                    &parameter_server->get_request, &parameter_server->get_response,
                    rclc_parameter_server_get_service_callback,
                    parameter_server);

    return ret;
}

rcl_ret_t rclc_add_parameter_bool(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        bool value)
{
    if (parameter_server->param_number >= RCLC_UXRCE_MAX_PARAMETERS)
    {
        return RCL_RET_ERROR;
    }

    if (!rosidl_runtime_c__String__assign(&parameter_server->parameter_list.data[parameter_server->param_number].name,
            parameter_name))
    {
        return RCL_RET_ERROR;
    }

    parameter_server->parameter_list.data[parameter_server->param_number].value.type = PARAMETER_BOOL;
    parameter_server->parameter_list.data[parameter_server->param_number].value.bool_value = value;

    // TODO: Add ret check
    rcl_interfaces__msg__Parameter__Sequence__init(&parameter_server->event_list.new_parameters, 1);
    rclc_parameter_copy(&parameter_server->event_list.new_parameters.data[0],
            &parameter_server->parameter_list.data[parameter_server->param_number]);
    //rclc_parameter_service_publish_event(parameter_server);
    rcl_interfaces__msg__Parameter__Sequence__fini(&parameter_server->event_list.new_parameters);

    parameter_server->param_number++;

    return RCL_RET_OK;
}

rcl_ret_t rclc_add_parameter_int(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        int64_t value)
{
    if (parameter_server->param_number >= RCLC_UXRCE_MAX_PARAMETERS ||
            rclc_search_parameter(&parameter_server->parameter_list, parameter_name) != NULL)
    {
        return RCL_RET_ERROR;
    }
    else if (!rosidl_runtime_c__String__assign(&parameter_server->parameter_list.data[parameter_server->param_number].
                    name,
            parameter_name))
    {
        return RCL_RET_ERROR;
    }

    parameter_server->parameter_list.data[parameter_server->param_number].value.type = PARAMETER_INTEGER;
    parameter_server->parameter_list.data[parameter_server->param_number].value.integer_value = value;

    rcl_interfaces__msg__Parameter__Sequence__init(&parameter_server->event_list.new_parameters, 1);
    rclc_parameter_copy(&parameter_server->event_list.new_parameters.data[0],
            &parameter_server->parameter_list.data[parameter_server->param_number]);
    //rclc_parameter_service_publish_event(parameter_server);
    rcl_interfaces__msg__Parameter__Sequence__fini(&parameter_server->event_list.new_parameters);

    parameter_server->param_number++;

    return RCL_RET_OK;
}

rcl_ret_t rclc_add_parameter_double(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        double value)
{
    if (parameter_server->param_number >= RCLC_UXRCE_MAX_PARAMETERS ||
            rclc_search_parameter(&parameter_server->parameter_list, parameter_name) != NULL)
    {
        return RCL_RET_ERROR;
    }
    else if (!rosidl_runtime_c__String__assign(&parameter_server->parameter_list.data[parameter_server->param_number].
                    name,
            parameter_name))
    {
        return RCL_RET_ERROR;
    }

    parameter_server->parameter_list.data[parameter_server->param_number].value.type = PARAMETER_DOUBLE;
    parameter_server->parameter_list.data[parameter_server->param_number].value.double_value = value;

    rcl_interfaces__msg__Parameter__Sequence__init(&parameter_server->event_list.new_parameters, 1);
    rclc_parameter_copy(&parameter_server->event_list.new_parameters.data[0],
            &parameter_server->parameter_list.data[parameter_server->param_number]);
    //rclc_parameter_service_publish_event(parameter_server);
    rcl_interfaces__msg__Parameter__Sequence__fini(&parameter_server->event_list.new_parameters);

    parameter_server->param_number++;

    return RCL_RET_OK;
}

rcl_ret_t rclc_add_parameter_string(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        char* value)
{
    if (parameter_server->param_number >= RCLC_UXRCE_MAX_PARAMETERS ||
            rclc_search_parameter(&parameter_server->parameter_list, parameter_name) != NULL)
    {
        return RCL_RET_ERROR;
    }
    else if (!rosidl_runtime_c__String__assign(&parameter_server->parameter_list.data[parameter_server->param_number].
                    name,
            parameter_name))
    {
        return RCL_RET_ERROR;
    }

    parameter_server->parameter_list.data[parameter_server->param_number].value.type = PARAMETER_STRING;
    rosidl_runtime_c__String__assign(
        &parameter_server->parameter_list.data[parameter_server->param_number].value.string_value,
        value);

    rcl_interfaces__msg__Parameter__Sequence__init(&parameter_server->event_list.new_parameters, 1);
    rclc_parameter_copy(&parameter_server->event_list.new_parameters.data[0],
            &parameter_server->parameter_list.data[parameter_server->param_number]);
    //rclc_parameter_service_publish_event(parameter_server);
    rcl_interfaces__msg__Parameter__Sequence__fini(&parameter_server->event_list.new_parameters);

    parameter_server->param_number++;

    return RCL_RET_OK;
}

rcl_ret_t rclc_parameter_set_bool(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        bool value)
{
    rcl_interfaces__msg__Parameter* parameter =
            rclc_search_parameter(&parameter_server->parameter_list, parameter_name);
    rcl_ret_t ret = rclc_parameter_set_value_bool(parameter, value);

    if (ret == RCL_RET_OK)
    {
        rcl_interfaces__msg__Parameter__Sequence__init(&parameter_server->event_list.changed_parameters, 1);
        rclc_parameter_copy(&parameter_server->event_list.changed_parameters.data[0], parameter);
        rclc_parameter_service_publish_event(parameter_server);
        rcl_interfaces__msg__Parameter__Sequence__fini(&parameter_server->event_list.changed_parameters);
    }

    return ret;
}

rcl_ret_t rclc_parameter_set_int(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        int64_t value)
{
    rcl_interfaces__msg__Parameter* parameter =
            rclc_search_parameter(&parameter_server->parameter_list, parameter_name);
    rcl_ret_t ret = rclc_parameter_set_value_int(parameter, value);

    if (ret == RCL_RET_OK)
    {
        rcl_interfaces__msg__Parameter__Sequence__init(&parameter_server->event_list.changed_parameters, 1);
        rclc_parameter_copy(parameter_server->event_list.changed_parameters.data, parameter);
        rclc_parameter_service_publish_event(parameter_server);
        rcl_interfaces__msg__Parameter__Sequence__fini(&parameter_server->event_list.changed_parameters);
    }

    return ret;
}

rcl_ret_t rclc_parameter_set_double(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        double value)
{
    rcl_interfaces__msg__Parameter* parameter =
            rclc_search_parameter(&parameter_server->parameter_list, parameter_name);
    rcl_ret_t ret = rclc_parameter_set_value_double(parameter, value);

    if (ret == RCL_RET_OK)
    {
        rcl_interfaces__msg__Parameter__Sequence__init(&parameter_server->event_list.changed_parameters, 1);
        rclc_parameter_copy(parameter_server->event_list.changed_parameters.data, parameter);
        rclc_parameter_service_publish_event(parameter_server);
        rcl_interfaces__msg__Parameter__Sequence__fini(&parameter_server->event_list.changed_parameters);
    }

    return ret;
}

rcl_ret_t rclc_parameter_set_string(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        char* value)
{
    rcl_interfaces__msg__Parameter* parameter =
            rclc_search_parameter(&parameter_server->parameter_list, parameter_name);
    rcl_ret_t ret = rclc_parameter_set_value_string(parameter, value);

    if (ret == RCL_RET_OK)
    {
        rcl_interfaces__msg__Parameter__Sequence__init(&parameter_server->event_list.changed_parameters, 1);
        rclc_parameter_copy(parameter_server->event_list.changed_parameters.data, parameter);
        rclc_parameter_service_publish_event(parameter_server);
        rcl_interfaces__msg__Parameter__Sequence__fini(&parameter_server->event_list.changed_parameters);
    }

    return ret;
}

rcl_ret_t rclc_parameter_get_bool(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        bool* output)
{
    rcl_interfaces__msg__Parameter* parameter =
            rclc_search_parameter(&parameter_server->parameter_list, parameter_name);
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
    rcl_interfaces__msg__Parameter* parameter =
            rclc_search_parameter(&parameter_server->parameter_list, parameter_name);
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
    rcl_interfaces__msg__Parameter* parameter =
            rclc_search_parameter(&parameter_server->parameter_list, parameter_name);
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

// Add max_lenght for output?
rcl_ret_t rclc_parameter_get_string(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        char* output)
{
    rcl_interfaces__msg__Parameter* parameter =
            rclc_search_parameter(&parameter_server->parameter_list, parameter_name);
    rcl_ret_t ret = RCL_RET_ERROR;

    if (parameter != NULL)
    {
        memcpy(output, parameter->value.string_value.data,
                parameter->value.string_value.capacity);
        ret = RCL_RET_OK;
    }

    return ret;
}

rcl_ret_t rclc_parameter_get_string_lenght(
        rcl_parameter_server_t* parameter_server,
        const char* parameter_name,
        size_t* output)
{
    rcl_interfaces__msg__Parameter* parameter =
            rclc_search_parameter(&parameter_server->parameter_list, parameter_name);
    rcl_ret_t ret = RCL_RET_ERROR;

    if (parameter != NULL)
    {
        *output = parameter->value.string_value.capacity;
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
    parameter_server->event_list.stamp.sec = ts.tv_sec;
    parameter_server->event_list.stamp.nanosec = ts.tv_nsec;

    rcl_ret_t ret = rcl_publish(&parameter_server->event_publisher, &parameter_server->event_list, NULL);

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
