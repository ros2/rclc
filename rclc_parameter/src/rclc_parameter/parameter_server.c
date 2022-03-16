// Copyright (c) 2021 - for information on the respective copyright owner
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

#include <time.h>

#include <rcutils/time.h>
#include <rclc_parameter/rclc_parameter.h>

#include "./parameter_utils.h"

rcl_ret_t
rclc_parameter_server_init_service(
  rcl_service_t * service,
  rcl_node_t * node,
  char * service_name,
  const rosidl_service_type_support_t * srv_type);

rcl_ret_t rclc_parameter_service_publish_event(
  rclc_parameter_server_t * parameter_server);

void
rclc_parameter_server_describe_service_callback(
  const void * req,
  void * res,
  void * parameter_server)
{
  rclc_parameter_server_t * param_server = (rclc_parameter_server_t *) parameter_server;
  DescribeParameters_Request * request = (DescribeParameters_Request *) req;
  DescribeParameters_Response * response = (DescribeParameters_Response *) res;

  size_t size = (request->names.size > param_server->parameter_list.size) ?
    param_server->parameter_list.size :
    request->names.size;

  response->descriptors.size = size;

  for (size_t i = 0; i < size; i++) {
    rclc_parameter_set_string(
      &response->descriptors.data[i].name,
      request->names.data[i].data);

    Parameter * parameter = rclc_parameter_search(
      &param_server->parameter_list,
      request->names.data[i].data);

    response->descriptors.data[i].type = (parameter != NULL) ?
      parameter->value.type :
      RCLC_PARAMETER_NOT_SET;
  }
}

void
rclc_parameter_server_list_service_callback(
  const void * req,
  void * res,
  void * parameter_server)
{
  (void) req;

  rclc_parameter_server_t * param_server = (rclc_parameter_server_t *) parameter_server;
  ListParameters_Response * response = (ListParameters_Response *) res;

  response->result.names.size = param_server->parameter_list.size;

  for (size_t i = 0; i < response->result.names.size; i++) {
    rclc_parameter_set_string(
      &response->result.names.data[i],
      param_server->parameter_list.data[i].name.data);
  }
}

void
rclc_parameter_server_get_service_callback(
  const void * req,
  void * res,
  void * parameter_server)
{
  const GetParameters_Request * request = (const GetParameters_Request *) req;
  GetParameters_Response * response = (GetParameters_Response *) res;
  rclc_parameter_server_t * param_server = (rclc_parameter_server_t *) parameter_server;

  size_t size = (request->names.size > param_server->parameter_list.size) ?
    param_server->parameter_list.size :
    request->names.size;

  response->values.size = size;

  for (size_t i = 0; i < size; i++) {
    Parameter * parameter = rclc_parameter_search(
      &param_server->parameter_list,
      request->names.data[i].data);

    if (parameter != NULL) {
      rclc_parameter_value_copy(&response->values.data[i], &parameter->value);
    } else {
      response->values.data[i].type = RCLC_PARAMETER_NOT_SET;
    }
  }
}

void
rclc_parameter_server_get_types_service_callback(
  const void * req,
  void * res,
  void * parameter_server)
{
  const GetParameterTypes_Request * request = (const GetParameterTypes_Request *)  req;
  GetParameterTypes_Response * response = (GetParameterTypes_Response *) res;
  rclc_parameter_server_t * param_server = (rclc_parameter_server_t *) parameter_server;

  if (request->names.size > response->types.capacity) {
    response->types.size = 0;
    return;
  }

  response->types.size = request->names.size;

  for (size_t i = 0; i < response->types.size; i++) {
    Parameter * parameter = rclc_parameter_search(
      &param_server->parameter_list,
      request->names.data[i].data);

    if (parameter != NULL) {
      response->types.data[i] = parameter->value.type;
    } else {
      response->types.data[i] = RCLC_PARAMETER_NOT_SET;
    }
  }
}

void
rclc_parameter_server_set_service_callback(
  const void * req,
  void * res,
  void * parameter_server)
{
  const SetParameters_Request * request = (const SetParameters_Request *) req;
  SetParameters_Response * response = (SetParameters_Response *) res;
  rclc_parameter_server_t * param_server = (rclc_parameter_server_t *) parameter_server;

  if (request->parameters.size > response->results.capacity) {
    response->results.size = 0;
    return;
  }

  response->results.size = request->parameters.size;

  for (size_t i = 0; i < response->results.size; i++) {
    rosidl_runtime_c__String * message =
      (rosidl_runtime_c__String *) &response->results.data[i].reason;
    Parameter * parameter = rclc_parameter_search(
      &param_server->parameter_list,
      request->parameters.data[i].name.data);
    rcl_ret_t ret = RCL_RET_OK;

    rclc_parameter_set_string(message, "");

    if (parameter != NULL) {
      response->results.data[i].successful = true;

      if (parameter->value.type != request->parameters.data[i].value.type) {
        rclc_parameter_set_string(message, "Type mismatch");
        response->results.data[i].successful = false;
        continue;
      }

      switch (request->parameters.data[i].value.type) {
        case RCLC_PARAMETER_NOT_SET:
          rclc_parameter_set_string(message, "Type not set");
          response->results.data[i].successful = false;
          break;

        case RCLC_PARAMETER_BOOL:
          ret =
            rclc_parameter_set_bool(
            parameter_server, parameter->name.data,
            request->parameters.data[i].value.bool_value);
          break;
        case RCLC_PARAMETER_INT:
          ret =
            rclc_parameter_set_int(
            parameter_server, parameter->name.data,
            request->parameters.data[i].value.integer_value);
          break;
        case RCLC_PARAMETER_DOUBLE:
          ret =
            rclc_parameter_set_double(
            parameter_server, parameter->name.data,
            request->parameters.data[i].value.double_value);
          break;

        default:
          rclc_parameter_set_string(message, "Type not supported");
          response->results.data[i].successful = false;
          break;
      }

      if (ret == RCL_RET_INVALID_ARGUMENT) {
        rclc_parameter_set_string(message, "Set parameter error");
        response->results.data[i].successful = false;
      }
    } else {
      rclc_parameter_set_string(message, "Parameter not found");
      response->results.data[i].successful = false;
    }
  }
}

const rclc_parameter_options_t DEFAULT_PARAMETER_SERVER_OPTIONS = {
  .notify_changed_over_dds = true,
  .max_params = 4
};

rcl_ret_t rclc_parameter_server_init_default(
  rclc_parameter_server_t * parameter_server,
  rcl_node_t * node)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    parameter_server, "parameter is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    node, "node is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  return rclc_parameter_server_init_with_option(
    parameter_server, node, &DEFAULT_PARAMETER_SERVER_OPTIONS);
}

rcl_ret_t
rclc_parameter_server_init_with_option(
  rclc_parameter_server_t * parameter_server,
  rcl_node_t * node,
  const rclc_parameter_options_t * options)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    parameter_server, "parameter is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    node, "node is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    options, "options is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  rcl_ret_t ret = RCL_RET_OK;

  const rosidl_service_type_support_t * get_ts = ROSIDL_GET_SRV_TYPE_SUPPORT(
    rcl_interfaces, srv,
    GetParameters);
  ret &= rclc_parameter_server_init_service(
    &parameter_server->get_service, node, "/get_parameters",
    get_ts);

  const rosidl_service_type_support_t * get_types_ts = ROSIDL_GET_SRV_TYPE_SUPPORT(
    rcl_interfaces,
    srv,
    GetParameterTypes);
  ret &= rclc_parameter_server_init_service(
    &parameter_server->get_types_service, node,
    "/get_parameter_types", get_types_ts);

  const rosidl_service_type_support_t * set_ts = ROSIDL_GET_SRV_TYPE_SUPPORT(
    rcl_interfaces, srv,
    SetParameters);
  ret &= rclc_parameter_server_init_service(
    &parameter_server->set_service, node, "/set_parameters",
    set_ts);

  const rosidl_service_type_support_t * list_ts = ROSIDL_GET_SRV_TYPE_SUPPORT(
    rcl_interfaces, srv,
    ListParameters);
  ret &= rclc_parameter_server_init_service(
    &parameter_server->list_service, node,
    "/list_parameters", list_ts);

  const rosidl_service_type_support_t * describe_ts = ROSIDL_GET_SRV_TYPE_SUPPORT(
    rcl_interfaces, srv,
    DescribeParameters);
  ret &= rclc_parameter_server_init_service(
    &parameter_server->describe_service, node,
    "/describe_parameters", describe_ts);

  parameter_server->notify_changed_over_dds = options->notify_changed_over_dds;
  if (parameter_server->notify_changed_over_dds) {
    const rosidl_message_type_support_t * event_ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
      rcl_interfaces,
      msg,
      ParameterEvent);
    ret &= rclc_publisher_init(
      &parameter_server->event_publisher, node, event_ts,
      "/parameter_events",
      &rmw_qos_profile_parameter_events);
  }

  static char empty_string[RCLC_PARAMETER_MAX_STRING_LENGTH];
  memset(empty_string, ' ', RCLC_PARAMETER_MAX_STRING_LENGTH);
  empty_string[RCLC_PARAMETER_MAX_STRING_LENGTH - 1] = '\0';

  bool mem_allocs_ok = true;
  mem_allocs_ok &= rcl_interfaces__msg__Parameter__Sequence__init(
    &parameter_server->parameter_list,
    options->max_params);
  parameter_server->parameter_list.size = 0;

  // Pre-init strings
  for (size_t i = 0; i < options->max_params; i++) {
    mem_allocs_ok &= rosidl_runtime_c__String__assign(
      &parameter_server->parameter_list.data[i].name,
      (const char *) empty_string);
  }

  mem_allocs_ok &=
    rcl_interfaces__srv__ListParameters_Request__init(&parameter_server->list_request);
  mem_allocs_ok &= rcl_interfaces__srv__ListParameters_Response__init(
    &parameter_server->list_response);
  mem_allocs_ok &= rosidl_runtime_c__String__Sequence__init(
    &parameter_server->list_response.result.names,
    options->max_params);
  parameter_server->list_response.result.names.size = 0;

  // Pre-init strings
  for (size_t i = 0; i < options->max_params; i++) {
    mem_allocs_ok &= rosidl_runtime_c__String__assign(
      &parameter_server->list_response.result.names.data[i],
      (const char *) empty_string);
  }

  mem_allocs_ok &= rcl_interfaces__srv__GetParameters_Request__init(&parameter_server->get_request);
  mem_allocs_ok &=
    rcl_interfaces__srv__GetParameters_Response__init(&parameter_server->get_response);
  mem_allocs_ok &= rosidl_runtime_c__String__Sequence__init(
    &parameter_server->get_request.names,
    options->max_params);
  parameter_server->get_request.names.size = 0;
  mem_allocs_ok &= rcl_interfaces__msg__ParameterValue__Sequence__init(
    &parameter_server->get_response.values,
    options->max_params);
  parameter_server->get_response.values.size = 0;

  // Pre-init strings
  for (size_t i = 0; i < options->max_params; i++) {
    mem_allocs_ok &= rosidl_runtime_c__String__assign(
      &parameter_server->get_request.names.data[i],
      (const char *) empty_string);
  }

  mem_allocs_ok &= rcl_interfaces__srv__SetParameters_Request__init(&parameter_server->set_request);
  mem_allocs_ok &=
    rcl_interfaces__srv__SetParameters_Response__init(&parameter_server->set_response);
  mem_allocs_ok &= rcl_interfaces__msg__Parameter__Sequence__init(
    &parameter_server->set_request.parameters,
    options->max_params);
  parameter_server->set_request.parameters.size = 0;
  mem_allocs_ok &= rcl_interfaces__msg__SetParametersResult__Sequence__init(
    &parameter_server->set_response.results,
    options->max_params);
  parameter_server->set_response.results.size = 0;

  // Pre-init strings
  for (size_t i = 0; i < options->max_params; i++) {
    mem_allocs_ok &= rosidl_runtime_c__String__assign(
      &parameter_server->set_request.parameters.data[i].name,
      (const char *) empty_string);
    mem_allocs_ok &= rosidl_runtime_c__String__assign(
      &parameter_server->set_response.results.data[i].reason,
      (const char *) empty_string);
  }

  mem_allocs_ok &= rcl_interfaces__srv__GetParameterTypes_Request__init(
    &parameter_server->get_types_request);
  mem_allocs_ok &= rcl_interfaces__srv__GetParameterTypes_Response__init(
    &parameter_server->get_types_response);
  mem_allocs_ok &= rosidl_runtime_c__String__Sequence__init(
    &parameter_server->get_types_request.names,
    options->max_params);
  parameter_server->get_types_request.names.size = 0;
  mem_allocs_ok &= rosidl_runtime_c__uint8__Sequence__init(
    &parameter_server->get_types_response.types,
    options->max_params);
  parameter_server->get_types_response.types.size = 0;

  for (size_t i = 0; i < options->max_params; i++) {
    mem_allocs_ok &= rosidl_runtime_c__String__assign(
      &parameter_server->get_types_request.names.data[i],
      (const char *) empty_string);
  }

  mem_allocs_ok &= rcl_interfaces__srv__DescribeParameters_Request__init(
    &parameter_server->describe_request);
  mem_allocs_ok &= rcl_interfaces__srv__DescribeParameters_Response__init(
    &parameter_server->describe_response);
  mem_allocs_ok &= rosidl_runtime_c__String__Sequence__init(
    &parameter_server->describe_request.names,
    options->max_params);
  parameter_server->describe_request.names.size = 0;
  mem_allocs_ok &= rcl_interfaces__msg__ParameterDescriptor__Sequence__init(
    &parameter_server->describe_response.descriptors,
    options->max_params);
  parameter_server->describe_response.descriptors.size = 0;

  for (size_t i = 0; i < options->max_params; i++) {
    mem_allocs_ok &= rosidl_runtime_c__String__assign(
      &parameter_server->describe_request.names.data[i],
      (const char *) empty_string);
    mem_allocs_ok &= rosidl_runtime_c__String__assign(
      &parameter_server->describe_response.descriptors.data[i].name,
      (const char *) empty_string);
  }

  mem_allocs_ok &= rcl_interfaces__msg__ParameterEvent__init(&parameter_server->event_list);
  mem_allocs_ok &= rosidl_runtime_c__String__assign(
    &parameter_server->event_list.node,
    rcl_node_get_name(node));

  if (!mem_allocs_ok) {
    ret = RCL_RET_ERROR;
  }

  return ret;
}

rcl_ret_t
rclc_parameter_server_fini(
  rclc_parameter_server_t * parameter_server,
  rcl_node_t * node)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    parameter_server, "parameter is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    node, "node is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  rcl_ret_t ret = RCL_RET_OK;

  ret &= rcl_service_fini(&parameter_server->list_service, node);
  ret &= rcl_service_fini(&parameter_server->set_service, node);
  ret &= rcl_service_fini(&parameter_server->get_service, node);
  ret &= rcl_service_fini(&parameter_server->get_types_service, node);
  ret &= rcl_service_fini(&parameter_server->describe_service, node);

  if (parameter_server->notify_changed_over_dds) {
    ret &= rcl_publisher_fini(&parameter_server->event_publisher, node);
  }

  rosidl_runtime_c__String__fini(&parameter_server->event_list.node);

  for (size_t i = 0; i < parameter_server->describe_request.names.capacity; i++) {
    rosidl_runtime_c__String__fini(&parameter_server->describe_request.names.data[i]);
    rosidl_runtime_c__String__fini(&parameter_server->describe_response.descriptors.data[i].name);
  }

  rcl_interfaces__msg__ParameterDescriptor__Sequence__fini(
    &parameter_server->describe_response.descriptors);
  rosidl_runtime_c__String__Sequence__fini(&parameter_server->describe_request.names);
  rcl_interfaces__srv__DescribeParameters_Response__fini(&parameter_server->describe_response);
  rcl_interfaces__srv__DescribeParameters_Request__fini(&parameter_server->describe_request);

  for (size_t i = 0; i < parameter_server->get_types_request.names.capacity; i++) {
    rosidl_runtime_c__String__fini(&parameter_server->get_types_request.names.data[i]);
  }

  rosidl_runtime_c__uint8__Sequence__fini(&parameter_server->get_types_response.types);
  rosidl_runtime_c__String__Sequence__fini(&parameter_server->get_types_request.names);
  rcl_interfaces__srv__GetParameterTypes_Response__fini(&parameter_server->get_types_response);
  rcl_interfaces__srv__GetParameterTypes_Request__fini(&parameter_server->get_types_request);

  for (size_t i = 0; i < parameter_server->set_request.parameters.capacity; i++) {
    rosidl_runtime_c__String__fini(&parameter_server->set_request.parameters.data[i].name);
    rosidl_runtime_c__String__fini(&parameter_server->set_response.results.data[i].reason);
  }

  rcl_interfaces__msg__SetParametersResult__Sequence__fini(&parameter_server->set_response.results);
  rcl_interfaces__msg__Parameter__Sequence__fini(&parameter_server->set_request.parameters);
  rcl_interfaces__srv__SetParameters_Response__fini(&parameter_server->set_response);
  rcl_interfaces__srv__SetParameters_Request__fini(&parameter_server->set_request);

  for (size_t i = 0; i < parameter_server->get_request.names.capacity; i++) {
    rosidl_runtime_c__String__fini(&parameter_server->get_request.names.data[i]);
  }

  rcl_interfaces__msg__ParameterValue__Sequence__fini(&parameter_server->get_response.values);
  rosidl_runtime_c__String__Sequence__fini(&parameter_server->get_request.names);
  rcl_interfaces__srv__GetParameters_Response__fini(&parameter_server->get_response);
  rcl_interfaces__srv__GetParameters_Request__fini(&parameter_server->get_request);

  for (size_t i = 0; i < parameter_server->list_response.result.names.capacity; i++) {
    rosidl_runtime_c__String__fini(&parameter_server->list_response.result.names.data[i]);
  }

  rosidl_runtime_c__String__Sequence__fini(&parameter_server->list_response.result.names);
  rcl_interfaces__srv__ListParameters_Response__fini(&parameter_server->list_response);
  rcl_interfaces__srv__ListParameters_Request__fini(&parameter_server->list_request);

  for (size_t i = 0; i < parameter_server->parameter_list.capacity; i++) {
    rosidl_runtime_c__String__fini(&parameter_server->parameter_list.data[i].name);
  }

  rcl_interfaces__msg__Parameter__Sequence__fini(&parameter_server->parameter_list);

  return ret;
}

rcl_ret_t
rclc_executor_add_parameter_server(
  rclc_executor_t * executor,
  rclc_parameter_server_t * parameter_server,
  ModifiedParameter_Callback on_modification)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    executor, "executor is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    parameter_server, "parameter_server is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  rcl_ret_t ret;

  parameter_server->on_modification = on_modification;

  ret = rclc_executor_add_service_with_context(
    executor, &parameter_server->list_service,
    &parameter_server->list_request, &parameter_server->list_response,
    rclc_parameter_server_list_service_callback, parameter_server);

  ret &= rclc_executor_add_service_with_context(
    executor, &parameter_server->get_types_service,
    &parameter_server->get_types_request, &parameter_server->get_types_response,
    rclc_parameter_server_get_types_service_callback, parameter_server);

  ret &= rclc_executor_add_service_with_context(
    executor, &parameter_server->set_service,
    &parameter_server->set_request, &parameter_server->set_response,
    rclc_parameter_server_set_service_callback,
    parameter_server);

  ret &= rclc_executor_add_service_with_context(
    executor, &parameter_server->get_service,
    &parameter_server->get_request, &parameter_server->get_response,
    rclc_parameter_server_get_service_callback,
    parameter_server);

  ret &= rclc_executor_add_service_with_context(
    executor, &parameter_server->describe_service,
    &parameter_server->describe_request, &parameter_server->describe_response,
    rclc_parameter_server_describe_service_callback,
    parameter_server);

  return ret;
}

rcl_ret_t
rclc_add_parameter(
  rclc_parameter_server_t * parameter_server,
  const char * parameter_name,
  rclc_parameter_type_t type)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    parameter_server, "parameter_server is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    parameter_name, "parameter_name is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  size_t index = parameter_server->parameter_list.size;

  if (index >= parameter_server->parameter_list.capacity ||
    rclc_parameter_search(&parameter_server->parameter_list, parameter_name) != NULL)
  {
    return RCL_RET_ERROR;
  }

  if (!rclc_parameter_set_string(
      &parameter_server->parameter_list.data[index].name,
      parameter_name))
  {
    return RCL_RET_ERROR;
  }

  parameter_server->parameter_list.data[index].value.type = type;
  parameter_server->parameter_list.size++;

  if (parameter_server->notify_changed_over_dds) {
    rclc_parameter_prepare_parameter_event(
      &parameter_server->event_list,
      &parameter_server->parameter_list.data[index],
      true);
    return rclc_parameter_service_publish_event(parameter_server);
  }

  return RCL_RET_OK;
}

rcl_ret_t
rclc_parameter_set_bool(
  rclc_parameter_server_t * parameter_server,
  const char * parameter_name,
  bool value)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    parameter_server, "parameter_server is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    parameter_name, "parameter_name is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  Parameter * parameter =
    rclc_parameter_search(&parameter_server->parameter_list, parameter_name);

  if (parameter == NULL) {
    return RCL_RET_ERROR;
  }

  if (parameter->value.type != RCLC_PARAMETER_BOOL) {
    return RCL_RET_INVALID_ARGUMENT;
  }

  parameter->value.bool_value = value;

  if (parameter_server->notify_changed_over_dds) {
    rclc_parameter_prepare_parameter_event(&parameter_server->event_list, parameter, false);
    rclc_parameter_service_publish_event(parameter_server);
  }

  if (parameter_server->on_modification) {
    parameter_server->on_modification(parameter);
  }

  return RCL_RET_OK;
}

rcl_ret_t
rclc_parameter_set_int(
  rclc_parameter_server_t * parameter_server,
  const char * parameter_name,
  int64_t value)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    parameter_server, "parameter_server is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    parameter_name, "parameter_name is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  Parameter * parameter =
    rclc_parameter_search(&parameter_server->parameter_list, parameter_name);

  if (parameter == NULL) {
    return RCL_RET_ERROR;
  }

  if (parameter->value.type != RCLC_PARAMETER_INT) {
    return RCL_RET_INVALID_ARGUMENT;
  }

  parameter->value.integer_value = value;

  if (parameter_server->notify_changed_over_dds) {
    rclc_parameter_prepare_parameter_event(&parameter_server->event_list, parameter, false);
    rclc_parameter_service_publish_event(parameter_server);
  }

  if (parameter_server->on_modification) {
    parameter_server->on_modification(parameter);
  }

  return RCL_RET_OK;
}

rcl_ret_t
rclc_parameter_set_double(
  rclc_parameter_server_t * parameter_server,
  const char * parameter_name,
  double value)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    parameter_server, "parameter_server is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    parameter_name, "parameter_name is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  Parameter * parameter =
    rclc_parameter_search(&parameter_server->parameter_list, parameter_name);

  if (parameter == NULL) {
    return RCL_RET_ERROR;
  }

  if (parameter->value.type != RCLC_PARAMETER_DOUBLE) {
    return RCL_RET_INVALID_ARGUMENT;
  }

  parameter->value.double_value = value;

  if (parameter_server->notify_changed_over_dds) {
    rclc_parameter_prepare_parameter_event(&parameter_server->event_list, parameter, false);
    rclc_parameter_service_publish_event(parameter_server);
  }

  if (parameter_server->on_modification) {
    parameter_server->on_modification(parameter);
  }

  return RCL_RET_OK;
}

rcl_ret_t
rclc_parameter_get_bool(
  rclc_parameter_server_t * parameter_server,
  const char * parameter_name,
  bool * output)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    parameter_server, "parameter_server is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    parameter_name, "parameter_name is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    output, "output is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  Parameter * parameter =
    rclc_parameter_search(&parameter_server->parameter_list, parameter_name);

  rcl_ret_t ret = RCL_RET_OK;

  if (parameter == NULL) {
    ret = RCL_RET_ERROR;
  } else if (parameter->value.type != RCLC_PARAMETER_BOOL) {
    ret = RCL_RET_INVALID_ARGUMENT;
  } else if (parameter != NULL) {
    *output = parameter->value.bool_value;
  }

  return ret;
}

rcl_ret_t
rclc_parameter_get_int(
  rclc_parameter_server_t * parameter_server,
  const char * parameter_name,
  int * output)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    parameter_server, "parameter_server is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    parameter_name, "parameter_name is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    output, "output is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  Parameter * parameter =
    rclc_parameter_search(&parameter_server->parameter_list, parameter_name);

  rcl_ret_t ret = RCL_RET_OK;

  if (parameter == NULL) {
    ret = RCL_RET_ERROR;
  } else if (parameter->value.type != RCLC_PARAMETER_INT) {
    ret = RCL_RET_INVALID_ARGUMENT;
  } else if (parameter != NULL) {
    *output = parameter->value.integer_value;
  }

  return ret;
}

rcl_ret_t
rclc_parameter_get_double(
  rclc_parameter_server_t * parameter_server,
  const char * parameter_name,
  double * output)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    parameter_server, "parameter_server is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    parameter_name, "parameter_name is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    output, "output is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  Parameter * parameter =
    rclc_parameter_search(&parameter_server->parameter_list, parameter_name);

  rcl_ret_t ret = RCL_RET_OK;

  if (parameter == NULL) {
    ret = RCL_RET_ERROR;
  } else if (parameter->value.type != RCLC_PARAMETER_DOUBLE) {
    ret = RCL_RET_INVALID_ARGUMENT;
  } else if (parameter != NULL) {
    *output = parameter->value.double_value;
  }

  return ret;
}

rcl_ret_t
rclc_parameter_service_publish_event(
  rclc_parameter_server_t * parameter_server)
{
  rcl_ret_t ret = RCL_RET_OK;

  rcutils_time_point_value_t now;
  ret &= rcutils_system_time_now(&now);

  parameter_server->event_list.stamp.sec = RCUTILS_NS_TO_S(now);
  parameter_server->event_list.stamp.nanosec =
    now - RCUTILS_S_TO_NS(parameter_server->event_list.stamp.sec);

  ret &= rcl_publish(
    &parameter_server->event_publisher, &parameter_server->event_list,
    NULL);

  return ret;
}

rcl_ret_t
rclc_parameter_server_init_service(
  rcl_service_t * service,
  rcl_node_t * node,
  char * service_name,
  const rosidl_service_type_support_t * srv_type)
{
  const char * node_name = rcl_node_get_name(node);

  static char get_service_name[RCLC_PARAMETER_MAX_STRING_LENGTH];
  memset(get_service_name, 0, RCLC_PARAMETER_MAX_STRING_LENGTH);
  memcpy(get_service_name, node_name, strlen(node_name) + 1);
  memcpy((get_service_name + strlen(node_name)), service_name, strlen(service_name) + 1);
  return rclc_service_init(service, node, srv_type, get_service_name, &rmw_qos_profile_parameters);
}

#if __cplusplus
}
#endif /* if __cplusplus */
