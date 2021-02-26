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
#endif

#include <rcl_interfaces/msg/parameter_event.h>
#include <rcl_interfaces/srv/describe_parameters.h>
#include <rcl_interfaces/srv/get_parameters.h>
#include <rcl_interfaces/srv/get_parameter_types.h>
#include <rcl_interfaces/srv/list_parameters.h>
#include <rcl_interfaces/srv/set_parameters.h>
#include <rcl_interfaces/srv/set_parameters_atomically.h>

#include <stdio.h>
#include <string.h>
#include "rmw/qos_profiles.h"

#include "rcl/allocator.h"
#include "rcl/error_handling.h"
#include "rcl/publisher.h"

#include "rclc_parameter/parameter_service.h"

// TODO (pablogs9): check if list is still used and remove

typedef struct rclc_parameter_service_impl_t
{
  rclc_parameter_service_options_t options;
  rcl_node_t * node;
  rcl_service_t get_service;
  rcl_service_t get_types_service;
  rcl_service_t set_service;
  rcl_service_t set_atomically_service;
  rcl_service_t list_service;
  rcl_service_t describe_service;

  rcl_publisher_t event_publisher;

  rcl_interfaces__srv__GetParameters_Request get_request;
  rcl_interfaces__srv__GetParameters_Response get_response;

  rcl_interfaces__srv__GetParameterTypes_Request get_types_request;
  rcl_interfaces__srv__GetParameterTypes_Response get_types_response;

  rcl_interfaces__srv__SetParameters_Request set_request;
  rcl_interfaces__srv__SetParameters_Response set_response;

  rcl_interfaces__srv__SetParametersAtomically_Request set_atomically_request;
  rcl_interfaces__srv__SetParametersAtomically_Response set_atomically_response;

  rcl_interfaces__srv__ListParameters_Request list_request;
  rcl_interfaces__srv__ListParameters_Response list_response;

  rcl_interfaces__srv__DescribeParameters_Request describe_request;
  rcl_interfaces__srv__DescribeParameters_Response describe_response;

  size_t wait_set_get_service_index;
  size_t wait_set_get_types_service_index;
  size_t wait_set_set_service_index;
  size_t wait_set_set_atomilcally_service;
  size_t wait_set_list_service_index;
  size_t wait_set_describe_service_index;
} rclc_parameter_service_impl_t;

rclc_parameter_service_options_t
rclc_parameter_service_get_default_options(void)
{
  static rclc_parameter_service_options_t default_options;
  default_options.qos = rmw_qos_profile_parameters;
  default_options.parameter_event_qos = rmw_qos_profile_parameter_events;
  default_options.allocator = rcl_get_default_allocator();
  default_options.remote_node_name = NULL;
  return default_options;
}

rclc_parameter_service_t
rclc_get_zero_initialized_parameter_service(void)
{
  static rclc_parameter_service_t null_service = {0};
  return null_service;
}

#define RCL_PARAMETER_INITIALIZE_SERVICE(VERB, SRV_TYPE_NAME, SRV_SUFFIX) \
  { \
    const rosidl_service_type_support_t * VERB ## _ts = ROSIDL_GET_SRV_TYPE_SUPPORT( \
      rcl_interfaces, srv, SRV_TYPE_NAME \
      ); \
 \
    size_t VERB ## len = strlen(node_name) + strlen(SRV_SUFFIX) + 1; \
    char * VERB ## _service_name = (char *)options->allocator.allocate( \
      VERB ## len, options->allocator.state); \
    memset(VERB ## _service_name, 0, VERB ## len); \
    VERB ## _service_name = memcpy( \
      VERB ## _service_name, node_name, strlen(node_name) + 1); \
    memcpy((VERB ## _service_name + strlen(node_name)), \
      SRV_SUFFIX, strlen(SRV_SUFFIX) + 1); \
 \
    ret = rcl_service_init( \
      &parameter_service->impl->VERB ## _service, \
      node, \
      VERB ## _ts, \
      VERB ## _service_name, \
      &service_options); \
    options->allocator.deallocate(VERB ## _service_name, options->allocator.state); \
    if (ret != RCL_RET_OK) { \
      fail_ret = ret; \
      goto fail_ ## VERB; \
    } \
    rcl_interfaces__srv__ ## SRV_TYPE_NAME ## _Request__init( \
      &parameter_service->impl->VERB ## _request); \
    rcl_interfaces__srv__ ## SRV_TYPE_NAME ## _Response__init( \
      &parameter_service->impl->VERB ## _response); \
  } \

#define RCL_PARAMETER_SERVICE_FINI(VERB, SRV_TYPE_NAME) \
  ret = rcl_service_fini( \
    &parameter_service->impl->VERB ## _service, parameter_service->impl->node); \
  if (ret != RCL_RET_OK) { \
    fprintf(stderr, "rcl_service_fini failed for service " #VERB "\n"); \
    fail_ret = ret; \
  } \
  rcl_interfaces__srv__ ## SRV_TYPE_NAME ## _Request__fini( \
    &parameter_service->impl->VERB ## _request); \
  rcl_interfaces__srv__ ## SRV_TYPE_NAME ## _Response__fini( \
    &parameter_service->impl->VERB ## _response); \


rcl_ret_t
rclc_parameter_service_init(
  rclc_parameter_service_t * parameter_service,
  rcl_node_t * node,
  const rclc_parameter_service_options_t * options
)
{

  // Check options and allocator first, so allocator can be used with errors.
  RCL_CHECK_ARGUMENT_FOR_NULL(options, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ALLOCATOR_WITH_MSG(&options->allocator, "invalid allocator", return RCL_RET_INVALID_ARGUMENT);

  RCL_CHECK_ARGUMENT_FOR_NULL(parameter_service, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(node, RCL_RET_INVALID_ARGUMENT);
  if (!node->impl) {
    RCL_SET_ERROR_MSG("invalid node");
    return RCL_RET_NODE_INVALID;
  }
  if (parameter_service->impl) {
    RCL_SET_ERROR_MSG(
      "service already initialized, or memory was unintialized");
    return RCL_RET_ALREADY_INIT;
  }
  rcl_ret_t ret;
  rcl_ret_t fail_ret = RCL_RET_ERROR;
  rcl_service_options_t service_options = rcl_service_get_default_options();
  service_options.qos = options->qos;
  service_options.allocator = options->allocator;

  parameter_service->impl = (rclc_parameter_service_impl_t *)options->allocator.allocate(
    sizeof(rclc_parameter_service_impl_t), options->allocator.state);
  if (!parameter_service->impl) {
    goto fail_alloc;
  }
  memset(parameter_service->impl, 0, sizeof(rclc_parameter_service_impl_t));
  parameter_service->impl->node = node;

  const char * node_name = options->remote_node_name !=
    NULL ? node_name = options->remote_node_name : rcl_node_get_name(node);

  // Initialize all services in impl storage
  RCL_PARAMETER_INITIALIZE_SERVICE(get, GetParameters, "/get_parameters");
  RCL_PARAMETER_INITIALIZE_SERVICE(get_types, GetParameterTypes, "/get_parameter_types");
  RCL_PARAMETER_INITIALIZE_SERVICE(set, SetParameters, "/set_parameters");
  RCL_PARAMETER_INITIALIZE_SERVICE(set_atomically, SetParametersAtomically, "/set_parameters_atomically");
  RCL_PARAMETER_INITIALIZE_SERVICE(list, ListParameters, "/list_parameters");
  RCL_PARAMETER_INITIALIZE_SERVICE(describe, DescribeParameters, "/describe_parameters");

  const rosidl_message_type_support_t * event_ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    rcl_interfaces, msg, ParameterEvent);
  // TODO(jacquelinekay) Should the parameter event topic name be namespaced?
  // Is this a configuration option?
  rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();
  publisher_options.allocator = options->allocator;
  publisher_options.qos = options->parameter_event_qos;
  const char * event_topic_name = "parameter_events";
  ret = rcl_publisher_init(
    &parameter_service->impl->event_publisher, node, event_ts, event_topic_name,
    &publisher_options);
  if (ret != RCL_RET_OK) {
    fail_ret = ret;
    goto fail;
  }

  parameter_service->impl->options = *options;
  return RCL_RET_OK;

fail:
  ret = rcl_publisher_fini(
    &parameter_service->impl->event_publisher, parameter_service->impl->node);
  if (ret != RCL_RET_OK) {
    fail_ret = ret;
    fprintf(stderr, "rcl_publisher_fini failed in fail block of rclc_parameter_service_init\n");
  }

fail_describe:
  RCL_PARAMETER_SERVICE_FINI(describe, DescribeParameters);

fail_list:
  RCL_PARAMETER_SERVICE_FINI(list, ListParameters);

fail_set_atomically:
  RCL_PARAMETER_SERVICE_FINI(set_atomically, SetParametersAtomically);

fail_set:
  RCL_PARAMETER_SERVICE_FINI(set, SetParameters);

fail_get_types:
  RCL_PARAMETER_SERVICE_FINI(get_types, GetParameterTypes);

fail_get:
  RCL_PARAMETER_SERVICE_FINI(get, GetParameters);

fail_alloc:

  return fail_ret;
}


rcl_ret_t
rclc_parameter_service_init_default(
  rclc_parameter_service_t * parameter_service,
  rcl_node_t * node)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    parameter_service, "parameter_service is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    node, "node is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  rclc_parameter_service_options_t opts = rclc_parameter_service_get_default_options();
  rcl_ret_t rc = rclc_parameter_service_init(parameter_service, node, &opts);
  if (rc != RCL_RET_OK) {
    // TODO (pablogs9): Where is ROS_PACKAGE_NAME defined?
    // PRINT_RCLC_ERROR(rclc_parameter_service_init_default, rclc_parameter_service_init);
  }
  return rc;
}

rcl_ret_t
rclc_parameter_service_fini(rclc_parameter_service_t * parameter_service)
{
  rcl_ret_t ret;
  rcl_ret_t fail_ret = RCL_RET_OK;
  RCL_PARAMETER_SERVICE_FINI(get, GetParameters);
  RCL_PARAMETER_SERVICE_FINI(get_types, GetParameterTypes);
  RCL_PARAMETER_SERVICE_FINI(set, SetParameters);
  RCL_PARAMETER_SERVICE_FINI(set_atomically, SetParametersAtomically);
  RCL_PARAMETER_SERVICE_FINI(list, ListParameters);

  ret =
    rcl_publisher_fini(&parameter_service->impl->event_publisher, parameter_service->impl->node);
  if (ret != RCL_RET_OK) {
    fail_ret = ret;
  }

  rcl_allocator_t allocator = parameter_service->impl->options.allocator;
  allocator.deallocate(parameter_service->impl, allocator.state);

  if (fail_ret != RCL_RET_OK) {
    return fail_ret;
  }
  return ret;
}

#define DEFINE_RCL_PARAMETER_SERVICE_TAKE_REQUEST(VERB, REQUEST_SUBTYPE, SUBFIELD_NAME) \
  REQUEST_SUBTYPE * \
  rclc_parameter_service_take_ ## VERB ## _request( \
    const rclc_parameter_service_t * parameter_service, \
    rmw_request_id_t * request_header) \
  { \
    RCL_CHECK_ARGUMENT_FOR_NULL(parameter_service, NULL); \
 \
    rcl_ret_t ret = rcl_take_request( \
      &parameter_service->impl->VERB ## _service, request_header, \
      &parameter_service->impl->VERB ## _request); \
    if (ret != RCL_RET_OK) { \
      return NULL; \
    } \
 \
    return &parameter_service->impl->VERB ## _request.SUBFIELD_NAME; \
  }

DEFINE_RCL_PARAMETER_SERVICE_TAKE_REQUEST(get, rosidl_runtime_c__String__Sequence, names)
DEFINE_RCL_PARAMETER_SERVICE_TAKE_REQUEST(get_types, rosidl_runtime_c__String__Sequence, names)
DEFINE_RCL_PARAMETER_SERVICE_TAKE_REQUEST(set, rcl_interfaces__msg__Parameter__Sequence, parameters)
DEFINE_RCL_PARAMETER_SERVICE_TAKE_REQUEST(set_atomically, rcl_interfaces__msg__Parameter__Sequence, parameters)
DEFINE_RCL_PARAMETER_SERVICE_TAKE_REQUEST(describe, rosidl_runtime_c__String__Sequence, names)

rcl_ret_t
rclc_parameter_service_take_list_request(
  const rclc_parameter_service_t * parameter_service,
  rmw_request_id_t * request_header,
  rosidl_runtime_c__String__Sequence * prefixes,
  uint64_t * depth)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(
    parameter_service, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(prefixes, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(depth, RCL_RET_INVALID_ARGUMENT);

  rcl_ret_t ret = rcl_take_request(
    &parameter_service->impl->list_service, request_header, &parameter_service->impl->list_request);

  prefixes = &parameter_service->impl->list_request.prefixes;
  depth = &parameter_service->impl->list_request.depth;

  return ret;
}

#define DEFINE_RCL_PARAMETER_SERVICE_SEND_RESPONSE(VERB, REQUEST_SUBTYPE, SUBFIELD_NAME) \
  rcl_ret_t \
  rclc_parameter_service_send_ ## VERB ## _response( \
    const rclc_parameter_service_t * parameter_service, \
    rmw_request_id_t * request_header, \
    const REQUEST_SUBTYPE * SUBFIELD_NAME) \
  { \
    RCL_CHECK_ARGUMENT_FOR_NULL(parameter_service, RCL_RET_INVALID_ARGUMENT); \
    RCL_CHECK_ARGUMENT_FOR_NULL(SUBFIELD_NAME, RCL_RET_INVALID_ARGUMENT); \
 \
    parameter_service->impl->VERB ## _response.SUBFIELD_NAME = *SUBFIELD_NAME; \
 \
    rcl_ret_t ret = rcl_send_response( \
      &parameter_service->impl->VERB ## _service, request_header, \
      &parameter_service->impl->VERB ## _response); \
 \
    return ret; \
  }

DEFINE_RCL_PARAMETER_SERVICE_SEND_RESPONSE(get, rcl_interfaces__msg__ParameterValue__Sequence, values)
DEFINE_RCL_PARAMETER_SERVICE_SEND_RESPONSE(get_types, rosidl_runtime_c__uint8__Sequence, types)
DEFINE_RCL_PARAMETER_SERVICE_SEND_RESPONSE(set, rcl_interfaces__msg__SetParametersResult__Sequence, results)
DEFINE_RCL_PARAMETER_SERVICE_SEND_RESPONSE(set_atomically, rcl_interfaces__msg__SetParametersResult, result)
DEFINE_RCL_PARAMETER_SERVICE_SEND_RESPONSE(list, rcl_interfaces__msg__ListParametersResult, result)
DEFINE_RCL_PARAMETER_SERVICE_SEND_RESPONSE(describe, rcl_interfaces__msg__ParameterDescriptor__Sequence, descriptors)

rcl_ret_t
rclc_parameter_service_publish_event(
  const rclc_parameter_service_t * parameter_service,
  const rcl_interfaces__msg__ParameterEvent * event)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(parameter_service, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(event, RCL_RET_INVALID_ARGUMENT);

  rcl_ret_t ret = rcl_publish(&parameter_service->impl->event_publisher, event, NULL);

  return ret;
}

rcl_ret_t
rclc_wait_set_add_parameter_service(
  rcl_wait_set_t * wait_set,
  const rclc_parameter_service_t * parameter_service)
{
  rcl_ret_t ret;

  RCL_CHECK_ARGUMENT_FOR_NULL(parameter_service->impl, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ALLOCATOR_WITH_MSG(
    &parameter_service->impl->options.allocator, "invalid allocator",
    return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(wait_set, RCL_RET_INVALID_ARGUMENT);

  ret = rcl_wait_set_add_service(
    wait_set,
    &parameter_service->impl->get_service,
    &parameter_service->impl->wait_set_get_service_index);
  if (ret != RCL_RET_OK) {
    RCL_SET_ERROR_MSG(
      "Failed to add get_parameters service to waitset!");
    return ret;
  }
  ret = rcl_wait_set_add_service(
    wait_set,
    &parameter_service->impl->get_types_service,
    &parameter_service->impl->wait_set_get_types_service_index);
  if (ret != RCL_RET_OK) {
    RCL_SET_ERROR_MSG(
      "Failed to add get_parameter_types service to waitset!");
    return ret;
  }
  ret = rcl_wait_set_add_service(
    wait_set,
    &parameter_service->impl->set_service,
    &parameter_service->impl->wait_set_set_service_index);
  if (ret != RCL_RET_OK) {
    RCL_SET_ERROR_MSG(
      "Failed to add set_parameters service to waitset!");
    return ret;
  }
  ret = rcl_wait_set_add_service(
    wait_set,
    &parameter_service->impl->set_atomically_service,
    &parameter_service->impl->wait_set_set_atomilcally_service);
  if (ret != RCL_RET_OK) {
    RCL_SET_ERROR_MSG(
      "Failed to add set_parameters_atomically service to waitset!");
    return ret;
  }
  ret = rcl_wait_set_add_service(
    wait_set,
    &parameter_service->impl->list_service,
    &parameter_service->impl->wait_set_list_service_index);
  if (ret != RCL_RET_OK) {
    RCL_SET_ERROR_MSG(
      "Failed to add list_parameters service to waitset!");
    return ret;
  }
  ret = rcl_wait_set_add_service(
    wait_set,
    &parameter_service->impl->describe_service,
    &parameter_service->impl->wait_set_describe_service_index);
  if (ret != RCL_RET_OK) {
    RCL_SET_ERROR_MSG(
      "Failed to add describe_parameters service to waitset!");
    return ret;
  }

  return ret;
}

rcl_ret_t
rcl_parameter_service_get_pending_action(
  const rcl_wait_set_t * wait_set,
  const rclc_parameter_service_t * parameter_service,
  rclc_param_action_t * action)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(wait_set, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(parameter_service, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(action, RCL_RET_INVALID_ARGUMENT);
  size_t i = 0;
  size_t j = 0;

  for (i = 0; i < wait_set->size_of_services; ++i) {
    for (j = 0; j < RCLC_PARAMETER_NUMBER_OF_SERVICES; ++j) {
      rcl_service_t * service_ptr = NULL;
      *action = j;
      switch (j) {
        case RCLC_GET_PARAMETERS:
          service_ptr = &parameter_service->impl->get_service;
          break;
        case RCLC_GET_PARAMETER_TYPES:
          service_ptr = &parameter_service->impl->get_types_service;
          break;
        case RCLC_SET_PARAMETERS:
          service_ptr = &parameter_service->impl->set_service;
          break;
        case RCLC_SET_PARAMETERS_ATOMICALLY:
          service_ptr = &parameter_service->impl->set_atomically_service;
          break;
        case RCLC_LIST_PARAMETERS:
          service_ptr = &parameter_service->impl->list_service;
          break;
        case RCLC_DESCRIBE_PARAMETERS:
          service_ptr = &parameter_service->impl->describe_service;
          break;
        default:
          *action = RCLC_PARAMETER_ACTION_UNKNOWN;
          return RCL_RET_ERROR;
      }
      if (service_ptr == wait_set->services[i]) {
        return RCL_RET_OK;
      }
    }
  }
  *action = RCLC_PARAMETER_ACTION_UNKNOWN;
  return RCL_RET_OK;
}

#if __cplusplus
}
#endif
