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

#include <gtest/gtest.h>

#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#include "rcl_interfaces/msg/detail/list_parameters_result__functions.h"
#include "rcl_interfaces/msg/detail/parameter__functions.h"
#include "rcl_interfaces/msg/detail/parameter_type__struct.h"
#include "rcl_interfaces/msg/detail/parameter_value__functions.h"
#include "rcl_interfaces/msg/detail/set_parameters_result__functions.h"

#include "rcl/error_handling.h"
#include "rcl/node.h"
#include "rclc_parameter/parameter.h"
#include "rclc_parameter/parameter_client.h"
#include "rclc_parameter/parameter_service.h"

#include "osrf_testing_tools_cpp/scope_exit.hpp"

#include "rcl/rcl.h"


#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

#define WAIT_TIME -1
// #define WAIT_TIME 1000000000
#define NUM_PARAMS (size_t)4


class CLASSNAME (TestParametersFixture, RMW_IMPLEMENTATION) : public ::testing::Test
{
public:
  rcl_context_t * context_ptr = nullptr;
  rcl_node_t * node_ptr = nullptr;
  rcl_wait_set_t * wait_set = nullptr;
  rclc_parameter_service_t * parameter_service = nullptr;
  rclc_parameter_client_t * parameter_client = nullptr;

  void SetUp()
  {
    rcl_ret_t ret;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    ret = rcl_init_options_init(&init_options, rcl_get_default_allocator());
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT({
      EXPECT_EQ(RCL_RET_OK, rcl_init_options_fini(&init_options)) << rcl_get_error_string().str;
    });

    this->context_ptr = new rcl_context_t;
    *this->context_ptr = rcl_get_zero_initialized_context();

    ret = rcl_init(0, nullptr, &init_options, this->context_ptr);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

    this->node_ptr = new rcl_node_t;
    *this->node_ptr = rcl_get_zero_initialized_node();

    rcl_node_options_t node_options = rcl_node_get_default_options();

    ret = rcl_node_init(this->node_ptr, "parameter_node", "", this->context_ptr, &node_options);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

    this->wait_set = new rcl_wait_set_t;
    *this->wait_set = rcl_get_zero_initialized_wait_set();
    ret = rcl_wait_set_init(wait_set, 0, 0, 0, 0, 0, 0, this->context_ptr, rcl_get_default_allocator());
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

    this->parameter_client = new rclc_parameter_client_t;
    *this->parameter_client = rclc_get_zero_initialized_parameter_client();
    rclc_parameter_client_options_t cs_options = rclc_parameter_client_get_default_options();
    ret = rclc_parameter_client_init(this->parameter_client, this->node_ptr, &cs_options);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

    this->parameter_service = new rclc_parameter_service_t;
    *this->parameter_service = rclc_get_zero_initialized_parameter_service();
    rclc_parameter_service_options_t ps_options = rclc_parameter_service_get_default_options();
    ret = rclc_parameter_service_init(this->parameter_service, this->node_ptr, &ps_options);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  }

  void TearDown()
  {
    rcl_ret_t ret;
    if (this->wait_set) {
      ret = rcl_wait_set_fini(this->wait_set);
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
      delete this->wait_set;
    }

    if (this->parameter_service) {
      ret = rclc_parameter_service_fini(this->parameter_service);
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
      delete this->parameter_service;
    }

    if (this->parameter_client) {
      ret = rclc_parameter_client_fini(this->parameter_client);
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
      delete this->parameter_client;
    }

    if (this->node_ptr) {
      ret = rcl_node_fini(this->node_ptr);
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
      delete this->node_ptr;
    }
    ret = rcl_shutdown(this->context_ptr);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  }
};

// Helper function for filling in hardcoded test values
rcl_ret_t fill_parameter_sequence(rcl_interfaces__msg__Parameter__Sequence * parameters)
{
  size_t parameters_idx = 0;
  rcl_ret_t ret = rclc_parameter_set_bool(&parameters->data[parameters_idx++], "bool_param", true);
  if (ret != RCL_RET_OK) {
    return ret;
  }
  ret = rclc_parameter_set_integer(&parameters->data[parameters_idx++], "int_param", 123);
  if (ret != RCL_RET_OK) {
    return ret;
  }
  ret = rclc_parameter_set_double(&parameters->data[parameters_idx++], "double_param", 45.67);
  if (ret != RCL_RET_OK) {
    return ret;
  }

  ret = rclc_parameter_set_string(
    &parameters->data[parameters_idx++], "string_param", "hello world");
  return ret;
}

void compare_parameter_sequence(const rcl_interfaces__msg__Parameter__Sequence * parameters)
{
  ASSERT_EQ(NUM_PARAMS, parameters->size);
  size_t parameters_idx = 0;

  EXPECT_EQ(0, strcmp(parameters->data[parameters_idx].name.data, "bool_param"));
  EXPECT_TRUE(parameters->data[parameters_idx++].value.bool_value);

  EXPECT_EQ(0, strcmp(parameters->data[parameters_idx].name.data, "int_param"));
  EXPECT_EQ(123, parameters->data[parameters_idx++].value.integer_value);

  EXPECT_EQ(0, strcmp(parameters->data[parameters_idx].name.data, "double_param"));
  EXPECT_EQ(45.67, parameters->data[parameters_idx++].value.double_value);

  EXPECT_EQ(0, strcmp(parameters->data[parameters_idx].name.data, "string_param"));
  EXPECT_EQ(0, strcmp(parameters->data[parameters_idx].value.string_value.data, "hello world"));
}

void compare_parameter_sequence(const rcl_interfaces__msg__ParameterValue__Sequence * parameters)
{
  ASSERT_EQ(NUM_PARAMS, parameters->size);
  size_t parameters_idx = 0;

  EXPECT_TRUE(parameters->data[parameters_idx++].bool_value);
  EXPECT_EQ(123, parameters->data[parameters_idx++].integer_value);
  EXPECT_EQ(45.67, parameters->data[parameters_idx++].double_value);
  EXPECT_EQ(0, strcmp(parameters->data[parameters_idx].string_value.data, "hello world"));
}

void compare_parameter_sequence(const rosidl_runtime_c__String__Sequence * parameter_names)
{
  ASSERT_EQ(NUM_PARAMS, parameter_names->size);
  size_t parameters_idx = 0;

  EXPECT_EQ(0, strcmp(parameter_names->data[parameters_idx++].data, "bool_param"));
  EXPECT_EQ(0, strcmp(parameter_names->data[parameters_idx++].data, "int_param"));
  EXPECT_EQ(0, strcmp(parameter_names->data[parameters_idx++].data, "double_param"));
  EXPECT_EQ(0, strcmp(parameter_names->data[parameters_idx++].data, "string_param"));
}

rcl_ret_t fill_parameter_sequence(rcl_interfaces__msg__ParameterValue__Sequence * parameters)
{
  size_t parameters_idx = 0;
  rcl_ret_t ret = rclc_parameter_set_value_bool(&parameters->data[parameters_idx++], true);
  if (ret != RCL_RET_OK) {
    return ret;
  }
  ret = rclc_parameter_set_value_integer(&parameters->data[parameters_idx++], 123);
  if (ret != RCL_RET_OK) {
    return ret;
  }
  ret = rclc_parameter_set_value_double(&parameters->data[parameters_idx++], 45.67);
  if (ret != RCL_RET_OK) {
    return ret;
  }

  ret = rclc_parameter_set_value_string(&parameters->data[parameters_idx++], "hello world");
  return ret;
}

bool fill_parameter_names_sequence(rosidl_runtime_c__String__Sequence * names)
{
  size_t parameters_idx = 0;
  if (!rosidl_runtime_c__String__assign(&names->data[parameters_idx++], "bool_param")) {
    return false;
  }
  if (!rosidl_runtime_c__String__assign(&names->data[parameters_idx++], "int_param")) {
    return false;
  }
  if (!rosidl_runtime_c__String__assign(&names->data[parameters_idx++], "double_param")) {
    return false;
  }
  if (!rosidl_runtime_c__String__assign(&names->data[parameters_idx++], "string_param")) {
    return false;
  }
  // rosidl_runtime_c__String__assign(&names->data[parameters_idx++], "bytes_param");
  return true;
}

rcl_ret_t prepare_wait_set(
  rcl_wait_set_t * wait_set, rclc_parameter_service_t * parameter_service,
  rclc_parameter_client_t * parameter_client)
{
  rcl_ret_t ret = rcl_wait_set_clear(wait_set);
  if (ret != RCL_RET_OK) {
    return ret;
  }
  ret = rcl_wait_set_resize(wait_set, 1, 0, 0, RCLC_PARAMETER_NUMBER_OF_SERVICES, RCLC_PARAMETER_NUMBER_OF_SERVICES, 0);
  if (ret != RCL_RET_OK) {
    return ret;
  }
  ret = rclc_wait_set_add_parameter_service(wait_set, parameter_service);
  if (ret != RCL_RET_OK) {
    return ret;
  }
  ret = rclc_wait_set_add_parameter_client(wait_set, parameter_client);
  return ret;
}

// TODO(jacquelinekay) Test un-setting parameters using set_parameters
TEST_F(CLASSNAME(TestParametersFixture, RMW_IMPLEMENTATION), test_set_parameters) {
  rmw_request_id_t request_header;
  rclc_param_action_t action = RCLC_PARAMETER_ACTION_UNKNOWN;
  rcl_ret_t ret;

  rcl_interfaces__msg__Parameter__Sequence parameters;
  EXPECT_TRUE(rcl_interfaces__msg__Parameter__Sequence__init(&parameters, NUM_PARAMS));

  ret = fill_parameter_sequence(&parameters);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  int64_t seq_num;
  ret = rclc_parameter_client_send_set_request(this->parameter_client, &parameters, &seq_num);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  ret = prepare_wait_set(this->wait_set, this->parameter_service, this->parameter_client);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  ret = rcl_wait(this->wait_set, WAIT_TIME);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  ret = rcl_parameter_service_get_pending_action(wait_set, parameter_service, &action);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  EXPECT_EQ(RCLC_SET_PARAMETERS, action);

  rcl_interfaces__msg__Parameter__Sequence * parameters_req = rclc_parameter_service_take_set_request(
    this->parameter_service, &request_header);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  // Validate the request
  compare_parameter_sequence(parameters_req);

  // For now we'll just set them all to be successful
  // Should SetParametersResult have a "name" field for the parameter key it describes?

  rcl_interfaces__msg__SetParametersResult__Sequence results;
  EXPECT_TRUE(rcl_interfaces__msg__SetParametersResult__Sequence__init(&results, NUM_PARAMS));

  for (size_t i = 0; i < NUM_PARAMS; ++i) {
    results.data[i].successful = true;
    rosidl_runtime_c__String__assign(&results.data[i].reason, "success");
  }
  ret = rclc_parameter_service_send_set_response(
    this->parameter_service, &request_header, &results);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  ret = prepare_wait_set(this->wait_set, this->parameter_service, this->parameter_client);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  ret = rcl_wait(this->wait_set, WAIT_TIME);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  ret = rclc_parameter_client_get_pending_action(wait_set, parameter_client, &action);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  EXPECT_EQ(RCLC_SET_PARAMETERS, action);

  rcl_interfaces__msg__SetParametersResult__Sequence * results_response =
    rclc_parameter_client_take_set_response(this->parameter_client, &request_header);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  for (size_t i = 0; i < NUM_PARAMS; ++i) {
    EXPECT_TRUE(results_response->data[i].successful);
    EXPECT_EQ(0, strcmp(results.data[i].reason.data, "success"));
  }

  rcl_interfaces__msg__Parameter__Sequence prior_state;
  EXPECT_TRUE(rcl_interfaces__msg__Parameter__Sequence__init(&prior_state, 3));

  // Bogus values for the previous state: one the same, one removed, one changed
  ret = rclc_parameter_set_integer(&prior_state.data[0], "int_param", 123);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  ret = rclc_parameter_set_integer(&prior_state.data[1], "deleted", 24);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  ret = rclc_parameter_set_double(&prior_state.data[2], "double_param", -45.67);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  rcl_interfaces__msg__ParameterEvent event;
  EXPECT_TRUE(rcl_interfaces__msg__ParameterEvent__init(&event));

  ret = rclc_parameter_convert_changes_to_event(&prior_state, parameters_req, &event);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  auto validate_event = [](const rcl_interfaces__msg__ParameterEvent & param_event) {
      EXPECT_EQ(0, strcmp(param_event.changed_parameters.data[0].name.data, "double_param"));
      EXPECT_EQ(45.67, param_event.changed_parameters.data[0].value.double_value);

      EXPECT_EQ(0, strcmp(param_event.deleted_parameters.data[0].name.data, "deleted"));

      // Ordering of new parameters doesn't matter
      // New parameters
      EXPECT_EQ(0, strcmp(param_event.new_parameters.data[0].name.data, "bool_param"));
      EXPECT_TRUE(param_event.new_parameters.data[0].value.bool_value);

      EXPECT_EQ(0, strcmp(param_event.new_parameters.data[1].name.data, "string_param"));
      EXPECT_EQ(
        0, strcmp(param_event.new_parameters.data[1].value.string_value.data, "hello world"));
    };

  validate_event(event);

  ret = rclc_parameter_service_publish_event(this->parameter_service, &event);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  ret = prepare_wait_set(this->wait_set, this->parameter_service, this->parameter_client);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  ret = rcl_wait(this->wait_set, WAIT_TIME);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  rcl_interfaces__msg__ParameterEvent event_response;
  EXPECT_TRUE(rcl_interfaces__msg__ParameterEvent__init(&event_response));
  ret = rclc_parameter_client_take_event(this->parameter_client, &event_response, nullptr);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  validate_event(event_response);
}

TEST_F(CLASSNAME(TestParametersFixture, RMW_IMPLEMENTATION), test_set_parameters_atomically) {
  rmw_request_id_t request_header;
  rclc_param_action_t action;
  rcl_ret_t ret;

  rcl_interfaces__msg__Parameter__Sequence parameters;
  EXPECT_TRUE(rcl_interfaces__msg__Parameter__Sequence__init(&parameters, NUM_PARAMS));

  rcl_interfaces__msg__SetParametersResult result;
  EXPECT_TRUE(rcl_interfaces__msg__SetParametersResult__init(&result));

  ret = fill_parameter_sequence(&parameters);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  int64_t seq_num;
  ret = rclc_parameter_client_send_set_atomically_request(
    this->parameter_client, &parameters, &seq_num);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  ret = prepare_wait_set(this->wait_set, this->parameter_service, this->parameter_client);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  ret = rcl_wait(this->wait_set, WAIT_TIME);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  ret = rcl_parameter_service_get_pending_action(this->wait_set, this->parameter_service, &action);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  EXPECT_EQ(RCLC_SET_PARAMETERS_ATOMICALLY, action);

  rcl_interfaces__msg__Parameter__Sequence * parameters_req =
    rclc_parameter_service_take_set_atomically_request(this->parameter_service, &request_header);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  compare_parameter_sequence(parameters_req);

  // For now we'll just set them all to be successful
  // Should SetParametersResult have a "name" field for the parameter key it describes?
  result.successful = true;
  rosidl_runtime_c__String__assign(&result.reason, "Because reasons");
  ret = rclc_parameter_service_send_set_atomically_response(
    this->parameter_service, &request_header, &result);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  ret = prepare_wait_set(this->wait_set, this->parameter_service, this->parameter_client);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  ret = rcl_wait(this->wait_set, WAIT_TIME);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  ret = rclc_parameter_client_get_pending_action(this->wait_set, this->parameter_client, &action);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  EXPECT_EQ(RCLC_SET_PARAMETERS_ATOMICALLY, action);

  rcl_interfaces__msg__SetParametersResult * result_response =
    rclc_parameter_client_take_set_atomically_response(this->parameter_client, &request_header);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  EXPECT_TRUE(result_response->successful);
  EXPECT_EQ(0, strcmp(result_response->reason.data, "Because reasons"));
}


TEST_F(CLASSNAME(TestParametersFixture, RMW_IMPLEMENTATION), test_get_parameters) {
  rmw_request_id_t request_header;
  rcl_ret_t ret;
  rclc_param_action_t action;
  (void) ret;

  rosidl_runtime_c__String__Sequence parameter_names;
  EXPECT_TRUE(rosidl_runtime_c__String__Sequence__init(&parameter_names, NUM_PARAMS));

  rcl_interfaces__msg__ParameterValue__Sequence parameter_values;
  EXPECT_TRUE(rcl_interfaces__msg__ParameterValue__Sequence__init(&parameter_values, NUM_PARAMS));

  EXPECT_TRUE(fill_parameter_names_sequence(&parameter_names)) << rcl_get_error_string().str;

  int64_t seq_num;
  ret = rclc_parameter_client_send_get_request(
    this->parameter_client, &parameter_names, &seq_num);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  ret = prepare_wait_set(this->wait_set, this->parameter_service, this->parameter_client);
  EXPECT_EQ(RCL_RET_OK, ret);
  ret = rcl_wait(this->wait_set, WAIT_TIME);
  EXPECT_EQ(RCL_RET_OK, ret);
  ret = rcl_parameter_service_get_pending_action(this->wait_set, this->parameter_service, &action);
  EXPECT_EQ(RCLC_GET_PARAMETERS, action);

  rosidl_runtime_c__String__Sequence * request = rclc_parameter_service_take_get_request(
    this->parameter_service, &request_header);
  EXPECT_EQ(RCL_RET_OK, ret);
  compare_parameter_sequence(request);

  // Assign some bogus values
  // In a real client library, these would be pulled from storage
  ret = fill_parameter_sequence(&parameter_values);
  EXPECT_EQ(RCL_RET_OK, ret);
  ret = rclc_parameter_service_send_get_response(
    this->parameter_service, &request_header, &parameter_values);
  EXPECT_EQ(RCL_RET_OK, ret);

  ret = prepare_wait_set(this->wait_set, this->parameter_service, this->parameter_client);
  EXPECT_EQ(RCL_RET_OK, ret);
  ret = rcl_wait(this->wait_set, WAIT_TIME);
  EXPECT_EQ(RCL_RET_OK, ret);
  ret = rclc_parameter_client_get_pending_action(this->wait_set, this->parameter_client, &action);
  EXPECT_EQ(RCLC_GET_PARAMETERS, action);

  // TODO(jacquelinekay): Should GetParameters_Response have a Parameter__Array subfield
  // instead of a ParameterValue__Array?
  rcl_interfaces__msg__ParameterValue__Sequence * response = rclc_parameter_client_take_get_response(
    this->parameter_client, &request_header);
  EXPECT_EQ(RCL_RET_OK, ret);

  compare_parameter_sequence(response);
}


TEST_F(CLASSNAME(TestParametersFixture, RMW_IMPLEMENTATION), test_get_parameter_types) {
  rmw_request_id_t request_header;
  rcl_ret_t ret;
  rclc_param_action_t action;
  (void) ret;

  rosidl_runtime_c__String__Sequence parameter_names;
  EXPECT_TRUE(rosidl_runtime_c__String__Sequence__init(&parameter_names, NUM_PARAMS));

  rosidl_runtime_c__uint8__Sequence parameter_types;
  EXPECT_TRUE(rosidl_runtime_c__uint8__Sequence__init(&parameter_types, NUM_PARAMS));

  EXPECT_TRUE(fill_parameter_names_sequence(&parameter_names)) << rcl_get_error_string().str;
  int64_t seq_num;
  ret = rclc_parameter_client_send_get_types_request(this->parameter_client, &parameter_names,
      &seq_num);

  ret = prepare_wait_set(this->wait_set, this->parameter_service, this->parameter_client);
  ret = rcl_wait(this->wait_set, WAIT_TIME);
  EXPECT_EQ(RCL_RET_OK, ret);
  ret = rcl_parameter_service_get_pending_action(this->wait_set, this->parameter_service, &action);
  EXPECT_EQ(RCLC_GET_PARAMETER_TYPES, action);

  rosidl_runtime_c__String__Sequence * request = rclc_parameter_service_take_get_types_request(
    this->parameter_service, &request_header);
  compare_parameter_sequence(request);

  {
    size_t parameters_idx = 0;
    parameter_types.data[parameters_idx++] = rcl_interfaces__msg__ParameterType__PARAMETER_BOOL;
    parameter_types.data[parameters_idx++] = rcl_interfaces__msg__ParameterType__PARAMETER_INTEGER;
    parameter_types.data[parameters_idx++] = rcl_interfaces__msg__ParameterType__PARAMETER_DOUBLE;
    parameter_types.data[parameters_idx++] = rcl_interfaces__msg__ParameterType__PARAMETER_STRING;
  }
  ret = rclc_parameter_service_send_get_types_response(this->parameter_service, &request_header,
      &parameter_types);

  ret = prepare_wait_set(this->wait_set, this->parameter_service, this->parameter_client);
  ret = rcl_wait(this->wait_set, WAIT_TIME);
  EXPECT_EQ(RCL_RET_OK, ret);
  ret = rclc_parameter_client_get_pending_action(this->wait_set, this->parameter_client, &action);
  EXPECT_EQ(RCLC_GET_PARAMETER_TYPES, action);

  rosidl_runtime_c__uint8__Sequence * response = rclc_parameter_client_take_get_types_response(
    this->parameter_client, &request_header);

  {
    size_t parameters_idx = 0;
    EXPECT_EQ(rcl_interfaces__msg__ParameterType__PARAMETER_BOOL, response->data[parameters_idx++]);
    EXPECT_EQ(rcl_interfaces__msg__ParameterType__PARAMETER_INTEGER,
      response->data[parameters_idx++]);
    EXPECT_EQ(rcl_interfaces__msg__ParameterType__PARAMETER_DOUBLE,
      response->data[parameters_idx++]);
    EXPECT_EQ(rcl_interfaces__msg__ParameterType__PARAMETER_STRING,
      response->data[parameters_idx++]);
    // EXPECT_EQ(
    //   rcl_interfaces__msg__ParameterType__PARAMETER_BYTES, response.data[parameters_idx++]);
  }
}

TEST_F(CLASSNAME(TestParametersFixture, RMW_IMPLEMENTATION), test_list_parameters) {
  rmw_request_id_t request_header;
  rcl_ret_t ret;
  rclc_param_action_t action;
  (void) ret;

  rcl_interfaces__msg__ListParametersResult list_result;
  EXPECT_TRUE(rcl_interfaces__msg__ListParametersResult__init(&list_result));
  EXPECT_TRUE(rosidl_runtime_c__String__Sequence__init(&list_result.names, NUM_PARAMS));
  EXPECT_TRUE(rosidl_runtime_c__String__Sequence__init(&list_result.prefixes, NUM_PARAMS));

  rosidl_runtime_c__String__Sequence prefixes;
  EXPECT_TRUE(rosidl_runtime_c__String__Sequence__init(&prefixes, 0));

  uint64_t depth = 0;
  int64_t seq_num;
  ret = rclc_parameter_client_send_list_request(this->parameter_client, &prefixes, depth, &seq_num);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  ret = prepare_wait_set(this->wait_set, this->parameter_service, this->parameter_client);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  ret = rcl_wait(this->wait_set, WAIT_TIME);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  ret = rcl_parameter_service_get_pending_action(this->wait_set, this->parameter_service, &action);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  EXPECT_EQ(RCLC_LIST_PARAMETERS, action);

  rosidl_runtime_c__String__Sequence prefixes_req;
  uint64_t depth_req;
  EXPECT_TRUE(rosidl_runtime_c__String__Sequence__init(&prefixes_req, 0));
  ret = rclc_parameter_service_take_list_request(this->parameter_service, &request_header,
      &prefixes_req, &depth_req);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  // put some names in
  EXPECT_TRUE(fill_parameter_names_sequence(&list_result.names)) << rcl_get_error_string().str;
  ret = rclc_parameter_service_send_list_response(this->parameter_service, &request_header,
      &list_result);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  ret = prepare_wait_set(this->wait_set, this->parameter_service, this->parameter_client);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  ret = rcl_wait(this->wait_set, WAIT_TIME);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  ret = rclc_parameter_client_get_pending_action(this->wait_set, this->parameter_client, &action);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  EXPECT_EQ(RCLC_LIST_PARAMETERS, action);

  rcl_interfaces__msg__ListParametersResult * result_response =
    rclc_parameter_client_take_list_response(this->parameter_client, &request_header);
  EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

  compare_parameter_sequence(&result_response->names);
}
