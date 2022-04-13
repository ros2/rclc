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

#include <gtest/gtest.h>

extern "C"
{
#include <rclc/node.h>
#include <rclc_parameter/rclc_parameter.h>
}

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

// #include "parameter_client.hpp"

static int callbacks_calls = 0;
static rclc_parameter_type_t expected_type = RCLC_PARAMETER_NOT_SET;
static union {
  bool bool_value;
  int64_t integer_value;
  double double_value;
} expected_value;

void on_parameter_changed(Parameter * param)
{
  callbacks_calls++;
  ASSERT_EQ(expected_type, param->value.type);
  switch (param->value.type) {
    case RCLC_PARAMETER_BOOL:
      ASSERT_EQ(param->value.bool_value, expected_value.bool_value);
      break;
    case RCLC_PARAMETER_INT:
      ASSERT_EQ(param->value.integer_value, expected_value.integer_value);
      break;
    case RCLC_PARAMETER_DOUBLE:
      ASSERT_EQ(param->value.double_value, expected_value.double_value);
      break;
    default:
      break;
  }
}

TEST(Test, rclc_node_init_default) {
  auto default_spin_timeout = std::chrono::duration<int64_t, std::milli>(5000);
  std::string node_name("test_node");

  // Reset global tests values
  expected_type = RCLC_PARAMETER_NOT_SET;
  expected_value.bool_value = false;
  callbacks_calls = 0;

  // Create auxiliar RCLCPP node
  rclcpp::init(0, NULL);
  auto param_client_node = std::make_shared<rclcpp::Node>("param_aux_client");
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
    param_client_node,
    node_name);

  // Init RCLC support
  rcl_ret_t rc;
  rclc_support_t support;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rc = rclc_support_init(&support, 0, nullptr, &allocator);
  EXPECT_EQ(RCL_RET_OK, rc);

  // Init node
  rcl_node_t node;
  rc = rclc_node_init_default(&node, node_name.c_str(), "", &support);
  EXPECT_EQ(RCL_RET_OK, rc);

  // Init parameter server
  rclc_parameter_server_t param_server;
  rc = rclc_parameter_server_init_default(&param_server, &node);
  EXPECT_EQ(RCL_RET_OK, rc);

  // Test with wrong arguments
  rc = rclc_parameter_server_init_default(NULL, &node);
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, rc);

  rc = rclc_parameter_server_init_default(&param_server, NULL);
  EXPECT_EQ(RCL_RET_INVALID_ARGUMENT, rc);

  // Add parameter to executor
  rclc_executor_t executor;
  rclc_executor_init(
    &executor, &support.context, RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER,
    &allocator);
  rc = rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed);
  EXPECT_EQ(RCL_RET_OK, rc);

  // Test add parameter
  std::vector<std::string> param_names;
  rc = rclc_add_parameter(&param_server, "param1", RCLC_PARAMETER_BOOL);
  param_names.push_back("param1");
  EXPECT_EQ(RCL_RET_OK, rc);

  // Fail on duplicated name
  rc = rclc_add_parameter(&param_server, "param1", RCLC_PARAMETER_DOUBLE);
  EXPECT_EQ(RCL_RET_ERROR, rc);

  // Fail on name length
  char overflow_name[RCLC_PARAMETER_MAX_STRING_LENGTH + 1];
  memset(overflow_name, ' ', RCLC_PARAMETER_MAX_STRING_LENGTH + 1);
  overflow_name[RCLC_PARAMETER_MAX_STRING_LENGTH] = '\0';

  rc = rclc_add_parameter(&param_server, overflow_name, RCLC_PARAMETER_BOOL);
  EXPECT_EQ(RCL_RET_ERROR, rc);

  // Add more parameters
  rc = rclc_add_parameter(&param_server, "param2", RCLC_PARAMETER_INT);
  param_names.push_back("param2");
  EXPECT_EQ(RCL_RET_OK, rc);

  rc = rclc_add_parameter(&param_server, "param3", RCLC_PARAMETER_DOUBLE);
  param_names.push_back("param3");
  EXPECT_EQ(RCL_RET_OK, rc);

  rc = rclc_add_parameter(&param_server, "param4", RCLC_PARAMETER_DOUBLE);
  param_names.push_back("param4");
  EXPECT_EQ(RCL_RET_OK, rc);

  // Fail on too much params
  rc = rclc_add_parameter(&param_server, "param5", RCLC_PARAMETER_DOUBLE);
  EXPECT_EQ(RCL_RET_ERROR, rc);

  // Set parameters
  expected_type = RCLC_PARAMETER_BOOL;
  expected_value.bool_value = true;
  rclc_parameter_set_bool(&param_server, "param1", static_cast<bool>(expected_value.bool_value));

  expected_type = RCLC_PARAMETER_INT;
  expected_value.integer_value = 10;
  rclc_parameter_set_int(&param_server, "param2", static_cast<int>(expected_value.integer_value));

  expected_type = RCLC_PARAMETER_DOUBLE;
  expected_value.double_value = 0.01;
  rclc_parameter_set_double(
    &param_server, "param3",
    static_cast<double>(expected_value.double_value));

  // Get parameters
  bool param1;
  int param2;
  double param3;

  rclc_parameter_get_bool(&param_server, "param1", &param1);
  ASSERT_EQ(param1, true);
  rclc_parameter_get_int(&param_server, "param2", &param2);
  ASSERT_EQ(param2, 10);
  rclc_parameter_get_double(&param_server, "param3", &param3);
  ASSERT_EQ(param3, 0.01);

  ASSERT_EQ(callbacks_calls, 3);

  // Spin RCLC parameter server in a thread
  bool spin = true;
  std::thread rclc_parameter_server_thread(
    [&]() -> void {
      while (spin) {
        ASSERT_EQ(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)), RCL_RET_OK);
      }
    }
  );

  // Wait for parameter server
  ASSERT_TRUE(parameters_client->wait_for_service(default_spin_timeout));

  // Use auxiliar RCLCPP node for check
  auto list_params = parameters_client->list_parameters({}, 10);
  ASSERT_EQ(list_params.names.size(), 4u);
  for (auto & name : list_params.names) {
    std::vector<std::string>::iterator it;
    it = std::find(param_names.begin(), param_names.end(), name);
    ASSERT_NE(it, param_names.end());
  }

  const std::string name("param1");
  const bool defaultvalue = 0;
  bool param_value = parameters_client->get_parameter(name, defaultvalue);
  ASSERT_EQ(param_value, true);

  // External set bool
  expected_type = RCLC_PARAMETER_BOOL;
  expected_value.bool_value = false;
  std::vector<rclcpp::Parameter> new_params =
  {rclcpp::Parameter("param1", expected_value.bool_value)};
  std::vector<rcl_interfaces::msg::SetParametersResult> result = parameters_client->set_parameters(
    new_params);
  ASSERT_TRUE(result[0].successful);

  // External fail type
  new_params.clear();
  new_params.push_back(rclcpp::Parameter("param1", static_cast<double>(12.2)));
  result = parameters_client->set_parameters(new_params);
  ASSERT_FALSE(result[0].successful);

  // External set int
  expected_type = RCLC_PARAMETER_INT;
  expected_value.integer_value = 12;
  new_params.clear();
  new_params.push_back(rclcpp::Parameter("param2", expected_value.integer_value));
  result = parameters_client->set_parameters(new_params);
  ASSERT_TRUE(result[0].successful);

  // External set double
  expected_type = RCLC_PARAMETER_DOUBLE;
  expected_value.double_value = 12.12;
  new_params.clear();
  new_params.push_back(rclcpp::Parameter("param3", expected_value.double_value));
  result = parameters_client->set_parameters(new_params);
  ASSERT_TRUE(result[0].successful);

  // External get types
  const std::vector<std::string> types_query = {
    "param1",
    "param2",
    "param3"
  };
  std::vector<rclcpp::ParameterType> types = parameters_client->get_parameter_types(types_query);
  ASSERT_EQ(types.size(), 3u);
  ASSERT_EQ(types[0], rclcpp::ParameterType::PARAMETER_BOOL);
  ASSERT_EQ(types[1], rclcpp::ParameterType::PARAMETER_INTEGER);
  ASSERT_EQ(types[2], rclcpp::ParameterType::PARAMETER_DOUBLE);

  // Test callback
  auto promise = std::make_shared<std::promise<void>>();
  auto future = promise->get_future();
  size_t on_parameter_calls = 0;
  auto sub = parameters_client->on_parameter_event(
    [&](const rcl_interfaces::msg::ParameterEvent::SharedPtr /* event */) -> void
    {
      on_parameter_calls++;
      promise->set_value();
    });

  // Sleep for pub/sub match
  std::this_thread::sleep_for(500ms);

  expected_type = RCLC_PARAMETER_BOOL;
  expected_value.double_value = false;
  rclc_parameter_set_bool(&param_server, "param1", false);

  ASSERT_EQ(
    rclcpp::spin_until_future_complete(
      param_client_node, future.share(),
      default_spin_timeout),
    rclcpp::FutureReturnCode::SUCCESS);

  ASSERT_EQ(on_parameter_calls, 1u);

  rclcpp::shutdown();

  spin = false;
  rclc_parameter_server_thread.join();

  // Destroy parameter server
  ASSERT_EQ(rclc_parameter_server_fini(&param_server, &node), RCL_RET_OK);
}
