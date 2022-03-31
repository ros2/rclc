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
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>
}

#include <string>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "rclc_parameter/parameter_utils.h"

using namespace std::chrono_literals;

TEST(ParameterTestUnitary, rclc_parameter_server_init_default) {
  // Init RCLC support
  rclc_support_t support;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  ASSERT_EQ(rclc_support_init(&support, 0, nullptr, &allocator), RCL_RET_OK);

  // Init node
  rcl_node_t node;
  ASSERT_EQ(rclc_node_init_default(&node, "test_node", "", &support), RCL_RET_OK);

  // Init parameter server
  rclc_parameter_server_t param_server;
  ASSERT_EQ(rclc_parameter_server_init_default(&param_server, &node), RCL_RET_OK);

  // Test with wrong arguments
  ASSERT_EQ(rclc_parameter_server_init_default(NULL, &node), RCL_RET_INVALID_ARGUMENT);
  rcutils_reset_error();

  ASSERT_EQ(rclc_parameter_server_init_default(&param_server, NULL), RCL_RET_INVALID_ARGUMENT);
  rcutils_reset_error();

  // Add parameter to executor
  rclc_executor_t executor;
  rclc_executor_init(
    &executor, &support.context, RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER,
    &allocator);
  ASSERT_EQ(rclc_executor_add_parameter_server(&executor, &param_server, nullptr), RCL_RET_OK);

  // Destroy parameter server
  ASSERT_EQ(rclc_parameter_server_fini(&param_server, &node), RCL_RET_OK);
  ASSERT_EQ(rclc_executor_fini(&executor), RCL_RET_OK);
  ASSERT_EQ(rcl_node_fini(&node), RCL_RET_OK);
}

TEST(ParameterTestUnitary, rclc_add_parameter) {
  // Init RCLC support
  rclc_support_t support;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  ASSERT_EQ(rclc_support_init(&support, 0, nullptr, &allocator), RCL_RET_OK);

  // Init node
  rcl_node_t node;
  ASSERT_EQ(rclc_node_init_default(&node, "test_node", "", &support), RCL_RET_OK);

  // Init parameter server
  rclc_parameter_server_t param_server;
  ASSERT_EQ(rclc_parameter_server_init_default(&param_server, &node), RCL_RET_OK);

  // Test add parameter
  ASSERT_EQ(rclc_add_parameter(&param_server, "param1", RCLC_PARAMETER_BOOL), RCL_RET_OK);

  // Fail on duplicated name
  ASSERT_EQ(rclc_add_parameter(&param_server, "param1", RCLC_PARAMETER_DOUBLE), RCL_RET_ERROR);

  // Fail on name length
  char overflow_name[RCLC_PARAMETER_MAX_STRING_LENGTH + 1];
  memset(overflow_name, ' ', RCLC_PARAMETER_MAX_STRING_LENGTH + 1);
  overflow_name[RCLC_PARAMETER_MAX_STRING_LENGTH] = '\0';

  ASSERT_EQ(rclc_add_parameter(&param_server, overflow_name, RCLC_PARAMETER_BOOL), RCL_RET_ERROR);

  // Add more parameters
  ASSERT_EQ(rclc_add_parameter(&param_server, "param2", RCLC_PARAMETER_INT), RCL_RET_OK);
  ASSERT_EQ(rclc_add_parameter(&param_server, "param3", RCLC_PARAMETER_DOUBLE), RCL_RET_OK);
  ASSERT_EQ(rclc_add_parameter(&param_server, "param4", RCLC_PARAMETER_DOUBLE), RCL_RET_OK);

  // Fail on too much params
  ASSERT_EQ(rclc_add_parameter(&param_server, "param5", RCLC_PARAMETER_DOUBLE), RCL_RET_ERROR);

  // Destroy parameter server
  ASSERT_EQ(rclc_parameter_server_fini(&param_server, &node), RCL_RET_OK);
  ASSERT_EQ(rcl_node_fini(&node), RCL_RET_OK);
}

class ParameterTestBase : public ::testing::TestWithParam<rclc_parameter_options_t>
{
public:
  ParameterTestBase()
  : default_spin_timeout(std::chrono::duration<int64_t, std::milli>(10000)),
    options(GetParam())
  {
    callcack_calls = 0;
    user_return = true;
    strncpy(old_parameter_name, "", sizeof(old_parameter_name));
    strncpy(new_parameter_name, "", sizeof(new_parameter_name));
    new_parameter_value.type = RCLC_PARAMETER_NOT_SET;
    old_parameter_value.type = RCLC_PARAMETER_NOT_SET;
  }

  ~ParameterTestBase() {}

  void SetUp() override
  {
    std::string node_name("test_node");

    // Init RCLC support
    allocator = rcl_get_default_allocator();
    EXPECT_EQ(rclc_support_init(&support, 0, nullptr, &allocator), RCL_RET_OK);

    // Init node
    EXPECT_EQ(rclc_node_init_default(&node, node_name.c_str(), "", &support), RCL_RET_OK);

    // Init parameter server
    EXPECT_EQ(rclc_parameter_server_init_with_option(&param_server, &node, &options), RCL_RET_OK);

    // Add parameter to executor
    rclc_executor_init(
      &executor, &support.context, RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER,
      &allocator);
    EXPECT_EQ(
      rclc_executor_add_parameter_server(
        &executor, &param_server,
        ParameterTestBase::on_parameter_changed), RCL_RET_OK);

    // Add initial parameters
    EXPECT_EQ(rclc_add_parameter(&param_server, "param1", RCLC_PARAMETER_BOOL), RCL_RET_OK);
    EXPECT_EQ(rclc_add_parameter(&param_server, "param2", RCLC_PARAMETER_INT), RCL_RET_OK);
    EXPECT_EQ(rclc_add_parameter(&param_server, "param3", RCLC_PARAMETER_DOUBLE), RCL_RET_OK);

    // Spin RCLC parameter server in a thread
    spin = true;
    rclc_parameter_server_thread = std::thread(
      [&]() -> void {
        while (spin) {
          ASSERT_EQ(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)), RCL_RET_OK);
        }
      }
    );

    // Init rclcpp node
    rclcpp::init(0, NULL);
    param_client_node = std::make_shared<rclcpp::Node>("param_aux_client");
    parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
      param_client_node,
      node_name);

    ASSERT_TRUE(parameters_client->wait_for_service(default_spin_timeout));
  }

  void TearDown() override
  {
    spin = false;
    rclc_parameter_server_thread.join();

    parameters_client.reset();
    param_client_node.reset();
    rclcpp::shutdown();

    // Destroy parameter server
    ASSERT_EQ(rclc_parameter_server_fini(&param_server, &node), RCL_RET_OK);
    ASSERT_EQ(rclc_executor_fini(&executor), RCL_RET_OK);
    ASSERT_EQ(rcl_node_fini(&node), RCL_RET_OK);
  }

  static bool on_parameter_changed(const Parameter * old_param, const Parameter * new_param)
  {
    if (new_param == NULL) {
      strncpy(new_parameter_name, "null", sizeof(new_parameter_name));
      new_parameter_value.type = RCLC_PARAMETER_NOT_SET;
    } else {
      strncpy(new_parameter_name, new_param->name.data, sizeof(new_parameter_name));
      rclc_parameter_value_copy(&new_parameter_value, &new_param->value);
    }

    if (old_param == NULL) {
      strncpy(old_parameter_name, "null", sizeof(old_parameter_name));
      old_parameter_value.type = RCLC_PARAMETER_NOT_SET;
    } else {
      strncpy(old_parameter_name, old_param->name.data, sizeof(old_parameter_name));
      rclc_parameter_value_copy(&old_parameter_value, &old_param->value);
    }

    callcack_calls++;
    return user_return;
  }

protected:
  // Callback
  static int callcack_calls;
  static bool user_return;
  static char old_parameter_name[RCLC_PARAMETER_MAX_STRING_LENGTH];
  static char new_parameter_name[RCLC_PARAMETER_MAX_STRING_LENGTH];
  static ParameterValue new_parameter_value;
  static ParameterValue old_parameter_value;

  // Rclcpp
  std::shared_ptr<rclcpp::Node> param_client_node;
  std::shared_ptr<rclcpp::SyncParametersClient> parameters_client;
  std::chrono::duration<int64_t, std::milli> default_spin_timeout;

  // Rclc
  rcl_allocator_t allocator;
  rclc_support_t support;
  rcl_node_t node;
  rclc_executor_t executor;
  rclc_parameter_options_t options;
  rclc_parameter_server_t param_server;
  std::thread rclc_parameter_server_thread;
  bool spin;
};

int ParameterTestBase::callcack_calls;
bool ParameterTestBase::user_return;
char ParameterTestBase::old_parameter_name[] = "";
char ParameterTestBase::new_parameter_name[] = "";
ParameterValue ParameterTestBase::new_parameter_value;
ParameterValue ParameterTestBase::old_parameter_value;

TEST_P(ParameterTestBase, rclc_set_get_parameter) {
  int expected_callcack_calls = 1;

  // Set parameters
  {
    const char * param_name = "param1";
    bool set_value = true;
    bool get_value;

    // Get initial value
    ASSERT_EQ(rclc_parameter_get_bool(&param_server, param_name, &get_value), RCL_RET_OK);
    ASSERT_EQ(get_value, false);

    // Set value
    ASSERT_EQ(rclc_parameter_set_bool(&param_server, param_name, set_value), RCL_RET_OK);
    ASSERT_EQ(strcmp(old_parameter_name, param_name), 0);
    ASSERT_EQ(strcmp(new_parameter_name, param_name), 0);
    ASSERT_EQ(old_parameter_value.type, RCLC_PARAMETER_BOOL);
    ASSERT_EQ(new_parameter_value.type, RCLC_PARAMETER_BOOL);
    ASSERT_EQ(old_parameter_value.bool_value, get_value);
    ASSERT_EQ(new_parameter_value.bool_value, set_value);
    ASSERT_EQ(callcack_calls, expected_callcack_calls);
    expected_callcack_calls++;

    // Get new value
    ASSERT_EQ(rclc_parameter_get_bool(&param_server, param_name, &get_value), RCL_RET_OK);
    ASSERT_EQ(set_value, get_value);
  }

  {
    const char * param_name = "param2";
    int set_value = 10;
    int get_value;

    // Get initial value
    ASSERT_EQ(rclc_parameter_get_int(&param_server, param_name, &get_value), RCL_RET_OK);
    ASSERT_EQ(get_value, 0);

    // Set value
    ASSERT_EQ(rclc_parameter_set_int(&param_server, param_name, set_value), RCL_RET_OK);
    ASSERT_EQ(strcmp(old_parameter_name, param_name), 0);
    ASSERT_EQ(strcmp(new_parameter_name, param_name), 0);
    ASSERT_EQ(old_parameter_value.type, RCLC_PARAMETER_INT);
    ASSERT_EQ(new_parameter_value.type, RCLC_PARAMETER_INT);
    ASSERT_EQ(old_parameter_value.integer_value, 0);
    ASSERT_EQ(new_parameter_value.integer_value, set_value);
    ASSERT_EQ(callcack_calls, expected_callcack_calls);
    expected_callcack_calls++;

    // Get new value
    ASSERT_EQ(rclc_parameter_get_int(&param_server, param_name, &get_value), RCL_RET_OK);
    ASSERT_EQ(set_value, get_value);
  }

  {
    const char * param_name = "param3";
    double set_value = 0.01;
    double get_value;

    // Get initial value
    ASSERT_EQ(rclc_parameter_get_double(&param_server, param_name, &get_value), RCL_RET_OK);
    ASSERT_EQ(get_value, 0.0);

    // Set value
    ASSERT_EQ(
      rclc_parameter_set_double(
        &param_server, param_name, set_value), RCL_RET_OK);
    ASSERT_EQ(strcmp(old_parameter_name, param_name), 0);
    ASSERT_EQ(strcmp(new_parameter_name, param_name), 0);
    ASSERT_EQ(old_parameter_value.type, RCLC_PARAMETER_DOUBLE);
    ASSERT_EQ(new_parameter_value.type, RCLC_PARAMETER_DOUBLE);
    ASSERT_EQ(old_parameter_value.double_value, 0.0);
    ASSERT_EQ(new_parameter_value.double_value, set_value);
    ASSERT_EQ(callcack_calls, expected_callcack_calls);
    expected_callcack_calls++;

    // Get new value
    ASSERT_EQ(rclc_parameter_get_double(&param_server, param_name, &get_value), RCL_RET_OK);
    ASSERT_EQ(set_value, get_value);
  }

  // Fail with user reject
  user_return = false;
  {
    const char * param_name = "param3";
    double set_value = 0.05;
    double first_get_value;
    double second_get_value;

    // Get initial value
    ASSERT_EQ(rclc_parameter_get_double(&param_server, param_name, &first_get_value), RCL_RET_OK);

    // Set value
    ASSERT_EQ(
      rclc_parameter_set_double(
        &param_server, param_name,
        set_value), PARAMETER_MODIFICATION_REJECTED);
    ASSERT_EQ(callcack_calls, 4);

    // Get value
    ASSERT_EQ(rclc_parameter_get_double(&param_server, param_name, &second_get_value), RCL_RET_OK);
    ASSERT_EQ(first_get_value, second_get_value);
  }
}

TEST_P(ParameterTestBase, rclcpp_set_get_parameter) {
  int expected_callcack_calls = 1;

  // List parameters check
  std::vector<std::string> param_names = {"param1", "param2", "param3"};
  auto list_params = parameters_client->list_parameters({}, 4, default_spin_timeout);
  ASSERT_EQ(list_params.names.size(), param_names.size());
  for (auto & name : list_params.names) {
    std::vector<std::string>::iterator it;
    it = std::find(param_names.begin(), param_names.end(), name);
    ASSERT_NE(it, param_names.end());
  }

  // Set parameters
  {
    std::vector<rclcpp::Parameter> param = {rclcpp::Parameter(param_names[0], true)};
    const std::string param_name = param[0].get_name();

    // Get initial value
    bool get_value = parameters_client->get_parameter<bool>(param_name);
    ASSERT_EQ(get_value, false);

    // Set value
    auto result = parameters_client->set_parameters(param, default_spin_timeout);
    ASSERT_FALSE(result.empty());
    ASSERT_TRUE(result[0].successful);
    ASSERT_EQ(result[0].reason, "");

    // Check callback values
    ASSERT_EQ(callcack_calls, expected_callcack_calls);
    ASSERT_EQ(new_parameter_value.type, RCLC_PARAMETER_BOOL);
    ASSERT_EQ(new_parameter_name, param_name);
    ASSERT_EQ(new_parameter_value.bool_value, param[0].as_bool());
    expected_callcack_calls++;

    // Get new value
    get_value = parameters_client->get_parameter<bool>(param_name);
    ASSERT_EQ(param[0].as_bool(), get_value);
  }

  {
    std::vector<rclcpp::Parameter> param = {rclcpp::Parameter(param_names[1], 10)};
    const std::string param_name = param[0].get_name();

    // Get initial value
    int get_value = parameters_client->get_parameter<int>(param_name);
    ASSERT_EQ(get_value, false);

    // Set value
    auto result = parameters_client->set_parameters(param, default_spin_timeout);
    ASSERT_FALSE(result.empty());
    ASSERT_TRUE(result[0].successful);
    ASSERT_EQ(result[0].reason, "");

    // Check callback values
    ASSERT_EQ(callcack_calls, expected_callcack_calls);
    ASSERT_EQ(new_parameter_value.type, RCLC_PARAMETER_INT);
    ASSERT_EQ(new_parameter_name, param_name);
    ASSERT_EQ(new_parameter_value.integer_value, param[0].as_int());
    expected_callcack_calls++;

    // Get new value
    get_value = parameters_client->get_parameter<int>(param_name);
    ASSERT_EQ(param[0].as_int(), get_value);
  }

  {
    std::vector<rclcpp::Parameter> param = {rclcpp::Parameter(param_names[2], -0.01)};
    const std::string param_name = param[0].get_name();

    // Get initial value
    double get_value = parameters_client->get_parameter<double>(param_name);
    ASSERT_EQ(get_value, false);

    // Set value
    auto result = parameters_client->set_parameters(param, default_spin_timeout);
    ASSERT_FALSE(result.empty());
    ASSERT_TRUE(result[0].successful);
    ASSERT_EQ(result[0].reason, "");

    // Check callback values
    ASSERT_EQ(callcack_calls, expected_callcack_calls);
    ASSERT_EQ(new_parameter_value.type, RCLC_PARAMETER_DOUBLE);
    ASSERT_EQ(new_parameter_name, param_name);
    ASSERT_EQ(new_parameter_value.double_value, param[0].as_double());
    expected_callcack_calls++;

    // Get new value
    get_value = parameters_client->get_parameter<double>(param_name);
    ASSERT_EQ(param[0].as_double(), get_value);
  }

  // Fail with user reject
  user_return = false;
  {
    std::vector<rclcpp::Parameter> param = {rclcpp::Parameter(param_names[2], -0.05)};
    const std::string param_name = param[0].get_name();

    // Get initial value
    double first_get_value = parameters_client->get_parameter<double>(param_name);

    // Set value
    auto result = parameters_client->set_parameters(param, default_spin_timeout);
    ASSERT_FALSE(result.empty());
    ASSERT_FALSE(result[0].successful);
    ASSERT_EQ(result[0].reason, "Rejected by server");
    ASSERT_EQ(callcack_calls, expected_callcack_calls);
    expected_callcack_calls++;

    // Get value
    double second_get_value = parameters_client->get_parameter<double>(param_name);
    ASSERT_EQ(first_get_value, second_get_value);
  }
}

TEST_P(ParameterTestBase, rclc_delete_parameter) {
  // Get parameter
  const char * param_name = "param1";
  bool param_value;

  ASSERT_EQ(rclc_parameter_get_bool(&param_server, param_name, &param_value), RCL_RET_OK);

  // Delete parameter
  EXPECT_EQ(rclc_delete_parameter(&param_server, param_name), RCL_RET_OK);

  // Get deleted parameter
  ASSERT_EQ(rclc_parameter_get_bool(&param_server, param_name, &param_value), RCL_RET_ERROR);
}

TEST_P(ParameterTestBase, rclcpp_delete_parameter) {
  int expected_callcack_calls = 1;

  // Use RCLCPP to delete and check parameters
  user_return = false;
  const std::vector<std::string> parameters = {"param1"};
  auto result = parameters_client->delete_parameters(parameters, default_spin_timeout);
  ASSERT_FALSE(result.empty());
  ASSERT_FALSE(result[0].successful);
  ASSERT_EQ(result[0].reason, "Rejected by server");
  ASSERT_EQ(callcack_calls, expected_callcack_calls);
  expected_callcack_calls++;

  auto list_params = parameters_client->list_parameters({}, 4, default_spin_timeout);
  ASSERT_EQ(list_params.names.size(), 3u);
  ASSERT_EQ(list_params.names[0], parameters[0]);

  user_return = true;
  result = parameters_client->delete_parameters(parameters, default_spin_timeout);
  ASSERT_FALSE(result.empty());
  ASSERT_TRUE(result[0].successful);
  ASSERT_EQ(result[0].reason, "");
  ASSERT_EQ(callcack_calls, expected_callcack_calls);
  expected_callcack_calls++;

  // Use auxiliar RCLCPP node for check
  list_params = parameters_client->list_parameters({}, 4, default_spin_timeout);
  ASSERT_EQ(list_params.names.size(), 2U);
  ASSERT_EQ(
    std::find(
      list_params.names.begin(),
      list_params.names.end(), parameters[0]), list_params.names.end());
}

TEST_P(ParameterTestBase, rclcpp_add_parameter) {
  std::vector<rclcpp::Parameter> param = {rclcpp::Parameter("param4", 10.5)};

  if (options.allow_undeclared_parameters) {
    int expected_callcack_calls = 1;

    // Reject add parameter
    user_return = false;
    auto result = parameters_client->set_parameters(param, default_spin_timeout);
    ASSERT_FALSE(result.empty());
    ASSERT_FALSE(result[0].successful);
    ASSERT_EQ(result[0].reason, "New parameter rejected");
    ASSERT_EQ(callcack_calls, expected_callcack_calls);
    expected_callcack_calls++;

    auto list_params = parameters_client->list_parameters({}, 4, default_spin_timeout);
    ASSERT_EQ(list_params.names.size(), 3u);
    ASSERT_EQ(
      std::find(
        list_params.names.begin(), list_params.names.end(),
        param[0].get_name()), list_params.names.end());

    // Accept add parameter
    user_return = true;
    result = parameters_client->set_parameters(param, default_spin_timeout);
    ASSERT_FALSE(result.empty());
    ASSERT_TRUE(result[0].successful);
    ASSERT_EQ(result[0].reason, "New parameter added");
    ASSERT_EQ(callcack_calls, expected_callcack_calls);

    list_params = parameters_client->list_parameters({}, 4, default_spin_timeout);
    ASSERT_EQ(list_params.names.size(), 4u);
    ASSERT_EQ(list_params.names[3], param[0].get_name());

    // Check value and type
    double param_value = parameters_client->get_parameter<double>(param[0].get_name());
    ASSERT_EQ(param_value, 10.5);

    // Reject parameter on full server
    user_return = false;
    param.clear();
    param.push_back(rclcpp::Parameter("param5", 12.2));
    result = parameters_client->set_parameters(param, default_spin_timeout);
    ASSERT_FALSE(result.empty());
    ASSERT_FALSE(result[0].successful);
    ASSERT_EQ(result[0].reason, "Parameter server is full");
    ASSERT_EQ(callcack_calls, expected_callcack_calls);
  } else {
    // Reject add parameter
    auto result = parameters_client->set_parameters(param, default_spin_timeout);
    ASSERT_FALSE(result.empty());
    ASSERT_FALSE(result[0].successful);
    ASSERT_EQ(result[0].reason, "Parameter not found");
    ASSERT_EQ(callcack_calls, 0);

    auto list_params = parameters_client->list_parameters({}, 4, default_spin_timeout);
    ASSERT_EQ(list_params.names.size(), 3u);
    ASSERT_EQ(
      std::find(
        list_params.names.begin(), list_params.names.end(),
        param[0].get_name()), list_params.names.end());
  }
}

TEST_P(ParameterTestBase, rclcpp_read_only_parameter) {
  ASSERT_EQ(rclc_set_parameter_read_only(&param_server, "param2", true), RCL_RET_OK);

  std::vector<rclcpp::Parameter> param = {rclcpp::Parameter("param2", 50)};

  // Reject set parameter on read only parameter
  auto result = parameters_client->set_parameters(param, default_spin_timeout);
  ASSERT_FALSE(result.empty());
  ASSERT_FALSE(result[0].successful);
  ASSERT_EQ(result[0].reason, "Read only parameter");
  ASSERT_EQ(callcack_calls, 0);

  // Check read only flag on descriptor
  std::vector<std::string> params = {param[0].get_name()};
  auto description = parameters_client->describe_parameters(params);
  ASSERT_FALSE(description.empty());
  ASSERT_TRUE(description[0].read_only);
}

TEST_P(ParameterTestBase, rclcpp_parameter_description) {
  if (options.low_mem_mode) {
    GTEST_SKIP();
  }

  std::vector<std::string> params = {"param2", "param3"};
  std::string parameter_description = "Parameter 2";
  std::string additional_constraints = "Constrains applied";
  int64_t int_from = -10;
  int64_t int_to = 20;
  uint64_t int_step = 2;
  double double_from = -0.5;
  double double_to = 0.5;
  double double_step = 0.01;

  // Set parameter description and constrains
  ASSERT_EQ(
    rclc_add_parameter_description(
      &param_server, "param2", parameter_description.c_str(),
      additional_constraints.c_str()), RCL_RET_OK);
  ASSERT_EQ(
    rclc_add_parameter_constraints_integer(
      &param_server, "param2", int_from, int_to,
      int_step), RCL_RET_OK);
  ASSERT_EQ(
    rclc_add_parameter_constraints_double(
      &param_server, "param3", double_from, double_to,
      double_step), RCL_RET_OK);

  // Check parameter descriptions
  auto description = parameters_client->describe_parameters(params);
  ASSERT_EQ(description.size(), params.size());
  ASSERT_EQ(description[0].name, params[0]);
  ASSERT_EQ(description[0].type, RCLC_PARAMETER_INT);
  ASSERT_EQ(description[0].description, parameter_description);
  ASSERT_EQ(description[0].additional_constraints, additional_constraints);
  ASSERT_EQ(description[0].floating_point_range.size(), 0U);
  ASSERT_EQ(description[0].integer_range.size(), 1U);
  ASSERT_EQ(description[0].integer_range[0].from_value, int_from);
  ASSERT_EQ(description[0].integer_range[0].to_value, int_to);
  ASSERT_EQ(description[0].integer_range[0].step, int_step);

  ASSERT_EQ(description[1].name, params[1]);
  ASSERT_EQ(description[1].type, RCLC_PARAMETER_DOUBLE);
  // TODO(acuadros95): Check empty string filled with spaces
  // ASSERT_TRUE(description[1].description.empty());
  // ASSERT_TRUE(description[1].additional_constraints.empty());
  ASSERT_EQ(description[1].integer_range.size(), 0U);
  ASSERT_EQ(description[1].floating_point_range.size(), 1U);
  ASSERT_EQ(description[1].floating_point_range[0].from_value, double_from);
  ASSERT_EQ(description[1].floating_point_range[0].to_value, double_to);
  ASSERT_EQ(description[1].floating_point_range[0].step, double_step);

  // TODO(acuadros95): Test again after new/delete parameter
}

TEST_P(ParameterTestBase, rclcpp_parameter_description_low) {
  if (!options.low_mem_mode) {
    GTEST_SKIP();
  }

  std::vector<rclcpp::Parameter> param = {
    rclcpp::Parameter("param2", 0),
    rclcpp::Parameter("param3", 0.1)
  };

  int64_t int_from = -10;
  int64_t int_to = 20;
  uint64_t int_step = 2;
  double double_from = -0.5;
  double double_to = 0.5;
  double double_step = 0.01;

  // Set parameter constrains
  ASSERT_EQ(
    rclc_add_parameter_description(&param_server, "param2", "", ""), UNSUPORTED_ON_LOW_MEM);
  ASSERT_EQ(
    rclc_add_parameter_constraints_integer(
      &param_server, "param2", int_from, int_to,
      int_step), RCL_RET_OK);
  ASSERT_EQ(
    rclc_add_parameter_constraints_double(
      &param_server, "param3", double_from, double_to,
      double_step), RCL_RET_OK);

  for (auto & parameter : param) {
    std::vector<std::string> request = {parameter.get_name()};

    // Check parameter descriptions
    auto description = parameters_client->describe_parameters(request);
    ASSERT_EQ(description.size(), request.size());
    ASSERT_EQ(description[0].name, request[0]);

    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      ASSERT_EQ(description[0].type, RCLC_PARAMETER_INT);
      ASSERT_EQ(description[0].floating_point_range.size(), 0U);
      ASSERT_EQ(description[0].integer_range.size(), 1U);
      ASSERT_EQ(description[0].integer_range[0].from_value, int_from);
      ASSERT_EQ(description[0].integer_range[0].to_value, int_to);
      ASSERT_EQ(description[0].integer_range[0].step, int_step);
    } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      ASSERT_EQ(description[0].type, RCLC_PARAMETER_DOUBLE);
      ASSERT_EQ(description[0].integer_range.size(), 0U);
      ASSERT_EQ(description[0].floating_point_range.size(), 1U);
      ASSERT_EQ(description[0].floating_point_range[0].from_value, double_from);
      ASSERT_EQ(description[0].floating_point_range[0].to_value, double_to);
      ASSERT_EQ(description[0].floating_point_range[0].step, double_step);
    }

    // TODO(acuadros95): Test again after new/delete parameter
  }
}

TEST_P(ParameterTestBase, notify_changed_over_dds) {
  // Subscribe to on_parameter_event
  auto promise = std::make_shared<std::promise<void>>();
  auto future = promise->get_future().share();
  bool parameter_change = false;
  bool parameter_added = false;
  bool parameter_deleted = false;

  auto sub = parameters_client->on_parameter_event(
    [&](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
      parameter_change = event->changed_parameters.size() > 0;
      parameter_added = event->new_parameters.size() > 0;
      parameter_deleted = event->deleted_parameters.size() > 0;
      promise->set_value();
    });

  ASSERT_EQ(rclc_parameter_set_bool(&param_server, "param1", false), RCL_RET_OK);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(
      param_client_node, future,
      default_spin_timeout),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(parameter_change);
  ASSERT_FALSE(parameter_added);
  ASSERT_FALSE(parameter_deleted);
}

TEST_P(ParameterTestBase, rclcpp_get_parameter_types) {
  // External get types
  std::vector<rclcpp::Parameter> param = {
    rclcpp::Parameter("param1", true),
    rclcpp::Parameter("param2", 10),
    rclcpp::Parameter("param3", 0.1)
  };

  if (options.low_mem_mode) {
    // Request types one by one
    for (auto & parameter : param) {
      const std::vector<std::string> param_name = {
        parameter.get_name()
      };
      auto types = parameters_client->get_parameter_types(
        param_name,
        default_spin_timeout);

      ASSERT_EQ(types.size(), 1U);
      ASSERT_EQ(types[0], parameter.get_type());
    }
  } else {
    // Request all parameters at once
    const std::vector<std::string> types_query = {
      param[0].get_name(),
      param[1].get_name(),
      param[2].get_name()
    };
    std::vector<rclcpp::ParameterType> types = parameters_client->get_parameter_types(
      types_query,
      default_spin_timeout);
    ASSERT_EQ(types.size(), types_query.size());
    ASSERT_EQ(types[0], param[0].get_type());
    ASSERT_EQ(types[1], param[1].get_type());
    ASSERT_EQ(types[2], param[2].get_type());
  }
}

// Init parameter server with allow_undeclared_parameters flag
rclc_parameter_options_t options_low_mem = {
  true,  // notify_changed_over_dds
  4,  // max_params
  true,  // allow_undeclared_parameters
  true  // low_mem_mode
};

rclc_parameter_options_t default_options = {
  true,  // notify_changed_over_dds
  4,  // max_params
  false,  // allow_undeclared_parameters
  false  // low_mem_mode
};

INSTANTIATE_TEST_SUITE_P(
  ParametersRclcpp,
  ParameterTestBase,
  ::testing::Values(
    default_options,
    options_low_mem));
