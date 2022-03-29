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

#include "rclc_parameter/parameter_utils.h"

using namespace std::chrono_literals;

TEST(ParameterTestUnitary, rclc_parameter_server_init_default) {
  std::string node_name("test_node");

  // Init RCLC support
  rclc_support_t support;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  ASSERT_EQ(rclc_support_init(&support, 0, nullptr, &allocator), RCL_RET_OK);

  // Init node
  rcl_node_t node;
  ASSERT_EQ(rclc_node_init_default(&node, node_name.c_str(), "", &support), RCL_RET_OK);

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
}

TEST(ParameterTestUnitary, rclc_add_parameter) {
  std::string node_name("test_node");

  // Init RCLC support
  rclc_support_t support;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  ASSERT_EQ(rclc_support_init(&support, 0, nullptr, &allocator), RCL_RET_OK);
  // Init node
  rcl_node_t node;
  ASSERT_EQ(rclc_node_init_default(&node, node_name.c_str(), "", &support), RCL_RET_OK);

  // Init parameter server
  rclc_parameter_server_t param_server;
  ASSERT_EQ(rclc_parameter_server_init_default(&param_server, &node), RCL_RET_OK);

  // Test add parameter
  std::vector<std::string> param_names;
  ASSERT_EQ(rclc_add_parameter(&param_server, "param1", RCLC_PARAMETER_BOOL), RCL_RET_OK);
  param_names.push_back("param1");

  // Fail on duplicated name
  ASSERT_EQ(rclc_add_parameter(&param_server, "param1", RCLC_PARAMETER_DOUBLE), RCL_RET_ERROR);

  // Fail on name length
  char overflow_name[RCLC_PARAMETER_MAX_STRING_LENGTH + 1];
  memset(overflow_name, ' ', RCLC_PARAMETER_MAX_STRING_LENGTH + 1);
  overflow_name[RCLC_PARAMETER_MAX_STRING_LENGTH] = '\0';

  ASSERT_EQ(rclc_add_parameter(&param_server, overflow_name, RCLC_PARAMETER_BOOL), RCL_RET_ERROR);

  // Add more parameters
  ASSERT_EQ(rclc_add_parameter(&param_server, "param2", RCLC_PARAMETER_INT), RCL_RET_OK);
  param_names.push_back("param2");

  ASSERT_EQ(rclc_add_parameter(&param_server, "param3", RCLC_PARAMETER_DOUBLE), RCL_RET_OK);
  param_names.push_back("param3");

  ASSERT_EQ(rclc_add_parameter(&param_server, "param4", RCLC_PARAMETER_DOUBLE), RCL_RET_OK);
  param_names.push_back("param4");

  // Fail on too much params
  ASSERT_EQ(rclc_add_parameter(&param_server, "param5", RCLC_PARAMETER_DOUBLE), RCL_RET_ERROR);

  // Destroy parameter server
  ASSERT_EQ(rclc_parameter_server_fini(&param_server, &node), RCL_RET_OK);
}

class ParameterTestBase : public ::testing::Test
{
public:
  ParameterTestBase()
  : default_spin_timeout(std::chrono::duration<int64_t, std::milli>(10000))
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
    rcl_allocator_t allocator = rcl_get_default_allocator();
    EXPECT_EQ(rclc_support_init(&support, 0, nullptr, &allocator), RCL_RET_OK);

    // Init node
    EXPECT_EQ(rclc_node_init_default(&node, node_name.c_str(), "", &support), RCL_RET_OK);

    // Init parameter server with allow_undeclared_parameters flag
    rclc_parameter_options_t options;
    options.notify_changed_over_dds = true;
    options.max_params = 4;
    options.allow_undeclared_parameters = true;

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
          ASSERT_EQ(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(500)), RCL_RET_OK);
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
    std::this_thread::sleep_for(500ms);
  }

  void TearDown() override
  {
    rclcpp::shutdown();

    spin = false;
    rclc_parameter_server_thread.join();

    // Destroy parameter server
    ASSERT_EQ(rclc_parameter_server_fini(&param_server, &node), RCL_RET_OK);
  }

  static bool on_parameter_changed(const Parameter * old_param, const Parameter * new_param)
  {
    if (new_param == NULL) {
      strncpy(new_parameter_name, "null", sizeof(new_parameter_name));
      new_parameter_value.type = RCLC_PARAMETER_NOT_SET;
    } else {
      strncpy(new_parameter_name, new_parameter_value->name.data, sizeof(new_parameter_name));
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
  rclc_support_t support;
  rcl_node_t node;
  rclc_executor_t executor;
  rclc_parameter_server_t param_server;
  std::thread rclc_parameter_server_thread;
  bool spin;
};

int ParameterTestBase::callcack_calls;
bool ParameterTestBase::user_return;
char ParameterTestBase::old_parameter_name[];
char ParameterTestBase::new_parameter_name[];
ParameterValue ParameterTestBase::new_parameter_value;
ParameterValue ParameterTestBase::old_parameter_value;

TEST_F(ParameterTestBase, rclc_set_get_parameter) {
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

TEST_F(ParameterTestBase, rclcpp_set_get_parameter) {
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

TEST_F(ParameterTestBase, rclc_delete_parameter) {
  // Get parameter
  const char * param_name = "param1";
  bool param_value;

  ASSERT_EQ(rclc_parameter_get_bool(&param_server, param_name, &param_value), RCL_RET_OK);

  // Delete parameter
  EXPECT_EQ(rclc_delete_parameter(&param_server, param_name), RCL_RET_OK);

  // Get deleted parameter
  ASSERT_EQ(rclc_parameter_get_bool(&param_server, param_name, &param_value), RCL_RET_ERROR);
}

TEST_F(ParameterTestBase, rclcpp_delete_parameter) {
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

TEST_F(ParameterTestBase, rclcpp_add_parameter) {
  int expected_callcack_calls = 1;

  // Reject add parameter
  user_return = false;
  std::vector<rclcpp::Parameter> param =
  {rclcpp::Parameter("param4", 10.5)};
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
}

TEST_F(ParameterTestBase, notify_changed_over_dds) {
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

TEST_F(ParameterTestBase, rclcpp_get_parameter_types) {
  // External get types
  const std::vector<std::string> types_query = {
    "param1",
    "param2",
    "param3"
  };
  std::vector<rclcpp::ParameterType> types = parameters_client->get_parameter_types(
    types_query,
    default_spin_timeout);
  ASSERT_EQ(types.size(), 3u);
  ASSERT_EQ(types[0], rclcpp::ParameterType::PARAMETER_BOOL);
  ASSERT_EQ(types[1], rclcpp::ParameterType::PARAMETER_INTEGER);
  ASSERT_EQ(types[2], rclcpp::ParameterType::PARAMETER_DOUBLE);
}
