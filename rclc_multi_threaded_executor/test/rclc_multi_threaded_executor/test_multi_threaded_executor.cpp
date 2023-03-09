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
#include <rclc_multi_threaded_executor/multi_threaded_executor.h>
#include "rclc/rclc.h"
}

TEST(Test, rclc_multi_threaded_executor) {
  // Init rclc
  rclc_support_t support;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  ASSERT_EQ(rclc_support_init(&support, 0, nullptr, &allocator), RCL_RET_OK);

  // Init node
  rcl_node_t node;
  ASSERT_EQ(rclc_node_init_default(&node, "test_node", "", &support), RCL_RET_OK);

  // Init executor
  rclc_executor_t executor;
  rclc_executor_init(
    &executor, &support.context, 10,
    &allocator);
  // Configure multi-threaded executor
  ASSERT_EQ(rclc_multi_threaded_executor_configure(&executor), RCL_RET_OK);

  // ASSERT_EQ(rclc_executor_add_subscription(&executor, &sub_1, nullptr), RCL_RET_OK);
  // rclc_executor_spin_some(&executor);


  // Destroy parameter server
  ASSERT_EQ(rclc_executor_fini(&executor), RCL_RET_OK);
  ASSERT_EQ(rcl_node_fini(&node), RCL_RET_OK);
}
