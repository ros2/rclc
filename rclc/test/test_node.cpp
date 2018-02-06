// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include "rclc/rclc.h"

#define EXPECT_NULL(ptr) EXPECT_EQ((void *)ptr, (void *)NULL)
#define EXPECT_NON_NULL(ptr) EXPECT_NE((void *)ptr, (void *)NULL)

class TestNode : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclc_init(0, NULL);
  }
};

/*
   Testing node construction and destruction.
 */
TEST_F(TestNode, construction_and_destruction) {
  {
    rclc_node_t * node = rclc_create_node("my_node", "/ns");
    EXPECT_NON_NULL(node);
    rclc_destroy_node(node);
  }

  {
    rclc_node_t * node = rclc_create_node("invalid_node?", "/ns");
    EXPECT_NULL(node);
  }

  {
    rclc_node_t * node = rclc_create_node("my_node", "/invalid_ns?");
    EXPECT_NULL(node);
  }
}
