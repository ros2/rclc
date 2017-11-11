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

#include <rclc/rclc.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ALLOCATE(s) \
  rcl_get_default_allocator().allocate(s, rcl_get_default_allocator().state)
#define DEALLOCATE(ptr) \
  rcl_get_default_allocator().deallocate(ptr, rcl_get_default_allocator().state)
#define REALLOCATE(ptr, s) \
  rcl_get_default_allocator().reallocate(ptr, s, rcl_get_default_allocator().state)
#define ZERO_ALLOCATE(s) \
  rcl_get_default_allocator().zero_allocate(s, 1, rcl_get_default_allocator().state)

#define PRINT_RCL_ERROR(rclc, rcl) \
  do { \
    fprintf(stderr, "[" #rclc "] error in " #rcl ": %s\n", rcl_get_error_string_safe()); \
    rcl_reset_error(); \
  } while (0)

struct rclc_subscription_t
{
  rcl_subscription_t rcl_subscription;
  rclc_callback_t user_callback;

  size_t typesupport_size_of;
  const rosidl_message_type_support_t * type_support;

  rclc_node_t * node;
};

struct rclc_publisher_t
{
  rcl_publisher_t rcl_publisher;

  rclc_node_t * node;
};

struct rclc_node_t
{
  rcl_node_t rcl_node;

  rclc_subscription_t ** subs;
  size_t subs_s;
};

rclc_ret_t
rclc_init(int argc, char ** argv)
{
  rcl_ret_t rc = rcl_init(argc, argv, rcl_get_default_allocator());
  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rclc_init, rcl_init);
  }
  return rc;
}

bool
rclc_ok(void)
{
  return rcl_ok();
}

static
inline
void
_rclc_spin_node_exit(rcl_wait_set_t * wait_set)
{
  rcl_ret_t rc = rcl_wait_set_fini(wait_set);
  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rclc_spin_node, rcl_wait_set_fini);
  }
}

void
rclc_spin_node_once(rclc_node_t * node, size_t timeout_ms)
{
  // [FIXME] Make the wait_set not empty if there is no subscription
  const size_t dummy = node->subs_s ? 0 : 1;

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_ret_t rc =
    rcl_wait_set_init(&wait_set, node->subs_s, 0, dummy, 0, 0, rcl_get_default_allocator());
  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rclc_spin_node, rcl_wait_set_init);
    return;
  }

  rc = rcl_wait_set_clear_subscriptions(&wait_set);
  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rclc_spin_node, rcl_wait_set_clear_subscriptions);
    _rclc_spin_node_exit(&wait_set);
    return;
  }

  for (size_t i = 0; i < node->subs_s; ++i) {
    rc = rcl_wait_set_add_subscription(&wait_set, &node->subs[i]->rcl_subscription);
    if (rc != RCL_RET_OK) {
      PRINT_RCL_ERROR(rclc_spin_node, rcl_wait_set_add_subscription);
      _rclc_spin_node_exit(&wait_set);
      return;
    }
  }

  rc = rcl_wait(&wait_set, RCL_MS_TO_NS(timeout_ms));
  if (rc == RCL_RET_TIMEOUT) {
    _rclc_spin_node_exit(&wait_set);
    return;
  }

  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rclc_spin_node, rcl_wait);
    _rclc_spin_node_exit(&wait_set);
    return;
  }

  for (size_t i = 0; i < wait_set.size_of_subscriptions; ++i) {
    rclc_subscription_t * sub = NULL;

    for (size_t j = 0; j < node->subs_s; ++j) {
      if (&node->subs[j]->rcl_subscription == wait_set.subscriptions[i]) {
        sub = node->subs[j];
      }
    }

    if (sub == NULL) {
      fprintf(stderr, "[rclc_spin_node] unable to find subscription in node.\n");
      _rclc_spin_node_exit(&wait_set);
    }

    void * msg = ZERO_ALLOCATE(sub->typesupport_size_of);

    rc = rcl_take(wait_set.subscriptions[i], msg, NULL);
    if (rc != RCL_RET_OK) {
      PRINT_RCL_ERROR(rclc_spin_node, rcl_take);
      _rclc_spin_node_exit(&wait_set);
      return;
    }

    sub->user_callback(msg);
  }

  _rclc_spin_node_exit(&wait_set);
}

void
rclc_spin_node(rclc_node_t * node)
{
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_ret_t rc =
    rcl_wait_set_init(&wait_set, node->subs_s, 0, 0, 0, 0, rcl_get_default_allocator());
  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rclc_spin_node, rcl_wait_set_init);
    return;
  }

  while (rcl_ok()) {
    rc = rcl_wait_set_clear_subscriptions(&wait_set);
    if (rc != RCL_RET_OK) {
      PRINT_RCL_ERROR(rclc_spin_node, rcl_wait_set_clear_subscriptions);
      _rclc_spin_node_exit(&wait_set);
      return;
    }

    for (size_t i = 0; i < node->subs_s; ++i) {
      rc = rcl_wait_set_add_subscription(&wait_set, &node->subs[i]->rcl_subscription);
      if (rc != RCL_RET_OK) {
        PRINT_RCL_ERROR(rclc_spin_node, rcl_wait_set_add_subscription);
        _rclc_spin_node_exit(&wait_set);
        return;
      }
    }

    rc = rcl_wait(&wait_set, -1);
    if (rc != RCL_RET_OK) {
      PRINT_RCL_ERROR(rclc_spin_node, rcl_wait);
      _rclc_spin_node_exit(&wait_set);
      return;
    }

    for (size_t i = 0; i < wait_set.size_of_subscriptions; ++i) {
      if (wait_set.subscriptions[i]) {
        rclc_subscription_t * sub = NULL;

        for (size_t j = 0; j < node->subs_s; ++j) {
          if (&node->subs[j]->rcl_subscription == wait_set.subscriptions[i]) {
            sub = node->subs[j];
          }
        }

        if (sub == NULL) {
          fprintf(stderr, "[rclc_spin_node] unable to find subscription in node.\n");
          _rclc_spin_node_exit(&wait_set);
        }

        void * msg = ZERO_ALLOCATE(sub->typesupport_size_of);

        rc = rcl_take(wait_set.subscriptions[i], msg, NULL);
        if (rc != RCL_RET_OK) {
          PRINT_RCL_ERROR(rclc_spin_node, rcl_take);
          _rclc_spin_node_exit(&wait_set);
          return;
        }

        sub->user_callback(msg);
      }
    }
  }

  _rclc_spin_node_exit(&wait_set);
}

rclc_node_t *
rclc_create_node(const char * name, const char * namespace_)
{
  rclc_node_t * rclc_node = ALLOCATE(sizeof(rclc_node_t));
  rclc_node->rcl_node = rcl_get_zero_initialized_node();
  rclc_node->subs = NULL;
  rclc_node->subs_s = 0;

  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rcl_ret_t rc = rcl_node_init(&rclc_node->rcl_node, name, namespace_, &node_ops);
  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rclc_create_node, rcl_node_init);
    return NULL;
  }

  return rclc_node;
}

rclc_ret_t
rclc_destroy_node(rclc_node_t * node)
{
  rcl_ret_t rc = rcl_node_fini(&node->rcl_node);
  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rclc_destroy_node, rcl_node_fini);
  }
  DEALLOCATE(node);
  return rc;
}

rclc_publisher_t *
rclc_create_publisher(
  rclc_node_t * node,
  const rclc_type_support_t type_support,
  const char * topic_name,
  size_t queue_size)
{
  (void)queue_size;

  rclc_publisher_t * rclc_publisher = ALLOCATE(sizeof(rclc_publisher_t));
  rclc_publisher->rcl_publisher = rcl_get_zero_initialized_publisher();
  rclc_publisher->node = node;

  rcl_publisher_options_t pub_opt = rcl_publisher_get_default_options();
  rcl_ret_t rc = rcl_publisher_init(
    &rclc_publisher->rcl_publisher,
    &node->rcl_node,
    type_support.rosidl_message_type_support,
    topic_name,
    &pub_opt);
  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rclc_create_publisher, rcl_publisher_init);
    return NULL;
  }

  return rclc_publisher;
}

rclc_ret_t
rclc_destroy_publisher(rclc_publisher_t * publisher)
{
  rcl_ret_t rc = rcl_publisher_fini(&publisher->rcl_publisher, &publisher->node->rcl_node);
  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rclc_destroy_publisher, rcl_publisher_fini);
  }
  DEALLOCATE(publisher);
  return rc;
}

rclc_ret_t
rclc_publish(const rclc_publisher_t * publisher, const void * ros_message)
{
  if (publisher == NULL) {
    fprintf(stderr, "[rclc_publish] null pointer to publisher");
    return RCL_RET_INVALID_ARGUMENT;
  }

  return rcl_publish(&publisher->rcl_publisher, ros_message);
}

rclc_subscription_t *
rclc_create_subscription(
  rclc_node_t * node,
  const rclc_type_support_t type_support,
  const char * topic_name,
  rclc_callback_t callback,
  size_t queue_size,
  bool ignore_local_publications)
{
  (void)queue_size;
  (void)ignore_local_publications;

  rclc_subscription_t * rclc_subscription = ALLOCATE(sizeof(rclc_subscription_t));
  rclc_subscription->rcl_subscription = rcl_get_zero_initialized_subscription();
  rclc_subscription->user_callback = callback;
  rclc_subscription->typesupport_size_of = type_support.size_of;
  rclc_subscription->type_support = type_support.rosidl_message_type_support;

  rclc_subscription->node = node;

  rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();

  rcl_ret_t rc = rcl_subscription_init(
    &rclc_subscription->rcl_subscription,
    &node->rcl_node,
    type_support.rosidl_message_type_support,
    topic_name,
    &subscription_ops);
  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rclc_create_subscription, rcl_subscription_init);
    return NULL;
  }

  if (node->subs == NULL) {
    node->subs = ALLOCATE(sizeof(rclc_subscription_t *));
  }
  node->subs_s++;
  node->subs = REALLOCATE(node->subs, sizeof(rclc_subscription_t *) * node->subs_s);
  node->subs[node->subs_s - 1] = rclc_subscription;
  return rclc_subscription;
}

rclc_ret_t
rclc_destroy_subscription(rclc_subscription_t * subscription)
{
  bool found = false;
  for (size_t i = 0; i < subscription->node->subs_s; ++i) {
    if (found) {
      subscription->node->subs[i - 1] = subscription->node->subs[i];
    }
    if (subscription == subscription->node->subs[i]) {
      found = true;
    }
  }
  if (found) {
    subscription->node->subs_s--;
  }

  rcl_ret_t rc =
    rcl_subscription_fini(&subscription->rcl_subscription, &subscription->node->rcl_node);
  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rclc_destroy_subscription, rcl_subscription_fini);
  }
  DEALLOCATE(subscription);
  return rc;
}
