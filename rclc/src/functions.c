#include <rclc/rclc.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*/
#define ALLOCATE(s) rcl_get_default_allocator().allocate(s, rcl_get_default_allocator().state)
#define DEALLOCATE(ptr) rcl_get_default_allocator().deallocate(ptr, rcl_get_default_allocator().state)
#define REALLOCATE(ptr, s) rcl_get_default_allocator().reallocate(ptr, s, rcl_get_default_allocator().state)
/*/
#define ALLOCATE(s) malloc(s)
#define DEALLOCATE(ptr) free(ptr)
#define REALLOCATE(ptr, s) realloc(ptr, s)
//*/

struct rclc_subscription_t
{
  rcl_subscription_t rcl_subscription;
  rclc_callback_t user_callback;

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
  rcl_ret_t ret = rcl_init(argc, argv, rcl_get_default_allocator());
  if(ret != RCL_RET_OK) {
    fprintf(stderr, "[rclc_init] error in rcl_init: %s\n", rcl_get_error_string_safe());
    rcl_reset_error();
  }
  return ret;
}

bool
rclc_ok(void)
{
  return true;
}

void
rclc_sleep_ms(size_t milliseconds)
{
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_ret_t rc = rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, rcl_get_default_allocator());
  if(rc != RCL_RET_OK) {
    fprintf(stderr, "[rclc_sleep_ms] error in rcl_wait_set_init: %s\n", rcl_get_error_string_safe());
    rcl_reset_error();
  }

  rc = rcl_wait_set_clear_subscriptions(&wait_set);
  if(rc != RCL_RET_OK) {
    fprintf(stderr, "[rclc_sleep_ms] error in rcl_wait_set_clear_subscriptions: %s\n", rcl_get_error_string_safe());
    rcl_reset_error();
  }

  rc = rcl_wait(&wait_set, RCL_MS_TO_NS(milliseconds));
  if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
    fprintf(stderr, "[rclc_sleep_ms] error in rcl_wait: %s\n", rcl_get_error_string_safe());
    rcl_reset_error();
    return;
  }

  rc = rcl_wait_set_fini(&wait_set);
  if(rc != RCL_RET_OK) {
    fprintf(stderr, "[rclc_sleep_ms] error in rcl_wait_set_fini: %s\n", rcl_get_error_string_safe());
    rcl_reset_error();
  }
}

void
rclc_spin_node(rclc_node_t * node)
{
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_ret_t rc = rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, rcl_get_default_allocator());
  if(rc != RCL_RET_OK) {
    fprintf(stderr, "[rclc_spin_node] error in rcl_wait_set_init: %s\n", rcl_get_error_string_safe());
    rcl_reset_error();
  }

  while(1) {
    rc = rcl_wait_set_clear_subscriptions(&wait_set);
    if(rc != RCL_RET_OK) {
      fprintf(stderr, "[rclc_spin_node] error in rcl_wait_set_clear_subscriptions: %s\n", rcl_get_error_string_safe());
      rcl_reset_error();
    }

    for (size_t i = 0; i < node->subs_s; ++i) {
      rc = rcl_wait_set_add_subscription(&wait_set, &node->subs[i]->rcl_subscription);
      if(rc != RCL_RET_OK) {
        fprintf(stderr, "[rclc_spin_node] error in rcl_wait_set_add_subscription: %s\n", rcl_get_error_string_safe());
        rcl_reset_error();
      }
    }

    rc = rcl_wait(&wait_set, RCL_MS_TO_NS(1000));  // 1000ms == 1s, passed as ns
    if (rc == RCL_RET_TIMEOUT) {
      continue;
    }
    else if(rc != RCL_RET_OK) {
      fprintf(stderr, "[rclc_spin_node] error in rcl_wait: %s\n", rcl_get_error_string_safe());
      rcl_reset_error();
    }

    for (size_t i = 0; i < wait_set.size_of_subscriptions; ++i) {
      if (wait_set.subscriptions[i]) {
        char msg[64];
        rc = rcl_take(wait_set.subscriptions[i], (void*)&msg, NULL);
        if(rc != RCL_RET_OK) {
          fprintf(stderr, "[rclc_spin_node] error in rcl_take: %s\n", rcl_get_error_string_safe());
          rcl_reset_error();
        }
        else {
          for (size_t i = 0; i < node->subs_s; ++i) {
            if(&node->subs[i]->rcl_subscription == wait_set.subscriptions[i]) {
              node->subs[i]->user_callback((const void*)&msg);
            }
          }
        }
      }
    }
  }

  rc = rcl_wait_set_fini(&wait_set);
  if(rc != RCL_RET_OK) {
    fprintf(stderr, "[rclc_spin_node] error in rcl_wait_set_fini: %s\n", rcl_get_error_string_safe());
    rcl_reset_error();
  }
}

rclc_node_t *
rclc_create_node(const char * name, const char * namespace_)
{
  rclc_node_t* ret = ALLOCATE(sizeof(rclc_node_t));
  ret->rcl_node = rcl_get_zero_initialized_node();
  ret->subs = NULL;
  ret->subs_s = 0;
  
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rcl_ret_t rc = rcl_node_init(&ret->rcl_node, name, namespace_, &node_ops);
  if(rc != RCL_RET_OK) {
    fprintf(stderr, "[rclc_create_node] error in rcl_node_init: %s\n", rcl_get_error_string_safe());
    rcl_reset_error();
    return NULL;
  }
  
  return ret;
}

rclc_ret_t
rclc_destroy_node(rclc_node_t * node)
{
  rcl_ret_t ret = rcl_node_fini(&node->rcl_node);
  DEALLOCATE(node);
  return ret;
}

rclc_publisher_t *
rclc_create_publisher(
  rclc_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name,
  size_t queue_size)
{
  (void)queue_size;
  
  rclc_publisher_t* ret = ALLOCATE(sizeof(rclc_publisher_t));
  ret->rcl_publisher = rcl_get_zero_initialized_publisher();
  ret->node = node;

  rcl_publisher_options_t pub_opt = rcl_publisher_get_default_options();
  rcl_ret_t rc = rcl_publisher_init(&ret->rcl_publisher, &node->rcl_node, type_support, topic_name, &pub_opt);
  if(rc != RCL_RET_OK) {
    fprintf(stderr, "[rclc_create_publisher] error in rcl_publisher_init: %s\n", rcl_get_error_string_safe());
    rcl_reset_error();
    return NULL;
  }

  return ret;
}

rclc_ret_t
rclc_destroy_publisher(rclc_publisher_t * publisher)
{
  rcl_ret_t rc = rcl_publisher_fini(&publisher->rcl_publisher, &publisher->node->rcl_node);
  if(rc != RCL_RET_OK) {
    fprintf(stderr, "[rclc_destroy_publisher] error in rcl_publisher_fini: %s\n", rcl_get_error_string_safe());
    rcl_reset_error();
  }
  DEALLOCATE(publisher);
  return RCL_RET_OK;
}

rclc_ret_t
rclc_publish(const rclc_publisher_t * publisher, const void * ros_message)
{
  return rcl_publish(&publisher->rcl_publisher, ros_message);
}

rclc_subscription_t *
rclc_create_subscription(
  rclc_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name,
  rclc_callback_t callback,
  size_t queue_size,
  bool ignore_local_publications)
{
  (void)queue_size;
  (void)ignore_local_publications;

  rclc_subscription_t* ret = ALLOCATE(sizeof(rclc_subscription_t));
  ret->rcl_subscription = rcl_get_zero_initialized_subscription();
  ret->user_callback = callback;
  ret->node = node;

  rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
  rcl_ret_t rc = rcl_subscription_init(&ret->rcl_subscription, &node->rcl_node, type_support, topic_name, &subscription_ops);
  if(rc != RCL_RET_OK) {
    fprintf(stderr, "[rclc_create_subscription] error in rcl_subscription_init: %s\n", rcl_get_error_string_safe());
    rcl_reset_error();
    return NULL;
  }

  if(node->subs == NULL) {
    node->subs = ALLOCATE(sizeof(rclc_subscription_t*));
  }
  node->subs_s++;
  node->subs = REALLOCATE(node->subs, sizeof(rclc_subscription_t*)*node->subs_s);
  node->subs[node->subs_s-1] = ret;
  return ret;
}

rclc_ret_t
rclc_destroy_subscription(rclc_subscription_t * subscription)
{
  // TODO : remove from subs list
  rcl_ret_t rc = rcl_subscription_fini(&subscription->rcl_subscription, &subscription->node->rcl_node);
  if(rc != RCL_RET_OK) {
    fprintf(stderr, "[rclc_destroy_subscription] error in rcl_subscription_fini: %s\n", rcl_get_error_string_safe());
    rcl_reset_error();
  }
  DEALLOCATE(subscription);
  return RCL_RET_OK;
}
