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

#define PRINT_RCL_ERROR(rclc, rcl) do { \
  fprintf(stderr, "["#rclc"] error in "#rcl": %s\n", rcl_get_error_string_safe()); \
  rcl_reset_error(); \
  } while(0)

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
    PRINT_RCL_ERROR(rclc_init, rcl_init);
  }
  return ret;
}

bool
rclc_ok(void)
{
  return rcl_ok();
}

void
rclc_sleep_ms(size_t milliseconds)
{
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_ret_t rc = rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, rcl_get_default_allocator());
  if(rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rclc_sleep_ms, rcl_wait_set_init);
  }

  rc = rcl_wait_set_clear_subscriptions(&wait_set);
  if(rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rclc_sleep_ms, rcl_wait_set_clear_subscriptions);
  }

  rc = rcl_wait(&wait_set, RCL_MS_TO_NS(milliseconds));
  if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
    PRINT_RCL_ERROR(rclc_sleep_ms, rcl_wait);
    return;
  }

  rc = rcl_wait_set_fini(&wait_set);
  if(rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rclc_sleep_ms, rcl_wait_set_fini);
  }
}

static
inline
void
_rclc_spin_node_exit(rcl_wait_set_t * wait_set) {
  rcl_ret_t rc = rcl_wait_set_fini(wait_set);
  if(rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rclc_spin_node, rcl_wait_set_fini);
  }
}

void
rclc_spin_node(rclc_node_t * node)
{
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_ret_t rc = rcl_wait_set_init(&wait_set, node->subs_s, 0, 0, 0, 0, rcl_get_default_allocator());
  if(rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rclc_spin_node, rcl_wait_set_init);
    return;
  }

  while(rcl_ok()) {
    rc = rcl_wait_set_clear_subscriptions(&wait_set);
    if(rc != RCL_RET_OK) {
      PRINT_RCL_ERROR(rclc_spin_node, rcl_wait_set_clear_subscriptions);
      _rclc_spin_node_exit(&wait_set);
      return;
    }

    for (size_t i = 0; i < node->subs_s; ++i) {
      rc = rcl_wait_set_add_subscription(&wait_set, &node->subs[i]->rcl_subscription);
      if(rc != RCL_RET_OK) {
        PRINT_RCL_ERROR(rclc_spin_node, rcl_wait_set_add_subscription);
        _rclc_spin_node_exit(&wait_set);
        return;
      }
    }

    rc = rcl_wait(&wait_set, -1);
    if(rc != RCL_RET_OK) {
      PRINT_RCL_ERROR(rclc_spin_node, rcl_wait);
      _rclc_spin_node_exit(&wait_set);
      return;
    }

    for (size_t i = 0; i < wait_set.size_of_subscriptions; ++i) {
      if (wait_set.subscriptions[i]) {
        char msg[64];
        rc = rcl_take(wait_set.subscriptions[i], (void*)&msg, NULL);
        if(rc != RCL_RET_OK) {
          PRINT_RCL_ERROR(rclc_spin_node, rcl_take);
          _rclc_spin_node_exit(&wait_set);
          return;
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

  _rclc_spin_node_exit(&wait_set);
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
    PRINT_RCL_ERROR(rclc_create_node, rcl_node_init);
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
    PRINT_RCL_ERROR(rclc_create_publisher, rcl_publisher_init);
    return NULL;
  }

  return ret;
}

rclc_ret_t
rclc_destroy_publisher(rclc_publisher_t * publisher)
{
  rcl_ret_t rc = rcl_publisher_fini(&publisher->rcl_publisher, &publisher->node->rcl_node);
  if(rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rclc_destroy_publisher, rcl_publisher_fini);
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
    PRINT_RCL_ERROR(rclc_create_subscription, rcl_subscription_init);
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
    PRINT_RCL_ERROR(rclc_destroy_subscription, rcl_subscription_fini);
  }
  DEALLOCATE(subscription);
  return RCL_RET_OK;
}
