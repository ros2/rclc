#include <rclc/rclc.h>

#include <rcl/rcl.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_SUB (10)

struct rclc_subscription_t
{
  rcl_subscription_t * rcl_subscription;
  rclc_callback_t user_callback;
};

struct rclc_publisher_t
{
  rcl_publisher_t * rcl_publisher;

  rclc_node_t * node;
};

struct rclc_node_t
{
  rcl_node_t * rcl_node;

  rclc_subscription_t ** subs;
  size_t subs_s;
};

rclc_ret_t rclc_init(int argc, char ** argv)
{
  rcl_ret_t ret = rcl_init(argc, argv, rcl_get_default_allocator());
  if(ret != RCL_RET_OK) {
    puts("ERROR : rcl_init");
  }
  return ret;
}

bool rclc_ok()
{
  return true;
}

void rclc_sleep_ms(size_t milliseconds)
{
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_ret_t rc = rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, rcl_get_default_allocator());
  if(rc != RCL_RET_OK) {
    puts("ERROR : rcl_wait_set_init");
  }

  rc = rcl_wait_set_clear_subscriptions(&wait_set);
  if(rc != RCL_RET_OK) {
    puts("ERROR : rcl_wait_set_clear_subscriptions");
  }

  rc = rcl_wait(&wait_set, RCL_MS_TO_NS(milliseconds));
  if (rc == RCL_RET_TIMEOUT) {
    rc = rcl_wait_set_fini(&wait_set);
    if(rc != RCL_RET_OK) {
      puts("ERROR : rcl_wait_set_fini");
    }
    return;
  }

  rc = rcl_wait_set_fini(&wait_set);
  if(rc != RCL_RET_OK) {
    puts("ERROR : rcl_wait_set_fini");
  }
}

void rclc_spin_node(rclc_node_t * node)
{
  (void)node;

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_ret_t rc = rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, rcl_get_default_allocator());
  if(rc != RCL_RET_OK) {
    puts("ERROR : rcl_wait_set_init");
  }

  while(1) {
    rc = rcl_wait_set_clear_subscriptions(&wait_set);
    if(rc != RCL_RET_OK) {
      puts("ERROR : rcl_wait_set_clear_subscriptions");
    }

    for (size_t i = 0; i < node->subs_s; ++i) {
      rc = rcl_wait_set_add_subscription(&wait_set, node->subs[i]->rcl_subscription);
      if(rc != RCL_RET_OK) {
        puts("ERROR : rcl_wait_set_add_subscription");
      }
    }

    rc = rcl_wait(&wait_set, RCL_MS_TO_NS(1000));  // 1000ms == 1s, passed as ns
    if (rc == RCL_RET_TIMEOUT) {
      continue;
    }

    for (size_t i = 0; i < wait_set.size_of_subscriptions; ++i) {
      if (wait_set.subscriptions[i]) {
        char msg[64];
        rc = rcl_take(wait_set.subscriptions[i], (void*)&msg, NULL);
        if(rc != RCL_RET_OK) {
          puts("ERROR : rcl_take");
        }
        else {
          for (size_t i = 0; i < node->subs_s; ++i) {
            if(node->subs[i]->rcl_subscription == wait_set.subscriptions[i]) {
              node->subs[i]->user_callback((const void*)&msg);
            }
          }
        }
      }
    }
  }

  rc = rcl_wait_set_fini(&wait_set);
  if(rc != RCL_RET_OK) {
    puts("ERROR : rcl_wait_set_fini");
  }
}

rclc_node_t * rclc_create_node(const char * name)
{
  const char* namespace_ = ""; // TODO : put as parameter

  rclc_node_t* ret = malloc(sizeof(rclc_node_t));
  memset(ret, 0, sizeof(rclc_node_t));

  {
    rcl_node_t node = rcl_get_zero_initialized_node();
    ret->rcl_node = malloc(sizeof(rcl_node_t));
    memcpy(ret->rcl_node, &node, sizeof(rcl_node_t));
  }
  
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rcl_ret_t rc = rcl_node_init(ret->rcl_node, name, namespace_, &node_ops);
  if(rc != RCL_RET_OK) {
    puts("ERROR : rcl_node_init");
    return NULL;
  }
  
  return ret;
}

rclc_ret_t rclc_destroy_node(rclc_node_t * node)
{
  rcl_ret_t ret = rcl_node_fini(node->rcl_node);
  free(node);
  return ret;
}

rclc_publisher_t * rclc_create_publisher(
  rclc_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name,
  size_t queue_size)
{
  (void)queue_size;
  
  rclc_publisher_t* ret = malloc(sizeof(rclc_publisher_t));
  rcl_publisher_t pub = rcl_get_zero_initialized_publisher();
  memcpy(ret, &pub, sizeof(pub));
  
  rcl_publisher_options_t pub_opt = rcl_publisher_get_default_options();
  rcl_ret_t rc = rcl_publisher_init(ret->rcl_publisher, node->rcl_node, type_support, topic_name, &pub_opt);
  if(rc != RCL_RET_OK) {
    puts("ERROR : rcl_publisher_init");
    return NULL;
  }

  ret->node = node;
  
  return ret;
}

rclc_ret_t rclc_destroy_publisher(rclc_publisher_t * publisher)
{
  //rcl_publisher_fini(publisher, node);
  free(publisher);
  return RCL_RET_OK;
}

rclc_ret_t rclc_publish(const rclc_publisher_t * publisher, const void * ros_message)
{
  return rcl_publish(publisher->rcl_publisher, ros_message);
}

rclc_subscription_t * rclc_create_subscription(
  rclc_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name,
  void (* callback)(const void *),
  size_t queue_size,
  bool ignore_local_publications)
{
  (void)queue_size;
  (void)ignore_local_publications;

  rclc_subscription_t* ret = malloc(sizeof(rclc_subscription_t));

  {
    rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
    ret->rcl_subscription = malloc(sizeof(rcl_subscription_t));
    memcpy(ret->rcl_subscription, &subscription, sizeof(rcl_subscription_t));
  }

  ret->user_callback = callback;

  rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
  rcl_ret_t rc = rcl_subscription_init(ret->rcl_subscription, node->rcl_node, type_support, topic_name, &subscription_ops);
  if(rc != RCL_RET_OK) {
    puts("ERROR : rcl_subscription_init");
    return NULL;
  }

  if(node->subs == NULL) {
    node->subs = malloc(sizeof(rclc_subscription_t*));
  }
  node->subs_s++;
  node->subs = realloc(node->subs, sizeof(rclc_subscription_t*)*node->subs_s);
  node->subs[node->subs_s-1] = ret;
  return ret;
}

rclc_ret_t rclc_destroy_subscription(rclc_subscription_t * subscription)
{
  // TODO : remove from subs list
  //rcl_subscription_fini(subscription, node);
  free(subscription);
  return RCL_RET_OK;
}
