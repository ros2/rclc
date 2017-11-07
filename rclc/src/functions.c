#include <rclc/rclc.h>

#include <rcl/rcl.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_SUB (10)

static rclc_subscription_t* _subs[MAX_SUB] = {NULL};
static size_t _subs_s = 0;

rclc_ret_t rclc_init(int argc, char ** argv) {
  rcl_ret_t ret = rcl_init(argc, argv, rcl_get_default_allocator());
  if(ret != RCL_RET_OK) {
    puts("ERROR : rcl_init");
  }
  return ret;
}

bool rclc_ok() {
  return true;
}

void rclc_sleep_ms(size_t milliseconds) {
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_ret_t rc = rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, rcl_get_default_allocator());
  if(rc != RCL_RET_OK) {
    puts("ERROR : rcl_wait_set_init");
  }

  rc = rcl_wait_set_clear_subscriptions(&wait_set);
  if(rc != RCL_RET_OK) {
    puts("ERROR : rcl_wait_set_clear_subscriptions");
  }

  for (size_t i = 0; i < _subs_s; ++i) {
    rc = rcl_wait_set_add_subscription(&wait_set, _subs[i]->rcl_subscription);
    if(rc != RCL_RET_OK) {
      puts("ERROR : rcl_wait_set_add_subscription");
    }
  }

  rc = rcl_wait(&wait_set, RCL_MS_TO_NS(milliseconds));
  if (rc == RCL_RET_TIMEOUT) {
    rc = rcl_wait_set_fini(&wait_set);
    if(rc != RCL_RET_OK) {
      puts("ERROR : rcl_wait_set_fini");
    }
    return;
  }

  for (size_t i = 0; i < wait_set.size_of_subscriptions; ++i) {
    if (wait_set.subscriptions[i]) {
      char msg[64];
      rc = rcl_take(wait_set.subscriptions[i], (void*)&msg, NULL);
      if(rc != RCL_RET_OK) {
        puts("ERROR : rcl_take");
      }
      else {
        for (size_t i = 0; i < _subs_s; ++i) {
          if(_subs[i]->rcl_subscription == wait_set.subscriptions[i]) {
            _subs[i]->user_callback((const void*)&msg);
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

void rclc_spin_node(rclc_node_t * node) {
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

    for (size_t i = 0; i < _subs_s; ++i) {
      rc = rcl_wait_set_add_subscription(&wait_set, _subs[i]->rcl_subscription);
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
        puts("rcl_take");
        rc = rcl_take(wait_set.subscriptions[i], (void*)&msg, NULL);
        if(rc != RCL_RET_OK) {
          puts("ERROR : rcl_take");
        }
        else {
          for (size_t i = 0; i < _subs_s; ++i) {
            if(_subs[i]->rcl_subscription == wait_set.subscriptions[i]) {
              _subs[i]->user_callback((const void*)&msg);
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

rclc_node_t * rclc_create_node(const char * name) {
  const char* namespace_ = ""; // TODO : put as parameter

  rclc_node_t* ret = malloc(sizeof(rclc_node_t));
  rcl_node_t node = rcl_get_zero_initialized_node();
  memcpy(ret, &node, sizeof(node));
  
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rcl_ret_t rc = rcl_node_init(ret, name, namespace_, &node_ops);
  if(rc != RCL_RET_OK) {
    puts("ERROR : rcl_node_init");
    return NULL;
  }
  
  return ret;
}

rclc_ret_t rclc_destroy_node(rclc_node_t * node) {
  rcl_ret_t ret = rcl_node_fini(node);
  free(node);
  return ret;
}

rclc_publisher_t * rclc_create_publisher(
  const rclc_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name,
  size_t queue_size) {
  (void)queue_size;
  
  rclc_publisher_t* ret = malloc(sizeof(rclc_publisher_t));
  rcl_publisher_t pub = rcl_get_zero_initialized_publisher();
  memcpy(ret, &pub, sizeof(pub));
  
  rcl_publisher_options_t pub_opt = rcl_publisher_get_default_options();
  rcl_ret_t rc = rcl_publisher_init(ret, node, type_support, topic_name, &pub_opt);
  if(rc != RCL_RET_OK) {
    puts("ERROR : rcl_publisher_init");
    return NULL;
  }
  
  return ret;
}

rclc_ret_t rclc_destroy_publisher(rclc_publisher_t * publisher) {
  //rcl_publisher_fini(publisher, node);
  free(publisher);
  return RCL_RET_OK;
}

rclc_ret_t rclc_publish(const rclc_publisher_t * publisher, const void * ros_message) {
  return rcl_publish(publisher, ros_message);
}

rclc_subscription_t * rclc_create_subscription(
  const rclc_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name,
  void (* callback)(const void *),
  size_t queue_size,
  bool ignore_local_publications) {
  (void)queue_size;
  (void)ignore_local_publications;
  
  rclc_subscription_t* ret = malloc(sizeof(rclc_subscription_t));
  ret->rcl_subscription = malloc(sizeof(rcl_subscription_t));
  rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
  memcpy(ret->rcl_subscription, &subscription, sizeof(subscription));
  ret->user_callback = callback;
  
  rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
  rcl_ret_t rc = rcl_subscription_init(ret->rcl_subscription, node, type_support, topic_name, &subscription_ops);
  if(rc != RCL_RET_OK) {
    puts("ERROR : rcl_subscription_init");
    return NULL;
  }
  
  _subs[_subs_s++] = ret;
  return ret;
}

rclc_ret_t rclc_destroy_subscription(rclc_subscription_t * subscription) {
  // TODO : remove from subs list
  //rcl_subscription_fini(subscription, node);
  free(subscription);
  return RCL_RET_OK;
}

rclc_executor_t * rclc_create_executor();
rclc_ret_t rclc_destroy_executor(rclc_executor_t * executor);
rclc_ret_t rclc_executor_add_node(rclc_executor_t * executor, const rclc_node_t * node);
rclc_ret_t rclc_executor_spin();
