// Copyright (c) 2022 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/ros2/rclc.
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

#include "rclc_dispatcher_executor/dispatcher_executor.h"
#include <rcutils/logging_macros.h>

static pthread_mutex_t * rclc_micro_ros_mutex;

rcl_ret_t rclc_dispatcher_executor_publish(
  const rcl_publisher_t * publisher,
  const void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  rcl_ret_t ret;
  pthread_mutex_lock(rclc_micro_ros_mutex);
  ret = rcl_publish(publisher, ros_message, allocation);
  pthread_mutex_unlock(rclc_micro_ros_mutex);
  return ret;
}

rclc_handle_multi_threaded_data_t * rclc_get_multi_threaded_handle(rclc_executor_handle_t * handle)
{
  return (rclc_handle_multi_threaded_data_t *) handle->multi_threaded;
}

rclc_executor_multi_threaded_data_t * rclc_get_multi_threaded_executor(rclc_executor_t * executor)
{
  return (rclc_executor_multi_threaded_data_t *) executor->multi_threaded;
}

rcl_ret_t
rclc_dispatcher_executor_add_subscription(
  rclc_executor_t * executor,
  rcl_subscription_t * subscription,
  void * msg,
  rclc_callback_t callback,
  rclc_executor_handle_invocation_t invocation,
  rclc_executor_sched_parameter_t * sparam)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(subscription, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(msg, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(callback, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(sparam, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;
  // array bound check
  if (executor->index >= executor->max_handles) {
    RCL_SET_ERROR_MSG(
      "Buffer overflow: increase number of handles!");
    return RCL_RET_ERROR;
  }

  // assign data fields
  executor->handles[executor->index].type = RCLC_SUBSCRIPTION;
  executor->handles[executor->index].subscription = subscription;
  executor->handles[executor->index].data = msg;
  executor->handles[executor->index].subscription_callback = callback;
  executor->handles[executor->index].invocation = invocation;
  (rclc_get_multi_threaded_handle(&executor->handles[executor->index]))->sparam = sparam;
  executor->handles[executor->index].initialized = true;

  // increase index of handle array
  executor->index++;

  // invalidate wait_set so that in next spin_some() call the
  // 'executor->wait_set' is updated accordingly
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in rclc_executor_add_subscription.");
      return ret;
    }
  }

  executor->info.number_of_subscriptions++;

  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Added a subscription with sched_param.");
  return ret;
}


/// initialization of handle object
rcl_ret_t
rclc_dispatcher_executor_handle_init(
  rclc_executor_t * executor,
  rclc_executor_handle_t * handle)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(handle, RCL_RET_INVALID_ARGUMENT);

  handle->multi_threaded = executor->allocator->allocate(
    sizeof(rclc_handle_multi_threaded_data_t), executor->allocator->state);
  if (NULL == handle->multi_threaded) {
    RCL_SET_ERROR_MSG("Could not allocate memory for 'handle->multi_threaded'.");
    return RCL_RET_BAD_ALLOC;
  }
  // note: worker_thread not initialized - okay
  (rclc_get_multi_threaded_handle(handle))->worker_thread_state = RCLC_THREAD_READY;
  pthread_cond_init(&(rclc_get_multi_threaded_handle(handle))->new_msg_cond, NULL);
  pthread_mutex_init(&(rclc_get_multi_threaded_handle(handle))->new_msg_mutex, NULL);
  (rclc_get_multi_threaded_handle(handle))->new_msg_avail = false;
  (rclc_get_multi_threaded_handle(handle))->sparam = NULL;
  // note: tattr not initialized - okay
  return RCL_RET_OK;
}
// note: add multi_threaded_initialized flag
//       otherwise the functions to add subscription will fail.
rcl_ret_t rclc_dispatcher_executor_init(rclc_executor_t * executor)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);

  // memory allocation and initialization for multi-threaded handles
  for (size_t i = 0; i < executor->max_handles; i++) {
    rclc_dispatcher_executor_handle_init(executor, &executor->handles[i]);
  }

  // memory allocation and initialization for multi-threaded executor
  executor->multi_threaded = executor->allocator->allocate(
    sizeof(rclc_executor_multi_threaded_data_t), executor->allocator->state);
  if (NULL == executor->multi_threaded) {
    RCL_SET_ERROR_MSG("Could not allocate memory for 'executor->multi_threaded'.");
    return RCL_RET_BAD_ALLOC;
  }

  // initialization of mutexes and condition variables
  // note: lock not necessary, because no worker_thread has been started yet
  pthread_mutex_init(&rclc_get_multi_threaded_executor(executor)->thread_state_mutex, NULL);
  pthread_mutex_init(&rclc_get_multi_threaded_executor(executor)->micro_ros_mutex, NULL);

  rclc_micro_ros_mutex = &rclc_get_multi_threaded_executor(executor)->micro_ros_mutex;
  return RCL_RET_OK;
}

static
void rclc_executor_change_worker_thread_state(
  rclc_executor_worker_thread_param_t * p,
  rclc_executor_thread_state_t new_state)
{
  pthread_mutex_lock(p->thread_state_mutex);
  rclc_get_multi_threaded_handle(p->handle)->worker_thread_state = new_state;
  pthread_mutex_unlock(p->thread_state_mutex);
}

static
bool rclc_executor_worker_thread_is_ready(
  rclc_executor_t * executor,
  rclc_executor_handle_t * handle)
{
  bool thread_is_ready = false;
  pthread_mutex_lock(&rclc_get_multi_threaded_executor(executor)->thread_state_mutex);
  if (rclc_get_multi_threaded_handle(handle)->worker_thread_state == RCLC_THREAD_READY) {
    thread_is_ready = true;
  }
  pthread_mutex_unlock(&rclc_get_multi_threaded_executor(executor)->thread_state_mutex);
  return thread_is_ready;
}

static
rcl_ret_t rclc_executor_rebuild_wait_set(rclc_executor_t * executor)
{
  rcl_ret_t rc = RCL_RET_OK;
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "rclc_executor_rebuild_wait_set");

  // initialize wait_set if
  // (1) this is the first invocation of rclc_executor_spin_*()
  // (2) rclc_executor_add_* has been called at runtime
  if (!rcl_wait_set_is_valid(&executor->wait_set)) {
    // calling wait_set on zero_initialized wait_set multiple times is ok.
    rcl_ret_t rc = rcl_wait_set_fini(&executor->wait_set);
    if (rc != RCL_RET_OK) {
      PRINT_RCLC_ERROR(rclc_executor_rebuild_wait_set, rcl_wait_set_fini);
    }
    // initialize wait_set
    executor->wait_set = rcl_get_zero_initialized_wait_set();
    // create sufficient memory space for all handles in the wait_set
    rc = rcl_wait_set_init(
      &executor->wait_set, executor->info.number_of_subscriptions,
      executor->info.number_of_guard_conditions, executor->info.number_of_timers,
      executor->info.number_of_clients, executor->info.number_of_services,
      executor->info.number_of_events,
      executor->context, rcl_get_default_allocator());
    if (rc != RCL_RET_OK) {
      PRINT_RCLC_ERROR(rclc_executor_rebuild_wait_set, rcl_wait_set_init);
      return rc;
    }
  }

  // set rmw fields to NULL
  rc = rcl_wait_set_clear(&executor->wait_set);
  if (rc != RCL_RET_OK) {
    PRINT_RCLC_ERROR(rclc_executor_rebuild_wait_set, rcl_wait_set_clear);
    return rc;
  }

  // (JanStaschulat) put in a sub-function - for improved readability
  // only add handles, of which its worker thread is ready, to wait_set
  for (size_t i = 0; (i < executor->max_handles && executor->handles[i].initialized); i++) {
    RCUTILS_LOG_DEBUG_NAMED(
      ROS_PACKAGE_NAME, "wait_set_add_* %d",
      executor->handles[i].type);

    switch (executor->handles[i].type) {
      case RCLC_SUBSCRIPTION:
        // add subscription to wait_set and save index
        rc = rcl_wait_set_add_subscription(
          &executor->wait_set, executor->handles[i].subscription,
          &executor->handles[i].index);
        if (rc == RCL_RET_OK) {
          RCUTILS_LOG_DEBUG_NAMED(
            ROS_PACKAGE_NAME,
            "Subscription added to wait_set_subscription[%ld]",
            executor->handles[i].index);
        } else {
          PRINT_RCLC_ERROR(rclc_executor_rebuild_wait_set, rcl_wait_set_add_subscription);
          return rc;
        }
        break;

      case RCLC_TIMER:
        // add timer to wait_set and save index
        rc = rcl_wait_set_add_timer(
          &executor->wait_set, executor->handles[i].timer,
          &executor->handles[i].index);
        if (rc == RCL_RET_OK) {
          RCUTILS_LOG_DEBUG_NAMED(
            ROS_PACKAGE_NAME, "Timer added to wait_set_timers[%ld]",
            executor->handles[i].index);
        } else {
          PRINT_RCLC_ERROR(rclc_executor_rebuild_wait_set, rcl_wait_set_add_timer);
          return rc;
        }
        break;

      case RCLC_SERVICE:
        // add service to wait_set and save index
        rc = rcl_wait_set_add_service(
          &executor->wait_set, executor->handles[i].service,
          &executor->handles[i].index);
        if (rc == RCL_RET_OK) {
          RCUTILS_LOG_DEBUG_NAMED(
            ROS_PACKAGE_NAME, "Service added to wait_set_service[%ld]",
            executor->handles[i].index);
        } else {
          PRINT_RCLC_ERROR(rclc_executor_rebuild_wait_set, rcl_wait_set_add_service);
          return rc;
        }
        break;


      case RCLC_CLIENT:
        // add client to wait_set and save index
        rc = rcl_wait_set_add_client(
          &executor->wait_set, executor->handles[i].client,
          &executor->handles[i].index);
        if (rc == RCL_RET_OK) {
          RCUTILS_LOG_DEBUG_NAMED(
            ROS_PACKAGE_NAME, "Client added to wait_set_client[%ld]",
            executor->handles[i].index);
        } else {
          PRINT_RCLC_ERROR(rclc_executor_rebuild_wait_set, rcl_wait_set_add_client);
          return rc;
        }
        break;

      case RCLC_GUARD_CONDITION:
        // add guard_condition to wait_set and save index
        rc = rcl_wait_set_add_guard_condition(
          &executor->wait_set, executor->handles[i].gc,
          &executor->handles[i].index);
        if (rc == RCL_RET_OK) {
          RCUTILS_LOG_DEBUG_NAMED(
            ROS_PACKAGE_NAME, "Guard_condition added to wait_set_client[%ld]",
            executor->handles[i].index);
          printf("added guard condition to index %ld\n", executor->handles[i].index);
        } else {
          PRINT_RCLC_ERROR(rclc_executor_rebuild_wait_set, rcl_wait_set_add_guard_condition);
          return rc;
        }
        break;

      default:
        RCUTILS_LOG_DEBUG_NAMED(
          ROS_PACKAGE_NAME, "Error: unknown handle type: %d",
          executor->handles[i].type);
        PRINT_RCLC_ERROR(rclc_executor_rebuild_wait_set, rcl_wait_set_add_unknown_handle);
        return RCL_RET_ERROR;
    }
  }
  return RCL_RET_OK;
}

void * rclc_executor_worker_thread(void * p)
{
  rclc_executor_worker_thread_param_t * param = (rclc_executor_worker_thread_param_t *)p;

  // print priority
  struct sched_param sp;
  int result;
  result = sched_getparam(0, &sp);
  if (result < 0) {
    printf("uros_rbs: sched_getparam failed: %d\n", result);
    return p;
  }
  printf("worker thread with prio %d\n", sp.sched_priority);

  // Worker-thread loop
  while (1) {
    // printf("worker-thread %ld.\n",param->handle->index); // not thread-safe
    pthread_mutex_lock(&(rclc_get_multi_threaded_handle(param->handle))->new_msg_mutex);
    while (!(rclc_get_multi_threaded_handle(param->handle))->new_msg_avail) {
      // printf("worker thread: idling\n");
      pthread_cond_wait(
        &(rclc_get_multi_threaded_handle(param->handle))->new_msg_cond,
        &(rclc_get_multi_threaded_handle(param->handle))->new_msg_mutex);
    }
    (rclc_get_multi_threaded_handle(param->handle))->new_msg_avail = false;
    pthread_mutex_unlock(&(rclc_get_multi_threaded_handle(param->handle))->new_msg_mutex);

    // execute subscription callback
    // pthread_mutex_lock(param->micro_ros_mutex);
    param->handle->subscription_callback(param->handle->data);
    // pthread_mutex_unlock(param->micro_ros_mutex);

    // change_worker thread state and signal guard condition
    rclc_executor_change_worker_thread_state(param, RCLC_THREAD_READY);
  }
  // only for linters
  return p;
}

rcl_ret_t
rclc_dispatcher_executor_spin(rclc_executor_t * executor)
{
  int result;
  rcl_ret_t rc;

  // initialize mutexes and condition variables
  // TODO(JanStaschulat): this needs to be done earlier
  // rclc_dispatcher_executor_init(executor);
  // if (! executor->type OR executor->multi_threaded_initialized)
  //     return RCL_RET_ERROR;

  // print priority
  struct sched_param sp;
  result = sched_getparam(0, &sp);
  if (result < 0) {
    printf("sched_getparam failed: %d\n", result);
    return 1;
  } else {
    printf("executor thread running with prio: %d\n", sp.sched_priority);
  }

  // 'params' contains all necessary pointers to data structures which are accessed
  // from the executor thread and worker thread.
  // allocate memory for params array

  rclc_executor_worker_thread_param_t * params = NULL;
  params = executor->allocator->allocate(
    (executor->max_handles * sizeof(rclc_executor_worker_thread_param_t)),
    executor->allocator->state);
  if (NULL == params) {
    RCL_SET_ERROR_MSG("Could not allocate memory for 'params'.");
    PRINT_RCLC_ERROR(rclc_dispatcher_executor_spin, allocate);
    return RCL_RET_ERROR;
  }

  // start worker threads for subscriptions
  for (size_t i = 0;
    (i < executor->max_handles && executor->handles[i].initialized) &&
    (executor->handles[i].type == RCLC_SUBSCRIPTION);
    i++)
  {
    params[i].thread_state_mutex = &rclc_get_multi_threaded_executor(executor)->thread_state_mutex;
    params[i].micro_ros_mutex = &rclc_get_multi_threaded_executor(executor)->micro_ros_mutex;
    params[i].handle = &executor->handles[i];
    // params[i].gc = &executor->gc_some_thread_is_ready;

    printf(
      "Creating worker thread %ld scheduler: %d (1=FIFO, 3=SPORADIC).\n", i,
      rclc_get_multi_threaded_handle(&executor->handles[i])->sparam->policy);
    pthread_attr_init(&rclc_get_multi_threaded_handle(&executor->handles[i])->tattr);

    result = pthread_attr_setschedpolicy(
      &rclc_get_multi_threaded_handle(&executor->handles[i])->tattr,
      rclc_get_multi_threaded_handle(&executor->handles[i])->sparam->policy);
    if (result != 0) {
      PRINT_RCLC_ERROR(
        rclc_dispatcher_executor_spin,
        pthread_attr_setschedpolicy);
      return RCL_RET_ERROR;
    }

    result = pthread_attr_setschedparam(
      &rclc_get_multi_threaded_handle(&executor->handles[i])->tattr,
      &rclc_get_multi_threaded_handle(&executor->handles[i])->sparam->param);
    if (result != 0) {
      PRINT_RCLC_ERROR(
        rclc_dispatcher_executor_spin,
        pthread_attr_setschedparam);
      return RCL_RET_ERROR;
    }

    result = pthread_create(
      &rclc_get_multi_threaded_handle(&executor->handles[i])->worker_thread,
      &rclc_get_multi_threaded_handle(&executor->handles[i])->tattr,
      &rclc_executor_worker_thread,
      &params[i]);
    if (result != 0) {
      PRINT_RCLC_ERROR(rclc_dispatcher_executor_spin, pthread_create);
      return RCL_RET_ERROR;
    } else {
      printf(" ... started thread %ld prio %d.\n", 
        rclc_get_multi_threaded_handle(&executor->handles[i])->worker_thread,
        rclc_get_multi_threaded_handle(&executor->handles[i])->sparam->param.sched_priority);
    }
  }

  // endless spin-method
  // executor->timeout_ns = 100000000; // 100ms timeout for rcl_wait
  printf("rclc_executor: rcl_wait timeout %ld ns\n", executor->timeout_ns);
  int ii = 0;
  while (rcl_context_is_valid(executor->context) ) {
    ii++;
    rclc_executor_rebuild_wait_set(executor);
    pthread_mutex_lock(&rclc_get_multi_threaded_executor(executor)->micro_ros_mutex);
    rc = rcl_wait(&executor->wait_set, executor->timeout_ns);
    pthread_mutex_unlock(&rclc_get_multi_threaded_executor(executor)->micro_ros_mutex);
    for (size_t i = 0;
      (i < executor->max_handles && executor->handles[i].initialized);
      i++)
    {
      rc = RCL_RET_ERROR;
      if (executor->handles[i].type == RCLC_SUBSCRIPTION) {
        if (rclc_executor_worker_thread_is_ready(executor, &executor->handles[i])) {
          assert(executor->handles[i].index < executor->wait_set.size_of_subscriptions);
          if (executor->wait_set.subscriptions[executor->handles[i].index]) {
            rmw_message_info_t messageInfo;
            pthread_mutex_lock(&rclc_get_multi_threaded_executor(executor)->micro_ros_mutex);
            rc = rcl_take(
              executor->handles[i].subscription, executor->handles[i].data, &messageInfo,
              NULL);
            pthread_mutex_unlock(&rclc_get_multi_threaded_executor(executor)->micro_ros_mutex);
            if (rc != RCL_RET_OK) {
              // rcl_take might return this error even with successfull rcl_wait
              if (rc != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
                // PRINT_RCLC_ERROR(multi_threaded_executor, rcl_take);
                RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME, "Error number: %d", rc);
              }
              //  report error on RCL_RET_SUBSCRIPTION_TAKE_FAILED?
            }
          }

          // only notify thread if rcl_take was called and was successful
          // executor->handles[i] and params[i].handle point to the same handle
          if (rc == RCL_RET_OK) {
            rclc_executor_change_worker_thread_state(&params[i], RCLC_THREAD_BUSY);
            pthread_mutex_lock(&(rclc_get_multi_threaded_handle(params[i].handle))->new_msg_mutex);
            (rclc_get_multi_threaded_handle(params[i].handle))->new_msg_avail = true;
            pthread_cond_signal(&(rclc_get_multi_threaded_handle(params[i].handle))->new_msg_cond);
            pthread_mutex_unlock(
              &(rclc_get_multi_threaded_handle(params[i].handle))->new_msg_mutex);
          }
        }
      }
    }
  }
  printf("rclc_executor: exited spin-while loop\n");

  for (size_t i = 0;
    (i < executor->max_handles && executor->handles[i].initialized) &&
    (executor->handles[i].type == RCLC_SUBSCRIPTION);
    i++)
  {
    printf("Stopping worker thread %ld\n", i);
    result = 0;
    result += pthread_cancel(rclc_get_multi_threaded_handle(&executor->handles[i])->worker_thread);
    if (result != 0) {
      printf("Error stopping thread %ld\n", i);
    }
  }

  free(params);

  if (result == 0) {
    return RCL_RET_OK;
  } else {
    return RCL_RET_ERROR;
  }
}

rcl_ret_t
rclc_dispatcher_executor_fini(rclc_executor_t * executor)
{
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);

  // de-allocate multi_threaded handles objects
  for (size_t i = 0; i < executor->max_handles; i++) {
    executor->allocator->deallocate(
      executor->handles[i].multi_threaded,
      executor->allocator->state);
  }

  // de-allocate multi_threaded executor object
  executor->allocator->deallocate(executor->multi_threaded, executor->allocator->state);
  // call 'base-class' de-allocate
  rclc_executor_fini(executor);
  return RCL_RET_OK;
}
