// Copyright (c) 2020 - for information on the respective copyright owner
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

#include "rclc/multi_threaded_executor.h"

/*
Features
- multi-threaded scheduling for embedded ROS 2 applications with scheduling parameter assignment
  (example: thread priority, NuttX: sporadic server(budget, period))

Limitations:
- only subscription (no timer, services, clients, gc)
- no trigger
- no LET semantics
- NuttX: sporadic server: one thread for each subscription
(reservation based scheduling: one thread for multiple subscriptions - later)
*/

static pthread_mutex_t * rclc_micro_ros_mutex;

void rclc_gc_callback(void)
{
  printf("guard condition called.\n");
}

rcl_ret_t rclc_executor_publish(
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

rcl_ret_t
rclc_executor_add_subscription_multi_threaded(
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
  executor->handles[executor->index].sparam = sparam;
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

void rclc_executor_init_multi_threaded(rclc_executor_t * e)
{
  // initialization of mutexes and condition variables
  // init worker_thread_state = READY
  // lock not necessary, because no worker_thread has been started yet
  pthread_mutex_init(&e->thread_state_mutex, NULL);
  pthread_mutex_init(&e->micro_ros_mutex, NULL);
  for (size_t i = 0; (i < e->max_handles && e->handles[i].initialized); i++) {
    e->handles[i].worker_thread_state = RCLC_THREAD_READY;
    pthread_mutex_init(&e->handles[i].new_msg_mutex, NULL);
    pthread_cond_init(&e->handles[i].new_msg_cond, NULL);
  }
  rclc_micro_ros_mutex = &e->micro_ros_mutex;

/*
  // initialization of guard condition
  printf("initialization guard condition\n");
  rcl_ret_t rc;
  e->gc_some_thread_is_ready = rcl_get_zero_initialized_guard_condition();
  rcl_guard_condition_options_t guard_options = rcl_guard_condition_get_default_options();
  rc = rcl_guard_condition_init(&e->gc_some_thread_is_ready, e->context, guard_options);
  if (rc != RCL_RET_OK) {
    RCL_SET_ERROR_MSG("Could not create gc_thread_is_ready guard");
    PRINT_RCLC_ERROR(rclc_executor_init_multi_threaded, rcl_guard_condition_init);
  }
  // add guard condition to executor
  // todo add max_handles + 1  for guard condition!
  rc = rclc_executor_add_guard_condition(e, &e->gc_some_thread_is_ready, rclc_gc_callback);
  if (rc != RCL_RET_OK) {
    RCL_SET_ERROR_MSG("Could not add guard condition to executor");
    PRINT_RCLC_ERROR(rclc_executor_init_multi_threaded, rclc_executor_add_guard_condition);
  }
  */
}

static
void rclc_executor_change_worker_thread_state(
  rclc_executor_worker_thread_param_t * p,
  rclc_executor_thread_state_t new_state)
{
  /*
  printf("change thread %ld to state ", p->handle->index);
  if (new_state == RCLC_THREAD_READY) {
    printf("READY\n");
  } else if (new_state == RCLC_THREAD_BUSY) {
    printf("BUSY\n");
  }
  */
  pthread_mutex_lock(p->thread_state_mutex);
  p->handle->worker_thread_state = new_state;
  /*
  if (new_state == RCLC_THREAD_READY) {
    rcl_ret_t rc = rcl_trigger_guard_condition(p->gc);
    if (rc != RCL_RET_OK) {
      printf("Error triggering guard condition.\n");
    }
  }
  */
  pthread_mutex_unlock(p->thread_state_mutex);
}

static
bool rclc_executor_worker_thread_is_ready(rclc_executor_t * e, rclc_executor_handle_t * handle)
{
  bool thread_is_ready = false;
  pthread_mutex_lock(&e->thread_state_mutex);
  // assume that the thread state is initialized with RCLC_THREAD_READY
  // and is not RCLC_THREAD_NONE
  if (handle->worker_thread_state == RCLC_THREAD_READY) {
    thread_is_ready = true;
  }
  pthread_mutex_unlock(&e->thread_state_mutex);
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

  // (jst3si) put in a sub-function - for improved readability
  // only add handles, of which its worker thread is ready, to wait_set
  for (size_t i = 0; (i < executor->max_handles && executor->handles[i].initialized); i++) {
    RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "wait_set_add_* %d", executor->handles[i].type);

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
          // printf(
          //  "added subscription to wait_set.subscriptions[%ld]\n",
          //  executor->handles[i].index);
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
    // todo (jast) is * void necessary?
    return p;
  }
  printf("worker thread with prio %d\n", sp.sched_priority);

  // endless worker_thread loop
  while (1) {
    // printf("worker-thread %ld.\n",param->handle->index); // not thread-safe
    pthread_mutex_lock(&param->handle->new_msg_mutex);
    while (!param->handle->new_msg_avail) {
      // printf("worker thread: idling\n");
      pthread_cond_wait(&param->handle->new_msg_cond, &param->handle->new_msg_mutex);
    }
    param->handle->new_msg_avail = false;
    pthread_mutex_unlock(&param->handle->new_msg_mutex);

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
rclc_executor_spin_multi_threaded(rclc_executor_t * e)
{
  int result;
  rcl_ret_t rc;

  // initialize mutexes and condition variables
  rclc_executor_init_multi_threaded(e);

  // print priority
  struct sched_param sp;
  result = sched_getparam(0, &sp);
  if (result < 0) {
    printf("uros_rbs: sched_getparam failed: %d\n", result);
    return 1;
  } else {
    printf("executor thread running with prio: %d\n", sp.sched_priority);
  }

  // 'params' contains all necessary pointers to data structures which are accessed
  // from the executor thread and worker thread.
  // allocate memory for params array

  rclc_executor_worker_thread_param_t * params = NULL;
  params = e->allocator->allocate(
    (e->max_handles * sizeof(rclc_executor_worker_thread_param_t)),
    e->allocator->state);
  if (NULL == params) {
    RCL_SET_ERROR_MSG("Could not allocate memory for 'params'.");
    PRINT_RCLC_ERROR(rclc_executor_spin_multi_threaded, allocate);
    return RCL_RET_ERROR;
  }

  // start worker threads for subscriptions
  for (size_t i = 0;
    (i < e->max_handles && e->handles[i].initialized) && (e->handles[i].type == RCLC_SUBSCRIPTION);
    i++)
  {
    params[i].thread_state_mutex = &e->thread_state_mutex;
    params[i].micro_ros_mutex = &e->micro_ros_mutex;
    params[i].handle = &e->handles[i];
    // params[i].gc = &e->gc_some_thread_is_ready;

    printf(
      "Creating worker thread %ld scheduler: %d (1=FIFO, 3=SPORADIC).\n", i,
      e->handles[i].sparam->policy);
    pthread_attr_init(&e->handles[i].tattr);

    result = pthread_attr_setschedpolicy(&e->handles[i].tattr, e->handles[i].sparam->policy);
    if (result != 0) {
      PRINT_RCLC_ERROR(
        rclc_executor_spin_multi_threaded,
        pthread_attr_setschedpolicy);
      return RCL_RET_ERROR;
    }

    result = pthread_attr_setschedparam(&e->handles[i].tattr, &e->handles[i].sparam->param);
    if (result != 0) {
      PRINT_RCLC_ERROR(
        rclc_executor_spin_multi_threaded,
        pthread_attr_setschedparam);
      return RCL_RET_ERROR;
    }

    /*
    result = pthread_attr_setstacksize(&e->handles[i].tattr, 2048); // STM32-E407: 196 kB RAM
    if (result != 0) {
      PRINT_RCLC_ERROR(rclc_executor_spin_multi_threaded,
                       pthread_attr_setstacksize);
      return RCL_RET_ERROR;
    }
    */

    result = pthread_create(
      &e->handles[i].worker_thread, &e->handles[i].tattr, &rclc_executor_worker_thread,
      &params[i]);
    if (result != 0) {
      PRINT_RCLC_ERROR(rclc_executor_spin_multi_threaded, pthread_create);
      return RCL_RET_ERROR;
    } else {
      printf(" ... started.\n");
    }
  }

  // endless spin-method
  // e->timeout_ns = 100000000; // 100ms timeout for rcl_wait
  printf("rclc_executor: rcl_wait timeout %ld ns\n", e->timeout_ns);
  int ii = 0;
  while (rcl_context_is_valid(e->context) ) {
    ii++;
    // printf("round %d\n", ii);

    // update wait_set only for subscriptions with a READY worker thread
    rclc_executor_rebuild_wait_set(e);

    // wait for new messages
    pthread_mutex_lock(&e->micro_ros_mutex);
    rc = rcl_wait(&e->wait_set, e->timeout_ns);
    pthread_mutex_unlock(&e->micro_ros_mutex);

    // take new messages (only if its corresponding worker thread is READY)
    for (size_t i = 0;
      (i < e->max_handles && e->handles[i].initialized);
      i++)
    {
      rc = RCL_RET_ERROR;
      if (e->handles[i].type == RCLC_SUBSCRIPTION) {
        if (rclc_executor_worker_thread_is_ready(e, &e->handles[i])) {
          // printf("accessing wait_set.subscriptions[%ld] size: %ld\n",
          //   e->handles[i].index, e->wait_set.size_of_subscriptions);

          assert(e->handles[i].index < e->wait_set.size_of_subscriptions);
          if (e->wait_set.subscriptions[e->handles[i].index]) {
            rmw_message_info_t messageInfo;
            pthread_mutex_lock(&e->micro_ros_mutex);
            rc = rcl_take(
              e->handles[i].subscription, e->handles[i].data, &messageInfo,
              NULL);
            pthread_mutex_unlock(&e->micro_ros_mutex);
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
          // e->handles[i] and params[i].handle point to the same handle
          if (rc == RCL_RET_OK) {
            rclc_executor_change_worker_thread_state(&params[i], RCLC_THREAD_BUSY);
            pthread_mutex_lock(&params[i].handle->new_msg_mutex);
            params[i].handle->new_msg_avail = true;
            // printf(
            //  "signaling worker thread %ld: handles[i].index=%ld \n", i, e->handles[i].index);
            pthread_cond_signal(&params[i].handle->new_msg_cond);
            pthread_mutex_unlock(&params[i].handle->new_msg_mutex);
          }
        }
      }
    }
  }
  printf("rclc_executor: exited spin-while loop\n");

  for (size_t i = 0;
    (i < e->max_handles && e->handles[i].initialized) && (e->handles[i].type == RCLC_SUBSCRIPTION);
    i++)
  {
    printf("Stopping worker thread %ld\n", i);
    result = 0;
    result += pthread_cancel(e->handles[i].worker_thread);
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

/*

// real-time executor feature:
// - assignment of priority and budget of NuttX threads
// - dispatching messages to NuttX-threads based on priority and not based how the wait_set was created
// - reacting as-fast as possible to new message (when thread becomes ready - rcl_wait is interrupted by gc) and wait_set is re-created
// when user adds subscription X with priority i (X, i);
(A,1)
(B,5)
(C,3)

reorder them according to their priority:
handle[] =( (B,5) (C,3) (A,1))

*/
