#include "rclc/rcl_foxy.h"
bool
rcl_wait_set_is_valid(const rcl_wait_set_t * wait_set) {
  return wait_set && wait_set->impl;
}