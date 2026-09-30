// Copyright (c) 2019 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/ros2/rclc.
// Copyright 2014 Open Source Robotics Foundation, Inc.
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

// Keep nanosleep() and usleep() declared with strict -std=c99 / -std=c11
#if !defined(WIN32) && !defined(_DEFAULT_SOURCE)
#define _DEFAULT_SOURCE
#endif

#include "rclc/sleep.h"

#ifdef WIN32
#include <Windows.h>
#else
#include <time.h>
#include <unistd.h>
#endif

void
rclc_sleep_ms(
  unsigned int ms)
{
#if defined(WIN32)
  Sleep(ms);
#elif defined(_POSIX_TIMERS) && (_POSIX_TIMERS > 0)
  struct timespec ts;
  ts.tv_sec = ms / 1000;
  ts.tv_nsec = (ms % 1000) * 1000000;
  nanosleep(&ts, NULL);
#else
  usleep(ms * 1000);
#endif
}
