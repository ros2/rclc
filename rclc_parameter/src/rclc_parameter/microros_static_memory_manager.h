// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

// micro-ROS static memory manager v1.0.0

#include <stdlib.h>
#include <stdbool.h>
#include <rcutils/allocator.h>

typedef struct microros_sm_item_t
{
    bool is_dynamic_memory;
    struct microros_sm_item_t* prev;
    struct microros_sm_item_t* next;
    void* data;
} microros_sm_item_t;

typedef struct microros_sm_mempool_t
{
    struct microros_sm_item_t* allocateditems;
    struct microros_sm_item_t* freeitems;
    bool is_initialized;
    bool dynamic_memory_allowed;
    size_t element_size;
    rcutils_allocator_t allocator;
} microros_sm_mempool_t;

static void microros_sm_put_memory_fn(
    microros_sm_mempool_t* mem,
    microros_sm_item_t* item)
{
    if (item->prev){
        item->prev->next = item->next;
    }
    if (item->next){
        item->next->prev = item->prev;
    }

    if (mem->allocateditems == item){
        mem->allocateditems = item->next;
    }

    if (mem->dynamic_memory_allowed && item->is_dynamic_memory)
    {
        mem->allocator.deallocate(item->data, mem->allocator.state);
        mem->allocator.deallocate(item, mem->allocator.state);
        return;
    }

    item->next = mem->freeitems;
    if (item->next){
        item->next->prev = item;
    }
    item->prev     = NULL;
    mem->freeitems = item;
}

static void* microros_sm_get_memory_fn(microros_sm_mempool_t* mem)
{
    microros_sm_item_t* item = NULL;

    if (mem->freeitems != NULL) {
        item = mem->freeitems;
        mem->freeitems = item->next;
        if (mem->freeitems){
            mem->freeitems->prev = NULL;
        }

        item->next = mem->allocateditems;
        if (item->next){
            item->next->prev = item;
        }
        item->prev = NULL;
        mem->allocateditems = item;
    }
    else if (mem->dynamic_memory_allowed)
    {
        item = (microros_sm_item_t*)mem->allocator.allocate(sizeof(microros_sm_item_t), mem->allocator.state);
        item->prev = NULL;
        item->next = NULL;
        item->data = (void*)mem->allocator.allocate(mem->element_size, mem->allocator.state);
        memset(item->data, 0, mem->element_size);
        item->is_dynamic_memory = false; // Allow to put element in free pool the first time
        microros_sm_put_memory_fn(mem, item);
        item->is_dynamic_memory = true;
        item = microros_sm_get_memory_fn(mem);
    }
    return (item == NULL) ? NULL : item->data;
}

#define microros_sm_create_memory(name, type, number, allow_dynamic) \
    typedef struct microros_sm_##name##_wrapper_t \
    {\
        microros_sm_item_t mem; \
        type data; \
    } microros_sm_##name##_wrapper_t; \
    microros_sm_mempool_t  name##_mempool = {0}; \
    microros_sm_##name##_wrapper_t  name##_buffer[number]; \
    void microros_sm_##name##_init_memory(microros_sm_mempool_t * memory, microros_sm_##name##_wrapper_t * array, size_t size) { \
        memory->dynamic_memory_allowed = allow_dynamic; \
        if (memory->dynamic_memory_allowed) \
        { \
            memory->allocator = rcutils_get_default_allocator(); \
        } \
        if (size > 0 && !memory->is_initialized) { \
            memory->is_initialized = true; \
            memory->element_size = sizeof( * array); \
            memory->allocateditems = NULL; \
            memory->freeitems = NULL; \
            for (size_t i = 0; i < size; i++) { \
                microros_sm_put_memory_fn(memory, &array[i].mem); \
                array[i].mem.data = (void * ) &array[i].data; \
                array[i].mem.is_dynamic_memory = false; \
            } \
        } \
    }

#define microros_sm_extern_include(name, type, number) \
    typedef struct microros_sm_##name##_wrapper_t \
    {\
        microros_sm_item_t mem; \
        type data; \
    } microros_sm_##name##_wrapper_t; \
    extern microros_sm_mempool_t  name##_mempool;

#define microros_sm_used_memory(name) (int)(sizeof(name##_mempool) +  sizeof(name##_buffer))

#define microros_sm_init_memory(name) \
    microros_sm_##name##_init_memory(&name##_mempool, name##_buffer, sizeof(name##_buffer)/sizeof(microros_sm_##name##_wrapper_t));

#define microros_sm_get_memory(name) \
    (void*) microros_sm_get_memory_fn(&name##_mempool);

#define microros_sm_put_memory(name, ptr) \
    { \
    microros_sm_item_t * item = (microros_sm_item_t *)((uint8_t*)ptr) - offsetof(microros_sm_##name##_wrapper_t, data); \
    microros_sm_put_memory_fn(&name##_mempool, item); \
    }

#define microros_sm_has_memory(name) (name##_mempool.freeitems != NULL)
#define microros_sm_is_init(name) (name##_mempool.is_initialized)
