/*
 * Copyright © 2015 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#ifndef BCMV_PRIVATE_H
#define BCMV_PRIVATE_H

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <pthread.h>
#include <assert.h>
#include <stdint.h>
#include <vc5_drm.h>

#ifdef HAVE_VALGRIND
#include <valgrind.h>
#include <memcheck.h>
#define VG(x) x
#define __gen_validate_value(x) VALGRIND_CHECK_MEM_IS_DEFINED(&(x), sizeof(x))
#else
#define VG(x)
#endif

#include "common/v3d_device_info.h"
#include "compiler/v3d_compiler.h"
#include "util/macros.h"
#include "util/list.h"
#include "util/u_atomic.h"
#include "util/u_vector.h"
#include "vk_alloc.h"

/* Pre-declarations needed for WSI entrypoints */
struct wl_surface;
struct wl_display;
typedef struct xcb_connection_t xcb_connection_t;
typedef uint32_t xcb_visualid_t;
typedef uint32_t xcb_window_t;
struct vc5_compiled_shader;

struct bcmv_buffer;
struct bcmv_buffer_view;
struct bcmv_image_view;

struct gen_l3_config;

#include <vulkan/vulkan.h>
#include <vulkan/vulkan_intel.h>
#include <vulkan/vk_icd.h>

#include "bcmv_entrypoints.h"

#include "common/v3d_debug.h"
#include "wsi_common.h"

/* Allowing different clear colors requires us to perform a depth resolve at
 * the end of certain render passes. This is because while slow clears store
 * the clear color in the HiZ buffer, fast clears (without a resolve) don't.
 * See the PRMs for examples describing when additional resolves would be
 * necessary. To enable fast clears without requiring extra resolves, we set
 * the clear value to a globally-defined one. We could allow different values
 * if the user doesn't expect coherent data during or after a render passes
 * (VK_ATTACHMENT_STORE_OP_DONT_CARE), but such users (aside from the CTS)
 * don't seem to exist yet. In almost all Vulkan applications tested thus far,
 * 1.0f seems to be the only value used. The only application that doesn't set
 * this value does so through the usage of an seemingly uninitialized clear
 * value.
 */
#define BCMV_HZ_FC_VAL 1.0f

#define MAX_VBS         28
#define MAX_SETS         8
#define MAX_RTS          8
#define MAX_VIEWPORTS   16
#define MAX_SCISSORS    16
#define MAX_PUSH_CONSTANTS_SIZE 128
#define MAX_DYNAMIC_BUFFERS 16
#define MAX_IMAGES 8
#define MAX_PUSH_DESCRIPTORS 32 /* Minimum requirement */

#define BCMV_SVGS_VB_INDEX    MAX_VBS
#define BCMV_DRAWID_VB_INDEX (MAX_VBS + 1)

#define bcmv_printflike(a, b) __attribute__((__format__(__printf__, a, b)))

static inline uint32_t
align_down_npot_u32(uint32_t v, uint32_t a)
{
        return v - (v % a);
}

static inline uint32_t
align_u32(uint32_t v, uint32_t a)
{
        assert(a != 0 && a == (a & -a));
        return (v + a - 1) & ~(a - 1);
}

static inline uint64_t
align_u64(uint64_t v, uint64_t a)
{
        assert(a != 0 && a == (a & -a));
        return (v + a - 1) & ~(a - 1);
}

static inline int32_t
align_i32(int32_t v, int32_t a)
{
        assert(a != 0 && a == (a & -a));
        return (v + a - 1) & ~(a - 1);
}

/** Alignment must be a power of 2. */
static inline bool
bcmv_is_aligned(uintmax_t n, uintmax_t a)
{
        assert(a == (a & -a));
        return (n & (a - 1)) == 0;
}

static inline uint32_t
bcmv_minify(uint32_t n, uint32_t levels)
{
        if (unlikely(n == 0))
                return 0;
        else
                return MAX2(n >> levels, 1);
}

static inline float
bcmv_clamp_f(float f, float min, float max)
{
        assert(min < max);

        if (f > max)
                return max;
        else if (f < min)
                return min;
        else
                return f;
}

static inline bool
bcmv_clear_mask(uint32_t *inout_mask, uint32_t clear_mask)
{
        if (*inout_mask & clear_mask) {
                *inout_mask &= ~clear_mask;
                return true;
        } else {
                return false;
        }
}

#if 0 /* XXX */
static inline union isl_color_value
vk_to_isl_color(VkClearColorValue color)
{
        return (union isl_color_value) {
                .u32 = {
                        color.uint32[0],
                        color.uint32[1],
                        color.uint32[2],
                        color.uint32[3],
                },
                        };
}
#endif

#define for_each_bit(b, dword)                          \
        for (uint32_t __dword = (dword);                \
             (b) = __builtin_ffs(__dword) - 1, __dword; \
             __dword &= ~(1 << (b)))

#define typed_memcpy(dest, src, count) ({                               \
                        STATIC_ASSERT(sizeof(*src) == sizeof(*dest));   \
                        memcpy((dest), (src), (count) * sizeof(*(src))); \
                })

/* Whenever we generate an error, pass it through this function. Useful for
 * debugging, where we can break on it. Only call at error site, not when
 * propagating errors. Might be useful to plug in a stack trace here.
 */

VkResult __vk_errorf(VkResult error, const char *file, int line, const char *format, ...);

#ifdef DEBUG
#define vk_error(error) __vk_errorf(error, __FILE__, __LINE__, NULL);
#define vk_errorf(error, format, ...) __vk_errorf(error, __FILE__, __LINE__, format, ## __VA_ARGS__);
#define bcmv_debug(format, ...) fprintf(stderr, "debug: " format, ##__VA_ARGS__)
#else
#define vk_error(error) error
#define vk_errorf(error, format, ...) error
#define bcmv_debug(format, ...)
#endif

/**
 * Warn on ignored extension structs.
 *
 * The Vulkan spec requires us to ignore unsupported or unknown structs in
 * a pNext chain.  In debug mode, emitting warnings for ignored structs may
 * help us discover structs that we should not have ignored.
 *
 *
 * From the Vulkan 1.0.38 spec:
 *
 *    Any component of the implementation (the loader, any enabled layers,
 *    and drivers) must skip over, without processing (other than reading the
 *    sType and pNext members) any chained structures with sType values not
 *    defined by extensions supported by that component.
 */
#define bcmv_debug_ignored_stype(sType)                                 \
        bcmv_debug("debug: %s: ignored VkStructureType %u\n", __func__, (sType))

void __bcmv_finishme(const char *file, int line, const char *format, ...)
        bcmv_printflike(3, 4);
void __bcmv_perf_warn(const char *file, int line, const char *format, ...)
        bcmv_printflike(3, 4);
void bcmv_loge(const char *format, ...) bcmv_printflike(1, 2);
void bcmv_loge_v(const char *format, va_list va);

/**
 * Print a FINISHME message, including its source location.
 */
#define bcmv_finishme(format, ...)                                      \
        do {                                                            \
                static bool reported = false;                           \
                if (!reported) {                                        \
                        __bcmv_finishme(__FILE__, __LINE__, format, ##__VA_ARGS__); \
                        reported = true;                                \
                }                                                       \
        } while (0)

/**
 * Print a perf warning message.  Set INTEL_DEBUG=perf to see these.
 */
#define bcmv_perf_warn(format, ...)                                     \
        do {                                                            \
                static bool reported = false;                           \
                if (!reported && unlikely(V3D_DEBUG & V3D_DEBUG_PERF)) {  \
                        __bcmv_perf_warn(__FILE__, __LINE__, format, ##__VA_ARGS__); \
                        reported = true;                                \
                }                                                       \
        } while (0)

/* A non-fatal assert.  Useful for debugging. */
#ifdef DEBUG
#define bcmv_assert(x) ({                                               \
                        if (unlikely(!(x)))                             \
                                fprintf(stderr, "%s:%d ASSERT: %s\n", __FILE__, __LINE__, #x); \
                })
#else
#define bcmv_assert(x)
#endif

/* A multi-pointer allocator
 *
 * When copying data structures from the user (such as a render pass), it's
 * common to need to allocate data for a bunch of different things.  Instead
 * of doing several allocations and having to handle all of the error checking
 * that entails, it can be easier to do a single allocation.  This struct
 * helps facilitate that.  The intended usage looks like this:
 *
 *    BCMV_MULTIALLOC(ma)
 *    bcmv_multialloc_add(&ma, &main_ptr, 1);
 *    bcmv_multialloc_add(&ma, &substruct1, substruct1Count);
 *    bcmv_multialloc_add(&ma, &substruct2, substruct2Count);
 *
 *    if (!bcmv_multialloc_alloc(&ma, pAllocator, VK_ALLOCATION_SCOPE_FOO))
 *       return vk_error(VK_ERROR_OUT_OF_HOST_MEORY);
 */
struct bcmv_multialloc {
        size_t size;
        size_t align;

        uint32_t ptr_count;
        void **ptrs[8];
};

#define BCMV_MULTIALLOC_INIT                    \
        ((struct bcmv_multialloc) { 0, })

#define BCMV_MULTIALLOC(_name)                                  \
        struct bcmv_multialloc _name = BCMV_MULTIALLOC_INIT

__attribute__((always_inline))
static inline void
_bcmv_multialloc_add(struct bcmv_multialloc *ma,
                     void **ptr, size_t size, size_t align)
{
        size_t offset = align_u64(ma->size, align);
        ma->size = offset + size;
        ma->align = MAX2(ma->align, align);

        /* Store the offset in the pointer. */
        *ptr = (void *)(uintptr_t)offset;

        assert(ma->ptr_count < ARRAY_SIZE(ma->ptrs));
        ma->ptrs[ma->ptr_count++] = ptr;
}

#define bcmv_multialloc_add(_ma, _ptr, _count)                          \
        _bcmv_multialloc_add((_ma), (void **)(_ptr),                    \
                             (_count) * sizeof(**(_ptr)), __alignof__(**(_ptr)))

__attribute__((always_inline))
static inline void *
bcmv_multialloc_alloc(struct bcmv_multialloc *ma,
                      const VkAllocationCallbacks *alloc,
                      VkSystemAllocationScope scope)
{
        void *ptr = vk_alloc(alloc, ma->size, ma->align, scope);
        if (!ptr)
                return NULL;

        /* Fill out each of the pointers with their final value.
         *
         *   for (uint32_t i = 0; i < ma->ptr_count; i++)
         *      *ma->ptrs[i] = ptr + (uintptr_t)*ma->ptrs[i];
         *
         * Unfortunately, even though ma->ptr_count is basically guaranteed to be a
         * constant, GCC is incapable of figuring this out and unrolling the loop
         * so we have to give it a little help.
         */
        STATIC_ASSERT(ARRAY_SIZE(ma->ptrs) == 8);
#define _BCMV_MULTIALLOC_UPDATE_POINTER(_i)                     \
        if ((_i) < ma->ptr_count)                               \
                *ma->ptrs[_i] = ptr + (uintptr_t)*ma->ptrs[_i]
        _BCMV_MULTIALLOC_UPDATE_POINTER(0);
        _BCMV_MULTIALLOC_UPDATE_POINTER(1);
        _BCMV_MULTIALLOC_UPDATE_POINTER(2);
        _BCMV_MULTIALLOC_UPDATE_POINTER(3);
        _BCMV_MULTIALLOC_UPDATE_POINTER(4);
        _BCMV_MULTIALLOC_UPDATE_POINTER(5);
        _BCMV_MULTIALLOC_UPDATE_POINTER(6);
        _BCMV_MULTIALLOC_UPDATE_POINTER(7);
#undef _BCMV_MULTIALLOC_UPDATE_POINTER

        return ptr;
}

__attribute__((always_inline))
static inline void *
bcmv_multialloc_alloc2(struct bcmv_multialloc *ma,
                       const VkAllocationCallbacks *parent_alloc,
                       const VkAllocationCallbacks *alloc,
                       VkSystemAllocationScope scope)
{
        return bcmv_multialloc_alloc(ma, alloc ? alloc : parent_alloc, scope);
}

struct bcmv_bo {
        uint32_t gem_handle;

        /* Index into the current validation list.  This is used by the
         * validation list building alrogithm to track which buffers are already
         * in the validation list so that we can ensure uniqueness.
         */
        uint32_t index;

        /* The offset of the BO in hte V3D MMU. */
        uint64_t offset;

        uint64_t size;
        void *map;

        /** Flags to pass to the kernel through drm_vc5_submit_cl::flags */
        uint32_t flags;
};

static inline void
bcmv_bo_init(struct bcmv_bo *bo, uint32_t gem_handle, uint64_t size)
{
        bo->gem_handle = gem_handle;
        bo->index = 0;
        bo->offset = -1;
        bo->size = size;
        bo->map = NULL;
        bo->flags = 0;
}

/* Represents a lock-free linked list of "free" things.  This is used by
 * both the block pool and the state pools.  Unfortunately, in order to
 * solve the ABA problem, we can't use a single uint32_t head.
 */
union bcmv_free_list {
        struct {
                int32_t offset;

                /* A simple count that is incremented every time the head changes. */
                uint32_t count;
        };
        uint64_t u64;
};

#define BCMV_FREE_LIST_EMPTY ((union bcmv_free_list) { { 1, 0 } })

struct bcmv_block_state {
        union {
                struct {
                        uint32_t next;
                        uint32_t end;
                };
                uint64_t u64;
        };
};

struct bcmv_block_pool {
        struct bcmv_device *device;

        struct bcmv_bo bo;

        /* The offset from the start of the bo to the "center" of the block
         * pool.  Pointers to allocated blocks are given by
         * bo.map + center_bo_offset + offsets.
         */
        uint32_t center_bo_offset;

        /* Current memory map of the block pool.  This pointer may or may not
         * point to the actual beginning of the block pool memory.  If
         * bcmv_block_pool_alloc_back has ever been called, then this pointer
         * will point to the "center" position of the buffer and all offsets
         * (negative or positive) given out by the block pool alloc functions
         * will be valid relative to this pointer.
         *
         * In particular, map == bo.map + center_offset
         */
        void *map;
        int fd;

        /**
         * Array of mmaps and gem handles owned by the block pool, reclaimed when
         * the block pool is destroyed.
         */
        struct u_vector mmap_cleanups;

        struct bcmv_block_state state;

        struct bcmv_block_state back_state;
};

/* Block pools are backed by a fixed-size 1GB memfd */
#define BLOCK_POOL_MEMFD_SIZE (1ul << 30)

/* The center of the block pool is also the middle of the memfd.  This may
 * change in the future if we decide differently for some reason.
 */
#define BLOCK_POOL_MEMFD_CENTER (BLOCK_POOL_MEMFD_SIZE / 2)

static inline uint32_t
bcmv_block_pool_size(struct bcmv_block_pool *pool)
{
        return pool->state.end + pool->back_state.end;
}

struct bcmv_state {
        int32_t offset;
        uint32_t alloc_size;
        void *map;
};

#define BCMV_STATE_NULL ((struct bcmv_state) { .alloc_size = 0 })

struct bcmv_fixed_size_state_pool {
        union bcmv_free_list free_list;
        struct bcmv_block_state block;
};

#define BCMV_MIN_STATE_SIZE_LOG2 6
#define BCMV_MAX_STATE_SIZE_LOG2 20

#define BCMV_STATE_BUCKETS (BCMV_MAX_STATE_SIZE_LOG2 - BCMV_MIN_STATE_SIZE_LOG2 + 1)

struct bcmv_state_pool {
        struct bcmv_block_pool block_pool;

        /* The size of blocks which will be allocated from the block pool */
        uint32_t block_size;

        /** Free list for "back" allocations */
        union bcmv_free_list back_alloc_free_list;

        struct bcmv_fixed_size_state_pool buckets[BCMV_STATE_BUCKETS];
};

struct bcmv_state_stream_block;

struct bcmv_state_stream {
        struct bcmv_state_pool *state_pool;

        /* The size of blocks to allocate from the state pool */
        uint32_t block_size;

        /* Current block we're allocating from */
        struct bcmv_state block;

        /* Offset into the current block at which to allocate the next state */
        uint32_t next;

        /* List of all blocks allocated from this pool */
        struct bcmv_state_stream_block *block_list;
};

/* The block_pool functions exported for testing only.  The block pool should
 * only be used via a state pool (see below).
 */
VkResult bcmv_block_pool_init(struct bcmv_block_pool *pool,
                              struct bcmv_device *device,
                              uint32_t initial_size);
void bcmv_block_pool_finish(struct bcmv_block_pool *pool);
int32_t bcmv_block_pool_alloc(struct bcmv_block_pool *pool,
                              uint32_t block_size);
int32_t bcmv_block_pool_alloc_back(struct bcmv_block_pool *pool,
                                   uint32_t block_size);

VkResult bcmv_state_pool_init(struct bcmv_state_pool *pool,
                              struct bcmv_device *device,
                              uint32_t block_size);
void bcmv_state_pool_finish(struct bcmv_state_pool *pool);
struct bcmv_state bcmv_state_pool_alloc(struct bcmv_state_pool *pool,
                                        uint32_t state_size, uint32_t alignment);
struct bcmv_state bcmv_state_pool_alloc_back(struct bcmv_state_pool *pool);
void bcmv_state_pool_free(struct bcmv_state_pool *pool, struct bcmv_state state);
void bcmv_state_stream_init(struct bcmv_state_stream *stream,
                            struct bcmv_state_pool *state_pool,
                            uint32_t block_size);
void bcmv_state_stream_finish(struct bcmv_state_stream *stream);
struct bcmv_state bcmv_state_stream_alloc(struct bcmv_state_stream *stream,
                                          uint32_t size, uint32_t alignment);

/**
 * Implements a pool of re-usable BOs.  The interface is identical to that
 * of block_pool except that each block is its own BO.
 */
struct bcmv_bo_pool {
        struct bcmv_device *device;

        void *free_list[16];
};

void bcmv_bo_pool_init(struct bcmv_bo_pool *pool, struct bcmv_device *device);
void bcmv_bo_pool_finish(struct bcmv_bo_pool *pool);
VkResult bcmv_bo_pool_alloc(struct bcmv_bo_pool *pool, struct bcmv_bo *bo,
                            uint32_t size);
void bcmv_bo_pool_free(struct bcmv_bo_pool *pool, const struct bcmv_bo *bo);

struct bcmv_scratch_bo {
        bool exists;
        struct bcmv_bo bo;
};

struct bcmv_scratch_pool {
        /* Indexed by Per-Thread Scratch Space number (the hardware value) and stage */
        struct bcmv_scratch_bo bos[16][MESA_SHADER_STAGES];
};

void bcmv_scratch_pool_init(struct bcmv_device *device,
                            struct bcmv_scratch_pool *pool);
void bcmv_scratch_pool_finish(struct bcmv_device *device,
                              struct bcmv_scratch_pool *pool);
struct bcmv_bo *bcmv_scratch_pool_alloc(struct bcmv_device *device,
                                        struct bcmv_scratch_pool *pool,
                                        gl_shader_stage stage,
                                        unsigned per_thread_scratch);

/** Implements a BO cache that ensures a 1-1 mapping of GEM BOs to bcmv_bos */
struct bcmv_bo_cache {
        struct hash_table *bo_map;
        pthread_mutex_t mutex;
};

VkResult bcmv_bo_cache_init(struct bcmv_bo_cache *cache);
void bcmv_bo_cache_finish(struct bcmv_bo_cache *cache);
VkResult bcmv_bo_cache_alloc(struct bcmv_device *device,
                             struct bcmv_bo_cache *cache,
                             uint64_t size, struct bcmv_bo **bo);
VkResult bcmv_bo_cache_import(struct bcmv_device *device,
                              struct bcmv_bo_cache *cache,
                              int fd, uint64_t size, struct bcmv_bo **bo);
VkResult bcmv_bo_cache_export(struct bcmv_device *device,
                              struct bcmv_bo_cache *cache,
                              struct bcmv_bo *bo_in, int *fd_out);
void bcmv_bo_cache_release(struct bcmv_device *device,
                           struct bcmv_bo_cache *cache,
                           struct bcmv_bo *bo);

struct bcmv_memory_type {
        /* Standard bits passed on to the client */
        VkMemoryPropertyFlags   propertyFlags;
        uint32_t                heapIndex;

        /* Driver-internal book-keeping */
        VkBufferUsageFlags      valid_buffer_usage;
};

struct bcmv_memory_heap {
        /* Standard bits passed on to the client */
        VkDeviceSize      size;
        VkMemoryHeapFlags flags;

        /* Driver-internal book-keeping */
        bool              supports_48bit_addresses;
};

struct bcmv_physical_device {
        VK_LOADER_DATA                              _loader_data;

        struct bcmv_instance *                       instance;
        uint32_t                                    chipset_id;
        char                                        path[20];
        const char *                                name;
        struct v3d_device_info                      info;
        struct v3d_compiler *                       compiler;
        int                                         cmd_parser_version;
        bool                                        has_exec_async;
        bool                                        has_exec_fence;

        uint32_t                                    eu_total;
        uint32_t                                    subslice_total;

        struct {
                uint32_t                                  type_count;
                struct bcmv_memory_type                    types[VK_MAX_MEMORY_TYPES];
                uint32_t                                  heap_count;
                struct bcmv_memory_heap                    heaps[VK_MAX_MEMORY_HEAPS];
        } memory;

        uint8_t                                     pipeline_cache_uuid[VK_UUID_SIZE];
        uint8_t                                     driver_uuid[VK_UUID_SIZE];
        uint8_t                                     device_uuid[VK_UUID_SIZE];

        struct wsi_device                       wsi_device;
        int                                         local_fd;
};

struct bcmv_instance {
        VK_LOADER_DATA                              _loader_data;

        VkAllocationCallbacks                       alloc;

        uint32_t                                    apiVersion;
        int                                         physicalDeviceCount;
        struct bcmv_physical_device                  physicalDevice;
};

VkResult bcmv_init_wsi(struct bcmv_physical_device *physical_device);
void bcmv_finish_wsi(struct bcmv_physical_device *physical_device);

bool bcmv_instance_extension_supported(const char *name);
uint32_t bcmv_physical_device_api_version(struct bcmv_physical_device *dev);
bool bcmv_physical_device_extension_supported(struct bcmv_physical_device *dev,
                                              const char *name);

struct bcmv_queue {
        VK_LOADER_DATA                              _loader_data;

        struct bcmv_device *                         device;

        struct bcmv_state_pool *                     pool;
};

struct bcmv_pipeline_cache {
        struct bcmv_device *                          device;
        pthread_mutex_t                              mutex;

        struct hash_table *                          cache;
};

struct bcmv_pipeline_bind_map;

void bcmv_pipeline_cache_init(struct bcmv_pipeline_cache *cache,
                              struct bcmv_device *device,
                              bool cache_enabled);
void bcmv_pipeline_cache_finish(struct bcmv_pipeline_cache *cache);

struct bcmv_shader_bin *
bcmv_pipeline_cache_search(struct bcmv_pipeline_cache *cache,
                           const void *key, uint32_t key_size);
struct bcmv_shader_bin *
bcmv_pipeline_cache_upload_kernel(struct bcmv_pipeline_cache *cache,
                                  const void *key_data, uint32_t key_size,
                                  const void *kernel_data, uint32_t kernel_size,
                                  const struct v3d_prog_data *prog_data,
                                  uint32_t prog_data_size,
                                  enum quniform_contents *uniform_contents,
                                  uint32_t *uniform_data,
                                  const struct bcmv_pipeline_bind_map *bind_map);

struct bcmv_device {
        VK_LOADER_DATA                              _loader_data;

        VkAllocationCallbacks                       alloc;

        struct bcmv_instance *                       instance;
        uint32_t                                    chipset_id;
        struct v3d_device_info                      info;
        int                                         context_id;
        int                                         fd;

        struct bcmv_bo_pool                          batch_bo_pool;

        struct bcmv_bo_cache                         bo_cache;

        struct bcmv_state_pool                       dynamic_state_pool;
        struct bcmv_state_pool                       instruction_state_pool;
        struct bcmv_state_pool                       surface_state_pool;

        struct bcmv_bo                               workaround_bo;
        struct bcmv_bo                               trivial_batch_bo;

        struct bcmv_pipeline_cache                   blorp_shader_cache;

        struct bcmv_state                            border_colors;

        struct bcmv_queue                            queue;

        struct bcmv_scratch_pool                     scratch_pool;

        uint32_t                                    default_mocs;

        pthread_mutex_t                             mutex;
        pthread_cond_t                              queue_submit;
        bool                                        lost;
};

static void inline
bcmv_state_flush(struct bcmv_device *device, struct bcmv_state state)
{
#if 0 /* XXX */
        if (device->info.has_llc)
                return;

        gen_flush_range(state.map, state.alloc_size);
#endif
}

void bcmv_device_init_blorp(struct bcmv_device *device);
void bcmv_device_finish_blorp(struct bcmv_device *device);

VkResult bcmv_device_submit_cl(struct bcmv_device *device,
                               struct drm_vc5_submit_cl *submit,
                               struct bcmv_bo **submit_bos);
VkResult bcmv_device_query_status(struct bcmv_device *device);
VkResult bcmv_device_bo_busy(struct bcmv_device *device, struct bcmv_bo *bo);
VkResult bcmv_device_wait(struct bcmv_device *device, struct bcmv_bo *bo,
                          int64_t timeout);

void* bcmv_gem_mmap(struct bcmv_device *device,
                    uint32_t gem_handle, uint64_t size);
void bcmv_gem_munmap(void *p, uint64_t size);
uint32_t bcmv_gem_create(struct bcmv_device *device, uint64_t size);
void bcmv_gem_close(struct bcmv_device *device, uint32_t gem_handle);
uint32_t bcmv_gem_userptr(struct bcmv_device *device, void *mem, size_t size);
int bcmv_gem_busy(struct bcmv_device *device, uint32_t gem_handle);
int bcmv_gem_wait(struct bcmv_device *device, uint32_t gem_handle, int64_t *timeout_ns);
int bcmv_gem_submit_cl(struct bcmv_device *device,
                       struct drm_vc5_submit_cl *execbuf);
int bcmv_gem_set_tiling(struct bcmv_device *device, uint32_t gem_handle,
                        uint32_t stride, uint32_t tiling);
int bcmv_gem_create_context(struct bcmv_device *device);
int bcmv_gem_destroy_context(struct bcmv_device *device, int context);
int bcmv_gem_get_param(int fd, uint32_t param);
int bcmv_gem_gpu_get_reset_stats(struct bcmv_device *device,
                                 uint32_t *active, uint32_t *pending);
int bcmv_gem_handle_to_fd(struct bcmv_device *device, uint32_t gem_handle);
uint32_t bcmv_gem_fd_to_handle(struct bcmv_device *device, int fd);
int bcmv_gem_set_caching(struct bcmv_device *device, uint32_t gem_handle, uint32_t caching);
int bcmv_gem_set_domain(struct bcmv_device *device, uint32_t gem_handle,
                        uint32_t read_domains, uint32_t write_domain);
int bcmv_gem_sync_file_merge(struct bcmv_device *device, int fd1, int fd2);
uint32_t bcmv_gem_syncobj_create(struct bcmv_device *device, uint32_t flags);
void bcmv_gem_syncobj_destroy(struct bcmv_device *device, uint32_t handle);
int bcmv_gem_syncobj_handle_to_fd(struct bcmv_device *device, uint32_t handle);
uint32_t bcmv_gem_syncobj_fd_to_handle(struct bcmv_device *device, int fd);
int bcmv_gem_syncobj_export_sync_file(struct bcmv_device *device,
                                      uint32_t handle);
int bcmv_gem_syncobj_import_sync_file(struct bcmv_device *device,
                                      uint32_t handle, int fd);
void bcmv_gem_syncobj_reset(struct bcmv_device *device, uint32_t handle);
bool bcmv_gem_supports_syncobj_wait(int fd);
int bcmv_gem_syncobj_wait(struct bcmv_device *device,
                          uint32_t *handles, uint32_t num_handles,
                          int64_t abs_timeout_ns, bool wait_all);

VkResult bcmv_bo_init_new(struct bcmv_bo *bo, struct bcmv_device *device, uint64_t size);

struct bcmv_reloc_list {
        uint32_t num_relocs;
        uint32_t array_length;
        uint32_t *handles;
        struct bcmv_bo **reloc_bos;
};

VkResult bcmv_reloc_list_init(struct bcmv_reloc_list *list,
                              const VkAllocationCallbacks *alloc);
void bcmv_reloc_list_finish(struct bcmv_reloc_list *list,
                            const VkAllocationCallbacks *alloc);

VkResult bcmv_reloc_list_add(struct bcmv_reloc_list *list,
                             const VkAllocationCallbacks *alloc,
                             uint32_t offset, struct bcmv_bo *target_bo,
                             uint32_t delta);

struct bcmv_batch_bo {
        /* Link in the bcmv_cmd_buffer.owned_batch_bos list */
        struct list_head link;

        struct bcmv_bo bo;

        struct bcmv_reloc_list relocs;
};

struct bcmv_cl {
        struct bcmv_batch *batch;
        void *base;
        void *next;
        struct bcmv_bo *bo;
        uint32_t size;
};

/* Struct for tracking CL commands.  A cmd_buffer contains a complete set in
 * BO memory, while a pipeline has an in-CPU-memory set of commands to be
 * emitted for changing to that pipeline.
 */
struct bcmv_batch {
        const VkAllocationCallbacks *alloc;

        struct bcmv_cl bcl;
        struct bcmv_cl rcl;
        struct bcmv_cl indirect;

        struct bcmv_reloc_list *relocs;

        struct bcmv_cmd_buffer *cmd_buffer;

        /**
         * Current error status of the command buffer. Used to track inconsistent
         * or incomplete command buffer states that are the consequence of run-time
         * errors such as out of memory scenarios. We want to track this in the
         * batch because the command buffer object is not visible to some parts
         * of the driver.
         */
        VkResult status;
};

struct bcmv_address {
        struct bcmv_bo *bo;
        uint32_t offset;
};

static inline struct bcmv_address bcmv_address(struct bcmv_bo *bo,
                                               uint32_t offset)
{
        struct bcmv_address address = {
                bo,
                offset
        };

        return address;
}

void *bcmv_get_cl_space_chained(struct bcmv_cl *cl, uint32_t size);
void *bcmv_get_cl_space_realloc(struct bcmv_cl *cl, uint32_t size);
VkResult bcmv_batch_init(struct bcmv_cmd_buffer *cmd_buffer);
void bcmv_batch_free(struct bcmv_batch *batch);
void bcmv_batch_emit_batch(struct bcmv_batch *batch, struct bcmv_batch *other);
void bcmv_batch_emit_reloc(struct bcmv_batch *batch,
                           const struct bcmv_address *address);
VkResult bcmv_device_submit_simple_batch(struct bcmv_device *device,
                                         struct bcmv_batch *batch);

static inline VkResult
bcmv_batch_set_error(struct bcmv_batch *batch, VkResult error)
{
        assert(error != VK_SUCCESS);
        if (batch->status == VK_SUCCESS)
                batch->status = error;
        return batch->status;
}

static inline bool
bcmv_batch_has_error(struct bcmv_batch *batch)
{
        return batch->status != VK_SUCCESS;
}

static inline uint32_t
_bcmv_address_offset(const struct bcmv_address *address)
{
        if (address->bo)
                return address->bo->offset + address->offset;
        else
                return address->offset;
}

#define __gen_address_type struct bcmv_address
#define __gen_user_data struct bcmv_batch
#define __gen_address_offset _bcmv_address_offset
#define __gen_emit_reloc bcmv_batch_emit_reloc

/* Wrapper macros needed to work around preprocessor argument issues.  In
 * particular, arguments don't get pre-evaluated if they are concatenated.
 * This means that, if you pass GENX(3DSTATE_PS) into the emit macro, the
 * GENX macro won't get evaluated if the emit macro contains "cmd ## foo".
 * We can work around this easily enough with these helpers.
 */
#define __bcmv_packet_length(cmd) cmd ## _length
#define __bcmv_packet_length_bias(cmd) cmd ## _length_bias
#define __bcmv_packet_header(cmd) cmd ## _header
#define __bcmv_packet_pack(cmd) cmd ## _pack
#define __bcmv_reg_num(reg) reg ## _num

static inline uint32_t bcmv_cl_offset(struct bcmv_cl *cl)
{
        return cl->next - cl->base;
}

/* Macro for setting up an emit of a CL struct.  A temporary unpacked struct
 * is created, which you get to set fields in of the form:
 *
 * cl_emit(bcl, FLAT_SHADE_FLAGS, flags) {
 *     .flags.flat_shade_flags = 1 << 2,
 * }
 *
 * or default values only can be emitted with just:
 *
 * cl_emit(bcl, FLAT_SHADE_FLAGS, flags);
 *
 * The trick here is that we make a for loop that will execute the body
 * (either the block or the ';' after the macro invocation) exactly once.
 */
#define bcmv_cl_emit(batch, cl, packet, name, get_space)         \
        for (struct packet name = {                              \
                        __bcmv_packet_header(packet)             \
             },                                                  \
             *_dst = get_space(cl, __bcmv_packet_length(packet)); \
             __builtin_expect(_dst != NULL, 1);                  \
        ({                                                       \
                __bcmv_packet_pack(packet)(batch, (uint8_t *)_dst, &name); \
                VG(VALGRIND_CHECK_MEM_IS_DEFINED(_dst,           \
                                                 __bcmv_packet_length(packet))); \
                _dst = NULL;                                     \
        }))                                                      \

#define bcmv_bcl_emit(batch, packet, name) bcmv_cl_emit(batch, &(batch)->bcl, packet, name, bcmv_get_cl_space_chained)
#define bcmv_rcl_emit(batch, packet, name) bcmv_cl_emit(batch, &(batch)->rcl, packet, name, bcmv_get_cl_space_chained)

#define bcmv_pack_struct(dst, struc, ...) do {                          \
                struct struc __template = {                             \
                        __VA_ARGS__                                     \
                };                                                      \
                __bcmv_packet_pack(struc)(NULL, dst, &__template);         \
                VG(VALGRIND_CHECK_MEM_IS_DEFINED(dst, __bcmv_packet_length(struc) * 4)); \
        } while (0)

#define bcmv_batch_emitn(batch, n, cmd, ...) ({                         \
                        void *__dst = bcmv_batch_emit_dwords(batch, n); \
                        if (__dst) {                                    \
                                struct cmd __template = {               \
                                        __bcmv_cmd_header(cmd),         \
                                        .DWordLength = n - __bcmv_cmd_length_bias(cmd), \
                                        __VA_ARGS__                     \
                                };                                      \
                                __bcmv_cmd_pack(cmd)(batch, __dst, &__template); \
                        }                                               \
                        __dst;                                          \
                })

#define bcmv_batch_emit_merge(batch, dwords0, dwords1)                  \
        do {                                                            \
                uint32_t *dw;                                           \
                                                                        \
                STATIC_ASSERT(ARRAY_SIZE(dwords0) == ARRAY_SIZE(dwords1)); \
                dw = bcmv_batch_emit_dwords((batch), ARRAY_SIZE(dwords0)); \
                if (!dw)                                                \
                        break;                                          \
                for (uint32_t i = 0; i < ARRAY_SIZE(dwords0); i++)      \
                        dw[i] = (dwords0)[i] | (dwords1)[i];            \
                VG(VALGRIND_CHECK_MEM_IS_DEFINED(dw, ARRAY_SIZE(dwords0) * 4)); \
        } while (0)

#define bcmv_batch_emit(batch, cmd, name)                               \
        for (struct cmd name = { __bcmv_cmd_header(cmd) },              \
                     *_dst = bcmv_batch_emit_dwords(batch, __bcmv_cmd_length(cmd)); \
             __builtin_expect(_dst != NULL, 1);                         \
             ({ __bcmv_cmd_pack(cmd)(batch, _dst, &name);               \
                     VG(VALGRIND_CHECK_MEM_IS_DEFINED(_dst, __bcmv_cmd_length(cmd) * 4)); \
                     _dst = NULL;                                       \
             }))

struct bcmv_device_memory {
        struct bcmv_bo *                              bo;
        struct bcmv_memory_type *                     type;
        VkDeviceSize                                 map_size;
        void *                                       map;
};

/**
 * Header for Vertex URB Entry (VUE)
 */
struct bcmv_vue_header {
        uint32_t Reserved;
        uint32_t RTAIndex; /* RenderTargetArrayIndex */
        uint32_t ViewportIndex;
        float PointWidth;
};

struct bcmv_descriptor_set_binding_layout {
#ifndef NDEBUG
        /* The type of the descriptors in this binding */
        VkDescriptorType type;
#endif

        /* Number of array elements in this binding */
        uint16_t array_size;

        /* Index into the flattend descriptor set */
        uint16_t descriptor_index;

        /* Index into the dynamic state array for a dynamic buffer */
        int16_t dynamic_offset_index;

        /* Index into the descriptor set buffer views */
        int16_t buffer_index;

        struct {
                /* Index into the binding table for the associated surface */
                int16_t surface_index;

                /* Index into the sampler table for the associated sampler */
                int16_t sampler_index;

                /* Index into the image table for the associated image */
                int16_t image_index;
        } stage[MESA_SHADER_STAGES];

        /* Immutable samplers (or NULL if no immutable samplers) */
        struct bcmv_sampler **immutable_samplers;
};

struct bcmv_descriptor_set_layout {
        /* Number of bindings in this descriptor set */
        uint16_t binding_count;

        /* Total size of the descriptor set with room for all array entries */
        uint16_t size;

        /* Shader stages affected by this descriptor set */
        uint16_t shader_stages;

        /* Number of buffers in this descriptor set */
        uint16_t buffer_count;

        /* Number of dynamic offsets used by this descriptor set */
        uint16_t dynamic_offset_count;

        /* Bindings in this descriptor set */
        struct bcmv_descriptor_set_binding_layout binding[0];
};

struct bcmv_descriptor {
        VkDescriptorType type;

        union {
                struct {
                        VkImageLayout layout;
                        struct bcmv_image_view *image_view;
                        struct bcmv_sampler *sampler;
                };

                struct {
                        struct bcmv_buffer *buffer;
                        uint64_t offset;
                        uint64_t range;
                };

                struct bcmv_buffer_view *buffer_view;
        };
};

struct bcmv_descriptor_set {
        const struct bcmv_descriptor_set_layout *layout;
        uint32_t size;
        uint32_t buffer_count;
        struct bcmv_buffer_view *buffer_views;
        struct bcmv_descriptor descriptors[0];
};

struct bcmv_buffer_view {
        /* XXX */uint32_t format; /**< VkBufferViewCreateInfo::format */
        struct bcmv_bo *bo;
        uint32_t offset; /**< Offset into bo. */
        uint64_t range; /**< VkBufferViewCreateInfo::range */

        struct bcmv_state surface_state;
        struct bcmv_state storage_surface_state;
        struct bcmv_state writeonly_storage_surface_state;

        /* XXX */ /* struct v3d_image_param storage_image_param; */
};

struct bcmv_push_descriptor_set {
        struct bcmv_descriptor_set set;

        /* Put this field right behind bcmv_descriptor_set so it fills up the
         * descriptors[0] field. */
        struct bcmv_descriptor descriptors[MAX_PUSH_DESCRIPTORS];

        struct bcmv_buffer_view buffer_views[MAX_PUSH_DESCRIPTORS];
};

struct bcmv_descriptor_pool {
        uint32_t size;
        uint32_t next;
        uint32_t free_list;

        struct bcmv_state_stream surface_state_stream;
        void *surface_state_free_list;

        char data[0];
};

enum bcmv_descriptor_template_entry_type {
        BCMV_DESCRIPTOR_TEMPLATE_ENTRY_TYPE_IMAGE,
        BCMV_DESCRIPTOR_TEMPLATE_ENTRY_TYPE_BUFFER,
        BCMV_DESCRIPTOR_TEMPLATE_ENTRY_TYPE_BUFFER_VIEW
};

struct bcmv_descriptor_template_entry {
        /* The type of descriptor in this entry */
        VkDescriptorType type;

        /* Binding in the descriptor set */
        uint32_t binding;

        /* Offset at which to write into the descriptor set binding */
        uint32_t array_element;

        /* Number of elements to write into the descriptor set binding */
        uint32_t array_count;

        /* Offset into the user provided data */
        size_t offset;

        /* Stride between elements into the user provided data */
        size_t stride;
};

struct bcmv_descriptor_update_template {
        /* The descriptor set this template corresponds to. This value is only
         * valid if the template was created with the templateType
         * VK_DESCRIPTOR_UPDATE_TEMPLATE_TYPE_DESCRIPTOR_SET_KHR.
         */
        uint8_t set;

        /* Number of entries in this template */
        uint32_t entry_count;

        /* Entries of the template */
        struct bcmv_descriptor_template_entry entries[0];
};

size_t
bcmv_descriptor_set_layout_size(const struct bcmv_descriptor_set_layout *layout);

void
bcmv_descriptor_set_write_image_view(struct bcmv_descriptor_set *set,
                                     const struct v3d_device_info * const devinfo,
                                     const VkDescriptorImageInfo * const info,
                                     VkDescriptorType type,
                                     uint32_t binding,
                                     uint32_t element);

void
bcmv_descriptor_set_write_buffer_view(struct bcmv_descriptor_set *set,
                                      VkDescriptorType type,
                                      struct bcmv_buffer_view *buffer_view,
                                      uint32_t binding,
                                      uint32_t element);

void
bcmv_descriptor_set_write_buffer(struct bcmv_descriptor_set *set,
                                 struct bcmv_device *device,
                                 struct bcmv_state_stream *alloc_stream,
                                 VkDescriptorType type,
                                 struct bcmv_buffer *buffer,
                                 uint32_t binding,
                                 uint32_t element,
                                 VkDeviceSize offset,
                                 VkDeviceSize range);

void
bcmv_descriptor_set_write_template(struct bcmv_descriptor_set *set,
                                   struct bcmv_device *device,
                                   struct bcmv_state_stream *alloc_stream,
                                   const struct bcmv_descriptor_update_template *template,
                                   const void *data);

VkResult
bcmv_descriptor_set_create(struct bcmv_device *device,
                           struct bcmv_descriptor_pool *pool,
                           const struct bcmv_descriptor_set_layout *layout,
                           struct bcmv_descriptor_set **out_set);

void
bcmv_descriptor_set_destroy(struct bcmv_device *device,
                            struct bcmv_descriptor_pool *pool,
                            struct bcmv_descriptor_set *set);

#define BCMV_DESCRIPTOR_SET_COLOR_ATTACHMENTS UINT8_MAX

struct bcmv_pipeline_binding {
        /* The descriptor set this surface corresponds to.  The special value of
         * BCMV_DESCRIPTOR_SET_COLOR_ATTACHMENTS indicates that the offset refers
         * to a color attachment and not a regular descriptor.
         */
        uint8_t set;

        /* Binding in the descriptor set */
        uint32_t binding;

        /* Index in the binding */
        uint32_t index;

        /* Input attachment index (relative to the subpass) */
        uint8_t input_attachment_index;

        /* For a storage image, whether it is write-only */
        bool write_only;
};

struct bcmv_pipeline_layout {
        struct {
                struct bcmv_descriptor_set_layout *layout;
                uint32_t dynamic_offset_start;
        } set[MAX_SETS];

        uint32_t num_sets;

        struct {
                bool has_dynamic_offsets;
        } stage[MESA_SHADER_STAGES];

        unsigned char sha1[20];
};

struct bcmv_buffer {
        struct bcmv_device *                          device;
        VkDeviceSize                                 size;

        VkBufferUsageFlags                           usage;

        /* Set when bound */
        struct bcmv_bo *                              bo;
        VkDeviceSize                                 offset;
};

static inline uint64_t
bcmv_buffer_get_range(struct bcmv_buffer *buffer, uint64_t offset, uint64_t range)
{
        assert(offset <= buffer->size);
        if (range == VK_WHOLE_SIZE) {
                return buffer->size - offset;
        } else {
                assert(range <= buffer->size);
                return range;
        }
}

enum bcmv_cmd_dirty_bits {
        BCMV_CMD_DIRTY_DYNAMIC_VIEWPORT                  = 1 << 0, /* VK_DYNAMIC_STATE_VIEWPORT */
        BCMV_CMD_DIRTY_DYNAMIC_SCISSOR                   = 1 << 1, /* VK_DYNAMIC_STATE_SCISSOR */
        BCMV_CMD_DIRTY_DYNAMIC_LINE_WIDTH                = 1 << 2, /* VK_DYNAMIC_STATE_LINE_WIDTH */
        BCMV_CMD_DIRTY_DYNAMIC_DEPTH_BIAS                = 1 << 3, /* VK_DYNAMIC_STATE_DEPTH_BIAS */
        BCMV_CMD_DIRTY_DYNAMIC_BLEND_CONSTANTS           = 1 << 4, /* VK_DYNAMIC_STATE_BLEND_CONSTANTS */
        BCMV_CMD_DIRTY_DYNAMIC_DEPTH_BOUNDS              = 1 << 5, /* VK_DYNAMIC_STATE_DEPTH_BOUNDS */
        BCMV_CMD_DIRTY_DYNAMIC_STENCIL_COMPARE_MASK      = 1 << 6, /* VK_DYNAMIC_STATE_STENCIL_COMPARE_MASK */
        BCMV_CMD_DIRTY_DYNAMIC_STENCIL_WRITE_MASK        = 1 << 7, /* VK_DYNAMIC_STATE_STENCIL_WRITE_MASK */
        BCMV_CMD_DIRTY_DYNAMIC_STENCIL_REFERENCE         = 1 << 8, /* VK_DYNAMIC_STATE_STENCIL_REFERENCE */
        BCMV_CMD_DIRTY_DYNAMIC_ALL                       = (1 << 9) - 1,
        BCMV_CMD_DIRTY_PIPELINE                          = 1 << 9,
        BCMV_CMD_DIRTY_INDEX_BUFFER                      = 1 << 10,
        BCMV_CMD_DIRTY_RENDER_TARGETS                    = 1 << 11,
};
typedef uint32_t bcmv_cmd_dirty_mask_t;

enum bcmv_pipe_bits {
        BCMV_PIPE_DEPTH_CACHE_FLUSH_BIT            = (1 << 0),
        BCMV_PIPE_STALL_AT_SCOREBOARD_BIT          = (1 << 1),
        BCMV_PIPE_STATE_CACHE_INVALIDATE_BIT       = (1 << 2),
        BCMV_PIPE_CONSTANT_CACHE_INVALIDATE_BIT    = (1 << 3),
        BCMV_PIPE_VF_CACHE_INVALIDATE_BIT          = (1 << 4),
        BCMV_PIPE_DATA_CACHE_FLUSH_BIT             = (1 << 5),
        BCMV_PIPE_TEXTURE_CACHE_INVALIDATE_BIT     = (1 << 10),
        BCMV_PIPE_INSTRUCTION_CACHE_INVALIDATE_BIT = (1 << 11),
        BCMV_PIPE_RENDER_TARGET_CACHE_FLUSH_BIT    = (1 << 12),
        BCMV_PIPE_DEPTH_STALL_BIT                  = (1 << 13),
        BCMV_PIPE_CS_STALL_BIT                     = (1 << 20),

        /* This bit does not exist directly in PIPE_CONTROL.  Instead it means that
         * a flush has happened but not a CS stall.  The next time we do any sort
         * of invalidation we need to insert a CS stall at that time.  Otherwise,
         * we would have to CS stall on every flush which could be bad.
         */
        BCMV_PIPE_NEEDS_CS_STALL_BIT               = (1 << 21),
};

#define BCMV_PIPE_FLUSH_BITS (                                  \
                BCMV_PIPE_DEPTH_CACHE_FLUSH_BIT |               \
                BCMV_PIPE_DATA_CACHE_FLUSH_BIT |                \
                BCMV_PIPE_RENDER_TARGET_CACHE_FLUSH_BIT)

#define BCMV_PIPE_STALL_BITS (                          \
                BCMV_PIPE_STALL_AT_SCOREBOARD_BIT |     \
                BCMV_PIPE_DEPTH_STALL_BIT |             \
                BCMV_PIPE_CS_STALL_BIT)

#define BCMV_PIPE_INVALIDATE_BITS (                             \
                BCMV_PIPE_STATE_CACHE_INVALIDATE_BIT |          \
                BCMV_PIPE_CONSTANT_CACHE_INVALIDATE_BIT |       \
                BCMV_PIPE_VF_CACHE_INVALIDATE_BIT |             \
                BCMV_PIPE_DATA_CACHE_FLUSH_BIT |                \
                BCMV_PIPE_TEXTURE_CACHE_INVALIDATE_BIT |        \
                BCMV_PIPE_INSTRUCTION_CACHE_INVALIDATE_BIT)

static inline enum bcmv_pipe_bits
bcmv_pipe_flush_bits_for_access_flags(VkAccessFlags flags)
{
        enum bcmv_pipe_bits pipe_bits = 0;

        unsigned b;
        for_each_bit(b, flags) {
                switch ((VkAccessFlagBits)(1 << b)) {
                case VK_ACCESS_SHADER_WRITE_BIT:
                        pipe_bits |= BCMV_PIPE_DATA_CACHE_FLUSH_BIT;
                        break;
                case VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT:
                        pipe_bits |= BCMV_PIPE_RENDER_TARGET_CACHE_FLUSH_BIT;
                        break;
                case VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT:
                        pipe_bits |= BCMV_PIPE_DEPTH_CACHE_FLUSH_BIT;
                        break;
                case VK_ACCESS_TRANSFER_WRITE_BIT:
                        pipe_bits |= BCMV_PIPE_RENDER_TARGET_CACHE_FLUSH_BIT;
                        pipe_bits |= BCMV_PIPE_DEPTH_CACHE_FLUSH_BIT;
                        break;
                default:
                        break; /* Nothing to do */
                }
        }

        return pipe_bits;
}

static inline enum bcmv_pipe_bits
bcmv_pipe_invalidate_bits_for_access_flags(VkAccessFlags flags)
{
        enum bcmv_pipe_bits pipe_bits = 0;

        unsigned b;
        for_each_bit(b, flags) {
                switch ((VkAccessFlagBits)(1 << b)) {
                case VK_ACCESS_INDIRECT_COMMAND_READ_BIT:
                case VK_ACCESS_INDEX_READ_BIT:
                case VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT:
                        pipe_bits |= BCMV_PIPE_VF_CACHE_INVALIDATE_BIT;
                        break;
                case VK_ACCESS_UNIFORM_READ_BIT:
                        pipe_bits |= BCMV_PIPE_CONSTANT_CACHE_INVALIDATE_BIT;
                        pipe_bits |= BCMV_PIPE_TEXTURE_CACHE_INVALIDATE_BIT;
                        break;
                case VK_ACCESS_SHADER_READ_BIT:
                case VK_ACCESS_INPUT_ATTACHMENT_READ_BIT:
                case VK_ACCESS_TRANSFER_READ_BIT:
                        pipe_bits |= BCMV_PIPE_TEXTURE_CACHE_INVALIDATE_BIT;
                        break;
                default:
                        break; /* Nothing to do */
                }
        }

        return pipe_bits;
}

struct bcmv_vertex_binding {
        struct bcmv_buffer *                          buffer;
        VkDeviceSize                                 offset;
};

struct bcmv_push_constants {
        /* Current allocated size of this push constants data structure.
         * Because a decent chunk of it may not be used (images on SKL, for
         * instance), we won't actually allocate the entire structure up-front.
         */
        uint32_t size;

        /* Push constant data provided by the client through vkPushConstants */
        uint8_t client_data[MAX_PUSH_CONSTANTS_SIZE];

        /* Our hardware only provides zero-based vertex and instance id so, in
         * order to satisfy the vulkan requirements, we may have to push one or
         * both of these into the shader.
         */
        uint32_t base_vertex;
        uint32_t base_instance;

        /* Image data for image_load_store on pre-SKL */
        /* XXX */ /*struct v3d_image_param images[MAX_IMAGES];*/
};

struct bcmv_dynamic_state {
        struct {
                uint32_t                                  count;
                VkViewport                                viewports[MAX_VIEWPORTS];
        } viewport;

        struct {
                uint32_t                                  count;
                VkRect2D                                  scissors[MAX_SCISSORS];
        } scissor;

        float                                        line_width;

        struct {
                float                                     bias;
                float                                     clamp;
                float                                     slope;
        } depth_bias;

        float                                        blend_constants[4];

        struct {
                float                                     min;
                float                                     max;
        } depth_bounds;

        struct {
                uint32_t                                  front;
                uint32_t                                  back;
        } stencil_compare_mask;

        struct {
                uint32_t                                  front;
                uint32_t                                  back;
        } stencil_write_mask;

        struct {
                uint32_t                                  front;
                uint32_t                                  back;
        } stencil_reference;
};

extern const struct bcmv_dynamic_state default_dynamic_state;

void bcmv_dynamic_state_copy(struct bcmv_dynamic_state *dest,
                             const struct bcmv_dynamic_state *src,
                             uint32_t copy_mask);

/**
 * Attachment state when recording a renderpass instance.
 *
 * The clear value is valid only if there exists a pending clear.
 */
struct bcmv_attachment_state {
        struct bcmv_state                             color_rt_state;
        struct bcmv_state                             input_att_state;

        VkImageLayout                                current_layout;
        VkImageAspectFlags                           pending_clear_aspects;
        bool                                         fast_clear;
        VkClearValue                                 clear_value;
        bool                                         clear_color_is_zero_one;
        bool                                         clear_color_is_zero;
};

/** State required while building cmd buffer */
struct bcmv_cmd_state {
        /* PIPELINE_SELECT.PipelineSelection */
        uint32_t                                     current_pipeline;
        const struct gen_l3_config *                 current_l3_config;
        uint32_t                                     vb_dirty;
        bcmv_cmd_dirty_mask_t                         dirty;
        bcmv_cmd_dirty_mask_t                         compute_dirty;
        enum bcmv_pipe_bits                           pending_pipe_bits;
        uint32_t                                     num_workgroups_offset;
        struct bcmv_bo                                *num_workgroups_bo;
        VkShaderStageFlags                           descriptors_dirty;
        VkShaderStageFlags                           push_constants_dirty;
        uint32_t                                     scratch_size;
        struct bcmv_pipeline *                        pipeline;
        struct bcmv_pipeline *                        compute_pipeline;
        struct bcmv_framebuffer *                     framebuffer;
        struct bcmv_render_pass *                     pass;
        struct bcmv_subpass *                         subpass;
        VkRect2D                                     render_area;
        uint32_t                                     restart_index;
        struct bcmv_vertex_binding                    vertex_bindings[MAX_VBS];
        struct bcmv_descriptor_set *                  descriptors[MAX_SETS];
        uint32_t                                     dynamic_offsets[MAX_DYNAMIC_BUFFERS];
        VkShaderStageFlags                           push_constant_stages;
        struct bcmv_push_constants *                  push_constants[MESA_SHADER_STAGES];
        struct bcmv_state                             binding_tables[MESA_SHADER_STAGES];
        struct bcmv_state                             samplers[MESA_SHADER_STAGES];
        struct bcmv_dynamic_state                     dynamic;
        bool                                         need_query_wa;

        struct bcmv_push_descriptor_set               push_descriptor;

        /**
         * Array length is bcmv_cmd_state::pass::attachment_count. Array content is
         * valid only when recording a render pass instance.
         */
        struct bcmv_attachment_state *                attachments;

        /**
         * Surface states for color render targets.  These are stored in a single
         * flat array.  For depth-stencil attachments, the surface state is simply
         * left blank.
         */
        struct bcmv_state                             render_pass_states;

        /**
         * A null surface state of the right size to match the framebuffer.  This
         * is one of the states in render_pass_states.
         */
        struct bcmv_state                             null_surface_state;

        struct bcmv_buffer *index_buffer;
        uint32_t index_type; /**< INDEX_TYPE_* */
        uint32_t index_offset;
};

struct bcmv_cmd_pool {
        VkAllocationCallbacks                        alloc;
        struct list_head                             cmd_buffers;
};

#define BCMV_CMD_BUFFER_BATCH_SIZE 8192

enum bcmv_cmd_buffer_exec_mode {
        BCMV_CMD_BUFFER_EXEC_MODE_PRIMARY,
        BCMV_CMD_BUFFER_EXEC_MODE_EMIT,
        BCMV_CMD_BUFFER_EXEC_MODE_GROW_AND_EMIT,
        BCMV_CMD_BUFFER_EXEC_MODE_CHAIN,
        BCMV_CMD_BUFFER_EXEC_MODE_COPY_AND_CHAIN,
};

struct bcmv_cmd_buffer {
        VK_LOADER_DATA                               _loader_data;

        struct bcmv_device *                          device;

        struct bcmv_cmd_pool *                        pool;
        struct list_head                             pool_link;

        struct bcmv_batch                             batch;

        /* Fields required for the actual chain of bcmv_batch_bo's.
         *
         * These fields are initialized by bcmv_cmd_buffer_init_batch_bo_chain().
         */
        struct list_head                             batch_bos;
        enum bcmv_cmd_buffer_exec_mode                exec_mode;

        /* A vector of bcmv_batch_bo pointers for every batch or surface buffer
         * referenced by this command buffer
         *
         * initialized by bcmv_cmd_buffer_init_batch_bo_chain()
         */
        struct u_vector                            seen_bbos;

        /* A vector of int32_t's for every block of binding tables.
         *
         * initialized by bcmv_cmd_buffer_init_batch_bo_chain()
         */
        struct u_vector                              bt_block_states;
        uint32_t                                     bt_next;

        struct bcmv_reloc_list                        surface_relocs;
        /** Last seen surface state block pool center bo offset */
        uint32_t                                     last_ss_pool_center;

        /* Serial for tracking buffer completion */
        uint32_t                                     serial;

        /* Stream objects for storing temporary data */
        struct bcmv_state_stream                      surface_state_stream;
        struct bcmv_state_stream                      dynamic_state_stream;

        VkCommandBufferUsageFlags                    usage_flags;
        VkCommandBufferLevel                         level;

        struct bcmv_cmd_state                         state;
};

VkResult bcmv_cmd_buffer_init_batch_bo_chain(struct bcmv_cmd_buffer *cmd_buffer);
void bcmv_cmd_buffer_fini_batch_bo_chain(struct bcmv_cmd_buffer *cmd_buffer);
void bcmv_cmd_buffer_reset_batch_bo_chain(struct bcmv_cmd_buffer *cmd_buffer);
void bcmv_cmd_buffer_end_batch_buffer(struct bcmv_cmd_buffer *cmd_buffer);
void bcmv_cmd_buffer_add_secondary(struct bcmv_cmd_buffer *primary,
                                   struct bcmv_cmd_buffer *secondary);
void bcmv_cmd_buffer_prepare_submit(struct bcmv_cmd_buffer *cmd_buffer);
VkResult bcmv_cmd_buffer_submit(struct bcmv_device *device,
                                struct bcmv_cmd_buffer *cmd_buffer,
                                const VkSemaphore *in_semaphores,
                                uint32_t num_in_semaphores,
                                 const VkSemaphore *out_semaphores,
                                uint32_t num_out_semaphores,
                                VkFence fence);

VkResult bcmv_cmd_buffer_reset(struct bcmv_cmd_buffer *cmd_buffer);

VkResult
bcmv_cmd_buffer_ensure_push_constants_size(struct bcmv_cmd_buffer *cmd_buffer,
                                           gl_shader_stage stage, uint32_t size);
#define bcmv_cmd_buffer_ensure_push_constant_field(cmd_buffer, stage, field) \
        bcmv_cmd_buffer_ensure_push_constants_size(cmd_buffer, stage,   \
                                                   (offsetof(struct bcmv_push_constants, field) + \
                                                    sizeof(cmd_buffer->state.push_constants[0]->field)))

struct bcmv_state bcmv_cmd_buffer_emit_dynamic(struct bcmv_cmd_buffer *cmd_buffer,
                                               const void *data, uint32_t size, uint32_t alignment);
struct bcmv_state bcmv_cmd_buffer_merge_dynamic(struct bcmv_cmd_buffer *cmd_buffer,
                                                uint32_t *a, uint32_t *b,
                                                uint32_t dwords, uint32_t alignment);

struct bcmv_address
bcmv_cmd_buffer_surface_base_address(struct bcmv_cmd_buffer *cmd_buffer);
struct bcmv_state
bcmv_cmd_buffer_alloc_binding_table(struct bcmv_cmd_buffer *cmd_buffer,
                                    uint32_t entries, uint32_t *state_offset);
struct bcmv_state
bcmv_cmd_buffer_alloc_surface_state(struct bcmv_cmd_buffer *cmd_buffer);
struct bcmv_state
bcmv_cmd_buffer_alloc_dynamic_state(struct bcmv_cmd_buffer *cmd_buffer,
                                    uint32_t size, uint32_t alignment);

VkResult
bcmv_cmd_buffer_new_binding_table_block(struct bcmv_cmd_buffer *cmd_buffer);

void gen8_cmd_buffer_emit_viewport(struct bcmv_cmd_buffer *cmd_buffer);
void gen8_cmd_buffer_emit_depth_viewport(struct bcmv_cmd_buffer *cmd_buffer,
                                         bool depth_clamp_enable);
void gen7_cmd_buffer_emit_scissor(struct bcmv_cmd_buffer *cmd_buffer);

void bcmv_cmd_buffer_setup_attachments(struct bcmv_cmd_buffer *cmd_buffer,
                                       struct bcmv_render_pass *pass,
                                       struct bcmv_framebuffer *framebuffer,
                                       const VkClearValue *clear_values);

void bcmv_cmd_buffer_emit_state_base_address(struct bcmv_cmd_buffer *cmd_buffer);

struct bcmv_state
bcmv_cmd_buffer_push_constants(struct bcmv_cmd_buffer *cmd_buffer,
                               gl_shader_stage stage);
struct bcmv_state
bcmv_cmd_buffer_cs_push_constants(struct bcmv_cmd_buffer *cmd_buffer);

void bcmv_cmd_buffer_clear_subpass(struct bcmv_cmd_buffer *cmd_buffer);
void bcmv_cmd_buffer_resolve_subpass(struct bcmv_cmd_buffer *cmd_buffer);

const struct bcmv_image_view *
bcmv_cmd_buffer_get_depth_stencil_view(const struct bcmv_cmd_buffer *cmd_buffer);

VkResult
bcmv_cmd_buffer_alloc_blorp_binding_table(struct bcmv_cmd_buffer *cmd_buffer,
                                          uint32_t num_entries,
                                          uint32_t *state_offset,
                                          struct bcmv_state *bt_state);

void bcmv_cmd_buffer_dump(struct bcmv_cmd_buffer *cmd_buffer);

enum bcmv_fence_type {
        BCMV_FENCE_TYPE_NONE = 0,
        BCMV_FENCE_TYPE_SYNCOBJ,
};

enum bcmv_bo_fence_state {
        /** Indicates that this is a new (or newly reset fence) */
        BCMV_BO_FENCE_STATE_RESET,

        /** Indicates that this fence has been submitted to the GPU but is still
         * (as far as we know) in use by the GPU.
         */
        BCMV_BO_FENCE_STATE_SUBMITTED,

        BCMV_BO_FENCE_STATE_SIGNALED,
};

struct bcmv_fence_impl {
        enum bcmv_fence_type type;


        /** DRM syncobj handle for syncobj-based fences */
        uint32_t syncobj;
};

struct bcmv_fence {
        /* Permanent fence state.  Every fence has some form of permanent state
         * (type != BCMV_SEMAPHORE_TYPE_NONE).  This may be a BO to fence on (for
         * cross-process fences0 or it could just be a dummy for use internally.
         */
        struct bcmv_fence_impl permanent;

        /* Temporary fence state.  A fence *may* have temporary state.  That state
         * is added to the fence by an import operation and is reset back to
         * BCMV_SEMAPHORE_TYPE_NONE when the fence is reset.  A fence with temporary
         * state cannot be signaled because the fence must already be signaled
         * before the temporary state can be exported from the fence in the other
         * process and imported here.
         */
        struct bcmv_fence_impl temporary;
};

struct bcmv_event {
        uint64_t                                     semaphore;
        struct bcmv_state                             state;
};

enum bcmv_semaphore_type {
        BCMV_SEMAPHORE_TYPE_NONE = 0,
        BCMV_SEMAPHORE_TYPE_DUMMY,
        BCMV_SEMAPHORE_TYPE_SYNC_FILE,
        BCMV_SEMAPHORE_TYPE_DRM_SYNCOBJ,
};

struct bcmv_semaphore_impl {
        enum bcmv_semaphore_type type;

        union {
                /* The sync file descriptor when type == AKV_SEMAPHORE_TYPE_SYNC_FILE.
                 * If the semaphore is in the unsignaled state due to either just being
                 * created or because it has been used for a wait, fd will be -1.
                 */
                int fd;

                /* Sync object handle when type == AKV_SEMAPHORE_TYPE_DRM_SYNCOBJ.
                 * Unlike GEM BOs, DRM sync objects aren't deduplicated by the kernel on
                 * import so we don't need to bother with a userspace cache.
                 */
                uint32_t syncobj;
        };
};

struct bcmv_semaphore {
        /* Permanent semaphore state.  Every semaphore has some form of permanent
         * state (type != BCMV_SEMAPHORE_TYPE_NONE).  This may be a BO to fence on
         * (for cross-process semaphores0 or it could just be a dummy for use
         * internally.
         */
        struct bcmv_semaphore_impl permanent;

        /* Temporary semaphore state.  A semaphore *may* have temporary state.
         * That state is added to the semaphore by an import operation and is reset
         * back to BCMV_SEMAPHORE_TYPE_NONE when the semaphore is waited on.  A
         * semaphore with temporary state cannot be signaled because the semaphore
         * must already be signaled before the temporary state can be exported from
         * the semaphore in the other process and imported here.
         */
        struct bcmv_semaphore_impl temporary;
};

void bcmv_semaphore_reset_temporary(struct bcmv_device *device,
                                    struct bcmv_semaphore *semaphore);

struct bcmv_shader_module {
        unsigned char                                sha1[20];
        uint32_t                                     size;
        char                                         data[0];
};

static inline gl_shader_stage
vk_to_mesa_shader_stage(VkShaderStageFlagBits vk_stage)
{
        assert(__builtin_popcount(vk_stage) == 1);
        return ffs(vk_stage) - 1;
}

static inline VkShaderStageFlagBits
mesa_to_vk_shader_stage(gl_shader_stage mesa_stage)
{
        return (1 << mesa_stage);
}

#define BCMV_STAGE_MASK ((1 << MESA_SHADER_STAGES) - 1)

#define bcmv_foreach_stage(stage, stage_bits)                           \
        for (gl_shader_stage stage,                                     \
                     __tmp = (gl_shader_stage)((stage_bits) & BCMV_STAGE_MASK); \
             stage = __builtin_ffs(__tmp) - 1, __tmp;                   \
             __tmp &= ~(1 << (stage)))

struct bcmv_pipeline_bind_map {
        uint32_t surface_count;
        uint32_t sampler_count;
        uint32_t image_count;

        struct bcmv_pipeline_binding *                surface_to_descriptor;
        struct bcmv_pipeline_binding *                sampler_to_descriptor;
};

struct bcmv_shader_bin_key {
        uint32_t size;
        uint8_t data[0];
};

struct bcmv_shader_bin {
        uint32_t ref_cnt;

        const struct bcmv_shader_bin_key *key;

        struct bcmv_bo bo;
        uint32_t kernel_size;

        const struct v3d_prog_data *prog_data;
        uint32_t prog_data_size;

        struct bcmv_pipeline_bind_map bind_map;

        /* Prog data follows, then uniforms, then the key, all aligned to 8-bytes */
};

struct bcmv_shader_bin *
bcmv_shader_bin_create(struct bcmv_device *device,
                       const void *key, uint32_t key_size,
                       const void *kernel, uint32_t kernel_size,
                       const struct v3d_prog_data *prog_data,
                       uint32_t prog_data_size,
                       const enum quniform_contents *uniform_contents,
                       const uint32_t *uniform_data,
                       const struct bcmv_pipeline_bind_map *bind_map);

void
bcmv_shader_bin_destroy(struct bcmv_device *device, struct bcmv_shader_bin *shader);

static inline void
bcmv_shader_bin_ref(struct bcmv_shader_bin *shader)
{
        assert(shader && shader->ref_cnt >= 1);
        p_atomic_inc(&shader->ref_cnt);
}

static inline void
bcmv_shader_bin_unref(struct bcmv_device *device, struct bcmv_shader_bin *shader)
{
        assert(shader && shader->ref_cnt >= 1);
        if (p_atomic_dec_zero(&shader->ref_cnt))
                bcmv_shader_bin_destroy(device, shader);
}

struct bcmv_pipeline {
        struct bcmv_device *                          device;
        struct bcmv_batch                             batch;
        uint32_t                                     batch_data[512];
        struct bcmv_reloc_list                        batch_relocs;
        uint32_t                                     dynamic_state_mask;
        struct bcmv_dynamic_state                     dynamic_state;

        struct bcmv_subpass *                         subpass;
        struct bcmv_pipeline_layout *                 layout;

        bool                                         needs_data_cache;

        struct bcmv_shader_bin *                      shaders[MESA_SHADER_STAGES];

        struct {
                const struct gen_l3_config *              l3_config;
                uint32_t                                  total_size;
        } urb;

        VkShaderStageFlags                           active_stages;
        struct bcmv_state                             blend_state;

        uint32_t                                     vb_used;
        uint32_t                                     binding_stride[MAX_VBS];
        bool                                         instancing_enable[MAX_VBS];
        bool                                         primitive_restart;
        uint32_t                                     topology;

        uint32_t                                     cs_right_mask;

        bool                                         writes_depth;
        bool                                         depth_test_enable;
        bool                                         writes_stencil;
        bool                                         stencil_test_enable;
        bool                                         depth_clamp_enable;
        bool                                         sample_shading_enable;
        bool                                         kill_pixel;

        uint8_t stencil_config_front[6];
        uint8_t stencil_config_back[6];

        struct {
                uint32_t                                  sf[7];
                uint32_t                                  depth_stencil_state[3];
        } gen7;

        struct {
                uint32_t                                  sf[4];
                uint32_t                                  raster[5];
                uint32_t                                  wm_depth_stencil[3];
        } gen8;

        struct {
                uint32_t                                  wm_depth_stencil[4];
        } gen9;

        uint32_t                                     interface_descriptor_data[8];
};

static inline bool
bcmv_pipeline_has_stage(const struct bcmv_pipeline *pipeline,
                        gl_shader_stage stage)
{
        return (pipeline->active_stages & mesa_to_vk_shader_stage(stage)) != 0;
}

#define BCMV_DECL_GET_PROG_DATA_FUNC(prefix, stage)                     \
        static inline const struct v3d_##prefix##_prog_data *           \
        get_##prefix##_prog_data(const struct bcmv_pipeline *pipeline)  \
        {                                                               \
                if (bcmv_pipeline_has_stage(pipeline, stage)) {         \
                        return (const struct v3d_##prefix##_prog_data *) \
                                pipeline->shaders[stage]->prog_data;    \
                } else {                                                \
                        return NULL;                                    \
                }                                                       \
        }

BCMV_DECL_GET_PROG_DATA_FUNC(vs, MESA_SHADER_VERTEX)
BCMV_DECL_GET_PROG_DATA_FUNC(tcs, MESA_SHADER_TESS_CTRL)
BCMV_DECL_GET_PROG_DATA_FUNC(tes, MESA_SHADER_TESS_EVAL)
BCMV_DECL_GET_PROG_DATA_FUNC(gs, MESA_SHADER_GEOMETRY)
BCMV_DECL_GET_PROG_DATA_FUNC(fs, MESA_SHADER_FRAGMENT)
BCMV_DECL_GET_PROG_DATA_FUNC(cs, MESA_SHADER_COMPUTE)

VkResult
bcmv_pipeline_init(struct bcmv_pipeline *pipeline, struct bcmv_device *device,
                   struct bcmv_pipeline_cache *cache,
                   const VkGraphicsPipelineCreateInfo *pCreateInfo,
                   const VkAllocationCallbacks *alloc);

VkResult
bcmv_pipeline_compile_cs(struct bcmv_pipeline *pipeline,
                         struct bcmv_pipeline_cache *cache,
                         const VkComputePipelineCreateInfo *info,
                         struct bcmv_shader_module *module,
                         const char *entrypoint,
                         const VkSpecializationInfo *spec_info);

struct bcmv_format {
        uint32_t format;
        uint8_t swizzle[4];
};

struct bcmv_format
bcmv_get_format(const struct v3d_device_info *devinfo, VkFormat format,
                VkImageAspectFlags aspect, VkImageTiling tiling);

#if 0 /* XXX */
static inline enum isl_format
bcmv_get_isl_format(const struct v3d_device_info *devinfo, VkFormat vk_format,
                    VkImageAspectFlags aspect, VkImageTiling tiling)
{
        return bcmv_get_format(devinfo, vk_format, aspect, tiling).isl_format;
}

static inline struct isl_swizzle
bcmv_swizzle_for_render(struct isl_swizzle swizzle)
{
        /* Sometimes the swizzle will have alpha map to one.  We do this to fake
         * RGB as RGBA for texturing
         */
        assert(swizzle.a == ISL_CHANNEL_SELECT_ONE ||
               swizzle.a == ISL_CHANNEL_SELECT_ALPHA);

        /* But it doesn't matter what we render to that channel */
        swizzle.a = ISL_CHANNEL_SELECT_ALPHA;

        return swizzle;
}
#endif

/**
 * Subsurface of an bcmv_image.
 */
struct bcmv_surface {
        /** Valid only if isl_surf::size > 0. */
        /* struct isl_surf isl; */ /* XXX */

        /**
         * Offset from VkImage's base address, as bound by vkBindImageMemory().
         */
        uint32_t offset;
};

struct bcmv_image {
        VkImageType type;
        /* The original VkFormat provided by the client.  This may not match any
         * of the actual surface formats.
         */
        VkFormat vk_format;
        VkImageAspectFlags aspects;
        VkExtent3D extent;
        uint32_t levels;
        uint32_t array_size;
        uint32_t samples; /**< VkImageCreateInfo::samples */
        VkImageUsageFlags usage; /**< Superset of VkImageCreateInfo::usage. */
        VkImageTiling tiling; /** VkImageCreateInfo::tiling */

        VkDeviceSize size;
        uint32_t alignment;

        /* Set when bound */
        struct bcmv_bo *bo;
        VkDeviceSize offset;

        /**
         * Image subsurfaces
         *
         * For each foo, bcmv_image::foo_surface is valid if and only if
         * bcmv_image::aspects has a foo aspect.
         *
         * The hardware requires that the depth buffer and stencil buffer be
         * separate surfaces.  From Vulkan's perspective, though, depth and stencil
         * reside in the same VkImage.  To satisfy both the hardware and Vulkan, we
         * allocate the depth and stencil buffers as separate surfaces in the same
         * bo.
         */
        union {
                struct bcmv_surface color_surface;

                struct {
                        struct bcmv_surface depth_surface;
                        struct bcmv_surface stencil_surface;
                };
        };

        struct bcmv_surface aux_surface;
};

void
bcmv_image_fast_clear(struct bcmv_cmd_buffer *cmd_buffer,
                      const struct bcmv_image *image,
                      const uint32_t base_level, const uint32_t level_count,
                      const uint32_t base_layer, uint32_t layer_count);

enum isl_aux_usage
bcmv_layout_to_aux_usage(const struct v3d_device_info * const devinfo,
                         const struct bcmv_image *image,
                         const VkImageAspectFlags aspects,
                         const VkImageLayout layout);

/* This is defined as a macro so that it works for both
 * VkImageSubresourceRange and VkImageSubresourceLayers
 */
#define bcmv_get_layerCount(_image, _range)                             \
        ((_range)->layerCount == VK_REMAINING_ARRAY_LAYERS ?            \
         (_image)->array_size - (_range)->baseArrayLayer : (_range)->layerCount)

static inline uint32_t
bcmv_get_levelCount(const struct bcmv_image *image,
                    const VkImageSubresourceRange *range)
{
        return range->levelCount == VK_REMAINING_MIP_LEVELS ?
                image->levels - range->baseMipLevel : range->levelCount;
}


struct bcmv_image_view {
        const struct bcmv_image *image; /**< VkImageViewCreateInfo::image */
        struct bcmv_bo *bo;
        uint32_t offset; /**< Offset into bo. */

        VkImageAspectFlags aspect_mask;
        VkFormat vk_format;
        VkExtent3D extent; /**< Extent of VkImageViewCreateInfo::baseMipLevel. */

        struct bcmv_state optimal_sampler_surface_state;

        struct bcmv_state general_sampler_surface_state;

        /**
         * RENDER_SURFACE_STATE when using image as a storage image. Separate states
         * for write-only and readable, using the real format for write-only and the
         * lowered format for readable.
         */
        struct bcmv_state storage_surface_state;
        struct bcmv_state writeonly_storage_surface_state;

        /* struct brw_image_param storage_image_param; */ /* XXX */
};

struct bcmv_image_create_info {
        const VkImageCreateInfo *vk_info;

        /* isl_tiling_flags_t isl_tiling_flags; */ /* XXX */

        uint32_t stride;
};

VkResult bcmv_image_create(VkDevice _device,
                           const struct bcmv_image_create_info *info,
                           const VkAllocationCallbacks* alloc,
                           VkImage *pImage);

const struct bcmv_surface *
bcmv_image_get_surface_for_aspect_mask(const struct bcmv_image *image,
                                       VkImageAspectFlags aspect_mask);

enum isl_format
bcmv_isl_format_for_descriptor_type(VkDescriptorType type);

static inline struct VkExtent3D
bcmv_sanitize_image_extent(const VkImageType imageType,
                           const struct VkExtent3D imageExtent)
{
        switch (imageType) {
        case VK_IMAGE_TYPE_1D:
                return (VkExtent3D) { imageExtent.width, 1, 1 };
        case VK_IMAGE_TYPE_2D:
                return (VkExtent3D) { imageExtent.width, imageExtent.height, 1 };
        case VK_IMAGE_TYPE_3D:
                return imageExtent;
        default:
                unreachable("invalid image type");
        }
}

static inline struct VkOffset3D
bcmv_sanitize_image_offset(const VkImageType imageType,
                           const struct VkOffset3D imageOffset)
{
        switch (imageType) {
        case VK_IMAGE_TYPE_1D:
                return (VkOffset3D) { imageOffset.x, 0, 0 };
        case VK_IMAGE_TYPE_2D:
                return (VkOffset3D) { imageOffset.x, imageOffset.y, 0 };
        case VK_IMAGE_TYPE_3D:
                return imageOffset;
        default:
                unreachable("invalid image type");
        }
}


void bcmv_fill_buffer_surface_state(struct bcmv_device *device,
                                    struct bcmv_state state,
                                    enum isl_format format,
                                    uint32_t offset, uint32_t range,
                                    uint32_t stride);

struct bcmv_sampler {
        uint32_t state[4];
};

struct bcmv_framebuffer {
        uint32_t                                     width;
        uint32_t                                     height;
        uint32_t                                     layers;

        uint32_t                                     attachment_count;
        struct bcmv_image_view *                      attachments[0];
};

struct bcmv_subpass {
        uint32_t                                     attachment_count;

        /**
         * A pointer to all attachment references used in this subpass.
         * Only valid if ::attachment_count > 0.
         */
        VkAttachmentReference *                      attachments;
        uint32_t                                     input_count;
        VkAttachmentReference *                      input_attachments;
        uint32_t                                     color_count;
        VkAttachmentReference *                      color_attachments;
        VkAttachmentReference *                      resolve_attachments;

        VkAttachmentReference                        depth_stencil_attachment;

        uint32_t                                     view_mask;

        /** Subpass has a depth/stencil self-dependency */
        bool                                         has_ds_self_dep;

        /** Subpass has at least one resolve attachment */
        bool                                         has_resolve;
};

static inline unsigned
bcmv_subpass_view_count(const struct bcmv_subpass *subpass)
{
        return MAX2(1, util_bitcount(subpass->view_mask));
}

struct bcmv_render_pass_attachment {
        /* TODO: Consider using VkAttachmentDescription instead of storing each of
         * its members individually.
         */
        VkFormat                                     format;
        uint32_t                                     samples;
        VkImageUsageFlags                            usage;
        VkAttachmentLoadOp                           load_op;
        VkAttachmentStoreOp                          store_op;
        VkAttachmentLoadOp                           stencil_load_op;
        VkImageLayout                                initial_layout;
        VkImageLayout                                final_layout;
        VkImageLayout                                first_subpass_layout;

        /* The subpass id in which the attachment will be used last. */
        uint32_t                                     last_subpass_idx;
};

struct bcmv_render_pass {
        uint32_t                                     attachment_count;
        uint32_t                                     subpass_count;
        /* An array of subpass_count+1 flushes, one per subpass boundary */
        enum bcmv_pipe_bits *                         subpass_flushes;
        struct bcmv_render_pass_attachment *          attachments;
        struct bcmv_subpass                           subpasses[0];
};

#define BCMV_PIPELINE_STATISTICS_MASK 0x000007ff

struct bcmv_query_pool {
        VkQueryType                                  type;
        VkQueryPipelineStatisticFlags                pipeline_statistics;
        /** Stride between slots, in bytes */
        uint32_t                                     stride;
        /** Number of slots in this query pool */
        uint32_t                                     slots;
        struct bcmv_bo                                bo;
};

void *bcmv_lookup_entrypoint(const struct v3d_device_info *devinfo,
                             const char *name);

void bcmv_dump_image_to_ppm(struct bcmv_device *device,
                            struct bcmv_image *image, unsigned miplevel,
                            unsigned array_layer, VkImageAspectFlagBits aspect,
                            const char *filename);

enum bcmv_dump_action {
        BCMV_DUMP_FRAMEBUFFERS_BIT = 0x1,
};

void bcmv_dump_start(struct bcmv_device *device, enum bcmv_dump_action actions);
void bcmv_dump_finish(void);

void bcmv_dump_add_framebuffer(struct bcmv_cmd_buffer *cmd_buffer,
                               struct bcmv_framebuffer *fb);

static inline uint32_t
bcmv_get_subpass_id(const struct bcmv_cmd_state * const cmd_state)
{
        /* This function must be called from within a subpass. */
        assert(cmd_state->pass && cmd_state->subpass);

        const uint32_t subpass_id = cmd_state->subpass - cmd_state->pass->subpasses;

        /* The id of this subpass shouldn't exceed the number of subpasses in this
         * render pass minus 1.
         */
        assert(subpass_id < cmd_state->pass->subpass_count);
        return subpass_id;
}

#define BCMV_DEFINE_HANDLE_CASTS(__bcmv_type, __VkType)         \
                                                                \
        static inline struct __bcmv_type *                      \
        __bcmv_type ## _from_handle(__VkType _handle)           \
        {                                                       \
                return (struct __bcmv_type *) _handle;          \
        }                                                       \
                                                                \
        static inline __VkType                                  \
        __bcmv_type ## _to_handle(struct __bcmv_type *_obj)     \
        {                                                       \
                return (__VkType) _obj;                         \
        }

#define BCMV_DEFINE_NONDISP_HANDLE_CASTS(__bcmv_type, __VkType)         \
                                                                        \
        static inline struct __bcmv_type *                              \
        __bcmv_type ## _from_handle(__VkType _handle)                   \
        {                                                               \
                return (struct __bcmv_type *)(uintptr_t) _handle;       \
        }                                                               \
                                                                        \
        static inline __VkType                                          \
        __bcmv_type ## _to_handle(struct __bcmv_type *_obj)             \
        {                                                               \
                return (__VkType)(uintptr_t) _obj;                      \
        }

#define BCMV_FROM_HANDLE(__bcmv_type, __name, __handle)                 \
        struct __bcmv_type *__name = __bcmv_type ## _from_handle(__handle)

BCMV_DEFINE_HANDLE_CASTS(bcmv_cmd_buffer, VkCommandBuffer)
BCMV_DEFINE_HANDLE_CASTS(bcmv_device, VkDevice)
BCMV_DEFINE_HANDLE_CASTS(bcmv_instance, VkInstance)
BCMV_DEFINE_HANDLE_CASTS(bcmv_physical_device, VkPhysicalDevice)
BCMV_DEFINE_HANDLE_CASTS(bcmv_queue, VkQueue)

BCMV_DEFINE_NONDISP_HANDLE_CASTS(bcmv_cmd_pool, VkCommandPool)
BCMV_DEFINE_NONDISP_HANDLE_CASTS(bcmv_buffer, VkBuffer)
BCMV_DEFINE_NONDISP_HANDLE_CASTS(bcmv_buffer_view, VkBufferView)
BCMV_DEFINE_NONDISP_HANDLE_CASTS(bcmv_descriptor_pool, VkDescriptorPool)
BCMV_DEFINE_NONDISP_HANDLE_CASTS(bcmv_descriptor_set, VkDescriptorSet)
BCMV_DEFINE_NONDISP_HANDLE_CASTS(bcmv_descriptor_set_layout, VkDescriptorSetLayout)
BCMV_DEFINE_NONDISP_HANDLE_CASTS(bcmv_descriptor_update_template, VkDescriptorUpdateTemplateKHR)
BCMV_DEFINE_NONDISP_HANDLE_CASTS(bcmv_device_memory, VkDeviceMemory)
BCMV_DEFINE_NONDISP_HANDLE_CASTS(bcmv_fence, VkFence)
BCMV_DEFINE_NONDISP_HANDLE_CASTS(bcmv_event, VkEvent)
BCMV_DEFINE_NONDISP_HANDLE_CASTS(bcmv_framebuffer, VkFramebuffer)
BCMV_DEFINE_NONDISP_HANDLE_CASTS(bcmv_image, VkImage)
BCMV_DEFINE_NONDISP_HANDLE_CASTS(bcmv_image_view, VkImageView);
BCMV_DEFINE_NONDISP_HANDLE_CASTS(bcmv_pipeline_cache, VkPipelineCache)
BCMV_DEFINE_NONDISP_HANDLE_CASTS(bcmv_pipeline, VkPipeline)
BCMV_DEFINE_NONDISP_HANDLE_CASTS(bcmv_pipeline_layout, VkPipelineLayout)
BCMV_DEFINE_NONDISP_HANDLE_CASTS(bcmv_query_pool, VkQueryPool)
BCMV_DEFINE_NONDISP_HANDLE_CASTS(bcmv_render_pass, VkRenderPass)
BCMV_DEFINE_NONDISP_HANDLE_CASTS(bcmv_sampler, VkSampler)
BCMV_DEFINE_NONDISP_HANDLE_CASTS(bcmv_semaphore, VkSemaphore)
BCMV_DEFINE_NONDISP_HANDLE_CASTS(bcmv_shader_module, VkShaderModule)

/* Version-specific function declarations */
#ifdef v3dx
#  include "bcmv_v3dx.h"
#else
#  define v3dx(x) v3d33_##x
#  include "bcmv_v3dx.h"
#  undef v3dx
#endif

#endif /* BCMV_PRIVATE_H */
