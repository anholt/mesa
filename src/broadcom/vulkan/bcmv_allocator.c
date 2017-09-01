/*
 * Copyright © 2015 Intel Corporation
 * Copyright © 2017 Broadcom
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

#include <unistd.h>
#include <sys/mman.h>

#include "bcmv_private.h"

#ifdef HAVE_VALGRIND
#define VG_NOACCESS_READ(__ptr) ({                       \
   VALGRIND_MAKE_MEM_DEFINED((__ptr), sizeof(*(__ptr))); \
   __typeof(*(__ptr)) __val = *(__ptr);                  \
   VALGRIND_MAKE_MEM_NOACCESS((__ptr), sizeof(*(__ptr)));\
   __val;                                                \
})
#define VG_NOACCESS_WRITE(__ptr, __val) ({                  \
   VALGRIND_MAKE_MEM_UNDEFINED((__ptr), sizeof(*(__ptr)));  \
   *(__ptr) = (__val);                                      \
   VALGRIND_MAKE_MEM_NOACCESS((__ptr), sizeof(*(__ptr)));   \
})
#else
#define VG_NOACCESS_READ(__ptr) (*(__ptr))
#define VG_NOACCESS_WRITE(__ptr, __val) (*(__ptr) = (__val))
#endif

/* All pointers in the ptr_free_list are assumed to be page-aligned.  This
 * means that the bottom 12 bits should all be zero.
 */
#define PFL_COUNT(x) ((uintptr_t)(x) & 0xfff)
#define PFL_PTR(x) ((void *)((uintptr_t)(x) & ~(uintptr_t)0xfff))
#define PFL_PACK(ptr, count) ({                                         \
                        (void *)(((uintptr_t)(ptr) & ~(uintptr_t)0xfff) | ((count) & 0xfff)); \
                })

static inline uint32_t
ilog2_round_up(uint32_t value)
{
   assert(value != 0);
   return 32 - __builtin_clz(value - 1);
}

static inline uint32_t
round_to_power_of_two(uint32_t value)
{
   return 1 << ilog2_round_up(value);
}

static bool
bcmv_ptr_free_list_pop(void **list, void **elem)
{
        void *current = *list;
        while (PFL_PTR(current) != NULL) {
                void **next_ptr = PFL_PTR(current);
                void *new_ptr = VG_NOACCESS_READ(next_ptr);
                unsigned new_count = PFL_COUNT(current) + 1;
                void *new = PFL_PACK(new_ptr, new_count);
                void *old = __sync_val_compare_and_swap(list, current, new);
                if (old == current) {
                        *elem = PFL_PTR(current);
                        return true;
                }
                current = old;
        }

        return false;
}

static void
bcmv_ptr_free_list_push(void **list, void *elem)
{
        void *old, *current;
        void **next_ptr = elem;

        /* The pointer-based free list requires that the pointer be
         * page-aligned.  This is because we use the bottom 12 bits of the
         * pointer to store a counter to solve the ABA concurrency problem.
         */
        assert(((uintptr_t)elem & 0xfff) == 0);

        old = *list;
        do {
                current = old;
                VG_NOACCESS_WRITE(next_ptr, PFL_PTR(current));
                unsigned new_count = PFL_COUNT(current) + 1;
                void *new = PFL_PACK(elem, new_count);
                old = __sync_val_compare_and_swap(list, current, new);
        } while (old != current);
}

struct bo_pool_bo_link {
        struct bo_pool_bo_link *next;
        struct bcmv_bo bo;
};

void
bcmv_bo_pool_init(struct bcmv_bo_pool *pool, struct bcmv_device *device)
{
        pool->device = device;
        memset(pool->free_list, 0, sizeof(pool->free_list));

        VG(VALGRIND_CREATE_MEMPOOL(pool, 0, false));
}

void
bcmv_bo_pool_finish(struct bcmv_bo_pool *pool)
{
        for (unsigned i = 0; i < ARRAY_SIZE(pool->free_list); i++) {
                struct bo_pool_bo_link *link = PFL_PTR(pool->free_list[i]);
                while (link != NULL) {
                        struct bo_pool_bo_link link_copy = VG_NOACCESS_READ(link);

                        munmap(link_copy.bo.map, link_copy.bo.size);
                        bcmv_gem_close(pool->device, link_copy.bo.gem_handle);
                        link = link_copy.next;
                }
        }

        VG(VALGRIND_DESTROY_MEMPOOL(pool));
}

VkResult
bcmv_bo_pool_alloc(struct bcmv_bo_pool *pool, struct bcmv_bo *bo, uint32_t size)
{
        VkResult result;

        const unsigned size_log2 = size < 4096 ? 12 : ilog2_round_up(size);
        const unsigned pow2_size = 1 << size_log2;
        const unsigned bucket = size_log2 - 12;
        assert(bucket < ARRAY_SIZE(pool->free_list));

        void *next_free_void;
        if (bcmv_ptr_free_list_pop(&pool->free_list[bucket], &next_free_void)) {
                struct bo_pool_bo_link *next_free = next_free_void;
                *bo = VG_NOACCESS_READ(&next_free->bo);
                assert(bo->gem_handle);
                assert(bo->map == next_free);
                assert(size <= bo->size);

                VG(VALGRIND_MEMPOOL_ALLOC(pool, bo->map, size));

                return VK_SUCCESS;
        }

        struct bcmv_bo new_bo;

        result = bcmv_bo_init_new(&new_bo, pool->device, pow2_size);
        if (result != VK_SUCCESS)
                return result;

        assert(new_bo.size == pow2_size);

        new_bo.map = bcmv_gem_mmap(pool->device, new_bo.gem_handle, pow2_size);
        if (new_bo.map == MAP_FAILED) {
                bcmv_gem_close(pool->device, new_bo.gem_handle);
                return vk_error(VK_ERROR_MEMORY_MAP_FAILED);
        }

        *bo = new_bo;

        VG(VALGRIND_MEMPOOL_ALLOC(pool, bo->map, size));

        return VK_SUCCESS;
}

void
bcmv_bo_pool_free(struct bcmv_bo_pool *pool, const struct bcmv_bo *bo_in)
{
        /* Make a copy in case the bcmv_bo happens to be storred in the BO */
        struct bcmv_bo bo = *bo_in;

        VG(VALGRIND_MEMPOOL_FREE(pool, bo.map));

        struct bo_pool_bo_link *link = bo.map;
        VG_NOACCESS_WRITE(&link->bo, bo);

        assert(util_is_power_of_two(bo.size));
        const unsigned size_log2 = ilog2_round_up(bo.size);
        const unsigned bucket = size_log2 - 12;
        assert(bucket < ARRAY_SIZE(pool->free_list));

        bcmv_ptr_free_list_push(&pool->free_list[bucket], link);
}

struct bcmv_cached_bo {
        struct bcmv_bo bo;
        uint32_t refcount;
};

VkResult
bcmv_bo_cache_init(struct bcmv_bo_cache *cache)
{
        cache->bo_map = _mesa_hash_table_create(NULL, _mesa_hash_pointer,
                                                _mesa_key_pointer_equal);
        if (!cache->bo_map)
                return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);

        if (pthread_mutex_init(&cache->mutex, NULL)) {
                _mesa_hash_table_destroy(cache->bo_map, NULL);
                return vk_errorf(VK_ERROR_OUT_OF_HOST_MEMORY,
                                 "pthread_mutex_init failed: %m");
        }

        return VK_SUCCESS;
}

void
bcmv_bo_cache_finish(struct bcmv_bo_cache *cache)
{
        _mesa_hash_table_destroy(cache->bo_map, NULL);
        pthread_mutex_destroy(&cache->mutex);
}

static struct bcmv_cached_bo *
bcmv_bo_cache_lookup_locked(struct bcmv_bo_cache *cache, uint32_t gem_handle)
{
        struct hash_entry *entry =
                _mesa_hash_table_search(cache->bo_map,
                                        (const void *)(uintptr_t)gem_handle);
        if (!entry)
                return NULL;

        struct bcmv_cached_bo *bo = (struct bcmv_cached_bo *)entry->data;
        assert(bo->bo.gem_handle == gem_handle);

        return bo;
}

UNUSED static struct bcmv_bo *
bcmv_bo_cache_lookup(struct bcmv_bo_cache *cache, uint32_t gem_handle)
{
        pthread_mutex_lock(&cache->mutex);

        struct bcmv_cached_bo *bo = bcmv_bo_cache_lookup_locked(cache, gem_handle);

        pthread_mutex_unlock(&cache->mutex);

        return bo ? &bo->bo : NULL;
}

VkResult
bcmv_bo_cache_alloc(struct bcmv_device *device,
                    struct bcmv_bo_cache *cache,
                    uint64_t size, struct bcmv_bo **bo_out)
{
        struct bcmv_cached_bo *bo =
                vk_alloc(&device->alloc, sizeof(struct bcmv_cached_bo), 8,
                         VK_SYSTEM_ALLOCATION_SCOPE_OBJECT);
        if (!bo)
                return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);

        bo->refcount = 1;

        /* The kernel is going to give us whole pages anyway */
        size = align_u64(size, 4096);

        VkResult result = bcmv_bo_init_new(&bo->bo, device, size);
        if (result != VK_SUCCESS) {
                vk_free(&device->alloc, bo);
                return result;
        }

        assert(bo->bo.gem_handle);

        pthread_mutex_lock(&cache->mutex);

        _mesa_hash_table_insert(cache->bo_map,
                                (void *)(uintptr_t)bo->bo.gem_handle, bo);

        pthread_mutex_unlock(&cache->mutex);

        *bo_out = &bo->bo;

        return VK_SUCCESS;
}

VkResult
bcmv_bo_cache_import(struct bcmv_device *device,
                     struct bcmv_bo_cache *cache,
                     int fd, uint64_t size, struct bcmv_bo **bo_out)
{
        pthread_mutex_lock(&cache->mutex);

        /* The kernel is going to give us whole pages anyway */
        size = align_u64(size, 4096);

        uint32_t gem_handle = bcmv_gem_fd_to_handle(device, fd);
        if (!gem_handle) {
                pthread_mutex_unlock(&cache->mutex);
                return vk_error(VK_ERROR_INVALID_EXTERNAL_HANDLE_KHR);
        }

        struct bcmv_cached_bo *bo = bcmv_bo_cache_lookup_locked(cache, gem_handle);
        if (bo) {
                if (bo->bo.size != size) {
                        pthread_mutex_unlock(&cache->mutex);
                        return vk_error(VK_ERROR_INVALID_EXTERNAL_HANDLE_KHR);
                }
                __sync_fetch_and_add(&bo->refcount, 1);
        } else {
                /* For security purposes, we reject BO imports where the size
                 * does not match exactly.  This prevents a malicious client
                 * from passing a buffer to a trusted client, lying about the
                 * size, and telling the trusted client to try and texture
                 * from an image that goes out-of-bounds.  This sort of thing
                 * could lead to GPU hangs or worse in the trusted client.
                 * The trusted client can protect itself against this sort of
                 * attack but only if it can trust the buffer size.
                 */
                off_t import_size = lseek(fd, 0, SEEK_END);
                if (import_size == (off_t)-1 || import_size != size) {
                        bcmv_gem_close(device, gem_handle);
                        pthread_mutex_unlock(&cache->mutex);
                        return vk_error(VK_ERROR_INVALID_EXTERNAL_HANDLE_KHR);
                }

                bo = vk_alloc(&device->alloc, sizeof(struct bcmv_cached_bo), 8,
                              VK_SYSTEM_ALLOCATION_SCOPE_OBJECT);
                if (!bo) {
                        bcmv_gem_close(device, gem_handle);
                        pthread_mutex_unlock(&cache->mutex);
                        return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);
                }

                bo->refcount = 1;

                bcmv_bo_init(&bo->bo, gem_handle, size);

                _mesa_hash_table_insert(cache->bo_map,
                                        (void *)(uintptr_t)gem_handle, bo);
        }

        pthread_mutex_unlock(&cache->mutex);

        /* From the Vulkan spec:
         *
         *    "Importing memory from a file descriptor transfers ownership of
         *    the file descriptor from the application to the Vulkan
         *    implementation. The application must not perform any operations on
         *    the file descriptor after a successful import."
         *
         * If the import fails, we leave the file descriptor open.
         */
        close(fd);

        *bo_out = &bo->bo;

        return VK_SUCCESS;
}

VkResult
bcmv_bo_cache_export(struct bcmv_device *device,
                     struct bcmv_bo_cache *cache,
                     struct bcmv_bo *bo_in, int *fd_out)
{
        assert(bcmv_bo_cache_lookup(cache, bo_in->gem_handle) == bo_in);
        struct bcmv_cached_bo *bo = (struct bcmv_cached_bo *)bo_in;

        int fd = bcmv_gem_handle_to_fd(device, bo->bo.gem_handle);
        if (fd < 0)
                return vk_error(VK_ERROR_TOO_MANY_OBJECTS);

        *fd_out = fd;

        return VK_SUCCESS;
}

static bool
atomic_dec_not_one(uint32_t *counter)
{
        uint32_t old, val;

        val = *counter;
        while (1) {
                if (val == 1)
                        return false;

                old = __sync_val_compare_and_swap(counter, val, val - 1);
                if (old == val)
                        return true;

                val = old;
        }
}

void
bcmv_bo_cache_release(struct bcmv_device *device,
                      struct bcmv_bo_cache *cache,
                      struct bcmv_bo *bo_in)
{
        assert(bcmv_bo_cache_lookup(cache, bo_in->gem_handle) == bo_in);
        struct bcmv_cached_bo *bo = (struct bcmv_cached_bo *)bo_in;

        /* Try to decrement the counter but don't go below one.  If this succeeds
         * then the refcount has been decremented and we are not the last
         * reference.
         */
        if (atomic_dec_not_one(&bo->refcount))
                return;

        pthread_mutex_lock(&cache->mutex);

        /* We are probably the last reference since our attempt to decrement above
         * failed.  However, we can't actually know until we are inside the mutex.
         * Otherwise, someone could import the BO between the decrement and our
         * taking the mutex.
         */
        if (unlikely(__sync_sub_and_fetch(&bo->refcount, 1) > 0)) {
                /* Turns out we're not the last reference.  Unlock and bail. */
                pthread_mutex_unlock(&cache->mutex);
                return;
        }

        struct hash_entry *entry =
                _mesa_hash_table_search(cache->bo_map,
                                        (const void *)(uintptr_t)bo->bo.gem_handle);
        assert(entry);
        _mesa_hash_table_remove(cache->bo_map, entry);

        if (bo->bo.map)
                munmap(bo->bo.map, bo->bo.size);

        bcmv_gem_close(device, bo->bo.gem_handle);

        /* Don't unlock until we've actually closed the BO.  The whole point of
         * the BO cache is to ensure that we correctly handle races with creating
         * and releasing GEM handles and we don't want to let someone import the BO
         * again between mutex unlock and closing the GEM handle.
         */
        pthread_mutex_unlock(&cache->mutex);

        vk_free(&device->alloc, bo);
}
