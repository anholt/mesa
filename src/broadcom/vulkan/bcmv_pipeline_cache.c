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

#include "util/hash_table.h"
#include "util/debug.h"
#include "bcmv_private.h"
#include "broadcom/compiler/v3d_compiler.h"

static size_t
bcmv_shader_bin_size(uint32_t prog_data_size,
                     const struct v3d_prog_data *prog_data,
                     uint32_t key_size,
                     uint32_t surface_count, uint32_t sampler_count)
{
        const uint32_t binding_data_size =
                (surface_count + sampler_count) * sizeof(struct bcmv_pipeline_binding);

        return align_u32(sizeof(struct bcmv_shader_bin), 8) +
                align_u32(prog_data_size, 8) +
                align_u32(prog_data->uniforms.count *
                          sizeof(enum quniform_contents), 8) +
                align_u32(prog_data->uniforms.count * sizeof(uint32_t), 8) +
                align_u32(sizeof(uint32_t) + key_size, 8) +
                align_u32(binding_data_size, 8);
}

static void
bcmv_shader_bin_serialize(void **data, void **new_prog, const void *old_prog,
                          uint32_t size)
{
        *new_prog = *data;
        memcpy(*new_prog, old_prog, size);
        (*data) += align_u32(size, 8);
}

struct bcmv_shader_bin *
bcmv_shader_bin_create(struct bcmv_device *device,
                       const void *key_data, uint32_t key_size,
                       const void *kernel_data, uint32_t kernel_size,
                       const struct v3d_prog_data *prog_data,
                       uint32_t prog_data_size,
                       const enum quniform_contents *uniform_contents,
                       const uint32_t *uniform_data,
                       const struct bcmv_pipeline_bind_map *bind_map)
{
        VkResult result;

        const size_t size =
                bcmv_shader_bin_size(prog_data_size, prog_data,
                                     key_size,
                                     bind_map->surface_count, bind_map->sampler_count);

        struct bcmv_shader_bin *shader =
                vk_alloc(&device->alloc, size, 8, VK_SYSTEM_ALLOCATION_SCOPE_DEVICE);
        if (!shader)
                return NULL;

        shader->ref_cnt = 1;

        result = bcmv_bo_pool_alloc(&device->batch_bo_pool, &shader->bo, kernel_size);
        if (result != VK_SUCCESS) {
                vk_free(&device->alloc, shader);
                return NULL;
        }

        memcpy(shader->bo.map, kernel_data, kernel_size);
        shader->kernel_size = kernel_size;
        shader->bind_map = *bind_map;
        shader->prog_data_size = prog_data_size;

        /* Now we fill out the floating data at the end */
        void *data = shader;
        data += align_u32(sizeof(struct bcmv_shader_bin), 8);

        bcmv_shader_bin_serialize(&data, (void **)&shader->prog_data, prog_data,
                                  prog_data_size);

        bcmv_shader_bin_serialize(&data,
                                  (void **)&shader->prog_data->uniforms.contents,
                                  uniform_contents,
                                  sizeof(*uniform_contents) *
                                  prog_data->uniforms.count);

        bcmv_shader_bin_serialize(&data,
                                  (void **)&shader->prog_data->uniforms.data,
                                  uniform_data,
                                  sizeof(*uniform_data) *
                                  prog_data->uniforms.count);

        shader->key = data;
        struct bcmv_shader_bin_key *key = data;
        key->size = key_size;
        memcpy(key->data, key_data, key_size);
        data += align_u32(sizeof(*key) + key_size, 8);

        shader->bind_map.surface_to_descriptor = data;
        memcpy(data, bind_map->surface_to_descriptor,
               bind_map->surface_count * sizeof(struct bcmv_pipeline_binding));
        data += bind_map->surface_count * sizeof(struct bcmv_pipeline_binding);

        shader->bind_map.sampler_to_descriptor = data;
        memcpy(data, bind_map->sampler_to_descriptor,
               bind_map->sampler_count * sizeof(struct bcmv_pipeline_binding));

        return shader;
}

void
bcmv_shader_bin_destroy(struct bcmv_device *device,
                        struct bcmv_shader_bin *shader)
{
        assert(shader->ref_cnt == 0);
        bcmv_bo_pool_free(&device->batch_bo_pool, &shader->bo);
        vk_free(&device->alloc, shader);
}

static size_t
bcmv_shader_bin_data_size(const struct bcmv_shader_bin *shader)
{
        return bcmv_shader_bin_size(shader->prog_data_size,
                                    shader->prog_data,
                                    shader->key->size,
                                    shader->bind_map.surface_count,
                                    shader->bind_map.sampler_count) +
                align_u32(shader->kernel_size, 8);
}

static void
bcmv_shader_bin_write_data(const struct bcmv_shader_bin *shader, void *data)
{
        size_t struct_size =
                bcmv_shader_bin_size(shader->prog_data_size,
                                     shader->prog_data, shader->key->size,
                                     shader->bind_map.surface_count,
                                     shader->bind_map.sampler_count);

        memcpy(data, shader, struct_size);
        data += struct_size;

        memcpy(data, shader->bo.map, shader->kernel_size);
}

/* Remaining work:
 *
 * - Compact binding table layout so it's tight and not dependent on
 *   descriptor set layout.
 *
 * - Review prog_data struct for size and cacheability: struct
 *   v3d_prog_data has binding_table which uses a lot of uint32_t for 8
 *   bit quantities etc; param, pull_param, and image_params are pointers, we
 *   just need the compation map. use bit fields for all bools, eg
 *   dual_src_blend.
 */

static uint32_t
shader_bin_key_hash_func(const void *void_key)
{
        const struct bcmv_shader_bin_key *key = void_key;
        return _mesa_hash_data(key->data, key->size);
}

static bool
shader_bin_key_compare_func(const void *void_a, const void *void_b)
{
        const struct bcmv_shader_bin_key *a = void_a, *b = void_b;
        if (a->size != b->size)
                return false;

        return memcmp(a->data, b->data, a->size) == 0;
}

void
bcmv_pipeline_cache_init(struct bcmv_pipeline_cache *cache,
                         struct bcmv_device *device,
                         bool cache_enabled)
{
        cache->device = device;
        pthread_mutex_init(&cache->mutex, NULL);

        if (cache_enabled) {
                cache->cache = _mesa_hash_table_create(NULL, shader_bin_key_hash_func,
                                                       shader_bin_key_compare_func);
        } else {
                cache->cache = NULL;
        }
}

void
bcmv_pipeline_cache_finish(struct bcmv_pipeline_cache *cache)
{
        pthread_mutex_destroy(&cache->mutex);

        if (cache->cache) {
                /* This is a bit unfortunate.  In order to keep things from randomly
                 * going away, the shader cache has to hold a reference to all shader
                 * binaries it contains.  We unref them when we destroy the cache.
                 */
                struct hash_entry *entry;
                hash_table_foreach(cache->cache, entry)
                        bcmv_shader_bin_unref(cache->device, entry->data);

                _mesa_hash_table_destroy(cache->cache, NULL);
        }
}

static struct bcmv_shader_bin *
bcmv_pipeline_cache_search_locked(struct bcmv_pipeline_cache *cache,
                                  const void *key_data, uint32_t key_size)
{
        uint32_t vla[1 + DIV_ROUND_UP(key_size, sizeof(uint32_t))];
        struct bcmv_shader_bin_key *key = (void *)vla;
        key->size = key_size;
        memcpy(key->data, key_data, key_size);

        struct hash_entry *entry = _mesa_hash_table_search(cache->cache, key);
        if (entry)
                return entry->data;
        else
                return NULL;
}

struct bcmv_shader_bin *
bcmv_pipeline_cache_search(struct bcmv_pipeline_cache *cache,
                           const void *key_data, uint32_t key_size)
{
        if (!cache->cache)
                return NULL;

        pthread_mutex_lock(&cache->mutex);

        struct bcmv_shader_bin *shader =
                bcmv_pipeline_cache_search_locked(cache, key_data, key_size);

        pthread_mutex_unlock(&cache->mutex);

        /* We increment refcount before handing it to the caller */
        if (shader)
                bcmv_shader_bin_ref(shader);

        return shader;
}

static struct bcmv_shader_bin *
bcmv_pipeline_cache_add_shader(struct bcmv_pipeline_cache *cache,
                               const void *key_data, uint32_t key_size,
                               const void *kernel_data, uint32_t kernel_size,
                               const struct v3d_prog_data *prog_data,
                               uint32_t prog_data_size,
                               const enum quniform_contents *uniform_contents,
                               const uint32_t *uniform_data,
                               const struct bcmv_pipeline_bind_map *bind_map)
{
        struct bcmv_shader_bin *shader =
                bcmv_pipeline_cache_search_locked(cache, key_data, key_size);
        if (shader)
                return shader;

        struct bcmv_shader_bin *bin =
                bcmv_shader_bin_create(cache->device, key_data, key_size,
                                       kernel_data, kernel_size,
                                       prog_data, prog_data_size,
                                       uniform_contents, uniform_data,
                                       bind_map);
        if (!bin)
                return NULL;

        _mesa_hash_table_insert(cache->cache, bin->key, bin);

        return bin;
}

struct bcmv_shader_bin *
bcmv_pipeline_cache_upload_kernel(struct bcmv_pipeline_cache *cache,
                                  const void *key_data, uint32_t key_size,
                                  const void *kernel_data, uint32_t kernel_size,
                                  const struct v3d_prog_data *prog_data,
                                  uint32_t prog_data_size,
                                  enum quniform_contents *uniform_contents,
                                  uint32_t *uniform_data,
                                  const struct bcmv_pipeline_bind_map *bind_map)
{
        if (cache->cache) {
                pthread_mutex_lock(&cache->mutex);

                struct bcmv_shader_bin *bin =
                        bcmv_pipeline_cache_add_shader(cache, key_data, key_size,
                                                       kernel_data, kernel_size,
                                                       prog_data, prog_data_size,
                                                       uniform_contents,
                                                       uniform_data,
                                                       bind_map);

                pthread_mutex_unlock(&cache->mutex);

                /* We increment refcount before handing it to the caller */
                if (bin)
                        bcmv_shader_bin_ref(bin);

                return bin;
        } else {
                /* In this case, we're not caching it so the caller owns it
                 * entirely
                 */
                return bcmv_shader_bin_create(cache->device, key_data, key_size,
                                              kernel_data, kernel_size,
                                              prog_data, prog_data_size,
                                              uniform_contents, uniform_data,
                                              bind_map);
        }
}

struct cache_header {
        uint32_t header_size;
        uint32_t header_version;
        uint32_t v3d_ver;
        uint32_t uifcfg;
        uint8_t  uuid[VK_UUID_SIZE];
};

static void
bcmv_pipeline_cache_load(struct bcmv_pipeline_cache *cache,
                         const void *data, size_t size)
{
        struct bcmv_device *device = cache->device;
        struct bcmv_physical_device *pdevice = &device->instance->physicalDevice;
        struct cache_header header;

        if (cache->cache == NULL)
                return;

        if (size < sizeof(header))
                return;
        memcpy(&header, data, sizeof(header));
        if (header.header_size < sizeof(header))
                return;
        if (header.header_version != VK_PIPELINE_CACHE_HEADER_VERSION_ONE)
                return;
        if (header.v3d_ver != device->info.ver)
                return;
        if (header.uifcfg != 0 /* XXX device->uifcfg */)
                return;
        if (memcmp(header.uuid, pdevice->pipeline_cache_uuid, VK_UUID_SIZE) != 0)
                return;

        const void *end = data + size;
        const void *p = data + header.header_size;

        /* Count is the total number of valid entries */
        uint32_t count;
        if (p + sizeof(count) >= end)
                return;
        memcpy(&count, p, sizeof(count));
        p += align_u32(sizeof(count), 8);

        for (uint32_t i = 0; i < count; i++) {
                struct bcmv_shader_bin bin;
                if (p + sizeof(bin) > end)
                        break;
                memcpy(&bin, p, sizeof(bin));
                p += align_u32(sizeof(struct bcmv_shader_bin), 8);

                const struct v3d_prog_data *prog_data = p;
                p += align_u32(bin.prog_data_size, 8);
                if (p > end)
                        break;

                uint32_t contents_size = (prog_data->uniforms.count *
                                          sizeof(*prog_data->uniforms.contents));
                const void *uniform_contents = p;
                p += align_u32(contents_size, 8);

                uint32_t data_size = (prog_data->uniforms.count *
                                      sizeof(*prog_data->uniforms.data));
                const void *uniform_data = p;
                p += align_u32(data_size, 8);

                struct bcmv_shader_bin_key key;
                if (p + sizeof(key) > end)
                        break;
                memcpy(&key, p, sizeof(key));
                const void *key_data = p + sizeof(key);
                p += align_u32(sizeof(key) + key.size, 8);

                /* We're going to memcpy this so getting rid of const is fine */
                struct bcmv_pipeline_binding *bindings = (void *)p;
                p += align_u32((bin.bind_map.surface_count + bin.bind_map.sampler_count) *
                               sizeof(struct bcmv_pipeline_binding), 8);
                bin.bind_map.surface_to_descriptor = bindings;
                bin.bind_map.sampler_to_descriptor = bindings + bin.bind_map.surface_count;

                const void *kernel_data = p;
                p += align_u32(bin.kernel_size, 8);

                if (p > end)
                        break;

                bcmv_pipeline_cache_add_shader(cache, key_data, key.size,
                                               kernel_data, bin.kernel_size,
                                               prog_data, bin.prog_data_size,
                                               uniform_contents, uniform_data,
                                               &bin.bind_map);
        }
}

static bool
pipeline_cache_enabled()
{
        static int enabled = -1;
        if (enabled < 0)
                enabled = env_var_as_boolean("BCMV_ENABLE_PIPELINE_CACHE", true);
        return enabled;
}

VkResult bcmv_CreatePipelineCache(
        VkDevice                                    _device,
        const VkPipelineCacheCreateInfo*            pCreateInfo,
        const VkAllocationCallbacks*                pAllocator,
        VkPipelineCache*                            pPipelineCache)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        struct bcmv_pipeline_cache *cache;

        assert(pCreateInfo->sType == VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO);
        assert(pCreateInfo->flags == 0);

        cache = vk_alloc2(&device->alloc, pAllocator,
                          sizeof(*cache), 8,
                          VK_SYSTEM_ALLOCATION_SCOPE_OBJECT);
        if (cache == NULL)
                return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);

        bcmv_pipeline_cache_init(cache, device, pipeline_cache_enabled());

        if (pCreateInfo->initialDataSize > 0)
                bcmv_pipeline_cache_load(cache,
                                         pCreateInfo->pInitialData,
                                         pCreateInfo->initialDataSize);

        *pPipelineCache = bcmv_pipeline_cache_to_handle(cache);

        return VK_SUCCESS;
}

void bcmv_DestroyPipelineCache(
        VkDevice                                    _device,
        VkPipelineCache                             _cache,
        const VkAllocationCallbacks*                pAllocator)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        BCMV_FROM_HANDLE(bcmv_pipeline_cache, cache, _cache);

        if (!cache)
                return;

        bcmv_pipeline_cache_finish(cache);

        vk_free2(&device->alloc, pAllocator, cache);
}

VkResult bcmv_GetPipelineCacheData(
        VkDevice                                    _device,
        VkPipelineCache                             _cache,
        size_t*                                     pDataSize,
        void*                                       pData)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        BCMV_FROM_HANDLE(bcmv_pipeline_cache, cache, _cache);
        struct bcmv_physical_device *pdevice = &device->instance->physicalDevice;
        struct cache_header *header;

        if (pData == NULL) {
                size_t size = align_u32(sizeof(*header), 8) +
                        align_u32(sizeof(uint32_t), 8);

                if (cache->cache) {
                        struct hash_entry *entry;
                        hash_table_foreach(cache->cache, entry)
                                size += bcmv_shader_bin_data_size(entry->data);
                }

                *pDataSize = size;
                return VK_SUCCESS;
        }

        if (*pDataSize < sizeof(*header)) {
                *pDataSize = 0;
                return VK_INCOMPLETE;
        }

        void *p = pData, *end = pData + *pDataSize;
        header = p;
        header->header_size = sizeof(*header);
        header->header_version = VK_PIPELINE_CACHE_HEADER_VERSION_ONE;
        header->v3d_ver = device->info.ver;
        header->uifcfg = 0; /* XXX */
        memcpy(header->uuid, pdevice->pipeline_cache_uuid, VK_UUID_SIZE);
        p += align_u32(header->header_size, 8);

        uint32_t *count = p;
        p += align_u32(sizeof(*count), 8);
        *count = 0;

        VkResult result = VK_SUCCESS;
        if (cache->cache) {
                struct hash_entry *entry;
                hash_table_foreach(cache->cache, entry) {
                        struct bcmv_shader_bin *shader = entry->data;
                        size_t data_size = bcmv_shader_bin_data_size(entry->data);
                        if (p + data_size > end) {
                                result = VK_INCOMPLETE;
                                break;
                        }

                        bcmv_shader_bin_write_data(shader, p);
                        p += data_size;

                        (*count)++;
                }
        }

        *pDataSize = p - pData;

        return result;
}

VkResult bcmv_MergePipelineCaches(
        VkDevice                                    _device,
        VkPipelineCache                             destCache,
        uint32_t                                    srcCacheCount,
        const VkPipelineCache*                      pSrcCaches)
{
        BCMV_FROM_HANDLE(bcmv_pipeline_cache, dst, destCache);

        if (!dst->cache)
                return VK_SUCCESS;

        for (uint32_t i = 0; i < srcCacheCount; i++) {
                BCMV_FROM_HANDLE(bcmv_pipeline_cache, src, pSrcCaches[i]);
                if (!src->cache)
                        continue;

                struct hash_entry *entry;
                hash_table_foreach(src->cache, entry) {
                        struct bcmv_shader_bin *bin = entry->data;
                        assert(bin);

                        if (_mesa_hash_table_search(dst->cache, bin->key))
                                continue;

                        bcmv_shader_bin_ref(bin);
                        _mesa_hash_table_insert(dst->cache, bin->key, bin);
                }
        }

        return VK_SUCCESS;
}
