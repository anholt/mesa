/*
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

#include <assert.h>
#include <stdbool.h>
#include <unistd.h>

#include "bcmv_private.h"
#include "cle/v3d_packet_v33_pack.h"

static void bcmv_cl_init(struct bcmv_batch *batch, struct bcmv_cl *cl)
{
        cl->batch = batch;
        cl->base =  NULL;
        cl->next = NULL;
        cl->bo = NULL;
        cl->size = 0;
}

static void bcmv_cl_free(struct bcmv_cl *cl)
{
        /* XXX */
}

VkResult bcmv_batch_init(struct bcmv_cmd_buffer *cmd_buffer)
{
        struct bcmv_batch *batch = &cmd_buffer->batch;

        bcmv_cl_init(batch, &batch->bcl);
        bcmv_cl_init(batch, &batch->rcl);
        bcmv_cl_init(batch, &batch->indirect);

        return VK_SUCCESS;
}

void bcmv_batch_free(struct bcmv_batch *batch)
{
        bcmv_cl_free(&batch->bcl);
        bcmv_cl_free(&batch->rcl);
        bcmv_cl_free(&batch->indirect);
}

static void *
bcmv_get_sl_space_must_succeed(struct bcmv_cl *cl, uint32_t size)
{
        assert(bcmv_cl_offset(cl) + size <= cl->size);
        return cl->next;
}

/* Called when emitting state to the RCL or BCL would overflow the buffer, to
 * allocate more space and branch to the new buffer.
 */
void *
bcmv_get_cl_space_chained(struct bcmv_cl *cl, uint32_t size)
{
        struct bcmv_cmd_buffer *cmd_buffer = cl->batch->cmd_buffer;
        VkResult result;

        if (bcmv_cl_offset(cl) + size + __bcmv_packet_length(V3D33_BRANCH) <= cl->size)
                return cl->next;

        struct bcmv_bo *new_bo = vk_alloc(&cmd_buffer->pool->alloc,
                                          sizeof(*new_bo),
                                          8, VK_SYSTEM_ALLOCATION_SCOPE_OBJECT);
        if (!new_bo)
                return NULL;

        result = bcmv_bo_pool_alloc(&cmd_buffer->device->batch_bo_pool, new_bo,
                                    4096);
        if (result != VK_SUCCESS)
                goto fail_alloc;

        assert(size <= new_bo->size);

        /* Chain to the new BO from the old one. */
        if (cl->bo) {
                bcmv_cl_emit(cl->batch, cl, V3D33_BRANCH, branch,
                             bcmv_get_sl_space_must_succeed) {
                        branch.address = bcmv_address(new_bo, 0);
                }
                // XXX vc5_bo_unreference(&cl->bo);
        } else {
                /* Root the first RCL/BCL BO in the job. */
                // XXX vc5_job_add_bo(cl->job, cl->bo);
        }

        cl->bo = new_bo;
        cl->base = cl->bo->map;
        cl->size = cl->bo->size;
        cl->next = cl->base;

        return cl->next;

 fail_alloc:
        vk_free(&cmd_buffer->pool->alloc, new_bo);
        return NULL;
}

/* Called when emitting state to the indirect buffer, to allocate a new BO and
 * return from it.
 */
void *
bcmv_get_cl_space_realloc(struct bcmv_cl *cl, uint32_t size)
{
        struct bcmv_cmd_buffer *cmd_buffer = cl->batch->cmd_buffer;
        VkResult result;

        if (bcmv_cl_offset(cl) + size <= cl->size)
                return cl->next;

        struct bcmv_bo *new_bo = vk_alloc(&cmd_buffer->pool->alloc,
                                          sizeof(*new_bo),
                                          8, VK_SYSTEM_ALLOCATION_SCOPE_OBJECT);
        if (!new_bo)
                return NULL;

        result = bcmv_bo_pool_alloc(&cmd_buffer->device->batch_bo_pool, new_bo,
                                    4096);
        if (result != VK_SUCCESS)
                goto fail_alloc;

        assert(size <= new_bo->size);

        cl->bo = new_bo;
        cl->base = cl->bo->map;
        cl->size = cl->bo->size;
        cl->next = cl->base;

        return cl->next;

 fail_alloc:
        vk_free(&cmd_buffer->pool->alloc, new_bo);
        return NULL;
}

/* Called by the pack header to record a reference to address.bo in the batch.
 */
void bcmv_batch_emit_reloc(struct bcmv_batch *batch,
                           const struct bcmv_address *address)
{
        /* XXX */
}

struct bcmv_submit {
        struct drm_vc5_submit_cl submit;

        uint32_t *objects;
        uint32_t bo_count;
        /* Allocated length of the 'objects' and 'bos' arrays */
        uint32_t array_length;
        struct bcmv_bo **bos;

        uint32_t fence_count;
        uint32_t fence_array_length;
#if 0 /* XXX */
        struct drm_i915_gem_exec_fence *fences;
#endif
        struct bcmv_syncobj **syncobjs;
};

static void
bcmv_submit_init(struct bcmv_submit *exec)
{
        memset(exec, 0, sizeof(*exec));
}

static void
bcmv_submit_finish(struct bcmv_submit *exec,
                    const VkAllocationCallbacks *alloc)
{
        vk_free(alloc, exec->objects);
        vk_free(alloc, exec->bos);
        // XXX vk_free(alloc, exec->fences);
        vk_free(alloc, exec->syncobjs);
}

static VkResult
bcmv_submit_add_bo(struct bcmv_submit *exec,
                    struct bcmv_bo *bo,
                    struct bcmv_reloc_list *relocs,
                    uint32_t extra_flags,
                    const VkAllocationCallbacks *alloc)
{
#if 0 /* XXX */
        struct drm_i915_gem_exec_object2 *obj = NULL;

        if (bo->index < exec->bo_count && exec->bos[bo->index] == bo)
                obj = &exec->objects[bo->index];

        if (obj == NULL) {
                /* We've never seen this one before.  Add it to the list and assign
                 * an id that we can use later.
                 */
                if (exec->bo_count >= exec->array_length) {
                        uint32_t new_len = exec->objects ? exec->array_length * 2 : 64;

                        struct drm_i915_gem_exec_object2 *new_objects =
                                vk_alloc(alloc, new_len * sizeof(*new_objects),
                                         8, VK_SYSTEM_ALLOCATION_SCOPE_COMMAND);
                        if (new_objects == NULL)
                                return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);

                        struct bcmv_bo **new_bos =
                                vk_alloc(alloc, new_len * sizeof(*new_bos),
                                         8, VK_SYSTEM_ALLOCATION_SCOPE_COMMAND);
                        if (new_bos == NULL) {
                                vk_free(alloc, new_objects);
                                return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);
                        }

                        if (exec->objects) {
                                memcpy(new_objects, exec->objects,
                                       exec->bo_count * sizeof(*new_objects));
                                memcpy(new_bos, exec->bos,
                                       exec->bo_count * sizeof(*new_bos));
                        }

                        vk_free(alloc, exec->objects);
                        vk_free(alloc, exec->bos);

                        exec->objects = new_objects;
                        exec->bos = new_bos;
                        exec->array_length = new_len;
                }

                assert(exec->bo_count < exec->array_length);

                bo->index = exec->bo_count++;
                obj = &exec->objects[bo->index];
                exec->bos[bo->index] = bo;

                obj->handle = bo->gem_handle;
                obj->relocation_count = 0;
                obj->relocs_ptr = 0;
                obj->alignment = 0;
                obj->offset = bo->offset;
                obj->flags = bo->flags | extra_flags;
                obj->rsvd1 = 0;
                obj->rsvd2 = 0;
        }

        if (relocs != NULL && obj->relocation_count == 0) {
                /* This is the first time we've ever seen a list of relocations for
                 * this BO.  Go ahead and set the relocations and then walk the list
                 * of relocations and add them all.
                 */
                obj->relocation_count = relocs->num_relocs;
                obj->relocs_ptr = (uintptr_t) relocs->relocs;

                for (size_t i = 0; i < relocs->num_relocs; i++) {
                        VkResult result;

                        /* A quick sanity check on relocations */
                        assert(relocs->relocs[i].offset < bo->size);
                        result = bcmv_submit_add_bo(exec, relocs->reloc_bos[i], NULL,
                                                     extra_flags, alloc);

                        if (result != VK_SUCCESS)
                                return result;
                }
        }
#endif

        return VK_SUCCESS;
}

static VkResult
bcmv_submit_add_syncobj(struct bcmv_submit *exec,
                        uint32_t handle, uint32_t flags,
                        const VkAllocationCallbacks *alloc)
{
        assert(flags != 0);
#if 0 /* XXX */

        if (exec->fence_count >= exec->fence_array_length) {
                uint32_t new_len = MAX2(exec->fence_array_length * 2, 64);

                exec->fences = vk_realloc(alloc, exec->fences,
                                          new_len * sizeof(*exec->fences),
                                          8, VK_SYSTEM_ALLOCATION_SCOPE_COMMAND);
                if (exec->fences == NULL)
                        return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);

                exec->fence_array_length = new_len;
        }

        exec->fences[exec->fence_count] = (struct drm_i915_gem_exec_fence) {
                .handle = handle,
                .flags = flags,
        };

        exec->fence_count++;
#endif

        return VK_SUCCESS;
}

static VkResult
setup_submit_for_cmd_buffer(struct bcmv_submit *submit,
                            struct bcmv_cmd_buffer *cmd_buffer)
{
        submit->submit = (struct drm_vc5_submit_cl) {
                .bo_handles = (uintptr_t) submit->objects,
                .bo_handle_count = submit->bo_count,

                /* XXX */
                .bcl_start = 0,
                .bcl_end = 0,
                .rcl_start = 0,
                .rcl_end = 0,
        };

        return VK_SUCCESS;
}

static VkResult
setup_empty_submit(struct bcmv_submit *submit, struct bcmv_device *device)
{
        VkResult result = bcmv_submit_add_bo(submit, &device->trivial_batch_bo,
                                             NULL, 0, &device->alloc);
        if (result != VK_SUCCESS)
                return result;

        submit->submit = (struct drm_vc5_submit_cl) {
                .bo_handles = (uintptr_t) submit->objects,
                .bo_handle_count = submit->bo_count,

                /* XXX */
                .bcl_start = 0,
                .bcl_end = 0,
                .rcl_start = 0,
                .rcl_end = 0,
        };

        return VK_SUCCESS;
}

VkResult
bcmv_cmd_buffer_submit(struct bcmv_device *device,
                       struct bcmv_cmd_buffer *cmd_buffer,
                       const VkSemaphore *in_semaphores,
                       uint32_t num_in_semaphores,
                       const VkSemaphore *out_semaphores,
                       uint32_t num_out_semaphores,
                       VkFence _fence)
{
        BCMV_FROM_HANDLE(bcmv_fence, fence, _fence);

        struct bcmv_submit submit;
        bcmv_submit_init(&submit);

        int in_fence = -1;
        VkResult result = VK_SUCCESS;
        for (uint32_t i = 0; i < num_in_semaphores; i++) {
                BCMV_FROM_HANDLE(bcmv_semaphore, semaphore, in_semaphores[i]);
                struct bcmv_semaphore_impl *impl =
                        semaphore->temporary.type != BCMV_SEMAPHORE_TYPE_NONE ?
                        &semaphore->temporary : &semaphore->permanent;

                switch (impl->type) {
                case BCMV_SEMAPHORE_TYPE_SYNC_FILE:
                        if (in_fence == -1) {
                                in_fence = impl->fd;
                        } else {
                                int merge = bcmv_gem_sync_file_merge(device, in_fence, impl->fd);
                                if (merge == -1)
                                        return vk_error(VK_ERROR_INVALID_EXTERNAL_HANDLE_KHR);

                                close(impl->fd);
                                close(in_fence);
                                in_fence = merge;
                        }

                        impl->fd = -1;
                        break;

                case BCMV_SEMAPHORE_TYPE_DRM_SYNCOBJ:
#if 0 /* XXX */
                        result = bcmv_submit_add_syncobj(&submit, impl->syncobj,
                                                         I915_EXEC_FENCE_WAIT,
                                                         &device->alloc);
                        if (result != VK_SUCCESS)
                                return result;
#endif
                        break;

                default:
                        break;
                }
        }

        bool need_out_fence = false;
        for (uint32_t i = 0; i < num_out_semaphores; i++) {
                BCMV_FROM_HANDLE(bcmv_semaphore, semaphore, out_semaphores[i]);

                /* Under most circumstances, out fences won't be temporary.
                 * However, the spec does allow it for opaque_fd.  From the
                 * Vulkan 1.0.53 spec:
                 *
                 *    "If the import is temporary, the implementation must
                 *     restore the semaphore to its prior permanent state
                 *     after submitting the next semaphore wait operation."
                 *
                 * The spec says nothing whatsoever about signal operations on
                 * temporarily imported semaphores so it appears they are
                 * allowed.  There are also CTS tests that require this to
                 * work.
                 */
                struct bcmv_semaphore_impl *impl =
                        semaphore->temporary.type != BCMV_SEMAPHORE_TYPE_NONE ?
                        &semaphore->temporary : &semaphore->permanent;

                switch (impl->type) {
                case BCMV_SEMAPHORE_TYPE_SYNC_FILE:
                        need_out_fence = true;
                        break;

                case BCMV_SEMAPHORE_TYPE_DRM_SYNCOBJ:
#if 0 /* XXX */
                        result = bcmv_submit_add_syncobj(&submit, impl->syncobj,
                                                         I915_EXEC_FENCE_SIGNAL,
                                                         &device->alloc);
                        if (result != VK_SUCCESS)
                                return result;
#endif
                        break;

                default:
                        break;
                }
        }

        if (fence) {
                /* Under most circumstances, out fences won't be temporary.
                 * However, the spec does allow it for opaque_fd.  From the
                 * Vulkan 1.0.53 spec:
                 *
                 *    "If the import is temporary, the implementation must
                 *     restore the semaphore to its prior permanent state
                 *     after submitting the next semaphore wait operation."
                 *
                 * The spec says nothing whatsoever about signal operations on
                 * temporarily imported semaphores so it appears they are
                 * allowed.  There are also CTS tests that require this to
                 * work.
                 */
#if 0 /* XXX */
                struct bcmv_fence_impl *impl =
                        fence->temporary.type != BCMV_FENCE_TYPE_NONE ?
                        &fence->temporary : &fence->permanent;

                result = bcmv_submit_add_syncobj(&submit, impl->syncobj,
                                                  I915_EXEC_FENCE_SIGNAL,
                                                  &device->alloc);
                if (result != VK_SUCCESS)
                        return result;
#endif
        }

        if (cmd_buffer)
                result = setup_submit_for_cmd_buffer(&submit, cmd_buffer);
        else
                result = setup_empty_submit(&submit, device);

        if (result != VK_SUCCESS)
                return result;

#if 0 /* XXX */
        if (submit.fence_count > 0) {
                assert(device->instance->physicalDevice.has_syncobj);
                submit.submit.flags |= I915_EXEC_FENCE_ARRAY;
                submit.submit.num_cliprects = submit.fence_count;
                submit.submit.cliprects_ptr = (uintptr_t) submit.fences;
        }

        if (in_fence != -1) {
                submit.submit.flags |= I915_EXEC_FENCE_IN;
                submit.submit.rsvd2 |= (uint32_t)in_fence;
        }

        if (need_out_fence)
                submit.submit.flags |= I915_EXEC_FENCE_OUT;
#endif

        result = bcmv_gem_submit_cl(device, &submit.submit);

        /* Submit does not consume the in_fence.  It's our job to close it. */
        if (in_fence != -1)
                close(in_fence);

        for (uint32_t i = 0; i < num_in_semaphores; i++) {
                BCMV_FROM_HANDLE(bcmv_semaphore, semaphore, in_semaphores[i]);
                /* From the Vulkan 1.0.53 spec:
                 *
                 *    "If the import is temporary, the implementation must
                 *     restore the semaphore to its prior permanent state after
                 *    submitting the next semaphore wait operation."
                 *
                 * This has to happen after the submit in case we close any
                 * syncobjs in the process.
                 */
                bcmv_semaphore_reset_temporary(device, semaphore);
        }

        if (result == VK_SUCCESS && need_out_fence) {
#if 0 /* XXX */
                int out_fence = submit.submit.rsvd2 >> 32;
                for (uint32_t i = 0; i < num_out_semaphores; i++) {
                        BCMV_FROM_HANDLE(bcmv_semaphore, semaphore, out_semaphores[i]);
                        /* Out fences can't have temporary state because that would imply
                         * that we imported a sync file and are trying to signal it.
                         */
                        assert(semaphore->temporary.type == BCMV_SEMAPHORE_TYPE_NONE);
                        struct bcmv_semaphore_impl *impl = &semaphore->permanent;

                        if (impl->type == BCMV_SEMAPHORE_TYPE_SYNC_FILE) {
                                assert(impl->fd == -1);
                                impl->fd = dup(out_fence);
                        }
                }
                close(out_fence);
#endif
        }

        bcmv_submit_finish(&submit, &device->alloc);

        return result;
}
