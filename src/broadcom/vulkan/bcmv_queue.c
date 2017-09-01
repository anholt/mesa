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

/**
 * This file implements VkQueue, VkFence, and VkSemaphore
 */

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/eventfd.h>

#include "bcmv_private.h"
#include "vk_util.h"

VkResult bcmv_QueueSubmit(
        VkQueue                                     _queue,
        uint32_t                                    submitCount,
        const VkSubmitInfo*                         pSubmits,
        VkFence                                     fence)
{
        BCMV_FROM_HANDLE(bcmv_queue, queue, _queue);
        struct bcmv_device *device = queue->device;

        /* Query for device status prior to submitting.  Technically, we don't
         * need to do this.  However, if we have a client that's submitting
         * piles of garbage, we would rather break as early as possible to
         * keep the GPU hanging contained.  If we don't check here, we'll
         * either be waiting for the kernel to kick us or we'll have to wait
         * until the client waits on a fence before we actually know whether
         * or not we've hung.
         */
        VkResult result = bcmv_device_query_status(device);
        if (result != VK_SUCCESS)
                return result;

        /* We lock around QueueSubmit for three main reasons:
         *
         *  1) When a block pool is resized, we create a new gem handle with a
         *     different size and, in the case of surface states, possibly a
         *     different center offset but we re-use the same bcmv_bo struct when
         *     we do so.  If this happens in the middle of setting up an execbuf,
         *     we could end up with our list of BOs out of sync with our list of
         *     gem handles.
         *
         *  2) The algorithm we use for building the list of unique buffers isn't
         *     thread-safe.  While the client is supposed to syncronize around
         *     QueueSubmit, this would be extremely difficult to debug if it ever
         *     came up in the wild due to a broken app.  It's better to play it
         *     safe and just lock around QueueSubmit.
         *
         *  3)  The bcmv_cmd_buffer_execbuf function may perform relocations in
         *      userspace.  Due to the fact that the surface state buffer is shared
         *      between batches, we can't afford to have that happen from multiple
         *      threads at the same time.  Even though the user is supposed to
         *      ensure this doesn't happen, we play it safe as in (2) above.
         *
         * Since the only other things that ever take the device lock such as
         * block pool resize only rarely happen, this will almost never be
         * contended so taking a lock isn't really an expensive operation in
         * this case.
         */
        pthread_mutex_lock(&device->mutex);

        if (fence && submitCount == 0) {
                /* If we don't have any command buffers, we need to submit a
                 * dummy batch to give GEM something to wait on.  We could,
                 * potentially, come up with something more efficient but this
                 * shouldn't be a common case.
                 */
                result = bcmv_cmd_buffer_submit(device, NULL, NULL, 0, NULL, 0, fence);
                goto out;
        }

        for (uint32_t i = 0; i < submitCount; i++) {
                /* Fence for this submit.  NULL for all but the last one */
                VkFence submit_fence = (i == submitCount - 1) ? fence : VK_NULL_HANDLE;

                if (pSubmits[i].commandBufferCount == 0) {
                        /* If we don't have any command buffers, we need to submit a dummy
                         * batch to give GEM something to wait on.  We could, potentially,
                         * come up with something more efficient but this shouldn't be a
                         * common case.
                         */
                        result = bcmv_cmd_buffer_submit(device, NULL,
                                                        pSubmits[i].pWaitSemaphores,
                                                        pSubmits[i].waitSemaphoreCount,
                                                        pSubmits[i].pSignalSemaphores,
                                                        pSubmits[i].signalSemaphoreCount,
                                                        submit_fence);
                        if (result != VK_SUCCESS)
                                goto out;

                        continue;
                }

                for (uint32_t j = 0; j < pSubmits[i].commandBufferCount; j++) {
                        BCMV_FROM_HANDLE(bcmv_cmd_buffer, cmd_buffer,
                                         pSubmits[i].pCommandBuffers[j]);
                        assert(cmd_buffer->level == VK_COMMAND_BUFFER_LEVEL_PRIMARY);
                        assert(!bcmv_batch_has_error(&cmd_buffer->batch));

                        /* Fence for this execbuf.  NULL for all but the last
                         * one
                         */
                        VkFence execbuf_fence =
                                (j == pSubmits[i].commandBufferCount - 1) ?
                                submit_fence : VK_NULL_HANDLE;

                        const VkSemaphore *in_semaphores = NULL, *out_semaphores = NULL;
                        uint32_t num_in_semaphores = 0, num_out_semaphores = 0;
                        if (j == 0) {
                                /* Only the first batch gets the in semaphores */
                                in_semaphores = pSubmits[i].pWaitSemaphores;
                                num_in_semaphores = pSubmits[i].waitSemaphoreCount;
                        }

                        if (j == pSubmits[i].commandBufferCount - 1) {
                                /* Only the last batch gets the out semaphores */
                                out_semaphores = pSubmits[i].pSignalSemaphores;
                                num_out_semaphores = pSubmits[i].signalSemaphoreCount;
                        }

                        result = bcmv_cmd_buffer_submit(device, cmd_buffer,
                                                        in_semaphores, num_in_semaphores,
                                                        out_semaphores, num_out_semaphores,
                                                        execbuf_fence);
                        if (result != VK_SUCCESS)
                                goto out;
                }
        }

        pthread_cond_broadcast(&device->queue_submit);

 out:
        if (result != VK_SUCCESS) {
                /* In the case that something has gone wrong we may end up with an
                 * inconsistent state from which it may not be trivial to recover.
                 * For example, we might have computed address relocations and
                 * any future attempt to re-submit this job will need to know about
                 * this and avoid computing relocation addresses again.
                 *
                 * To avoid this sort of issues, we assume that if something was
                 * wrong during submission we must already be in a really bad situation
                 * anyway (such us being out of memory) and return
                 * VK_ERROR_DEVICE_LOST to ensure that clients do not attempt to
                 * submit the same job again to this device.
                 */
                result = vk_errorf(VK_ERROR_DEVICE_LOST, "vkQueueSubmit() failed");
                device->lost = true;
        }

        pthread_mutex_unlock(&device->mutex);

        return result;
}

VkResult bcmv_QueueWaitIdle(VkQueue _queue)
{
        BCMV_FROM_HANDLE(bcmv_queue, queue, _queue);

        return bcmv_DeviceWaitIdle(bcmv_device_to_handle(queue->device));
}

VkResult bcmv_CreateFence(
        VkDevice                                    _device,
        const VkFenceCreateInfo*                    pCreateInfo,
        const VkAllocationCallbacks*                pAllocator,
        VkFence*                                    pFence)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        struct bcmv_fence *fence;

        assert(pCreateInfo->sType == VK_STRUCTURE_TYPE_FENCE_CREATE_INFO);

        fence = vk_zalloc2(&device->alloc, pAllocator, sizeof(*fence), 8,
                           VK_SYSTEM_ALLOCATION_SCOPE_OBJECT);
        if (fence == NULL)
                return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);

        fence->permanent.type = BCMV_FENCE_TYPE_SYNCOBJ;

        uint32_t create_flags = 0;
        if (pCreateInfo->flags & VK_FENCE_CREATE_SIGNALED_BIT)
                create_flags |= DRM_SYNCOBJ_CREATE_SIGNALED;

        fence->permanent.syncobj = bcmv_gem_syncobj_create(device, create_flags);
        if (!fence->permanent.syncobj)
                return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);

        *pFence = bcmv_fence_to_handle(fence);

        return VK_SUCCESS;
}

static void
bcmv_fence_impl_cleanup(struct bcmv_device *device,
                        struct bcmv_fence_impl *impl)
{
        switch (impl->type) {
        case BCMV_FENCE_TYPE_NONE:
                /* Dummy.  Nothing to do */
                return;

        case BCMV_FENCE_TYPE_SYNCOBJ:
                bcmv_gem_syncobj_destroy(device, impl->syncobj);
                return;
        }

        unreachable("Invalid fence type");
}

void bcmv_DestroyFence(
        VkDevice                                    _device,
        VkFence                                     _fence,
        const VkAllocationCallbacks*                pAllocator)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        BCMV_FROM_HANDLE(bcmv_fence, fence, _fence);

        if (!fence)
                return;

        bcmv_fence_impl_cleanup(device, &fence->temporary);
        bcmv_fence_impl_cleanup(device, &fence->permanent);

        vk_free2(&device->alloc, pAllocator, fence);
}

VkResult bcmv_ResetFences(
        VkDevice                                    _device,
        uint32_t                                    fenceCount,
        const VkFence*                              pFences)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);

        for (uint32_t i = 0; i < fenceCount; i++) {
                BCMV_FROM_HANDLE(bcmv_fence, fence, pFences[i]);

                /* From the Vulkan 1.0.53 spec:
                 *
                 *    "If any member of pFences currently has its payload imported with
                 *    temporary permanence, that fence’s prior permanent payload is
                 *    first restored. The remaining operations described therefore
                 *    operate on the restored payload.
                 */
                if (fence->temporary.type != BCMV_FENCE_TYPE_NONE) {
                        bcmv_fence_impl_cleanup(device, &fence->temporary);
                        fence->temporary.type = BCMV_FENCE_TYPE_NONE;
                }

                struct bcmv_fence_impl *impl = &fence->permanent;

                if (impl->type == BCMV_FENCE_TYPE_SYNCOBJ)
                        bcmv_gem_syncobj_reset(device, impl->syncobj);
        }

        return VK_SUCCESS;
}

VkResult bcmv_GetFenceStatus(
        VkDevice                                    _device,
        VkFence                                     _fence)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        BCMV_FROM_HANDLE(bcmv_fence, fence, _fence);

        if (unlikely(device->lost))
                return VK_ERROR_DEVICE_LOST;

        struct bcmv_fence_impl *impl =
                fence->temporary.type != BCMV_FENCE_TYPE_NONE ?
                &fence->temporary : &fence->permanent;

        int ret = bcmv_gem_syncobj_wait(device, &impl->syncobj, 1, 0, true);
        if (ret == -1) {
                if (errno == ETIME) {
                        return VK_NOT_READY;
                } else {
                        /* We don't know the real error. */
                        device->lost = true;
                        return vk_errorf(VK_ERROR_DEVICE_LOST,
                                         "drm_syncobj_wait failed: %m");
                }
        } else {
                return VK_SUCCESS;
        }
}

#define NSEC_PER_SEC 1000000000
#define INT_TYPE_MAX(type) ((1ull << (sizeof(type) * 8 - 1)) - 1)

static uint64_t
gettime_ns(void)
{
        struct timespec current;
        clock_gettime(CLOCK_MONOTONIC, &current);
        return (uint64_t)current.tv_sec * NSEC_PER_SEC + current.tv_nsec;
}

static VkResult
bcmv_wait_for_syncobj_fences(struct bcmv_device *device,
                             uint32_t fenceCount,
                             const VkFence *pFences,
                             bool waitAll,
                             uint64_t _timeout)
{
        uint32_t *syncobjs = vk_zalloc(&device->alloc,
                                       sizeof(*syncobjs) * fenceCount, 8,
                                       VK_SYSTEM_ALLOCATION_SCOPE_COMMAND);
        if (!syncobjs)
                return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);

        for (uint32_t i = 0; i < fenceCount; i++) {
                BCMV_FROM_HANDLE(bcmv_fence, fence, pFences[i]);
                assert(fence->permanent.type == BCMV_FENCE_TYPE_SYNCOBJ);

                struct bcmv_fence_impl *impl =
                        fence->temporary.type != BCMV_FENCE_TYPE_NONE ?
                        &fence->temporary : &fence->permanent;

                assert(impl->type == BCMV_FENCE_TYPE_SYNCOBJ);
                syncobjs[i] = impl->syncobj;
        }

        int64_t abs_timeout_ns = 0;
        if (_timeout > 0) {
                uint64_t current_ns = gettime_ns();

                /* Add but saturate to INT32_MAX */
                if (current_ns + _timeout < current_ns)
                        abs_timeout_ns = INT64_MAX;
                else if (current_ns + _timeout > INT64_MAX)
                        abs_timeout_ns = INT64_MAX;
                else
                        abs_timeout_ns = current_ns + _timeout;
        }

        /* The gem_syncobj_wait ioctl may return early due to an inherent
         * limitation in the way it computes timeouts.  Loop until we've actually
         * passed the timeout.
         */
        int ret;
        do {
                ret = bcmv_gem_syncobj_wait(device, syncobjs, fenceCount,
                                            abs_timeout_ns, waitAll);
        } while (ret == -1 && errno == ETIME && gettime_ns() < abs_timeout_ns);

        vk_free(&device->alloc, syncobjs);

        if (ret == -1) {
                if (errno == ETIME) {
                        return VK_TIMEOUT;
                } else {
                        /* We don't know the real error. */
                        device->lost = true;
                        return vk_errorf(VK_ERROR_DEVICE_LOST,
                                         "drm_syncobj_wait failed: %m");
                }
        } else {
                return VK_SUCCESS;
        }
}

VkResult bcmv_WaitForFences(
        VkDevice                                    _device,
        uint32_t                                    fenceCount,
        const VkFence*                              pFences,
        VkBool32                                    waitAll,
        uint64_t                                    timeout)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);

        if (unlikely(device->lost))
                return VK_ERROR_DEVICE_LOST;

        return bcmv_wait_for_syncobj_fences(device, fenceCount, pFences,
                                            waitAll, timeout);
}

void bcmv_GetPhysicalDeviceExternalFencePropertiesKHR(
        VkPhysicalDevice                            physicalDevice,
        const VkPhysicalDeviceExternalFenceInfoKHR* pExternalFenceInfo,
        VkExternalFencePropertiesKHR*               pExternalFenceProperties)
{
        switch (pExternalFenceInfo->handleType) {
        case VK_EXTERNAL_FENCE_HANDLE_TYPE_OPAQUE_FD_BIT_KHR:
        case VK_EXTERNAL_FENCE_HANDLE_TYPE_SYNC_FD_BIT_KHR:
                pExternalFenceProperties->exportFromImportedHandleTypes =
                        VK_EXTERNAL_FENCE_HANDLE_TYPE_OPAQUE_FD_BIT_KHR |
                        VK_EXTERNAL_FENCE_HANDLE_TYPE_SYNC_FD_BIT_KHR;
                pExternalFenceProperties->compatibleHandleTypes =
                        VK_EXTERNAL_FENCE_HANDLE_TYPE_OPAQUE_FD_BIT_KHR |
                        VK_EXTERNAL_FENCE_HANDLE_TYPE_SYNC_FD_BIT_KHR;
                pExternalFenceProperties->externalFenceFeatures =
                        VK_EXTERNAL_FENCE_FEATURE_EXPORTABLE_BIT_KHR |
                        VK_EXTERNAL_FENCE_FEATURE_IMPORTABLE_BIT_KHR;
                return;

        default:
                break;
        }

        pExternalFenceProperties->exportFromImportedHandleTypes = 0;
        pExternalFenceProperties->compatibleHandleTypes = 0;
        pExternalFenceProperties->externalFenceFeatures = 0;
}

VkResult bcmv_ImportFenceFdKHR(
        VkDevice                                    _device,
        const VkImportFenceFdInfoKHR*               pImportFenceFdInfo)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        BCMV_FROM_HANDLE(bcmv_fence, fence, pImportFenceFdInfo->fence);
        int fd = pImportFenceFdInfo->fd;

        assert(pImportFenceFdInfo->sType ==
               VK_STRUCTURE_TYPE_IMPORT_FENCE_FD_INFO_KHR);

        struct bcmv_fence_impl new_impl = {
                .type = BCMV_FENCE_TYPE_NONE,
        };

        switch (pImportFenceFdInfo->handleType) {
        case VK_EXTERNAL_FENCE_HANDLE_TYPE_OPAQUE_FD_BIT_KHR:
                new_impl.type = BCMV_FENCE_TYPE_SYNCOBJ;

                new_impl.syncobj = bcmv_gem_syncobj_fd_to_handle(device, fd);
                if (!new_impl.syncobj)
                        return vk_error(VK_ERROR_INVALID_EXTERNAL_HANDLE_KHR);

                break;

        case VK_EXTERNAL_FENCE_HANDLE_TYPE_SYNC_FD_BIT_KHR:
                /* Sync files are a bit tricky.  Because we want to continue using the
                 * syncobj implementation of WaitForFences, we don't use the sync file
                 * directly but instead import it into a syncobj.
                 */
                new_impl.type = BCMV_FENCE_TYPE_SYNCOBJ;

                new_impl.syncobj = bcmv_gem_syncobj_create(device, 0);
                if (!new_impl.syncobj)
                        return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);

                if (bcmv_gem_syncobj_import_sync_file(device, new_impl.syncobj, fd)) {
                        bcmv_gem_syncobj_destroy(device, new_impl.syncobj);
                        return vk_errorf(VK_ERROR_INVALID_EXTERNAL_HANDLE_KHR,
                                         "syncobj sync file import failed: %m");
                }
                break;

        default:
                return vk_error(VK_ERROR_INVALID_EXTERNAL_HANDLE_KHR);
        }

        /* From the Vulkan 1.0.53 spec:
         *
         *    "Importing a fence payload from a file descriptor transfers
         *    ownership of the file descriptor from the application to the
         *    Vulkan implementation. The application must not perform any
         *    operations on the file descriptor after a successful import."
         *
         * If the import fails, we leave the file descriptor open.
         */
        close(fd);

        if (pImportFenceFdInfo->flags & VK_FENCE_IMPORT_TEMPORARY_BIT_KHR) {
                bcmv_fence_impl_cleanup(device, &fence->temporary);
                fence->temporary = new_impl;
        } else {
                bcmv_fence_impl_cleanup(device, &fence->permanent);
                fence->permanent = new_impl;
        }

        return VK_SUCCESS;
}

VkResult bcmv_GetFenceFdKHR(
        VkDevice                                    _device,
        const VkFenceGetFdInfoKHR*                  pGetFdInfo,
        int*                                        pFd)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        BCMV_FROM_HANDLE(bcmv_fence, fence, pGetFdInfo->fence);

        assert(pGetFdInfo->sType == VK_STRUCTURE_TYPE_FENCE_GET_FD_INFO_KHR);

        struct bcmv_fence_impl *impl =
                fence->temporary.type != BCMV_FENCE_TYPE_NONE ?
                &fence->temporary : &fence->permanent;

        assert(impl->type == BCMV_FENCE_TYPE_SYNCOBJ);
        switch (pGetFdInfo->handleType) {
        case VK_EXTERNAL_FENCE_HANDLE_TYPE_OPAQUE_FD_BIT_KHR: {
                int fd = bcmv_gem_syncobj_handle_to_fd(device, impl->syncobj);
                if (fd < 0)
                        return vk_error(VK_ERROR_TOO_MANY_OBJECTS);

                *pFd = fd;
                break;
        }

        case VK_EXTERNAL_FENCE_HANDLE_TYPE_SYNC_FD_BIT_KHR: {
                int fd = bcmv_gem_syncobj_export_sync_file(device, impl->syncobj);
                if (fd < 0)
                        return vk_error(VK_ERROR_TOO_MANY_OBJECTS);

                *pFd = fd;
                break;
        }

        default:
                unreachable("Invalid fence export handle type");
        }

        /* From the Vulkan 1.0.53 spec:
         *
         *    "Export operations have the same transference as the specified handle
         *    type’s import operations. [...] If the fence was using a
         *    temporarily imported payload, the fence’s prior permanent payload
         *    will be restored.
         */
        if (impl == &fence->temporary)
                bcmv_fence_impl_cleanup(device, impl);

        return VK_SUCCESS;
}

// Queue semaphore functions

VkResult bcmv_CreateSemaphore(
        VkDevice                                    _device,
        const VkSemaphoreCreateInfo*                pCreateInfo,
        const VkAllocationCallbacks*                pAllocator,
        VkSemaphore*                                pSemaphore)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        struct bcmv_semaphore *semaphore;

        assert(pCreateInfo->sType == VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO);

        semaphore = vk_alloc2(&device->alloc, pAllocator, sizeof(*semaphore), 8,
                              VK_SYSTEM_ALLOCATION_SCOPE_OBJECT);
        if (semaphore == NULL)
                return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);

        const VkExportSemaphoreCreateInfoKHR *export =
                vk_find_struct_const(pCreateInfo->pNext, EXPORT_SEMAPHORE_CREATE_INFO_KHR);
        VkExternalSemaphoreHandleTypeFlagsKHR handleTypes =
                export ? export->handleTypes : 0;

        if (handleTypes == 0) {
                /* The DRM execbuffer ioctl always execute in-oder so long as you stay
                 * on the same ring.  Since we don't expose the blit engine as a DMA
                 * queue, a dummy no-op semaphore is a perfectly valid implementation.
                 */
                semaphore->permanent.type = BCMV_SEMAPHORE_TYPE_DUMMY;
        } else if (handleTypes & VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT_KHR) {
                assert(handleTypes == VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT_KHR);
                semaphore->permanent.type = BCMV_SEMAPHORE_TYPE_DRM_SYNCOBJ;
                semaphore->permanent.syncobj = bcmv_gem_syncobj_create(device, 0);
                if (!semaphore->permanent.syncobj) {
                        vk_free2(&device->alloc, pAllocator, semaphore);
                        return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);
                }
        } else if (handleTypes & VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_SYNC_FD_BIT_KHR) {
                assert(handleTypes == VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_SYNC_FD_BIT_KHR);

                semaphore->permanent.type = BCMV_SEMAPHORE_TYPE_SYNC_FILE;
                semaphore->permanent.fd = -1;
        } else {
                assert(!"Unknown handle type");
                vk_free2(&device->alloc, pAllocator, semaphore);
                return vk_error(VK_ERROR_INVALID_EXTERNAL_HANDLE_KHR);
        }

        semaphore->temporary.type = BCMV_SEMAPHORE_TYPE_NONE;

        *pSemaphore = bcmv_semaphore_to_handle(semaphore);

        return VK_SUCCESS;
}

static void
bcmv_semaphore_impl_cleanup(struct bcmv_device *device,
                            struct bcmv_semaphore_impl *impl)
{
        switch (impl->type) {
        case BCMV_SEMAPHORE_TYPE_NONE:
        case BCMV_SEMAPHORE_TYPE_DUMMY:
                /* Dummy.  Nothing to do */
                return;

        case BCMV_SEMAPHORE_TYPE_SYNC_FILE:
                close(impl->fd);
                return;

        case BCMV_SEMAPHORE_TYPE_DRM_SYNCOBJ:
                bcmv_gem_syncobj_destroy(device, impl->syncobj);
                return;
        }

        unreachable("Invalid semaphore type");
}

void
bcmv_semaphore_reset_temporary(struct bcmv_device *device,
                               struct bcmv_semaphore *semaphore)
{
        if (semaphore->temporary.type == BCMV_SEMAPHORE_TYPE_NONE)
                return;

        bcmv_semaphore_impl_cleanup(device, &semaphore->temporary);
        semaphore->temporary.type = BCMV_SEMAPHORE_TYPE_NONE;
}

void bcmv_DestroySemaphore(
        VkDevice                                    _device,
        VkSemaphore                                 _semaphore,
        const VkAllocationCallbacks*                pAllocator)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        BCMV_FROM_HANDLE(bcmv_semaphore, semaphore, _semaphore);

        if (semaphore == NULL)
                return;

        bcmv_semaphore_impl_cleanup(device, &semaphore->temporary);
        bcmv_semaphore_impl_cleanup(device, &semaphore->permanent);

        vk_free2(&device->alloc, pAllocator, semaphore);
}

void bcmv_GetPhysicalDeviceExternalSemaphorePropertiesKHR(
        VkPhysicalDevice                            physicalDevice,
        const VkPhysicalDeviceExternalSemaphoreInfoKHR* pExternalSemaphoreInfo,
        VkExternalSemaphorePropertiesKHR*           pExternalSemaphoreProperties)
{
        BCMV_FROM_HANDLE(bcmv_physical_device, device, physicalDevice);

        switch (pExternalSemaphoreInfo->handleType) {
        case VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT_KHR:
                pExternalSemaphoreProperties->exportFromImportedHandleTypes =
                        VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT_KHR;
                pExternalSemaphoreProperties->compatibleHandleTypes =
                        VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT_KHR;
                pExternalSemaphoreProperties->externalSemaphoreFeatures =
                        VK_EXTERNAL_SEMAPHORE_FEATURE_EXPORTABLE_BIT_KHR |
                        VK_EXTERNAL_SEMAPHORE_FEATURE_IMPORTABLE_BIT_KHR;
                return;

        case VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_SYNC_FD_BIT_KHR:
                if (device->has_exec_fence) {
                        pExternalSemaphoreProperties->exportFromImportedHandleTypes = 0;
                        pExternalSemaphoreProperties->compatibleHandleTypes =
                                VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_SYNC_FD_BIT_KHR;
                        pExternalSemaphoreProperties->externalSemaphoreFeatures =
                                VK_EXTERNAL_SEMAPHORE_FEATURE_EXPORTABLE_BIT_KHR |
                                VK_EXTERNAL_SEMAPHORE_FEATURE_IMPORTABLE_BIT_KHR;
                        return;
                }
                break;

        default:
                break;
        }

        pExternalSemaphoreProperties->exportFromImportedHandleTypes = 0;
        pExternalSemaphoreProperties->compatibleHandleTypes = 0;
        pExternalSemaphoreProperties->externalSemaphoreFeatures = 0;
}

VkResult bcmv_ImportSemaphoreFdKHR(
        VkDevice                                    _device,
        const VkImportSemaphoreFdInfoKHR*           pImportSemaphoreFdInfo)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        BCMV_FROM_HANDLE(bcmv_semaphore, semaphore, pImportSemaphoreFdInfo->semaphore);
        int fd = pImportSemaphoreFdInfo->fd;

        struct bcmv_semaphore_impl new_impl = {
                .type = BCMV_SEMAPHORE_TYPE_NONE,
        };

        switch (pImportSemaphoreFdInfo->handleType) {
        case VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT_KHR:
                new_impl.type = BCMV_SEMAPHORE_TYPE_DRM_SYNCOBJ;

                new_impl.syncobj = bcmv_gem_syncobj_fd_to_handle(device, fd);
                if (!new_impl.syncobj)
                        return vk_error(VK_ERROR_INVALID_EXTERNAL_HANDLE_KHR);

                /* From the Vulkan spec:
                 *
                 *    "Importing semaphore state from a file descriptor transfers
                 *    ownership of the file descriptor from the application to the
                 *    Vulkan implementation. The application must not perform any
                 *    operations on the file descriptor after a successful import."
                 *
                 * If the import fails, we leave the file descriptor open.
                 */
                close(pImportSemaphoreFdInfo->fd);
                break;

        case VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_SYNC_FD_BIT_KHR:
                new_impl = (struct bcmv_semaphore_impl) {
                        .type = BCMV_SEMAPHORE_TYPE_SYNC_FILE,
                        .fd = fd,
                };
                break;

        default:
                return vk_error(VK_ERROR_INVALID_EXTERNAL_HANDLE_KHR);
        }

        if (pImportSemaphoreFdInfo->flags & VK_SEMAPHORE_IMPORT_TEMPORARY_BIT_KHR) {
                bcmv_semaphore_impl_cleanup(device, &semaphore->temporary);
                semaphore->temporary = new_impl;
        } else {
                bcmv_semaphore_impl_cleanup(device, &semaphore->permanent);
                semaphore->permanent = new_impl;
        }

        return VK_SUCCESS;
}

VkResult bcmv_GetSemaphoreFdKHR(
        VkDevice                                    _device,
        const VkSemaphoreGetFdInfoKHR*              pGetFdInfo,
        int*                                        pFd)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        BCMV_FROM_HANDLE(bcmv_semaphore, semaphore, pGetFdInfo->semaphore);
        int fd;

        assert(pGetFdInfo->sType == VK_STRUCTURE_TYPE_SEMAPHORE_GET_FD_INFO_KHR);

        struct bcmv_semaphore_impl *impl =
                semaphore->temporary.type != BCMV_SEMAPHORE_TYPE_NONE ?
                &semaphore->temporary : &semaphore->permanent;

        switch (impl->type) {
        case BCMV_SEMAPHORE_TYPE_SYNC_FILE:
                /* There are two reasons why this could happen:
                 *
                 *  1) The user is trying to export without submitting something that
                 *     signals the semaphore.  If this is the case, it's their bug so
                 *     what we return here doesn't matter.
                 *
                 *  2) The kernel didn't give us a file descriptor.  The most likely
                 *     reason for this is running out of file descriptors.
                 */
                if (impl->fd < 0)
                        return vk_error(VK_ERROR_TOO_MANY_OBJECTS);

                *pFd = impl->fd;

                /* From the Vulkan 1.0.53 spec:
                 *
                 *    "...exporting a semaphore payload to a handle with copy
                 *    transference has the same side effects on the source
                 *    semaphore’s payload as executing a semaphore wait operation."
                 *
                 * In other words, it may still be a SYNC_FD semaphore, but it's now
                 * considered to have been waited on and no longer has a sync file
                 * attached.
                 */
                impl->fd = -1;
                return VK_SUCCESS;

        case BCMV_SEMAPHORE_TYPE_DRM_SYNCOBJ:
                fd = bcmv_gem_syncobj_handle_to_fd(device, impl->syncobj);
                if (fd < 0)
                        return vk_error(VK_ERROR_TOO_MANY_OBJECTS);
                *pFd = fd;
                break;

        default:
                return vk_error(VK_ERROR_INVALID_EXTERNAL_HANDLE_KHR);
        }

        /* From the Vulkan 1.0.53 spec:
         *
         *    "Export operations have the same transference as the specified handle
         *    type’s import operations. [...] If the semaphore was using a
         *    temporarily imported payload, the semaphore’s prior permanent payload
         *    will be restored.
         */
        if (impl == &semaphore->temporary)
                bcmv_semaphore_impl_cleanup(device, impl);

        return VK_SUCCESS;
}
