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

#include "bcmv_private.h"
#include "wsi_common.h"
// XXX #include "vk_format_info.h"
#include "vk_util.h"

#ifdef VK_USE_PLATFORM_WAYLAND_KHR
static const struct wsi_callbacks wsi_cbs = {
        .get_phys_device_format_properties = bcmv_GetPhysicalDeviceFormatProperties,
};
#endif

VkResult
bcmv_init_wsi(struct bcmv_physical_device *physical_device)
{
        VkResult result;

        memset(physical_device->wsi_device.wsi, 0, sizeof(physical_device->wsi_device.wsi));

#ifdef VK_USE_PLATFORM_XCB_KHR
        result = wsi_x11_init_wsi(&physical_device->wsi_device, &physical_device->instance->alloc);
        if (result != VK_SUCCESS)
                return result;
#endif

#ifdef VK_USE_PLATFORM_WAYLAND_KHR
        result = wsi_wl_init_wsi(&physical_device->wsi_device, &physical_device->instance->alloc,
                                 bcmv_physical_device_to_handle(physical_device),
                                 &wsi_cbs);
        if (result != VK_SUCCESS) {
#ifdef VK_USE_PLATFORM_XCB_KHR
                wsi_x11_finish_wsi(&physical_device->wsi_device, &physical_device->instance->alloc);
#endif
                return result;
        }
#endif

        return VK_SUCCESS;
}

void
bcmv_finish_wsi(struct bcmv_physical_device *physical_device)
{
#ifdef VK_USE_PLATFORM_WAYLAND_KHR
        wsi_wl_finish_wsi(&physical_device->wsi_device, &physical_device->instance->alloc);
#endif
#ifdef VK_USE_PLATFORM_XCB_KHR
        wsi_x11_finish_wsi(&physical_device->wsi_device, &physical_device->instance->alloc);
#endif
}

void bcmv_DestroySurfaceKHR(
        VkInstance                                   _instance,
        VkSurfaceKHR                                 _surface,
        const VkAllocationCallbacks*                 pAllocator)
{
        BCMV_FROM_HANDLE(bcmv_instance, instance, _instance);
        ICD_FROM_HANDLE(VkIcdSurfaceBase, surface, _surface);

        if (!surface)
                return;

        vk_free2(&instance->alloc, pAllocator, surface);
}

VkResult bcmv_GetPhysicalDeviceSurfaceSupportKHR(
        VkPhysicalDevice                            physicalDevice,
        uint32_t                                    queueFamilyIndex,
        VkSurfaceKHR                                _surface,
        VkBool32*                                   pSupported)
{
        BCMV_FROM_HANDLE(bcmv_physical_device, device, physicalDevice);
        ICD_FROM_HANDLE(VkIcdSurfaceBase, surface, _surface);
        struct wsi_interface *iface = device->wsi_device.wsi[surface->platform];

        return iface->get_support(surface, &device->wsi_device,
                                  &device->instance->alloc,
                                  queueFamilyIndex, device->local_fd, false, pSupported);
}

VkResult bcmv_GetPhysicalDeviceSurfaceCapabilitiesKHR(
        VkPhysicalDevice                            physicalDevice,
        VkSurfaceKHR                                _surface,
        VkSurfaceCapabilitiesKHR*                   pSurfaceCapabilities)
{
        BCMV_FROM_HANDLE(bcmv_physical_device, device, physicalDevice);
        ICD_FROM_HANDLE(VkIcdSurfaceBase, surface, _surface);
        struct wsi_interface *iface = device->wsi_device.wsi[surface->platform];

        return iface->get_capabilities(surface, pSurfaceCapabilities);
}

VkResult bcmv_GetPhysicalDeviceSurfaceCapabilities2KHR(
        VkPhysicalDevice                            physicalDevice,
        const VkPhysicalDeviceSurfaceInfo2KHR*      pSurfaceInfo,
        VkSurfaceCapabilities2KHR*                  pSurfaceCapabilities)
{
        BCMV_FROM_HANDLE(bcmv_physical_device, device, physicalDevice);
        ICD_FROM_HANDLE(VkIcdSurfaceBase, surface, pSurfaceInfo->surface);
        struct wsi_interface *iface = device->wsi_device.wsi[surface->platform];

        return iface->get_capabilities2(surface, pSurfaceInfo->pNext,
                                        pSurfaceCapabilities);
}

VkResult bcmv_GetPhysicalDeviceSurfaceFormatsKHR(
        VkPhysicalDevice                            physicalDevice,
        VkSurfaceKHR                                _surface,
        uint32_t*                                   pSurfaceFormatCount,
        VkSurfaceFormatKHR*                         pSurfaceFormats)
{
        BCMV_FROM_HANDLE(bcmv_physical_device, device, physicalDevice);
        ICD_FROM_HANDLE(VkIcdSurfaceBase, surface, _surface);
        struct wsi_interface *iface = device->wsi_device.wsi[surface->platform];

        return iface->get_formats(surface, &device->wsi_device, pSurfaceFormatCount,
                                  pSurfaceFormats);
}

VkResult bcmv_GetPhysicalDeviceSurfaceFormats2KHR(
        VkPhysicalDevice                            physicalDevice,
        const VkPhysicalDeviceSurfaceInfo2KHR*      pSurfaceInfo,
        uint32_t*                                   pSurfaceFormatCount,
        VkSurfaceFormat2KHR*                        pSurfaceFormats)
{
        BCMV_FROM_HANDLE(bcmv_physical_device, device, physicalDevice);
        ICD_FROM_HANDLE(VkIcdSurfaceBase, surface, pSurfaceInfo->surface);
        struct wsi_interface *iface = device->wsi_device.wsi[surface->platform];

        return iface->get_formats2(surface, &device->wsi_device, pSurfaceInfo->pNext,
                                   pSurfaceFormatCount, pSurfaceFormats);
}

VkResult bcmv_GetPhysicalDeviceSurfacePresentModesKHR(
        VkPhysicalDevice                            physicalDevice,
        VkSurfaceKHR                                _surface,
        uint32_t*                                   pPresentModeCount,
        VkPresentModeKHR*                           pPresentModes)
{
        BCMV_FROM_HANDLE(bcmv_physical_device, device, physicalDevice);
        ICD_FROM_HANDLE(VkIcdSurfaceBase, surface, _surface);
        struct wsi_interface *iface = device->wsi_device.wsi[surface->platform];

        return iface->get_present_modes(surface, pPresentModeCount,
                                        pPresentModes);
}


static VkResult
bcmv_wsi_image_create(VkDevice device_h,
                      const VkSwapchainCreateInfoKHR *pCreateInfo,
                      const VkAllocationCallbacks* pAllocator,
                      bool different_gpu,
                      bool linear,
                      VkImage *image_p,
                      VkDeviceMemory *memory_p,
                      uint32_t *size,
                      uint32_t *offset,
                      uint32_t *row_pitch, int *fd_p)
{
        struct bcmv_device *device = bcmv_device_from_handle(device_h);
        VkImage image_h;
        struct bcmv_image *image;

        VkResult result;
#if 0
        result = bcmv_image_create(bcmv_device_to_handle(device),
                                   &(struct bcmv_image_create_info) {
                                           .isl_tiling_flags = ISL_TILING_X_BIT,
                                                   .stride = 0,
                                                   .vk_info =
                                                   &(VkImageCreateInfo) {
                                                   .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
                                                   .imageType = VK_IMAGE_TYPE_2D,
                                                   .format = pCreateInfo->imageFormat,
                                                   .extent = {
                                                           .width = pCreateInfo->imageExtent.width,
                                                           .height = pCreateInfo->imageExtent.height,
                                                           .depth = 1
                                                   },
                                                   .mipLevels = 1,
                                                   .arrayLayers = 1,
                                                   .samples = 1,
                                                   /* FIXME: Need a way to use X tiling to allow scanout */
                                                   .tiling = VK_IMAGE_TILING_OPTIMAL,
                                                   .usage = (pCreateInfo->imageUsage |
                                                             VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT),
                                                   .flags = 0,
                                           }},
                                   NULL,
                                   &image_h);
        if (result != VK_SUCCESS)
                return result;

        image = bcmv_image_from_handle(image_h);
        assert(vk_format_is_color(image->vk_format));
#endif /* XXX */

        VkDeviceMemory memory_h;
        struct bcmv_device_memory *memory;
        result = bcmv_AllocateMemory(bcmv_device_to_handle(device),
                                     &(VkMemoryAllocateInfo) {
                                             .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
                                                     .allocationSize = image->size,
                                                     .memoryTypeIndex = 0,
                                                     },
                                     NULL /* XXX: pAllocator */,
                                     &memory_h);
        if (result != VK_SUCCESS)
                goto fail_create_image;

        memory = bcmv_device_memory_from_handle(memory_h);

        bcmv_BindImageMemory(device_h, image_h, memory_h, 0);

        // XXX struct bcmv_surface *surface = &image->color_surface;
        // XXX assert(surface->isl.tiling == ISL_TILING_X);

        // XXX *row_pitch = surface->isl.row_pitch;

        int fd = bcmv_gem_handle_to_fd(device, memory->bo->gem_handle);
        if (fd == -1) {
                /* FINISHME: Choose a better error. */
                result = vk_errorf(VK_ERROR_OUT_OF_DEVICE_MEMORY,
                                   "handle_to_fd failed: %m");
                goto fail_alloc_memory;
        }

        *image_p = image_h;
        *memory_p = memory_h;
        *fd_p = fd;
        *size = image->size;
        *offset = image->offset;
        return VK_SUCCESS;
 fail_alloc_memory:
        bcmv_FreeMemory(device_h, memory_h, pAllocator);

 fail_create_image:
        bcmv_DestroyImage(device_h, image_h, pAllocator);
        return result;
}

static void
bcmv_wsi_image_free(VkDevice device,
                    const VkAllocationCallbacks* pAllocator,
                    VkImage image_h,
                    VkDeviceMemory memory_h)
{
        bcmv_DestroyImage(device, image_h, pAllocator);

        bcmv_FreeMemory(device, memory_h, pAllocator);
}

static const struct wsi_image_fns bcmv_wsi_image_fns = {
        .create_wsi_image = bcmv_wsi_image_create,
        .free_wsi_image = bcmv_wsi_image_free,
};

VkResult bcmv_CreateSwapchainKHR(
        VkDevice                                     _device,
        const VkSwapchainCreateInfoKHR*              pCreateInfo,
        const VkAllocationCallbacks*                 pAllocator,
        VkSwapchainKHR*                              pSwapchain)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        ICD_FROM_HANDLE(VkIcdSurfaceBase, surface, pCreateInfo->surface);
        struct wsi_interface *iface =
                device->instance->physicalDevice.wsi_device.wsi[surface->platform];
        struct wsi_swapchain *swapchain;
        const VkAllocationCallbacks *alloc;

        if (pAllocator)
                alloc = pAllocator;
        else
                alloc = &device->alloc;
        VkResult result = iface->create_swapchain(surface, _device,
                                                  &device->instance->physicalDevice.wsi_device,
                                                  device->instance->physicalDevice.local_fd,
                                                  pCreateInfo,
                                                  alloc, &bcmv_wsi_image_fns,
                                                  &swapchain);
        if (result != VK_SUCCESS)
                return result;

        swapchain->alloc = *alloc;

        for (unsigned i = 0; i < ARRAY_SIZE(swapchain->fences); i++)
                swapchain->fences[i] = VK_NULL_HANDLE;

        *pSwapchain = wsi_swapchain_to_handle(swapchain);

        return VK_SUCCESS;
}

void bcmv_DestroySwapchainKHR(
        VkDevice                                     _device,
        VkSwapchainKHR                               _swapchain,
        const VkAllocationCallbacks*                 pAllocator)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        BCMV_FROM_HANDLE(wsi_swapchain, swapchain, _swapchain);
        const VkAllocationCallbacks *alloc;

        if (!swapchain)
                return;

        if (pAllocator)
                alloc = pAllocator;
        else
                alloc = &device->alloc;
        for (unsigned i = 0; i < ARRAY_SIZE(swapchain->fences); i++) {
                if (swapchain->fences[i] != VK_NULL_HANDLE)
                        bcmv_DestroyFence(_device, swapchain->fences[i], pAllocator);
        }

        swapchain->destroy(swapchain, alloc);
}

VkResult bcmv_GetSwapchainImagesKHR(
        VkDevice                                     device,
        VkSwapchainKHR                               _swapchain,
        uint32_t*                                    pSwapchainImageCount,
        VkImage*                                     pSwapchainImages)
{
        BCMV_FROM_HANDLE(wsi_swapchain, swapchain, _swapchain);

        return swapchain->get_images(swapchain, pSwapchainImageCount,
                                     pSwapchainImages);
}

VkResult bcmv_AcquireNextImageKHR(
        VkDevice                                     _device,
        VkSwapchainKHR                               _swapchain,
        uint64_t                                     timeout,
        VkSemaphore                                  semaphore,
        VkFence                                      _fence,
        uint32_t*                                    pImageIndex)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        BCMV_FROM_HANDLE(wsi_swapchain, swapchain, _swapchain);
        BCMV_FROM_HANDLE(bcmv_fence, fence, _fence);

        VkResult result = swapchain->acquire_next_image(swapchain, timeout,
                                                        semaphore, pImageIndex);

        /* Thanks to implicit sync, the image is ready immediately.  However, we
         * should wait for the current GPU state to finish.
         */
        if (fence)
                bcmv_QueueSubmit(bcmv_queue_to_handle(&device->queue), 0, NULL, _fence);

        return result;
}

VkResult bcmv_QueuePresentKHR(
        VkQueue                                  _queue,
        const VkPresentInfoKHR*                  pPresentInfo)
{
        BCMV_FROM_HANDLE(bcmv_queue, queue, _queue);
        VkResult result = VK_SUCCESS;

        const VkPresentRegionsKHR *regions =
                vk_find_struct_const(pPresentInfo->pNext, PRESENT_REGIONS_KHR);

        for (uint32_t i = 0; i < pPresentInfo->swapchainCount; i++) {
                BCMV_FROM_HANDLE(wsi_swapchain, swapchain, pPresentInfo->pSwapchains[i]);
                VkResult item_result;

                const VkPresentRegionKHR *region = NULL;
                if (regions && regions->pRegions)
                        region = &regions->pRegions[i];

                assert(bcmv_device_from_handle(swapchain->device) == queue->device);

                if (swapchain->fences[0] == VK_NULL_HANDLE) {
                        item_result = bcmv_CreateFence(bcmv_device_to_handle(queue->device),
                                                       &(VkFenceCreateInfo) {
                                                               .sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO,
                                                                       .flags = 0,
                                                                       }, &swapchain->alloc, &swapchain->fences[0]);
                        if (pPresentInfo->pResults != NULL)
                                pPresentInfo->pResults[i] = item_result;
                        result = result == VK_SUCCESS ? item_result : result;
                        if (item_result != VK_SUCCESS)
                                continue;
                } else {
                        bcmv_ResetFences(bcmv_device_to_handle(queue->device),
                                         1, &swapchain->fences[0]);
                }

                bcmv_QueueSubmit(_queue, 0, NULL, swapchain->fences[0]);

                item_result = swapchain->queue_present(swapchain,
                                                       pPresentInfo->pImageIndices[i],
                                                       region);
                /* TODO: What if one of them returns OUT_OF_DATE? */
                if (pPresentInfo->pResults != NULL)
                        pPresentInfo->pResults[i] = item_result;
                result = result == VK_SUCCESS ? item_result : result;
                if (item_result != VK_SUCCESS)
                        continue;

                VkFence last = swapchain->fences[2];
                swapchain->fences[2] = swapchain->fences[1];
                swapchain->fences[1] = swapchain->fences[0];
                swapchain->fences[0] = last;

                if (last != VK_NULL_HANDLE) {
                        bcmv_WaitForFences(bcmv_device_to_handle(queue->device),
                                           1, &last, true, 1);
                }
        }

        return VK_SUCCESS;
}
