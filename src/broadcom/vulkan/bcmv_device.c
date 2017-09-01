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

#include <assert.h>
#include <stdbool.h>
#include <string.h>
#include <sys/errno.h>
#include <sys/mman.h>
#include <sys/sysinfo.h>
#include <unistd.h>
#include <fcntl.h>
#include <xf86drm.h>

#include "bcmv_private.h"
#include "util/strtod.h"
#include "util/debug.h"
#include "util/build_id.h"
#include "util/mesa-sha1.h"
#include "vk_util.h"

// XXX #include "genxml/gen7_pack.h"

static void
compiler_debug_log(void *data, const char *fmt, ...)
{ }

static void
compiler_perf_log(void *data, const char *fmt, ...)
{
        va_list args;
        va_start(args, fmt);

        if (unlikely(V3D_DEBUG & V3D_DEBUG_PERF))
                vfprintf(stderr, fmt, args);

        va_end(args);
}

static VkResult
bcmv_compute_heap_size(int fd, uint64_t *heap_size)
{
        /* Query the total ram from the system */
        struct sysinfo info;
        sysinfo(&info);

        uint64_t total_ram = (uint64_t)info.totalram * (uint64_t)info.mem_unit;

        /* We don't want to burn too much ram with the GPU.  If the user has
         * 4GiB or less, we use at most half.  If they have more than 4GiB, we
         * use 3/4.
         */
        uint64_t available_ram;
        if (total_ram <= 4ull * 1024ull * 1024ull * 1024ull)
                available_ram = total_ram / 2;
        else
                available_ram = total_ram * 3 / 4;

        /* The GPU address space is at most 4GB */
        *heap_size = MIN2(available_ram, ~0u);

        return VK_SUCCESS;
}

static VkResult
bcmv_physical_device_init_heaps(struct bcmv_physical_device *device, int fd)
{
        uint64_t heap_size;
        VkResult result = bcmv_compute_heap_size(fd, &heap_size);
        if (result != VK_SUCCESS)
                return result;

        /* In this case, everything fits nicely into the 32-bit
         * address space, so there's no need for supporting 48bit
         * addresses on client-allocated memory objects.
         */
        device->memory.heap_count = 1;
        device->memory.heaps[0] = (struct bcmv_memory_heap) {
                .size = heap_size,
                .flags = VK_MEMORY_HEAP_DEVICE_LOCAL_BIT,
        };

        uint32_t type_count = 0;
        for (uint32_t heap = 0; heap < device->memory.heap_count; heap++) {
                uint32_t valid_buffer_usage = ~0;
                /* The spec requires that we expose a host-visible,
                 * coherent memory type, but V3D doesn't participate
                 * in cache coherency with the CPU. Thus we offer two
                 * memory types to give the application a choice
                 * between cached, but not coherent and coherent but
                 * uncached (WC though).
                 */
                device->memory.types[type_count++] = (struct bcmv_memory_type) {
                        .propertyFlags = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT |
                        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                        VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                        .heapIndex = heap,
                        .valid_buffer_usage = valid_buffer_usage,
                };
                device->memory.types[type_count++] = (struct bcmv_memory_type) {
                        .propertyFlags = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT |
                        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                        VK_MEMORY_PROPERTY_HOST_CACHED_BIT,
                        .heapIndex = heap,
                        .valid_buffer_usage = valid_buffer_usage,
                };
        }
        device->memory.type_count = type_count;

        return VK_SUCCESS;
}

static VkResult
bcmv_physical_device_init_uuids(struct bcmv_physical_device *device)
{
        const struct build_id_note *note =
                build_id_find_nhdr_for_addr(bcmv_physical_device_init_uuids);
        if (!note) {
                return vk_errorf(VK_ERROR_INITIALIZATION_FAILED,
                                 "Failed to find build-id");
        }

        unsigned build_id_len = build_id_length(note);
        if (build_id_len < 20) {
                return vk_errorf(VK_ERROR_INITIALIZATION_FAILED,
                                 "build-id too short.  It needs to be a SHA");
        }

        struct mesa_sha1 sha1_ctx;
        uint8_t sha1[20];
        STATIC_ASSERT(VK_UUID_SIZE <= sizeof(sha1));

        /* The pipeline cache UUID is used for determining when a pipeline
         * cache is invalid.  It needs both a driver build and the PCI ID of
         * the device.
         */
        _mesa_sha1_init(&sha1_ctx);
        _mesa_sha1_update(&sha1_ctx, build_id_data(note), build_id_len);
        _mesa_sha1_update(&sha1_ctx, &device->chipset_id,
                          sizeof(device->chipset_id));
        _mesa_sha1_final(&sha1_ctx, sha1);
        memcpy(device->pipeline_cache_uuid, sha1, VK_UUID_SIZE);

        /* The driver UUID is used for determining sharability of images and
         * memory between two Vulkan instances in separate processes.  People
         * who want to share memory need to also check the device UUID (below)
         * so all this needs to be is the build-id.
         */
        memcpy(device->driver_uuid, build_id_data(note), VK_UUID_SIZE);

        /* The device UUID uniquely identifies the given device within the
         * machine.  Since we never have more than one device, this doesn't
         * need to be a real UUID.  However, on the off-chance that someone
         * tries to use this to cache pre-tiled images or something of the
         * like, we use the PCI ID and some bits of ISL info to ensure that
         * this is safe.
         */
        _mesa_sha1_init(&sha1_ctx);
        _mesa_sha1_update(&sha1_ctx, &device->info.ver,
                          sizeof(device->info.ver));
        /* XXX: Hash UIFCFG */
        _mesa_sha1_final(&sha1_ctx, sha1);
        memcpy(device->device_uuid, sha1, VK_UUID_SIZE);

        return VK_SUCCESS;
}

static bool
bcmv_get_device_info(int fd, struct v3d_device_info *devinfo)
{

        uint32_t core_ident0 =
                bcmv_gem_get_param(fd, DRM_VC5_PARAM_V3D_CORE0_IDENT0);
        if (!core_ident0)
                return false;

        uint32_t core_ident1 =
                bcmv_gem_get_param(fd, DRM_VC5_PARAM_V3D_CORE0_IDENT1);
        if (!core_ident1)
                return false;

        /* XXX: Process ident0/1 */
        devinfo->ver = 33;

        return true;
}

static const char *
bcmv_get_device_name(const struct v3d_device_info *devinfo)
{
        switch (devinfo->ver) {
        case 33:
                return "V3D 3.3";
        default:
                return NULL;
        }
}

static VkResult
bcmv_physical_device_init(struct bcmv_physical_device *device,
                          struct bcmv_instance *instance,
                          const char *path)
{
        VkResult result;
        int fd;

        fd = open(path, O_RDWR | O_CLOEXEC);
        if (fd < 0)
                return vk_error(VK_ERROR_INCOMPATIBLE_DRIVER);

        device->_loader_data.loaderMagic = ICD_LOADER_MAGIC;
        device->instance = instance;

        assert(strlen(path) < ARRAY_SIZE(device->path));
        strncpy(device->path, path, ARRAY_SIZE(device->path));

        if (!bcmv_get_device_info(fd, &device->info)) {
                result = vk_error(VK_ERROR_INCOMPATIBLE_DRIVER);
                goto fail;
        }

        switch (device->info.ver) {
        case 33:
                break;
        default:
                result = vk_errorf(VK_ERROR_INCOMPATIBLE_DRIVER,
                                   "Vulkan not yet supported on V3D %d.%d",
                                   device->info.ver / 10,
                                   device->info.ver % 10);
                goto fail;
        }

        device->name = bcmv_get_device_name(&device->info);

        result = bcmv_physical_device_init_heaps(device, fd);
        if (result != VK_SUCCESS)
                goto fail;

        /* XXX: Get UIFCFG */

        v3d_process_debug_variable();

        // XXX device->compiler = brw_compiler_create(NULL, &device->info);
        if (device->compiler == NULL) {
                result = vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);
                goto fail;
        }
        // XXX device->compiler->shader_debug_log = compiler_debug_log;
        // XXX device->compiler->shader_perf_log = compiler_perf_log;
        (void)compiler_debug_log;
        (void)compiler_perf_log;

        // XXX isl_device_init(&device->isl_dev, &device->info, swizzled);

        result = bcmv_physical_device_init_uuids(device);
        if (result != VK_SUCCESS)
                goto fail;

        result = bcmv_init_wsi(device);
        if (result != VK_SUCCESS) {
                ralloc_free(device->compiler);
                goto fail;
        }

        device->local_fd = fd;
        return VK_SUCCESS;

 fail:
        close(fd);
        return result;
}

static void
bcmv_physical_device_finish(struct bcmv_physical_device *device)
{
        bcmv_finish_wsi(device);
        ralloc_free(device->compiler);
        close(device->local_fd);
}

static void *
default_alloc_func(void *pUserData, size_t size, size_t align,
                   VkSystemAllocationScope allocationScope)
{
        return malloc(size);
}

static void *
default_realloc_func(void *pUserData, void *pOriginal, size_t size,
                     size_t align, VkSystemAllocationScope allocationScope)
{
        return realloc(pOriginal, size);
}

static void
default_free_func(void *pUserData, void *pMemory)
{
        free(pMemory);
}

static const VkAllocationCallbacks default_alloc = {
        .pUserData = NULL,
        .pfnAllocation = default_alloc_func,
        .pfnReallocation = default_realloc_func,
        .pfnFree = default_free_func,
};

VkResult bcmv_CreateInstance(const VkInstanceCreateInfo *pCreateInfo,
                             const VkAllocationCallbacks *pAllocator,
                             VkInstance *pInstance)
{
        struct bcmv_instance *instance;

        assert(pCreateInfo->sType == VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO);

        uint32_t client_version;
        if (pCreateInfo->pApplicationInfo &&
            pCreateInfo->pApplicationInfo->apiVersion != 0) {
                client_version = pCreateInfo->pApplicationInfo->apiVersion;
        } else {
                client_version = VK_MAKE_VERSION(1, 0, 0);
        }

        if (VK_MAKE_VERSION(1, 0, 0) > client_version ||
            client_version > VK_MAKE_VERSION(1, 0, 0xfff)) {
                return vk_errorf(VK_ERROR_INCOMPATIBLE_DRIVER,
                                 "Client requested version %d.%d.%d",
                                 VK_VERSION_MAJOR(client_version),
                                 VK_VERSION_MINOR(client_version),
                                 VK_VERSION_PATCH(client_version));
        }

        for (uint32_t i = 0; i < pCreateInfo->enabledExtensionCount; i++) {
                const char *ext_name = pCreateInfo->ppEnabledExtensionNames[i];
                if (!bcmv_instance_extension_supported(ext_name))
                        return vk_error(VK_ERROR_EXTENSION_NOT_PRESENT);
        }

        instance = vk_alloc2(&default_alloc, pAllocator, sizeof(*instance), 8,
                             VK_SYSTEM_ALLOCATION_SCOPE_INSTANCE);
        if (!instance)
                return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);

        instance->_loader_data.loaderMagic = ICD_LOADER_MAGIC;

        if (pAllocator)
                instance->alloc = *pAllocator;
        else
                instance->alloc = default_alloc;

        instance->apiVersion = client_version;
        instance->physicalDeviceCount = -1;

        _mesa_locale_init();

        VG(VALGRIND_CREATE_MEMPOOL(instance, 0, false));

        *pInstance = bcmv_instance_to_handle(instance);

        return VK_SUCCESS;
}

void bcmv_DestroyInstance(VkInstance _instance,
                          const VkAllocationCallbacks *pAllocator)
{
        BCMV_FROM_HANDLE(bcmv_instance, instance, _instance);

        if (!instance)
                return;

        if (instance->physicalDeviceCount > 0) {
                /* We support at most one physical device. */
                assert(instance->physicalDeviceCount == 1);
                bcmv_physical_device_finish(&instance->physicalDevice);
        }

        VG(VALGRIND_DESTROY_MEMPOOL(instance));

        _mesa_locale_fini();

        vk_free(&instance->alloc, instance);
}

static VkResult
bcmv_enumerate_devices(struct bcmv_instance *instance)
{
        /* TODO: Check for more devices ? */
        drmDevicePtr devices[8];
        VkResult result = VK_ERROR_INCOMPATIBLE_DRIVER;
        int max_devices;

        instance->physicalDeviceCount = 0;

        max_devices = drmGetDevices2(0, devices, ARRAY_SIZE(devices));
        if (max_devices < 1)
                return VK_ERROR_INCOMPATIBLE_DRIVER;

        (void)bcmv_physical_device_init;
#if 0 /* XXX */
        for (unsigned i = 0; i < (unsigned)max_devices; i++) {
                if (devices[i]->available_nodes & 1 << DRM_NODE_RENDER &&
                    devices[i]->bustype == DRM_BUS_PCI &&
                    devices[i]->deviceinfo.pci->vendor_id == 0x8086) {

                        result = bcmv_physical_device_init(&instance->physicalDevice,
                                                           instance,
                                                           devices[i]->nodes[DRM_NODE_RENDER]);
                        if (result != VK_ERROR_INCOMPATIBLE_DRIVER)
                                break;
                }
        }
#endif

        drmFreeDevices(devices, max_devices);

        if (result == VK_SUCCESS)
                instance->physicalDeviceCount = 1;

        return result;
}


VkResult bcmv_EnumeratePhysicalDevices(
        VkInstance                                  _instance,
        uint32_t*                                   pPhysicalDeviceCount,
        VkPhysicalDevice*                           pPhysicalDevices)
{
        BCMV_FROM_HANDLE(bcmv_instance, instance, _instance);
        VK_OUTARRAY_MAKE(out, pPhysicalDevices, pPhysicalDeviceCount);
        VkResult result;

        if (instance->physicalDeviceCount < 0) {
                result = bcmv_enumerate_devices(instance);
                if (result != VK_SUCCESS &&
                    result != VK_ERROR_INCOMPATIBLE_DRIVER)
                        return result;
        }

        if (instance->physicalDeviceCount > 0) {
                assert(instance->physicalDeviceCount == 1);
                vk_outarray_append(&out, i) {
                        *i = bcmv_physical_device_to_handle(&instance->physicalDevice);
                }
        }

        return vk_outarray_status(&out);
}

void bcmv_GetPhysicalDeviceFeatures(
        VkPhysicalDevice                            physicalDevice,
        VkPhysicalDeviceFeatures*                   pFeatures)
{
        *pFeatures = (VkPhysicalDeviceFeatures) {
                .robustBufferAccess                       = true,
                .fullDrawIndexUint32                      = true,
                .imageCubeArray                           = true,
                .independentBlend                         = true,
                .geometryShader                           = false,
                .tessellationShader                       = false,
                .sampleRateShading                        = true,
                .dualSrcBlend                             = false,
                .logicOp                                  = false,
                .multiDrawIndirect                        = true,
                .drawIndirectFirstInstance                = true,
                .depthClamp                               = true,
                .depthBiasClamp                           = true,
                .fillModeNonSolid                         = true,
                .depthBounds                              = false,
                .wideLines                                = true,
                .largePoints                              = true,
                .alphaToOne                               = true,
                .multiViewport                            = true,
                .samplerAnisotropy                        = true,
                .textureCompressionETC2                   = true,
                .textureCompressionASTC_LDR               = true,
                .textureCompressionBC                     = true,
                .occlusionQueryPrecise                    = true,
                .pipelineStatisticsQuery                  = true,
                .fragmentStoresAndAtomics                 = true,
                .shaderTessellationAndGeometryPointSize   = true,
                .shaderImageGatherExtended                = true,
                .shaderStorageImageExtendedFormats        = true,
                .shaderStorageImageMultisample            = false,
                .shaderStorageImageReadWithoutFormat      = false,
                .shaderStorageImageWriteWithoutFormat     = true,
                .shaderUniformBufferArrayDynamicIndexing  = true,
                .shaderSampledImageArrayDynamicIndexing   = true,
                .shaderStorageBufferArrayDynamicIndexing  = true,
                .shaderStorageImageArrayDynamicIndexing   = true,
                .shaderClipDistance                       = true,
                .shaderCullDistance                       = true,
                .shaderFloat64                            = false,
                .shaderInt64                              = false,
                .shaderInt16                              = false,
                .shaderResourceMinLod                     = false,
                .variableMultisampleRate                  = false,
                .inheritedQueries                         = true,
                .vertexPipelineStoresAndAtomics           = true,
        };
}

void bcmv_GetPhysicalDeviceFeatures2KHR(
        VkPhysicalDevice                            physicalDevice,
        VkPhysicalDeviceFeatures2KHR*               pFeatures)
{
        bcmv_GetPhysicalDeviceFeatures(physicalDevice, &pFeatures->features);

        vk_foreach_struct(ext, pFeatures->pNext) {
                switch (ext->sType) {
                case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MULTIVIEW_FEATURES_KHX: {
                        VkPhysicalDeviceMultiviewFeaturesKHX *features =
                                (VkPhysicalDeviceMultiviewFeaturesKHX *)ext;
                        features->multiview = true;
                        features->multiviewGeometryShader = true;
                        features->multiviewTessellationShader = true;
                        break;
                }

                case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VARIABLE_POINTER_FEATURES_KHR: {
                        VkPhysicalDeviceVariablePointerFeaturesKHR *features = (void *)ext;
                        features->variablePointersStorageBuffer = true;
                        features->variablePointers = false;
                        break;
                }

                default:
                        bcmv_debug_ignored_stype(ext->sType);
                        break;
                }
        }
}

void bcmv_GetPhysicalDeviceProperties(
        VkPhysicalDevice                            physicalDevice,
        VkPhysicalDeviceProperties*                 pProperties)
{
        BCMV_FROM_HANDLE(bcmv_physical_device, pdevice, physicalDevice);

        /* See assertions made when programming the buffer surface state. */
        const uint32_t max_raw_buffer_sz = (1ul << 30);

        const uint32_t max_samplers = 128;

        VkSampleCountFlags sample_counts = (VK_SAMPLE_COUNT_1_BIT |
                                            VK_SAMPLE_COUNT_4_BIT);

        VkPhysicalDeviceLimits limits = {
                .maxImageDimension1D                      = (1 << 14),
                .maxImageDimension2D                      = (1 << 14),
                .maxImageDimension3D                      = (1 << 11),
                .maxImageDimensionCube                    = (1 << 14),
                .maxImageArrayLayers                      = (1 << 11),
                .maxTexelBufferElements                   = 128 * 1024 * 1024,
                .maxUniformBufferRange                    = (1ul << 27),
                .maxStorageBufferRange                    = max_raw_buffer_sz,
                .maxPushConstantsSize                     = MAX_PUSH_CONSTANTS_SIZE,
                .maxMemoryAllocationCount                 = UINT32_MAX,
                .maxSamplerAllocationCount                = 64 * 1024,
                .bufferImageGranularity                   = 64, /* A cache line */
                .sparseAddressSpaceSize                   = 0,
                .maxBoundDescriptorSets                   = MAX_SETS,
                .maxPerStageDescriptorSamplers            = max_samplers,
                .maxPerStageDescriptorUniformBuffers      = 64,
                .maxPerStageDescriptorStorageBuffers      = 64,
                .maxPerStageDescriptorSampledImages       = max_samplers,
                .maxPerStageDescriptorStorageImages       = 64,
                .maxPerStageDescriptorInputAttachments    = 64,
                .maxPerStageResources                     = 250,
                .maxDescriptorSetSamplers                 = 256,
                .maxDescriptorSetUniformBuffers           = 256,
                .maxDescriptorSetUniformBuffersDynamic    = MAX_DYNAMIC_BUFFERS / 2,
                .maxDescriptorSetStorageBuffers           = 256,
                .maxDescriptorSetStorageBuffersDynamic    = MAX_DYNAMIC_BUFFERS / 2,
                .maxDescriptorSetSampledImages            = 256,
                .maxDescriptorSetStorageImages            = 256,
                .maxDescriptorSetInputAttachments         = 256,
                .maxVertexInputAttributes                 = MAX_VBS,
                .maxVertexInputBindings                   = MAX_VBS,
                .maxVertexInputAttributeOffset            = 2047,
                .maxVertexInputBindingStride              = 2048,
                .maxVertexOutputComponents                = 128,
                .maxTessellationGenerationLevel           = 64,
                .maxTessellationPatchSize                 = 32,
                .maxTessellationControlPerVertexInputComponents = 128,
                .maxTessellationControlPerVertexOutputComponents = 128,
                .maxTessellationControlPerPatchOutputComponents = 128,
                .maxTessellationControlTotalOutputComponents = 2048,
                .maxTessellationEvaluationInputComponents = 128,
                .maxTessellationEvaluationOutputComponents = 128,
                .maxGeometryShaderInvocations             = 32,
                .maxGeometryInputComponents               = 64,
                .maxGeometryOutputComponents              = 128,
                .maxGeometryOutputVertices                = 256,
                .maxGeometryTotalOutputComponents         = 1024,
                .maxFragmentInputComponents               = 128,
                .maxFragmentOutputAttachments             = 8,
                .maxFragmentDualSrcAttachments            = 1,
                .maxFragmentCombinedOutputResources       = 8,
                .maxComputeSharedMemorySize               = 32768,
                .maxComputeWorkGroupCount                 = { 65535, 65535, 65535 },
                .maxComputeWorkGroupInvocations           = 16 * 8, /* XXX */
                .maxComputeWorkGroupSize = {
                        16 * 8,
                        16 * 8,
                        16 * 8, /* XXX */
                },
                .subPixelPrecisionBits                    = 4 /* FIXME */,
                .subTexelPrecisionBits                    = 4 /* FIXME */,
                .mipmapPrecisionBits                      = 4 /* FIXME */,
                .maxDrawIndexedIndexValue                 = UINT32_MAX,
                .maxDrawIndirectCount                     = UINT32_MAX,
                .maxSamplerLodBias                        = 16,
                .maxSamplerAnisotropy                     = 16,
                .maxViewports                             = MAX_VIEWPORTS,
                .maxViewportDimensions                    = { (1 << 14), (1 << 14) },
                .viewportBoundsRange                      = { INT16_MIN, INT16_MAX },
                .viewportSubPixelBits                     = 13, /* We take a float? */
                .minMemoryMapAlignment                    = 4096, /* A page */
                .minTexelBufferOffsetAlignment            = 1,
                .minUniformBufferOffsetAlignment          = 16,
                .minStorageBufferOffsetAlignment          = 4,
                .minTexelOffset                           = -8,
                .maxTexelOffset                           = 7,
                .minTexelGatherOffset                     = -32,
                .maxTexelGatherOffset                     = 31,
                .minInterpolationOffset                   = -0.5,
                .maxInterpolationOffset                   = 0.4375,
                .subPixelInterpolationOffsetBits          = 4,
                .maxFramebufferWidth                      = (1 << 14),
                .maxFramebufferHeight                     = (1 << 14),
                .maxFramebufferLayers                     = (1 << 11),
                .framebufferColorSampleCounts             = sample_counts,
                .framebufferDepthSampleCounts             = sample_counts,
                .framebufferStencilSampleCounts           = sample_counts,
                .framebufferNoAttachmentsSampleCounts     = sample_counts,
                .maxColorAttachments                      = MAX_RTS,
                .sampledImageColorSampleCounts            = sample_counts,
                .sampledImageIntegerSampleCounts          = VK_SAMPLE_COUNT_1_BIT,
                .sampledImageDepthSampleCounts            = sample_counts,
                .sampledImageStencilSampleCounts          = sample_counts,
                .storageImageSampleCounts                 = VK_SAMPLE_COUNT_1_BIT,
                .maxSampleMaskWords                       = 1,
                .timestampComputeAndGraphics              = false,
#if 0 /* XXX */
                .timestampPeriod                          = 1000000000.0 / devinfo->timestamp_frequency,
#endif
                .maxClipDistances                         = 8,
                .maxCullDistances                         = 8,
                .maxCombinedClipAndCullDistances          = 8,
                .discreteQueuePriorities                  = 1,
                .pointSizeRange                           = { 0.125, 255.875 },
                .lineWidthRange                           = { 0.0, 7.9921875 },
                .pointSizeGranularity                     = (1.0 / 8.0),
                .lineWidthGranularity                     = (1.0 / 128.0),
                .strictLines                              = false, /* FINISHME */
                .standardSampleLocations                  = true,
                .optimalBufferCopyOffsetAlignment         = 128,
                .optimalBufferCopyRowPitchAlignment       = 128,
                .nonCoherentAtomSize                      = 64,
        };

        *pProperties = (VkPhysicalDeviceProperties) {
                .apiVersion = bcmv_physical_device_api_version(pdevice),
                .driverVersion = vk_get_driver_version(),
                .vendorID = 0x8086,
                .deviceID = pdevice->chipset_id,
                .deviceType = VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU,
                .limits = limits,
                .sparseProperties = {0}, /* Broadwell doesn't do sparse. */
        };

        snprintf(pProperties->deviceName, sizeof(pProperties->deviceName),
                 "%s", pdevice->name);
        memcpy(pProperties->pipelineCacheUUID,
               pdevice->pipeline_cache_uuid, VK_UUID_SIZE);
}

void bcmv_GetPhysicalDeviceProperties2KHR(
        VkPhysicalDevice                            physicalDevice,
        VkPhysicalDeviceProperties2KHR*             pProperties)
{
        BCMV_FROM_HANDLE(bcmv_physical_device, pdevice, physicalDevice);

        bcmv_GetPhysicalDeviceProperties(physicalDevice, &pProperties->properties);

        vk_foreach_struct(ext, pProperties->pNext) {
                switch (ext->sType) {
                case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PUSH_DESCRIPTOR_PROPERTIES_KHR: {
                        VkPhysicalDevicePushDescriptorPropertiesKHR *properties =
                                (VkPhysicalDevicePushDescriptorPropertiesKHR *) ext;

                        properties->maxPushDescriptors = MAX_PUSH_DESCRIPTORS;
                        break;
                }

                case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ID_PROPERTIES_KHR: {
                        VkPhysicalDeviceIDPropertiesKHR *id_props =
                                (VkPhysicalDeviceIDPropertiesKHR *)ext;
                        memcpy(id_props->deviceUUID, pdevice->device_uuid, VK_UUID_SIZE);
                        memcpy(id_props->driverUUID, pdevice->driver_uuid, VK_UUID_SIZE);
                        /* The LUID is for Windows. */
                        id_props->deviceLUIDValid = false;
                        break;
                }

                case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MULTIVIEW_PROPERTIES_KHX: {
                        VkPhysicalDeviceMultiviewPropertiesKHX *properties =
                                (VkPhysicalDeviceMultiviewPropertiesKHX *)ext;
                        properties->maxMultiviewViewCount = 16;
                        properties->maxMultiviewInstanceIndex = UINT32_MAX / 16;
                        break;
                }

                default:
                        bcmv_debug_ignored_stype(ext->sType);
                        break;
                }
        }
}

/* We support exactly one queue family for now.  Should expose the TFU later. */
static const VkQueueFamilyProperties
bcmv_queue_family_properties = {
        .queueFlags = VK_QUEUE_GRAPHICS_BIT |
        VK_QUEUE_COMPUTE_BIT |
        VK_QUEUE_TRANSFER_BIT,
        .queueCount = 1,
        .timestampValidBits = 36, /* XXX: Real value here */
        .minImageTransferGranularity = { 1, 1, 1 },
};

void bcmv_GetPhysicalDeviceQueueFamilyProperties(
        VkPhysicalDevice                            physicalDevice,
        uint32_t*                                   pCount,
        VkQueueFamilyProperties*                    pQueueFamilyProperties)
{
        VK_OUTARRAY_MAKE(out, pQueueFamilyProperties, pCount);

        vk_outarray_append(&out, p) {
                *p = bcmv_queue_family_properties;
        }
}

void bcmv_GetPhysicalDeviceQueueFamilyProperties2KHR(
        VkPhysicalDevice                            physicalDevice,
        uint32_t*                                   pQueueFamilyPropertyCount,
        VkQueueFamilyProperties2KHR*                pQueueFamilyProperties)
{

        VK_OUTARRAY_MAKE(out, pQueueFamilyProperties, pQueueFamilyPropertyCount);

        vk_outarray_append(&out, p) {
                p->queueFamilyProperties = bcmv_queue_family_properties;

                vk_foreach_struct(s, p->pNext) {
                        bcmv_debug_ignored_stype(s->sType);
                }
        }
}

void bcmv_GetPhysicalDeviceMemoryProperties(
        VkPhysicalDevice                            physicalDevice,
        VkPhysicalDeviceMemoryProperties*           pMemoryProperties)
{
        BCMV_FROM_HANDLE(bcmv_physical_device, physical_device, physicalDevice);

        pMemoryProperties->memoryTypeCount = physical_device->memory.type_count;
        for (uint32_t i = 0; i < physical_device->memory.type_count; i++) {
                pMemoryProperties->memoryTypes[i] = (VkMemoryType) {
                        .propertyFlags = physical_device->memory.types[i].propertyFlags,
                        .heapIndex     = physical_device->memory.types[i].heapIndex,
                };
        }

        pMemoryProperties->memoryHeapCount = physical_device->memory.heap_count;
        for (uint32_t i = 0; i < physical_device->memory.heap_count; i++) {
                pMemoryProperties->memoryHeaps[i] = (VkMemoryHeap) {
                        .size    = physical_device->memory.heaps[i].size,
                        .flags   = physical_device->memory.heaps[i].flags,
                };
        }
}

void bcmv_GetPhysicalDeviceMemoryProperties2KHR(
        VkPhysicalDevice                            physicalDevice,
        VkPhysicalDeviceMemoryProperties2KHR*       pMemoryProperties)
{
        bcmv_GetPhysicalDeviceMemoryProperties(physicalDevice,
                                               &pMemoryProperties->memoryProperties);

        vk_foreach_struct(ext, pMemoryProperties->pNext) {
                switch (ext->sType) {
                default:
                        bcmv_debug_ignored_stype(ext->sType);
                        break;
                }
        }
}

PFN_vkVoidFunction bcmv_GetInstanceProcAddr(
        VkInstance                                  instance,
        const char*                                 pName)
{
        return bcmv_lookup_entrypoint(NULL, pName);
}

/* With version 1+ of the loader interface the ICD should expose
 * vk_icdGetInstanceProcAddr to work around certain LD_PRELOAD issues seen in
 * apps.
 */
PUBLIC
VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL vk_icdGetInstanceProcAddr(
        VkInstance                                  instance,
        const char*                                 pName);

PUBLIC
VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL vk_icdGetInstanceProcAddr(
        VkInstance                                  instance,
        const char*                                 pName)
{
        return bcmv_GetInstanceProcAddr(instance, pName);
}

PFN_vkVoidFunction bcmv_GetDeviceProcAddr(
        VkDevice                                    _device,
        const char*                                 pName)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        return bcmv_lookup_entrypoint(&device->info, pName);
}

static void
bcmv_queue_init(struct bcmv_device *device, struct bcmv_queue *queue)
{
        queue->_loader_data.loaderMagic = ICD_LOADER_MAGIC;
        queue->device = device;
        queue->pool = &device->surface_state_pool;
}

static void
bcmv_queue_finish(struct bcmv_queue *queue)
{
}

#if 0 /* XXX */
static struct bcmv_state
bcmv_state_pool_emit_data(struct bcmv_state_pool *pool, size_t size, size_t align, const void *p)
{
        struct bcmv_state state;

        state = bcmv_state_pool_alloc(pool, size, align);
        memcpy(state.map, p, size);

        bcmv_state_flush(pool->block_pool.device, state);

        return state;
}

struct gen8_border_color {
        union {
                float float32[4];
                uint32_t uint32[4];
        };
        /* Pad out to 64 bytes */
        uint32_t _pad[12];
};
#endif

static void
bcmv_device_init_border_colors(struct bcmv_device *device)
{
#if 0 /* XXX */
        static const struct gen8_border_color border_colors[] = {
                [VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK] =  { .float32 = { 0.0, 0.0, 0.0, 0.0 } },
                [VK_BORDER_COLOR_FLOAT_OPAQUE_BLACK] =       { .float32 = { 0.0, 0.0, 0.0, 1.0 } },
                [VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE] =       { .float32 = { 1.0, 1.0, 1.0, 1.0 } },
                [VK_BORDER_COLOR_INT_TRANSPARENT_BLACK] =    { .uint32 = { 0, 0, 0, 0 } },
                [VK_BORDER_COLOR_INT_OPAQUE_BLACK] =         { .uint32 = { 0, 0, 0, 1 } },
                [VK_BORDER_COLOR_INT_OPAQUE_WHITE] =         { .uint32 = { 1, 1, 1, 1 } },
        };

        device->border_colors = bcmv_state_pool_emit_data(&device->dynamic_state_pool,
                                                          sizeof(border_colors), 64,
                                                          border_colors);
#endif
}

static void
bcmv_device_init_trivial_batch(struct bcmv_device *device)
{
#if 0 /* XXX */
        bcmv_bo_init_new(&device->trivial_batch_bo, device, 4096);

        if (device->instance->physicalDevice.has_exec_async)
                device->trivial_batch_bo.flags |= EXEC_OBJECT_ASYNC;

        void *map = bcmv_gem_mmap(device, device->trivial_batch_bo.gem_handle,
                                  4096);

        struct bcmv_batch batch = {
                .start = map,
                .next = map,
                .end = map + 4096,
        };

        bcmv_rcl_emit(&batch, V3D_HALT, noop);

        munmap(map, device->trivial_batch_bo.size);
#endif
}

VkResult bcmv_CreateDevice(
        VkPhysicalDevice                            physicalDevice,
        const VkDeviceCreateInfo*                   pCreateInfo,
        const VkAllocationCallbacks*                pAllocator,
        VkDevice*                                   pDevice)
{
        BCMV_FROM_HANDLE(bcmv_physical_device, physical_device, physicalDevice);
        VkResult result;
        struct bcmv_device *device;

        assert(pCreateInfo->sType == VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO);

        for (uint32_t i = 0; i < pCreateInfo->enabledExtensionCount; i++) {
                const char *ext_name = pCreateInfo->ppEnabledExtensionNames[i];
                if (!bcmv_physical_device_extension_supported(physical_device, ext_name))
                        return vk_error(VK_ERROR_EXTENSION_NOT_PRESENT);
        }

        /* Check enabled features */
        if (pCreateInfo->pEnabledFeatures) {
                VkPhysicalDeviceFeatures supported_features;
                bcmv_GetPhysicalDeviceFeatures(physicalDevice, &supported_features);
                VkBool32 *supported_feature = (VkBool32 *)&supported_features;
                VkBool32 *enabled_feature = (VkBool32 *)pCreateInfo->pEnabledFeatures;
                unsigned num_features = sizeof(VkPhysicalDeviceFeatures) / sizeof(VkBool32);
                for (uint32_t i = 0; i < num_features; i++) {
                        if (enabled_feature[i] && !supported_feature[i])
                                return vk_error(VK_ERROR_FEATURE_NOT_PRESENT);
                }
        }

        device = vk_alloc2(&physical_device->instance->alloc, pAllocator,
                           sizeof(*device), 8,
                           VK_SYSTEM_ALLOCATION_SCOPE_DEVICE);
        if (!device)
                return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);

        device->_loader_data.loaderMagic = ICD_LOADER_MAGIC;
        device->instance = physical_device->instance;
        device->chipset_id = physical_device->chipset_id;
        device->lost = false;

        if (pAllocator)
                device->alloc = *pAllocator;
        else
                device->alloc = physical_device->instance->alloc;

        /* XXX(chadv): Can we dup() physicalDevice->fd here? */
        device->fd = open(physical_device->path, O_RDWR | O_CLOEXEC);
        if (device->fd == -1) {
                result = vk_error(VK_ERROR_INITIALIZATION_FAILED);
                goto fail_device;
        }

        device->info = physical_device->info;
        // XXX device->isl_dev = physical_device->isl_dev;

        if (pthread_mutex_init(&device->mutex, NULL) != 0) {
                result = vk_error(VK_ERROR_INITIALIZATION_FAILED);
                goto fail_fd;
        }

        pthread_condattr_t condattr;
        if (pthread_condattr_init(&condattr) != 0) {
                result = vk_error(VK_ERROR_INITIALIZATION_FAILED);
                goto fail_mutex;
        }
        if (pthread_condattr_setclock(&condattr, CLOCK_MONOTONIC) != 0) {
                pthread_condattr_destroy(&condattr);
                result = vk_error(VK_ERROR_INITIALIZATION_FAILED);
                goto fail_mutex;
        }
        if (pthread_cond_init(&device->queue_submit, NULL) != 0) {
                pthread_condattr_destroy(&condattr);
                result = vk_error(VK_ERROR_INITIALIZATION_FAILED);
                goto fail_mutex;
        }
        pthread_condattr_destroy(&condattr);

        bcmv_bo_pool_init(&device->batch_bo_pool, device);

        result = bcmv_bo_cache_init(&device->bo_cache);
        if (result != VK_SUCCESS)
                goto fail_batch_bo_pool;

#if 0 /* XXX */
        result = bcmv_state_pool_init(&device->dynamic_state_pool, device, 16384);
        if (result != VK_SUCCESS)
                goto fail_bo_cache;

        result = bcmv_state_pool_init(&device->instruction_state_pool, device, 16384);
        if (result != VK_SUCCESS)
                goto fail_dynamic_state_pool;

        result = bcmv_state_pool_init(&device->surface_state_pool, device, 4096);
        if (result != VK_SUCCESS)
                goto fail_instruction_state_pool;
#endif

        result = bcmv_bo_init_new(&device->workaround_bo, device, 1024);
        if (result != VK_SUCCESS)
                goto fail_surface_state_pool;

        bcmv_device_init_trivial_batch(device);

        bcmv_scratch_pool_init(device, &device->scratch_pool);

        bcmv_queue_init(device, &device->queue);

        /* XXX: any init_device_state? */
        if (0)
                goto fail_workaround_bo;

        // XXX bcmv_device_init_blorp(device);

        bcmv_device_init_border_colors(device);

        *pDevice = bcmv_device_to_handle(device);

        return VK_SUCCESS;

 fail_workaround_bo:
        bcmv_queue_finish(&device->queue);
        bcmv_scratch_pool_finish(device, &device->scratch_pool);
        bcmv_gem_munmap(device->workaround_bo.map, device->workaround_bo.size);
        bcmv_gem_close(device, device->workaround_bo.gem_handle);
 fail_surface_state_pool:
#if 0 /* XXX */
        bcmv_state_pool_finish(&device->surface_state_pool);
 fail_instruction_state_pool:
        bcmv_state_pool_finish(&device->instruction_state_pool);
 fail_dynamic_state_pool:
        bcmv_state_pool_finish(&device->dynamic_state_pool);
#endif
        bcmv_bo_cache_finish(&device->bo_cache);
 fail_batch_bo_pool:
        bcmv_bo_pool_finish(&device->batch_bo_pool);
        pthread_cond_destroy(&device->queue_submit);
 fail_mutex:
        pthread_mutex_destroy(&device->mutex);
 fail_fd:
        close(device->fd);
 fail_device:
        vk_free(&device->alloc, device);

        return result;
}

void bcmv_DestroyDevice(
        VkDevice                                    _device,
        const VkAllocationCallbacks*                pAllocator)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);

        if (!device)
                return;

        bcmv_queue_finish(&device->queue);

        bcmv_scratch_pool_finish(device, &device->scratch_pool);

        munmap(device->workaround_bo.map, device->workaround_bo.size);
        bcmv_gem_close(device, device->workaround_bo.gem_handle);

        bcmv_gem_close(device, device->trivial_batch_bo.gem_handle);

#if 0 /* XXX */
        bcmv_state_pool_finish(&device->surface_state_pool);
        bcmv_state_pool_finish(&device->instruction_state_pool);
        bcmv_state_pool_finish(&device->dynamic_state_pool);
#endif

        bcmv_bo_cache_finish(&device->bo_cache);

        bcmv_bo_pool_finish(&device->batch_bo_pool);

        pthread_cond_destroy(&device->queue_submit);
        pthread_mutex_destroy(&device->mutex);

        close(device->fd);

        vk_free(&device->alloc, device);
}

VkResult bcmv_EnumerateInstanceLayerProperties(
        uint32_t*                                   pPropertyCount,
        VkLayerProperties*                          pProperties)
{
        if (pProperties == NULL) {
                *pPropertyCount = 0;
                return VK_SUCCESS;
        }

        /* None supported at this time */
        return vk_error(VK_ERROR_LAYER_NOT_PRESENT);
}

VkResult bcmv_EnumerateDeviceLayerProperties(
        VkPhysicalDevice                            physicalDevice,
        uint32_t*                                   pPropertyCount,
        VkLayerProperties*                          pProperties)
{
        if (pProperties == NULL) {
                *pPropertyCount = 0;
                return VK_SUCCESS;
        }

        /* None supported at this time */
        return vk_error(VK_ERROR_LAYER_NOT_PRESENT);
}

void bcmv_GetDeviceQueue(
        VkDevice                                    _device,
        uint32_t                                    queueNodeIndex,
        uint32_t                                    queueIndex,
        VkQueue*                                    pQueue)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);

        assert(queueIndex == 0);

        *pQueue = bcmv_queue_to_handle(&device->queue);
}

VkResult
bcmv_device_query_status(struct bcmv_device *device)
{
        /* This isn't likely as most of the callers of this function already check
         * for it.  However, it doesn't hurt to check and it potentially lets us
         * avoid an ioctl.
         */
        if (unlikely(device->lost))
                return VK_ERROR_DEVICE_LOST;

#if 0 /* XXX */
        uint32_t active, pending;
        int ret = bcmv_gem_gpu_get_reset_stats(device, &active, &pending);
        if (ret == -1) {
                /* We don't know the real error. */
                device->lost = true;
                return vk_errorf(VK_ERROR_DEVICE_LOST, "get_reset_stats failed: %m");
        }

        if (active) {
                device->lost = true;
                return vk_errorf(VK_ERROR_DEVICE_LOST,
                                 "GPU hung on one of our command buffers");
        } else if (pending) {
                device->lost = true;
                return vk_errorf(VK_ERROR_DEVICE_LOST,
                                 "GPU hung with commands in-flight");
        }
#endif

        return VK_SUCCESS;
}

VkResult
bcmv_device_bo_busy(struct bcmv_device *device, struct bcmv_bo *bo)
{
        /* Note:  This only returns whether or not the BO is in use by an i915 GPU.
         * Other usages of the BO (such as on different hardware) will not be
         * flagged as "busy" by this ioctl.  Use with care.
         */
        int ret = bcmv_gem_busy(device, bo->gem_handle);
        if (ret == 1) {
                return VK_NOT_READY;
        } else if (ret == -1) {
                /* We don't know the real error. */
                device->lost = true;
                return vk_errorf(VK_ERROR_DEVICE_LOST, "gem wait failed: %m");
        }

        /* Query for device status after the busy call.  If the BO we're checking
         * got caught in a GPU hang we don't want to return VK_SUCCESS to the
         * client because it clearly doesn't have valid data.  Yes, this most
         * likely means an ioctl, but we just did an ioctl to query the busy status
         * so it's no great loss.
         */
        return bcmv_device_query_status(device);
}

VkResult
bcmv_device_wait(struct bcmv_device *device, struct bcmv_bo *bo,
                 int64_t timeout)
{
        int ret = bcmv_gem_wait(device, bo->gem_handle, &timeout);
        if (ret == -1 && errno == ETIME) {
                return VK_TIMEOUT;
        } else if (ret == -1) {
                /* We don't know the real error. */
                device->lost = true;
                return vk_errorf(VK_ERROR_DEVICE_LOST, "gem wait failed: %m");
        }

        /* Query for device status after the wait.  If the BO we're waiting on got
         * caught in a GPU hang we don't want to return VK_SUCCESS to the client
         * because it clearly doesn't have valid data.  Yes, this most likely means
         * an ioctl, but we just did an ioctl to wait so it's no great loss.
         */
        return bcmv_device_query_status(device);
}

VkResult bcmv_DeviceWaitIdle(
        VkDevice                                    _device)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        if (unlikely(device->lost))
                return VK_ERROR_DEVICE_LOST;

#if 0 /* XXX */
        struct bcmv_batch batch;

        uint32_t cmds[8];
        batch.start = batch.next = cmds;
        batch.end = (void *) cmds + sizeof(cmds);

        bcmv_batch_emit(&batch, GEN7_MI_BATCH_BUFFER_END, bbe);
        bcmv_batch_emit(&batch, GEN7_MI_NOOP, noop);

        return bcmv_device_submit_simple_batch(device, &batch);
#endif
        return VK_SUCCESS;
}

VkResult
bcmv_bo_init_new(struct bcmv_bo *bo, struct bcmv_device *device, uint64_t size)
{
        uint32_t gem_handle = bcmv_gem_create(device, size);
        if (!gem_handle)
                return vk_error(VK_ERROR_OUT_OF_DEVICE_MEMORY);

        bcmv_bo_init(bo, gem_handle, size);

        return VK_SUCCESS;
}

VkResult bcmv_AllocateMemory(
        VkDevice                                    _device,
        const VkMemoryAllocateInfo*                 pAllocateInfo,
        const VkAllocationCallbacks*                pAllocator,
        VkDeviceMemory*                             pMem)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        struct bcmv_physical_device *pdevice = &device->instance->physicalDevice;
        struct bcmv_device_memory *mem;
        VkResult result = VK_SUCCESS;

        assert(pAllocateInfo->sType == VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO);

        /* The Vulkan 1.0.33 spec says "allocationSize must be greater than
         * 0".
         */
        assert(pAllocateInfo->allocationSize > 0);

        /* FINISHME: Fail if allocation request exceeds heap size. */

        mem = vk_alloc2(&device->alloc, pAllocator, sizeof(*mem), 8,
                        VK_SYSTEM_ALLOCATION_SCOPE_OBJECT);
        if (mem == NULL)
                return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);

        assert(pAllocateInfo->memoryTypeIndex < pdevice->memory.type_count);
        mem->type = &pdevice->memory.types[pAllocateInfo->memoryTypeIndex];
        mem->map = NULL;
        mem->map_size = 0;

        const VkImportMemoryFdInfoKHR *fd_info =
                vk_find_struct_const(pAllocateInfo->pNext,
                                     IMPORT_MEMORY_FD_INFO_KHR);

        /* The Vulkan spec permits handleType to be 0, in which case the
         * struct is ignored.
         */
        if (fd_info && fd_info->handleType) {
                /* At the moment, we only support the OPAQUE_FD memory type
                 * which is just a GEM buffer.
                 */
                assert(fd_info->handleType ==
                       VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR);

                result = bcmv_bo_cache_import(device, &device->bo_cache,
                                              fd_info->fd, pAllocateInfo->allocationSize,
                                              &mem->bo);
                if (result != VK_SUCCESS)
                        goto fail;
        } else {
                result = bcmv_bo_cache_alloc(device, &device->bo_cache,
                                             pAllocateInfo->allocationSize,
                                             &mem->bo);
                if (result != VK_SUCCESS)
                        goto fail;
        }

        assert(mem->type->heapIndex < pdevice->memory.heap_count);

        *pMem = bcmv_device_memory_to_handle(mem);

        return VK_SUCCESS;

 fail:
        vk_free2(&device->alloc, pAllocator, mem);

        return result;
}

VkResult bcmv_GetMemoryFdKHR(
        VkDevice                                    device_h,
        const VkMemoryGetFdInfoKHR*                 pGetFdInfo,
        int*                                        pFd)
{
        BCMV_FROM_HANDLE(bcmv_device, dev, device_h);
        BCMV_FROM_HANDLE(bcmv_device_memory, mem, pGetFdInfo->memory);

        assert(pGetFdInfo->sType == VK_STRUCTURE_TYPE_MEMORY_GET_FD_INFO_KHR);

        /* We support only one handle type. */
        assert(pGetFdInfo->handleType ==
               VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR);

        return bcmv_bo_cache_export(dev, &dev->bo_cache, mem->bo, pFd);
}

VkResult bcmv_GetMemoryFdPropertiesKHR(
        VkDevice                                    device_h,
        VkExternalMemoryHandleTypeFlagBitsKHR       handleType,
        int                                         fd,
        VkMemoryFdPropertiesKHR*                    pMemoryFdProperties)
{
        /* The valid usage section for this function says:
         *
         *    "handleType must not be one of the handle types defined as opaque."
         *
         * Since we only handle opaque handles for now, there are no FD properties.
         */
        return VK_ERROR_INVALID_EXTERNAL_HANDLE_KHR;
}

void bcmv_FreeMemory(
        VkDevice                                    _device,
        VkDeviceMemory                              _mem,
        const VkAllocationCallbacks*                pAllocator)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        BCMV_FROM_HANDLE(bcmv_device_memory, mem, _mem);

        if (mem == NULL)
                return;

        if (mem->map)
                bcmv_UnmapMemory(_device, _mem);

        bcmv_bo_cache_release(device, &device->bo_cache, mem->bo);

        vk_free2(&device->alloc, pAllocator, mem);
}

VkResult bcmv_MapMemory(
        VkDevice                                    _device,
        VkDeviceMemory                              _memory,
        VkDeviceSize                                offset,
        VkDeviceSize                                size,
        VkMemoryMapFlags                            flags,
        void**                                      ppData)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        BCMV_FROM_HANDLE(bcmv_device_memory, mem, _memory);

        if (mem == NULL) {
                *ppData = NULL;
                return VK_SUCCESS;
        }

        if (size == VK_WHOLE_SIZE)
                size = mem->bo->size - offset;

        /* From the Vulkan spec version 1.0.32 docs for MapMemory:
         *
         *  * If size is not equal to VK_WHOLE_SIZE, size must be greater than 0
         *    assert(size != 0);
         *  * If size is not equal to VK_WHOLE_SIZE, size must be less than or
         *    equal to the size of the memory minus offset
         */
        assert(size > 0);
        assert(offset + size <= mem->bo->size);

        /* FIXME: Is this supposed to be thread safe? Since vkUnmapMemory()
         * only takes a VkDeviceMemory pointer, it seems like only one map of
         * the memory at a time is valid. We could just mmap up front and
         * return an offset pointer here, but that may exhaust virtual memory
         * on 32 bit userspace.
         */

        // uint32_t gem_flags = 0;

        /* XXX
        if (mem->type->propertyFlags & VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)
                gem_flags |= I915_MMAP_WC;
        */

        /* GEM will fail to map if the offset isn't 4k-aligned.  Round down. */
        uint64_t map_offset = offset & ~4095ull;
        assert(offset >= map_offset);
        uint64_t map_size = (offset + size) - map_offset;

        /* Let's map whole pages */
        map_size = align_u64(map_size, 4096);

        void *map = bcmv_gem_mmap(device, mem->bo->gem_handle,
                                  /* XXX map_offset, */map_size /* XXX, gem_flags */);
        if (map == MAP_FAILED)
                return vk_error(VK_ERROR_MEMORY_MAP_FAILED);

        mem->map = map;
        mem->map_size = map_size;

        *ppData = mem->map + (offset - map_offset);

        return VK_SUCCESS;
}

void bcmv_UnmapMemory(VkDevice _device,
                      VkDeviceMemory _memory)
{
        BCMV_FROM_HANDLE(bcmv_device_memory, mem, _memory);

        if (mem == NULL)
                return;

        munmap(mem->map, mem->map_size);

        mem->map = NULL;
        mem->map_size = 0;
}

static void
clflush_mapped_ranges(struct bcmv_device         *device,
                      uint32_t                   count,
                      const VkMappedMemoryRange *ranges)
{
        for (uint32_t i = 0; i < count; i++) {
                BCMV_FROM_HANDLE(bcmv_device_memory, mem, ranges[i].memory);
                if (ranges[i].offset >= mem->map_size)
                        continue;

                /* XXX
                gen_clflush_range(mem->map + ranges[i].offset,
                                  MIN2(ranges[i].size, mem->map_size - ranges[i].offset));
                */
        }
}

VkResult bcmv_FlushMappedMemoryRanges(
        VkDevice                                    _device,
        uint32_t                                    memoryRangeCount,
        const VkMappedMemoryRange*                  pMemoryRanges)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);

        /* Make sure the writes we're flushing have landed. */
        // XXX __builtin_ia32_mfence();

        clflush_mapped_ranges(device, memoryRangeCount, pMemoryRanges);

        return VK_SUCCESS;
}

VkResult bcmv_InvalidateMappedMemoryRanges(
        VkDevice                                    _device,
        uint32_t                                    memoryRangeCount,
        const VkMappedMemoryRange*                  pMemoryRanges)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);

        clflush_mapped_ranges(device, memoryRangeCount, pMemoryRanges);

        /* Make sure no reads get moved up above the invalidate. */
        // XXX __builtin_ia32_mfence();

        return VK_SUCCESS;
}

void bcmv_GetBufferMemoryRequirements(
        VkDevice                                    _device,
        VkBuffer                                    _buffer,
        VkMemoryRequirements*                       pMemoryRequirements)
{
        BCMV_FROM_HANDLE(bcmv_buffer, buffer, _buffer);
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        struct bcmv_physical_device *pdevice = &device->instance->physicalDevice;

        /* The Vulkan spec (git aaed022) says:
         *
         *    memoryTypeBits is a bitfield and contains one bit set for every
         *    supported memory type for the resource. The bit `1<<i` is set if and
         *    only if the memory type `i` in the VkPhysicalDeviceMemoryProperties
         *    structure for the physical device is supported.
         */
        uint32_t memory_types = 0;
        for (uint32_t i = 0; i < pdevice->memory.type_count; i++) {
                uint32_t valid_usage = pdevice->memory.types[i].valid_buffer_usage;
                if ((valid_usage & buffer->usage) == buffer->usage)
                        memory_types |= (1u << i);
        }

        pMemoryRequirements->size = buffer->size;
        pMemoryRequirements->alignment = 16;
        pMemoryRequirements->memoryTypeBits = memory_types;
}

void bcmv_GetBufferMemoryRequirements2KHR(
        VkDevice                                    _device,
        const VkBufferMemoryRequirementsInfo2KHR*   pInfo,
        VkMemoryRequirements2KHR*                   pMemoryRequirements)
{
        bcmv_GetBufferMemoryRequirements(_device, pInfo->buffer,
                                         &pMemoryRequirements->memoryRequirements);

        vk_foreach_struct(ext, pMemoryRequirements->pNext) {
                switch (ext->sType) {
                case VK_STRUCTURE_TYPE_MEMORY_DEDICATED_REQUIREMENTS_KHR: {
                        VkMemoryDedicatedRequirementsKHR *requirements = (void *)ext;
                        requirements->prefersDedicatedAllocation = VK_FALSE;
                        requirements->requiresDedicatedAllocation = VK_FALSE;
                        break;
                }

                default:
                        bcmv_debug_ignored_stype(ext->sType);
                        break;
                }
        }
}

void bcmv_GetImageMemoryRequirements(
        VkDevice                                    _device,
        VkImage                                     _image,
        VkMemoryRequirements*                       pMemoryRequirements)
{
        BCMV_FROM_HANDLE(bcmv_image, image, _image);
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        struct bcmv_physical_device *pdevice = &device->instance->physicalDevice;

        /* The Vulkan spec (git aaed022) says:
         *
         *    memoryTypeBits is a bitfield and contains one bit set for every
         *    supported memory type for the resource. The bit `1<<i` is set if and
         *    only if the memory type `i` in the VkPhysicalDeviceMemoryProperties
         *    structure for the physical device is supported.
         *
         * All types are currently supported for images.
         */
        uint32_t memory_types = (1ull << pdevice->memory.type_count) - 1;

        pMemoryRequirements->size = image->size;
        pMemoryRequirements->alignment = image->alignment;
        pMemoryRequirements->memoryTypeBits = memory_types;
}

void bcmv_GetImageMemoryRequirements2KHR(
        VkDevice                                    _device,
        const VkImageMemoryRequirementsInfo2KHR*    pInfo,
        VkMemoryRequirements2KHR*                   pMemoryRequirements)
{
        bcmv_GetImageMemoryRequirements(_device, pInfo->image,
                                        &pMemoryRequirements->memoryRequirements);

        vk_foreach_struct(ext, pMemoryRequirements->pNext) {
                switch (ext->sType) {
                case VK_STRUCTURE_TYPE_MEMORY_DEDICATED_REQUIREMENTS_KHR: {
                        VkMemoryDedicatedRequirementsKHR *requirements = (void *)ext;
                        requirements->prefersDedicatedAllocation = VK_FALSE;
                        requirements->requiresDedicatedAllocation = VK_FALSE;
                        break;
                }

                default:
                        bcmv_debug_ignored_stype(ext->sType);
                        break;
                }
        }
}

void bcmv_GetImageSparseMemoryRequirements(
        VkDevice                                    device,
        VkImage                                     image,
        uint32_t*                                   pSparseMemoryRequirementCount,
        VkSparseImageMemoryRequirements*            pSparseMemoryRequirements)
{
        *pSparseMemoryRequirementCount = 0;
}

void bcmv_GetImageSparseMemoryRequirements2KHR(
        VkDevice                                    device,
        const VkImageSparseMemoryRequirementsInfo2KHR* pInfo,
        uint32_t*                                   pSparseMemoryRequirementCount,
        VkSparseImageMemoryRequirements2KHR*        pSparseMemoryRequirements)
{
        *pSparseMemoryRequirementCount = 0;
}

void bcmv_GetDeviceMemoryCommitment(
        VkDevice                                    device,
        VkDeviceMemory                              memory,
        VkDeviceSize*                               pCommittedMemoryInBytes)
{
        *pCommittedMemoryInBytes = 0;
}

VkResult bcmv_BindBufferMemory(
        VkDevice                                    device,
        VkBuffer                                    _buffer,
        VkDeviceMemory                              _memory,
        VkDeviceSize                                memoryOffset)
{
        BCMV_FROM_HANDLE(bcmv_device_memory, mem, _memory);
        BCMV_FROM_HANDLE(bcmv_buffer, buffer, _buffer);

        if (mem) {
                assert((buffer->usage & mem->type->valid_buffer_usage) == buffer->usage);
                buffer->bo = mem->bo;
                buffer->offset = memoryOffset;
        } else {
                buffer->bo = NULL;
                buffer->offset = 0;
        }

        return VK_SUCCESS;
}

VkResult bcmv_QueueBindSparse(
        VkQueue                                     _queue,
        uint32_t                                    bindInfoCount,
        const VkBindSparseInfo*                     pBindInfo,
        VkFence                                     fence)
{
        BCMV_FROM_HANDLE(bcmv_queue, queue, _queue);
        if (unlikely(queue->device->lost))
                return VK_ERROR_DEVICE_LOST;

        return vk_error(VK_ERROR_FEATURE_NOT_PRESENT);
}

// Event functions

VkResult bcmv_CreateEvent(
        VkDevice                                    _device,
        const VkEventCreateInfo*                    pCreateInfo,
        const VkAllocationCallbacks*                pAllocator,
        VkEvent*                                    pEvent)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        struct bcmv_state state;
        struct bcmv_event *event;

        assert(pCreateInfo->sType == VK_STRUCTURE_TYPE_EVENT_CREATE_INFO);

        state = bcmv_state_pool_alloc(&device->dynamic_state_pool,
                                      sizeof(*event), 8);
        event = state.map;
        event->state = state;
        event->semaphore = VK_EVENT_RESET;

        /* Make sure the writes we're flushing have landed. */
        /* XXX
        __builtin_ia32_mfence();
        __builtin_ia32_clflush(event);
        */

        *pEvent = bcmv_event_to_handle(event);

        return VK_SUCCESS;
}

void bcmv_DestroyEvent(
        VkDevice                                    _device,
        VkEvent                                     _event,
        const VkAllocationCallbacks*                pAllocator)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        BCMV_FROM_HANDLE(bcmv_event, event, _event);

        if (!event)
                return;

        bcmv_state_pool_free(&device->dynamic_state_pool, event->state);
}

VkResult bcmv_GetEventStatus(
        VkDevice                                    _device,
        VkEvent                                     _event)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        BCMV_FROM_HANDLE(bcmv_event, event, _event);

        if (unlikely(device->lost))
                return VK_ERROR_DEVICE_LOST;

        /* Invalidate read cache before reading event written by GPU. */
        /* XXX
        __builtin_ia32_clflush(event);
        __builtin_ia32_mfence();
        */

        return event->semaphore;
}

VkResult bcmv_SetEvent(
        VkDevice                                    _device,
        VkEvent                                     _event)
{
        BCMV_FROM_HANDLE(bcmv_event, event, _event);

        event->semaphore = VK_EVENT_SET;

        /* Make sure the writes we're flushing have landed. */
        /* XXX
        __builtin_ia32_mfence();
        __builtin_ia32_clflush(event);
        */

        return VK_SUCCESS;
}

VkResult bcmv_ResetEvent(
        VkDevice                                    _device,
        VkEvent                                     _event)
{
        BCMV_FROM_HANDLE(bcmv_event, event, _event);

        event->semaphore = VK_EVENT_RESET;

        /* Make sure the writes we're flushing have landed. */
        /* XXX
        __builtin_ia32_mfence();
        __builtin_ia32_clflush(event);
        */

        return VK_SUCCESS;
}

// Buffer functions

VkResult bcmv_CreateBuffer(
        VkDevice                                    _device,
        const VkBufferCreateInfo*                   pCreateInfo,
        const VkAllocationCallbacks*                pAllocator,
        VkBuffer*                                   pBuffer)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        struct bcmv_buffer *buffer;

        assert(pCreateInfo->sType == VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO);

        buffer = vk_alloc2(&device->alloc, pAllocator, sizeof(*buffer), 8,
                           VK_SYSTEM_ALLOCATION_SCOPE_OBJECT);
        if (buffer == NULL)
                return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);

        buffer->size = pCreateInfo->size;
        buffer->usage = pCreateInfo->usage;
        buffer->bo = NULL;
        buffer->offset = 0;

        *pBuffer = bcmv_buffer_to_handle(buffer);

        return VK_SUCCESS;
}

void bcmv_DestroyBuffer(
        VkDevice                                    _device,
        VkBuffer                                    _buffer,
        const VkAllocationCallbacks*                pAllocator)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        BCMV_FROM_HANDLE(bcmv_buffer, buffer, _buffer);

        if (!buffer)
                return;

        vk_free2(&device->alloc, pAllocator, buffer);
}

#if 0 /* XXX */
void
bcmv_fill_buffer_surface_state(struct bcmv_device *device, struct bcmv_state state,
                               enum isl_format format,
                               uint32_t offset, uint32_t range, uint32_t stride)
{
        isl_buffer_fill_state(&device->isl_dev, state.map,
                              .address = offset,
                              .mocs = device->default_mocs,
                              .size = range,
                              .format = format,
                              .stride = stride);

        bcmv_state_flush(device, state);
}
#endif

void bcmv_DestroySampler(
        VkDevice                                    _device,
        VkSampler                                   _sampler,
        const VkAllocationCallbacks*                pAllocator)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        BCMV_FROM_HANDLE(bcmv_sampler, sampler, _sampler);

        if (!sampler)
                return;

        vk_free2(&device->alloc, pAllocator, sampler);
}

VkResult bcmv_CreateFramebuffer(
        VkDevice                                    _device,
        const VkFramebufferCreateInfo*              pCreateInfo,
        const VkAllocationCallbacks*                pAllocator,
        VkFramebuffer*                              pFramebuffer)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        struct bcmv_framebuffer *framebuffer;

        assert(pCreateInfo->sType == VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO);

        size_t size = sizeof(*framebuffer) +
                sizeof(struct bcmv_image_view *) * pCreateInfo->attachmentCount;
        framebuffer = vk_alloc2(&device->alloc, pAllocator, size, 8,
                                VK_SYSTEM_ALLOCATION_SCOPE_OBJECT);
        if (framebuffer == NULL)
                return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);

        framebuffer->attachment_count = pCreateInfo->attachmentCount;
        for (uint32_t i = 0; i < pCreateInfo->attachmentCount; i++) {
                VkImageView _iview = pCreateInfo->pAttachments[i];
                framebuffer->attachments[i] = bcmv_image_view_from_handle(_iview);
        }

        framebuffer->width = pCreateInfo->width;
        framebuffer->height = pCreateInfo->height;
        framebuffer->layers = pCreateInfo->layers;

        *pFramebuffer = bcmv_framebuffer_to_handle(framebuffer);

        return VK_SUCCESS;
}

void bcmv_DestroyFramebuffer(
        VkDevice                                    _device,
        VkFramebuffer                               _fb,
        const VkAllocationCallbacks*                pAllocator)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        BCMV_FROM_HANDLE(bcmv_framebuffer, fb, _fb);

        if (!fb)
                return;

        vk_free2(&device->alloc, pAllocator, fb);
}

/* vk_icd.h does not declare this function, so we declare it here to
 * suppress Wmissing-prototypes.
 */
PUBLIC VKAPI_ATTR VkResult VKAPI_CALL
vk_icdNegotiateLoaderICDInterfaceVersion(uint32_t* pSupportedVersion);

PUBLIC VKAPI_ATTR VkResult VKAPI_CALL
vk_icdNegotiateLoaderICDInterfaceVersion(uint32_t* pSupportedVersion)
{
        /* For the full details on loader interface versioning, see
         * <https://github.com/KhronosGroup/Vulkan-LoaderAndValidationLayers/blob/master/loader/LoaderAndLayerInterface.md>.
         * What follows is a condensed summary, to help you navigate the large and
         * confusing official doc.
         *
         *   - Loader interface v0 is incompatible with later versions. We don't
         *     support it.
         *
         *   - In loader interface v1:
         *       - The first ICD entrypoint called by the loader is
         *         vk_icdGetInstanceProcAddr(). The ICD must statically expose this
         *         entrypoint.
         *       - The ICD must statically expose no other Vulkan symbol unless it is
         *         linked with -Bsymbolic.
         *       - Each dispatchable Vulkan handle created by the ICD must be
         *         a pointer to a struct whose first member is VK_LOADER_DATA. The
         *         ICD must initialize VK_LOADER_DATA.loadMagic to ICD_LOADER_MAGIC.
         *       - The loader implements vkCreate{PLATFORM}SurfaceKHR() and
         *         vkDestroySurfaceKHR(). The ICD must be capable of working with
         *         such loader-managed surfaces.
         *
         *    - Loader interface v2 differs from v1 in:
         *       - The first ICD entrypoint called by the loader is
         *         vk_icdNegotiateLoaderICDInterfaceVersion(). The ICD must
         *         statically expose this entrypoint.
         *
         *    - Loader interface v3 differs from v2 in:
         *        - The ICD must implement vkCreate{PLATFORM}SurfaceKHR(),
         *          vkDestroySurfaceKHR(), and other API which uses VKSurfaceKHR,
         *          because the loader no longer does so.
         */
        *pSupportedVersion = MIN2(*pSupportedVersion, 3u);
        return VK_SUCCESS;
}
