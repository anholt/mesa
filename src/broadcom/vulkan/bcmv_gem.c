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

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>

#include "bcmv_private.h"

static int
bcmv_ioctl(int fd, unsigned long request, void *arg)
{
        int ret;

        do {
                ret = ioctl(fd, request, arg);
        } while (ret == -1 && (errno == EINTR || errno == EAGAIN));

        return ret;
}

/**
 * Wrapper around DRM_IOCTL_VC5_CREATE_BO.
 *
 * Return gem handle, or 0 on failure. Gem handles are never 0.
 */
uint32_t
bcmv_gem_create(struct bcmv_device *device, uint64_t size)
{
        struct drm_vc5_create_bo gem_create = {
                .size = size,
        };

        int ret = bcmv_ioctl(device->fd, DRM_IOCTL_VC5_CREATE_BO, &gem_create);
        if (ret != 0) {
                /* FIXME: What do we do if this fails? */
                return 0;
        }

        return gem_create.handle;
}

void
bcmv_gem_close(struct bcmv_device *device, uint32_t gem_handle)
{
        struct drm_gem_close close = {
                .handle = gem_handle,
        };

        bcmv_ioctl(device->fd, DRM_IOCTL_GEM_CLOSE, &close);
}

/**
 * Wrapper around DRM_IOCTL_VC5_MMAP_BO. Returns MAP_FAILED on error.
 */
void*
bcmv_gem_mmap(struct bcmv_device *device, uint32_t gem_handle, uint64_t size)
{
        struct drm_vc5_mmap_bo gem_mmap = {
                .handle = gem_handle,
        };

        int ret = bcmv_ioctl(device->fd, DRM_IOCTL_VC5_MMAP_BO, &gem_mmap);
        if (ret != 0)
                return MAP_FAILED;

        return mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED,
                    device->fd, gem_mmap.offset);
}

#if 0 /* XXX */
uint32_t
bcmv_gem_userptr(struct bcmv_device *device, void *mem, size_t size)
{
        struct drm_i915_gem_userptr userptr = {
                .user_ptr = (__u64)((unsigned long) mem),
                .user_size = size,
                .flags = 0,
        };

        int ret = bcmv_ioctl(device->fd, DRM_IOCTL_I915_GEM_USERPTR, &userptr);
        if (ret == -1)
                return 0;

        return userptr.handle;
}
#endif

#if 0 /* XXX */
/**
 * Returns 0, 1, or negative to indicate error
 */
int
bcmv_gem_busy(struct bcmv_device *device, uint32_t gem_handle)
{
        struct drm_i915_gem_busy busy = {
                .handle = gem_handle,
        };

        int ret = bcmv_ioctl(device->fd, DRM_IOCTL_I915_GEM_BUSY, &busy);
        if (ret < 0)
                return ret;

        return busy.busy != 0;
}
#endif

/**
 * On error, \a timeout_ns holds the remaining time.
 */
int
bcmv_gem_wait(struct bcmv_device *device, uint32_t gem_handle, int64_t *timeout_ns)
{
        struct drm_vc5_wait_bo wait = {
                .handle = gem_handle,
                .timeout_ns = *timeout_ns,
        };

        int ret = bcmv_ioctl(device->fd, DRM_IOCTL_VC5_WAIT_BO, &wait);
        *timeout_ns = wait.timeout_ns;

        return ret;
}

int
bcmv_gem_submit_cl(struct bcmv_device *device,
                   struct drm_vc5_submit_cl *execbuf)
{
        return bcmv_ioctl(device->fd, DRM_IOCTL_VC5_SUBMIT_CL, execbuf);
}

int
bcmv_gem_get_param(int fd, enum drm_vc5_param param)
{
        struct drm_vc5_get_param gp = {
                .param = param,
        };

        int ret = bcmv_ioctl(fd, DRM_IOCTL_VC5_GET_PARAM, &gp);
        if (ret == 0)
                return gp.value;

        return 0;
}

#if 0 /* XXX */
int
bcmv_gem_gpu_get_reset_stats(struct bcmv_device *device,
                             uint32_t *active, uint32_t *pending)
{
        struct drm_i915_reset_stats stats = {
                .ctx_id = device->context_id,
        };

        int ret = bcmv_ioctl(device->fd, DRM_IOCTL_I915_GET_RESET_STATS, &stats);
        if (ret == 0) {
                *active = stats.batch_active;
                *pending = stats.batch_pending;
        }

        return ret;
}
#endif

int
bcmv_gem_handle_to_fd(struct bcmv_device *device, uint32_t gem_handle)
{
        struct drm_prime_handle args = {
                .handle = gem_handle,
                .flags = DRM_CLOEXEC,
        };

        int ret = bcmv_ioctl(device->fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &args);
        if (ret == -1)
                return -1;

        return args.fd;
}

uint32_t
bcmv_gem_fd_to_handle(struct bcmv_device *device, int fd)
{
        struct drm_prime_handle args = {
                .fd = fd,
        };

        int ret = bcmv_ioctl(device->fd, DRM_IOCTL_PRIME_FD_TO_HANDLE, &args);
        if (ret == -1)
                return 0;

        return args.handle;
}

#ifndef SYNC_IOC_MAGIC
/* duplicated from linux/sync_file.h to avoid build-time dependency
 * on new (v4.7) kernel headers.  Once distro's are mostly using
 * something newer than v4.7 drop this and #include <linux/sync_file.h>
 * instead.
 */
struct sync_merge_data {
        char  name[32];
        __s32 fd2;
        __s32 fence;
        __u32 flags;
        __u32 pad;
};

#define SYNC_IOC_MAGIC '>'
#define SYNC_IOC_MERGE _IOWR(SYNC_IOC_MAGIC, 3, struct sync_merge_data)
#endif

int
bcmv_gem_sync_file_merge(struct bcmv_device *device, int fd1, int fd2)
{
        const char name[] = "bcmv merge fence";
        struct sync_merge_data args = {
                .fd2 = fd2,
                .fence = -1,
        };
        memcpy(args.name, name, sizeof(name));

        int ret = bcmv_ioctl(fd1, SYNC_IOC_MERGE, &args);
        if (ret == -1)
                return -1;

        return args.fence;
}

uint32_t
bcmv_gem_syncobj_create(struct bcmv_device *device, uint32_t flags)
{
        struct drm_syncobj_create args = {
                .flags = flags,
        };

        int ret = bcmv_ioctl(device->fd, DRM_IOCTL_SYNCOBJ_CREATE, &args);
        if (ret)
                return 0;

        return args.handle;
}

void
bcmv_gem_syncobj_destroy(struct bcmv_device *device, uint32_t handle)
{
        struct drm_syncobj_destroy args = {
                .handle = handle,
        };

        bcmv_ioctl(device->fd, DRM_IOCTL_SYNCOBJ_DESTROY, &args);
}

int
bcmv_gem_syncobj_handle_to_fd(struct bcmv_device *device, uint32_t handle)
{
        struct drm_syncobj_handle args = {
                .handle = handle,
        };

        int ret = bcmv_ioctl(device->fd, DRM_IOCTL_SYNCOBJ_HANDLE_TO_FD, &args);
        if (ret)
                return -1;

        return args.fd;
}

uint32_t
bcmv_gem_syncobj_fd_to_handle(struct bcmv_device *device, int fd)
{
        struct drm_syncobj_handle args = {
                .fd = fd,
        };

        int ret = bcmv_ioctl(device->fd, DRM_IOCTL_SYNCOBJ_FD_TO_HANDLE, &args);
        if (ret)
                return 0;

        return args.handle;
}

int
bcmv_gem_syncobj_export_sync_file(struct bcmv_device *device, uint32_t handle)
{
        struct drm_syncobj_handle args = {
                .handle = handle,
                .flags = DRM_SYNCOBJ_HANDLE_TO_FD_FLAGS_EXPORT_SYNC_FILE,
        };

        int ret = bcmv_ioctl(device->fd, DRM_IOCTL_SYNCOBJ_HANDLE_TO_FD, &args);
        if (ret)
                return -1;

        return args.fd;
}

int
bcmv_gem_syncobj_import_sync_file(struct bcmv_device *device,
                                  uint32_t handle, int fd)
{
        struct drm_syncobj_handle args = {
                .handle = handle,
                .fd = fd,
                .flags = DRM_SYNCOBJ_FD_TO_HANDLE_FLAGS_IMPORT_SYNC_FILE,
        };

        return bcmv_ioctl(device->fd, DRM_IOCTL_SYNCOBJ_FD_TO_HANDLE, &args);
}

void
bcmv_gem_syncobj_reset(struct bcmv_device *device, uint32_t handle)
{
        struct drm_syncobj_array args = {
                .handles = (uint64_t)(uintptr_t)&handle,
                .count_handles = 1,
        };

        bcmv_ioctl(device->fd, DRM_IOCTL_SYNCOBJ_RESET, &args);
}

bool
bcmv_gem_supports_syncobj_wait(int fd)
{
        int ret;

        struct drm_syncobj_create create = {
                .flags = 0,
        };
        ret = bcmv_ioctl(fd, DRM_IOCTL_SYNCOBJ_CREATE, &create);
        if (ret)
                return false;

        uint32_t syncobj = create.handle;

        struct drm_syncobj_wait wait = {
                .handles = (uint64_t)(uintptr_t)&create,
                .count_handles = 1,
                .timeout_nsec = 0,
                .flags = DRM_SYNCOBJ_WAIT_FLAGS_WAIT_FOR_SUBMIT,
        };
        ret = bcmv_ioctl(fd, DRM_IOCTL_SYNCOBJ_WAIT, &wait);

        struct drm_syncobj_destroy destroy = {
                .handle = syncobj,
        };
        bcmv_ioctl(fd, DRM_IOCTL_SYNCOBJ_DESTROY, &destroy);

        /* If it timed out, then we have the ioctl and it supports the
         * DRM_SYNCOBJ_WAIT_FLAGS_WAIT_FOR_SUBMIT flag.
         */
        return ret == -1 && errno == ETIME;
}

int
bcmv_gem_syncobj_wait(struct bcmv_device *device,
                      uint32_t *handles, uint32_t num_handles,
                      int64_t abs_timeout_ns, bool wait_all)
{
        struct drm_syncobj_wait args = {
                .handles = (uint64_t)(uintptr_t)handles,
                .count_handles = num_handles,
                .timeout_nsec = abs_timeout_ns,
                .flags = DRM_SYNCOBJ_WAIT_FLAGS_WAIT_FOR_SUBMIT,
        };

        if (wait_all)
                args.flags |= DRM_SYNCOBJ_WAIT_FLAGS_WAIT_ALL;

        return bcmv_ioctl(device->fd, DRM_IOCTL_SYNCOBJ_WAIT, &args);
}
