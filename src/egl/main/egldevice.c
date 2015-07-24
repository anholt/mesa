/**************************************************************************
 *
 * Copyright 2015, 2018 Collabora
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 **************************************************************************/

#include "util/macros.h"

#include "eglcurrent.h"
#include "egldevice.h"
#include "eglglobals.h"
#include "egltypedefs.h"


struct _egl_device {
   _EGLDevice *Next;

   const char *extensions;
};

void
_eglFiniDevice(void)
{
   _EGLDevice *dev_list, *dev;

   /* atexit function is called with global mutex locked */

   dev_list = _eglGlobal.DeviceList;
   while (dev_list) {
      /* pop list head */
      dev = dev_list;
      dev_list = dev_list->Next;

      free(dev);
   }

   _eglGlobal.DeviceList = NULL;
}

EGLBoolean
_eglCheckDeviceHandle(EGLDeviceEXT device)
{
   _EGLDevice *cur;

   mtx_lock(_eglGlobal.Mutex);
   cur = _eglGlobal.DeviceList;
   while (cur) {
      if (cur == (_EGLDevice *) device)
         break;
      cur = cur->Next;
   }
   mtx_unlock(_eglGlobal.Mutex);
   return (cur != NULL);
}

/* Adds a device in DeviceList, if needed for the given fd.
 *
 * If a software device, the fd is ignored.
 */
_EGLDevice *
_eglAddDevice(int fd, bool software)
{
   _EGLDevice *dev;

   mtx_lock(_eglGlobal.Mutex);

   dev = NULL;

out:
   mtx_unlock(_eglGlobal.Mutex);
   return dev;
}

EGLBoolean
_eglDeviceSupports(_EGLDevice *dev, _EGLDeviceExtension ext)
{
   switch (ext) {
   default:
      assert(0);
      return EGL_FALSE;
   };
}

EGLBoolean
_eglQueryDeviceAttribEXT(_EGLDevice *dev, EGLint attribute,
                         EGLAttrib *value)
{
   switch (attribute) {
   default:
      _eglError(EGL_BAD_ATTRIBUTE, "eglQueryDeviceStringEXT");
      return EGL_FALSE;
   }
}

const char *
_eglQueryDeviceStringEXT(_EGLDevice *dev, EGLint name)
{
   switch (name) {
   case EGL_EXTENSIONS:
      return dev->extensions;
   default:
      _eglError(EGL_BAD_PARAMETER, "eglQueryDeviceStringEXT");
      return NULL;
   };
}

/* Do a fresh lookup for devices.
 *
 * Walks through the DeviceList, discarding no longer available ones
 * and adding new ones as applicable.
 *
 * Must be called with the global lock held.
 */
static int
_eglRefreshDeviceList(void)
{
   MAYBE_UNUSED _EGLDevice *dev;
   int count = 0;

   dev = _eglGlobal.DeviceList;

   return count;
}

EGLBoolean
_eglQueryDevicesEXT(EGLint max_devices,
                    _EGLDevice **devices,
                    EGLint *num_devices)
{
   _EGLDevice *dev, *devs;
   int i = 0, num_devs;

   if ((devices && max_devices <= 0) || !num_devices)
      return _eglError(EGL_BAD_PARAMETER, "eglQueryDevicesEXT");

   mtx_lock(_eglGlobal.Mutex);

   num_devs = _eglRefreshDeviceList();
   devs = _eglGlobal.DeviceList;

   /* bail early if we only care about the count */
   if (!devices) {
      *num_devices = num_devs;
      goto out;
   }

   *num_devices = MIN2(num_devs, max_devices);

   for (i = 0, dev = devs; i < *num_devices; i++) {
      devices[i] = dev;
      dev = dev->Next;
   }

out:
   mtx_unlock(_eglGlobal.Mutex);

   return EGL_TRUE;
}
