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

#ifndef V3D_MACROS_H
#define V3D_MACROS_H

/* Macros for handling per-v3d compilation.
 *
 * The prefixing macros V3DX() and v3dx() automatically prefix whatever you
 * give them by V3DX_ or v3dx_  where X is the v3d number.
 *
 * You can do pseudo-runtime checks in your function such as
 *
 * if (V3D_VERSION >= 33) {
 *    // Do something
 * }
 *
 * The contents of the if statement must be valid regardless of v3d, but
 * the if will get compiled away.
 *
 * For places where you really do have a compile-time conflict, you can
 * use preprocessor logic:
 *
 * #if (V3D_VERSION >= 33)
 *    // Do something
 * #endif
 *
 * However, it is strongly recommended that the former be used whenever
 * possible.
 */

/* Base macro defined on the command line.  If we don't have this, we can't
 * do anything.
 */
#ifndef V3D_VERSION
#  error "The V3D_VERSION macro must be defined"
#endif

/* Prefixing macros */
#if (V3D_VERSION == 21)
#  define V3DX(X) V3D21_##X
#  define v3dx(x) v3d21_##x
#elif (V3D_VERSION == 33)
#  define V3DX(X) V3D33_##X
#  define v3dx(x) v3d33_##x
#else
#  error "Need to add prefixing macros for this v3d"
#endif

#endif /* V3D_MACROS_H */
