/*
 * Copyright © 2016 Intel Corporation
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

/*
 * NOTE: The header can be included multiple times, from the same file.
 */

/*
 * Gen-specific function declarations.  This header must *not* be included
 * directly.  Instead, it is included multiple times by bcmv_private.h.
 *
 * In this header file, the usual genx() macro is available.
 */

#ifndef BCMV_PRIVATE_H
#error This file is included by means other than bcmv_private.h
#endif

VkResult v3dx(init_device_state)(struct bcmv_device *device);

void v3dx(cmd_buffer_emit_state_base_address)(struct bcmv_cmd_buffer *cmd_buffer);

void v3dx(cmd_buffer_apply_pipe_flushes)(struct bcmv_cmd_buffer *cmd_buffer);

void v3dx(cmd_buffer_emit_gen7_depth_flush)(struct bcmv_cmd_buffer *cmd_buffer);

void v3dx(flush_pipeline_select_3d)(struct bcmv_cmd_buffer *cmd_buffer);
void v3dx(flush_pipeline_select_gpgpu)(struct bcmv_cmd_buffer *cmd_buffer);

void v3dx(cmd_buffer_config_l3)(struct bcmv_cmd_buffer *cmd_buffer,
                                const struct gen_l3_config *cfg);

void v3dx(cmd_buffer_flush_state)(struct bcmv_cmd_buffer *cmd_buffer);
void v3dx(cmd_buffer_flush_dynamic_state)(struct bcmv_cmd_buffer *cmd_buffer);

void v3dx(cmd_buffer_flush_compute_state)(struct bcmv_cmd_buffer *cmd_buffer);

void v3dx(cmd_buffer_enable_pma_fix)(struct bcmv_cmd_buffer *cmd_buffer,
                                     bool enable);

void
v3dx(emit_urb_setup)(struct bcmv_device *device, struct bcmv_batch *batch,
                     const struct gen_l3_config *l3_config,
                     VkShaderStageFlags active_stages,
                     const unsigned entry_size[4]);

void v3dx(cmd_buffer_so_memcpy)(struct bcmv_cmd_buffer *cmd_buffer,
                                struct bcmv_bo *dst, uint32_t dst_offset,
                                struct bcmv_bo *src, uint32_t src_offset,
                                uint32_t size);

void v3dx(cmd_buffer_mi_memcpy)(struct bcmv_cmd_buffer *cmd_buffer,
                                struct bcmv_bo *dst, uint32_t dst_offset,
                                struct bcmv_bo *src, uint32_t src_offset,
                                uint32_t size);
