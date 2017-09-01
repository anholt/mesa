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

#ifndef BCMV_NIR_H
#define BCMV_NIR_H

#include "nir/nir.h"
#include "bcmv_private.h"

#ifdef __cplusplus
extern "C" {
#endif

void bcmv_nir_lower_input_attachments(nir_shader *shader);

void bcmv_nir_lower_push_constants(nir_shader *shader);

bool bcmv_nir_lower_multiview(nir_shader *shader, uint32_t view_mask);

void bcmv_nir_apply_pipeline_layout(struct bcmv_pipeline *pipeline,
                                   nir_shader *shader,
                                   struct v3d_prog_data *prog_data,
                                   struct bcmv_pipeline_bind_map *map);

#ifdef __cplusplus
}
#endif

#endif /* BCMV_NIR_H */
