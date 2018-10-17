/**************************************************************************
 *
 * Copyright 2018 Advanced Micro Devices, Inc.
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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 **************************************************************************/

#include <assert.h>
#include <stdio.h>

#include "pipe/p_video_codec.h"

#include "util/u_memory.h"
#include "util/u_video.h"

#include "radeonsi/si_pipe.h"
#include "radeon_video.h"
#include "radeon_vcn_dec.h"

static struct pb_buffer *radeon_jpeg_get_decode_param(struct radeon_decoder *dec,
					struct pipe_video_buffer *target,
					struct pipe_picture_desc *picture)
{
	struct si_texture *luma = (struct si_texture *)
				((struct vl_video_buffer *)target)->resources[0];
	struct si_texture *chroma = (struct si_texture *)
				((struct vl_video_buffer *)target)->resources[1];

	dec->jpg.bsd_size = align(dec->bs_size, 128);
	dec->jpg.dt_luma_top_offset = luma->surface.u.gfx9.surf_offset;
	if (target->buffer_format == PIPE_FORMAT_NV12) {
		dec->jpg.dt_chroma_top_offset = chroma->surface.u.gfx9.surf_offset;
		dec->jpg.dt_pitch = luma->surface.u.gfx9.surf_pitch * luma->surface.blk_w;
	}
	else if (target->buffer_format == PIPE_FORMAT_YUYV)
		dec->jpg.dt_pitch = luma->surface.u.gfx9.surf_pitch;
	dec->jpg.dt_uv_pitch = dec->jpg.dt_pitch / 2;

	return luma->buffer.buf;
}

/* send a bitstream buffer command */
static void send_cmd_bitstream(struct radeon_decoder *dec,
		     struct pb_buffer* buf, uint32_t off,
		     enum radeon_bo_usage usage, enum radeon_bo_domain domain)
{
	/* TODO */
}

/* send a target buffer command */
static void send_cmd_target(struct radeon_decoder *dec,
		     struct pb_buffer* buf, uint32_t off,
		     enum radeon_bo_usage usage, enum radeon_bo_domain domain)
{
	/* TODO */
}

/**
 * send cmd for vcn jpeg
 */
void send_cmd_jpeg(struct radeon_decoder *dec,
			   struct pipe_video_buffer *target,
			   struct pipe_picture_desc *picture)
{
	struct pb_buffer *dt;
	struct rvid_buffer *bs_buf;

	bs_buf = &dec->bs_buffers[dec->cur_buffer];

	memset(dec->bs_ptr, 0, align(dec->bs_size, 128) - dec->bs_size);
	dec->ws->buffer_unmap(bs_buf->res->buf);

	dt = radeon_jpeg_get_decode_param(dec, target, picture);

	send_cmd_bitstream(dec, bs_buf->res->buf,
		 0, RADEON_USAGE_READ, RADEON_DOMAIN_GTT);
	send_cmd_target(dec, dt, 0,
		 RADEON_USAGE_WRITE, RADEON_DOMAIN_VRAM);
}
