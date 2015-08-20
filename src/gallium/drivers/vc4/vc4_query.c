/*
 * Copyright © 2014 Broadcom
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
 * VC4 query support.
 *
 * Since we expose support for GL 2.0, we have to expose occlusion queries,
 * but the spec allows you to expose 0 query counter bits, so we just return 0
 * as the result of all our queries.
 *
 * We also support various software queries for debugging driver performance
 * behavior.
 */
#include "vc4_context.h"

enum {
        VC4_QUERY_DRAW_CALLS = PIPE_QUERY_DRIVER_SPECIFIC,
        VC4_QUERY_BATCHES,
        VC4_QUERY_LOAD_COLOR,
        VC4_QUERY_LOAD_DEPTH,
        VC4_QUERY_STORE_COLOR,
        VC4_QUERY_STORE_DEPTH,
};

struct vc4_query
{
        unsigned query_type;

        uint64_t start_value;
        uint64_t end_value;
};

static uint64_t
vc4_query_get_value(struct pipe_context *ctx, unsigned query_type)
{
        struct vc4_context *vc4 = vc4_context(ctx);

        switch (query_type) {
        case PIPE_QUERY_OCCLUSION_COUNTER:
                return 0;
        case VC4_QUERY_DRAW_CALLS:
                return vc4->stats.draws;
        case VC4_QUERY_BATCHES:
                return vc4->stats.jobs;
        case VC4_QUERY_LOAD_COLOR:
                return vc4->stats.load_color;
        case VC4_QUERY_STORE_COLOR:
                return vc4->stats.store_color;
        case VC4_QUERY_LOAD_DEPTH:
                return vc4->stats.load_depth;
        case VC4_QUERY_STORE_DEPTH:
                return vc4->stats.store_depth;
        default:
                unreachable("bad query type");
        }
}

static struct pipe_query *
vc4_create_query(struct pipe_context *ctx, unsigned query_type, unsigned index)
{
        struct vc4_query *query = calloc(1, sizeof(*query));

        query->query_type = query_type;

        /* Note that struct pipe_query isn't actually defined anywhere. */
        return (struct pipe_query *)query;
}

static void
vc4_destroy_query(struct pipe_context *ctx, struct pipe_query *query)
{
        free(query);
}

static boolean
vc4_begin_query(struct pipe_context *ctx, struct pipe_query *pquery)
{
        struct vc4_query *query = (struct vc4_query *)pquery;

        query->start_value = vc4_query_get_value(ctx, query->query_type);

        return true;
}

static bool
vc4_end_query(struct pipe_context *ctx, struct pipe_query *pquery)
{
        struct vc4_query *query = (struct vc4_query *)pquery;

        query->end_value = vc4_query_get_value(ctx, query->query_type);

        return true;
}

static boolean
vc4_get_query_result(struct pipe_context *ctx, struct pipe_query *pquery,
                     boolean wait, union pipe_query_result *vresult)
{
        struct vc4_query *query = (struct vc4_query *)pquery;
        uint64_t *result = &vresult->u64;

        *result = query->end_value - query->start_value;

        return true;
}

static void
vc4_set_active_query_state(struct pipe_context *pipe, boolean enable)
{
}

void
vc4_query_context_init(struct pipe_context *pctx)
{
        pctx->create_query = vc4_create_query;
        pctx->destroy_query = vc4_destroy_query;
        pctx->begin_query = vc4_begin_query;
        pctx->end_query = vc4_end_query;
        pctx->get_query_result = vc4_get_query_result;
	pctx->set_active_query_state = vc4_set_active_query_state;
}

static int
vc4_get_driver_query_info(struct pipe_screen *pscreen,
                          unsigned index, struct pipe_driver_query_info *info)
{
	static const struct pipe_driver_query_info list[] = {
                {
                        "draw-calls",
                        VC4_QUERY_DRAW_CALLS,
                        {0},
                        PIPE_DRIVER_QUERY_TYPE_UINT64,
                },
                {
                        "batches",
                        VC4_QUERY_BATCHES,
                        {0},
                        PIPE_DRIVER_QUERY_TYPE_UINT64,
                },
                {
                        "load_color",
                        VC4_QUERY_LOAD_COLOR,
                        {0},
                        PIPE_DRIVER_QUERY_TYPE_BYTES,
                },
                {
                        "store_color",
                        VC4_QUERY_STORE_COLOR,
                        {0},
                        PIPE_DRIVER_QUERY_TYPE_BYTES,
                },
                {
                        "load_depth",
                        VC4_QUERY_LOAD_DEPTH,
                        {0},
                        PIPE_DRIVER_QUERY_TYPE_BYTES,
                },
                {
                        "store_depth",
                        VC4_QUERY_STORE_DEPTH,
                        {0},
                        PIPE_DRIVER_QUERY_TYPE_BYTES,
                },
	};

	if (!info)
		return ARRAY_SIZE(list);

	if (index >= ARRAY_SIZE(list))
		return 0;

	*info = list[index];
	return 1;
}

void
vc4_query_screen_init(struct pipe_screen *pscreen)
{
        pscreen->get_driver_query_info = vc4_get_driver_query_info;
}
