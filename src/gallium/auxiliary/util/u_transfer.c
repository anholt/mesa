#include "pipe/p_context.h"
#include "util/u_surface.h"
#include "util/u_inlines.h"
#include "util/u_transfer.h"
#include "util/u_memory.h"

void u_default_buffer_subdata(struct pipe_context *pipe,
                              struct pipe_resource *resource,
                              unsigned usage, unsigned offset,
                              unsigned size, const void *data)
{
   struct pipe_transfer *transfer = NULL;
   struct pipe_box box;
   uint8_t *map = NULL;

   assert(!(usage & PIPE_TRANSFER_READ));

   /* the write flag is implicit by the nature of buffer_subdata */
   usage |= PIPE_TRANSFER_WRITE;

   /* buffer_subdata implicitly discards the rewritten buffer range */
   if (offset == 0 && size == resource->width0) {
      usage |= PIPE_TRANSFER_DISCARD_WHOLE_RESOURCE;
   } else {
      usage |= PIPE_TRANSFER_DISCARD_RANGE;
   }

   u_box_1d(offset, size, &box);

   map = pipe->transfer_map(pipe, resource, 0, usage, &box, &transfer);
   if (!map)
      return;

   memcpy(map, data, size);
   pipe_transfer_unmap(pipe, transfer);
}

void u_default_texture_subdata(struct pipe_context *pipe,
                               struct pipe_resource *resource,
                               unsigned level,
                               unsigned usage,
                               const struct pipe_box *box,
                               const void *data,
                               unsigned stride,
                               unsigned layer_stride)
{
   struct pipe_transfer *transfer = NULL;
   const uint8_t *src_data = data;
   uint8_t *map = NULL;

   assert(!(usage & PIPE_TRANSFER_READ));

   /* the write flag is implicit by the nature of texture_subdata */
   usage |= PIPE_TRANSFER_WRITE;

   /* texture_subdata implicitly discards the rewritten buffer range */
   usage |= PIPE_TRANSFER_DISCARD_RANGE;

   map = pipe->transfer_map(pipe,
                            resource,
                            level,
                            usage,
                            box, &transfer);
   if (!map)
      return;

   util_copy_box(map,
                 resource->format,
                 transfer->stride, /* bytes */
                 transfer->layer_stride, /* bytes */
                 0, 0, 0,
                 box->width,
                 box->height,
                 box->depth,
                 src_data,
                 stride,       /* bytes */
                 layer_stride, /* bytes */
                 0, 0, 0);

   pipe_transfer_unmap(pipe, transfer);
}


boolean u_default_resource_get_handle(UNUSED struct pipe_screen *screen,
                                      UNUSED struct pipe_resource *resource,
                                      UNUSED struct winsys_handle *handle)
{
   return FALSE;
}



void u_default_transfer_flush_region(UNUSED struct pipe_context *pipe,
                                     UNUSED struct pipe_transfer *transfer,
                                     UNUSED const struct pipe_box *box)
{
   /* This is a no-op implementation, nothing to do.
    */
}

void u_default_transfer_unmap(UNUSED struct pipe_context *pipe,
                              UNUSED struct pipe_transfer *transfer)
{
}


static inline struct u_resource *
u_resource( struct pipe_resource *res )
{
   return (struct u_resource *)res;
}

boolean u_resource_get_handle_vtbl(struct pipe_screen *screen,
                                   UNUSED struct pipe_context *ctx,
                                   struct pipe_resource *resource,
                                   struct winsys_handle *handle,
                                   UNUSED unsigned usage)
{
   struct u_resource *ur = u_resource(resource);
   return ur->vtbl->resource_get_handle(screen, resource, handle);
}

void u_resource_destroy_vtbl(struct pipe_screen *screen,
                             struct pipe_resource *resource)
{
   struct u_resource *ur = u_resource(resource);
   ur->vtbl->resource_destroy(screen, resource);
}

void *u_transfer_map_vtbl(struct pipe_context *context,
                          struct pipe_resource *resource,
                          unsigned level,
                          unsigned usage,
                          const struct pipe_box *box,
                          struct pipe_transfer **transfer)
{
   struct u_resource *ur = u_resource(resource);
   return ur->vtbl->transfer_map(context, resource, level, usage, box,
                                 transfer);
}

void u_transfer_flush_region_vtbl( struct pipe_context *pipe,
                                   struct pipe_transfer *transfer,
                                   const struct pipe_box *box)
{
   struct u_resource *ur = u_resource(transfer->resource);
   ur->vtbl->transfer_flush_region(pipe, transfer, box);
}

void u_transfer_unmap_vtbl( struct pipe_context *pipe,
                            struct pipe_transfer *transfer )
{
   struct u_resource *ur = u_resource(transfer->resource);
   ur->vtbl->transfer_unmap(pipe, transfer);
}

struct u_transfer_msaa_helper {
   struct pipe_transfer base;
   struct pipe_resource *ss;
   struct pipe_transfer *wrapped;
};

/**
 * Helper to implement the implicit MSAA resolve necessary in the
 * pipe_transfer API.
 *
 * The driver should call this when the resource is multisampled.  We create a
 * temporary single-sampled texture, blit to do the resolve if needed, and
 * then call back to the driver to map the single-sampled texture.
 *
 * Note that the driver's unmap will be called with our ptrans: They need to
 * detect it and call u_transfer_unmap_msaa_helper() and return immediately.
 */
void *
u_transfer_map_msaa_helper(struct pipe_context *pctx,
                           struct pipe_resource *prsc,
                           unsigned level, unsigned usage,
                           const struct pipe_box *box,
                           struct pipe_transfer **pptrans)
{
   struct pipe_screen *pscreen = pctx->screen;
   assert(prsc->nr_samples > 1);

   struct u_transfer_msaa_helper *trans = calloc(1, sizeof(*trans));
   if (!trans)
      return NULL;
   struct pipe_transfer *ptrans = &trans->base;

   pipe_resource_reference(&ptrans->resource, prsc);
   ptrans->level = level;
   ptrans->usage = usage;
   ptrans->box = *box;

   struct pipe_resource temp_setup = {
      .target = prsc->target,
      .format = prsc->format,
      .width0 = box->width,
      .height0 = box->height,
      .depth0 = 1,
      .array_size = 1,
   };
   trans->ss = pscreen->resource_create(pscreen, &temp_setup);
   if (!trans->ss) {
      free(trans);
      return NULL;
   }

   if (usage & PIPE_TRANSFER_READ) {
      struct pipe_blit_info blit;
      memset(&blit, 0, sizeof(blit));

      blit.src.resource = ptrans->resource;
      blit.src.format = ptrans->resource->format;
      blit.src.level = ptrans->level;
      blit.src.box = *box;

      blit.dst.resource = trans->ss;
      blit.dst.format = trans->ss->format;
      blit.dst.box.width = box->width;
      blit.dst.box.height = box->height;
      blit.dst.box.depth = 1;

      blit.mask = util_format_get_mask(prsc->format);
      blit.filter = PIPE_TEX_FILTER_NEAREST;

      pctx->blit(pctx, &blit);
   }

   void *ss_map = pctx->transfer_map(pctx, trans->ss, 0, usage, box,
                                     &trans->wrapped);
   if (!ss_map) {
      free(trans);
      return NULL;
   }

   *pptrans = ptrans;
   return ss_map;
}

void u_transfer_unmap_msaa_helper(struct pipe_context *pctx,
                                  struct pipe_transfer *ptrans)
{
   struct u_transfer_msaa_helper *trans =
      (struct u_transfer_msaa_helper *)ptrans;

   /* Unmap the single-sample resource, finishing whatever driver side storing
    * is necessary.
    */
   pipe_transfer_unmap(pctx, trans->wrapped);

   if (ptrans->usage & PIPE_TRANSFER_WRITE) {
      struct pipe_blit_info blit;
      memset(&blit, 0, sizeof(blit));

      blit.src.resource = trans->ss;
      blit.src.format = trans->ss->format;
      blit.src.box.width = ptrans->box.width;
      blit.src.box.height = ptrans->box.height;
      blit.src.box.depth = 1;

      blit.dst.resource = ptrans->resource;
      blit.dst.format = ptrans->resource->format;
      blit.dst.level = ptrans->level;
      blit.dst.box = ptrans->box;

      blit.mask = util_format_get_mask(ptrans->resource->format);
      blit.filter = PIPE_TEX_FILTER_NEAREST;

      pctx->blit(pctx, &blit);
   }

   pipe_resource_reference(&trans->ss, NULL);
   free(trans);
}
