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

#include "broadcom/cle/v3dx_macros.h"
#include "cle/v3dx_pack.h"
#include "vk_format.h"

#if 0 /* XXX */
static uint32_t
vertex_element_comp_control(enum isl_format format, unsigned comp)
{
        uint8_t bits;
        switch (comp) {
        case 0: bits = isl_format_layouts[format].channels.r.bits; break;
        case 1: bits = isl_format_layouts[format].channels.g.bits; break;
        case 2: bits = isl_format_layouts[format].channels.b.bits; break;
        case 3: bits = isl_format_layouts[format].channels.a.bits; break;
        default: unreachable("Invalid component");
        }

        /*
         * Take in account hardware restrictions when dealing with 64-bit floats.
         *
         * From Broadwell spec, command reference structures, page 586:
         *  "When SourceElementFormat is set to one of the *64*_PASSTHRU formats,
         *   64-bit components are stored * in the URB without any conversion. In
         *   this case, vertex elements must be written as 128 or 256 bits, with
         *   VFCOMP_STORE_0 being used to pad the output as required. E.g., if
         *   R64_PASSTHRU is used to copy a 64-bit Red component into the URB,
         *   Component 1 must be specified as VFCOMP_STORE_0 (with Components 2,3
         *   set to VFCOMP_NOSTORE) in order to output a 128-bit vertex element, or
         *   Components 1-3 must be specified as VFCOMP_STORE_0 in order to output
         *   a 256-bit vertex element. Likewise, use of R64G64B64_PASSTHRU requires
         *   Component 3 to be specified as VFCOMP_STORE_0 in order to output a
         *   256-bit vertex element."
         */
        if (bits) {
                return VFCOMP_STORE_SRC;
        } else if (comp >= 2 &&
                   !isl_format_layouts[format].channels.b.bits &&
                   isl_format_layouts[format].channels.r.type == ISL_RAW) {
                /* When emitting 64-bit attributes, we need to write either 128 or 256
                 * bit chunks, using VFCOMP_NOSTORE when not writing the chunk, and
                 * VFCOMP_STORE_0 to pad the written chunk */
                return VFCOMP_NOSTORE;
        } else if (comp < 3 ||
                   isl_format_layouts[format].channels.r.type == ISL_RAW) {
                /* Note we need to pad with value 0, not 1, due hardware restrictions
                 * (see comment above) */
                return VFCOMP_STORE_0;
        } else if (isl_format_layouts[format].channels.r.type == ISL_UINT ||
                   isl_format_layouts[format].channels.r.type == ISL_SINT) {
                assert(comp == 3);
                return VFCOMP_STORE_1_INT;
        } else {
                assert(comp == 3);
                return VFCOMP_STORE_1_FP;
        }
}
#endif

static void
emit_vertex_input(struct bcmv_pipeline *pipeline,
                  const VkPipelineVertexInputStateCreateInfo *info)
{
#if 0 /* XXX */
        const struct brw_vs_prog_data *vs_prog_data = get_vs_prog_data(pipeline);

        /* Pull inputs_read out of the VS prog data */
        const uint64_t inputs_read = vs_prog_data->inputs_read;
        assert((inputs_read & ((1 << VERT_ATTRIB_GENERIC0) - 1)) == 0);
        const uint32_t elements = inputs_read >> VERT_ATTRIB_GENERIC0;
        const bool needs_svgs_elem = vs_prog_data->uses_vertexid ||
                vs_prog_data->uses_instanceid ||
                vs_prog_data->uses_basevertex ||
                vs_prog_data->uses_baseinstance;

        uint32_t elem_count = __builtin_popcount(elements) -
                __builtin_popcount(elements_double) / 2;

        const uint32_t total_elems =
                elem_count + needs_svgs_elem + vs_prog_data->uses_drawid;
        if (total_elems == 0)
                return;

        uint32_t *p;

        const uint32_t num_dwords = 1 + total_elems * 2;
        p = bcmv_batch_emitn(&pipeline->batch, num_dwords,
                             V3DX(3DSTATE_VERTEX_ELEMENTS));
        if (!p)
                return;
        memset(p + 1, 0, (num_dwords - 1) * 4);

        for (uint32_t i = 0; i < info->vertexAttributeDescriptionCount; i++) {
                const VkVertexInputAttributeDescription *desc =
                        &info->pVertexAttributeDescriptions[i];
                enum isl_format format = bcmv_get_isl_format(&pipeline->device->info,
                                                             desc->format,
                                                             VK_IMAGE_ASPECT_COLOR_BIT,
                                                             VK_IMAGE_TILING_LINEAR);

                assert(desc->binding < MAX_VBS);

                if ((elements & (1 << desc->location)) == 0)
                        continue; /* Binding unused */

                uint32_t slot =
                        __builtin_popcount(elements & ((1 << desc->location) - 1)) -
                        DIV_ROUND_UP(__builtin_popcount(elements_double &
                                                        ((1 << desc->location) -1)), 2);

                struct V3DX(VERTEX_ELEMENT_STATE) element = {
                        .VertexBufferIndex = desc->binding,
                        .Valid = true,
                        .SourceElementFormat = (enum V3DX(SURFACE_FORMAT)) format,
                        .EdgeFlagEnable = false,
                        .SourceElementOffset = desc->offset,
                        .Component0Control = vertex_element_comp_control(format, 0),
                        .Component1Control = vertex_element_comp_control(format, 1),
                        .Component2Control = vertex_element_comp_control(format, 2),
                        .Component3Control = vertex_element_comp_control(format, 3),
                };
                V3DX(VERTEX_ELEMENT_STATE_pack)(NULL, &p[1 + slot * 2], &element);

#if GEN_GEN >= 8
                /* On Broadwell and later, we have a separate VF_INSTANCING packet
                 * that controls instancing.  On Haswell and prior, that's part of
                 * VERTEX_BUFFER_STATE which we emit later.
                 */
                bcmv_batch_emit(&pipeline->batch, V3DX(3DSTATE_VF_INSTANCING), vfi) {
                        vfi.InstancingEnable = pipeline->instancing_enable[desc->binding];
                        vfi.VertexElementIndex = slot;
                        /* Our implementation of VK_KHX_multiview uses instancing to draw
                         * the different views.  If the client asks for instancing, we
                         * need to use the Instance Data Step Rate to ensure that we
                         * repeat the client's per-instance data once for each view.
                         */
                        vfi.InstanceDataStepRate = bcmv_subpass_view_count(pipeline->subpass);
                }
#endif
        }

        const uint32_t id_slot = elem_count;
        if (needs_svgs_elem) {
                /* From the Broadwell PRM for the 3D_Vertex_Component_Control enum:
                 *    "Within a VERTEX_ELEMENT_STATE structure, if a Component
                 *    Control field is set to something other than VFCOMP_STORE_SRC,
                 *    no higher-numbered Component Control fields may be set to
                 *    VFCOMP_STORE_SRC"
                 *
                 * This means, that if we have BaseInstance, we need BaseVertex as
                 * well.  Just do all or nothing.
                 */
                uint32_t base_ctrl = (vs_prog_data->uses_basevertex ||
                                      vs_prog_data->uses_baseinstance) ?
                        VFCOMP_STORE_SRC : VFCOMP_STORE_0;

                struct V3DX(VERTEX_ELEMENT_STATE) element = {
                        .VertexBufferIndex = BCMV_SVGS_VB_INDEX,
                        .Valid = true,
                        .SourceElementFormat = (enum V3DX(SURFACE_FORMAT)) ISL_FORMAT_R32G32_UINT,
                        .Component0Control = base_ctrl,
                        .Component1Control = base_ctrl,
#if GEN_GEN >= 8
                        .Component2Control = VFCOMP_STORE_0,
                        .Component3Control = VFCOMP_STORE_0,
#else
                        .Component2Control = VFCOMP_STORE_VID,
                        .Component3Control = VFCOMP_STORE_IID,
#endif
                };
                V3DX(VERTEX_ELEMENT_STATE_pack)(NULL, &p[1 + id_slot * 2], &element);
        }

#if GEN_GEN >= 8
        bcmv_batch_emit(&pipeline->batch, V3DX(3DSTATE_VF_SGVS), sgvs) {
                sgvs.VertexIDEnable              = vs_prog_data->uses_vertexid;
                sgvs.VertexIDComponentNumber     = 2;
                sgvs.VertexIDElementOffset       = id_slot;
                sgvs.InstanceIDEnable            = vs_prog_data->uses_instanceid;
                sgvs.InstanceIDComponentNumber   = 3;
                sgvs.InstanceIDElementOffset     = id_slot;
        }
#endif

        const uint32_t drawid_slot = elem_count + needs_svgs_elem;
        if (vs_prog_data->uses_drawid) {
                struct V3DX(VERTEX_ELEMENT_STATE) element = {
                        .VertexBufferIndex = BCMV_DRAWID_VB_INDEX,
                        .Valid = true,
                        .SourceElementFormat = (enum V3DX(SURFACE_FORMAT)) ISL_FORMAT_R32_UINT,
                        .Component0Control = VFCOMP_STORE_SRC,
                        .Component1Control = VFCOMP_STORE_0,
                        .Component2Control = VFCOMP_STORE_0,
                        .Component3Control = VFCOMP_STORE_0,
                };
                V3DX(VERTEX_ELEMENT_STATE_pack)(NULL,
                                                &p[1 + drawid_slot * 2],
                                                &element);

#if GEN_GEN >= 8
                bcmv_batch_emit(&pipeline->batch, V3DX(3DSTATE_VF_INSTANCING), vfi) {
                        vfi.VertexElementIndex = drawid_slot;
                }
#endif
        }
#endif
}

#if 0 /* XXX */
void
v3dx(emit_urb_setup)(struct bcmv_device *device, struct bcmv_batch *batch,
                     const struct gen_l3_config *l3_config,
                     VkShaderStageFlags active_stages,
                     const unsigned entry_size[4])
{
#if 0 /* XXX */
        const struct v3d_device_info *devinfo = &device->info;
#if GEN_IS_HASWELL
        const unsigned push_constant_kb = devinfo->gt == 3 ? 32 : 16;
#else
        const unsigned push_constant_kb = GEN_GEN >= 8 ? 32 : 16;
#endif

        const unsigned urb_size_kb = gen_get_l3_config_urb_size(devinfo, l3_config);

        unsigned entries[4];
        unsigned start[4];
        gen_get_urb_config(devinfo,
                           1024 * push_constant_kb, 1024 * urb_size_kb,
                           active_stages &
                           VK_SHADER_STAGE_TESSELLATION_EVALUATION_BIT,
                           active_stages & VK_SHADER_STAGE_GEOMETRY_BIT,
                           entry_size, entries, start);

#if GEN_GEN == 7 && !GEN_IS_HASWELL
        /* From the IVB PRM Vol. 2, Part 1, Section 3.2.1:
         *
         *    "A PIPE_CONTROL with Post-Sync Operation set to 1h and a depth stall
         *    needs to be sent just prior to any 3DSTATE_VS, 3DSTATE_URB_VS,
         *    3DSTATE_CONSTANT_VS, 3DSTATE_BINDING_TABLE_POINTER_VS,
         *    3DSTATE_SAMPLER_STATE_POINTER_VS command.  Only one PIPE_CONTROL
         *    needs to be sent before any combination of VS associated 3DSTATE."
         */
        bcmv_batch_emit(batch, GEN7_PIPE_CONTROL, pc) {
                pc.DepthStallEnable  = true;
                pc.PostSyncOperation = WriteImmediateData;
                pc.Address           = (struct bcmv_address) { &device->workaround_bo, 0 };
        }
#endif

        for (int i = 0; i <= MESA_SHADER_GEOMETRY; i++) {
                bcmv_batch_emit(batch, V3DX(3DSTATE_URB_VS), urb) {
                        urb._3DCommandSubOpcode      += i;
                        urb.VSURBStartingAddress      = start[i];
                        urb.VSURBEntryAllocationSize  = entry_size[i] - 1;
                        urb.VSNumberofURBEntries      = entries[i];
                }
        }
#endif
}
#endif

static void
emit_urb_setup(struct bcmv_pipeline *pipeline)
{
#if 0 /* XXX */
        unsigned entry_size[4];
        for (int i = MESA_SHADER_VERTEX; i <= MESA_SHADER_GEOMETRY; i++) {
                const struct brw_vue_prog_data *prog_data =
                        !bcmv_pipeline_has_stage(pipeline, i) ? NULL :
                        (const struct brw_vue_prog_data *) pipeline->shaders[i]->prog_data;

                entry_size[i] = prog_data ? prog_data->urb_entry_size : 1;
        }

        v3dx(emit_urb_setup)(pipeline->device, &pipeline->batch,
                             pipeline->urb.l3_config,
                             pipeline->active_stages, entry_size);
#endif
}

static void
emit_3dstate_sbe(struct bcmv_pipeline *pipeline)
{
#if 0 /* XXX */
        const struct brw_wm_prog_data *wm_prog_data = get_wm_prog_data(pipeline);

        if (!bcmv_pipeline_has_stage(pipeline, MESA_SHADER_FRAGMENT)) {
                bcmv_batch_emit(&pipeline->batch, V3DX(3DSTATE_SBE), sbe);
#if GEN_GEN >= 8
                bcmv_batch_emit(&pipeline->batch, V3DX(3DSTATE_SBE_SWIZ), sbe);
#endif
                return;
        }

        const struct brw_vue_map *fs_input_map =
                &bcmv_pipeline_get_last_vue_prog_data(pipeline)->vue_map;

        struct V3DX(3DSTATE_SBE) sbe = {
                V3DX(3DSTATE_SBE_header),
                .AttributeSwizzleEnable = true,
                .PointSpriteTextureCoordinateOrigin = UPPERLEFT,
                .NumberofSFOutputAttributes = wm_prog_data->num_varying_inputs,
                .ConstantInterpolationEnable = wm_prog_data->flat_inputs,
        };

#if GEN_GEN >= 9
        for (unsigned i = 0; i < 32; i++)
                sbe.AttributeActiveComponentFormat[i] = ACF_XYZW;
#endif

#if GEN_GEN >= 8
        /* On Broadwell, they broke 3DSTATE_SBE into two packets */
        struct V3DX(3DSTATE_SBE_SWIZ) swiz = {
                V3DX(3DSTATE_SBE_SWIZ_header),
        };
#else
#  define swiz sbe
#endif

        /* Skip the VUE header and position slots by default */
        unsigned urb_entry_read_offset = 1;
        int max_source_attr = 0;
        for (int attr = 0; attr < VARYING_SLOT_MAX; attr++) {
                int input_index = wm_prog_data->urb_setup[attr];

                if (input_index < 0)
                        continue;

                /* gl_Layer is stored in the VUE header */
                if (attr == VARYING_SLOT_LAYER) {
                        urb_entry_read_offset = 0;
                        continue;
                }

                if (attr == VARYING_SLOT_PNTC) {
                        sbe.PointSpriteTextureCoordinateEnable = 1 << input_index;
                        continue;
                }

                const int slot = fs_input_map->varying_to_slot[attr];

                if (input_index >= 16)
                        continue;

                if (slot == -1) {
                        /* This attribute does not exist in the VUE--that means that the
                         * vertex shader did not write to it.  It could be that it's a
                         * regular varying read by the fragment shader but not written by
                         * the vertex shader or it's gl_PrimitiveID. In the first case the
                         * value is undefined, in the second it needs to be
                         * gl_PrimitiveID.
                         */
                        swiz.Attribute[input_index].ConstantSource = PRIM_ID;
                        swiz.Attribute[input_index].ComponentOverrideX = true;
                        swiz.Attribute[input_index].ComponentOverrideY = true;
                        swiz.Attribute[input_index].ComponentOverrideZ = true;
                        swiz.Attribute[input_index].ComponentOverrideW = true;
                } else {
                        /* We have to subtract two slots to accout for the URB entry output
                         * read offset in the VS and GS stages.
                         */
                        assert(slot >= 2);
                        const int source_attr = slot - 2 * urb_entry_read_offset;
                        max_source_attr = MAX2(max_source_attr, source_attr);
                        swiz.Attribute[input_index].SourceAttribute = source_attr;
                }
        }

        sbe.VertexURBEntryReadOffset = urb_entry_read_offset;
        sbe.VertexURBEntryReadLength = DIV_ROUND_UP(max_source_attr + 1, 2);
#if GEN_GEN >= 8
        sbe.ForceVertexURBEntryReadOffset = true;
        sbe.ForceVertexURBEntryReadLength = true;
#endif

        uint32_t *dw = bcmv_batch_emit_dwords(&pipeline->batch,
                                              V3DX(3DSTATE_SBE_length));
        if (!dw)
                return;
        V3DX(3DSTATE_SBE_pack)(&pipeline->batch, dw, &sbe);

#if GEN_GEN >= 8
        dw = bcmv_batch_emit_dwords(&pipeline->batch, V3DX(3DSTATE_SBE_SWIZ_length));
        if (!dw)
                return;
        V3DX(3DSTATE_SBE_SWIZ_pack)(&pipeline->batch, dw, &swiz);
#endif
#endif
}

#if 0 /* XXX */
static const uint32_t vk_to_v3d_fillmode[] = {
        [VK_POLYGON_MODE_FILL]                    = FILL_MODE_SOLID,
        [VK_POLYGON_MODE_LINE]                    = FILL_MODE_WIREFRAME,
        [VK_POLYGON_MODE_POINT]                   = FILL_MODE_POINT,
};
#endif

static uint16_t
float_to_187_half(float f)
{
        return fui(f) >> 16;
}

static void
emit_rs_state(struct bcmv_pipeline *pipeline,
              const VkPipelineRasterizationStateCreateInfo *rs_info,
              const VkPipelineMultisampleStateCreateInfo *ms_info,
              const struct bcmv_render_pass *pass,
              const struct bcmv_subpass *subpass)
{

#if 0 /* XXX */
        struct V3DX(3DSTATE_SF) sf = {
                V3DX(3DSTATE_SF_header),
        };

        sf.ViewportTransformEnable = true;
        sf.StatisticsEnable = true;
        sf.TriangleStripListProvokingVertexSelect = 0;
        sf.LineStripListProvokingVertexSelect = 0;
        sf.TriangleFanProvokingVertexSelect = 1;

        const struct brw_vue_prog_data *last_vue_prog_data =
                bcmv_pipeline_get_last_vue_prog_data(pipeline);

        if (last_vue_prog_data->vue_map.slots_valid & VARYING_BIT_PSIZ) {
                sf.PointWidthSource = Vertex;
        } else {
                sf.PointWidthSource = State;
                sf.PointWidth = 1.0;
        }

#if GEN_GEN >= 8
        struct V3DX(3DSTATE_RASTER) raster = {
                V3DX(3DSTATE_RASTER_header),
        };
#else
#  define raster sf
#endif

        /* For details on 3DSTATE_RASTER multisample state, see the BSpec table
         * "Multisample Modes State".
         */
#if GEN_GEN >= 8
        raster.DXMultisampleRasterizationEnable = true;
        /* NOTE: 3DSTATE_RASTER::ForcedSampleCount affects the BDW and SKL PMA fix
         * computations.  If we ever set this bit to a different value, they will
         * need to be updated accordingly.
         */
        raster.ForcedSampleCount = FSC_NUMRASTSAMPLES_0;
        raster.ForceMultisampling = false;
#else
        raster.MultisampleRasterizationMode =
                (ms_info && ms_info->rasterizationSamples > 1) ?
                MSRASTMODE_ON_PATTERN : MSRASTMODE_OFF_PIXEL;
#endif

        raster.FrontWinding = vk_to_v3d_front_face[rs_info->frontFace];
        raster.CullMode = vk_to_v3d_cullmode[rs_info->cullMode];
        raster.FrontFaceFillMode = vk_to_v3d_fillmode[rs_info->polygonMode];
        raster.BackFaceFillMode = vk_to_v3d_fillmode[rs_info->polygonMode];
        raster.ScissorRectangleEnable = true;

#if GEN_GEN >= 9
        /* GEN9+ splits ViewportZClipTestEnable into near and far enable bits */
        raster.ViewportZFarClipTestEnable = !pipeline->depth_clamp_enable;
        raster.ViewportZNearClipTestEnable = !pipeline->depth_clamp_enable;
#elif GEN_GEN >= 8
        raster.ViewportZClipTestEnable = !pipeline->depth_clamp_enable;
#endif

        raster.GlobalDepthOffsetEnableSolid = rs_info->depthBiasEnable;
        raster.GlobalDepthOffsetEnableWireframe = rs_info->depthBiasEnable;
        raster.GlobalDepthOffsetEnablePoint = rs_info->depthBiasEnable;

#if GEN_GEN == 7
        /* Gen7 requires that we provide the depth format in 3DSTATE_SF so that it
         * can get the depth offsets correct.
         */
        if (subpass->depth_stencil_attachment.attachment < pass->attachment_count) {
                VkFormat vk_format =
                        pass->attachments[subpass->depth_stencil_attachment.attachment].format;
                assert(vk_format_is_depth_or_stencil(vk_format));
                if (vk_format_aspects(vk_format) & VK_IMAGE_ASPECT_DEPTH_BIT) {
                        enum isl_format isl_format =
                                bcmv_get_isl_format(&pipeline->device->info, vk_format,
                                                    VK_IMAGE_ASPECT_DEPTH_BIT,
                                                    VK_IMAGE_TILING_OPTIMAL);
                        sf.DepthBufferSurfaceFormat =
                                isl_format_get_depth_format(isl_format, false);
                }
        }
#endif

#if GEN_GEN >= 8
        V3DX(3DSTATE_SF_pack)(NULL, pipeline->gen8.sf, &sf);
        V3DX(3DSTATE_RASTER_pack)(NULL, pipeline->gen8.raster, &raster);
#else
#  undef raster
        V3DX(3DSTATE_SF_pack)(NULL, &pipeline->gen7.sf, &sf);
#endif
#endif
}

static void
emit_ms_state(struct bcmv_pipeline *pipeline,
              const VkPipelineMultisampleStateCreateInfo *info)
{
#if 0 /* XXX */
        uint32_t samples = 1;
        uint32_t log2_samples = 0;

        /* From the Vulkan 1.0 spec:
         *    If pSampleMask is NULL, it is treated as if the mask has all bits
         *    enabled, i.e. no coverage is removed from fragments.
         *
         * 3DSTATE_SAMPLE_MASK.SampleMask is 16 bits.
         */
#if GEN_GEN >= 8
        uint32_t sample_mask = 0xffff;
#else
        uint32_t sample_mask = 0xff;
#endif

        if (info) {
                samples = info->rasterizationSamples;
                log2_samples = __builtin_ffs(samples) - 1;
        }

        if (info && info->pSampleMask)
                sample_mask &= info->pSampleMask[0];

        bcmv_batch_emit(&pipeline->batch, V3DX(3DSTATE_MULTISAMPLE), ms) {
                ms.NumberofMultisamples       = log2_samples;

                ms.PixelLocation              = CENTER;
#if GEN_GEN >= 8
                /* The PRM says that this bit is valid only for DX9:
                 *
                 *    SW can choose to set this bit only for DX9 API. DX10/OGL API's
                 *    should not have any effect by setting or not setting this bit.
                 */
                ms.PixelPositionOffsetEnable  = false;
#else

                switch (samples) {
                case 1:
                        GEN_SAMPLE_POS_1X(ms.Sample);
                        break;
                case 2:
                        GEN_SAMPLE_POS_2X(ms.Sample);
                        break;
                case 4:
                        GEN_SAMPLE_POS_4X(ms.Sample);
                        break;
                case 8:
                        GEN_SAMPLE_POS_8X(ms.Sample);
                        break;
                default:
                        break;
                }
#endif
        }

        bcmv_batch_emit(&pipeline->batch, V3DX(3DSTATE_SAMPLE_MASK), sm) {
                sm.SampleMask = sample_mask;
        }
#endif
}

#if 0 /* XXX */
static const uint32_t vk_to_v3d_logic_op[] = {
        [VK_LOGIC_OP_COPY]                        = LOGICOP_COPY,
        [VK_LOGIC_OP_CLEAR]                       = LOGICOP_CLEAR,
        [VK_LOGIC_OP_AND]                         = LOGICOP_AND,
        [VK_LOGIC_OP_AND_REVERSE]                 = LOGICOP_AND_REVERSE,
        [VK_LOGIC_OP_AND_INVERTED]                = LOGICOP_AND_INVERTED,
        [VK_LOGIC_OP_NO_OP]                       = LOGICOP_NOOP,
        [VK_LOGIC_OP_XOR]                         = LOGICOP_XOR,
        [VK_LOGIC_OP_OR]                          = LOGICOP_OR,
        [VK_LOGIC_OP_NOR]                         = LOGICOP_NOR,
        [VK_LOGIC_OP_EQUIVALENT]                  = LOGICOP_EQUIV,
        [VK_LOGIC_OP_INVERT]                      = LOGICOP_INVERT,
        [VK_LOGIC_OP_OR_REVERSE]                  = LOGICOP_OR_REVERSE,
        [VK_LOGIC_OP_COPY_INVERTED]               = LOGICOP_COPY_INVERTED,
        [VK_LOGIC_OP_OR_INVERTED]                 = LOGICOP_OR_INVERTED,
        [VK_LOGIC_OP_NAND]                        = LOGICOP_NAND,
        [VK_LOGIC_OP_SET]                         = LOGICOP_SET,
};
#endif

static const uint32_t vk_to_v3d_blend[] = {
        [VK_BLEND_FACTOR_ZERO]                    = V3D_BLEND_FACTOR_ZERO,
        [VK_BLEND_FACTOR_ONE]                     = V3D_BLEND_FACTOR_ONE,
        [VK_BLEND_FACTOR_SRC_COLOR]               = V3D_BLEND_FACTOR_SRC_COLOR,
        [VK_BLEND_FACTOR_ONE_MINUS_SRC_COLOR]     = V3D_BLEND_FACTOR_INV_SRC_COLOR,
        [VK_BLEND_FACTOR_DST_COLOR]               = V3D_BLEND_FACTOR_DST_COLOR,
        [VK_BLEND_FACTOR_ONE_MINUS_DST_COLOR]     = V3D_BLEND_FACTOR_INV_DST_COLOR,
        [VK_BLEND_FACTOR_SRC_ALPHA]               = V3D_BLEND_FACTOR_SRC_ALPHA,
        [VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA]     = V3D_BLEND_FACTOR_INV_SRC_ALPHA,
        [VK_BLEND_FACTOR_DST_ALPHA]               = V3D_BLEND_FACTOR_DST_ALPHA,
        [VK_BLEND_FACTOR_ONE_MINUS_DST_ALPHA]     = V3D_BLEND_FACTOR_INV_DST_ALPHA,
        [VK_BLEND_FACTOR_CONSTANT_COLOR]          = V3D_BLEND_FACTOR_CONST_COLOR,
        [VK_BLEND_FACTOR_ONE_MINUS_CONSTANT_COLOR]= V3D_BLEND_FACTOR_INV_CONST_COLOR,
        [VK_BLEND_FACTOR_CONSTANT_ALPHA]          = V3D_BLEND_FACTOR_CONST_ALPHA,
        [VK_BLEND_FACTOR_ONE_MINUS_CONSTANT_ALPHA]= V3D_BLEND_FACTOR_INV_CONST_ALPHA,
        [VK_BLEND_FACTOR_SRC_ALPHA_SATURATE]      = V3D_BLEND_FACTOR_SRC_ALPHA_SATURATE,
};

static const uint32_t vk_to_v3d_blend_op[] = {
        [VK_BLEND_OP_ADD]                         = V3D_BLEND_MODE_ADD,
        [VK_BLEND_OP_SUBTRACT]                    = V3D_BLEND_MODE_SUB,
        [VK_BLEND_OP_REVERSE_SUBTRACT]            = V3D_BLEND_MODE_RSUB,
        [VK_BLEND_OP_MIN]                         = V3D_BLEND_MODE_MIN,
        [VK_BLEND_OP_MAX]                         = V3D_BLEND_MODE_MAX,
};

static const uint32_t vk_to_v3d_compare_op[] = {
        [VK_COMPARE_OP_NEVER]                        = V3D_COMPARE_FUNC_NEVER,
        [VK_COMPARE_OP_LESS]                         = V3D_COMPARE_FUNC_LESS,
        [VK_COMPARE_OP_EQUAL]                        = V3D_COMPARE_FUNC_EQUAL,
        [VK_COMPARE_OP_LESS_OR_EQUAL]                = V3D_COMPARE_FUNC_LEQUAL,
        [VK_COMPARE_OP_GREATER]                      = V3D_COMPARE_FUNC_GREATER,
        [VK_COMPARE_OP_NOT_EQUAL]                    = V3D_COMPARE_FUNC_NOTEQUAL,
        [VK_COMPARE_OP_GREATER_OR_EQUAL]             = V3D_COMPARE_FUNC_GEQUAL,
        [VK_COMPARE_OP_ALWAYS]                       = V3D_COMPARE_FUNC_ALWAYS,
};

static const uint32_t vk_to_v3d_stencil_op[] = {
        [VK_STENCIL_OP_KEEP]                         = V3D_STENCIL_OP_KEEP,
        [VK_STENCIL_OP_ZERO]                         = V3D_STENCIL_OP_ZERO,
        [VK_STENCIL_OP_REPLACE]                      = V3D_STENCIL_OP_REPLACE,
        [VK_STENCIL_OP_INCREMENT_AND_CLAMP]          = V3D_STENCIL_OP_INCR,
        [VK_STENCIL_OP_DECREMENT_AND_CLAMP]          = V3D_STENCIL_OP_DECR,
        [VK_STENCIL_OP_INVERT]                       = V3D_STENCIL_OP_INVERT,
        [VK_STENCIL_OP_INCREMENT_AND_WRAP]           = V3D_STENCIL_OP_INCWRAP,
        [VK_STENCIL_OP_DECREMENT_AND_WRAP]           = V3D_STENCIL_OP_DECWRAP,
};

/* This function sanitizes the VkStencilOpState by looking at the compare ops
 * and trying to determine whether or not a given stencil op can ever actually
 * occur.  Stencil ops which can never occur are set to VK_STENCIL_OP_KEEP.
 * This function returns true if, after sanitation, any of the stencil ops are
 * set to something other than VK_STENCIL_OP_KEEP.
 */
static bool
sanitize_stencil_face(VkStencilOpState *face,
                      VkCompareOp depthCompareOp)
{
        /* If compareOp is ALWAYS then the stencil test will never fail and
         * failOp will never happen.  Set failOp to KEEP in this case.
         */
        if (face->compareOp == VK_COMPARE_OP_ALWAYS)
                face->failOp = VK_STENCIL_OP_KEEP;

        /* If compareOp is NEVER or depthCompareOp is NEVER then one of the
         * depth or stencil tests will fail and passOp will never happen.
         */
        if (face->compareOp == VK_COMPARE_OP_NEVER ||
            depthCompareOp == VK_COMPARE_OP_NEVER)
                face->passOp = VK_STENCIL_OP_KEEP;

        /* If compareOp is NEVER or depthCompareOp is ALWAYS then either the
         * stencil test will fail or the depth test will pass.  In either
         * case, depthFailOp will never happen.
         */
        if (face->compareOp == VK_COMPARE_OP_NEVER ||
            depthCompareOp == VK_COMPARE_OP_ALWAYS)
                face->depthFailOp = VK_STENCIL_OP_KEEP;

        return face->failOp != VK_STENCIL_OP_KEEP ||
                face->depthFailOp != VK_STENCIL_OP_KEEP ||
                face->passOp != VK_STENCIL_OP_KEEP;
}

/* Disables Z/S update and test flags when they're a no-op. */
static void
sanitize_ds_state(VkPipelineDepthStencilStateCreateInfo *state,
                  bool *stencilWriteEnable,
                  VkImageAspectFlags ds_aspects)
{
        *stencilWriteEnable = state->stencilTestEnable;

        /* If the depth test is disabled, we won't be writing anything. */
        if (!state->depthTestEnable)
                state->depthWriteEnable = false;

        /* The Vulkan spec requires that if either depth or stencil is not present,
         * the pipeline is to act as if the test silently passes.
         */
        if (!(ds_aspects & VK_IMAGE_ASPECT_DEPTH_BIT)) {
                state->depthWriteEnable = false;
                state->depthCompareOp = VK_COMPARE_OP_ALWAYS;
        }

        if (!(ds_aspects & VK_IMAGE_ASPECT_STENCIL_BIT)) {
                *stencilWriteEnable = false;
                state->front.compareOp = VK_COMPARE_OP_ALWAYS;
                state->back.compareOp = VK_COMPARE_OP_ALWAYS;
        }

        /* If the stencil test is enabled and always fails, then we will never get
         * to the depth test so we can just disable the depth test entirely.
         */
        if (state->stencilTestEnable &&
            state->front.compareOp == VK_COMPARE_OP_NEVER &&
            state->back.compareOp == VK_COMPARE_OP_NEVER) {
                state->depthTestEnable = false;
                state->depthWriteEnable = false;
        }

        /* If depthCompareOp is EQUAL then the value we would be writing to
         * the depth buffer is the same as the value that's already there so
         * there's no point in writing it.
         */
        if (state->depthCompareOp == VK_COMPARE_OP_EQUAL)
                state->depthWriteEnable = false;

        /* If the stencil ops are such that we don't actually ever modify the
         * stencil buffer, we should disable writes.
         */
        if (!sanitize_stencil_face(&state->front, state->depthCompareOp) &&
            !sanitize_stencil_face(&state->back, state->depthCompareOp))
                *stencilWriteEnable = false;

        /* If the depth test always passes and we never write out depth, that's the
         * same as if the depth test is disabled entirely.
         */
        if (state->depthCompareOp == VK_COMPARE_OP_ALWAYS &&
            !state->depthWriteEnable)
                state->depthTestEnable = false;

        /* If the stencil test always passes and we never write out stencil, that's
         * the same as if the stencil test is disabled entirely.
         */
        if (state->front.compareOp == VK_COMPARE_OP_ALWAYS &&
            state->back.compareOp == VK_COMPARE_OP_ALWAYS &&
            !*stencilWriteEnable)
                state->stencilTestEnable = false;
}

static void
emit_ds_state(struct bcmv_pipeline *pipeline,
              const VkPipelineDepthStencilStateCreateInfo *pCreateInfo,
              const struct bcmv_render_pass *pass,
              const struct bcmv_subpass *subpass)
{
        if (pCreateInfo == NULL) {
                /* We're going to OR this together with the dynamic (stencil
                 * mask/ref) state.  We need to make sure it's initialized to
                 * something useful.
                 */
                pipeline->writes_stencil = false;
                pipeline->stencil_test_enable = false;
                pipeline->writes_depth = false;
                pipeline->depth_test_enable = false;
                return;
        }

        VkImageAspectFlags ds_aspects = 0;
        if (subpass->depth_stencil_attachment.attachment != VK_ATTACHMENT_UNUSED) {
                VkFormat depth_stencil_format =
                        pass->attachments[subpass->depth_stencil_attachment.attachment].format;
                ds_aspects = vk_format_aspects(depth_stencil_format);
        }

        VkPipelineDepthStencilStateCreateInfo info = *pCreateInfo;
        sanitize_ds_state(&info, &pipeline->writes_stencil, ds_aspects);
        pipeline->stencil_test_enable = info.stencilTestEnable;
        pipeline->writes_depth = info.depthWriteEnable;
        pipeline->depth_test_enable = info.depthTestEnable;

        /* VkBool32 depthBoundsTestEnable; // optional (depth_bounds_test) */

#if 0 /* XXX */
        struct V3DX(DEPTH_STENCIL_STATE) depth_stencil = {
                .DepthTestEnable = info.depthTestEnable,
                .DepthBufferWriteEnable = info.depthWriteEnable,
                .DepthTestFunction = ,
        };
#endif

        if (info.stencilTestEnable) {
                /* The stencil config packet contains both the pipeline state
                 * (funcs and ops) and the dynamic (ref/mask) state.
                 */
                struct V3DX(STENCIL_CONFIG) front = {
                        .front_config = true,
                        .back_config = false,

                        .stencil_test_function = vk_to_v3d_compare_op[info.front.compareOp],
                        .stencil_pass_op = vk_to_v3d_stencil_op[info.front.passOp],
                        .depth_test_fail_op = vk_to_v3d_stencil_op[info.front.depthFailOp],
                        .stencil_test_fail_op = vk_to_v3d_stencil_op[info.front.failOp],
                };

                struct V3DX(STENCIL_CONFIG) back = {
                        .front_config = false,
                        .back_config = true,

                        .stencil_test_function = vk_to_v3d_compare_op[info.back.compareOp],
                        .stencil_pass_op = vk_to_v3d_stencil_op[info.back.passOp],
                        .depth_test_fail_op = vk_to_v3d_stencil_op[info.back.depthFailOp],
                        .stencil_test_fail_op = vk_to_v3d_stencil_op[info.back.failOp],
                };

                V3DX(STENCIL_CONFIG_pack)(NULL, pipeline->stencil_config_front,
                                          &front);
                V3DX(STENCIL_CONFIG_pack)(NULL, pipeline->stencil_config_back,
                                          &back);
        }
}

static void
emit_config_bits(struct bcmv_pipeline *pipeline,
                 const VkPipelineRasterizationStateCreateInfo *rs_info,
                 const VkPipelineDepthStencilStateCreateInfo *zsinfo,
                 const VkPipelineColorBlendStateCreateInfo *cbinfo)
{
        bcmv_bcl_emit(&pipeline->batch, V3DX(CONFIGURATION_BITS), config) {
                config.blend_enable = (cbinfo->attachmentCount > 0 &&
                                       cbinfo->pAttachments[0].blendEnable);

                config.z_updates_enable = zsinfo->depthWriteEnable;
                config.stencil_enable = zsinfo->stencilTestEnable;
                config.depth_test_function =
                        vk_to_v3d_compare_op[zsinfo->depthCompareOp];

                /* XXX: EZ */

                config.enable_depth_offset = rs_info->depthBiasEnable;
                config.clockwise_primitives =
                        rs_info->frontFace == VK_FRONT_FACE_CLOCKWISE;
                config.enable_forward_facing_primitive =
                        rs_info->cullMode & VK_CULL_MODE_FRONT_BIT;
                config.enable_reverse_facing_primitive =
                        rs_info->cullMode & VK_CULL_MODE_BACK_BIT;

                config.direct3d_wireframe_triangles_mode =
                        rs_info->polygonMode != VK_POLYGON_MODE_FILL;
                config.direct3d_point_fill_mode =
                        rs_info->polygonMode == VK_POLYGON_MODE_LINE;
        }
}

static void
emit_cb_state(struct bcmv_pipeline *pipeline,
              const VkPipelineColorBlendStateCreateInfo *info,
              const VkPipelineMultisampleStateCreateInfo *ms_info)
{
        if (info->attachmentCount > 0) {
                /* XXX: If the per-RT blend state doesn't match, then we need
                 * to do blending in the FS.
                 */
                const VkPipelineColorBlendAttachmentState *a =
                        &info->pAttachments[0];

                bcmv_bcl_emit(&pipeline->batch, V3DX(BLEND_CONFIG), blend) {
                        blend.colour_blend_mode =
                                vk_to_v3d_blend_op[a->colorBlendOp],
                        blend.colour_blend_dst_factor =
                                vk_to_v3d_blend[a->dstColorBlendFactor];
                        blend.colour_blend_src_factor =
                                vk_to_v3d_blend[a->srcColorBlendFactor];

                        blend.alpha_blend_mode =
                                vk_to_v3d_blend_op[a->alphaBlendOp],
                        blend.alpha_blend_dst_factor =
                                vk_to_v3d_blend[a->dstAlphaBlendFactor];
                        blend.alpha_blend_src_factor =
                                vk_to_v3d_blend[a->srcAlphaBlendFactor];
                }
                uint8_t wm = (!a->colorWriteMask) & 0xf;
                bcmv_bcl_emit(&pipeline->batch, V3DX(COLOUR_WRITE_MASKS), mask) {
                        mask.render_target_0_per_colour_component_write_masks =
                                wm;
                        mask.render_target_1_per_colour_component_write_masks =
                                wm;
                        mask.render_target_2_per_colour_component_write_masks =
                                wm;
                        mask.render_target_3_per_colour_component_write_masks =
                                wm;
                }
        }

        /* XXX */
#if 0
        .AlphaToCoverageEnable = ms_info && ms_info->alphaToCoverageEnable;
        .AlphaToOneEnable = ms_info && ms_info->alphaToOneEnable;
        .LogicOpEnable = info->logicOpEnable;
        .LogicOpFunction = vk_to_v3d_logic_op[info->logicOp];
#endif
}

static void
emit_3dstate_clip(struct bcmv_pipeline *pipeline,
                  const VkPipelineViewportStateCreateInfo *vp_info,
                  const VkPipelineRasterizationStateCreateInfo *rs_info)
{
#if 0 /* XXX */
        const struct brw_wm_prog_data *wm_prog_data = get_wm_prog_data(pipeline);
        (void) wm_prog_data;
        bcmv_batch_emit(&pipeline->batch, V3DX(3DSTATE_CLIP), clip) {
                clip.ClipEnable               = true;
                clip.StatisticsEnable         = true;
                clip.EarlyCullEnable          = true;
                clip.APIMode                  = APIMODE_D3D,
                        clip.ViewportXYClipTestEnable = true;

                clip.ClipMode = CLIPMODE_NORMAL;

                clip.TriangleStripListProvokingVertexSelect = 0;
                clip.LineStripListProvokingVertexSelect     = 0;
                clip.TriangleFanProvokingVertexSelect       = 1;

                clip.MinimumPointWidth = 0.125;
                clip.MaximumPointWidth = 255.875;

                const struct brw_vue_prog_data *last =
                        bcmv_pipeline_get_last_vue_prog_data(pipeline);

                /* From the Vulkan 1.0.45 spec:
                 *
                 *    "If the last active vertex processing stage shader entry point's
                 *    interface does not include a variable decorated with
                 *    ViewportIndex, then the first viewport is used."
                 */
                if (vp_info && (last->vue_map.slots_valid & VARYING_BIT_VIEWPORT)) {
                        clip.MaximumVPIndex = vp_info->viewportCount - 1;
                } else {
                        clip.MaximumVPIndex = 0;
                }

                /* From the Vulkan 1.0.45 spec:
                 *
                 *    "If the last active vertex processing stage shader entry point's
                 *    interface does not include a variable decorated with Layer, then
                 *    the first layer is used."
                 */
                clip.ForceZeroRTAIndexEnable =
                        !(last->vue_map.slots_valid & VARYING_BIT_LAYER);

#if GEN_GEN == 7
                clip.FrontWinding            = vk_to_v3d_front_face[rs_info->frontFace];
                clip.CullMode                = vk_to_v3d_cullmode[rs_info->cullMode];
                clip.ViewportZClipTestEnable = !pipeline->depth_clamp_enable;
                if (last) {
                        clip.UserClipDistanceClipTestEnableBitmask = last->clip_distance_mask;
                        clip.UserClipDistanceCullTestEnableBitmask = last->cull_distance_mask;
                }
#else
                clip.NonPerspectiveBarycentricEnable = wm_prog_data ?
                        (wm_prog_data->barycentric_interp_modes &
                         BRW_BARYCENTRIC_NONPERSPECTIVE_BITS) != 0 : 0;
#endif
        }
#endif
}

static void
emit_3dstate_streamout(struct bcmv_pipeline *pipeline,
                       const VkPipelineRasterizationStateCreateInfo *rs_info)
{
#if 0 /* XXX */
        bcmv_batch_emit(&pipeline->batch, V3DX(3DSTATE_STREAMOUT), so) {
                so.RenderingDisable = rs_info->rasterizerDiscardEnable;
        }
#endif
}

#if 0 /* XXX */
static uint32_t
get_sampler_count(const struct bcmv_shader_bin *bin)
{
        return DIV_ROUND_UP(bin->bind_map.sampler_count, 4);
}

static uint32_t
get_binding_table_entry_count(const struct bcmv_shader_bin *bin)
{
        return DIV_ROUND_UP(bin->bind_map.surface_count, 32);
}
#endif

static void
emit_3dstate_vs(struct bcmv_pipeline *pipeline)
{
#if 0 /* XXX */
        const struct gen_device_info *devinfo = &pipeline->device->info;
        const struct brw_vs_prog_data *vs_prog_data = get_vs_prog_data(pipeline);
        const struct bcmv_shader_bin *vs_bin =
                pipeline->shaders[MESA_SHADER_VERTEX];

        assert(bcmv_pipeline_has_stage(pipeline, MESA_SHADER_VERTEX));

        bcmv_batch_emit(&pipeline->batch, V3DX(3DSTATE_VS), vs) {
                vs.Enable               = true;
                vs.StatisticsEnable     = true;
                vs.KernelStartPointer   = vs_bin->kernel.offset;
#if GEN_GEN >= 8
                vs.SIMD8DispatchEnable  =
                        vs_prog_data->base.dispatch_mode == DISPATCH_MODE_SIMD8;
#endif

                assert(!vs_prog_data->base.base.use_alt_mode);
                vs.SingleVertexDispatch       = false;
                vs.VectorMaskEnable           = false;
                vs.SamplerCount               = get_sampler_count(vs_bin);
                vs.BindingTableEntryCount     = get_binding_table_entry_count(vs_bin);
                vs.FloatingPointMode          = IEEE754;
                vs.IllegalOpcodeExceptionEnable = false;
                vs.SoftwareExceptionEnable    = false;
                vs.MaximumNumberofThreads     = devinfo->max_vs_threads - 1;
                vs.VertexCacheDisable         = false;

                vs.VertexURBEntryReadLength      = vs_prog_data->base.urb_read_length;
                vs.VertexURBEntryReadOffset      = 0;
                vs.DispatchGRFStartRegisterForURBData =
                        vs_prog_data->base.base.dispatch_grf_start_reg;

#if GEN_GEN >= 8
                vs.VertexURBEntryOutputReadOffset = get_urb_output_offset();
                vs.VertexURBEntryOutputLength     = get_urb_output_length(vs_bin);

                vs.UserClipDistanceClipTestEnableBitmask =
                        vs_prog_data->base.clip_distance_mask;
                vs.UserClipDistanceCullTestEnableBitmask =
                        vs_prog_data->base.cull_distance_mask;
#endif

                vs.PerThreadScratchSpace   = get_scratch_space(vs_bin);
                vs.ScratchSpaceBasePointer =
                        get_scratch_address(pipeline, MESA_SHADER_VERTEX, vs_bin);
        }
#endif
}

static bool
has_color_buffer_write_enabled(const struct bcmv_pipeline *pipeline)
{
        const struct bcmv_shader_bin *shader_bin =
                pipeline->shaders[MESA_SHADER_FRAGMENT];
        if (!shader_bin)
                return false;

        const struct bcmv_pipeline_bind_map *bind_map = &shader_bin->bind_map;
        for (int i = 0; i < bind_map->surface_count; i++) {
                if (bind_map->surface_to_descriptor[i].set !=
                    BCMV_DESCRIPTOR_SET_COLOR_ATTACHMENTS)
                        continue;
                if (bind_map->surface_to_descriptor[i].index != UINT32_MAX)
                        return true;
        }

        return false;
}

static void
emit_3dstate_wm(struct bcmv_pipeline *pipeline, struct bcmv_subpass *subpass,
                const VkPipelineMultisampleStateCreateInfo *multisample)
{
#if 0 /* XXX */
        const struct brw_wm_prog_data *wm_prog_data = get_wm_prog_data(pipeline);

        MAYBE_UNUSED uint32_t samples =
                multisample ? multisample->rasterizationSamples : 1;

        bcmv_batch_emit(&pipeline->batch, V3DX(3DSTATE_WM), wm) {
                wm.StatisticsEnable                    = true;
                wm.LineEndCapAntialiasingRegionWidth   = _05pixels;
                wm.LineAntialiasingRegionWidth         = _10pixels;
                wm.PointRasterizationRule              = RASTRULE_UPPER_RIGHT;

                if (bcmv_pipeline_has_stage(pipeline, MESA_SHADER_FRAGMENT)) {
                        if (wm_prog_data->early_fragment_tests) {
                                wm.EarlyDepthStencilControl         = EDSC_PREPS;
                        } else if (wm_prog_data->has_side_effects) {
                                wm.EarlyDepthStencilControl         = EDSC_PSEXEC;
                        } else {
                                wm.EarlyDepthStencilControl         = EDSC_NORMAL;
                        }

                        wm.BarycentricInterpolationMode =
                                wm_prog_data->barycentric_interp_modes;

#if GEN_GEN < 8
                        wm.PixelShaderComputedDepthMode  = wm_prog_data->computed_depth_mode;
                        wm.PixelShaderUsesSourceDepth    = wm_prog_data->uses_src_depth;
                        wm.PixelShaderUsesSourceW        = wm_prog_data->uses_src_w;
                        wm.PixelShaderUsesInputCoverageMask = wm_prog_data->uses_sample_mask;

                        /* If the subpass has a depth or stencil self-dependency, then we
                         * need to force the hardware to do the depth/stencil write *after*
                         * fragment shader execution.  Otherwise, the writes may hit memory
                         * before we get around to fetching from the input attachment and we
                         * may get the depth or stencil value from the current draw rather
                         * than the previous one.
                         */
                        wm.PixelShaderKillsPixel         = subpass->has_ds_self_dep ||
                                wm_prog_data->uses_kill;

                        if (wm.PixelShaderComputedDepthMode != PSCDEPTH_OFF ||
                            wm_prog_data->has_side_effects ||
                            wm.PixelShaderKillsPixel ||
                            has_color_buffer_write_enabled(pipeline))
                                wm.ThreadDispatchEnable = true;

                        if (samples > 1) {
                                wm.MultisampleRasterizationMode = MSRASTMODE_ON_PATTERN;
                                if (wm_prog_data->persample_dispatch) {
                                        wm.MultisampleDispatchMode = MSDISPMODE_PERSAMPLE;
                                } else {
                                        wm.MultisampleDispatchMode = MSDISPMODE_PERPIXEL;
                                }
                        } else {
                                wm.MultisampleRasterizationMode = MSRASTMODE_OFF_PIXEL;
                                wm.MultisampleDispatchMode = MSDISPMODE_PERSAMPLE;
                        }
#endif
                }
        }
#endif
}

UNUSED static bool
is_dual_src_blend_factor(VkBlendFactor factor)
{
        return factor == VK_BLEND_FACTOR_SRC1_COLOR ||
                factor == VK_BLEND_FACTOR_ONE_MINUS_SRC1_COLOR ||
                factor == VK_BLEND_FACTOR_SRC1_ALPHA ||
                factor == VK_BLEND_FACTOR_ONE_MINUS_SRC1_ALPHA;
}

static void
emit_3dstate_ps(struct bcmv_pipeline *pipeline,
                const VkPipelineColorBlendStateCreateInfo *blend)
{
#if 0 /* XXX */
        MAYBE_UNUSED const struct gen_device_info *devinfo = &pipeline->device->info;
        const struct bcmv_shader_bin *fs_bin =
                pipeline->shaders[MESA_SHADER_FRAGMENT];

        if (!bcmv_pipeline_has_stage(pipeline, MESA_SHADER_FRAGMENT)) {
                bcmv_batch_emit(&pipeline->batch, V3DX(3DSTATE_PS), ps) {
#if GEN_GEN == 7
                        /* Even if no fragments are ever dispatched, gen7 hardware hangs if
                         * we don't at least set the maximum number of threads.
                         */
                        ps.MaximumNumberofThreads = devinfo->max_wm_threads - 1;
#endif
                }
                return;
        }

        const struct brw_wm_prog_data *wm_prog_data = get_wm_prog_data(pipeline);

#if GEN_GEN < 8
        /* The hardware wedges if you have this bit set but don't turn on any dual
         * source blend factors.
         */
        bool dual_src_blend = false;
        if (wm_prog_data->dual_src_blend && blend) {
                for (uint32_t i = 0; i < blend->attachmentCount; i++) {
                        const VkPipelineColorBlendAttachmentState *bstate =
                                &blend->pAttachments[i];

                        if (bstate->blendEnable &&
                            (is_dual_src_blend_factor(bstate->srcColorBlendFactor) ||
                             is_dual_src_blend_factor(bstate->dstColorBlendFactor) ||
                             is_dual_src_blend_factor(bstate->srcAlphaBlendFactor) ||
                             is_dual_src_blend_factor(bstate->dstAlphaBlendFactor))) {
                                dual_src_blend = true;
                                break;
                        }
                }
        }
#endif

        bcmv_batch_emit(&pipeline->batch, V3DX(3DSTATE_PS), ps) {
                ps.KernelStartPointer0        = fs_bin->kernel.offset;
                ps.KernelStartPointer1        = 0;
                ps.KernelStartPointer2        = fs_bin->kernel.offset +
                        wm_prog_data->prog_offset_2;
                ps._8PixelDispatchEnable      = wm_prog_data->dispatch_8;
                ps._16PixelDispatchEnable     = wm_prog_data->dispatch_16;
                ps._32PixelDispatchEnable     = false;

                ps.SingleProgramFlow          = false;
                ps.VectorMaskEnable           = true;
                ps.SamplerCount               = get_sampler_count(fs_bin);
                ps.BindingTableEntryCount     = get_binding_table_entry_count(fs_bin);
                ps.PushConstantEnable         = wm_prog_data->base.nr_params > 0;
                ps.PositionXYOffsetSelect     = wm_prog_data->uses_pos_offset ?
                        POSOFFSET_SAMPLE: POSOFFSET_NONE;
#if GEN_GEN < 8
                ps.AttributeEnable            = wm_prog_data->num_varying_inputs > 0;
                ps.oMaskPresenttoRenderTarget = wm_prog_data->uses_omask;
                ps.DualSourceBlendEnable      = dual_src_blend;
#endif

#if GEN_IS_HASWELL
                /* Haswell requires the sample mask to be set in this packet as well
                 * as in 3DSTATE_SAMPLE_MASK; the values should match.
                 */
                ps.SampleMask                 = 0xff;
#endif

#if GEN_GEN >= 9
                ps.MaximumNumberofThreadsPerPSD  = 64 - 1;
#elif GEN_GEN >= 8
                ps.MaximumNumberofThreadsPerPSD  = 64 - 2;
#else
                ps.MaximumNumberofThreads        = devinfo->max_wm_threads - 1;
#endif

                ps.DispatchGRFStartRegisterForConstantSetupData0 =
                        wm_prog_data->base.dispatch_grf_start_reg;
                ps.DispatchGRFStartRegisterForConstantSetupData1 = 0;
                ps.DispatchGRFStartRegisterForConstantSetupData2 =
                        wm_prog_data->dispatch_grf_start_reg_2;

                ps.PerThreadScratchSpace   = get_scratch_space(fs_bin);
                ps.ScratchSpaceBasePointer =
                        get_scratch_address(pipeline, MESA_SHADER_FRAGMENT, fs_bin);
        }
#endif
}

#if GEN_GEN >= 8
static void
emit_3dstate_ps_extra(struct bcmv_pipeline *pipeline,
                      struct bcmv_subpass *subpass)
{
#if 0 /* XXX */
        const struct brw_wm_prog_data *wm_prog_data = get_wm_prog_data(pipeline);

        if (!bcmv_pipeline_has_stage(pipeline, MESA_SHADER_FRAGMENT)) {
                bcmv_batch_emit(&pipeline->batch, V3DX(3DSTATE_PS_EXTRA), ps);
                return;
        }

        bcmv_batch_emit(&pipeline->batch, V3DX(3DSTATE_PS_EXTRA), ps) {
                ps.PixelShaderValid              = true;
                ps.AttributeEnable               = wm_prog_data->num_varying_inputs > 0;
                ps.oMaskPresenttoRenderTarget    = wm_prog_data->uses_omask;
                ps.PixelShaderIsPerSample        = wm_prog_data->persample_dispatch;
                ps.PixelShaderComputedDepthMode  = wm_prog_data->computed_depth_mode;
                ps.PixelShaderUsesSourceDepth    = wm_prog_data->uses_src_depth;
                ps.PixelShaderUsesSourceW        = wm_prog_data->uses_src_w;

                /* If the subpass has a depth or stencil self-dependency, then we need
                 * to force the hardware to do the depth/stencil write *after* fragment
                 * shader execution.  Otherwise, the writes may hit memory before we get
                 * around to fetching from the input attachment and we may get the depth
                 * or stencil value from the current draw rather than the previous one.
                 */
                ps.PixelShaderKillsPixel         = subpass->has_ds_self_dep ||
                        wm_prog_data->uses_kill;

                /* The stricter cross-primitive coherency guarantees that the hardware
                 * gives us with the "Accesses UAV" bit set for at least one shader stage
                 * and the "UAV coherency required" bit set on the 3DPRIMITIVE command are
                 * redundant within the current image, atomic counter and SSBO GL APIs,
                 * which all have very loose ordering and coherency requirements and
                 * generally rely on the application to insert explicit barriers when a
                 * shader invocation is expected to see the memory writes performed by the
                 * invocations of some previous primitive.  Regardless of the value of
                 * "UAV coherency required", the "Accesses UAV" bits will implicitly cause
                 * an in most cases useless DC flush when the lowermost stage with the bit
                 * set finishes execution.
                 *
                 * It would be nice to disable it, but in some cases we can't because on
                 * Gen8+ it also has an influence on rasterization via the PS UAV-only
                 * signal (which could be set independently from the coherency mechanism
                 * in the 3DSTATE_WM command on Gen7), and because in some cases it will
                 * determine whether the hardware skips execution of the fragment shader
                 * or not via the ThreadDispatchEnable signal.  However if we know that
                 * GEN8_PS_BLEND_HAS_WRITEABLE_RT is going to be set and
                 * GEN8_PSX_PIXEL_SHADER_NO_RT_WRITE is not set it shouldn't make any
                 * difference so we may just disable it here.
                 *
                 * Gen8 hardware tries to compute ThreadDispatchEnable for us but doesn't
                 * take into account KillPixels when no depth or stencil writes are
                 * enabled. In order for occlusion queries to work correctly with no
                 * attachments, we need to force-enable here.
                 */
                if ((wm_prog_data->has_side_effects || wm_prog_data->uses_kill) &&
                    !has_color_buffer_write_enabled(pipeline))
                        ps.PixelShaderHasUAV = true;

#if GEN_GEN >= 9
                ps.PixelShaderPullsBary    = wm_prog_data->pulls_bary;
                ps.InputCoverageMaskState  = wm_prog_data->uses_sample_mask ?
                        ICMS_INNER_CONSERVATIVE : ICMS_NONE;
#else
                ps.PixelShaderUsesInputCoverageMask = wm_prog_data->uses_sample_mask;
#endif
        }
#endif
}

static void
emit_3dstate_vf_topology(struct bcmv_pipeline *pipeline)
{
#if 0 /* XXX */
        bcmv_batch_emit(&pipeline->batch, V3DX(3DSTATE_VF_TOPOLOGY), vft) {
                vft.PrimitiveTopologyType = pipeline->topology;
        }
#endif
}
#endif

#if 0 /* XXX */
static void
compute_kill_pixel(struct bcmv_pipeline *pipeline,
                   const VkPipelineMultisampleStateCreateInfo *ms_info,
                   const struct bcmv_subpass *subpass)
{
        if (!bcmv_pipeline_has_stage(pipeline, MESA_SHADER_FRAGMENT)) {
                pipeline->kill_pixel = false;
                return;
        }

        const struct brw_wm_prog_data *wm_prog_data = get_wm_prog_data(pipeline);

        /* This computes the KillPixel portion of the computation for whether or
         * not we want to enable the PMA fix on gen8 or gen9.  It's given by this
         * chunk of the giant formula:
         *
         *    (3DSTATE_PS_EXTRA::PixelShaderKillsPixels ||
         *     3DSTATE_PS_EXTRA::oMask Present to RenderTarget ||
         *     3DSTATE_PS_BLEND::AlphaToCoverageEnable ||
         *     3DSTATE_PS_BLEND::AlphaTestEnable ||
         *     3DSTATE_WM_CHROMAKEY::ChromaKeyKillEnable)
         *
         * 3DSTATE_WM_CHROMAKEY::ChromaKeyKillEnable is always false and so is
         * 3DSTATE_PS_BLEND::AlphaTestEnable since Vulkan doesn't have a concept
         * of an alpha test.
         */
        pipeline->kill_pixel =
                subpass->has_ds_self_dep || wm_prog_data->uses_kill ||
                wm_prog_data->uses_omask ||
                (ms_info && ms_info->alphaToCoverageEnable);
}
#endif

static VkResult
v3dx(graphics_pipeline_create)(
        VkDevice                                    _device,
        struct bcmv_pipeline_cache *                 cache,
        const VkGraphicsPipelineCreateInfo*         pCreateInfo,
        const VkAllocationCallbacks*                pAllocator,
        VkPipeline*                                 pPipeline)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        BCMV_FROM_HANDLE(bcmv_render_pass, pass, pCreateInfo->renderPass);
        struct bcmv_subpass *subpass = &pass->subpasses[pCreateInfo->subpass];
        struct bcmv_pipeline *pipeline;
        VkResult result;

        assert(pCreateInfo->sType == VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO);

        pipeline = vk_alloc2(&device->alloc, pAllocator, sizeof(*pipeline), 8,
                             VK_SYSTEM_ALLOCATION_SCOPE_OBJECT);
        if (pipeline == NULL)
                return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);

        result = bcmv_pipeline_init(pipeline, device, cache,
                                    pCreateInfo, pAllocator);
        if (result != VK_SUCCESS) {
                vk_free2(&device->alloc, pAllocator, pipeline);
                return result;
        }

        assert(pCreateInfo->pVertexInputState);
        emit_vertex_input(pipeline, pCreateInfo->pVertexInputState);
        assert(pCreateInfo->pRasterizationState);
        emit_rs_state(pipeline, pCreateInfo->pRasterizationState,
                      pCreateInfo->pMultisampleState, pass, subpass);
        emit_ms_state(pipeline, pCreateInfo->pMultisampleState);
        emit_ds_state(pipeline, pCreateInfo->pDepthStencilState, pass, subpass);
        emit_config_bits(pipeline,
                         pCreateInfo->pRasterizationState,
                         pCreateInfo->pDepthStencilState,
                         pCreateInfo->pColorBlendState);
        emit_cb_state(pipeline, pCreateInfo->pColorBlendState,
                      pCreateInfo->pMultisampleState);
        // XXX compute_kill_pixel(pipeline, pCreateInfo->pMultisampleState, subpass);

        emit_urb_setup(pipeline);

        emit_3dstate_clip(pipeline, pCreateInfo->pViewportState,
                          pCreateInfo->pRasterizationState);
        emit_3dstate_streamout(pipeline, pCreateInfo->pRasterizationState);

#if 0
        /* From gen7_vs_state.c */

        /**
         * From Graphics BSpec: 3D-Media-GPGPU Engine > 3D Pipeline Stages >
         * Geometry > Geometry Shader > State:
         *
         *     "Note: Because of corruption in IVB:GT2, software needs to flush the
         *     whole fixed function pipeline when the GS enable changes value in
         *     the 3DSTATE_GS."
         *
         * The hardware architects have clarified that in this context "flush the
         * whole fixed function pipeline" means to emit a PIPE_CONTROL with the "CS
         * Stall" bit set.
         */
        if (!device->info.is_haswell && !device->info.is_baytrail)
                gen7_emit_vs_workaround_flush(brw);
#endif

        emit_3dstate_vs(pipeline);
        emit_3dstate_sbe(pipeline);
        emit_3dstate_wm(pipeline, subpass, pCreateInfo->pMultisampleState);
        emit_3dstate_ps(pipeline, pCreateInfo->pColorBlendState);
#if GEN_GEN >= 8
        emit_3dstate_ps_extra(pipeline, subpass);
        emit_3dstate_vf_topology(pipeline);
#endif

        *pPipeline = bcmv_pipeline_to_handle(pipeline);

        return pipeline->batch.status;
}

static VkResult
compute_pipeline_create(
        VkDevice                                    _device,
        struct bcmv_pipeline_cache *                 cache,
        const VkComputePipelineCreateInfo*          pCreateInfo,
        const VkAllocationCallbacks*                pAllocator,
        VkPipeline*                                 pPipeline)
{
#if 0 /* XXX */
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        const struct bcmv_physical_device *physical_device =
                &device->instance->physicalDevice;
        const struct v3d_device_info *devinfo = &physical_device->info;
        struct bcmv_pipeline *pipeline;
        VkResult result;

        assert(pCreateInfo->sType == VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO);

        pipeline = vk_alloc2(&device->alloc, pAllocator, sizeof(*pipeline), 8,
                             VK_SYSTEM_ALLOCATION_SCOPE_OBJECT);
        if (pipeline == NULL)
                return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);

        pipeline->device = device;
        pipeline->layout = bcmv_pipeline_layout_from_handle(pCreateInfo->layout);

        pipeline->blend_state.map = NULL;

        result = bcmv_reloc_list_init(&pipeline->batch_relocs,
                                      pAllocator ? pAllocator : &device->alloc);
        if (result != VK_SUCCESS) {
                vk_free2(&device->alloc, pAllocator, pipeline);
                return result;
        }
        pipeline->batch.bcl.next = pipeline->batch_data;
        pipeline->batch.bcl.base = pipeline->batch_data;
        pipeline->batch.bcl.size = sizeof(pipeline->batch_data);
        pipeline->batch.relocs = &pipeline->batch_relocs;
        pipeline->batch.status = VK_SUCCESS;

        /* When we free the pipeline, we detect stages based on the NULL status
         * of various prog_data pointers.  Make them NULL by default.
         */
        memset(pipeline->shaders, 0, sizeof(pipeline->shaders));

        pipeline->active_stages = 0;

        pipeline->needs_data_cache = false;

        assert(pCreateInfo->stage.stage == VK_SHADER_STAGE_COMPUTE_BIT);
        BCMV_FROM_HANDLE(bcmv_shader_module, module,  pCreateInfo->stage.module);
        result = bcmv_pipeline_compile_cs(pipeline, cache, pCreateInfo, module,
                                          pCreateInfo->stage.pName,
                                          pCreateInfo->stage.pSpecializationInfo);
        if (result != VK_SUCCESS) {
                vk_free2(&device->alloc, pAllocator, pipeline);
                return result;
        }

        const struct brw_cs_prog_data *cs_prog_data = get_cs_prog_data(pipeline);

        bcmv_pipeline_setup_l3_config(pipeline, cs_prog_data->base.total_shared > 0);

        uint32_t group_size = cs_prog_data->local_size[0] *
                cs_prog_data->local_size[1] * cs_prog_data->local_size[2];
        uint32_t remainder = group_size & (cs_prog_data->simd_size - 1);

        if (remainder > 0)
                pipeline->cs_right_mask = ~0u >> (32 - remainder);
        else
                pipeline->cs_right_mask = ~0u >> (32 - cs_prog_data->simd_size);

        const uint32_t vfe_curbe_allocation =
                ALIGN(cs_prog_data->push.per_thread.regs * cs_prog_data->threads +
                      cs_prog_data->push.cross_thread.regs, 2);

        const uint32_t subslices = MAX2(physical_device->subslice_total, 1);

        const struct bcmv_shader_bin *cs_bin =
                pipeline->shaders[MESA_SHADER_COMPUTE];

        bcmv_batch_emit(&pipeline->batch, V3DX(MEDIA_VFE_STATE), vfe) {
                vfe.StackSize              = 0;
                vfe.MaximumNumberofThreads =
                        devinfo->max_cs_threads * subslices - 1;
                vfe.NumberofURBEntries     = GEN_GEN <= 7 ? 0 : 2;
                vfe.ResetGatewayTimer      = true;
                vfe.URBEntryAllocationSize = GEN_GEN <= 7 ? 0 : 2;
                vfe.CURBEAllocationSize    = vfe_curbe_allocation;

                vfe.PerThreadScratchSpace = get_scratch_space(cs_bin);
                vfe.ScratchSpaceBasePointer =
                        get_scratch_address(pipeline, MESA_SHADER_COMPUTE, cs_bin);
        }

        struct V3DX(INTERFACE_DESCRIPTOR_DATA) desc = {
                .KernelStartPointer     = cs_bin->kernel.offset,

                .SamplerCount           = get_sampler_count(cs_bin),
                .BindingTableEntryCount = get_binding_table_entry_count(cs_bin),
                .BarrierEnable          = cs_prog_data->uses_barrier,
                .SharedLocalMemorySize  =
                encode_slm_size(GEN_GEN, cs_prog_data->base.total_shared),

#if !GEN_IS_HASWELL
                .ConstantURBEntryReadOffset = 0,
#endif
                .ConstantURBEntryReadLength = cs_prog_data->push.per_thread.regs,
#if GEN_GEN >= 8 || GEN_IS_HASWELL
                .CrossThreadConstantDataReadLength =
                cs_prog_data->push.cross_thread.regs,
#endif

                .NumberofThreadsinGPGPUThreadGroup = cs_prog_data->threads,
        };
        V3DX(INTERFACE_DESCRIPTOR_DATA_pack)(NULL,
                                             pipeline->interface_descriptor_data,
                                             &desc);

        *pPipeline = bcmv_pipeline_to_handle(pipeline);

        return pipeline->batch.status;
#endif
        return VK_SUCCESS;
}

VkResult v3dx(CreateGraphicsPipelines)(
        VkDevice                                    _device,
        VkPipelineCache                             pipelineCache,
        uint32_t                                    count,
        const VkGraphicsPipelineCreateInfo*         pCreateInfos,
        const VkAllocationCallbacks*                pAllocator,
        VkPipeline*                                 pPipelines)
{
        BCMV_FROM_HANDLE(bcmv_pipeline_cache, pipeline_cache, pipelineCache);

        VkResult result = VK_SUCCESS;

        unsigned i;
        for (i = 0; i < count; i++) {
                result = v3dx(graphics_pipeline_create)(_device,
                                                        pipeline_cache,
                                                        &pCreateInfos[i],
                                                        pAllocator, &pPipelines[i]);

                /* Bail out on the first error as it is not obvious what error
                 * should be reported upon 2 different failures.
                 */
                if (result != VK_SUCCESS)
                        break;
        }

        for (; i < count; i++)
                pPipelines[i] = VK_NULL_HANDLE;

        return result;
}

VkResult v3dx(CreateComputePipelines)(
        VkDevice                                    _device,
        VkPipelineCache                             pipelineCache,
        uint32_t                                    count,
        const VkComputePipelineCreateInfo*          pCreateInfos,
        const VkAllocationCallbacks*                pAllocator,
        VkPipeline*                                 pPipelines)
{
        BCMV_FROM_HANDLE(bcmv_pipeline_cache, pipeline_cache, pipelineCache);

        VkResult result = VK_SUCCESS;

        unsigned i;
        for (i = 0; i < count; i++) {
                result = compute_pipeline_create(_device, pipeline_cache,
                                                 &pCreateInfos[i],
                                                 pAllocator, &pPipelines[i]);

                /* Bail out on the first error as it is not obvious what error should be
                 * report upon 2 different failures. */
                if (result != VK_SUCCESS)
                        break;
        }

        for (; i < count; i++)
                pPipelines[i] = VK_NULL_HANDLE;

        return result;
}
