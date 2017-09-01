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

#include <assert.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include "util/mesa-sha1.h"
#include "bcmv_private.h"
#include "broadcom/cle/v3d_packet_v33_pack.h"
#include "broadcom/compiler/v3d_compiler.h"
#include "bcmv_nir.h"
#include "spirv/nir_spirv.h"

// Shader functions

VkResult bcmv_CreateShaderModule(VkDevice _device,
                                 const VkShaderModuleCreateInfo *pCreateInfo,
                                 const VkAllocationCallbacks *pAllocator,
                                 VkShaderModule *pShaderModule)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        struct bcmv_shader_module *module;

        assert(pCreateInfo->sType == VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO);
        assert(pCreateInfo->flags == 0);

        module = vk_alloc2(&device->alloc, pAllocator,
                           sizeof(*module) + pCreateInfo->codeSize, 8,
                           VK_SYSTEM_ALLOCATION_SCOPE_OBJECT);
        if (module == NULL)
                return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);

        module->size = pCreateInfo->codeSize;
        memcpy(module->data, pCreateInfo->pCode, module->size);

        _mesa_sha1_compute(module->data, module->size, module->sha1);

        *pShaderModule = bcmv_shader_module_to_handle(module);

        return VK_SUCCESS;
}

void bcmv_DestroyShaderModule(VkDevice _device,
                              VkShaderModule _module,
                              const VkAllocationCallbacks *pAllocator)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        BCMV_FROM_HANDLE(bcmv_shader_module, module, _module);

        if (!module)
                return;

        vk_free2(&device->alloc, pAllocator, module);
}

#define SPIR_V_MAGIC_NUMBER 0x07230203

/* Eventually, this will become part of bcmv_CreateShader, but it was written
 * before nir_clone() existed.
 */
static nir_shader *
bcmv_shader_compile_to_nir(struct bcmv_pipeline *pipeline,
                           struct bcmv_shader_module *module,
                           const char *entrypoint_name,
                           gl_shader_stage stage,
                           const VkSpecializationInfo *spec_info)
{
        const nir_shader_compiler_options *nir_options = &v3d_nir_options;

        uint32_t *spirv = (uint32_t *) module->data;
        assert(spirv[0] == SPIR_V_MAGIC_NUMBER);
        assert(module->size % 4 == 0);

        uint32_t num_spec_entries = 0;
        struct nir_spirv_specialization *spec_entries = NULL;
        if (spec_info && spec_info->mapEntryCount > 0) {
                num_spec_entries = spec_info->mapEntryCount;
                spec_entries = malloc(num_spec_entries * sizeof(*spec_entries));
                for (uint32_t i = 0; i < num_spec_entries; i++) {
                        VkSpecializationMapEntry entry = spec_info->pMapEntries[i];
                        const void *data = spec_info->pData + entry.offset;
                        assert(data + entry.size <= spec_info->pData + spec_info->dataSize);

                        spec_entries[i].id = spec_info->pMapEntries[i].constantID;
                        if (spec_info->dataSize == 8)
                                spec_entries[i].data64 = *(const uint64_t *)data;
                        else
                                spec_entries[i].data32 = *(const uint32_t *)data;
                }
        }

        const struct nir_spirv_supported_extensions supported_ext = {
                .float64 = false,
                .int64 = false,
                .tessellation = false,
                .draw_parameters = false,
                .image_write_without_format = false,
                .multiview = false,
                .variable_pointers = false,
        };

        nir_function *entry_point =
                spirv_to_nir(spirv, module->size / 4,
                             spec_entries, num_spec_entries,
                             stage, entrypoint_name, &supported_ext, nir_options);
        nir_shader *nir = entry_point->shader;
        assert(nir->stage == stage);
        nir_validate_shader(nir);

        free(spec_entries);

        /* We have to lower away local constant initializers right before we
         * inline functions.  That way they get properly initialized at the top
         * of the function and not at the top of its caller.
         */
        NIR_PASS_V(nir, nir_lower_constant_initializers, nir_var_local);
        NIR_PASS_V(nir, nir_lower_returns);
        NIR_PASS_V(nir, nir_inline_functions);

        /* Pick off the single entrypoint that we want */
        foreach_list_typed_safe(nir_function, func, node, &nir->functions) {
                if (func != entry_point)
                        exec_node_remove(&func->node);
        }
        assert(exec_list_length(&nir->functions) == 1);
        entry_point->name = ralloc_strdup(entry_point, "main");

        NIR_PASS_V(nir, nir_remove_dead_variables,
                   nir_var_shader_in | nir_var_shader_out | nir_var_system_value);

        if (stage == MESA_SHADER_FRAGMENT)
                NIR_PASS_V(nir, nir_lower_wpos_center, pipeline->sample_shading_enable);

        /* Now that we've deleted all but the main function, we can go ahead and
         * lower the rest of the constant initializers.
         */
        NIR_PASS_V(nir, nir_lower_constant_initializers, ~0);
        NIR_PASS_V(nir, nir_propagate_invariant);
        NIR_PASS_V(nir, nir_lower_io_to_temporaries,
                   entry_point->impl, true, false);
        NIR_PASS_V(nir, nir_lower_system_values);

        /* Vulkan uses the separate-shader linking model */
        nir->info.separate_shader = true;

        // XXX nir = brw_preprocess_nir(compiler, nir);

        NIR_PASS_V(nir, nir_lower_clip_cull_distance_arrays);

        if (stage == MESA_SHADER_FRAGMENT)
                NIR_PASS_V(nir, bcmv_nir_lower_input_attachments);

        return nir;
}

void bcmv_DestroyPipeline(VkDevice _device,
                          VkPipeline _pipeline,
                          const VkAllocationCallbacks *pAllocator)
{
        BCMV_FROM_HANDLE(bcmv_device, device, _device);
        BCMV_FROM_HANDLE(bcmv_pipeline, pipeline, _pipeline);

        if (!pipeline)
                return;

        bcmv_reloc_list_finish(&pipeline->batch_relocs,
                               pAllocator ? pAllocator : &device->alloc);
        if (pipeline->blend_state.map)
                bcmv_state_pool_free(&device->dynamic_state_pool, pipeline->blend_state);

        for (unsigned s = 0; s < MESA_SHADER_STAGES; s++) {
                if (pipeline->shaders[s])
                        bcmv_shader_bin_unref(device, pipeline->shaders[s]);
        }

        vk_free2(&device->alloc, pAllocator, pipeline);
}

static const uint32_t vk_to_v3d_primitive_type[] = {
        [VK_PRIMITIVE_TOPOLOGY_POINT_LIST]     = V3D_PRIM_POINTS,
        [VK_PRIMITIVE_TOPOLOGY_LINE_LIST]      = V3D_PRIM_LINES,
        [VK_PRIMITIVE_TOPOLOGY_LINE_STRIP]     = V3D_PRIM_LINE_STRIP,
        [VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST]  = V3D_PRIM_TRIANGLES,
        [VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP] = V3D_PRIM_TRIANGLE_STRIP,
        [VK_PRIMITIVE_TOPOLOGY_TRIANGLE_FAN]   = V3D_PRIM_TRIANGLE_FAN,
};

static void
populate_sampler_key(const struct v3d_device_info *devinfo,
                     struct v3d_key *key)
{
        /* Assume color sampler, no swizzling. (Need 32-bit return fix) */
        for (int i = 0; i < V3D_MAX_TEXTURE_SAMPLERS; i++) {
                key->tex[i].return_size = 16;
                key->tex[i].return_channels = 4;
                for (int j = 0; j < 4; j++)
                        key->tex[i].swizzle[j] = j;
        }
}

static void
populate_vs_key(const struct v3d_device_info *devinfo,
                struct v3d_vs_key *key)
{
        memset(key, 0, sizeof(*key));

        populate_sampler_key(devinfo, &key->base);

        /* XXX: Handle vertex input work-arounds */
}

static void
populate_fs_key(const struct bcmv_pipeline *pipeline,
                const VkGraphicsPipelineCreateInfo *info,
                struct v3d_fs_key *key)
{
        const struct v3d_device_info *devinfo = &pipeline->device->info;

        memset(key, 0, sizeof(*key));

        populate_sampler_key(devinfo, &key->base);

        /* TODO: we could set this to 0 based on the information in
         * nir_shader, but this function is called before spirv_to_nir.
         */
        /* XXX
        const struct brw_vue_map *vue_map =
                &bcmv_pipeline_get_last_vue_prog_data(pipeline)->vue_map;
        key->input_slots_valid = vue_map->slots_valid;
        */

        /* XXX Vulkan doesn't appear to specify */
        key->clamp_color = false;

        // XXX key->nr_color_regions = pipeline->subpass->color_count;

        /* XXX
        key->replicate_alpha = key->nr_color_regions > 1 &&
                info->pMultisampleState &&
                info->pMultisampleState->alphaToCoverageEnable;
        */

        if (info->pMultisampleState) {
                /* We should probably pull this out of the shader, but it's
                 * fairly harmless to compute it and then let dead-code take
                 * care of it.
                 */
                if (info->pMultisampleState->rasterizationSamples > 1) {
                        /* XXX
                        key->persample_interp =
                                (info->pMultisampleState->minSampleShading *
                                 info->pMultisampleState->rasterizationSamples) > 1;
                        */
                        key->msaa = true;
                }

                /* XXX
                key->frag_coord_adds_sample_pos =
                        info->pMultisampleState->sampleShadingEnable;
                */
        }
}

/* XXX
static void
populate_cs_key(const struct v3d_device_info *devinfo,
                struct v3d_cs_key *key)
{
        memset(key, 0, sizeof(*key));

        populate_sampler_key(devinfo, &key->base);
}
*/

static void
bcmv_pipeline_hash_shader(struct bcmv_pipeline *pipeline,
                          struct bcmv_shader_module *module,
                          const char *entrypoint,
                          gl_shader_stage stage,
                          const VkSpecializationInfo *spec_info,
                          const void *key, size_t key_size,
                          unsigned char *sha1_out)
{
        struct mesa_sha1 ctx;

        _mesa_sha1_init(&ctx);
        if (stage != MESA_SHADER_COMPUTE) {
                _mesa_sha1_update(&ctx, &pipeline->subpass->view_mask,
                                  sizeof(pipeline->subpass->view_mask));
        }
        if (pipeline->layout) {
                _mesa_sha1_update(&ctx, pipeline->layout->sha1,
                                  sizeof(pipeline->layout->sha1));
        }
        _mesa_sha1_update(&ctx, module->sha1, sizeof(module->sha1));
        _mesa_sha1_update(&ctx, entrypoint, strlen(entrypoint));
        _mesa_sha1_update(&ctx, &stage, sizeof(stage));
        if (spec_info) {
                _mesa_sha1_update(&ctx, spec_info->pMapEntries,
                                  spec_info->mapEntryCount * sizeof(*spec_info->pMapEntries));
                _mesa_sha1_update(&ctx, spec_info->pData, spec_info->dataSize);
        }
        _mesa_sha1_update(&ctx, key, key_size);
        _mesa_sha1_final(&ctx, sha1_out);
}

static nir_shader *
bcmv_pipeline_compile(struct bcmv_pipeline *pipeline,
                      struct bcmv_shader_module *module,
                      const char *entrypoint,
                      gl_shader_stage stage,
                      const VkSpecializationInfo *spec_info,
                      struct v3d_prog_data *prog_data,
                      struct bcmv_pipeline_bind_map *map)
{
        nir_shader *nir = bcmv_shader_compile_to_nir(pipeline,
                                                     module, entrypoint, stage,
                                                     spec_info);
        if (nir == NULL)
                return NULL;

        // XXX NIR_PASS_V(nir, bcmv_nir_lower_push_constants);

        /* XXX
        if (stage == MESA_SHADER_COMPUTE) {
                NIR_PASS_V(nir, brw_nir_lower_cs_shared);
                prog_data->total_shared = nir->num_shared;
        }
        */

        nir_shader_gather_info(nir, nir_shader_get_entrypoint(nir));

        /* XXX
        if (nir->info.num_images > 0) {
                prog_data->nr_params += nir->info.num_images * BRW_IMAGE_PARAM_SIZE;
                pipeline->needs_data_cache = true;
        }
        */

        /* XXX
        if (nir->info.num_ssbos > 0)
                pipeline->needs_data_cache = true;
        */

#if 0 /* XXX */
        if (prog_data->nr_params > 0) {
                /* XXX: I think we're leaking this */
                prog_data->param = (const union gl_constant_value **)
                        malloc(prog_data->nr_params * sizeof(union gl_constant_value *));

                /* We now set the param values to be offsets into a
                 * bcmv_push_constant_data structure.  Since the compiler doesn't
                 * actually dereference any of the gl_constant_value pointers in the
                 * params array, it doesn't really matter what we put here.
                 */
                struct bcmv_push_constants *null_data = NULL;
                if (nir->num_uniforms > 0) {
                        /* Fill out the push constants section of the param array */
                        for (unsigned i = 0; i < MAX_PUSH_CONSTANTS_SIZE / sizeof(float); i++)
                                prog_data->param[i] = (const union gl_constant_value *)
                                        &null_data->client_data[i * sizeof(float)];
                }
        }

        /* Apply the actual pipeline layout to UBOs, SSBOs, and textures */
        if (pipeline->layout)
                bcmv_nir_apply_pipeline_layout(pipeline, nir, prog_data, map);

        /* nir_lower_io will only handle the push constants; we need to set this
         * to the full number of possible uniforms.
         */
        nir->num_uniforms = prog_data->nr_params * 4;
#endif

        return nir;
}

static void
bcmv_fill_binding_table(struct v3d_prog_data *prog_data, unsigned bias)
{
#if 0 /* XXX */
        prog_data->binding_table.size_bytes = 0;
        prog_data->binding_table.texture_start = bias;
        prog_data->binding_table.gather_texture_start = bias;
        prog_data->binding_table.ubo_start = bias;
        prog_data->binding_table.ssbo_start = bias;
        prog_data->binding_table.image_start = bias;
#endif
}

static struct bcmv_shader_bin *
bcmv_pipeline_upload_kernel(struct bcmv_pipeline *pipeline,
                            struct bcmv_pipeline_cache *cache,
                            const void *key_data, uint32_t key_size,
                            const void *kernel_data, uint32_t kernel_size,
                            const struct v3d_prog_data *prog_data,
                            uint32_t prog_data_size,
                            const struct bcmv_pipeline_bind_map *bind_map)
{
        if (cache) {
                return bcmv_pipeline_cache_upload_kernel(cache, key_data, key_size,
                                                         kernel_data, kernel_size,
                                                         prog_data,
                                                         prog_data_size,
                                                         prog_data->uniforms.contents,
                                                         prog_data->uniforms.data,
                                                         bind_map);
        } else {
                return bcmv_shader_bin_create(pipeline->device, key_data, key_size,
                                              kernel_data, kernel_size,
                                              prog_data,
                                              prog_data_size,
                                              prog_data->uniforms.contents,
                                              prog_data->uniforms.data,
                                              bind_map);
        }
}


static void
bcmv_pipeline_add_compiled_stage(struct bcmv_pipeline *pipeline,
                                 gl_shader_stage stage,
                                 struct bcmv_shader_bin *shader)
{
        pipeline->shaders[stage] = shader;
        pipeline->active_stages |= mesa_to_vk_shader_stage(stage);
}

static VkResult
bcmv_pipeline_compile_vs(struct bcmv_pipeline *pipeline,
                         struct bcmv_pipeline_cache *cache,
                         const VkGraphicsPipelineCreateInfo *info,
                         struct bcmv_shader_module *module,
                         const char *entrypoint,
                         const VkSpecializationInfo *spec_info)
{
        const struct v3d_compiler *compiler =
                pipeline->device->instance->physicalDevice.compiler;
        struct bcmv_pipeline_bind_map map;
        struct v3d_vs_key key;
        struct bcmv_shader_bin *bin = NULL;
        unsigned char sha1[20];

        populate_vs_key(&pipeline->device->info, &key);

        if (cache) {
                bcmv_pipeline_hash_shader(pipeline, module, entrypoint,
                                          MESA_SHADER_VERTEX, spec_info,
                                          &key, sizeof(key), sha1);
                bin = bcmv_pipeline_cache_search(cache, sha1, 20);
        }

        if (bin == NULL) {
                struct v3d_vs_prog_data prog_data = {};
                struct bcmv_pipeline_binding surface_to_descriptor[256];
                struct bcmv_pipeline_binding sampler_to_descriptor[256];

                map = (struct bcmv_pipeline_bind_map) {
                        .surface_to_descriptor = surface_to_descriptor,
                        .sampler_to_descriptor = sampler_to_descriptor
                };

                nir_shader *nir = bcmv_pipeline_compile(pipeline, module, entrypoint,
                                                        MESA_SHADER_VERTEX, spec_info,
                                                        &prog_data.base, &map);
                if (nir == NULL)
                        return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);

                bcmv_fill_binding_table(&prog_data.base, 0);

                void *mem_ctx = ralloc_context(NULL);

                ralloc_steal(mem_ctx, nir);

                /* XXX
                brw_compute_vue_map(&pipeline->device->info,
                                    &prog_data.base.vue_map,
                                    nir->info.outputs_written,
                                    nir->info.separate_shader);
                */

                unsigned code_size;
                const uint64_t *shader_code =
                        v3d_compile_vs(compiler, &key, &prog_data, nir,
                                       0, 0, &code_size);
                if (shader_code == NULL) {
                        ralloc_free(mem_ctx);
                        return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);
                }

                bin = bcmv_pipeline_upload_kernel(pipeline, cache, sha1, 20,
                                                  shader_code, code_size,
                                                  &prog_data.base,
                                                  sizeof(prog_data),
                                                  &map);
                if (!bin) {
                        ralloc_free(mem_ctx);
                        return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);
                }

                ralloc_free(mem_ctx);
        }

        bcmv_pipeline_add_compiled_stage(pipeline, MESA_SHADER_VERTEX, bin);

        return VK_SUCCESS;
}

static VkResult
bcmv_pipeline_compile_fs(struct bcmv_pipeline *pipeline,
                         struct bcmv_pipeline_cache *cache,
                         const VkGraphicsPipelineCreateInfo *info,
                         struct bcmv_shader_module *module,
                         const char *entrypoint,
                         const VkSpecializationInfo *spec_info)
{
        const struct v3d_compiler *compiler =
                pipeline->device->instance->physicalDevice.compiler;
        struct bcmv_pipeline_bind_map map;
        struct v3d_fs_key key;
        struct bcmv_shader_bin *bin = NULL;
        unsigned char sha1[20];

        populate_fs_key(pipeline, info, &key);

        if (cache) {
                bcmv_pipeline_hash_shader(pipeline, module, entrypoint,
                                          MESA_SHADER_FRAGMENT, spec_info,
                                          &key, sizeof(key), sha1);
                bin = bcmv_pipeline_cache_search(cache, sha1, 20);
        }

        if (bin == NULL) {
                struct v3d_fs_prog_data prog_data = {};
                struct bcmv_pipeline_binding surface_to_descriptor[256];
                struct bcmv_pipeline_binding sampler_to_descriptor[256];

                map = (struct bcmv_pipeline_bind_map) {
                        .surface_to_descriptor = surface_to_descriptor + 8,
                        .sampler_to_descriptor = sampler_to_descriptor
                };

                nir_shader *nir = bcmv_pipeline_compile(pipeline, module, entrypoint,
                                                        MESA_SHADER_FRAGMENT, spec_info,
                                                        &prog_data.base, &map);
                if (nir == NULL)
                        return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);

                unsigned num_rts = 0;
                struct bcmv_pipeline_binding rt_bindings[8];
                // XXX nir_function_impl *impl = nir_shader_get_entrypoint(nir);
                nir_foreach_variable_safe(var, &nir->outputs) {
                        if (var->data.location < FRAG_RESULT_DATA0)
                                continue;

                        /* XXX */
#if 0
                        unsigned rt = var->data.location - FRAG_RESULT_DATA0;
                        if (rt >= key.nr_color_regions) {
                                /* Out-of-bounds, throw it away */
                                var->data.mode = nir_var_local;
                                exec_node_remove(&var->node);
                                exec_list_push_tail(&impl->locals, &var->node);
                                continue;
                        }
#endif

                        /* Give it a new, compacted, location */
                        var->data.location = FRAG_RESULT_DATA0 + num_rts;

                        unsigned array_len =
                                glsl_type_is_array(var->type) ? glsl_get_length(var->type) : 1;
                        assert(num_rts + array_len <= 8);
#if 0 /* XXX */
                        for (unsigned i = 0; i < array_len; i++) {
                                rt_bindings[num_rts + i] = (struct bcmv_pipeline_binding) {
                                        .set = BCMV_DESCRIPTOR_SET_COLOR_ATTACHMENTS,
                                        .binding = 0,
                                        .index = rt + i,
                                };
                        }

                        num_rts += array_len;
#endif
                }

                if (num_rts == 0) {
                        /* If we have no render targets, we need a null render target */
                        rt_bindings[0] = (struct bcmv_pipeline_binding) {
                                .set = BCMV_DESCRIPTOR_SET_COLOR_ATTACHMENTS,
                                .binding = 0,
                                .index = UINT32_MAX,
                        };
                        num_rts = 1;
                }

                assert(num_rts <= 8);
                map.surface_to_descriptor -= num_rts;
                map.surface_count += num_rts;
                assert(map.surface_count <= 256);
                memcpy(map.surface_to_descriptor, rt_bindings,
                       num_rts * sizeof(*rt_bindings));

                bcmv_fill_binding_table(&prog_data.base, num_rts);

                void *mem_ctx = ralloc_context(NULL);

                ralloc_steal(mem_ctx, nir);

                unsigned code_size;
                const uint64_t *shader_code =
                        v3d_compile_fs(compiler, &key, &prog_data, nir,
                                       0, 0, &code_size);
                if (shader_code == NULL) {
                        ralloc_free(mem_ctx);
                        return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);
                }

                bin = bcmv_pipeline_upload_kernel(pipeline, cache, sha1, 20,
                                                  shader_code, code_size,
                                                  &prog_data.base, sizeof(prog_data),
                                                  &map);
                if (!bin) {
                        ralloc_free(mem_ctx);
                        return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);
                }

                ralloc_free(mem_ctx);
        }

        bcmv_pipeline_add_compiled_stage(pipeline, MESA_SHADER_FRAGMENT, bin);

        return VK_SUCCESS;
}

#if 0 /* XXX */
VkResult
bcmv_pipeline_compile_cs(struct bcmv_pipeline *pipeline,
                         struct bcmv_pipeline_cache *cache,
                         const VkComputePipelineCreateInfo *info,
                         struct bcmv_shader_module *module,
                         const char *entrypoint,
                         const VkSpecializationInfo *spec_info)
{
        const struct v3d_compiler *compiler =
                pipeline->device->instance->physicalDevice.compiler;
        struct bcmv_pipeline_bind_map map;
        struct v3d_cs_key key;
        struct bcmv_shader_bin *bin = NULL;
        unsigned char sha1[20];

        populate_cs_key(&pipeline->device->info, &key);

        if (cache) {
                bcmv_pipeline_hash_shader(pipeline, module, entrypoint,
                                          MESA_SHADER_COMPUTE, spec_info,
                                          &key, sizeof(key), sha1);
                bin = bcmv_pipeline_cache_search(cache, sha1, 20);
        }

        if (bin == NULL) {
                struct brw_cs_prog_data prog_data = {};
                struct bcmv_pipeline_binding surface_to_descriptor[256];
                struct bcmv_pipeline_binding sampler_to_descriptor[256];

                map = (struct bcmv_pipeline_bind_map) {
                        .surface_to_descriptor = surface_to_descriptor,
                        .sampler_to_descriptor = sampler_to_descriptor
                };

                nir_shader *nir = bcmv_pipeline_compile(pipeline, module, entrypoint,
                                                        MESA_SHADER_COMPUTE, spec_info,
                                                        &prog_data.base, &map);
                if (nir == NULL)
                        return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);

                bcmv_fill_binding_table(&prog_data.base, 1);

                void *mem_ctx = ralloc_context(NULL);

                ralloc_steal(mem_ctx, nir);

                unsigned code_size;
                const unsigned *shader_code =
                        v3d_compile_cs(compiler, NULL, mem_ctx, &key,
                                       &prog_data, nir,
                                       -1, &code_size, NULL);
                if (shader_code == NULL) {
                        ralloc_free(mem_ctx);
                        return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);
                }

                bin = bcmv_pipeline_upload_kernel(pipeline, cache, sha1, 20,
                                                  shader_code, code_size,
                                                  &prog_data.base, sizeof(prog_data),
                                                  &map);
                if (!bin) {
                        ralloc_free(mem_ctx);
                        return vk_error(VK_ERROR_OUT_OF_HOST_MEMORY);
                }

                ralloc_free(mem_ctx);
        }

        bcmv_pipeline_add_compiled_stage(pipeline, MESA_SHADER_COMPUTE, bin);

        return VK_SUCCESS;
}
#endif

/**
 * Copy pipeline state not marked as dynamic.
 * Dynamic state is pipeline state which hasn't been provided at pipeline
 * creation time, but is dynamically provided afterwards using various
 * vkCmdSet* functions.
 *
 * The set of state considered "non_dynamic" is determined by the pieces of
 * state that have their corresponding VkDynamicState enums omitted from
 * VkPipelineDynamicStateCreateInfo::pDynamicStates.
 *
 * @param[out] pipeline    Destination non_dynamic state.
 * @param[in]  pCreateInfo Source of non_dynamic state to be copied.
 */
static void
copy_non_dynamic_state(struct bcmv_pipeline *pipeline,
                       const VkGraphicsPipelineCreateInfo *pCreateInfo)
{
        bcmv_cmd_dirty_mask_t states = BCMV_CMD_DIRTY_DYNAMIC_ALL;
        struct bcmv_subpass *subpass = pipeline->subpass;

        pipeline->dynamic_state = default_dynamic_state;

        if (pCreateInfo->pDynamicState) {
                /* Remove all of the states that are marked as dynamic */
                uint32_t count = pCreateInfo->pDynamicState->dynamicStateCount;
                for (uint32_t s = 0; s < count; s++)
                        states &= ~(1 << pCreateInfo->pDynamicState->pDynamicStates[s]);
        }

        struct bcmv_dynamic_state *dynamic = &pipeline->dynamic_state;

        /* Section 9.2 of the Vulkan 1.0.15 spec says:
         *
         *    pViewportState is [...] NULL if the pipeline
         *    has rasterization disabled.
         */
        if (!pCreateInfo->pRasterizationState->rasterizerDiscardEnable) {
                assert(pCreateInfo->pViewportState);

                dynamic->viewport.count = pCreateInfo->pViewportState->viewportCount;
                if (states & (1 << VK_DYNAMIC_STATE_VIEWPORT)) {
                        typed_memcpy(dynamic->viewport.viewports,
                                     pCreateInfo->pViewportState->pViewports,
                                     pCreateInfo->pViewportState->viewportCount);
                }

                dynamic->scissor.count = pCreateInfo->pViewportState->scissorCount;
                if (states & (1 << VK_DYNAMIC_STATE_SCISSOR)) {
                        typed_memcpy(dynamic->scissor.scissors,
                                     pCreateInfo->pViewportState->pScissors,
                                     pCreateInfo->pViewportState->scissorCount);
                }
        }

        if (states & (1 << VK_DYNAMIC_STATE_LINE_WIDTH)) {
                assert(pCreateInfo->pRasterizationState);
                dynamic->line_width = pCreateInfo->pRasterizationState->lineWidth;
        }

        if (states & (1 << VK_DYNAMIC_STATE_DEPTH_BIAS)) {
                assert(pCreateInfo->pRasterizationState);
                dynamic->depth_bias.bias =
                        pCreateInfo->pRasterizationState->depthBiasConstantFactor;
                dynamic->depth_bias.clamp =
                        pCreateInfo->pRasterizationState->depthBiasClamp;
                dynamic->depth_bias.slope =
                        pCreateInfo->pRasterizationState->depthBiasSlopeFactor;
        }

        /* Section 9.2 of the Vulkan 1.0.15 spec says:
         *
         *    pColorBlendState is [...] NULL if the pipeline has rasterization
         *    disabled or if the subpass of the render pass the pipeline is
         *    created against does not use any color attachments.
         */
        bool uses_color_att = false;
        for (unsigned i = 0; i < subpass->color_count; ++i) {
                if (subpass->color_attachments[i].attachment != VK_ATTACHMENT_UNUSED) {
                        uses_color_att = true;
                        break;
                }
        }

        if (uses_color_att &&
            !pCreateInfo->pRasterizationState->rasterizerDiscardEnable) {
                assert(pCreateInfo->pColorBlendState);

                if (states & (1 << VK_DYNAMIC_STATE_BLEND_CONSTANTS))
                        typed_memcpy(dynamic->blend_constants,
                                     pCreateInfo->pColorBlendState->blendConstants, 4);
        }

        /* If there is no depthstencil attachment, then don't read
         * pDepthStencilState. The Vulkan spec states that pDepthStencilState may
         * be NULL in this case. Even if pDepthStencilState is non-NULL, there is
         * no need to override the depthstencil defaults in
         * bcmv_pipeline::dynamic_state when there is no depthstencil attachment.
         *
         * Section 9.2 of the Vulkan 1.0.15 spec says:
         *
         *    pDepthStencilState is [...] NULL if the pipeline has rasterization
         *    disabled or if the subpass of the render pass the pipeline is created
         *    against does not use a depth/stencil attachment.
         */
        if (!pCreateInfo->pRasterizationState->rasterizerDiscardEnable &&
            subpass->depth_stencil_attachment.attachment != VK_ATTACHMENT_UNUSED) {
                assert(pCreateInfo->pDepthStencilState);

                if (states & (1 << VK_DYNAMIC_STATE_DEPTH_BOUNDS)) {
                        dynamic->depth_bounds.min =
                                pCreateInfo->pDepthStencilState->minDepthBounds;
                        dynamic->depth_bounds.max =
                                pCreateInfo->pDepthStencilState->maxDepthBounds;
                }

                if (states & (1 << VK_DYNAMIC_STATE_STENCIL_COMPARE_MASK)) {
                        dynamic->stencil_compare_mask.front =
                                pCreateInfo->pDepthStencilState->front.compareMask;
                        dynamic->stencil_compare_mask.back =
                                pCreateInfo->pDepthStencilState->back.compareMask;
                }

                if (states & (1 << VK_DYNAMIC_STATE_STENCIL_WRITE_MASK)) {
                        dynamic->stencil_write_mask.front =
                                pCreateInfo->pDepthStencilState->front.writeMask;
                        dynamic->stencil_write_mask.back =
                                pCreateInfo->pDepthStencilState->back.writeMask;
                }

                if (states & (1 << VK_DYNAMIC_STATE_STENCIL_REFERENCE)) {
                        dynamic->stencil_reference.front =
                                pCreateInfo->pDepthStencilState->front.reference;
                        dynamic->stencil_reference.back =
                                pCreateInfo->pDepthStencilState->back.reference;
                }
        }

        pipeline->dynamic_state_mask = states;
}

static void
bcmv_pipeline_validate_create_info(const VkGraphicsPipelineCreateInfo *info)
{
#ifdef DEBUG
        struct bcmv_render_pass *renderpass = NULL;
        struct bcmv_subpass *subpass = NULL;

        /* Assert that all required members of VkGraphicsPipelineCreateInfo are
         * present.  See the Vulkan 1.0.28 spec, Section 9.2 Graphics Pipelines.
         */
        assert(info->sType == VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO);

        renderpass = bcmv_render_pass_from_handle(info->renderPass);
        assert(renderpass);

        assert(info->subpass < renderpass->subpass_count);
        subpass = &renderpass->subpasses[info->subpass];

        assert(info->stageCount >= 1);
        assert(info->pVertexInputState);
        assert(info->pInputAssemblyState);
        assert(info->pRasterizationState);
        if (!info->pRasterizationState->rasterizerDiscardEnable) {
                assert(info->pViewportState);
                assert(info->pMultisampleState);

                if (subpass && subpass->depth_stencil_attachment.attachment != VK_ATTACHMENT_UNUSED)
                        assert(info->pDepthStencilState);

                if (subpass && subpass->color_count > 0)
                        assert(info->pColorBlendState);
        }

        for (uint32_t i = 0; i < info->stageCount; ++i) {
                switch (info->pStages[i].stage) {
                case VK_SHADER_STAGE_TESSELLATION_CONTROL_BIT:
                case VK_SHADER_STAGE_TESSELLATION_EVALUATION_BIT:
                        assert(info->pTessellationState);
                        break;
                default:
                        break;
                }
        }
#endif
}

VkResult
bcmv_pipeline_init(struct bcmv_pipeline *pipeline,
                   struct bcmv_device *device,
                   struct bcmv_pipeline_cache *cache,
                   const VkGraphicsPipelineCreateInfo *pCreateInfo,
                   const VkAllocationCallbacks *alloc)
{
        VkResult result;

        bcmv_pipeline_validate_create_info(pCreateInfo);

        if (alloc == NULL)
                alloc = &device->alloc;

        pipeline->device = device;

        BCMV_FROM_HANDLE(bcmv_render_pass, render_pass, pCreateInfo->renderPass);
        assert(pCreateInfo->subpass < render_pass->subpass_count);
        pipeline->subpass = &render_pass->subpasses[pCreateInfo->subpass];

        pipeline->layout = bcmv_pipeline_layout_from_handle(pCreateInfo->layout);

        result = bcmv_reloc_list_init(&pipeline->batch_relocs, alloc);
        if (result != VK_SUCCESS)
                return result;

        pipeline->batch.alloc = alloc;
        pipeline->batch.bcl.base = pipeline->batch_data;
        pipeline->batch.bcl.next = pipeline->batch_data;
        pipeline->batch.bcl.size = sizeof(pipeline->batch_data);

        pipeline->batch.relocs = &pipeline->batch_relocs;
        pipeline->batch.status = VK_SUCCESS;

        copy_non_dynamic_state(pipeline, pCreateInfo);
        pipeline->depth_clamp_enable = pCreateInfo->pRasterizationState &&
                pCreateInfo->pRasterizationState->depthClampEnable;

        pipeline->sample_shading_enable = pCreateInfo->pMultisampleState &&
                pCreateInfo->pMultisampleState->sampleShadingEnable;

        pipeline->needs_data_cache = false;

        /* When we free the pipeline, we detect stages based on the NULL status
         * of various prog_data pointers.  Make them NULL by default.
         */
        memset(pipeline->shaders, 0, sizeof(pipeline->shaders));

        pipeline->active_stages = 0;

        const VkPipelineShaderStageCreateInfo *pStages[MESA_SHADER_STAGES] = {};
        struct bcmv_shader_module *modules[MESA_SHADER_STAGES] = {};
        for (uint32_t i = 0; i < pCreateInfo->stageCount; i++) {
                gl_shader_stage stage = ffs(pCreateInfo->pStages[i].stage) - 1;
                pStages[stage] = &pCreateInfo->pStages[i];
                modules[stage] = bcmv_shader_module_from_handle(pStages[stage]->module);
        }

        if (modules[MESA_SHADER_VERTEX]) {
                result = bcmv_pipeline_compile_vs(pipeline, cache, pCreateInfo,
                                                  modules[MESA_SHADER_VERTEX],
                                                  pStages[MESA_SHADER_VERTEX]->pName,
                                                  pStages[MESA_SHADER_VERTEX]->pSpecializationInfo);
                if (result != VK_SUCCESS)
                        goto compile_fail;
        }

        if (modules[MESA_SHADER_FRAGMENT]) {
                result = bcmv_pipeline_compile_fs(pipeline, cache, pCreateInfo,
                                                  modules[MESA_SHADER_FRAGMENT],
                                                  pStages[MESA_SHADER_FRAGMENT]->pName,
                                                  pStages[MESA_SHADER_FRAGMENT]->pSpecializationInfo);
                if (result != VK_SUCCESS)
                        goto compile_fail;
        }

        assert(pipeline->active_stages & VK_SHADER_STAGE_VERTEX_BIT);

        const VkPipelineVertexInputStateCreateInfo *vi_info =
                pCreateInfo->pVertexInputState;

        const uint64_t inputs_read = 0; //XXX get_vs_prog_data(pipeline)->inputs_read;

        pipeline->vb_used = 0;
        for (uint32_t i = 0; i < vi_info->vertexAttributeDescriptionCount; i++) {
                const VkVertexInputAttributeDescription *desc =
                        &vi_info->pVertexAttributeDescriptions[i];

                if (inputs_read & (1ull << (VERT_ATTRIB_GENERIC0 + desc->location)))
                        pipeline->vb_used |= 1 << desc->binding;
        }

        for (uint32_t i = 0; i < vi_info->vertexBindingDescriptionCount; i++) {
                const VkVertexInputBindingDescription *desc =
                        &vi_info->pVertexBindingDescriptions[i];

                pipeline->binding_stride[desc->binding] = desc->stride;

                /* Step rate is programmed per vertex element (attribute), not
                 * binding. Set up a map of which bindings step per instance, for
                 * reference by vertex element setup. */
                switch (desc->inputRate) {
                default:
                case VK_VERTEX_INPUT_RATE_VERTEX:
                        pipeline->instancing_enable[desc->binding] = false;
                        break;
                case VK_VERTEX_INPUT_RATE_INSTANCE:
                        pipeline->instancing_enable[desc->binding] = true;
                        break;
                }
        }

        const VkPipelineInputAssemblyStateCreateInfo *ia_info =
                pCreateInfo->pInputAssemblyState;
        pipeline->primitive_restart = ia_info->primitiveRestartEnable;
        pipeline->topology = vk_to_v3d_primitive_type[ia_info->topology];

        return VK_SUCCESS;

 compile_fail:
        for (unsigned s = 0; s < MESA_SHADER_STAGES; s++) {
                if (pipeline->shaders[s])
                        bcmv_shader_bin_unref(device, pipeline->shaders[s]);
        }

        bcmv_reloc_list_finish(&pipeline->batch_relocs, alloc);

        return result;
}
