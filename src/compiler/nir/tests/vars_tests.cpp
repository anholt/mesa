/*
 * Copyright © 2018 Intel Corporation
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
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <gtest/gtest.h>

#include "nir.h"
#include "nir_builder.h"

namespace {

class nir_vars_test : public ::testing::Test {
protected:
   nir_vars_test();
   ~nir_vars_test();

   nir_variable *create_int(nir_variable_mode mode, const char *name) {
      if (mode == nir_var_local)
         return nir_local_variable_create(b->impl, glsl_int_type(), name);
      return nir_variable_create(b->shader, mode, glsl_int_type(), name);
   }

   nir_variable *create_ivec2(nir_variable_mode mode, const char *name) {
      const glsl_type *var_type = glsl_vector_type(GLSL_TYPE_INT, 2);
      if (mode == nir_var_local)
         return nir_local_variable_create(b->impl, var_type, name);
      return nir_variable_create(b->shader, mode, var_type, name);
   }

   nir_variable **create_many_int(nir_variable_mode mode, const char *prefix, unsigned count) {
      nir_variable **result = (nir_variable **)linear_alloc_child(lin_ctx, sizeof(nir_variable *) * count);
      for (unsigned i = 0; i < count; i++)
         result[i] = create_int(mode, linear_asprintf(lin_ctx, "%s%u", prefix, i));
      return result;
   }

   nir_variable **create_many_ivec2(nir_variable_mode mode, const char *prefix, unsigned count) {
      nir_variable **result = (nir_variable **)linear_alloc_child(lin_ctx, sizeof(nir_variable *) * count);
      for (unsigned i = 0; i < count; i++)
         result[i] = create_ivec2(mode, linear_asprintf(lin_ctx, "%s%u", prefix, i));
      return result;
   }

   unsigned count_intrinsics(nir_intrinsic_op intrinsic);

   nir_intrinsic_instr *find_next_intrinsic(nir_intrinsic_op intrinsic,
                                            nir_intrinsic_instr *after);

   void *mem_ctx;
   void *lin_ctx;

   nir_builder *b;
};

nir_vars_test::nir_vars_test()
{
   mem_ctx = ralloc_context(NULL);
   lin_ctx = linear_alloc_parent(mem_ctx, 0);
   static const nir_shader_compiler_options options = { };
   b = rzalloc(mem_ctx, nir_builder);
   nir_builder_init_simple_shader(b, mem_ctx, MESA_SHADER_FRAGMENT, &options);
}

nir_vars_test::~nir_vars_test()
{
   if (HasFailure()) {
      printf("\nShader from the failed test:\n\n");
      nir_print_shader(b->shader, stdout);
   }

   ralloc_free(mem_ctx);
}

unsigned
nir_vars_test::count_intrinsics(nir_intrinsic_op intrinsic)
{
   unsigned count = 0;
   nir_foreach_block(block, b->impl) {
      nir_foreach_instr(instr, block) {
         if (instr->type != nir_instr_type_intrinsic)
            continue;
         nir_intrinsic_instr *intrin = nir_instr_as_intrinsic(instr);
         if (intrin->intrinsic == intrinsic)
            count++;
      }
   }
   return count;
}

nir_intrinsic_instr *
nir_vars_test::find_next_intrinsic(nir_intrinsic_op intrinsic,
                                   nir_intrinsic_instr *after)
{
   bool seen = after == NULL;
   nir_foreach_block(block, b->impl) {
      /* Skip blocks before the 'after' instruction. */
      if (!seen && block != after->instr.block)
         continue;
      nir_foreach_instr(instr, block) {
         if (instr->type != nir_instr_type_intrinsic)
            continue;
         nir_intrinsic_instr *intrin = nir_instr_as_intrinsic(instr);
         if (!seen) {
            seen = (after == intrin);
            continue;
         }
         if (intrin->intrinsic == intrinsic)
            return intrin;
      }
   }
   return NULL;
}

/* Allow grouping the tests while still sharing the helpers. */
class nir_copy_prop_vars_test : public nir_vars_test {};

} // namespace

TEST_F(nir_copy_prop_vars_test, simple_copies)
{
   nir_variable *in   = create_int(nir_var_shader_in,  "in");
   nir_variable *temp = create_int(nir_var_local,      "temp");
   nir_variable *out  = create_int(nir_var_shader_out, "out");

   nir_copy_var(b, temp, in);
   nir_copy_var(b, out, temp);

   nir_validate_shader(b->shader);

   bool progress = nir_opt_copy_prop_vars(b->shader);
   EXPECT_TRUE(progress);

   nir_validate_shader(b->shader);

   nir_intrinsic_instr *copy = NULL;
   copy = find_next_intrinsic(nir_intrinsic_copy_deref, copy);
   ASSERT_TRUE(copy->src[1].is_ssa);
   nir_ssa_def *first_src = copy->src[1].ssa;

   copy = find_next_intrinsic(nir_intrinsic_copy_deref, copy);
   ASSERT_TRUE(copy->src[1].is_ssa);
   EXPECT_EQ(copy->src[1].ssa, first_src);
}

TEST_F(nir_copy_prop_vars_test, simple_store_load)
{
   nir_variable **v = create_many_ivec2(nir_var_local, "v", 2);
   unsigned mask = 1 | 2;

   nir_ssa_def *stored_value = nir_imm_ivec2(b, 10, 20);
   nir_store_var(b, v[0], stored_value, mask);

   nir_ssa_def *read_value = nir_load_var(b, v[0]);
   nir_store_var(b, v[1], read_value, mask);

   nir_validate_shader(b->shader);

   bool progress = nir_opt_copy_prop_vars(b->shader);
   EXPECT_TRUE(progress);

   nir_validate_shader(b->shader);

   ASSERT_EQ(count_intrinsics(nir_intrinsic_store_deref), 2);

   nir_intrinsic_instr *store = NULL;
   for (int i = 0; i < 2; i++) {
      store = find_next_intrinsic(nir_intrinsic_store_deref, store);
      ASSERT_TRUE(store->src[1].is_ssa);
      EXPECT_EQ(store->src[1].ssa, stored_value);
   }
}
