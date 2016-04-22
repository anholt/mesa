/*
 * Copyright © 2016 Broadcom Limited
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

#include "nir.h"
#include "nir_builder.h"

struct nir_lower_alu_newton_raphson_state {
   nir_builder b;
   nir_lower_alu_newton_raphson_flags flags;
   bool progress;
};

static void
lower_alu_instr_scalar(nir_alu_instr *instr,
                       struct nir_lower_alu_newton_raphson_state *state)
{
   nir_builder *b = &state->b;

   assert(instr->dest.dest.is_ssa);

   b->cursor = nir_before_instr(&instr->instr);
   b->exact = instr->exact;

   /* Note that we can't use the current instr's dest in our fixups, because
    * we're going to use nir_ssa_def_rewrite_uses() on it.
    */
   nir_ssa_def *replace = NULL, *approx, *arg0;

   switch (instr->op) {
   case nir_op_frcp:
      if (!(state->flags & nir_lower_alu_newton_raphson_frcp))
         break;

      arg0 = nir_fmov_alu(b, instr->src[0], instr->dest.dest.ssa.num_components);
      approx = nir_frcp(b, arg0);

      replace = nir_fmul(b,
                         approx,
                         nir_fsub(b,
                                  nir_imm_float(b, 2.0),
                                  nir_fmul(b, arg0, approx)));
      break;

   case nir_op_frsq:
      if (!(state->flags & nir_lower_alu_newton_raphson_frsq))
         break;

      arg0 = nir_fmov_alu(b, instr->src[0], instr->dest.dest.ssa.num_components);
      approx = nir_frsq(b, arg0);

      replace = nir_fmul(b,
                         approx,
                         nir_fsub(b,
                                  nir_imm_float(b, 1.5),
                                  nir_fmul(b,
                                           nir_fmul(b,
                                                    arg0,
                                                    nir_imm_float(b, 0.5)),
                                           nir_fmul(b,
                                                    approx,
                                                    approx))));
      break;

   default:
      break;
   }

   if (replace) {
      nir_ssa_def_rewrite_uses(&instr->dest.dest.ssa, nir_src_for_ssa(replace));
      nir_instr_remove(&instr->instr);
      state->progress = true;
   }
}

static bool
lower_alu_newton_raphson_block(nir_block *block, void *state)
{
   nir_foreach_instr_safe(block, instr) {
      if (instr->type == nir_instr_type_alu)
         lower_alu_instr_scalar(nir_instr_as_alu(instr), state);
   }

   return true;
}

bool
nir_lower_alu_newton_raphson(nir_shader *shader,
                             nir_lower_alu_newton_raphson_flags flags)
{
   struct nir_lower_alu_newton_raphson_state state = {
      .flags = flags,
      .progress = false,
   };

   nir_foreach_function(shader, function) {
      nir_function_impl *impl = function->impl;

      nir_builder_init(&state.b, impl);

      nir_foreach_block_call(impl, lower_alu_newton_raphson_block, &state);
   }

   return state.progress;
}
