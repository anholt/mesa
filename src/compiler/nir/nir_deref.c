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
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include "nir.h"
#include "nir_builder.h"
#include "nir_deref.h"

void
nir_deref_path_init(struct nir_deref_path *path,
                    nir_deref_instr *deref, void *mem_ctx)
{
   /* The length of the short path is at most ARRAY_SIZE - 1 because we need
    * room for the NULL terminator.
    */
   static const int max_short_path_len = ARRAY_SIZE(path->_short_path) - 1;

   int count = 0;

   nir_deref_instr **tail = &path->_short_path[max_short_path_len];
   nir_deref_instr **head = tail;

   *tail = NULL;
   for (nir_deref_instr *d = deref; d; d = nir_deref_instr_parent(d)) {
      count++;
      if (count <= max_short_path_len)
         *(--head) = d;
   }

   if (count <= max_short_path_len) {
      /* If we're under max_short_path_len, just use the short path. */
      path->path = head;
      goto done;
   }

   path->path = ralloc_array(mem_ctx, nir_deref_instr *, count + 1);
   head = tail = path->path + count;
   *tail = NULL;
   for (nir_deref_instr *d = deref; d; d = nir_deref_instr_parent(d))
      *(--head) = d;

done:
   assert(head == path->path);
   assert(tail == head + count);
   assert((*head)->deref_type == nir_deref_type_var);
   assert(*tail == NULL);
}

void
nir_deref_path_finish(struct nir_deref_path *path)
{
   if (path->path < &path->_short_path[0] ||
       path->path > &path->_short_path[ARRAY_SIZE(path->_short_path) - 1])
      ralloc_free(path->path);
}

bool
nir_deref_instr_cleanup(nir_deref_instr *instr)
{
   bool progress = false;

   for (nir_deref_instr *d = instr; d; d = nir_deref_instr_parent(d)) {
      /* If anyone is using this deref, leave it alone */
      assert(d->dest.is_ssa);
      if (!list_empty(&d->dest.ssa.uses))
         break;

      nir_instr_remove(&d->instr);
      progress = true;
   }

   return progress;
}

void
nir_deref_function_cleanup(nir_function_impl *impl)
{
   bool progress;
   do {
      progress = false;
      nir_foreach_block(block, impl)
         nir_foreach_instr_safe(instr, block)
            if (instr->type == nir_instr_type_deref)
               progress |= nir_deref_instr_cleanup(nir_instr_as_deref(instr));
   } while (progress);
}

void
nir_fixup_deref_modes(nir_shader *shader)
{
   nir_foreach_function(function, shader) {
      if (!function->impl)
         continue;

      nir_foreach_block(block, function->impl) {
         nir_foreach_instr(instr, block) {
            if (instr->type != nir_instr_type_deref)
               continue;

            nir_deref_instr *deref = nir_instr_as_deref(instr);

            nir_variable_mode parent_mode;
            if (deref->deref_type == nir_deref_type_var) {
               parent_mode = deref->var->data.mode;
            } else {
               assert(deref->parent.is_ssa);
               nir_deref_instr *parent =
                  nir_instr_as_deref(deref->parent.ssa->parent_instr);
               parent_mode = parent->mode;
            }

            deref->mode = parent_mode;
         }
      }
   }
}
