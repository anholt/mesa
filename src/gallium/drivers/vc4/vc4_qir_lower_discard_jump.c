/*
 * Copyright © 2016 Broadcom
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

#include "vc4_qir.h"

static bool debug = true;

static struct qblock *
split_block(struct vc4_compile *c, struct qinst *inst, struct qblock *block)
{
        /* Make a new block to tack onto the end of the program. */
        struct qblock *new_block = qir_new_block(c);
        list_add(&new_block->link, &block->link);

        /* The new block is what will jump to the former block's successors. */
        for (int i = 0; i < ARRAY_SIZE(new_block->successors); i++) {
                new_block->successors[i] = block->successors[i];
                block->successors[i] = NULL;

                if (new_block->successors[i]) {
                        struct set *pred =
                                new_block->successors[i]->predecessors;
                        _mesa_set_remove(pred, _mesa_set_search(pred, block));
                        _mesa_set_add(pred, new_block);
                }
        }

        /* Move everything after "inst" to the new block. */
        qir_for_each_inst_safe_rev(move_inst, block) {
                list_del(&move_inst->link);
                list_add(&move_inst->link, &new_block->instructions);
                if (move_inst == inst)
                        break;
        }

        /* Don't set up the successor from block to new_block yet, as that
         * will be different for the two callers.
         */

        return new_block;
}

static void
lower_one_jump(struct vc4_compile *c,
               struct qinst *inst,
               struct qblock *block,
               struct qblock *jump_target_block)
{
        struct qinst *sf = qir_inst(QOP_MOV, c->undef,
                                    inst->src[0], c->undef);
        sf->sf = true;
        list_addtail(&sf->link, &inst->link);

        struct qinst *branch = qir_inst(QOP_BRANCH, c->undef,
                                        c->undef, c->undef);
        list_addtail(&branch->link, &inst->link);

        struct qblock *new_block = split_block(c, inst, block);
        qir_remove_instruction(c, inst);

        /* If c->discard has been cleared in all channels, then jump to the
         * end of the program.
         */
        branch->cond = QPU_COND_BRANCH_ALL_ZC;
        qir_link_blocks(block, jump_target_block);
        qir_link_blocks(block, new_block);

        if (debug) {
                fprintf(stderr, "QIR after emitting a branch:\n");
                qir_dump(c);
                fprintf(stderr, "\n");
        }
}


void
qir_lower_discard_jump(struct vc4_compile *c)
{
        struct qblock *end_block = list_last_entry(&c->blocks,
                                                   struct qblock, link);

        if (debug) {
                fprintf(stderr, "QIR before lowering discard jumps:\n");
                qir_dump(c);
                fprintf(stderr, "\n");
        }

        /* Walk the last block to find where we should jump to, and also
         * process any DISCARD_JUMPs we find.
         */
        struct qinst *first_tlb = NULL;
        int first_tlb_ip = 0;
        int ip_from_end = 0;
        qir_for_each_inst_safe_rev(inst, end_block) {
                ip_from_end++;

                switch (inst->dst.file) {
                case QFILE_TLB_Z_WRITE:
                case QFILE_TLB_COLOR_WRITE:
                case QFILE_TLB_COLOR_WRITE_MS:
                        first_tlb = inst;
                        first_tlb_ip = ip_from_end;
                        continue;
                default:
                        break;
                }

        }

        ip_from_end = 0;
        bool found_tex = false;
        struct qblock *jump_target = NULL;
        qir_for_each_block_rev(block, c) {
                qir_for_each_inst_safe_rev(inst, block) {
                        ip_from_end++;

                        if (qir_is_tex(inst))
                                found_tex = true;

                        if (inst->op != QOP_DISCARD_JUMP)
                                continue;

                        /* If we're only doing a few straight-line ALU ops
                         * before the end of the program, then the branch cost
                         * may be higher than the benefit of skipping them.
                         */
                        if ((ip_from_end - first_tlb_ip < 20) &&
                            block == end_block && !found_tex) {
                                qir_remove_instruction(c, inst);
                                continue;
                        }

                        if (!jump_target) {
                                jump_target = split_block(c, first_tlb,
                                                          end_block);
                                qir_link_blocks(end_block, jump_target);

                                /* The jump target is the first TLB write of
                                 * hte program, which is predicated on a
                                 * c->discard comparison, but the SF setting
                                 * it up may have been folded into any former
                                 * instruction.  Just emit a new one.
                                 */
                                assert(first_tlb->cond == QPU_COND_ZS);
                                struct qinst *sf = qir_inst(QOP_MOV, c->undef,
                                                            c->discard,
                                                            c->undef);
                                sf->sf = true;
                                list_addtail(&sf->link, &first_tlb->link);

                                if (debug) {
                                        fprintf(stderr, "QIR after splitting "
                                                "the end block:\n");
                                        qir_dump(c);
                                        fprintf(stderr, "\n");
                                }
                        }
                        lower_one_jump(c, inst, end_block, jump_target);
                }
        }

        if (debug) {
                fprintf(stderr, "QIR after lowering discard jumps:\n");
                qir_dump(c);
                fprintf(stderr, "\n");
        }
}
