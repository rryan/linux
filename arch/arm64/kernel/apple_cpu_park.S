/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Park loop of an offline Apple CPU, see apple_cpu_park().
 */

#include <linux/linkage.h>
#include <asm/assembler.h>

#define SYS_IMP_APL_CYC_OVRD		s3_5_c15_c5_0
#define SYS_IMP_APL_IPI_SR_EL1		s3_5_c15_c1_1
#define IPI_SR_PENDING			1

	.text

/*
 * u64 apple_cpu_park_wfi(u64 *cmd, u64 cyc, u64 ways, u64 sets,
 *			  u64 way_shift, u64 line_shift)
 *
 * Clean and invalidate the L1 data cache by set/way (@ways, @sets and the
 * shifts of the way and set fields), set CYC_OVRD to @cyc, then WFI until
 * *@cmd is non-zero and return it. Every wake-up acknowledges a fast IPI.
 *
 * A WFI that powers the core down with state retention keeps only sp and
 * pc, so the callee-saved registers and @cmd are kept on the stack. Nothing
 * is written to memory after the clean.
 */
SYM_FUNC_START(apple_cpu_park_wfi)
	stp	x29, x30, [sp, #-112]!
	mov	x29, sp
	stp	x18, x19, [sp, #16]
	stp	x20, x21, [sp, #32]
	stp	x22, x23, [sp, #48]
	stp	x24, x25, [sp, #64]
	stp	x26, x27, [sp, #80]
	stp	x28, x0, [sp, #96]

	sub	x2, x2, #1
1:	sub	x10, x3, #1
2:	lslv	x11, x2, x4
	lslv	x12, x10, x5
	orr	x11, x11, x12
	dc	cisw, x11
	subs	x10, x10, #1
	b.ge	2b
	subs	x2, x2, #1
	b.ge	1b
	dsb	sy

	msr	SYS_IMP_APL_CYC_OVRD, x1
	isb

3:	dsb	sy
	wfi
	mov	x9, #IPI_SR_PENDING
	msr	SYS_IMP_APL_IPI_SR_EL1, x9
	isb
	ldr	x9, [sp, #104]
	ldar	x0, [x9]
	cbz	x0, 3b

	ldp	x18, x19, [sp, #16]
	ldp	x20, x21, [sp, #32]
	ldp	x22, x23, [sp, #48]
	ldp	x24, x25, [sp, #64]
	ldp	x26, x27, [sp, #80]
	ldr	x28, [sp, #96]
	ldp	x29, x30, [sp], #112
	ret
SYM_FUNC_END(apple_cpu_park_wfi)
