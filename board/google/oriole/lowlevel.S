/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * The stock bootloader enters with x0 pointing at its device tree, which
 * carries the real memory banks and per-device data in /chosen. Keep it.
 */

#include <linux/linkage.h>

ENTRY(save_boot_params)
	adrp	x9, oriole_abl_fdt
	add	x9, x9, :lo12:oriole_abl_fdt
	str	x0, [x9]
	b	save_boot_params_ret
ENDPROC(save_boot_params)

	.data
	.align	3
	.globl	oriole_abl_fdt
oriole_abl_fdt:
	.quad	0
