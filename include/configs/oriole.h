/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef __ORIOLE_H
#define __ORIOLE_H

#define CFG_SYS_SDRAM_BASE	0x80000000

#define CFG_EXTRA_ENV_SETTINGS \
	"stdin=usbacm,buttons\0" \
	"stdout=usbacm,vidconsole\0" \
	"stderr=usbacm,vidconsole\0" \
	"kernel_addr_r=0x80080000\0" \
	"kernel_comp_addr_r=0x88000000\0" \
	"kernel_comp_size=0x4000000\0" \
	"fdt_addr_r=0x9e000000\0" \
	"ramdisk_addr_r=0xa0000000\0" \
	"pmos_map=ufs init; scsi scan; " \
		"part start scsi 0 userdata udstart; " \
		"part size scsi 0 userdata udsize; " \
		"blkmap destroy pmos; blkmap create pmos; " \
		"blkmap map pmos 0 0x${udsize} linear scsi 0 0x${udstart}\0" \
	"boot_pmos=run pmos_map; " \
		"load blkmap 0:1 ${kernel_comp_addr_r} vmlinuz && " \
		"unlz4l ${kernel_comp_addr_r} ${kernel_addr_r} ${filesize} && " \
		"load blkmap 0:1 ${ramdisk_addr_r} initramfs && " \
		"setenv rd_size ${filesize} && " \
		"load blkmap 0:1 ${fdt_addr_r} gs101-oriole.dtb && " \
		"booti ${kernel_addr_r} ${ramdisk_addr_r}:${rd_size} ${fdt_addr_r}\0" \
	"bootmenu_0=Boot postmarketOS=run boot_pmos\0" \
	"bootmenu_1=U-Boot console=echo\0" \
	"bootmenu_2=Scan UFS=ufs init; scsi scan\0" \
	"bootmenu_3=Reboot=reset\0" \
	"bootmenu_4=Power off=poweroff\0" \
	"bootcmd=bootmenu 30\0"

#endif
