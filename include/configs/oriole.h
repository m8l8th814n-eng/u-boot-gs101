/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef __ORIOLE_H
#define __ORIOLE_H

#define CFG_SYS_SDRAM_BASE	0x80000000

#define CFG_EXTRA_ENV_SETTINGS \
	/* \
	 * The USB console reuses the controller the stock bootloader leaves \
	 * running in fastboot mode; on a normal boot from boot_a it is not \
	 * set up and the console hangs, so it is opt-in from the menu. \
	 * "stdin=usbacm,buttons\0" \
	 * "stdout=usbacm,vidconsole\0" \
	 * "stderr=usbacm,vidconsole\0" \
	 */ \
	"stdin=buttons\0" \
	"stdout=vidconsole\0" \
	"stderr=vidconsole\0" \
	"usbcon=setenv stdin usbacm,buttons; setenv stdout usbacm,vidconsole; " \
		"setenv stderr usbacm,vidconsole\0" \
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
	"boot_efi=run pmos_map; " \
		"load blkmap 0:1 ${fdt_addr_r} gs101-oriole.dtb && " \
		"load blkmap 0:1 ${kernel_addr_r} EFI/BOOT/BOOTAA64.EFI && " \
		"bootefi ${kernel_addr_r} ${fdt_addr_r}\0" \
	"bootmenu_0=systemd-boot (ESP)=run boot_efi\0" \
	"bootmenu_1=Boot postmarketOS=run boot_pmos\0" \
	"bootmenu_2=U-Boot console=echo\0" \
	"bootmenu_3=USB console (fastboot boot only)=run usbcon\0" \
	"bootmenu_4=USB console (boot_a)=echo usb: phy; oriole_usb 5; echo usb: acm; run usbcon; echo usb: done\0" \
	"bootmenu_5=USB step 4: PHY init=oriole_usb 4; sleep 60\0" \
	"bootmenu_6=USB step 3: all=oriole_usb 3; sleep 60\0" \
	"bootmenu_7=USB step 0: dump=oriole_usb 0; sleep 20\0" \
	"bootmenu_8=USB step 1: clocks+isolation=oriole_usb 1; sleep 20\0" \
	"bootmenu_9=USB step 2: read PHY/DWC3=oriole_usb 2; sleep 20\0" \
	"bootmenu_10=Scan UFS=ufs init; scsi scan\0" \
	"bootmenu_11=Reboot=reset\0" \
	"bootmenu_12=Power off=poweroff\0" \
	"bootcmd=bootmenu 30\0"

#endif
