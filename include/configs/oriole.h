/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef __ORIOLE_H
#define __ORIOLE_H

#define CFG_SYS_SDRAM_BASE	0x80000000

#define CFG_EXTRA_ENV_SETTINGS \
	"stdin=usbacm,buttons\0" \
	"stdout=usbacm,vidconsole\0" \
	"stderr=usbacm,vidconsole\0" \
	"bootmenu_0=U-Boot console=echo\0" \
	"bootmenu_1=Scan UFS=ufs init; scsi scan\0" \
	"bootmenu_2=Reboot=reset\0" \
	"bootmenu_3=Power off=poweroff\0" \
	"bootcmd=bootmenu 30\0"

#endif
