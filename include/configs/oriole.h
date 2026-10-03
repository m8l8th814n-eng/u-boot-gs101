/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef __ORIOLE_H
#define __ORIOLE_H

#define CFG_SYS_SDRAM_BASE	0x80000000

#define CFG_EXTRA_ENV_SETTINGS \
	"stdin=usbacm\0" \
	"stdout=usbacm,vidconsole\0" \
	"stderr=usbacm,vidconsole\0"

#endif
