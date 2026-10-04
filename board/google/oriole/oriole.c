// SPDX-License-Identifier: GPL-2.0+

#include <init.h>
#include <stdio.h>
#include <asm/armv8/mmu.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <cpu_func.h>
#include <cyclic.h>
#include <debug_uart.h>
#include <linux/bitops.h>
#include <linux/sizes.h>

#define GS101_WDT_CL0		0x10060000
#define GS101_WDT_CL1		0x10070000
#define WTCON			0x00

#define GS101_DSIM0		0x1c2c0000

#define GS101_DECON0		0x1c300000
#define DECON_TRIG_CON		0x30
#define DECON_TRIG_HW		(BIT(13) | BIT(12) | BIT(6) | BIT(5) | BIT(0))
#define DECON_SHD_REG_UP_REQ	0x50
#define DECON_SHD_UP_ALL	(BIT(31) | BIT(20) | GENMASK(5, 0))

#define GS101_DPP0_DMA		0x1c0b0000
#define RDMA_SRC_SIZE		0x10
#define RDMA_IMG_SIZE		0x18
#define RDMA_BASEADDR_Y8	0x40
#define ORIOLE_FB		0xfac00000UL
#define FB_WIDTH		1080
#define FB_STRIDE		4320

static int oriole_fbs(ulong *fbs)
{
	int n = 0, ch, k;

	fbs[n++] = 0xfac00000UL;
	for (ch = 0; ch < 6; ch++) {
		ulong fb = readl(GS101_DPP0_DMA + (ulong)ch * 0x1000 + RDMA_BASEADDR_Y8);

		if (fb < 0x80000000UL || fb >= 0x100000000UL)
			continue;
		for (k = 0; k < n && fbs[k] != fb; k++)
			;
		if (k == n)
			fbs[n++] = fb;
	}
	return n;
}

static void oriole_fill(ulong fb, int x, int y, int w, int h, u32 color)
{
	int i, j;

	for (j = y; j < y + h; j++) {
		u32 *row = (u32 *)(fb + (ulong)j * FB_STRIDE);

		for (i = x; i < x + w && i < FB_WIDTH; i++)
			row[i] = color;
	}
	flush_dcache_range(fb + (ulong)y * FB_STRIDE,
			   fb + (ulong)(y + h) * FB_STRIDE);
}

static void oriole_kick(void)
{
	writel(DECON_TRIG_HW, GS101_DECON0 + DECON_TRIG_CON);
	writel(DECON_SHD_UP_ALL, GS101_DECON0 + DECON_SHD_REG_UP_REQ);
}

static void __maybe_unused oriole_mark(int slot, u32 color)
{
	ulong fbs[7];
	int i, n = oriole_fbs(fbs);

	for (i = 0; i < n; i++)
		oriole_fill(fbs[i], 0, slot * 150, FB_WIDTH, 150, color);
	oriole_kick();
}

static void __maybe_unused oriole_bits(int y, u32 val)
{
	ulong fbs[7];
	int i, f, n = oriole_fbs(fbs);

	for (f = 0; f < n; f++)
		for (i = 0; i < 32; i++)
			oriole_fill(fbs[f], i * 33, y, 30, 60,
				    (val >> (31 - i)) & 1 ? 0xffffffff : 0xff202020);
	oriole_kick();
}

DECLARE_GLOBAL_DATA_PTR;

static struct mm_region oriole_mem_map[] = {
	{
		.virt = 0x00000000UL,
		.phys = 0x00000000UL,
		.size = 0x80000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			 PTE_BLOCK_NON_SHARE |
			 PTE_BLOCK_PXN | PTE_BLOCK_UXN,
	}, {
		.virt = 0x80000000UL,
		.phys = 0x80000000UL,
		.size = 0x80000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			 PTE_BLOCK_INNER_SHARE,
	}, {
		0,
	}
};

struct mm_region *mem_map = oriole_mem_map;

int dram_init(void)
{
	/* oriole_mark(1, 0xffffff00); */
	gd->ram_size = SZ_2G;
	return 0;
}

phys_addr_t board_get_usable_ram_top(phys_size_t total_size)
{
	return 0xf8800000;
}

/*
 * phys_addr_t board_get_usable_ram_top(phys_size_t total_size)
 * {
 *	return 0xe0000000;
 * }
 */

static void oriole_take_over_display(void)
{
	u32 size = (2400 << 16) | FB_WIDTH;

	writel(size, GS101_DPP0_DMA + RDMA_SRC_SIZE);
	writel(size, GS101_DPP0_DMA + RDMA_IMG_SIZE);
	writel(ORIOLE_FB, GS101_DPP0_DMA + RDMA_BASEADDR_Y8);
	oriole_kick();
}

int board_early_init_f(void)
{
	writel(0, GS101_WDT_CL0 + WTCON);
	writel(0, GS101_WDT_CL1 + WTCON);
	oriole_take_over_display();
	if (IS_ENABLED(CONFIG_DEBUG_UART_ORIOLE_FB)) {
		debug_uart_init();
		printascii("oriole: board_early_init_f\n");
	}
	/* oriole_mark(0, 0xffff0000); */
	return 0;
}

int board_early_init_r(void)
{
	/* oriole_mark(4, 0xff00ffff); */
	return 0;
}

int misc_init_r(void)
{
	/* oriole_mark(5, 0xffff00ff); */
	return 0;
}

static struct cyclic_info oriole_kick_cyclic;

static void oriole_kick_cb(struct cyclic_info *c)
{
	ulong fbs[7];
	int i, n = oriole_fbs(fbs);

	for (i = 0; i < n; i++)
		flush_dcache_range(fbs[i], fbs[i] + 2400 * FB_STRIDE);
	oriole_kick();
}

int board_init(void)
{
	/* oriole_mark(2, 0xff00ff00); */
	cyclic_register(&oriole_kick_cyclic, oriole_kick_cb, 50000, "decon_kick");
	return 0;
}

static void oriole_dump_dsim(void)
{
	static const struct {
		const char *name;
		ulong off;
	} regs[] = {
		{ "VERSION", 0x00 },
		{ "LINK_STATUS0", 0x08 },
		{ "LINK_STATUS1", 0x0c },
		{ "LINK_STATUS2", 0x10 },
		{ "LINK_STATUS3", 0x14 },
		{ "MIPI_STATUS", 0x18 },
		{ "DPHY_STATUS", 0x1c },
		{ "CLK_CTRL", 0x20 },
		{ "ESCMODE", 0x2c },
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(regs); i++)
		printf("DSIM0 %-13s 0x%08x\n", regs[i].name,
		       readl(GS101_DSIM0 + regs[i].off));
}

int board_late_init(void)
{
	/* oriole_mark(3, 0xff0000ff); */
	/* oriole_bits(920, readl(GS101_DSIM0 + 0x0c)); */
	/* oriole_bits(1000, readl(GS101_DSIM0 + 0x1c)); */
	oriole_dump_dsim();
	return 0;
}

void ft_board_setup_ex(void *blob, struct bd_info *bd)
{
}
