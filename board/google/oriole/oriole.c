// SPDX-License-Identifier: GPL-2.0+

#include <init.h>
#include <stdio.h>
#include <asm/armv8/mmu.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <cpu_func.h>
#include <cyclic.h>
#include <debug_uart.h>
#include <dm.h>
#include <sysreset.h>
#include <stdio_dev.h>
#include <time.h>
#include <video.h>
#include <env.h>
#include <malloc.h>
#include <lmb.h>
#include <efi_loader.h>
#include <command.h>
#include <mapmem.h>
#include <u-boot/lz4.h>
#include <linux/unaligned/le_byteshift.h>
#include <fdt_support.h>
#include <linux/libfdt.h>
#include <linux/arm-smccc.h>
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
#define RDMA_IN_CTRL_0		0x08
#define IDMA_IMG_FORMAT_MASK	(0x3f << 8)
#define IDMA_IMG_FORMAT_ARGB2101010	(19 << 8)
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
		/* DRAM above 4 GiB, where the remaining banks live */
		.virt = 0x800000000UL,
		.phys = 0x800000000UL,
		.size = 0x800000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			 PTE_BLOCK_INNER_SHARE,
	}, {
		0,
	}
};

struct mm_region *mem_map = oriole_mem_map;

extern ulong oriole_abl_fdt;

static struct {
	u64 start;
	u64 size;
} oriole_banks[CONFIG_NR_DRAM_BANKS] __section(".data");
static int oriole_nbanks __section(".data");

/* Read the memory banks the stock bootloader filled into its device tree */
static int oriole_abl_banks(void)
{
	const void *fdt = (const void *)oriole_abl_fdt;
	int node, na, ns, len, n = 0;
	const fdt32_t *reg;

	if (!fdt || fdt_check_header(fdt))
		return 0;
	na = fdt_address_cells(fdt, 0);
	ns = fdt_size_cells(fdt, 0);

	for (node = fdt_node_offset_by_prop_value(fdt, -1, "device_type",
						  "memory", 7);
	     node >= 0;
	     node = fdt_node_offset_by_prop_value(fdt, node, "device_type",
						  "memory", 7)) {
		reg = fdt_getprop(fdt, node, "reg", &len);
		if (!reg)
			continue;
		len /= sizeof(*reg);
		while (len >= na + ns && n < CONFIG_NR_DRAM_BANKS) {
			oriole_banks[n].start = fdt_read_number(reg, na);
			oriole_banks[n].size = fdt_read_number(reg + na, ns);
			if (oriole_banks[n].size)
				n++;
			reg += na + ns;
			len -= na + ns;
		}
	}
	return n;
}

int dram_init(void)
{
	int i;

	/* oriole_mark(1, 0xffffff00); */
	oriole_nbanks = oriole_abl_banks();
	if (!oriole_nbanks) {
		oriole_banks[0].start = 0x80000000;
		oriole_banks[0].size = SZ_2G;
		oriole_nbanks = 1;
	}

	gd->ram_size = 0;
	for (i = 0; i < oriole_nbanks; i++)
		gd->ram_size += oriole_banks[i].size;
	return 0;
}

int dram_init_banksize(void)
{
	int i;

	for (i = 0; i < oriole_nbanks; i++) {
		gd->dram[i].start = oriole_banks[i].start;
		gd->dram[i].size = oriole_banks[i].size;
	}
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

	/*
	 * clrsetbits_le32(GS101_DPP0_DMA + RDMA_IN_CTRL_0, IDMA_IMG_FORMAT_MASK,
	 *		IDMA_IMG_FORMAT_ARGB2101010);
	 */
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

void board_video_sync(void)
{
	static bool busy;
	struct udevice *vid;

	/* keep the U-Boot logo in the top right corner over menus and text */
	if (!busy && !uclass_first_device_err(UCLASS_VIDEO, &vid)) {
		busy = true;
		video_bmp_display(vid, map_to_sysmem(video_get_u_boot_logo()),
				  -4, 4, true);
		busy = false;
	}
	oriole_kick();
}

/*
 * The kernel's Image.lz4 uses the LZ4 legacy format (lz4 -l): a 0x184c2102
 * magic, then blocks of a 32-bit compressed length and an LZ4 block of up
 * to 8 MiB uncompressed, with the uncompressed size appended at the end.
 * U-Boot only knows the LZ4 frame format, so unpack it here.
 */
#define LZ4_LEGACY_MAGIC	0x184c2102
#define LZ4_LEGACY_BLOCK	(8 << 20)

static int do_unlz4l(struct cmd_tbl *cmdtp, int flag, int argc,
		     char *const argv[])
{
	const u8 *src, *end;
	u8 *dst, *out;
	ulong srclen;
	u32 len;
	int n;

	if (argc != 4)
		return CMD_RET_USAGE;
	src = map_sysmem(hextoul(argv[1], NULL), 0);
	dst = out = map_sysmem(hextoul(argv[2], NULL), 0);
	srclen = hextoul(argv[3], NULL);
	end = src + srclen;

	if (srclen < 8 || get_unaligned_le32(src) != LZ4_LEGACY_MAGIC) {
		printf("not an LZ4 legacy stream\n");
		return CMD_RET_FAILURE;
	}
	src += 4;
	while (end - src > 4) {
		len = get_unaligned_le32(src);
		src += 4;
		if (len == LZ4_LEGACY_MAGIC)
			continue;
		if (!len || len > end - src)
			break;
		n = LZ4_decompress_safe((const char *)src, (char *)out, len,
					LZ4_LEGACY_BLOCK);
		if (n < 0) {
			printf("lz4 block error %d\n", n);
			return CMD_RET_FAILURE;
		}
		src += len;
		out += n;
	}
	printf("%lu bytes unpacked\n", (ulong)(out - dst));
	env_set_hex("filesize", out - dst);
	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(unlz4l, 4, 0, do_unlz4l,
	   "unpack an LZ4 legacy stream (kernel Image.lz4)",
	   "<src> <dst> <srclen>");

/*
 * Volume keys as console input, so menus work without a keyboard:
 * volume up = arrow up, volume down = arrow down, power or both = Enter.
 * The volume keys sit in the far-alive pin controller, power in the alive
 * one (gpa10-1); all read low when pressed.
 */
#define GS101_ALIVE		0x174d0000
#define GPA10_DAT		(GS101_ALIVE + 0xe4)
#define KEY_POWER_BIT		BIT(1)
#define GS101_FAR_ALIVE		0x174e0000
#define GPA7_DAT		(GS101_FAR_ALIVE + 0x24)
#define GPA8_DAT		(GS101_FAR_ALIVE + 0x44)
#define KEY_VOLDOWN_BIT		BIT(3)
#define KEY_VOLUP_BIT		BIT(1)

static char keys_buf[4];
static int keys_len, keys_pos;
static int keys_last;
static ulong keys_time;

static int keys_read(void)
{
	int up = !(readl(GPA8_DAT) & KEY_VOLUP_BIT);
	int down = !(readl(GPA7_DAT) & KEY_VOLDOWN_BIT);

	int power = !(readl(GPA10_DAT) & KEY_POWER_BIT);

	return power ? 3 : (up ? 1 : 0) | (down ? 2 : 0);
}

static void keys_poll(void)
{
	int now;

	if (keys_pos < keys_len || get_timer(keys_time) < 30)
		return;
	keys_time = get_timer(0);

	now = keys_read();
	if (now == keys_last)
		return;
	if (now == 3) {
		keys_buf[0] = '\r';
		keys_len = 1;
	} else if (now && !keys_last) {
		keys_buf[0] = 0x1b;
		keys_buf[1] = '[';
		keys_buf[2] = now == 1 ? 'A' : 'B';
		keys_len = 3;
	} else {
		keys_len = 0;
	}
	keys_pos = 0;
	keys_last = now;
}

static int keys_tstc(struct stdio_dev *dev)
{
	keys_poll();
	return keys_pos < keys_len;
}

static int keys_getc(struct stdio_dev *dev)
{
	while (!keys_tstc(dev))
		schedule();
	return keys_buf[keys_pos++];
}

static void oriole_keys_register(void)
{
	struct stdio_dev dev = {
		.name	= "buttons",
		.flags	= DEV_FLAGS_INPUT,
		.tstc	= keys_tstc,
		.getc	= keys_getc,
	};

	keys_last = keys_read();
	stdio_register(&dev);
}

static int oriole_video_ret = 1;

int board_early_init_r(void)
{
	struct udevice *vid;
	int ret;

	ret = uclass_first_device_err(UCLASS_VIDEO, &vid);
	oriole_video_ret = ret;
	if (ret)
		printf("oriole: video probe failed %d\n", ret);

	oriole_keys_register();
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

/*
 * A private copy of the stock bootloader's device tree, taken once U-Boot
 * runs from its relocated copy and before anything loads over the
 * original. Its /chosen (bootargs from vendor_boot, BT and Wi-Fi
 * addresses in config/, plat for the modem) is handed on to Linux.
 */
static void *oriole_abl_copy;

static void oriole_save_abl_fdt(void)
{
	const void *fdt = (const void *)oriole_abl_fdt;
	int size;

	if (!fdt || fdt_check_header(fdt))
		return;
	size = fdt_totalsize(fdt);
	oriole_abl_copy = memalign(8, size);
	if (oriole_abl_copy)
		memcpy(oriole_abl_copy, fdt, size);
}

static int oriole_copy_node(const void *src, int soff, void *dst, int doff)
{
	const char *name;
	const void *val;
	int prop, sub, dsub, len, ret;

	fdt_for_each_property_offset(prop, src, soff) {
		val = fdt_getprop_by_offset(src, prop, &name, &len);
		if (!val || !strcmp(name, "linux,initrd-start") ||
		    !strcmp(name, "linux,initrd-end"))
			continue;
		ret = fdt_setprop(dst, doff, name, val, len);
		if (ret)
			return ret;
	}
	fdt_for_each_subnode(sub, src, soff) {
		name = fdt_get_name(src, sub, NULL);
		dsub = fdt_subnode_offset(dst, doff, name);
		if (dsub < 0)
			dsub = fdt_add_subnode(dst, doff, name);
		if (dsub < 0)
			return dsub;
		ret = oriole_copy_node(src, sub, dst, dsub);
		if (ret)
			return ret;
	}
	return 0;
}

/*
 * The stock bootloader adds reserved-memory nodes at run time that are
 * not in the kernel's device tree: secure DRAM and its page tables
 * (sec_dram, sec_pt), pKVM guest firmware and debug_kinfo. Linux must not
 * touch them, so carry over every node the kernel's tree lacks.
 */
static void oriole_copy_reserved(void *blob)
{
	const void *abl = oriole_abl_copy;
	const char *name;
	int soff, doff, sub, dsub;

	soff = fdt_path_offset(abl, "/reserved-memory");
	doff = fdt_path_offset(blob, "/reserved-memory");
	if (soff < 0 || doff < 0)
		return;
	fdt_for_each_subnode(sub, abl, soff) {
		name = fdt_get_name(abl, sub, NULL);
		if (fdt_subnode_offset(blob, doff, name) >= 0)
			continue;
		dsub = fdt_add_subnode(blob, doff, name);
		if (dsub < 0 || oriole_copy_node(abl, sub, blob, dsub))
			printf("oriole: copying reserved-memory/%s failed\n", name);
	}
}

/* keep U-Boot and EFI allocations out of the same regions */
static void oriole_reserve_abl_regions(void)
{
	const void *abl = oriole_abl_copy;
	const fdt32_t *reg;
	phys_addr_t base;
	u64 size;
	int off, sub, len;

	off = fdt_path_offset(abl, "/reserved-memory");
	if (off < 0)
		return;
	fdt_for_each_subnode(sub, abl, off) {
		reg = fdt_getprop(abl, sub, "reg", &len);
		if (!reg || len != 12)
			continue;
		base = ((u64)fdt32_to_cpu(reg[0]) << 32) | fdt32_to_cpu(reg[1]);
		size = fdt32_to_cpu(reg[2]);
		lmb_alloc_mem(LMB_MEM_ALLOC_ADDR, 0, &base, size, LMB_NOMAP);
		if (IS_ENABLED(CONFIG_EFI_LOADER))
			efi_add_memory_map(base, size, EFI_RESERVED_MEMORY_TYPE);
	}
}

int ft_board_setup(void *blob, struct bd_info *bd)
{
	int soff, doff, ret;

	if (!oriole_abl_copy)
		return 0;
	oriole_copy_reserved(blob);
	soff = fdt_path_offset(oriole_abl_copy, "/chosen");
	if (soff < 0)
		return 0;
	doff = fdt_path_offset(blob, "/chosen");
	if (doff < 0)
		doff = fdt_add_subnode(blob, 0, "chosen");
	if (doff < 0)
		return doff;
	ret = oriole_copy_node(oriole_abl_copy, soff, blob, doff);
	if (ret)
		printf("oriole: copying bootloader /chosen failed: %s\n",
		       fdt_strerror(ret));
	return 0;
}

int board_late_init(void)
{
	env_set_ulong("oriole_video_ret", (ulong)(long)oriole_video_ret);
	env_set_hex("abl_fdt", oriole_abl_fdt);
	oriole_save_abl_fdt();
	if (oriole_abl_copy)
		oriole_reserve_abl_regions();
	/* oriole_mark(3, 0xff0000ff); */
	/* oriole_bits(920, readl(GS101_DSIM0 + 0x0c)); */
	/* oriole_bits(1000, readl(GS101_DSIM0 + 0x1c)); */
	oriole_dump_dsim();
	return 0;
}

void ft_board_setup_ex(void *blob, struct bd_info *bd)
{
}

#define TENSOR_SMC_PMU_SEC_REG	0x82000504
#define TENSOR_PMUREG_RMW	2
#define GS101_PMU		0x17460000
#define PMU_SYSTEM_CONFIGURATION	0x3a00
#define PMU_SWRESET_SYSTEM	BIT(1)
#define PMU_PAD_CTRL_PWR_HOLD	0x3e9c
#define PMU_PWR_HOLD		BIT(8)

static void oriole_pmu_rmw(u32 reg, u32 mask, u32 val)
{
	struct arm_smccc_res res;

	arm_smccc_smc(TENSOR_SMC_PMU_SEC_REG, GS101_PMU + reg,
		      TENSOR_PMUREG_RMW, mask, val, 0, 0, 0, &res);
}

static int oriole_sysreset_request(struct udevice *dev, enum sysreset_t type)
{
	switch (type) {
	case SYSRESET_WARM:
	case SYSRESET_COLD:
		oriole_pmu_rmw(PMU_SYSTEM_CONFIGURATION, PMU_SWRESET_SYSTEM,
			       PMU_SWRESET_SYSTEM);
		break;
	case SYSRESET_POWER_OFF:
		oriole_pmu_rmw(PMU_PAD_CTRL_PWR_HOLD, PMU_PWR_HOLD, 0);
		break;
	default:
		return -EPROTONOSUPPORT;
	}
	return -EINPROGRESS;
}

static struct sysreset_ops oriole_sysreset_ops = {
	.request = oriole_sysreset_request,
};

U_BOOT_DRIVER(oriole_sysreset) = {
	.name	= "oriole_sysreset",
	.id	= UCLASS_SYSRESET,
	.ops	= &oriole_sysreset_ops,
};

U_BOOT_DRVINFO(oriole_sysreset) = {
	.name	= "oriole_sysreset",
};
