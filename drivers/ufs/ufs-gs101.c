// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Tensor gs101 UFS host controller and UFS PHY.
 *
 * Ported from the Linux exynos UFS host driver (ufs-exynos.c) and the
 * Samsung UFS PHY driver (phy-samsung-ufs.c, phy-gs101-ufs.c). The stock
 * bootloader has the UFS clocks, power and PHY isolation set up already, so
 * this only programs the controller, UniPro and the PHY calibration. The
 * link is kept in PWM (SLOW) mode: the HS calibration steps need power-mode
 * change hooks the U-Boot UFS core does not provide.
 */

#include <dm.h>
#include <dm/device_compat.h>
#include <dm/ofnode.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/iopoll.h>

#include "ufs.h"
#include "ufshci.h"
#include "unipro.h"

/* clock rates the bootloader leaves set (core_clk, sclk_unipro_main) */
#define GS101_UFS_PCLK_RATE	266496000UL
#define GS101_UFS_MCLK_RATE	133248000UL

/* vendor HCI registers */
#define HCI_TXPRDT_ENTRY_SIZE	0x00
#define HCI_RXPRDT_ENTRY_SIZE	0x04
#define HCI_1US_TO_CNT_VAL	0x0c
#define CNT_VAL_1US_MASK	0x3ff
#define HCI_UTRL_NEXUS_TYPE	0x40
#define HCI_UTMRL_NEXUS_TYPE	0x44
#define HCI_SW_RST		0x50
#define UFS_LINK_SW_RST		BIT(0)
#define UFS_UNIPRO_SW_RST	BIT(1)
#define UFS_SW_RST_MASK		(UFS_UNIPRO_SW_RST | UFS_LINK_SW_RST)
#define HCI_DATA_REORDER	0x60
#define HCI_AXIDMA_RWDATA_BURST_LEN	0x6c
#define WLU_EN			BIT(31)
#define WLU_BURST_LEN(x)	((x) << 27 | ((x) & 0xf))
#define HCI_GPIO_OUT		0x70
#define HCI_ERR_EN_DL_LAYER	0x7c
#define HCI_ERR_EN_N_LAYER	0x80
#define HCI_ERR_EN_T_LAYER	0x84
#define HCI_V2P1_CTRL		0x8c
#define IA_TICK_SEL		BIT(16)
#define HCI_CLKSTOP_CTRL	0xb0
#define CLK_STOP_MASK		GENMASK(4, 0)
#define HCI_MISC		0xb4
#define HCI_CORECLK_CTRL_EN	BIT(4)
#define CLK_CTRL_EN_MASK	(BIT(7) | BIT(6) | BIT(5))
#define HCI_IOP_ACG_DISABLE	0x100
#define HCI_IOP_ACG_DISABLE_EN	BIT(0)

#define DFES_ERR_EN		BIT(31)
#define DFES_DEF_L2_ERRS	(BIT(0) | BIT(13))
#define DFES_DEF_L3_ERRS	(BIT(0) | BIT(1) | BIT(2))
#define DFES_DEF_L4_ERRS	(BIT(0) | BIT(1) | BIT(2) | BIT(5))

#define UFS_GS101_SHARABLE	(BIT(1) | BIT(0))
#define UFS_SHAREABILITY_OFFSET	0x710

/* UniPro block */
#define COMP_CLK_PERIOD		0x44
#define UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER0	0x7888
#define UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER1	0x788c
#define UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER2	0x7890
#define UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER0	0x78b8
#define UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER1	0x78bc
#define UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER2	0x78c0

/* DME attributes */
#define PA_DBG_MODE		0x9529
#define PA_GS101_DBG_OPTION_SUITE1	0x956a
#define PA_GS101_DBG_OPTION_SUITE2	0x956d
#define VND_TX_CLK_PRD		0xaa
#define VND_TX_CLK_PRD_EN	0xa9
#define VND_TX_LINERESET_PVALUE0	0xad
#define VND_TX_LINERESET_PVALUE1	0xac
#define VND_TX_LINERESET_PVALUE2	0xab
#define TX_LINE_RESET_TIME	3200
#define VND_RX_CLK_PRD		0x12
#define VND_RX_CLK_PRD_EN	0x11
#define VND_RX_LINERESET_VALUE0	0x1d
#define VND_RX_LINERESET_VALUE1	0x1c
#define VND_RX_LINERESET_VALUE2	0x1b
#define RX_LINE_RESET_TIME	1000

#ifndef CPORT_IDLE
#define CPORT_IDLE		0
#endif
#ifndef CPORT_CONNECTED
#define CPORT_CONNECTED		1
#endif
#ifndef CPORT_DEF_FLAGS
#define CPORT_DEF_FLAGS		0x6
#endif

#define IATOVAL_NSEC		20000
#define CNTR_DIV_VAL		40
#define DATA_UNIT_SHIFT		12
#define PA_GRANULARITY_VAL	0x6
#define PA_TACTIVATE_VAL	0x3
#define PA_HIBERN8TIME_VAL	0x20

/* PHY */
#define PHY_APB_ADDR(off)	((off) << 2)
#define PHY_GS101_LANE_OFFSET	0x200
#define PHY_TRSV_ADDR(reg, lane) \
	PHY_APB_ADDR((reg) + (lane) * PHY_GS101_LANE_OFFSET)
#define TRSV_REG338		0x338
#define LN0_MON_RX_CAL_DONE	BIT(3)

struct gs101_phy_cfg {
	u16 off;
	u8 val;
	u8 trsv;
};

#define C(o, v)	{ (o), (v), 0 }
#define T(o, v)	{ (o), (v), 1 }

static const struct gs101_phy_cfg gs101_phy_pre_init[] = {
	C(0x43, 0x10), C(0x3c, 0x14), C(0x46, 0x48),
	T(0x200, 0x00), T(0x201, 0x06), T(0x202, 0x06), T(0x203, 0x0a),
	T(0x204, 0x00), T(0x205, 0x11), T(0x207, 0x0c), T(0x2e1, 0xc0),
	T(0x22d, 0xb8), T(0x234, 0x60), T(0x238, 0x13), T(0x239, 0x48),
	T(0x23a, 0x01), T(0x23b, 0x25), T(0x23c, 0x2a), T(0x23d, 0x01),
	T(0x23e, 0x13), T(0x23f, 0x13), T(0x240, 0x4a), T(0x243, 0x40),
	T(0x244, 0x02), T(0x25d, 0x00), T(0x25e, 0x3f), T(0x25f, 0xff),
	T(0x273, 0x33), T(0x274, 0x50), T(0x284, 0x02), T(0x285, 0x02),
	T(0x2a2, 0x04), T(0x25d, 0x01), T(0x2fa, 0x01), T(0x286, 0x03),
	T(0x287, 0x03), T(0x288, 0x03), T(0x289, 0x03), T(0x2b3, 0x04),
	T(0x2b6, 0x0b), T(0x2b7, 0x0b), T(0x2b8, 0x0b), T(0x2b9, 0x0b),
	T(0x2ba, 0x0b), T(0x2bb, 0x06), T(0x2bc, 0x06), T(0x2bd, 0x06),
	T(0x29e, 0x06), T(0x2e4, 0x1a), T(0x2ed, 0x25), T(0x269, 0x1a),
	T(0x2f4, 0x2f), T(0x34b, 0x01), T(0x34c, 0x23), T(0x34d, 0x23),
	T(0x34e, 0x45), T(0x34f, 0x00), T(0x350, 0x31), T(0x351, 0x00),
	T(0x352, 0x02), T(0x353, 0x00), T(0x354, 0x01),
	C(0x43, 0x18), C(0x43, 0x00),
};

struct gs101_ufs {
	struct ufs_hba *hba;
	void __iomem *hci;
	void __iomem *unipro;
	void __iomem *pma;
	void __iomem *sysreg;
	u32 avail_ln;
};

static inline void hci_writel(struct gs101_ufs *ufs, u32 val, u32 reg)
{
	writel(val, ufs->hci + reg);
}

static inline u32 hci_readl(struct gs101_ufs *ufs, u32 reg)
{
	return readl(ufs->hci + reg);
}

static long gs101_calc_time_cntr(long period)
{
	const int precise = 10;
	long pclk = GS101_UFS_PCLK_RATE;
	long clk_period = 1000000000L / pclk;
	long fraction = ((1000000000L % pclk) * precise) / pclk;

	return (period * precise) / ((clk_period * precise) + fraction);
}

static int gs101_phy_calibrate_init(struct gs101_ufs *ufs)
{
	u32 val;
	int i, lane, err;

	for (i = 0; i < ARRAY_SIZE(gs101_phy_pre_init); i++) {
		const struct gs101_phy_cfg *c = &gs101_phy_pre_init[i];

		for (lane = 0; lane < ufs->avail_ln; lane++) {
			if (!c->trsv) {
				if (lane == 0)
					writel(c->val, ufs->pma + PHY_APB_ADDR(c->off));
				continue;
			}
			writel(c->val, ufs->pma + PHY_TRSV_ADDR(c->off, lane));
		}
	}

	for (lane = 0; lane < ufs->avail_ln; lane++) {
		err = readl_poll_timeout(ufs->pma + PHY_TRSV_ADDR(TRSV_REG338, lane),
					 val, val & LN0_MON_RX_CAL_DONE, 40000);
		if (err) {
			dev_err(ufs->hba->dev, "phy lane %d cal timeout\n", lane);
			return err;
		}
	}

	return 0;
}

static void gs101_ufs_host_reset(struct gs101_ufs *ufs)
{
	u32 misc = hci_readl(ufs, HCI_MISC);
	int i;

	hci_writel(ufs, misc & ~HCI_CORECLK_CTRL_EN, HCI_MISC);
	hci_writel(ufs, UFS_SW_RST_MASK, HCI_SW_RST);
	for (i = 0; i < 1000; i++) {
		if (!(hci_readl(ufs, HCI_SW_RST) & UFS_SW_RST_MASK))
			break;
		udelay(1);
	}
	if (i == 1000)
		dev_err(ufs->hba->dev, "timeout host sw-reset\n");
	hci_writel(ufs, misc, HCI_MISC);
}

static int gs101_ufs_device_reset(struct ufs_hba *hba)
{
	struct gs101_ufs *ufs = dev_get_priv(hba->dev);

	hci_writel(ufs, 0, HCI_GPIO_OUT);
	udelay(5);
	hci_writel(ufs, 1, HCI_GPIO_OUT);
	/* the first link startup after a reset found no device without this */
	mdelay(20);
	return 0;
}

static int gs101_ufs_hce_enable_notify(struct ufs_hba *hba,
				       enum ufs_notify_change_status status)
{
	struct gs101_ufs *ufs = dev_get_priv(hba->dev);

	if (status == PRE_CHANGE) {
		gs101_ufs_host_reset(ufs);
		gs101_ufs_device_reset(hba);
	} else {
		hci_writel(ufs, hci_readl(ufs, HCI_MISC) | HCI_CORECLK_CTRL_EN,
			   HCI_MISC);
	}
	return 0;
}

static void gs101_ufs_pre_link(struct gs101_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	u32 mclk = GS101_UFS_MCLK_RATE;
	u32 clk_prd = DIV_ROUND_UP(1000000000UL, mclk);
	u32 rx_reset = (RX_LINE_RESET_TIME * (u64)mclk) / 1000000;
	u32 tx_reset = (TX_LINE_RESET_TIME * (u64)mclk) / 1000000;
	u32 ctrl, misc;
	int i;

	hci_writel(ufs, DFES_ERR_EN | DFES_DEF_L2_ERRS, HCI_ERR_EN_DL_LAYER);
	hci_writel(ufs, DFES_ERR_EN | DFES_DEF_L3_ERRS, HCI_ERR_EN_N_LAYER);
	hci_writel(ufs, DFES_ERR_EN | DFES_DEF_L4_ERRS, HCI_ERR_EN_T_LAYER);

	ctrl = hci_readl(ufs, HCI_CLKSTOP_CTRL);
	misc = hci_readl(ufs, HCI_MISC);
	hci_writel(ufs, ctrl & ~CLK_STOP_MASK, HCI_CLKSTOP_CTRL);
	hci_writel(ufs, misc & ~CLK_CTRL_EN_MASK, HCI_MISC);

	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_TXTRAILINGCLOCKS), 0xff);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_GS101_DBG_OPTION_SUITE1), 0x90913c1c);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_GS101_DBG_OPTION_SUITE2), 0xe01c115f);

	writel(16 * 1000 * 1000000UL / mclk, ufs->unipro + COMP_CLK_PERIOD);

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x40);
	for (i = 0; i < ufs->avail_ln; i++) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_CLK_PRD, i), clk_prd);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_CLK_PRD_EN, i), 0x0);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE2, i),
			       (rx_reset >> 16) & 0xff);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE1, i),
			       (rx_reset >> 8) & 0xff);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE0, i),
			       rx_reset & 0xff);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x2f, i), 0x69);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x84, i), 0x1);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x25, i), 0xf6);
	}
	for (i = 0; i < ufs->avail_ln; i++) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_CLK_PRD, i), clk_prd);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_CLK_PRD_EN, i), 0x02);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE2, i),
			       (tx_reset >> 16) & 0xff);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE1, i),
			       (tx_reset >> 8) & 0xff);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE0, i),
			       tx_reset & 0xff);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x04, i), 1);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x7f, i), 0);
	}
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_LOCAL_TX_LCC_ENABLE), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(N_DEVICEID), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(N_DEVICEID_VALID), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_PEERDEVICEID), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_CONNECTIONSTATE), CPORT_CONNECTED);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0xa006), 0x8000);
}

static void gs101_ufs_post_link(struct gs101_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	u32 nutrs, nutmrs, cap;

	ufshcd_dme_set(hba, UIC_ARG_MIB(T_CONNECTIONSTATE), CPORT_IDLE);
	ufshcd_dme_set(hba, UIC_ARG_MIB(N_DEVICEID), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(N_DEVICEID_VALID), 1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_PEERDEVICEID), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_PEERCPORTID), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_CPORTFLAGS), CPORT_DEF_FLAGS);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_TRAFFICCLASS), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_CONNECTIONSTATE), CPORT_CONNECTED);

	hci_writel(ufs, hci_readl(ufs, HCI_V2P1_CTRL) | IA_TICK_SEL,
		   HCI_V2P1_CTRL);
	hci_writel(ufs, gs101_calc_time_cntr(IATOVAL_NSEC / CNTR_DIV_VAL) &
		   CNT_VAL_1US_MASK, HCI_1US_TO_CNT_VAL);

	cap = readl(hba->mmio_base + REG_CONTROLLER_CAPABILITIES);
	nutrs = (cap & 0x1f) + 1;
	nutmrs = ((cap >> 16) & 0x7) + 1;

	hci_writel(ufs, 0xa, HCI_DATA_REORDER);
	hci_writel(ufs, DATA_UNIT_SHIFT, HCI_TXPRDT_ENTRY_SIZE);
	hci_writel(ufs, DATA_UNIT_SHIFT, HCI_RXPRDT_ENTRY_SIZE);
	hci_writel(ufs, BIT(nutrs) - 1, HCI_UTRL_NEXUS_TYPE);
	hci_writel(ufs, BIT(nutmrs) - 1, HCI_UTMRL_NEXUS_TYPE);
	hci_writel(ufs, 0xf, HCI_AXIDMA_RWDATA_BURST_LEN);

	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_MODE), 1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_GRANULARITY), PA_GRANULARITY_VAL);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_MODE), 0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_TACTIVATE), PA_TACTIVATE_VAL);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_HIBERN8TIME), PA_HIBERN8TIME_VAL);

	hci_writel(ufs, WLU_EN | WLU_BURST_LEN(3), HCI_AXIDMA_RWDATA_BURST_LEN);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_MODE), 1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_SAVECONFIGTIME), 0x3e8);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_MODE), 0);
}

static int gs101_ufs_link_startup_notify(struct ufs_hba *hba,
					 enum ufs_notify_change_status status)
{
	struct gs101_ufs *ufs = dev_get_priv(hba->dev);
	u32 rx = 0, tx = 0;
	int err;

	if (status == POST_CHANGE) {
		gs101_ufs_post_link(ufs);
		return 0;
	}

	ufshcd_dme_get(hba, UIC_ARG_MIB(PA_AVAILRXDATALANES), &rx);
	ufshcd_dme_get(hba, UIC_ARG_MIB(PA_AVAILTXDATALANES), &tx);
	ufs->avail_ln = min(rx, tx);
	if (!ufs->avail_ln || ufs->avail_ln > 2)
		ufs->avail_ln = 2;

	gs101_ufs_pre_link(ufs);

	err = gs101_phy_calibrate_init(ufs);
	if (err)
		dev_err(hba->dev, "phy calibration failed %d\n", err);
	return err;
}

static void gs101_ufs_setup_xfer_req(struct ufs_hba *hba, int tag,
				    bool is_scsi_cmd)
{
	struct gs101_ufs *ufs = dev_get_priv(hba->dev);
	u32 type = hci_readl(ufs, HCI_UTRL_NEXUS_TYPE);

	if (is_scsi_cmd)
		hci_writel(ufs, type | BIT(tag), HCI_UTRL_NEXUS_TYPE);
	else
		hci_writel(ufs, type & ~BIT(tag), HCI_UTRL_NEXUS_TYPE);
}

static int gs101_ufs_get_max_pwr_mode(struct ufs_hba *hba,
				      struct ufs_pwr_mode_info *max)
{
	struct gs101_ufs *ufs = dev_get_priv(hba->dev);

	max->info.pwr_rx = SLOW_MODE;
	max->info.pwr_tx = SLOW_MODE;
	max->info.gear_rx = 1;
	max->info.gear_tx = 1;

	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA0), 12000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA1), 32000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA2), 16000);
	writel(8064, ufs->unipro + UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER0);
	writel(28224, ufs->unipro + UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER1);
	writel(20160, ufs->unipro + UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER2);
	writel(12000, ufs->unipro + UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER0);
	writel(32000, ufs->unipro + UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER1);
	writel(16000, ufs->unipro + UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER2);
	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_FC0PROTTIMEOUTVAL), 8064);
	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_TC0REPLAYTIMEOUTVAL), 28224);
	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_AFC0REQTIMEOUTVAL), 20160);
	return 0;
}

static void __iomem *gs101_phandle_addr(struct udevice *dev, const char *prop)
{
	struct ofnode_phandle_args args;

	if (dev_read_phandle_with_args(dev, prop, NULL, 0, 0, &args))
		return NULL;
	return (void __iomem *)ofnode_get_addr(args.node);
}

static int gs101_ufs_init(struct ufs_hba *hba)
{
	struct udevice *dev = hba->dev;
	struct gs101_ufs *ufs = dev_get_priv(dev);
	struct ofnode_phandle_args args;
	u32 reg;

	ufs->hba = hba;
	ufs->hci = dev_read_addr_name_ptr(dev, "vs_hci");
	ufs->unipro = dev_read_addr_name_ptr(dev, "unipro");
	if (!ufs->hci || !ufs->unipro)
		return -EINVAL;

	if (dev_read_phandle_with_args(dev, "phys", "#phy-cells", 0, 0, &args))
		return -EINVAL;
	ufs->pma = (void __iomem *)ofnode_get_addr(args.node);

	ufs->sysreg = gs101_phandle_addr(dev, "samsung,sysreg");
	if (ufs->sysreg)
		clrbits_le32(ufs->sysreg + UFS_SHAREABILITY_OFFSET,
			     UFS_GS101_SHARABLE);

	reg = hci_readl(ufs, HCI_IOP_ACG_DISABLE);
	hci_writel(ufs, reg & ~HCI_IOP_ACG_DISABLE_EN, HCI_IOP_ACG_DISABLE);

	hba->quirks = UFSHCD_QUIRK_PRDT_BYTE_GRAN |
		      UFSHCI_QUIRK_SKIP_RESET_INTR_AGGR |
		      UFSHCI_QUIRK_BROKEN_REQ_LIST_CLR |
		      UFSHCD_QUIRK_BROKEN_OCS_FATAL_ERROR |
		      UFSHCI_QUIRK_SKIP_MANUAL_WB_FLUSH_CTRL |
		      UFSHCD_QUIRK_SKIP_DEF_UNIPRO_TIMEOUT_SETTING;
	return 0;
}

static struct ufs_hba_ops gs101_ufs_ops = {
	.init = gs101_ufs_init,
	.get_max_pwr_mode = gs101_ufs_get_max_pwr_mode,
	.hce_enable_notify = gs101_ufs_hce_enable_notify,
	.link_startup_notify = gs101_ufs_link_startup_notify,
	.device_reset = gs101_ufs_device_reset,
	.setup_xfer_req = gs101_ufs_setup_xfer_req,
};

static int gs101_ufs_probe(struct udevice *dev)
{
	int err = ufshcd_probe(dev, &gs101_ufs_ops);

	if (err)
		dev_err(dev, "ufshcd_probe() failed %d\n", err);
	return err;
}

static const struct udevice_id gs101_ufs_ids[] = {
	{ .compatible = "google,gs101-ufs" },
	{},
};

U_BOOT_DRIVER(gs101_ufs) = {
	.name		= "ufshcd-gs101",
	.id		= UCLASS_UFS,
	.of_match	= gs101_ufs_ids,
	.probe		= gs101_ufs_probe,
	.priv_auto	= sizeof(struct gs101_ufs),
};
