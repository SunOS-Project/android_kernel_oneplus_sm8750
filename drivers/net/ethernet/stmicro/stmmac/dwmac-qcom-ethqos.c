// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2018-19, Linaro Limited
/* Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries. */

#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/ipc_logging.h>
#include <linux/micrel_phy.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>
#include <linux/phy.h>
#include <linux/phy/phy.h>
#include <linux/poll.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include "dwmac-qcom-ethqos.h"
#include "stmmac_platform.h"
#include "stmmac_ptp.h"

#define EMAC_SYSTEM_LOW_POWER_DEBUG	0x28
#define EMAC_WRAPPER_SGMII_PHY_CNTRL1	0xf4

/* RGMII_IO_MACRO_CONFIG fields */
#define RGMII_CONFIG_FUNC_CLK_EN		BIT(30)
#define RGMII_CONFIG_POS_NEG_DATA_SEL		BIT(23)
#define RGMII_CONFIG_GPIO_CFG_RX_INT		GENMASK(21, 20)
#define RGMII_CONFIG_GPIO_CFG_TX_INT		GENMASK(19, 17)
#define RGMII_CONFIG_MAX_SPD_PRG_9		GENMASK(16, 8)
#define RGMII_CONFIG_MAX_SPD_PRG_2		GENMASK(7, 6)
#define RGMII_CONFIG_INTF_SEL			GENMASK(5, 4)
#define RGMII_CONFIG_BYPASS_TX_ID_EN		BIT(3)
#define RGMII_CONFIG_LOOPBACK_EN		BIT(2)
#define RGMII_CONFIG_PROG_SWAP			BIT(1)
#define RGMII_CONFIG_DDR_MODE			BIT(0)
#define RGMII_CONFIG_SGMII_CLK_DVDR		GENMASK(18, 10)

/* SDCC_HC_REG_DLL_CONFIG fields */
#define SDCC_DLL_CONFIG_DLL_RST			BIT(30)
#define SDCC_DLL_CONFIG_PDN			BIT(29)
#define SDCC_DLL_CONFIG_MCLK_FREQ		GENMASK(26, 24)
#define SDCC_DLL_CONFIG_CDR_SELEXT		GENMASK(23, 20)
#define SDCC_DLL_CONFIG_CDR_EXT_EN		BIT(19)
#define SDCC_DLL_CONFIG_CK_OUT_EN		BIT(18)
#define SDCC_DLL_CONFIG_CDR_EN			BIT(17)
#define SDCC_DLL_CONFIG_DLL_EN			BIT(16)
#define SDCC_DLL_MCLK_GATING_EN			BIT(5)
#define SDCC_DLL_CDR_FINE_PHASE			GENMASK(3, 2)

/* SDCC_HC_REG_DDR_CONFIG fields */
#define SDCC_DDR_CONFIG_PRG_DLY_EN		BIT(31)
#define SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY	GENMASK(26, 21)
#define SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_CODE	GENMASK(29, 27)
#define SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_EN	BIT(30)
#define SDCC_DDR_CONFIG_TCXO_CYCLES_CNT		GENMASK(11, 9)
#define SDCC_DDR_CONFIG_PRG_RCLK_DLY		GENMASK(8, 0)

/* SDCC_HC_REG_DLL_CONFIG2 fields */
#define SDCC_DLL_CONFIG2_DLL_CLOCK_DIS		BIT(21)
#define SDCC_DLL_CONFIG2_MCLK_FREQ_CALC		GENMASK(17, 10)
#define SDCC_DLL_CONFIG2_DDR_TRAFFIC_INIT_SEL	GENMASK(3, 2)
#define SDCC_DLL_CONFIG2_DDR_TRAFFIC_INIT_SW	BIT(1)
#define SDCC_DLL_CONFIG2_DDR_CAL_EN		BIT(0)

/* SDC4_STATUS bits */
#define SDC4_STATUS_DLL_LOCK			BIT(7)

/* RGMII_IO_MACRO_CONFIG2 fields */
#define RGMII_CONFIG2_RSVD_CONFIG15		GENMASK(31, 17)
#define RGMII_CONFIG2_RGMII_CLK_SEL_CFG		BIT(16)
#define RGMII_CONFIG2_TX_TO_RX_LOOPBACK_EN	BIT(13)
#define RGMII_CONFIG2_CLK_DIVIDE_SEL		BIT(12)
#define RGMII_CONFIG2_RX_PROG_SWAP		BIT(7)
#define RGMII_CONFIG2_DATA_DIVIDE_CLK_SEL	BIT(6)
#define RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN	BIT(5)

/* MAC_CTRL_REG bits */
#define ETHQOS_MAC_CTRL_SPEED_MODE		BIT(14)
#define ETHQOS_MAC_CTRL_PORT_SEL		BIT(15)

/* EMAC_WRAPPER_SGMII_PHY_CNTRL1 bits */
#define SGMII_PHY_CNTRL1_SGMII_TX_TO_RX_LOOPBACK_EN	BIT(3)

#define SGMII_10M_RX_CLK_DVDR			0x31

#define EMAC_I0_EMAC_CORE_HW_VERSION_RGOFFADDR 0x00000070
#define EMAC_HW_v2_3_2_RG 0x20030002

#define MII_BUSY 0x00000001
#define MII_WRITE 0x00000002

/* GMAC4 defines */
#define MII_GMAC4_GOC_SHIFT		2
#define MII_GMAC4_WRITE			BIT(MII_GMAC4_GOC_SHIFT)
#define MII_GMAC4_READ			(3 << MII_GMAC4_GOC_SHIFT)

#define MII_BUSY 0x00000001
#define MII_WRITE 0x00000002

#define DWC_ETH_QOS_PHY_INTR_STATUS     0x0013

#define LINK_UP 1
#define LINK_DOWN 0

#define LINK_DOWN_STATE 0x800
#define LINK_UP_STATE 0x400

#define MICREL_PHY_ID PHY_ID_KSZ9031
#define DWC_ETH_QOS_MICREL_PHY_INTCS 0x1b
#define DWC_ETH_QOS_MICREL_PHY_CTL 0x1f
#define DWC_ETH_QOS_MICREL_INTR_LEVEL 0x4000
#define DWC_ETH_QOS_BASIC_STATUS     0x0001
#define LINK_STATE_MASK 0x4
#define AUTONEG_STATE_MASK 0x20
#define MICREL_LINK_UP_INTR_STATUS BIT(0)

struct emac_emb_smmu_cb_ctx emac_emb_smmu_ctx = {0};
struct plat_stmmacenet_data *plat_dat;

inline void *qcom_ethqos_get_priv(struct qcom_ethqos *ethqos)
{
	struct platform_device *pdev = ethqos->pdev;
	struct net_device *dev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(dev);

	return priv;
}

static int rgmii_readl(struct qcom_ethqos *ethqos, unsigned int offset)
{
	return readl(ethqos->rgmii_base + offset);
}

static void rgmii_writel(struct qcom_ethqos *ethqos,
			 int value, unsigned int offset)
{
	writel(value, ethqos->rgmii_base + offset);
}

static void rgmii_updatel(struct qcom_ethqos *ethqos,
			  int mask, int val, unsigned int offset)
{
	unsigned int temp;

	temp = rgmii_readl(ethqos, offset);
	temp = (temp & ~(mask)) | val;
	rgmii_writel(ethqos, temp, offset);
}

static void rgmii_dump(void *priv)
{
	struct qcom_ethqos *ethqos = priv;
	struct device *dev = &ethqos->pdev->dev;

	dev_dbg(dev, "Rgmii register dump\n");
	dev_dbg(dev, "RGMII_IO_MACRO_CONFIG: %x\n",
		rgmii_readl(ethqos, RGMII_IO_MACRO_CONFIG));
	dev_dbg(dev, "SDCC_HC_REG_DLL_CONFIG: %x\n",
		rgmii_readl(ethqos, SDCC_HC_REG_DLL_CONFIG));
	dev_dbg(dev, "SDCC_HC_REG_DDR_CONFIG: %x\n",
		rgmii_readl(ethqos, SDCC_HC_REG_DDR_CONFIG));
	dev_dbg(dev, "SDCC_HC_REG_DLL_CONFIG2: %x\n",
		rgmii_readl(ethqos, SDCC_HC_REG_DLL_CONFIG2));
	dev_dbg(dev, "SDC4_STATUS: %x\n",
		rgmii_readl(ethqos, SDC4_STATUS));
	dev_dbg(dev, "SDCC_USR_CTL: %x\n",
		rgmii_readl(ethqos, SDCC_USR_CTL));
	dev_dbg(dev, "RGMII_IO_MACRO_CONFIG2: %x\n",
		rgmii_readl(ethqos, RGMII_IO_MACRO_CONFIG2));
	dev_dbg(dev, "RGMII_IO_MACRO_DEBUG1: %x\n",
		rgmii_readl(ethqos, RGMII_IO_MACRO_DEBUG1));
	dev_dbg(dev, "EMAC_SYSTEM_LOW_POWER_DEBUG: %x\n",
		rgmii_readl(ethqos, EMAC_SYSTEM_LOW_POWER_DEBUG));
}

/* Clock rates */
#define RGMII_1000_NOM_CLK_FREQ			(250 * 1000 * 1000UL)
#define RGMII_ID_MODE_100_LOW_SVS_CLK_FREQ	 (50 * 1000 * 1000UL)
#define RGMII_ID_MODE_10_LOW_SVS_CLK_FREQ	  (5 * 1000 * 1000UL)

static void
ethqos_update_link_clk(struct qcom_ethqos *ethqos, unsigned int speed)
{
	switch (speed) {
	case SPEED_1000:
		ethqos->link_clk_rate =  RGMII_1000_NOM_CLK_FREQ;
		break;

	case SPEED_100:
		ethqos->link_clk_rate =  RGMII_ID_MODE_100_LOW_SVS_CLK_FREQ;
		break;

	case SPEED_10:
		ethqos->link_clk_rate =  RGMII_ID_MODE_10_LOW_SVS_CLK_FREQ;
		break;
	}

	clk_set_rate(ethqos->link_clk, ethqos->link_clk_rate);
}

static void
qcom_ethqos_set_sgmii_loopback(struct qcom_ethqos *ethqos, bool enable)
{
	if (!ethqos->needs_sgmii_loopback ||
	    ethqos->phy_mode != PHY_INTERFACE_MODE_2500BASEX)
		return;

	rgmii_updatel(ethqos,
		      SGMII_PHY_CNTRL1_SGMII_TX_TO_RX_LOOPBACK_EN,
		      enable ? SGMII_PHY_CNTRL1_SGMII_TX_TO_RX_LOOPBACK_EN : 0,
		      EMAC_WRAPPER_SGMII_PHY_CNTRL1);
}

static void ethqos_set_func_clk_en(struct qcom_ethqos *ethqos)
{
	qcom_ethqos_set_sgmii_loopback(ethqos, true);
	rgmii_updatel(ethqos, RGMII_CONFIG_FUNC_CLK_EN,
		      RGMII_CONFIG_FUNC_CLK_EN, RGMII_IO_MACRO_CONFIG);
}

static const struct ethqos_emac_por emac_v2_3_0_por[] = {
	{ .offset = RGMII_IO_MACRO_CONFIG,	.value = 0x00C01343 },
	{ .offset = SDCC_HC_REG_DLL_CONFIG,	.value = 0x2004642C },
	{ .offset = SDCC_HC_REG_DDR_CONFIG,	.value = 0x00000000 },
	{ .offset = SDCC_HC_REG_DLL_CONFIG2,	.value = 0x00200000 },
	{ .offset = SDCC_USR_CTL,		.value = 0x00010800 },
	{ .offset = RGMII_IO_MACRO_CONFIG2,	.value = 0x00002060 },
};

static const struct ethqos_emac_driver_data emac_v2_3_0_data = {
	.por = emac_v2_3_0_por,
	.num_por = ARRAY_SIZE(emac_v2_3_0_por),
	.rgmii_config_loopback_en = true,
	.has_emac_ge_3 = false,
};

static const struct ethqos_emac_por emac_v2_1_0_por[] = {
	{ .offset = RGMII_IO_MACRO_CONFIG,	.value = 0x40C01343 },
	{ .offset = SDCC_HC_REG_DLL_CONFIG,	.value = 0x2004642C },
	{ .offset = SDCC_HC_REG_DDR_CONFIG,	.value = 0x00000000 },
	{ .offset = SDCC_HC_REG_DLL_CONFIG2,	.value = 0x00200000 },
	{ .offset = SDCC_USR_CTL,		.value = 0x00010800 },
	{ .offset = RGMII_IO_MACRO_CONFIG2,	.value = 0x00002060 },
};

static const struct ethqos_emac_driver_data emac_v2_1_0_data = {
	.por = emac_v2_1_0_por,
	.num_por = ARRAY_SIZE(emac_v2_1_0_por),
	.rgmii_config_loopback_en = false,
	.has_emac_ge_3 = false,
};

static const struct ethqos_emac_por emac_v3_0_0_por[] = {
	{ .offset = RGMII_IO_MACRO_CONFIG,	.value = 0x40c01343 },
	{ .offset = SDCC_HC_REG_DLL_CONFIG,	.value = 0x2004642c },
	{ .offset = SDCC_HC_REG_DDR_CONFIG,	.value = 0x80040800 },
	{ .offset = SDCC_HC_REG_DLL_CONFIG2,	.value = 0x00200000 },
	{ .offset = SDCC_USR_CTL,		.value = 0x00010800 },
	{ .offset = RGMII_IO_MACRO_CONFIG2,	.value = 0x00002060 },
};

static const struct ethqos_emac_driver_data emac_v3_0_0_data = {
	.por = emac_v3_0_0_por,
	.num_por = ARRAY_SIZE(emac_v3_0_0_por),
	.rgmii_config_loopback_en = false,
	.has_emac_ge_3 = true,
	.dwmac4_addrs = {
		.dma_chan = 0x00008100,
		.dma_chan_offset = 0x1000,
		.mtl_chan = 0x00008000,
		.mtl_chan_offset = 0x1000,
		.mtl_ets_ctrl = 0x00008010,
		.mtl_ets_ctrl_offset = 0x1000,
		.mtl_txq_weight = 0x00008018,
		.mtl_txq_weight_offset = 0x1000,
		.mtl_send_slp_cred = 0x0000801c,
		.mtl_send_slp_cred_offset = 0x1000,
		.mtl_high_cred = 0x00008020,
		.mtl_high_cred_offset = 0x1000,
		.mtl_low_cred = 0x00008024,
		.mtl_low_cred_offset = 0x1000,
	},
};

static const struct ethqos_emac_por emac_v4_0_0_por[] = {
	{ .offset = RGMII_IO_MACRO_CONFIG,	.value = 0x40c01343 },
	{ .offset = SDCC_HC_REG_DLL_CONFIG,	.value = 0x2004642c },
	{ .offset = SDCC_HC_REG_DDR_CONFIG,	.value = 0x80040800 },
	{ .offset = SDCC_HC_REG_DLL_CONFIG2,	.value = 0x00200000 },
	{ .offset = SDCC_USR_CTL,		.value = 0x00010800 },
	{ .offset = RGMII_IO_MACRO_CONFIG2,	.value = 0x00002060 },
};

static const struct ethqos_emac_driver_data emac_v4_0_0_data = {
	.por = emac_v4_0_0_por,
	.num_por = ARRAY_SIZE(emac_v4_0_0_por),
	.rgmii_config_loopback_en = false,
	.has_emac_ge_3 = true,
	.link_clk_name = "phyaux",
	.has_integrated_pcs = true,
	.needs_sgmii_loopback = true,
	.dma_addr_width = 36,
	.dwmac4_addrs = {
		.dma_chan = 0x00008100,
		.dma_chan_offset = 0x1000,
		.mtl_chan = 0x00008000,
		.mtl_chan_offset = 0x1000,
		.mtl_ets_ctrl = 0x00008010,
		.mtl_ets_ctrl_offset = 0x1000,
		.mtl_txq_weight = 0x00008018,
		.mtl_txq_weight_offset = 0x1000,
		.mtl_send_slp_cred = 0x0000801c,
		.mtl_send_slp_cred_offset = 0x1000,
		.mtl_high_cred = 0x00008020,
		.mtl_high_cred_offset = 0x1000,
		.mtl_low_cred = 0x00008024,
		.mtl_low_cred_offset = 0x1000,
	},
};

static int ethqos_dll_configure(struct qcom_ethqos *ethqos)
{
	struct device *dev = &ethqos->pdev->dev;
	unsigned int val;
	int retry = 1000;

	/* Set CDR_EN */
	if (ethqos->emac_ver == EMAC_HW_v2_3_2_RG ||
	    ethqos->emac_ver == EMAC_HW_v2_1_2)
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG_CDR_EN,
			      0, SDCC_HC_REG_DLL_CONFIG);
	else
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG_CDR_EN,
			      SDCC_DLL_CONFIG_CDR_EN, SDCC_HC_REG_DLL_CONFIG);

	/* Set CDR_EXT_EN */
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG_CDR_EXT_EN,
		      SDCC_DLL_CONFIG_CDR_EXT_EN, SDCC_HC_REG_DLL_CONFIG);

	/* Clear CK_OUT_EN */
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG_CK_OUT_EN,
		      0, SDCC_HC_REG_DLL_CONFIG);

	/* Set DLL_EN */
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG_DLL_EN,
		      SDCC_DLL_CONFIG_DLL_EN, SDCC_HC_REG_DLL_CONFIG);

	if (!ethqos->has_emac_ge_3 ||
	    (ethqos->emac_ver != EMAC_HW_v2_3_2_RG &&
		 ethqos->emac_ver != EMAC_HW_v2_1_2)) {
		rgmii_updatel(ethqos, SDCC_DLL_MCLK_GATING_EN,
			      0, SDCC_HC_REG_DLL_CONFIG);
		rgmii_updatel(ethqos, SDCC_DLL_CDR_FINE_PHASE,
			      0, SDCC_HC_REG_DLL_CONFIG);
	}

	/* Wait for CK_OUT_EN clear */
	do {
		val = rgmii_readl(ethqos, SDCC_HC_REG_DLL_CONFIG);
		val &= SDCC_DLL_CONFIG_CK_OUT_EN;
		if (!val)
			break;
		mdelay(1);
		retry--;
	} while (retry > 0);
	if (!retry)
		dev_err(dev, "Clear CK_OUT_EN timedout\n");

	/* Set CK_OUT_EN */
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG_CK_OUT_EN,
		      SDCC_DLL_CONFIG_CK_OUT_EN, SDCC_HC_REG_DLL_CONFIG);

	/* Wait for CK_OUT_EN set */
	retry = 1000;
	do {
		val = rgmii_readl(ethqos, SDCC_HC_REG_DLL_CONFIG);
		val &= SDCC_DLL_CONFIG_CK_OUT_EN;
		if (val)
			break;
		mdelay(1);
		retry--;
	} while (retry > 0);
	if (!retry)
		dev_err(dev, "Set CK_OUT_EN timedout\n");

	/* Set DDR_CAL_EN */
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG2_DDR_CAL_EN,
		      SDCC_DLL_CONFIG2_DDR_CAL_EN, SDCC_HC_REG_DLL_CONFIG2);

	if (!ethqos->has_emac_ge_3 ||
	    (ethqos->emac_ver != EMAC_HW_v2_3_2_RG &&
	     ethqos->emac_ver != EMAC_HW_v2_1_2)) {
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG2_DLL_CLOCK_DIS,
			      0, SDCC_HC_REG_DLL_CONFIG2);

		rgmii_updatel(ethqos, SDCC_DLL_CONFIG2_MCLK_FREQ_CALC,
			      0x1A << 10, SDCC_HC_REG_DLL_CONFIG2);

		rgmii_updatel(ethqos, SDCC_DLL_CONFIG2_DDR_TRAFFIC_INIT_SEL,
			      BIT(2), SDCC_HC_REG_DLL_CONFIG2);

		rgmii_updatel(ethqos, SDCC_DLL_CONFIG2_DDR_TRAFFIC_INIT_SW,
			      SDCC_DLL_CONFIG2_DDR_TRAFFIC_INIT_SW,
			      SDCC_HC_REG_DLL_CONFIG2);
	}

	return 0;
}

static int ethqos_rgmii_macro_init(struct qcom_ethqos *ethqos)
{
	struct device *dev = &ethqos->pdev->dev;
	int phase_shift;
	int loopback;

	/* Determine if the PHY adds a 2 ns TX delay or the MAC handles it */
	if ((ethqos->phy_mode != PHY_INTERFACE_MODE_RGMII_ID &&
	     ethqos->phy_mode != PHY_INTERFACE_MODE_RGMII_TXID) ||
		ethqos->emac_ver == EMAC_HW_v2_3_2_RG ||
		ethqos->emac_ver == EMAC_HW_v2_1_2)
		phase_shift = RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN;
	else
		phase_shift = 0;

	/* Disable loopback mode */
	rgmii_updatel(ethqos, RGMII_CONFIG2_TX_TO_RX_LOOPBACK_EN,
		      0, RGMII_IO_MACRO_CONFIG2);

	/* Determine if this platform wants loopback enabled after programming */
	if (ethqos->rgmii_config_loopback_en ||
	    (ethqos->emac_ver != EMAC_HW_v2_3_2_RG ||
	     ethqos->emac_ver != EMAC_HW_v2_1_2))
		loopback = RGMII_CONFIG_LOOPBACK_EN;
	else
		loopback = 0;

	/* Disable loopback mode.
	 * Select RGMII, write 0 to interface select.
	 */
	rgmii_updatel(ethqos, RGMII_CONFIG_INTF_SEL,
		      0, RGMII_IO_MACRO_CONFIG);

	switch (ethqos->speed) {
	case SPEED_1000:
		rgmii_updatel(ethqos, RGMII_CONFIG_DDR_MODE,
			      RGMII_CONFIG_DDR_MODE, RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_BYPASS_TX_ID_EN,
			      0, RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_POS_NEG_DATA_SEL,
			      RGMII_CONFIG_POS_NEG_DATA_SEL,
			      RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_PROG_SWAP,
			      RGMII_CONFIG_PROG_SWAP, RGMII_IO_MACRO_CONFIG);

		if (ethqos->emac_ver != EMAC_HW_v2_1_2)
			rgmii_updatel(ethqos, RGMII_CONFIG2_DATA_DIVIDE_CLK_SEL,
				      0, RGMII_IO_MACRO_CONFIG2);

		rgmii_updatel(ethqos, RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN,
			      phase_shift, RGMII_IO_MACRO_CONFIG2);
		rgmii_updatel(ethqos, RGMII_CONFIG2_RSVD_CONFIG15,
			      0, RGMII_IO_MACRO_CONFIG2);
		rgmii_updatel(ethqos, RGMII_CONFIG2_RX_PROG_SWAP,
			      RGMII_CONFIG2_RX_PROG_SWAP,
				RGMII_IO_MACRO_CONFIG2);

		/* PRG_RCLK_DLY = TCXO period * TCXO_CYCLES_CNT / 2 * RX delay ns,
		 * in practice this becomes PRG_RCLK_DLY = 52 * 4 / 2 * RX delay ns
		 */
		if (ethqos->has_emac_ge_3)
			/* 0.9 ns */
			rgmii_updatel(ethqos, SDCC_DDR_CONFIG_PRG_RCLK_DLY,
				      115, SDCC_HC_REG_DDR_CONFIG);
		else if (ethqos->emac_ver == EMAC_HW_v2_3_2_RG)
			rgmii_updatel(ethqos, SDCC_DDR_CONFIG_PRG_RCLK_DLY,
				      69, SDCC_HC_REG_DDR_CONFIG);
		else if (ethqos->emac_ver == EMAC_HW_v2_1_2)
			rgmii_updatel(ethqos, SDCC_DDR_CONFIG_PRG_RCLK_DLY,
				      52, SDCC_HC_REG_DDR_CONFIG);
		else
			/* 1.8 ns */
			rgmii_updatel(ethqos, SDCC_DDR_CONFIG_PRG_RCLK_DLY,
				      57, SDCC_HC_REG_DDR_CONFIG);

		rgmii_updatel(ethqos, SDCC_DDR_CONFIG_PRG_DLY_EN,
			      SDCC_DDR_CONFIG_PRG_DLY_EN,
			      SDCC_HC_REG_DDR_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_LOOPBACK_EN,
			      loopback, RGMII_IO_MACRO_CONFIG);
		break;

	case SPEED_100:
		rgmii_updatel(ethqos, RGMII_CONFIG_DDR_MODE,
			      RGMII_CONFIG_DDR_MODE, RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_BYPASS_TX_ID_EN,
			      RGMII_CONFIG_BYPASS_TX_ID_EN,
			      RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_POS_NEG_DATA_SEL,
			      0, RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_PROG_SWAP,
			      0, RGMII_IO_MACRO_CONFIG);
		if (ethqos->emac_ver != EMAC_HW_v2_1_2)
			rgmii_updatel(ethqos, RGMII_CONFIG2_DATA_DIVIDE_CLK_SEL,
				      0, RGMII_IO_MACRO_CONFIG2);
		rgmii_updatel(ethqos, RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN,
			      phase_shift, RGMII_IO_MACRO_CONFIG2);
		rgmii_updatel(ethqos, RGMII_CONFIG_MAX_SPD_PRG_2,
			      BIT(6), RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG2_RSVD_CONFIG15,
			      0, RGMII_IO_MACRO_CONFIG2);

		if (ethqos->has_emac_ge_3 ||
		    ethqos->emac_ver == EMAC_HW_v2_3_2_RG ||
		    ethqos->emac_ver == EMAC_HW_v2_1_2)
			rgmii_updatel(ethqos, RGMII_CONFIG2_RX_PROG_SWAP,
				      RGMII_CONFIG2_RX_PROG_SWAP,
				      RGMII_IO_MACRO_CONFIG2);
		else
			rgmii_updatel(ethqos, RGMII_CONFIG2_RX_PROG_SWAP,
				      0, RGMII_IO_MACRO_CONFIG2);

		/* Write 0x5 to PRG_RCLK_DLY_CODE */
		rgmii_updatel(ethqos, SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_CODE,
			      (BIT(29) | BIT(27)), SDCC_HC_REG_DDR_CONFIG);
		rgmii_updatel(ethqos, SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY,
			      SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY,
			      SDCC_HC_REG_DDR_CONFIG);
		rgmii_updatel(ethqos, SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_EN,
			      SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_EN,
			      SDCC_HC_REG_DDR_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_LOOPBACK_EN,
			      loopback, RGMII_IO_MACRO_CONFIG);
		break;

	case SPEED_10:
		rgmii_updatel(ethqos, RGMII_CONFIG_DDR_MODE,
			      RGMII_CONFIG_DDR_MODE, RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_BYPASS_TX_ID_EN,
			      RGMII_CONFIG_BYPASS_TX_ID_EN,
			      RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_POS_NEG_DATA_SEL,
			      0, RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_PROG_SWAP,
			      0, RGMII_IO_MACRO_CONFIG);
		if (ethqos->emac_ver != EMAC_HW_v2_1_2)
			rgmii_updatel(ethqos, RGMII_CONFIG2_DATA_DIVIDE_CLK_SEL,
				      0, RGMII_IO_MACRO_CONFIG2);
		rgmii_updatel(ethqos, RGMII_CONFIG2_TX_CLK_PHASE_SHIFT_EN,
			      phase_shift, RGMII_IO_MACRO_CONFIG2);
		rgmii_updatel(ethqos, RGMII_CONFIG_MAX_SPD_PRG_9,
			      BIT(12) | GENMASK(9, 8),
			      RGMII_IO_MACRO_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG2_RSVD_CONFIG15,
			      0, RGMII_IO_MACRO_CONFIG2);
		if (ethqos->has_emac_ge_3 ||
		    ethqos->emac_ver == EMAC_HW_v2_3_2_RG ||
			ethqos->emac_ver == EMAC_HW_v2_1_2)
			rgmii_updatel(ethqos, RGMII_CONFIG2_RX_PROG_SWAP,
				      RGMII_CONFIG2_RX_PROG_SWAP,
				      RGMII_IO_MACRO_CONFIG2);
		else
			rgmii_updatel(ethqos, RGMII_CONFIG2_RX_PROG_SWAP,
				      0, RGMII_IO_MACRO_CONFIG2);
		/* Write 0x5 to PRG_RCLK_DLY_CODE */
		rgmii_updatel(ethqos, SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_CODE,
			      (BIT(29) | BIT(27)), SDCC_HC_REG_DDR_CONFIG);
		rgmii_updatel(ethqos, SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY,
			      SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY,
			      SDCC_HC_REG_DDR_CONFIG);
		rgmii_updatel(ethqos, SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_EN,
			      SDCC_DDR_CONFIG_EXT_PRG_RCLK_DLY_EN,
			      SDCC_HC_REG_DDR_CONFIG);
		rgmii_updatel(ethqos, RGMII_CONFIG_LOOPBACK_EN,
			      loopback, RGMII_IO_MACRO_CONFIG);
		break;
	default:
		dev_err(dev, "Invalid speed %d\n", ethqos->speed);
		return -EINVAL;
	}

	return 0;
}

static int ethqos_configure_rgmii(struct qcom_ethqos *ethqos)
{
	struct device *dev = &ethqos->pdev->dev;
	volatile unsigned int dll_lock;
	unsigned int i, retry = 1000;

	/* Reset to POR values and enable clk */
	for (i = 0; i < ethqos->num_por; i++)
		rgmii_writel(ethqos, ethqos->por[i].value,
			     ethqos->por[i].offset);
	ethqos_set_func_clk_en(ethqos);

	/* Initialize the DLL first */

	/* Set DLL_RST */
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG_DLL_RST,
		      SDCC_DLL_CONFIG_DLL_RST, SDCC_HC_REG_DLL_CONFIG);

	/* Set PDN */
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG_PDN,
		      SDCC_DLL_CONFIG_PDN, SDCC_HC_REG_DLL_CONFIG);

	if (ethqos->has_emac_ge_3) {
		if (ethqos->speed == SPEED_1000) {
			rgmii_writel(ethqos, 0x1800000, SDCC_TEST_CTL);
			rgmii_writel(ethqos, 0x2C010800, SDCC_USR_CTL);
			rgmii_writel(ethqos, 0xA001, SDCC_HC_REG_DLL_CONFIG2);
		} else {
			rgmii_writel(ethqos, 0x40010800, SDCC_USR_CTL);
			rgmii_writel(ethqos, 0xA001, SDCC_HC_REG_DLL_CONFIG2);
		}
	}

	/* Clear DLL_RST */
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG_DLL_RST, 0,
		      SDCC_HC_REG_DLL_CONFIG);

	/* Clear PDN */
	rgmii_updatel(ethqos, SDCC_DLL_CONFIG_PDN, 0,
		      SDCC_HC_REG_DLL_CONFIG);

	if (ethqos->speed != SPEED_100 && ethqos->speed != SPEED_10) {
		/* Set DLL_EN */
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG_DLL_EN,
			      SDCC_DLL_CONFIG_DLL_EN, SDCC_HC_REG_DLL_CONFIG);

		/* Set CK_OUT_EN */
		rgmii_updatel(ethqos, SDCC_DLL_CONFIG_CK_OUT_EN,
			      SDCC_DLL_CONFIG_CK_OUT_EN,
			      SDCC_HC_REG_DLL_CONFIG);

		/* Set USR_CTL bit 26 with mask of 3 bits */
		if (!ethqos->has_emac_ge_3)
			rgmii_updatel(ethqos, GENMASK(26, 24), BIT(26),
				      SDCC_USR_CTL);

		/* wait for DLL LOCK */
		do {
			mdelay(1);
			dll_lock = rgmii_readl(ethqos, SDC4_STATUS);
			if (dll_lock & SDC4_STATUS_DLL_LOCK)
				break;
			retry--;
		} while (retry > 0);
		if (!retry)
			dev_err(dev, "Timeout while waiting for DLL lock\n");
	}

	if (ethqos->speed == SPEED_1000)
		ethqos_dll_configure(ethqos);

	ethqos_rgmii_macro_init(ethqos);

	return 0;
}

/* On interface toggle MAC registers gets reset.
 * Configure MAC block for SGMII on ethernet phy link up
 */
static int ethqos_configure_sgmii(struct qcom_ethqos *ethqos)
{
	int val;

	val = readl(ethqos->mac_base + MAC_CTRL_REG);

	switch (ethqos->speed) {
	case SPEED_1000:
		val &= ~ETHQOS_MAC_CTRL_PORT_SEL;
		rgmii_updatel(ethqos, RGMII_CONFIG2_RGMII_CLK_SEL_CFG,
			      RGMII_CONFIG2_RGMII_CLK_SEL_CFG,
			      RGMII_IO_MACRO_CONFIG2);
		break;
	case SPEED_100:
		val |= ETHQOS_MAC_CTRL_PORT_SEL | ETHQOS_MAC_CTRL_SPEED_MODE;
		break;
	case SPEED_10:
		val |= ETHQOS_MAC_CTRL_PORT_SEL;
		val &= ~ETHQOS_MAC_CTRL_SPEED_MODE;
		rgmii_updatel(ethqos, RGMII_CONFIG_SGMII_CLK_DVDR,
			      FIELD_PREP(RGMII_CONFIG_SGMII_CLK_DVDR,
					 SGMII_10M_RX_CLK_DVDR),
			      RGMII_IO_MACRO_CONFIG);
		break;
	}

	writel(val, ethqos->mac_base + MAC_CTRL_REG);

	return val;
}

static int ethqos_configure(struct qcom_ethqos *ethqos)
{
	return ethqos->configure_func(ethqos);
}

static void ethqos_fix_mac_speed(void *priv, unsigned int speed, unsigned int mode)
{
	struct qcom_ethqos *ethqos = priv;

	qcom_ethqos_set_sgmii_loopback(ethqos, false);
	ethqos->speed = speed;
	ethqos_update_link_clk(ethqos, speed);
	ethqos_configure(ethqos);
}

static int qcom_ethqos_serdes_powerup(struct net_device *ndev, void *priv)
{
	struct qcom_ethqos *ethqos = priv;
	int ret;

	ret = phy_init(ethqos->serdes_phy);
	if (ret)
		return ret;

	ret = phy_power_on(ethqos->serdes_phy);
	if (ret)
		return ret;

	return phy_set_speed(ethqos->serdes_phy, ethqos->speed);
}

static void qcom_ethqos_serdes_powerdown(struct net_device *ndev, void *priv)
{
	struct qcom_ethqos *ethqos = priv;

	phy_power_off(ethqos->serdes_phy);
	phy_exit(ethqos->serdes_phy);
}

static int ethqos_clks_config(void *priv, bool enabled)
{
	struct qcom_ethqos *ethqos = priv;
	int ret = 0;

	if (enabled) {
		ret = clk_prepare_enable(ethqos->link_clk);
		if (ret) {
			dev_err(&ethqos->pdev->dev, "link_clk enable failed\n");
			return ret;
		}

		/* Enable functional clock to prevent DMA reset to timeout due
		 * to lacking PHY clock after the hardware block has been power
		 * cycled. The actual configuration will be adjusted once
		 * ethqos_fix_mac_speed() is invoked.
		 */
		ethqos_set_func_clk_en(ethqos);
	} else {
		clk_disable_unprepare(ethqos->link_clk);
	}

	return ret;
}

static void ethqos_clks_disable(void *data)
{
	ethqos_clks_config(data, false);
}

static void ethqos_ptp_clk_freq_config(struct stmmac_priv *priv)
{
	struct plat_stmmacenet_data *plat_dat = priv->plat;
	int err;

	if (!plat_dat->clk_ptp_ref)
		return;

	/* Max the PTP ref clock out to get the best resolution possible */
	err = clk_set_rate(plat_dat->clk_ptp_ref, ULONG_MAX);
	if (err)
		netdev_err(priv->dev, "Failed to max out clk_ptp_ref: %d\n", err);
	plat_dat->clk_ptp_rate = clk_get_rate(plat_dat->clk_ptp_ref);

	netdev_dbg(priv->dev, "PTP rate %d\n", plat_dat->clk_ptp_rate);
}

static int ethqos_mdio_read(struct stmmac_priv  *priv, int phyaddr, int phyreg)
{
	unsigned int mii_address = priv->hw->mii.addr;
	unsigned int mii_data = priv->hw->mii.data;
	u32 v;
	int data;
	u32 value = MII_BUSY;

	value |= (phyaddr << priv->hw->mii.addr_shift)
		& priv->hw->mii.addr_mask;
	value |= (phyreg << priv->hw->mii.reg_shift) & priv->hw->mii.reg_mask;
	value |= (priv->clk_csr << priv->hw->mii.clk_csr_shift)
		& priv->hw->mii.clk_csr_mask;
	if (priv->plat->has_gmac4)
		value |= MII_GMAC4_READ;

	if (readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
			       100, 10000))
		return -EBUSY;

	writel_relaxed(value, priv->ioaddr + mii_address);

	if (readl_poll_timeout(priv->ioaddr + mii_address, v, !(v & MII_BUSY),
			       100, 10000))
		return -EBUSY;

	/* Read the data from the MII data register */
	data = (int)readl_relaxed(priv->ioaddr + mii_data);

	return data;
}

static int ethqos_phy_intr_config(struct qcom_ethqos *ethqos)
{
	int ret = 0;

	ethqos->phy_intr = platform_get_irq_byname(ethqos->pdev, "phy-intr");

	if (ethqos->phy_intr < 0) {
		dev_err(&ethqos->pdev->dev,
			"PHY IRQ configuration information not found\n");
		ret = 1;
	}

	return ret;
}

static void ethqos_handle_phy_interrupt(struct qcom_ethqos *ethqos)
{
	int phy_intr_status = 0;
	struct platform_device *pdev = ethqos->pdev;

	struct net_device *dev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(dev);
	int micrel_intr_status = 0;

	if (priv->phydev && (priv->phydev->phy_id &
	    priv->phydev->drv->phy_id_mask)
	    == MICREL_PHY_ID) {
		phy_intr_status = ethqos_mdio_read(priv,
						   priv->plat->phy_addr,
						   DWC_ETH_QOS_BASIC_STATUS);
		micrel_intr_status = ethqos_mdio_read(priv,
						      priv->plat->phy_addr,
						      DWC_ETH_QOS_MICREL_PHY_INTCS);

		/* Interrupt received for link state change */
		if (phy_intr_status & LINK_STATE_MASK) {
			if (micrel_intr_status & MICREL_LINK_UP_INTR_STATUS)
				phy_mac_interrupt(priv->phydev);
		} else if (!(phy_intr_status & LINK_STATE_MASK)) {
			phy_mac_interrupt(priv->phydev);
		} else if (!(phy_intr_status & AUTONEG_STATE_MASK)) {
			/* NOTE: Nothing to be done for now.
			 *
			 * TODO: add required logic to handle
			 * AUTONEG_STATE change.
			 */
		}
	} else {
		phy_intr_status =
			ethqos_mdio_read(priv, priv->plat->phy_addr,
					 DWC_ETH_QOS_PHY_INTR_STATUS);

		if (phy_intr_status & LINK_UP_STATE)
			phylink_mac_change(priv->phylink, LINK_UP);
		else if (phy_intr_status & LINK_DOWN_STATE)
			phylink_mac_change(priv->phylink, LINK_DOWN);
	}
}

static void ethqos_defer_phy_isr_work(struct work_struct *work)
{
	struct qcom_ethqos *ethqos =
		container_of(work, struct qcom_ethqos, emac_phy_work);

	ethqos_handle_phy_interrupt(ethqos);
}

static irqreturn_t ethqos_phy_isr(int irq, void *dev_data)
{
	struct qcom_ethqos *ethqos = (struct qcom_ethqos *)dev_data;

	queue_work(system_wq, &ethqos->emac_phy_work);
	return IRQ_HANDLED;
}

static int ethqos_phy_intr_enable(struct qcom_ethqos *ethqos)
{
	int ret = 0;
	struct stmmac_priv *priv = qcom_ethqos_get_priv(ethqos);

	INIT_WORK(&ethqos->emac_phy_work, ethqos_defer_phy_isr_work);
	ret = request_irq(ethqos->phy_intr, ethqos_phy_isr,
			  IRQF_SHARED, "stmmac", ethqos);
	if (ret) {
		ETHQOSERR("Unable to register PHY IRQ %d\n",
			  ethqos->phy_intr);
		return ret;
	}

	priv->plat->phy_intr_en_extn_stm = true;

	return ret;
}

static void emac_emb_smmu_exit(void)
{
	emac_emb_smmu_ctx.valid = false;
	emac_emb_smmu_ctx.pdev_master = NULL;
	emac_emb_smmu_ctx.smmu_pdev = NULL;
	emac_emb_smmu_ctx.iommu_domain = NULL;
}

static int emac_emb_smmu_cb_probe(struct platform_device *pdev,
				  struct plat_stmmacenet_data *plat_dat)
{
	int result = 0;
	u32 iova_ap_mapping[2];
	struct device *dev = &pdev->dev;

	ETHQOSDBG("EMAC EMB SMMU CB probe: smmu pdev=%p\n", pdev);

	result = of_property_read_u32_array(dev->of_node,
					    "qcom,iommu-dma-addr-pool",
					    iova_ap_mapping,
					    ARRAY_SIZE(iova_ap_mapping));
	if (result) {
		ETHQOSERR("Failed to read EMB start/size iova addresses\n");
		return result;
	}

	emac_emb_smmu_ctx.smmu_pdev = pdev;

	if (dma_set_mask(dev, DMA_BIT_MASK(32)) ||
	    dma_set_coherent_mask(dev, DMA_BIT_MASK(32))) {
		ETHQOSERR("DMA set 32bit mask failed\n");
		return -EOPNOTSUPP;
	}

	emac_emb_smmu_ctx.valid = true;

	emac_emb_smmu_ctx.iommu_domain =
		iommu_get_domain_for_dev(&emac_emb_smmu_ctx.smmu_pdev->dev);

	ETHQOSINFO("Successfully attached to IOMMU\n");
	plat_dat->stmmac_emb_smmu_ctx = emac_emb_smmu_ctx;
	if (emac_emb_smmu_ctx.pdev_master)
		goto smmu_probe_done;

smmu_probe_done:
	emac_emb_smmu_ctx.ret = result;
	return result;
}

static int qcom_ethqos_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct ethqos_emac_driver_data *data;
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct device *dev = &pdev->dev;
	struct qcom_ethqos *ethqos = NULL;

	int ret;

	if (of_device_is_compatible(pdev->dev.of_node,
				    "qcom,emac-smmu-embedded"))
		return emac_emb_smmu_cb_probe(pdev, plat_dat);

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);

	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to get platform resources\n");

	plat_dat = devm_stmmac_probe_config_dt(pdev, stmmac_res.mac);
	if (IS_ERR(plat_dat)) {
		return dev_err_probe(dev, PTR_ERR(plat_dat),
				     "dt configuration failed\n");
	}

	ethqos = devm_kzalloc(&pdev->dev, sizeof(*ethqos), GFP_KERNEL);
	if (!ethqos)
		return -ENOMEM;

	plat_dat->clks_config = ethqos_clks_config;

	ret = of_get_phy_mode(np, &ethqos->phy_mode);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get phy mode\n");

	switch (ethqos->phy_mode) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		ethqos->configure_func = ethqos_configure_rgmii;
		break;
	case PHY_INTERFACE_MODE_SGMII:
		ethqos->configure_func = ethqos_configure_sgmii;
		break;
	default:
		dev_err(dev, "Unsupported phy mode %s\n",
			phy_modes(ethqos->phy_mode));
		return -EINVAL;
	}

	ethqos->pdev = pdev;
	ethqos->rgmii_base = devm_platform_ioremap_resource_byname(pdev, "rgmii");
	if (IS_ERR(ethqos->rgmii_base))
		return dev_err_probe(dev, PTR_ERR(ethqos->rgmii_base),
				     "Failed to map rgmii resource\n");

	ethqos->mac_base = stmmac_res.addr;

	data = of_device_get_match_data(dev);
	ethqos->por = data->por;
	ethqos->num_por = data->num_por;
	ethqos->rgmii_config_loopback_en = data->rgmii_config_loopback_en;
	ethqos->has_emac_ge_3 = data->has_emac_ge_3;
	ethqos->needs_sgmii_loopback = data->needs_sgmii_loopback;

	ethqos->link_clk = devm_clk_get(dev, data->link_clk_name ?: "rgmii");
	if (IS_ERR(ethqos->link_clk))
		return dev_err_probe(dev, PTR_ERR(ethqos->link_clk),
				     "Failed to get link_clk\n");

	ret = ethqos_init_regulators(ethqos);
	if (ret)
		goto err;

	ret = ethqos_init_gpio(ethqos);
	if (ret)
		goto err;

	ret = ethqos_clks_config(ethqos, true);
	if (ret)
		goto err;

	ret = devm_add_action_or_reset(dev, ethqos_clks_disable, ethqos);
	if (ret)
		goto err;

	ethqos->serdes_phy = devm_phy_optional_get(dev, "serdes");
	if (IS_ERR(ethqos->serdes_phy))
		return dev_err_probe(dev, PTR_ERR(ethqos->serdes_phy),
				     "Failed to get serdes phy\n");

	ethqos->speed = SPEED_1000;
	ethqos_update_link_clk(ethqos, SPEED_1000);
	ethqos_set_func_clk_en(ethqos);

	plat_dat->bsp_priv = ethqos;
	plat_dat->fix_mac_speed = ethqos_fix_mac_speed;
	plat_dat->dump_debug_regs = rgmii_dump;
	plat_dat->ptp_clk_freq_config = ethqos_ptp_clk_freq_config;
	plat_dat->has_gmac4 = 1;
	if (ethqos->has_emac_ge_3)
		plat_dat->dwmac4_addrs = &data->dwmac4_addrs;
	/* Set mdio phy addr probe capability to c22 .
	 * If c22_c45 is set then multiple phy is getting detected.
	 */
	if (of_property_read_bool(np, "eth-c22-mdio-probe"))
		plat_dat->has_c22_mdio_probe_capability = 1;
	else
		plat_dat->has_c22_mdio_probe_capability = 0;
	plat_dat->pmt = 1;
	if (of_property_read_bool(np, "snps,tso"))
		plat_dat->flags |= STMMAC_FLAG_TSO_EN;
	if (of_device_is_compatible(np, "qcom,qcs404-ethqos"))
		plat_dat->flags |= STMMAC_FLAG_RX_CLK_RUNS_IN_LPI;
	if (data->has_integrated_pcs)
		plat_dat->flags |= STMMAC_FLAG_HAS_INTEGRATED_PCS;
	if (data->dma_addr_width)
		plat_dat->host_dma_width = data->dma_addr_width;

	if (ethqos->serdes_phy) {
		plat_dat->serdes_powerup = qcom_ethqos_serdes_powerup;
		plat_dat->serdes_powerdown  = qcom_ethqos_serdes_powerdown;
	}

	if (of_property_read_bool(pdev->dev.of_node, "qcom,arm-smmu")) {
		emac_emb_smmu_ctx.pdev_master = pdev;
		ret = of_platform_populate(pdev->dev.of_node,
					   qcom_ethqos_match, NULL, &pdev->dev);
		if (ret)
			ETHQOSERR("Failed to populate EMAC platform\n");
		if (emac_emb_smmu_ctx.ret) {
			ETHQOSERR("smmu probe failed\n");
			of_platform_depopulate(&pdev->dev);
			ret = emac_emb_smmu_ctx.ret;
			emac_emb_smmu_ctx.ret = 0;
		}
	}

	if (of_property_read_bool(pdev->dev.of_node,
				  "emac-core-version")) {
		/* Read emac core version value from dtsi */
		ret = of_property_read_u32(pdev->dev.of_node,
					   "emac-core-version",
					   &ethqos->emac_ver);
		if (ret) {
			ETHQOSDBG(":resource emac-hw-ver! not in dtsi\n");
			ethqos->emac_ver = EMAC_HW_NONE;
			WARN_ON(1);
		}
	} else {
		ethqos->emac_ver =
			rgmii_readl(ethqos, EMAC_I0_EMAC_CORE_HW_VERSION_RGOFFADDR);
	}

	ETHQOSDBG(": emac_core_version = %d\n", ethqos->emac_ver);

	if (!ethqos_phy_intr_config(ethqos)) {
		ret = ethqos_phy_intr_enable(ethqos);
		if (ret)
			ETHQOSERR("ethqos_phy_intr_enable failed\n");
	} else {
		ETHQOSERR("Phy interrupt configuration failed\n");
	}

	return devm_stmmac_pltfr_probe(pdev, plat_dat, &stmmac_res);

err:
	rgmii_dump(ethqos);
	return ret;
}

static int qcom_ethqos_remove(struct platform_device *pdev)
{
	struct qcom_ethqos *ethqos;
	int ret;
	struct stmmac_priv *priv;

	ethqos = get_stmmac_bsp_priv(&pdev->dev);
	if (!ethqos)
		return -ENODEV;

	priv = qcom_ethqos_get_priv(ethqos);
	if (priv->plat->phy_intr_en_extn_stm)
		free_irq(ethqos->phy_intr, ethqos);

	emac_emb_smmu_exit();
	ethqos_disable_regulators(ethqos);
	ethqos_clks_config(ethqos, false);
}

static const struct of_device_id qcom_ethqos_match[] = {
	{ .compatible = "qcom,qcs404-ethqos", .data = &emac_v2_3_0_data},
	{ .compatible = "qcom,sa8775p-ethqos", .data = &emac_v4_0_0_data},
	{ .compatible = "qcom,sc8280xp-ethqos", .data = &emac_v3_0_0_data},
	{ .compatible = "qcom,sm8150-ethqos", .data = &emac_v2_1_0_data},
	{ .compatible = "qcom,stmmac-ethqos", },
	{ .compatible = "qcom,emac-smmu-embedded", },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_ethqos_match);

static struct platform_driver qcom_ethqos_driver = {
	.probe  = qcom_ethqos_probe,
	.remove	= qcom_ethqos_remove,
	.driver = {
		.name           = DRV_NAME,
		.pm		= &stmmac_pltfr_pm_ops,
		.of_match_table = of_match_ptr(qcom_ethqos_match),
	},
};
module_platform_driver(qcom_ethqos_driver);

MODULE_DESCRIPTION("Qualcomm ETHQOS driver");
MODULE_LICENSE("GPL v2");
