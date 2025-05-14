/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved. */

#ifndef	_DWMAC_QCOM_ETHQOS_H
#define	_DWMAC_QCOM_ETHQOS_H

#include "stmmac.h"

#define DRV_NAME "qcom-ethqos"
#define ETHQOSDBG(fmt, args...) \
	pr_debug(DRV_NAME " %s:%d " fmt, __func__, __LINE__, ## args)
#define ETHQOSERR(fmt, args...) \
	pr_err(DRV_NAME " %s:%d " fmt, __func__, __LINE__, ## args)
#define ETHQOSINFO(fmt, args...) \
	pr_info(DRV_NAME " %s:%d " fmt, __func__, __LINE__, ## args)

#define RGMII_IO_MACRO_CONFIG		0x0
#define SDCC_HC_REG_DLL_CONFIG		0x4
#define SDCC_TEST_CTL			0x8
#define SDCC_HC_REG_DDR_CONFIG		0xC
#define SDCC_HC_REG_DLL_CONFIG2		0x10
#define SDC4_STATUS			0x14
#define SDCC_USR_CTL			0x18
#define RGMII_IO_MACRO_CONFIG2		0x1C
#define RGMII_IO_MACRO_DEBUG1		0x20

struct ethqos_emac_por {
	unsigned int offset;
	unsigned int value;
};

struct ethqos_emac_driver_data {
	const struct ethqos_emac_por *por;
	unsigned int num_por;
	bool rgmii_config_loopback_en;
	bool has_emac_ge_3;
	const char *link_clk_name;
	bool has_integrated_pcs;
	u32 dma_addr_width;
	struct dwmac4_addrs dwmac4_addrs;
	bool needs_sgmii_loopback;
};

struct qcom_ethqos {
	struct platform_device *pdev;
	void __iomem *rgmii_base;
	void __iomem *mac_base;
	int (*configure_func)(struct qcom_ethqos *ethqos);

	unsigned int link_clk_rate;
	struct clk *link_clk;
	struct phy *serdes_phy;
	unsigned int speed;
	phy_interface_t phy_mode;

	const struct ethqos_emac_por *por;
	unsigned int num_por;
	unsigned int emac_ver;
	bool rgmii_config_loopback_en;
	bool has_emac_ge_3;
	bool needs_sgmii_loopback;

	struct regulator *gdsc_emac;
	struct regulator *reg_rgmii;
	struct regulator *reg_emac_phy;
	struct regulator *reg_rgmii_io_pads;
};

int ethqos_init_regulators(struct qcom_ethqos *ethqos);
void ethqos_disable_regulators(struct qcom_ethqos *ethqos);
#endif
