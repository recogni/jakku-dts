/*
 * t19x-nvlink-endpt.c:
 * This is the NVLINK endpoint driver for the Tegra NVLINK controller.
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/of.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/of_graph.h>
#include <linux/platform/tegra/mc.h>
#include <linux/platform/tegra/mc-regs-t19x.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/tegra_pm_domains.h>
#include <linux/tegra-powergate.h>

#include "t19x-nvlink-endpt.h"
#include "nvlink-hw.h"

#define PLLNVHS_FREQ_150MHZ	(150 * 1000 * 1000)

/* NVLINK TOM is the top of the NVLINK aperture */
#define NVLINK_TOM_GB	512

/* Convert the NVLINK TOM value from GB to MB for register programming */
#define NVLINK_TOM_MB	(((NVLINK_TOM_GB) * 1024) - 1)

static struct of_device_id t19x_nvlink_controller_of_match[] = {
	{
		.compatible     = "nvidia,t19x-nvlink-controller",
	}, {
	},
};

#if IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS)
static struct of_device_id tegra_nvl_pd[] = {
	{ .compatible = "nvidia,tegra194-nvl-pd", },
	{},
};
#endif

MODULE_DEVICE_TABLE(of, t19x_nvlink_controller_of_match);

u32 nvlw_tioctrl_readl(struct tnvlink_dev *tdev, u32 reg)
{
	return readl(tdev->nvlw_tioctrl_base + reg);
}

void nvlw_tioctrl_writel(struct tnvlink_dev *tdev, u32 reg, u32 val)
{
	writel(val, tdev->nvlw_tioctrl_base + reg);
}

u32 nvlw_nvlipt_readl(struct tnvlink_dev *tdev, u32 reg)
{
	return readl(tdev->nvlw_nvlipt_base + reg);
}

void nvlw_nvlipt_writel(struct tnvlink_dev *tdev, u32 reg, u32 val)
{
	writel(val, tdev->nvlw_nvlipt_base + reg);
}

u32 nvlw_minion_readl(struct tnvlink_dev *tdev, u32 reg)
{
	return readl(tdev->nvlw_minion_base + reg);
}

void nvlw_minion_writel(struct tnvlink_dev *tdev, u32 reg, u32 val)
{
	writel(val, tdev->nvlw_minion_base + reg);
}

u32 nvlw_nvl_readl(struct tnvlink_dev *tdev, u32 reg)
{
	return readl(tdev->tlink.nvlw_nvl_base + reg);
}

void nvlw_nvl_writel(struct tnvlink_dev *tdev, u32 reg, u32 val)
{
	writel(val, tdev->tlink.nvlw_nvl_base + reg);
}

u32 nvlw_sync2x_readl(struct tnvlink_dev *tdev, u32 reg)
{
	return readl(tdev->nvlw_sync2x_base + reg);
}

void nvlw_sync2x_writel(struct tnvlink_dev *tdev, u32 reg, u32 val)
{
	writel(val, tdev->nvlw_sync2x_base + reg);
}

u32 nvlw_nvltlc_readl(struct tnvlink_dev *tdev, u32 reg)
{
	return readl(tdev->tlink.nvlw_nvltlc_base + reg);
}

void nvlw_nvltlc_writel(struct tnvlink_dev *tdev, u32 reg, u32 val)
{
	writel(val, tdev->tlink.nvlw_nvltlc_base + reg);
}

static inline u32 mssnvlink_0_readl(struct tnvlink_dev *tdev, u32 reg)
{
	return readl(tdev->mssnvlink_0_base + reg);
}

static inline void mssnvlink_0_writel(struct tnvlink_dev *tdev, u32 reg,
									u32 val)
{
	writel(val, tdev->mssnvlink_0_base + reg);
}

/* TODO: Remove all non-NVLINK MMIO register writes from the driver */
static inline void non_nvlink_writel(u32 reg, u32 val)
{
	void __iomem *ptr = ioremap(reg, 0x4);

	__raw_writel(val, ptr);
	iounmap(ptr);
}

/*
 * Wait for a bit to be set or cleared in an NVLINK register. If the desired bit
 * condition doesn't happen in a certain amount of time, a timeout will happen.
 */
int wait_for_reg_cond_nvlink(
			struct tnvlink_dev *tdev,
			u32 reg,
			u32 bit,
			int bit_set,
			char *bit_name,
			u32 (*reg_readl)(struct tnvlink_dev *, u32),
			u32 *reg_val)
{
	u32 elapsed_us = 0;

	do {
		usleep_range(DEFAULT_LOOP_SLEEP_US, DEFAULT_LOOP_SLEEP_US*2);
		elapsed_us += DEFAULT_LOOP_SLEEP_US;

		*reg_val = reg_readl(tdev, reg);
		if ((bit_set && (*reg_val & BIT(bit))) ||
		    (!bit_set && ((*reg_val & BIT(bit)) == 0)))
			break;
	} while (elapsed_us < DEFAULT_LOOP_TIMEOUT_US);
	if (elapsed_us >= DEFAULT_LOOP_TIMEOUT_US) {
		if (bit_set) {
			nvlink_err("Timeout waiting for the %s bit to get set",
				bit_name);
		} else {
			nvlink_err(
				"Timeout waiting for the %s bit to get cleared",
				bit_name);
		}
		return -1;
	}

	return 0;
}

/*
 * tegra_nvlink_car_init(): initializes UPHY mgmt and sys clk
 * clears the resets to uphy
 */
static int tegra_nvlink_car_enable(struct tnvlink_dev *tdev)
{
	int ret;
	unsigned long clk_rate = 0;
	u32 reg_val = 0;

	/* enable the Management clock */
	ret = clk_prepare_enable(tdev->clk_nvhs_pll0_mgmt);
	if (ret < 0) {
		nvlink_err("nvlink mgmt clock enable failed : %d", ret);
		goto fail;
	}
	/* Clear reset for UPHY PM */
	reset_control_deassert(tdev->rst_nvhs_uphy_pm);
	/* Enable clock for nvlink_sys */
	ret = clk_prepare_enable(tdev->clk_nvlink_sys);
	if (ret < 0) {
		nvlink_err("nvlink sys clock enable failed : %d", ret);
		goto nvlink_sys_fail;
	}

#if IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS)
	ret = tegra_unpowergate_partition(tdev->pgid_nvl);
	if (ret < 0) {
		nvlink_err("Couldn't unpowergate : %d", ret);
		goto unpowergate_partition_fail;
	}
#endif
	/* Take link out of reset */
	reg_val = nvlw_tioctrl_readl(tdev, NVLW_RESET) |
			BIT(NVLW_RESET_LINKRESET);
	nvlw_tioctrl_writel(tdev, NVLW_RESET, reg_val);
	udelay(NVLW_POST_RESET_DELAY_US);

	/* Reset persistent HW state for this link */
	reg_val = nvlw_tioctrl_readl(tdev, NVLW_DEBUG_RESET) &
			~BIT(NVLW_DEBUG_RESET_LINK);
	nvlw_tioctrl_writel(tdev, NVLW_DEBUG_RESET, reg_val);
	udelay(NVLW_POST_RESET_DELAY_US);
	reg_val = nvlw_tioctrl_readl(tdev, NVLW_DEBUG_RESET) |
					BIT(NVLW_DEBUG_RESET_LINK) |
					BIT(NVLW_DEBUG_RESET_COMMON);
	nvlw_tioctrl_writel(tdev, NVLW_DEBUG_RESET, reg_val);
	udelay(NVLW_POST_RESET_DELAY_US);

	reset_control_deassert(tdev->rst_nvhs_uphy_l0);
	reset_control_deassert(tdev->rst_nvhs_uphy_l1);
	reset_control_deassert(tdev->rst_nvhs_uphy_l2);
	reset_control_deassert(tdev->rst_nvhs_uphy_l3);
	reset_control_deassert(tdev->rst_nvhs_uphy_l4);
	reset_control_deassert(tdev->rst_nvhs_uphy_l5);
	reset_control_deassert(tdev->rst_nvhs_uphy_l6);
	reset_control_deassert(tdev->rst_nvhs_uphy_l7);
	reset_control_deassert(tdev->rst_nvhs_uphy);
	reset_control_deassert(tdev->rst_nvhs_uphy_pll0);

	if (tdev->refclk == NVLINK_REFCLK_150) {
		ret = clk_set_rate(tdev->clk_pllnvhs,
				   PLLNVHS_FREQ_150MHZ);
		if (ret < 0) {
			nvlink_err("nvlink pllnvhs setrate failed : %d", ret);
			goto pllnvhs_fail;
		} else {
			clk_rate = clk_get_rate(tdev->clk_pllnvhs);
			if (clk_rate != PLLNVHS_FREQ_150MHZ) {
				nvlink_err("clk_pllnvhs rate = %lu", clk_rate);
				ret = -EINVAL;
				goto pllnvhs_fail;
			}
		}
		ret = clk_prepare_enable(tdev->clk_pllnvhs);
		if (ret < 0) {
			nvlink_err("pllnvhs clock enable failed : %d", ret);
			goto pllnvhs_fail;
		}
	}
	return ret;

pllnvhs_fail:
#if IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS)
	tegra_powergate_partition(tdev->pgid_nvl);
#endif
unpowergate_partition_fail:
	clk_disable_unprepare(tdev->clk_nvlink_sys);
nvlink_sys_fail:
	clk_disable_unprepare(tdev->clk_nvhs_pll0_mgmt);
fail:
	return ret;
}

static void init_tlc_buffers(struct tnvlink_dev *tdev)
{
	nvlink_dbg("Initializing TLC buffers");

	nvlw_nvltlc_writel(tdev, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC0, 0x10000C0);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC1, 0x0);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC2, 0x0);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC3, 0x0);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC4, 0x0);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC5, 0x10000C0);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC6, 0x0);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_CTRL_BUFFER_CREDITS_VC7, 0x0);

	nvlw_nvltlc_writel(tdev, NVLTLC_RX_CTRL_BUFFER_SZ_VC0, 0xff00bf);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_CTRL_BUFFER_SZ_VC1, 0xff00bf);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_CTRL_BUFFER_SZ_VC2, 0xff00bf);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_CTRL_BUFFER_SZ_VC3, 0xff00bf);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_CTRL_BUFFER_SZ_VC4, 0xff00bf);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_CTRL_BUFFER_SZ_VC5, 0x1ff017f);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_CTRL_BUFFER_SZ_VC6, 0x1ff017f);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_CTRL_BUFFER_SZ_VC7, 0x1ff017f);

	nvlw_nvltlc_writel(tdev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC0, 0x800040);
	nvlw_nvltlc_writel(tdev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC1, 0x0);
	nvlw_nvltlc_writel(tdev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC2, 0x0);
	nvlw_nvltlc_writel(tdev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC3, 0x0);
	nvlw_nvltlc_writel(tdev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC4, 0x0);
	nvlw_nvltlc_writel(tdev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC5, 0x800040);
	nvlw_nvltlc_writel(tdev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC6, 0x0);
	nvlw_nvltlc_writel(tdev, NVLTLC_TX_CTRL_BUFFER_CREDITS_VC7, 0x0);

	nvlw_nvltlc_writel(tdev, NVLTLC_TX_CTRL_BUFFER_SZ_VC0, 0x7f003f);
	nvlw_nvltlc_writel(tdev, NVLTLC_TX_CTRL_BUFFER_SZ_VC1, 0x7f003f);
	nvlw_nvltlc_writel(tdev, NVLTLC_TX_CTRL_BUFFER_SZ_VC2, 0x7f003f);
	nvlw_nvltlc_writel(tdev, NVLTLC_TX_CTRL_BUFFER_SZ_VC3, 0x7f003f);
	nvlw_nvltlc_writel(tdev, NVLTLC_TX_CTRL_BUFFER_SZ_VC4, 0x7f003f);
	nvlw_nvltlc_writel(tdev, NVLTLC_TX_CTRL_BUFFER_SZ_VC5, 0xff007f);
	nvlw_nvltlc_writel(tdev, NVLTLC_TX_CTRL_BUFFER_SZ_VC6, 0xff007f);
	nvlw_nvltlc_writel(tdev, NVLTLC_TX_CTRL_BUFFER_SZ_VC7, 0xff007f);

	nvlw_nvltlc_writel(tdev, NVLTLC_TX_ERR_LOG_EN_0, 0x3ffffff);
	nvlw_nvltlc_writel(tdev, NVLTLC_TX_ERR_REPORT_EN_0, 0x3ffffff);
	nvlw_nvltlc_writel(tdev, NVLTLC_TX_ERR_CONTAIN_EN_0, 0x3ffffff);

	nvlw_nvltlc_writel(tdev, NVLTLC_RX_ERR_LOG_EN_0, 0xffffff);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_ERR_REPORT_EN_0, 0xffffff);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_ERR_CONTAIN_EN_0, 0xffffff);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_ERR_LOG_EN_1, 0x3fffff);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_ERR_REPORT_EN_1, 0x3fffff);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_ERR_CONTAIN_EN_1, 0x3fffff);

	nvlw_nvltlc_writel(tdev, NVLTLC_TX_CTRL_BUFFER_READY, 0x1);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_CTRL_BUFFER_READY, 0x1);
}

static void init_tlc(struct tnvlink_dev *tdev)
{
	init_tlc_buffers(tdev);
	init_single_lane_params(tdev);
}

/*
 * mssnvlink_init:
 * Do the folllwing to initialize MSSNVLINK. This initialization is required to
 * allow traffic to flow between the NVLINK controller and MSSNVLINK:
 *    - Program the upper limit of the NVLINK aperture in MSSNVLINK. The bottom
 *      of the aperture is fixed at 128 GB. So we don't need to program that.
 *    - Release MSSNVLINK header and data credits to the NVLINK controller.
 *
 * TODO: Convert the magic values being programmed below into something that's
 * more understandable.
 */
static void mssnvlink_init(struct tnvlink_dev *tdev)
{
	/* Program the upper limit of the NVLINK aperture in MSSNVLINK */
	nvlink_dbg("Programming MSSNVLINK_TOM to %u GB", NVLINK_TOM_GB);
	non_nvlink_writel(MCB_BASE + MC_MSSNVLINK_TOM, NVLINK_TOM_MB);
	non_nvlink_writel(MCB_BASE + MC_MSSNVLINK_REG_CTRL, 0x1);

	/* MSSNVLINK credit programming */
	mssnvlink_0_writel(tdev, MSSNVLINK_MASTER_CREDIT_TRANSINFO, 0x15455000);
	mssnvlink_0_writel(tdev, MSSNVLINK_MASTER_CREDIT_INGR_DATA, 0x8020000);
	mssnvlink_0_writel(tdev, MSSNVLINK_SLAVE_CREDIT_TRANSINFO, 0x14050000);
	mssnvlink_0_writel(tdev, MSSNVLINK_SLAVE_CREDIT_INGR_DATA, 0x300c0000);

	/*
	 * Performance settings for balancing request and response bandwidth
	 * across NVLINK
	 */
	non_nvlink_writel(MCB_BASE + MC_MCF_IREQX_VCARB_CONFIG, 0x8f0);
	non_nvlink_writel(MCB_BASE + MC_MCF_OREQX_VCARB_CONFIG, 0x8f0);
}

/*
 * Program the upper limit of the NVLINK aperture in SCF.
 * The bottom of the aperture is fixed at 128 GB. So we don't need to program
 * that.
 */
static inline void program_scf_tom(void)
{
	u32 reg_val = SCF_NVLINK_CFG_TOM_MB_F(NVLINK_TOM_MB) |
			BIT(SCF_NVLINK_CFG_EN);

	nvlink_dbg("Programming SCF TOM to %u GB", NVLINK_TOM_GB);
	asm volatile("msr s3_0_c15_c0_3, %0" : : "r" (reg_val));
}

/*
 * Performs device level initialization like setting up the clocks and
 * resets, booting the minion and configuring the device level interrupts.
 */
int t19x_nvlink_dev_early_init(struct nvlink_device *ndev)
{
	struct tnvlink_dev *tdev = NULL;
	int ret = 0;

	if (!ndev) {
		nvlink_err("Invalid device struct pointer");
		return -EINVAL;
	}
	tdev = (struct tnvlink_dev *)ndev->priv;
	/* TODO: Add return value check for all subfunctions */
	tegra_nvlink_car_enable(tdev);
	ret = minion_boot(tdev);
	if (ret < 0)
		goto fail;
	nvlink_config_common_intr(tdev);
	nvlink_dbg("Device early init done for dev%u", ndev->device_id);
	goto success;
fail:
	nvlink_err("Device early init failed for dev%u", ndev->device_id);
success:
	return ret;
}

/*
 * Performs link level initialization like phy_init, setting up the link
 * interrupts and enabling the AN0 packets.
 */
int t19x_nvlink_link_early_init(struct nvlink_device *ndev)
{
	struct tnvlink_dev *tdev = NULL;
	int ret = 0;

	if (!ndev) {
		nvlink_err("Invalid device struct pointer");
		return -EINVAL;
	}
	tdev = (struct tnvlink_dev *)ndev->priv;
	/* TODO: Add return value check for all subfunctions */
	nvlink_enable_AN0_packets(tdev);
	ret = init_nvhs_phy(tdev);
	if (ret < 0)
		goto fail;
	nvlink_enable_link_interrupts(tdev);
	init_tlc(tdev);
	nvlink_dbg("Link early init done for dev%u", ndev->device_id);
	goto success;
fail:
	nvlink_err("link early init failed for dev%u", ndev->device_id);
success:
	return ret;
}

/*
 * Performs memory interface initialization
 */
int t19x_nvlink_dev_interface_init(struct nvlink_device *ndev)
{
	struct tnvlink_dev *tdev = NULL;
	int ret = 0;

	if (!ndev) {
		nvlink_err("Invalid device struct pointer");
		return -EINVAL;
	}
	tdev = (struct tnvlink_dev *)ndev->priv;
	/* TODO: Add return value check for all subfunctions */
	mssnvlink_init(tdev);
	program_scf_tom();
	nvlink_dbg("Link interface init done for dev%u", ndev->device_id);

	return ret;
}

/*
 * Reg_init programs the prod-setting if any.
 */
int t19x_nvlink_dev_reg_init(struct nvlink_device *ndev)
{
	struct tnvlink_dev *tdev = NULL;
	int ret = 0;

	if (!ndev) {
		nvlink_err("Invalid device struct pointer");
		return -EINVAL;
	}
	tdev = (struct tnvlink_dev *)ndev->priv;
	if (tdev->prod_list) {
		ret = tegra_prod_set_by_name(&tdev->mssnvlink_0_base,
				"prod", tdev->prod_list);
		if (ret < 0) {
			/* prod setting failures should not stop nvlink init */
			ret = 0;
			goto fail;
		}
	}
	nvlink_dbg("Device reg init done for dev%u", ndev->device_id);
	goto success;
fail:
	nvlink_err("Device reg init failed for dev%u", ndev->device_id);
success:
	return ret;
}

static int tegra_nvlink_clk_rst_init(struct tnvlink_dev *tdev)
{
	/* clocks */
	tdev->clk_nvhs_pll0_mgmt = devm_clk_get(tdev->dev,
			"nvhs_pll0_mgmt");
	if (IS_ERR(tdev->clk_nvhs_pll0_mgmt)) {
		nvlink_err("missing mgmt clock");
		return PTR_ERR(tdev->clk_nvhs_pll0_mgmt);
	}

	tdev->clk_nvlink_sys = devm_clk_get(tdev->dev,
			"nvlink_sys");
	if (IS_ERR(tdev->clk_nvlink_sys)) {
		nvlink_err("missing sys clock");
		return PTR_ERR(tdev->clk_nvlink_sys);
	}

	tdev->clk_pllnvhs = devm_clk_get(tdev->dev,
			"pllnvhs");
	if (IS_ERR(tdev->clk_pllnvhs)) {
		nvlink_err("missing pllnvhs clock");
		return PTR_ERR(tdev->clk_pllnvhs);
	}

	tdev->clk_m = devm_clk_get(tdev->dev, "clk_m");
	if (IS_ERR(tdev->clk_m)) {
		nvlink_err("missing clk_m clock");
		return PTR_ERR(tdev->clk_m);
	}

	tdev->clk_nvlink_pll_txclk = devm_clk_get(tdev->dev,
				"nvlink_pll_txclk");
	if (IS_ERR(tdev->clk_nvlink_pll_txclk)) {
		nvlink_err("missing nvlink_pll_txclk clock");
		return PTR_ERR(tdev->clk_nvlink_pll_txclk);
	}

	tdev->clk_nvlink_tx = devm_clk_get(tdev->dev, "nvlink_tx");
	if (IS_ERR(tdev->clk_nvlink_tx)) {
		nvlink_err("missing nvlink_tx clock");
		return PTR_ERR(tdev->clk_nvlink_tx);
	}

	/* Resets */
	tdev->rst_nvhs_uphy_pm = devm_reset_control_get(tdev->dev,
			"nvhs_uphy_pm");
	if (IS_ERR(tdev->rst_nvhs_uphy_pm)) {
		nvlink_err("missing rst_nvhs_uphy_pm reset");
		return PTR_ERR(tdev->rst_nvhs_uphy_pm);
	}
	tdev->rst_nvhs_uphy = devm_reset_control_get(tdev->dev,
			"nvhs_uphy");
	if (IS_ERR(tdev->rst_nvhs_uphy)) {
		nvlink_err("missing rst_nvhs_uphy reset");
		return PTR_ERR(tdev->rst_nvhs_uphy);
	}
	tdev->rst_nvhs_uphy_pll0 = devm_reset_control_get(tdev->dev,
			"nvhs_uphy_pll0");
	if (IS_ERR(tdev->rst_nvhs_uphy_pll0)) {
		nvlink_err("missing rst_nvhs_uphy_pll0 reset");
		return PTR_ERR(tdev->rst_nvhs_uphy_pll0);
	}
	tdev->rst_nvhs_uphy_l0 = devm_reset_control_get(tdev->dev,
			"nvhs_uphy_l0");
	if (IS_ERR(tdev->rst_nvhs_uphy_l0)) {
		nvlink_err("missing rst_nvhs_uphy_l0 reset");
		return PTR_ERR(tdev->rst_nvhs_uphy_l0);
	}
	tdev->rst_nvhs_uphy_l1 = devm_reset_control_get(tdev->dev,
			"nvhs_uphy_l1");
	if (IS_ERR(tdev->rst_nvhs_uphy_l1)) {
		nvlink_err("missing rst_nvhs_uphy_l1 reset");
		return PTR_ERR(tdev->rst_nvhs_uphy_l1);
	}
	tdev->rst_nvhs_uphy_l2 = devm_reset_control_get(tdev->dev,
			"nvhs_uphy_l2");
	if (IS_ERR(tdev->rst_nvhs_uphy_l2)) {
		nvlink_err("missing rst_nvhs_uphy_l2 reset");
		return PTR_ERR(tdev->rst_nvhs_uphy_l2);
	}
	tdev->rst_nvhs_uphy_l3 = devm_reset_control_get(tdev->dev,
			"nvhs_uphy_l3");
	if (IS_ERR(tdev->rst_nvhs_uphy_l3)) {
		nvlink_err("missing rst_nvhs_uphy_l3 reset");
		return PTR_ERR(tdev->rst_nvhs_uphy_l3);
	}
	tdev->rst_nvhs_uphy_l4 = devm_reset_control_get(tdev->dev,
			"nvhs_uphy_l4");
	if (IS_ERR(tdev->rst_nvhs_uphy_l4)) {
		nvlink_err("missing rst_nvhs_uphy_l4 reset");
		return PTR_ERR(tdev->rst_nvhs_uphy_l4);
	}
	tdev->rst_nvhs_uphy_l5 = devm_reset_control_get(tdev->dev,
			"nvhs_uphy_l5");
	if (IS_ERR(tdev->rst_nvhs_uphy_l5)) {
		nvlink_err("missing rst_nvhs_uphy_l5 reset");
		return PTR_ERR(tdev->rst_nvhs_uphy_l5);
	}
	tdev->rst_nvhs_uphy_l6 = devm_reset_control_get(tdev->dev,
			"nvhs_uphy_l6");
	if (IS_ERR(tdev->rst_nvhs_uphy_l6)) {
		nvlink_err("missing rst_nvhs_uphy_l6 reset");
		return PTR_ERR(tdev->rst_nvhs_uphy_l6);
	}
	tdev->rst_nvhs_uphy_l7 = devm_reset_control_get(tdev->dev,
			"nvhs_uphy_l7");
	if (IS_ERR(tdev->rst_nvhs_uphy_l7)) {
		nvlink_err("missing rst_nvhs_uphy_l7 reset");
		return PTR_ERR(tdev->rst_nvhs_uphy_l7);
	}

#if IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS)
	tdev->pgid_nvl = tegra_pd_get_powergate_id(tegra_nvl_pd);
#endif

	return 0;
}

static void tegra_nvlink_clk_rst_deinit(struct tnvlink_dev *tdev)
{
	/* clocks */
	if (tdev->clk_nvhs_pll0_mgmt)
		devm_clk_put(tdev->dev, tdev->clk_nvhs_pll0_mgmt);

	if (tdev->clk_nvlink_sys)
		devm_clk_put(tdev->dev, tdev->clk_nvlink_sys);

	if (tdev->clk_pllnvhs)
		devm_clk_put(tdev->dev, tdev->clk_pllnvhs);

	if (tdev->clk_m)
		devm_clk_put(tdev->dev, tdev->clk_m);

	if (tdev->clk_nvlink_pll_txclk)
		devm_clk_put(tdev->dev, tdev->clk_nvlink_pll_txclk);

	if (tdev->clk_nvlink_tx)
		devm_clk_put(tdev->dev, tdev->clk_nvlink_tx);

	reset_control_assert(tdev->rst_nvhs_uphy_pm);
	reset_control_assert(tdev->rst_nvhs_uphy);
	reset_control_assert(tdev->rst_nvhs_uphy_pll0);
	reset_control_assert(tdev->rst_nvhs_uphy_l0);
	reset_control_assert(tdev->rst_nvhs_uphy_l1);
	reset_control_assert(tdev->rst_nvhs_uphy_l2);
	reset_control_assert(tdev->rst_nvhs_uphy_l3);
	reset_control_assert(tdev->rst_nvhs_uphy_l4);
	reset_control_assert(tdev->rst_nvhs_uphy_l5);
	reset_control_assert(tdev->rst_nvhs_uphy_l6);
	reset_control_assert(tdev->rst_nvhs_uphy_l7);

}

static int t19x_nvlink_endpt_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct tnvlink_dev *tdev;
	struct nvlink_device *ndev;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *endpt_dt_node = NULL;
	struct device *dev = NULL;
	struct tegra_prod *nvlink_prod;

	if (!np) {
		nvlink_err("Invalid device_node");
		ret = -ENODEV;
		goto fail;
	}

	ndev = kzalloc(sizeof(struct nvlink_device), GFP_KERNEL);
	if (!ndev) {
		nvlink_err("Couldn't allocate memory for nvlink device struct");
		ret = -ENOMEM;
		goto err_alloc_ndev;
	}

	tdev = kzalloc(sizeof(struct tnvlink_dev), GFP_KERNEL);
	if (!tdev) {
		nvlink_err("Couldn't allocate memory for tegra nvlink device");
		ret = -ENOMEM;
		goto err_alloc_tdev;
	}

	/* Map NVLINK apertures listed in device tree node */
	tdev->nvlw_tioctrl_base =
				of_io_request_and_map(np, 0,
						"NVLW_TIOCTRL aperture");
	if (IS_ERR(tdev->nvlw_tioctrl_base)) {
		nvlink_err("Couldn't map the NVLW_TIOCTRL aperture");
		ret = PTR_ERR(tdev->nvlw_tioctrl_base);
		goto err_mapping;
	}

	tdev->nvlw_nvlipt_base =
				of_io_request_and_map(np, 1,
						"NVLW_NVLIPT aperture");
	if (IS_ERR(tdev->nvlw_nvlipt_base)) {
		nvlink_err("Couldn't map the NVLW_NVLIPT aperture");
		ret = PTR_ERR(tdev->nvlw_nvlipt_base);
		goto err_mapping;
	}

	tdev->nvlw_minion_base =
				of_io_request_and_map(np, 2,
						"NVLW_MINION aperture");
	if (IS_ERR(tdev->nvlw_minion_base)) {
		nvlink_err("Couldn't map the NVLW_MINION aperture");
		ret = PTR_ERR(tdev->nvlw_minion_base);
		goto err_mapping;
	}

	tdev->tlink.nvlw_nvl_base =
				of_io_request_and_map(np, 3,
						"NVLW_NVL aperture");
	if (IS_ERR(tdev->tlink.nvlw_nvl_base)) {
		nvlink_err("Couldn't map the NVLW_NVL aperture");
		ret = PTR_ERR(tdev->tlink.nvlw_nvl_base);
		goto err_mapping;
	}

	tdev->nvlw_sync2x_base = of_io_request_and_map(np, 4,
							"NVLW_SYNC2X aperture");
	if (IS_ERR(tdev->nvlw_sync2x_base)) {
		nvlink_err("Couldn't map the NVLW_SYNC2X aperture");
		ret = PTR_ERR(tdev->nvlw_sync2x_base);
		goto err_mapping;
	}

	tdev->tlink.nvlw_nvltlc_base =
				of_io_request_and_map(np, 5,
						"NVLW_NVLTLC aperture");
	if (IS_ERR(tdev->tlink.nvlw_nvltlc_base)) {
		nvlink_err("Couldn't map the NVLW_NVLTLC aperture");
		ret = PTR_ERR(tdev->tlink.nvlw_nvltlc_base);
		goto err_mapping;
	}

	tdev->mssnvlink_0_base = of_io_request_and_map(np, 6,
							"MSSNVLINK_0 aperture");
	if (IS_ERR(tdev->mssnvlink_0_base)) {
		nvlink_err("Couldn't map the MSSNVLINK_0 aperture");
		ret = PTR_ERR(tdev->mssnvlink_0_base);
		goto err_mapping;
	}

	tdev->is_nea = DEFAULT_IS_NEA;
	tdev->irq = platform_get_irq(pdev, 0);
	if (tdev->irq < 0) {
		nvlink_err("Couldn't get interrupt listed in device tree");
		ret = -EINVAL;
		goto err_mapping;
	}

	ret = devm_request_threaded_irq(&pdev->dev, tdev->irq,
					NULL, t19x_nvlink_endpt_isr,
					IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
					dev_name(&pdev->dev), tdev);
	if (ret < 0) {
		nvlink_err("Failed to register irq %d", tdev->irq);
		goto err_mapping;
	}

	tdev->dev = &pdev->dev;
	tdev->class.owner = THIS_MODULE;
	tdev->class.name = NVLINK_DRV_NAME;

	/* Create device node */
	ret = class_register(&tdev->class);
	if (ret) {
		nvlink_err("Failed to register class");
		goto err_mapping;
	}

	ret = alloc_chrdev_region(&tdev->dev_t, 0, 1, dev_name(tdev->dev));
	if (ret) {
		nvlink_err("Failed to allocate dev_t");
		goto err_chrdev_region;
	}

	cdev_init(&tdev->cdev, &t19x_nvlink_endpt_ops);
	tdev->cdev.owner = THIS_MODULE;

	ret = cdev_add(&tdev->cdev, tdev->dev_t, 1);
	if (ret) {
		nvlink_err("Failed to add cdev");
		goto err_cdev;
	}

	dev = device_create(&tdev->class,
				NULL,
				tdev->dev_t,
				NULL,
				NVLINK_DRV_NAME);
	if (IS_ERR(dev)) {
		nvlink_err("Failed to create device");
		ret = PTR_ERR(dev);
		goto err_device;
	}

	ret = tegra_nvlink_clk_rst_init(tdev);
	if (ret)
		goto err_clk_rst;

	nvlink_prod = devm_tegra_prod_get(tdev->dev);
	if (IS_ERR_OR_NULL(nvlink_prod)) {
		nvlink_err("Prod-setting not available");
		nvlink_prod = NULL;
	}
	tdev->prod_list = nvlink_prod;
	tdev->refclk = NVLINK_REFCLK_156;
	ndev->speed = NVLINK_SPEED_25;
	ndev->link_bitrate = LINK_BITRATE_156MHZ_25GBPS;
	tdev->ndev = ndev;

	tdev->tlink.sl_params = entry_100us_sl_params;
	tdev->tlink.nlink = &(ndev->link);
	t19x_nvlink_endpt_debugfs_init(tdev);

	/* Fill in the nvlink_device struct */
	/* Read NVLINK topology information in device tree */
	endpt_dt_node = of_get_child_by_name(np, "endpoint");
	of_property_read_u32(endpt_dt_node, "local_dev_id",
					&ndev->device_id);
	of_property_read_u32(endpt_dt_node, "local_link_id",
					&ndev->link.link_id);
	ndev->is_master = of_property_read_bool(endpt_dt_node, "is_master");
	of_property_read_u32(endpt_dt_node, "remote_dev_id",
					&ndev->link.remote_dev_info.device_id);
	of_property_read_u32(endpt_dt_node, "remote_link_id",
					&ndev->link.remote_dev_info.link_id);

	nvlink_dbg("Device Tree Topology Information:");
	nvlink_dbg("  - Local Device: Device ID = %d, Link ID = %d, Is master? = %s",
		ndev->device_id,
		ndev->link.link_id,
		ndev->is_master ? "True" : "False");
	nvlink_dbg("  - Remote Device: Device ID = %d, Link ID = %d",
		ndev->link.remote_dev_info.device_id,
		ndev->link.remote_dev_info.link_id);

	ndev->dev_ops.dev_early_init = t19x_nvlink_dev_early_init;
	ndev->dev_ops.dev_interface_init = t19x_nvlink_dev_interface_init;
	ndev->dev_ops.dev_reg_init = t19x_nvlink_dev_reg_init;
	/* Point priv of ndev to the tegra nvlink endpoint device struct */
	ndev->priv = (void *) tdev;

	/* Fill in the nvlink_link struct */
	ndev->link.device_id = ndev->device_id;
	ndev->link.is_connected = true;
	ndev->link.link_ops.get_link_mode = t19x_nvlink_get_link_mode;
	ndev->link.link_ops.set_link_mode = t19x_nvlink_set_link_mode;
	ndev->link.link_ops.get_sublink_mode = t19x_nvlink_get_sublink_mode;
	ndev->link.link_ops.set_sublink_mode = t19x_nvlink_set_sublink_mode;
	ndev->link.link_ops.get_link_state = t19x_nvlink_get_link_state;
	ndev->link.link_ops.get_tx_sublink_state =
					t19x_nvlink_get_tx_sublink_state;
	ndev->link.link_ops.get_rx_sublink_state =
					t19x_nvlink_get_rx_sublink_state;
	ndev->link.link_ops.link_early_init =
				t19x_nvlink_link_early_init;
	/* Point priv of ndev->link to the tegra nvlink endpoint link struct */
	ndev->link.priv = (void *)&(tdev->tlink);

	platform_set_drvdata(pdev, tdev);

	/* Register device with core driver*/
	ret = nvlink_register_device(ndev);
	if (ret < 0) {
		goto err_ndev_register;
	}

	/* Register link with core driver */
	ret = nvlink_register_link(&ndev->link);
	if (ret < 0) {
		goto err_nlink_register;
	}

	nvlink_dbg("Probe successful!");
	goto success;

err_nlink_register:
	nvlink_unregister_device(ndev);
err_ndev_register:
	t19x_nvlink_endpt_debugfs_deinit(tdev);
	tegra_nvlink_clk_rst_deinit(tdev);
err_clk_rst:
	device_destroy(&tdev->class, tdev->dev_t);
err_device:
	cdev_del(&tdev->cdev);
err_cdev:
	unregister_chrdev_region(tdev->dev_t, 1);
err_chrdev_region:
	class_unregister(&tdev->class);

err_mapping:
	if (!IS_ERR(tdev->nvlw_tioctrl_base))
		iounmap(tdev->nvlw_tioctrl_base);

	if (!IS_ERR(tdev->nvlw_nvlipt_base))
		iounmap(tdev->nvlw_nvlipt_base);

	if (!IS_ERR(tdev->nvlw_minion_base))
		iounmap(tdev->nvlw_minion_base);

	if (!IS_ERR(tdev->tlink.nvlw_nvl_base))
		iounmap(tdev->tlink.nvlw_nvl_base);

	if (!IS_ERR(tdev->nvlw_sync2x_base))
		iounmap(tdev->nvlw_sync2x_base);

	if (!IS_ERR(tdev->tlink.nvlw_nvltlc_base))
		iounmap(tdev->tlink.nvlw_nvltlc_base);

	if (!IS_ERR(tdev->mssnvlink_0_base))
		iounmap(tdev->mssnvlink_0_base);

	kfree(tdev);
err_alloc_tdev:
	kfree(ndev);
err_alloc_ndev:
fail:
	nvlink_err("Probe failed!");
success:
	return ret;
}

static int t19x_nvlink_endpt_remove(struct platform_device *pdev)
{
	struct nvlink_device *ndev = NULL;
	struct tnvlink_dev *tdev = NULL;

	tdev = platform_get_drvdata(pdev);
	ndev = tdev->ndev;

	nvlink_unregister_link(&ndev->link);
	nvlink_unregister_device(ndev);
	t19x_nvlink_endpt_debugfs_deinit(tdev);
	tegra_nvlink_clk_rst_deinit(tdev);
	device_destroy(&tdev->class, tdev->dev_t);
	cdev_del(&tdev->cdev);
	unregister_chrdev_region(tdev->dev_t, 1);
	class_unregister(&tdev->class);
	iounmap(tdev->nvlw_tioctrl_base);
	iounmap(tdev->nvlw_nvlipt_base);
	iounmap(tdev->nvlw_minion_base);
	iounmap(tdev->tlink.nvlw_nvl_base);
	iounmap(tdev->nvlw_sync2x_base);
	iounmap(tdev->tlink.nvlw_nvltlc_base);
	iounmap(tdev->mssnvlink_0_base);
	kfree(tdev);
	kfree(ndev);
	return 0;
}

static struct platform_driver t19x_nvlink_endpt_pdrv = {
	.probe		= t19x_nvlink_endpt_probe,
	.remove		= t19x_nvlink_endpt_remove,
	.driver		= {
		.name	= NVLINK_DRV_NAME,
		.of_match_table = of_match_ptr(t19x_nvlink_controller_of_match),
	},
};

static int __init t19x_nvlink_endpt_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&t19x_nvlink_endpt_pdrv);
	if (ret < 0)
		nvlink_err("Platform driver register failed");

	return ret;
}

static void __exit t19x_nvlink_endpt_exit(void)
{
	nvlink_dbg("Unloading the T19x NVLINK endpoint driver");
	platform_driver_unregister(&t19x_nvlink_endpt_pdrv);
}

module_init(t19x_nvlink_endpt_init);
module_exit(t19x_nvlink_endpt_exit);

MODULE_ALIAS(NVLINK_DRV_NAME);
MODULE_DESCRIPTION("T19x NVLINK Endpoint Driver");
MODULE_LICENSE("GPL v2");
