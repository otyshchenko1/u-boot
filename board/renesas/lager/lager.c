// SPDX-License-Identifier: GPL-2.0
/*
 * board/renesas/lager/lager.c
 *     This file is lager board support.
 *
 * Copyright (C) 2013 Renesas Electronics Corporation
 * Copyright (C) 2013 Nobuhiro Iwamatsu <nobuhiro.iwamatsu.yj@renesas.com>
 */

#include <common.h>
#include <environment.h>
#include <malloc.h>
#include <netdev.h>
#include <dm.h>
#include <dm/platform_data/serial_sh.h>
#include <asm/processor.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <linux/errno.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/arch/rmobile.h>
#include <asm/arch/rcar-mstp.h>
#include <asm/arch/mmc.h>
#include <asm/arch/sh_sdhi.h>
#include <miiphy.h>
#include <i2c.h>
#include <mmc.h>
#include "qos.h"

DECLARE_GLOBAL_DATA_PTR;

#define CLK2MHZ(clk)	(clk / 1000 / 1000)
void s_init(void)
{
	struct rcar_rwdt *rwdt = (struct rcar_rwdt *)RWDT_BASE;
	struct rcar_swdt *swdt = (struct rcar_swdt *)SWDT_BASE;

	/* Watchdog init */
	writel(0xA5A5A500, &rwdt->rwtcsra);
	writel(0xA5A5A500, &swdt->swtcsra);

	/* CPU frequency setting. Set to 1.4GHz */
	if (rmobile_get_cpu_rev_integer() >= R8A7790_CUT_ES2X) {
		u32 stat = 0;
		u32 stc = ((1400 / CLK2MHZ(CONFIG_SYS_CLK_FREQ)) - 1)
			<< PLL0_STC_BIT;
		clrsetbits_le32(PLL0CR, PLL0_STC_MASK, stc);

		do {
			stat = readl(PLLECR) & PLL0ST;
		} while (stat == 0x0);
	}

	/* QoS(Quality-of-Service) Init */
	qos_init();
}

#define TMU0_MSTP125	BIT(25)

#define SD1CKCR		0xE6150078
#define SD2CKCR		0xE615026C
#define SD_97500KHZ	0x7

int board_early_init_f(void)
{
	mstp_clrbits_le32(MSTPSR1, SMSTPCR1, TMU0_MSTP125);

	/*
	 * SD0 clock is set to 97.5MHz by default.
	 * Set SD1 and SD2 to the 97.5MHz as well.
	 */
	writel(SD_97500KHZ, SD1CKCR);
	writel(SD_97500KHZ, SD2CKCR);

	return 0;
}

#ifdef CONFIG_ARMV7_NONSEC
#define TIMER_BASE		0xE6080000
#define TIMER_CNTCR		0x00
#define TIMER_CNTFID0	0x20

/*
 * Taking into the account that arch timer is only configurable in secure mode
 * and we are going to enter kernel/hypervisor in a non-secure mode, update
 * arch timer right now to avoid possible issues. Make sure arch timer is
 * enabled and configured to use proper frequency.
 */
static void rcar_gen2_timer_init(void)
{
	u32 freq = RMOBILE_XTAL_CLK / 2;

	/*
	 * Update the arch timer if it is either not running, or is not at the
	 * right frequency.
	 */
	if ((readl(TIMER_BASE + TIMER_CNTCR) & 1) == 0 ||
			readl(TIMER_BASE + TIMER_CNTFID0) != freq) {
		/* Update registers with correct frequency */
		writel(freq, TIMER_BASE + TIMER_CNTFID0);
		asm volatile("mcr p15, 0, %0, c14, c0, 0" : : "r" (freq));

		/* Make sure arch timer is started by setting bit 0 of CNTCR */
		writel(1, TIMER_BASE + TIMER_CNTCR);
	}
}

/*
 * In order not to break compilation if someone decides to build with PSCI
 * support disabled, keep these dummy functions.
 */
void smp_set_core_boot_addr(unsigned long addr, int corenr)
{
}

void smp_kick_all_cpus(void)
{
}

void smp_waitloop(unsigned previous_address)
{
}
#endif

#define ETHERNET_PHY_RESET	185	/* GPIO 5 31 */

int board_init(void)
{
	/* adress of boot parameters */
	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;

	/* Force ethernet PHY out of reset */
	gpio_request(ETHERNET_PHY_RESET, "phy_reset");
	gpio_direction_output(ETHERNET_PHY_RESET, 0);
	mdelay(10);
	gpio_direction_output(ETHERNET_PHY_RESET, 1);

#ifdef CONFIG_ARMV7_NONSEC
	rcar_gen2_timer_init();
#endif

	return 0;
}

int dram_init(void)
{
	if (fdtdec_setup_memory_size() != 0)
		return -EINVAL;

	return 0;
}

int dram_init_banksize(void)
{
	fdtdec_setup_memory_banksize();

	return 0;
}

/* KSZ8041NL/RNL */
#define PHY_CONTROL1		0x1E
#define PHY_LED_MODE		0xC0000
#define PHY_LED_MODE_ACK	0x4000
int board_phy_config(struct phy_device *phydev)
{
	int ret = phy_read(phydev, MDIO_DEVAD_NONE, PHY_CONTROL1);
	ret &= ~PHY_LED_MODE;
	ret |= PHY_LED_MODE_ACK;
	ret = phy_write(phydev, MDIO_DEVAD_NONE, PHY_CONTROL1, (u16)ret);

	return 0;
}

void reset_cpu(ulong addr)
{
	struct udevice *dev;
	const u8 pmic_bus = 2;
	const u8 pmic_addr = 0x58;
	u8 data;
	int ret;

	ret = i2c_get_chip_for_busnum(pmic_bus, pmic_addr, 1, &dev);
	if (ret)
		hang();

	ret = dm_i2c_read(dev, 0x13, &data, 1);
	if (ret)
		hang();

	data |= BIT(1);

	ret = dm_i2c_write(dev, 0x13, &data, 1);
	if (ret)
		hang();
}

enum env_location env_get_location(enum env_operation op, int prio)
{
	const u32 load_magic = 0xb33fc0de;

	/* Block environment access if loaded using JTAG */
	if ((readl(CONFIG_SPL_TEXT_BASE + 0x24) == load_magic) &&
	    (op != ENVOP_INIT))
		return ENVL_UNKNOWN;

	if (prio)
		return ENVL_UNKNOWN;

	return ENVL_SPI_FLASH;
}
