/*
 * board/renesas/r8a7790stout/r8a7790stout.c
 *     This file is r8a7790-stout board support.
 *
 * Copyright (C) 2015 Renesas Electronics Europe GmbH
 * Copyright (C) 2015 Renesas Electronics Corporation
 * Copyright (C) 2015 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <common.h>
#include <malloc.h>
#include <asm/processor.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/arch/rmobile.h>
#include <netdev.h>
#include <i2c.h>
#include "r8a7790stout.h"

DECLARE_GLOBAL_DATA_PTR;

#define PLL0CR		0xE61500D8
#define PLLECR		0xE61500D0
#define PLL0ST		0x100

void s_init(void)
{
	struct r8a7790_rwdt *rwdt = (struct r8a7790_rwdt *)RWDT_BASE;
	struct r8a7790_swdt *swdt = (struct r8a7790_swdt *)SWDT_BASE;
	u32 val;
	u32 pll0_status;

	/* Watchdog init */
	writel(0xA5A5A500, &rwdt->rwtcsra);
	writel(0xA5A5A500, &swdt->swtcsra);

	/* cpu frequency setting */
	if (rmobile_get_cpu_rev_integer() >= R8A7790_CUT_ES2X) {
		val = readl(PLL0CR);
		val &= ~0x7F000000;
		val |= 0x45000000; /* 1.4GHz */
		writel(val, PLL0CR);

		do {
			pll0_status = readl(PLLECR) & PLL0ST;
		} while (pll0_status == 0x0);
	}

	/* QoS */
#if !(defined(CONFIG_EXTRAM_BOOT))
	qos_init();
#endif
}

#define TMU0_MSTP125	(1 << 25)
#define SCIFA0_MSTP204	(1 << 4)
#define SDHI0_MSTP314	(1 << 14)
#define SDHI2_MSTP312	(1 << 12)
#define ETHER_MSTP813	(1 << 13)

#define SD2CKCR		0xE6150078
#define SD2_97500KHZ	0x7

int board_early_init_f(void)
{
	u32 val;

	/* TMU0 */
	val = readl(MSTPSR1);
	val &= ~TMU0_MSTP125;
	writel(val, SMSTPCR1);

	/* SCIFA0 */
	val = readl(MSTPSR2);
	val &= ~SCIFA0_MSTP204;
	writel(val, SMSTPCR2);

	/* ETHER */
	val = readl(MSTPSR8);
	val &= ~ETHER_MSTP813;
	writel(val, SMSTPCR8);

	/* SD */
	val = readl(MSTPSR3);
	val &= ~(SDHI0_MSTP314 | SDHI2_MSTP312);
	writel(val, SMSTPCR3);

	/*
	 * SD0 clock is set to 97.5MHz by default.
	 * Set SD2 to the 97.5MHz as well.
	 */
	writel(SD2_97500KHZ, SD2CKCR);

	return 0;
}

#ifdef CONFIG_ARMV7_VIRT
static int shmobile_init_time(void);
#endif

int board_init(void)
{
	u32 val;
	u8 val_i2c;

	/* adress of boot parameters */
	gd->bd->bi_boot_params = R8A7790STOUT_SDRAM_BASE + 0x100;

	/* Init PFC controller */
	r8a7790_pinmux_init();

	/* CPLD setup */
	cpld_init();

	sh_timer_init();

#ifdef CONFIG_SH_ETHER
	/* ETHER Enable */
	gpio_request(GPIO_FN_ETH_CRS_DV, NULL);
	gpio_request(GPIO_FN_ETH_RX_ER, NULL);
	gpio_request(GPIO_FN_ETH_RXD0, NULL);
	gpio_request(GPIO_FN_ETH_RXD1, NULL);
	gpio_request(GPIO_FN_ETH_LINK, NULL);
	gpio_request(GPIO_FN_ETH_REF_CLK, NULL);
	gpio_request(GPIO_FN_ETH_MDIO, NULL);
	gpio_request(GPIO_FN_ETH_TXD1, NULL);
	gpio_request(GPIO_FN_ETH_TX_EN, NULL);
	gpio_request(GPIO_FN_ETH_MAGIC, NULL);
	gpio_request(GPIO_FN_ETH_TXD0, NULL);
	gpio_request(GPIO_FN_ETH_MDC, NULL);
	gpio_request(GPIO_FN_IRQ1, NULL);

	gpio_request(GPIO_GP_3_31, NULL);	/* PHY_RST */
	gpio_direction_output(GPIO_GP_3_31, 0);
	mdelay(20);
	gpio_set_value(GPIO_GP_3_31, 1);
	udelay(1);
#endif

#ifdef CONFIG_SH_SDHI
	gpio_request(GPIO_FN_SD0_DAT0, NULL);
	gpio_request(GPIO_FN_SD0_DAT1, NULL);
	gpio_request(GPIO_FN_SD0_DAT2, NULL);
	gpio_request(GPIO_FN_SD0_DAT3, NULL);
	gpio_request(GPIO_FN_SD0_CLK, NULL);
	gpio_request(GPIO_FN_SD0_CMD, NULL);
	gpio_request(GPIO_FN_SD0_CD, NULL);
	gpio_request(GPIO_FN_SD2_DAT0, NULL);
	gpio_request(GPIO_FN_SD2_DAT1, NULL);
	gpio_request(GPIO_FN_SD2_DAT2, NULL);
	gpio_request(GPIO_FN_SD2_DAT3, NULL);
	gpio_request(GPIO_FN_SD2_CLK, NULL);
	gpio_request(GPIO_FN_SD2_CMD, NULL);
	gpio_request(GPIO_FN_SD2_CD, NULL);

	/* sdhi0 - needs CPLD mux setup */
	gpio_request(GPIO_GP_3_30, NULL);
	gpio_direction_output(GPIO_GP_3_30, 1); /* VLDO3=3.3V */
	gpio_request(GPIO_GP_5_24, NULL);
	gpio_direction_output(GPIO_GP_5_24, 1); /* power on */
	/* sdhi2 - needs CPLD mux setup */
	gpio_request(GPIO_GP_3_29, NULL);
	gpio_direction_output(GPIO_GP_3_29, 1); /* VLDO4=3.3V */
	gpio_request(GPIO_GP_5_25, NULL);
	gpio_direction_output(GPIO_GP_5_25, 1); /* power on */
#endif

#ifdef CONFIG_USB_EHCI
	gpio_request(GPIO_FN_USB0_PWEN, NULL);
	gpio_request(GPIO_FN_USB0_OVC_VBUS, NULL);
	gpio_request(GPIO_FN_USB1_PWEN, NULL);
	gpio_request(GPIO_FN_USB1_OVC, NULL);
	gpio_request(GPIO_FN_USB2_PWEN, NULL);
	gpio_request(GPIO_FN_USB2_OVC, NULL);
#endif

	do {
		val = readl(0xE6600B0C) & 0xF;
	} while (val != 0x2);
	writel(0x2, 0xE6600B80);
	do {
		val = readl(0xE6600A14) & 0x1;
	} while (val != 0x0);
	writel(0x0, 0xE660012C);

	do {
		val = readl(0xE6620B0C) & 0xF;
	} while (val != 0x2);
	writel(0x2, 0xE6620B80);
	do {
		val = readl(0xE6620A14) & 0x1;
	} while (val != 0x0);
	writel(0x0, 0xE662012C);

	/* wait 5ms */
	udelay(5000);

#ifdef CONFIG_ARMV7_VIRT
	/* init timer */
	shmobile_init_time();
#endif

	return 0;
}

int board_eth_init(bd_t *bis)
{
	int ret = -ENODEV;
	u32 val;
	unsigned char enetaddr[6];

#ifdef CONFIG_SH_ETHER
	ret = sh_eth_initialize(bis);
	if (!eth_getenv_enetaddr("ethaddr", enetaddr))
		return ret;

	/* Set Mac address */
	val = enetaddr[0] << 24 | enetaddr[1] << 16 |
	    enetaddr[2] << 8 | enetaddr[3];
	writel(val, 0xEE7003C0);

	val = enetaddr[4] << 8 | enetaddr[5];
	writel(val, 0xEE7003C8);
#endif

	return ret;
}

int dram_init(void)
{
	gd->ram_size = CONFIG_SYS_SDRAM_SIZE;

	return 0;
}

const struct rmobile_sysinfo sysinfo = {
	CONFIG_RMOBILE_BOARD_STRING
};

void dram_init_banksize(void)
{
	gd->bd->bi_dram[0].start = R8A7790STOUT_SDRAM_BASE;
	gd->bd->bi_dram[0].size = R8A7790STOUT_SDRAM_SIZE;
}

int board_late_init(void)
{
	return 0;
}

int board_mmc_init(bd_t *bis)
{
	int ret = 0;

#ifdef CONFIG_SH_SDHI
	/* SDHI0/SDHI2 */
	ret = sdhi_mmc_init(SDHI0_BASE, 0);
	if (ret)
		return ret;

	ret = sdhi_mmc_init(SDHI2_BASE, 2);
#endif

	return ret;
}

enum {
	MSTP00, MSTP01, MSTP02, MSTP03, MSTP04, MSTP05,
	MSTP07, MSTP08, MSTP09, MSTP10, MSTP11,
	MSTP_NR,
};

struct mstp_ctl {
	u32 s_addr;
	u32 s_dis;
	u32 s_ena;
	u32 r_addr;
	u32 r_dis;
	u32 r_ena;
} mstptbl[MSTP_NR] = {
	[MSTP00] = { SMSTPCR0,  0x00640801, 0x00400000,
		     RMSTPCR0,  0x00640801, 0x00000000 },
	[MSTP01] = { SMSTPCR1,  0xDB6E9BDF, 0x00000000,
		     RMSTPCR1,  0xDB6E9BDF, 0x00000000 },
	[MSTP02] = { SMSTPCR2,  0x300DA1FC, 0x00002010,
		     RMSTPCR2,  0x300DA1FC, 0x00000000 },
	[MSTP03] = { SMSTPCR3,  0xF08CF831, 0x00000000,
		     RMSTPCR3,  0xF08CF831, 0x00000000 },
	[MSTP04] = { SMSTPCR4,  0x80000184, 0x00000180,
		     RMSTPCR4,  0x80000184, 0x00000000 },
	[MSTP05] = { SMSTPCR5,  0x44C00046, 0x00000000,
		     RMSTPCR5,  0x44C00046, 0x00000000 },
	[MSTP07] = { SMSTPCR7,  0x07F30718, 0x00000000,
		     RMSTPCR7,  0x07F30718, 0x00000000 },
	[MSTP08] = { SMSTPCR8,  0x01F0FF84, 0x00000000,
		     RMSTPCR8,  0x01F0FF84, 0x00000000 },
	[MSTP09] = { SMSTPCR9,  0xF5979FCF, 0x00000000,
		     RMSTPCR9,  0xF5979FCF, 0x00000000 },
	[MSTP10] = { SMSTPCR10, 0xFFFEFFE0, 0x00000000,
		     RMSTPCR10, 0xFFFEFFE0, 0x00000000 },
	[MSTP11] = { SMSTPCR11, 0x00000000, 0x00000000,
		     RMSTPCR11, 0x00000000, 0x00000000 },
};

#define TSTR0		4
#define TSTR0_STR0	0x1

void arch_preboot_os()
{
	u32 val;
	int i;

	/* stop TMU0 */
	val = readb(TMU_BASE + TSTR0);
	val &= ~TSTR0_STR0;
	writeb(val, TMU_BASE + TSTR0);

	/* stop all module clock*/
	for (i = MSTP00; i < MSTP_NR; i++) {
		val = readl(mstptbl[i].s_addr);
		writel((val | mstptbl[i].s_dis) & ~(mstptbl[i].s_ena),
		       mstptbl[i].s_addr);
		val = readl(mstptbl[i].r_addr);
		writel((val | mstptbl[i].r_dis) & ~(mstptbl[i].r_ena),
		       mstptbl[i].r_addr);
	}
}

#ifdef CONFIG_ARMV7_VIRT
extern void shmobile_boot_vector(void);
extern unsigned long shmobile_boot_size;

#define r8a7790_clst_id(cpu) ((cpu & 4) > 0 ? 1 : 0)
#define r8a7790_cpu_id(cpu) ((cpu) & 0x3)
#define LAGER_APMU_BASE                        0xE6150000
#define LAGER_APMU_CA15WUPCR_OFFSET            0x2010
#define LAGER_APMU_CA15CPUCMCR_OFFSET          0x2184
#define LAGER_APMU_CA7WUPCR_OFFSET             0x1010
#define LAGER_APMU_CA7CPUCMCR_OFFSET           0x1184
#define LAGER_RST_BASE                         0xE6160000
#define LAGER_RST_CA15BAR_OFFSET               0x20
#define LAGER_RST_CA7BAR_OFFSET                0x30
#define LAGER_RST_CA15BAR_BAREN                (1 << 4)
#define LAGER_RST_CA7BAR_BAREN                 (1 << 4)
#define LAGER_RST_CA15RESCNT_OFFSET            0x40
#define LAGER_RST_CA7RESCNT_OFFSET             0x44
#define		BIT(x)	(1 << (x))
#define LAGER_XEN_INIT_SECONDARY_START         0xE63C0FFC
#define LAGER_RST_BASE                         0xE6160000
#define LAGER_RST_CA15BAR                      0xE6160020
#define LAGER_RST_CA7BAR                       0xE6160030
#define LAGER_LAGER_RAM                        0xE63C0000
#define LAGER_MAX_CPUS                         4
#define TIMER_BASE                  0xE6080000
#define TIMER_CNTCR                 0x0
#define TIMER_CNTFID0               0x20
#define MODEMR                      0xE6160060
#define MD(nr)                      BIT(nr)

static int shmobile_init_time(void)
{
    uint32_t freq;
    int extal_mhz = 0;
    unsigned int mode = readl(MODEMR);

    /* At Linux boot time the r8a7790 arch timer comes up
     * with the counter disabled. Moreover, it may also report
     * a potentially incorrect fixed 13 MHz frequency. To be
     * correct these registers need to be updated to use the
     * frequency EXTAL / 2 which can be determined by the MD pins.
     */

    switch ( mode & (MD(14) | MD(13)) ) {
    case 0:
        extal_mhz = 15;
        break;
    case MD(13):
        extal_mhz = 20;
        break;
    case MD(14):
        extal_mhz = 26;
        break;
    case MD(13) | MD(14):
        extal_mhz = 30;
        break;
    }

    /* The arch timer frequency equals EXTAL / 2 */
    freq = extal_mhz * (1000000 / 2);

    /*
     * Update the timer if it is either not running, or is not at the
     * right frequency. The timer is only configurable in secure mode
     * so this avoids an abort if the loader started the timer and
     * entered the kernel in non-secure mode.
     */

    if ( (readl(TIMER_BASE + TIMER_CNTCR) & 1) == 0 ||
            readl(TIMER_BASE + TIMER_CNTFID0) != freq ) {
        /* Update registers with correct frequency */
        writel(freq, TIMER_BASE + TIMER_CNTFID0);
        asm volatile("mcr p15, 0, %0, c14, c0, 0" : : "r" (freq));

       /* make sure arch timer is started by setting bit 0 of CNTCR */
        writel(1, TIMER_BASE + TIMER_CNTCR);
    }
    return 0;
}

enum { R8A7790_CLST_CA15, R8A7790_CLST_CA7, R8A7790_CLST_NR };
static struct {
	unsigned int wupcr;
	unsigned int bar;
	unsigned int rescnt;
	unsigned int rescnt_magic;
} r8a7790_clst[R8A7790_CLST_NR] = {
	[R8A7790_CLST_CA15] = {
		.wupcr = LAGER_APMU_CA15WUPCR_OFFSET,
		.bar = LAGER_RST_CA15BAR_OFFSET,
		.rescnt = LAGER_RST_CA15RESCNT_OFFSET,
		.rescnt_magic = 0xa5a50000,
},
	[R8A7790_CLST_CA7] = {
		.wupcr = LAGER_APMU_CA7WUPCR_OFFSET,
		.bar = LAGER_RST_CA7BAR_OFFSET,
		.rescnt = LAGER_RST_CA7RESCNT_OFFSET,
		.rescnt_magic = 0x5a5a0000,
	},
};

static void assert_reset(unsigned int cpu)
{
	void *rescnt;
	u32 mask, magic;
	unsigned int clst_id = r8a7790_clst_id(cpu);

	/* disable per-core clocks */
	mask = BIT(3 - r8a7790_cpu_id(cpu));
	magic = r8a7790_clst[clst_id].rescnt_magic;
	rescnt = (void *) (LAGER_RST_BASE + r8a7790_clst[clst_id].rescnt);
	writel((readl(rescnt) | mask) | magic, rescnt);
}

static void deassert_reset(unsigned int cpu)
{
	void *rescnt;
	u32 mask, magic;
	unsigned int clst_id = r8a7790_clst_id(cpu);

	/* enable per-core clocks */
	mask = BIT(3 - r8a7790_cpu_id(cpu));
    magic = r8a7790_clst[clst_id].rescnt_magic;
	rescnt = (void *) (LAGER_RST_BASE + r8a7790_clst[clst_id].rescnt);
	writel((readl(rescnt) & ~mask) | magic, rescnt);
}

static void power_on(unsigned int cpu)
{
	void *cawupcr;
	unsigned int clst_id = r8a7790_clst_id(cpu);

	cawupcr = (void *) (LAGER_APMU_BASE + r8a7790_clst[clst_id].wupcr);
	writel(BIT(r8a7790_cpu_id(cpu)), cawupcr);

	/* wait for APMU to finish */
	while (readl(cawupcr) != 0);
}

void smp_kick_all_cpus(void)
{
	int i;
	for (i = 1; i < LAGER_MAX_CPUS; i++)
	{
		assert_reset(i);
		power_on(i);
		deassert_reset(i);
	}
}

void smp_set_core_boot_addr(unsigned long addr, int corenr)
{

	void __iomem *p;
	unsigned long *f;
	unsigned long bar;

	p = (void __iomem*) LAGER_LAGER_RAM;
	memcpy (p, shmobile_boot_vector, shmobile_boot_size);
	f = (void __iomem *)((long unsigned)p + shmobile_boot_size - 4);
	*((unsigned long *) f) = addr;
	dmb(); /* make sure we have finished io operations */

	bar = (LAGER_LAGER_RAM >> 8) & 0xfffffc00;

	writel(bar, LAGER_RST_CA15BAR);
	writel(bar | 0x10, LAGER_RST_CA15BAR);
	writel(bar, LAGER_RST_CA7BAR);
	writel(bar | 0x10, LAGER_RST_CA7BAR);

	f = (unsigned long *)(LAGER_XEN_INIT_SECONDARY_START);
	*f = 0;

	/* make sure this write is really executed */
	__asm__ volatile ("dsb\n");
}


asm(".arm \n"
	".align 2 \n"
	".global smp_waitloop \n"
	"smp_waitloop: \n"
	"1: 	wfe \n"
	"ldr 	r0, =0xE63C0FFC \n"
	"ldr	r0, [r0] \n"
	"teq	r0, #0x0 \n"
	"beq 	1b \n"

	"b		_do_nonsec_entry \n"
	".type smp_waitloop, %function \n"
	".size smp_waitloop, .-smp_waitloop \n");

asm(
	".arm \n"
	".globl shmobile_boot_vector \n"
	".align 2 \n"
	"shmobile_boot_vector: \n"
	"ldr    pc, 1f \n"
	".type shmobile_boot_vector, %function \n"
	".size shmobile_boot_vector, .-shmobile_boot_vector \n"
    ".align	2 \n"
		"func:\n"
"1:	.space	4 \n"
	".globl	shmobile_boot_size \n"
"shmobile_boot_size: \n"
	".long	.-shmobile_boot_vector \n");
#endif

