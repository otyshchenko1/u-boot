// SPDX-License-Identifier: GPL-2.0+
/*
 * CPU power management support for Renesas r8a7790 SoC
 *
 * Contains functions to control ARM Cortex A15/A7 cores and
 * related peripherals basically used for PSCI.
 *
 * Copyright (C) 2018 EPAM Systems Inc.
 *
 * Mainly based on Renesas R-Car Gen2 platform code from Linux:
 *    arch/arm/mach-shmobile/...
 */

#include <common.h>
#include <asm/secure.h>
#include <asm/io.h>

#include "pm-r8a7790.h"

/*****************************************************************************
 * APMU definitions
 *****************************************************************************/
#define CA15_APMU_BASE	0xe6152000
#define CA7_APMU_BASE	0xe6151000

/* Wake Up Control Register */
#define WUPCR_OFFS		0x10
/* Power Status Register */
#define PSTR_OFFS		0x40
/* CPUn Power Status Control Register */
#define CPUNCR_OFFS(n)	(0x100 + (0x10 * (n)))

#define CPUPWR_STANDBY	0x3

/*****************************************************************************
 * RST definitions
 *****************************************************************************/
#define RST_BASE	0xe6160000

/* Boot Address Registers */
#define CA15BAR		0x20
#define CA7BAR		0x30

/* Reset Control Registers */
#define CA15RESCNT	0x40
#define CA7RESCNT	0x44

#define CA15RESCNT_CODE	0xa5a50000
#define CA7RESCNT_CODE	0x5a5a0000

/* SYS Boot Address Register */
#define SBAR_BAREN	BIT(4)	/* SBAR is valid */

/* Watchdog Timer Reset Control Register */
#define WDTRSTCR		0x54

#define WDTRSTCR_CODE	0xa55a0000

/*****************************************************************************
 * SYSC definitions
 *****************************************************************************/
#define SYSC_BASE	0xe6180000

/* SYSC Status Register */
#define SYSCSR		0x00
/* Interrupt Status Register */
#define SYSCISR		0x04
/* Interrupt Status Clear Register */
#define SYSCISCR	0x08
/* Interrupt Enable Register */
#define SYSCIER		0x0c
/* Interrupt Mask Register */
#define SYSCIMR		0x10

/* Power Status Register */
#define PWRSR_OFFS		0x00
/* Power Resume Control Register */
#define PWRONCR_OFFS	0x0c
/* Power Shutoff/Resume Error Register */
#define PWRER_OFFS		0x14

/* PWRSR5 .. PWRER5 */
#define CA15_SCU_CHAN_OFFS	0x180
/* PWRSR3 .. PWRER3 */
#define CA7_SCU_CHAN_OFFS	0x100

#define CA15_SCU_ISR_BIT	12
#define CA7_SCU_ISR_BIT		21

#define SYSCSR_RETRIES		100
#define SYSCSR_DELAY_US		1

#define PWRER_RETRIES		100
#define PWRER_DELAY_US		1

#define SYSCISR_RETRIES		1000
#define SYSCISR_DELAY_US	1

#define CA15_SCU	0
#define CA7_SCU		1

/*****************************************************************************
 * CCI-400 definitions
 *****************************************************************************/
#define CCI_BASE		0xf0090000
#define CCI_SLAVE3		0x4000
#define CCI_SLAVE4		0x5000
#define CCI_SNOOP		0x0000
#define CCI_STATUS		0x000c
#define CCI_ENABLE_REQ	0x0003

/*****************************************************************************
 * RWDT definitions
 *****************************************************************************/
/* Watchdog Timer Counter Register */
#define RWTCNT		0x0
/* Watchdog Timer Control/Status Register A */
#define RWTCSRA		0x4

#define RWTCNT_CODE		0x5a5a0000
#define RWTCSRA_CODE	0xa5a5a500

#define RWTCSRA_WRFLG	BIT(5)
#define RWTCSRA_TME		BIT(7)

/*****************************************************************************
 * Other definitions
 *****************************************************************************/
/* On-chip RAM */
#define ICRAM1	0xe63c0000	/* Inter Connect RAM1 (4 KiB) */

/* On-chip ROM */
#define BOOTROM	0xe6340000

/*****************************************************************************
 * Functions which intended to be called from PSCI handlers. These functions
 * marked as __secure and are placed in .secure section.
 *****************************************************************************/
void __secure r8a7790_apmu_power_on(u32 cpu)
{
	u32 cluster = r8a7790_cluster_id(cpu);
	u32 apmu_base;

	apmu_base = cluster == 0 ? CA15_APMU_BASE : CA7_APMU_BASE;

	/* Request power on */
	writel(BIT(r8a7790_core_id(cpu)), apmu_base + WUPCR_OFFS);

	/* Wait for APMU to finish */
	while (readl(apmu_base + WUPCR_OFFS))
		;
}

void __secure r8a7790_apmu_power_off(u32 cpu)
{
	u32 cluster = r8a7790_cluster_id(cpu);
	u32 apmu_base;

	apmu_base = cluster == 0 ? CA15_APMU_BASE : CA7_APMU_BASE;

	/* Request Core Standby for next WFI */
	writel(CPUPWR_STANDBY, apmu_base + CPUNCR_OFFS(r8a7790_core_id(cpu)));
}

void __secure r8a7790_assert_reset(u32 cpu)
{
	u32 cluster = r8a7790_cluster_id(cpu);
	u32 mask, magic, rescnt;

	mask = BIT(3 - r8a7790_core_id(cpu));
	magic = cluster == 0 ? CA15RESCNT_CODE : CA7RESCNT_CODE;
	rescnt = RST_BASE + (cluster == 0 ? CA15RESCNT : CA7RESCNT);
	writel((readl(rescnt) | mask) | magic, rescnt);
}

void __secure r8a7790_deassert_reset(u32 cpu)
{
	u32 cluster = r8a7790_cluster_id(cpu);
	u32 mask, magic, rescnt;

	mask = BIT(3 - r8a7790_core_id(cpu));
	magic = cluster == 0 ? CA15RESCNT_CODE : CA7RESCNT_CODE;
	rescnt = RST_BASE + (cluster == 0 ? CA15RESCNT : CA7RESCNT);
	writel((readl(rescnt) & ~mask) | magic, rescnt);
}

void __secure r8a7790_system_reset(void)
{
	u32 bar;

	/*
	 * Before configuring internal watchdog timer (RWDT) to reboot the system
	 * we need to re-program BAR registers for the boot CPU to jump to bootrom
	 * code. Without taking care of, the boot CPU will jump to the reset vector
	 * previously installed in ICRAM1, since BAR registers keep their values
	 * after watchdog triggered reset.
	 */
	bar = (BOOTROM >> 8) & 0xfffffc00;
	writel(bar, RST_BASE + CA15BAR);
	writel(bar | SBAR_BAREN, RST_BASE + CA15BAR);
	writel(bar, RST_BASE + CA7BAR);
	writel(bar | SBAR_BAREN, RST_BASE + CA7BAR);
	dsb();

	/* Now, configure watchdog timer to reboot the system */

	/* Trigger reset when counter overflows */
	writel(WDTRSTCR_CODE | 0x2, RST_BASE + WDTRSTCR);
	dsb();

	/* Stop counter */
	writel(RWTCSRA_CODE, RWDT_BASE + RWTCSRA);

	/* Initialize counter with the highest value  */
	writel(RWTCNT_CODE | 0xffff, RWDT_BASE + RWTCNT);

	while (readb(RWDT_BASE + RWTCSRA) & RWTCSRA_WRFLG)
		;

	/* Start counter */
	writel(RWTCSRA_CODE | RWTCSRA_TME, RWDT_BASE + RWTCSRA);
}

/*****************************************************************************
 * Functions which intended to be called from PSCI board initialization.
 *****************************************************************************/
static int sysc_power_up(u8 scu)
{
	u32 status, chan_offs, isr_bit;
	int i, j, ret = 0;

	if (scu == CA15_SCU) {
		chan_offs = CA15_SCU_CHAN_OFFS;
		isr_bit = CA15_SCU_ISR_BIT;
	} else {
		chan_offs = CA7_SCU_CHAN_OFFS;
		isr_bit = CA7_SCU_ISR_BIT;
	}

	writel(BIT(isr_bit), SYSC_BASE + SYSCISCR);

	/* Submit power resume request until it was accepted */
	for (i = 0; i < PWRER_RETRIES; i++) {
		/* Wait until SYSC is ready to accept a power resume request */
		for (j = 0; j < SYSCSR_RETRIES; j++) {
			if (readl(SYSC_BASE + SYSCSR) & BIT(1))
				break;

			udelay(SYSCSR_DELAY_US);
		}

		if (j == SYSCSR_RETRIES)
			return -EAGAIN;

		/* Submit power resume request */
		writel(BIT(0), SYSC_BASE + chan_offs + PWRONCR_OFFS);

		/* Check if power resume request was accepted */
		status = readl(SYSC_BASE + chan_offs + PWRER_OFFS);
		if (!(status & BIT(0)))
			break;

		udelay(PWRER_DELAY_US);
	}

	if (i == PWRER_RETRIES)
		return -EIO;

	/* Wait until the power resume request has completed */
	for (i = 0; i < SYSCISR_RETRIES; i++) {
		if (readl(SYSC_BASE + SYSCISR) & BIT(isr_bit))
			break;
		udelay(SYSCISR_DELAY_US);
	}

	if (i == SYSCISR_RETRIES)
		ret = -EIO;

	writel(BIT(isr_bit), SYSC_BASE + SYSCISCR);

	return ret;
}

static bool sysc_power_is_off(u8 scu)
{
	u32 status, chan_offs;

	chan_offs = scu == CA15_SCU ? CA15_SCU_CHAN_OFFS : CA7_SCU_CHAN_OFFS;

	/* Check if SCU is in power shutoff state */
	status = readl(SYSC_BASE + chan_offs + PWRSR_OFFS);
	if (status & BIT(0))
		return true;

	return false;
}

/*
 * Reset vector for secondary CPUs.
 * This will be mapped at address 0 by SBAR register.
 * We need _long_ jump to the physical address.
 */
asm (
	"	.arm \n"
	"	.align 12 \n"
	"	.globl shmobile_boot_vector \n"
	"shmobile_boot_vector: \n"
	"	ldr r1, 1f \n"
	"	bx	r1 \n"
	"	.type shmobile_boot_vector, %function \n"
	"	.size shmobile_boot_vector, .-shmobile_boot_vector \n"
	"	.align	2 \n"
	"	.globl	shmobile_boot_fn \n"
	"shmobile_boot_fn: \n"
	"1:	.space	4 \n"
	"	.globl	shmobile_boot_size \n"
	"shmobile_boot_size: \n"
	"	.long	.-shmobile_boot_vector \n");

extern void shmobile_boot_vector(void);
extern unsigned long shmobile_boot_fn;
extern unsigned long shmobile_boot_size;

void r8a7790_prepare_secondary_cpus(unsigned long addr)
{
	u32 boot_cpu = get_current_cpu();
	u32 bar;
	int i, ret = 0;

	shmobile_boot_fn = addr;
	dsb();

	/* Install reset vector */
	memcpy_toio((void __iomem *)ICRAM1, shmobile_boot_vector,
			shmobile_boot_size);
	dmb();

	/* Setup reset vectors */
	bar = (ICRAM1 >> 8) & 0xfffffc00;
	writel(bar, RST_BASE + CA15BAR);
	writel(bar | SBAR_BAREN, RST_BASE + CA15BAR);
	writel(bar, RST_BASE + CA7BAR);
	writel(bar | SBAR_BAREN, RST_BASE + CA7BAR);
	dsb();

	/*
	 * Indicate the completion status of power shutoff/resume procedure
	 * for CA15/CA7 SCU.
	 */
	writel(BIT(CA15_SCU_ISR_BIT) | BIT(CA7_SCU_ISR_BIT), SYSC_BASE + SYSCIER);

	/* Power on CA15/CA7 SCU */
	if (sysc_power_is_off(CA15_SCU))
		ret += sysc_power_up(CA15_SCU);
	if (sysc_power_is_off(CA7_SCU))
		ret += sysc_power_up(CA7_SCU);
	if (ret)
		printk("warning: some of secondary CPUs may not boot\n");

	/* Keep secondary CPUs in reset */
	for (i = 0; i < R8A7790_PSCI_NR_CPUS; i++) {
		/* Make sure we don't reset the boot CPU */
		if (boot_cpu == i)
			continue;

		r8a7790_assert_reset(i);
	}

	/*
	 * Enable snoop requests and DVM message requests for slave interfaces
	 * S4 (CA7) and S3 (CA15).
	 */
	writel(readl(CCI_BASE + CCI_SLAVE3 + CCI_SNOOP) | CCI_ENABLE_REQ,
		CCI_BASE + CCI_SLAVE3 + CCI_SNOOP);
	writel(readl(CCI_BASE + CCI_SLAVE4 + CCI_SNOOP) | CCI_ENABLE_REQ,
		CCI_BASE + CCI_SLAVE4 + CCI_SNOOP);
	/* Wait for pending bit low */
	while (readl(CCI_BASE + CCI_STATUS))
		;
}
