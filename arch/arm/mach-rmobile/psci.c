// SPDX-License-Identifier: GPL-2.0+
/*
 * This file implements basic PSCI support for Renesas r8a7790 SoC
 *
 * Copyright (C) 2018 EPAM Systems Inc.
 *
 * Based on arch/arm/mach-uniphier/arm32/psci.c
 */

#include <common.h>
#include <linux/psci.h>
#include <asm/io.h>
#include <asm/psci.h>
#include <asm/secure.h>

#include "pm-r8a7790.h"

#define GICC_CTLR_OFFSET	0x2000

u32 __secure psci_get_cpu_id(void)
{
	return get_current_cpu();
}

u32 __secure psci_version(void)
{
	return ARM_PSCI_VER_1_0;
}

int __secure psci_cpu_on(u32 function_id, u32 mpidr, u32 entry_point)
{
	u32 cpu = mpidr_to_cpu(mpidr);

	psci_save_target_pc(cpu, entry_point);

	r8a7790_assert_reset(cpu);
	r8a7790_apmu_power_on(cpu);
	r8a7790_deassert_reset(cpu);

	return PSCI_RET_SUCCESS;
}

void __secure psci_cpu_off(void)
{
	u32 cpu = get_current_cpu();

	/*
	 * Place the CPU interface in a state where it can never make a CPU exit
	 * WFI as result of an asserted interrupt.
	 */
	writel(0, CONFIG_ARM_GIC_BASE_ADDRESS + GICC_CTLR_OFFSET);
	dsb();

	/* Select next sleep mode using the APMU */
	r8a7790_apmu_power_off(cpu);

	/* Do ARM specific CPU shutdown */
	psci_cpu_off_common();

	/* Drain the WB before WFI */
	dsb();

	while (1)
		wfi();
}

void __secure psci_system_reset(u32 function_id)
{
	r8a7790_system_reset();

	/* Drain the WB before WFI */
	dsb();

	/* The system is about to be rebooted, so just waiting for this */
	while (1)
		wfi();
}

void __secure psci_system_off(u32 function_id)
{
	/* Drain the WB before WFI */
	dsb();

	/* System Off is not implemented yet, so waiting for powering off manually */
	while (1)
		wfi();
}

void psci_board_init(void)
{
	r8a7790_prepare_secondary_cpus((unsigned long)psci_cpu_entry);
}
