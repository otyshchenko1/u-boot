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

#define R8A7790_PSCI_NR_CPUS	8
#if R8A7790_PSCI_NR_CPUS > CONFIG_ARMV7_PSCI_NR_CPUS
#error "invalid value for CONFIG_ARMV7_PSCI_NR_CPUS"
#endif

#define GICC_CTLR_OFFSET	0x2000

/*
 * The boot CPU is powered on by default, but it's index is not a const value.
 * An index the boot CPU has, depends on whether it is CA15 (index 0) or
 * CA7 (index 4).
 * So, we update state for the boot CPU during PSCI board initialization.
 */
u8 psci_state[R8A7790_PSCI_NR_CPUS] __secure_data = {
		PSCI_AFFINITY_LEVEL_OFF,
		PSCI_AFFINITY_LEVEL_OFF,
		PSCI_AFFINITY_LEVEL_OFF,
		PSCI_AFFINITY_LEVEL_OFF,
		PSCI_AFFINITY_LEVEL_OFF,
		PSCI_AFFINITY_LEVEL_OFF,
		PSCI_AFFINITY_LEVEL_OFF,
		PSCI_AFFINITY_LEVEL_OFF};

void __secure psci_set_state(int cpu, u8 state)
{
	psci_state[cpu] = state;
	dsb();
	isb();
}

u32 __secure psci_get_cpu_id(void)
{
	return get_current_cpu();
}

void __secure psci_arch_cpu_entry(void)
{
	int cpu = get_current_cpu();

	psci_set_state(cpu, PSCI_AFFINITY_LEVEL_ON);
}

int __secure psci_features(u32 function_id, u32 psci_fid)
{
	switch (psci_fid) {
	case ARM_PSCI_0_2_FN_PSCI_VERSION:
	case ARM_PSCI_0_2_FN_CPU_OFF:
	case ARM_PSCI_0_2_FN_CPU_ON:
	case ARM_PSCI_0_2_FN_AFFINITY_INFO:
	case ARM_PSCI_0_2_FN_MIGRATE_INFO_TYPE:
	case ARM_PSCI_0_2_FN_SYSTEM_OFF:
	case ARM_PSCI_0_2_FN_SYSTEM_RESET:
		return 0x0;
	}

	return ARM_PSCI_RET_NI;
}

u32 __secure psci_version(u32 function_id)
{
	return ARM_PSCI_VER_1_0;
}

int __secure psci_affinity_info(u32 function_id, u32 target_affinity,
		u32 lowest_affinity_level)
{
	int cpu;

	if (lowest_affinity_level > 0)
		return ARM_PSCI_RET_INVAL;

	cpu = mpidr_to_cpu_index(target_affinity);
	if (cpu == -1)
		return ARM_PSCI_RET_INVAL;

	/* TODO flush cache */
	return psci_state[cpu];
}

int __secure psci_migrate_info_type(u32 function_id)
{
	/* Trusted OS is either not present or does not require migration */
	return 2;
}

int __secure psci_cpu_on(u32 function_id, u32 target_cpu, u32 entry_point,
		u32 context_id)
{
	int cpu;

	cpu = mpidr_to_cpu_index(target_cpu);
	if (cpu == -1)
		return ARM_PSCI_RET_INVAL;

	if (psci_state[cpu] == PSCI_AFFINITY_LEVEL_ON)
		return ARM_PSCI_RET_ALREADY_ON;

	if (psci_state[cpu] == PSCI_AFFINITY_LEVEL_ON_PENDING)
		return ARM_PSCI_RET_ON_PENDING;

	psci_save(cpu, entry_point, context_id);

	psci_set_state(cpu, PSCI_AFFINITY_LEVEL_ON_PENDING);

	r8a7790_assert_reset(cpu);
	r8a7790_apmu_power_on(cpu);
	r8a7790_deassert_reset(cpu);

	return ARM_PSCI_RET_SUCCESS;
}

int __secure psci_cpu_off(void)
{
	int cpu = get_current_cpu();

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

	psci_set_state(cpu, PSCI_AFFINITY_LEVEL_OFF);

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
	int cpu = get_current_cpu();

	/* Update state for the boot CPU */
	psci_set_state(cpu, PSCI_AFFINITY_LEVEL_ON);

	/* Perform needed actions for the secondary CPUs to be ready for powering on */
	r8a7790_prepare_cpus((unsigned long)psci_cpu_entry, R8A7790_PSCI_NR_CPUS);
}
