// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 EPAM Systems Inc.
 */

#ifndef __PM_R8A7790_H__
#define __PM_R8A7790_H__

#include <linux/types.h>

#define R8A7790_PSCI_NR_CPUS	8

void r8a7790_apmu_power_on(u32 cpu);
void r8a7790_apmu_power_off(u32 cpu);
int r8a7790_apmu_power_off_poll(u32 cpu);
void r8a7790_assert_reset(u32 cpu);
void r8a7790_deassert_reset(u32 cpu);

void r8a7790_prepare_secondary_cpus(unsigned long addr);
void r8a7790_system_reset(void);

#define r8a7790_cluster_id(cpu)	(((cpu) >> 2) & 0x1)
#define r8a7790_core_id(cpu)	((cpu) & 0x3)

/* The inline functions below can be called from .secure section */
static inline u32 cp15_read_mpidr(void)
{
	u32 mpidr;

	asm volatile("mrc p15, 0, %0, c0, c0, 5" : "=r"(mpidr));

	return mpidr;
}

/*
 * Convert target CPU ID in MPIDR format to CPU index:
 * MPIDR[1:0]  = Core ID in cluster (0,1,2,3)
 * MPIDR[11:8] = Cluster ID (0,1)
 *
 * Below the possible CPU IDs and corresponding CPU indexes:
 *
 * CPU ID       CPU index
 * 0x80000000 - 0x00000000
 * 0x80000001 - 0x00000001
 * 0x80000002 - 0x00000002
 * 0x80000003 - 0x00000003
 * 0x80000100 - 0x00000004
 * 0x80000101 - 0x00000005
 * 0x80000102 - 0x00000006
 * 0x80000103 - 0x00000007
 */
static inline u32 mpidr_to_cpu(u32 mpidr)
{
	mpidr |= mpidr >> 6;
	mpidr &= 0x7;

	return mpidr;
}

/* Return an index of the CPU which performs this call */
static inline u32 get_current_cpu(void)
{
	return mpidr_to_cpu(cp15_read_mpidr());
}

#endif /* __PM_R8A7790_H__ */
