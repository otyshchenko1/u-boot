// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 EPAM Systems Inc.
 */

#ifndef __PM_R8A7790_H__
#define __PM_R8A7790_H__

#include <linux/types.h>

void r8a7790_apmu_power_on(int cpu);
void r8a7790_apmu_power_off(int cpu);
int r8a7790_apmu_power_off_poll(int cpu);
void r8a7790_assert_reset(int cpu);
void r8a7790_deassert_reset(int cpu);

void r8a7790_prepare_cpus(unsigned long addr, u32 nr_cpus);
void r8a7790_system_reset(void);

#define R8A7790_NR_CLUSTERS				2
#define R8A7790_NR_CPUS_PER_CLUSTER		4

/* Convert linear CPU index to core/cluster ID */
#define r8a7790_cluster_id(cpu)	((cpu) / R8A7790_NR_CPUS_PER_CLUSTER)
#define r8a7790_core_id(cpu)	((cpu) % R8A7790_NR_CPUS_PER_CLUSTER)

#define MPIDR_AFFLVL_MASK	GENMASK(7, 0)
#define MPIDR_AFF0_SHIFT	0
#define MPIDR_AFF1_SHIFT	8

/* All functions are inline so that they can be called from .secure section. */

/*
 * Convert CPU ID in MPIDR format to linear CPU index.
 *
 * Below the possible CPU IDs and corresponding CPU indexes:
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
static inline int mpidr_to_cpu_index(u32 mpidr)
{
	u32 cluster_id, cpu_id;

	cluster_id = (mpidr >> MPIDR_AFF1_SHIFT) & MPIDR_AFFLVL_MASK;
	cpu_id = (mpidr >> MPIDR_AFF0_SHIFT) & MPIDR_AFFLVL_MASK;

	if (cluster_id >= R8A7790_NR_CLUSTERS)
		return -1;

	if (cpu_id >= R8A7790_NR_CPUS_PER_CLUSTER)
		return -1;

	return (cpu_id + (cluster_id * R8A7790_NR_CPUS_PER_CLUSTER));
}

/* Return an index of the CPU which performs this call */
static inline int get_current_cpu(void)
{
	u32 mpidr;

	asm volatile("mrc p15, 0, %0, c0, c0, 5" : "=r"(mpidr));

	return mpidr_to_cpu_index(mpidr);
}

#endif /* __PM_R8A7790_H__ */
