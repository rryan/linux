/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __ARM64_KERNEL_SMP_SPIN_TABLE_H
#define __ARM64_KERNEL_SMP_SPIN_TABLE_H

#include <linux/init.h>
#include <linux/types.h>

struct cpu_operations;

extern const struct cpu_operations smp_spin_table_ops;

int smp_spin_table_cpu_init(unsigned int cpu);
int smp_spin_table_cpu_prepare(unsigned int cpu);
int smp_spin_table_cpu_boot(unsigned int cpu);
void smp_spin_table_write_pen_release(u64 val);

/* Entered with the MMU off, at the address cpu_prepare hands out */
void secondary_holding_pen(void);

/*
 * Spin-table CPUs whose power can be controlled through the Apple PMGR
 * use these operations instead, which add CPU hotplug.
 */
extern const struct cpu_operations apple_spin_table_ops;

#ifdef CONFIG_ARM64_APPLE_CPU_HOTPLUG
bool __init apple_spin_table_probe(unsigned int cpu);
#else
static inline bool apple_spin_table_probe(unsigned int cpu)
{
	return false;
}
#endif

#endif /* __ARM64_KERNEL_SMP_SPIN_TABLE_H */
