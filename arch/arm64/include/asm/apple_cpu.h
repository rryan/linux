/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * CPU power control for Apple SoCs.
 */
#ifndef __ASM_APPLE_CPU_H
#define __ASM_APPLE_CPU_H

#include <linux/bits.h>
#include <linux/compiler_attributes.h>
#include <linux/errno.h>
#include <linux/types.h>

/* Keep IRQs and FIQs from waking the core out of its power-down WFI. */
#define APPLE_CPU_SLEEP_MASK_IRQ	BIT(0)
/* Allow the whole CPU complex to enter deep sleep at this WFI. */
#define APPLE_CPU_SLEEP_ACC_DEEP	BIT(1)

#ifdef CONFIG_ARM64_APPLE_CPU_HOTPLUG
void __noreturn apple_cpu_deep_sleep(unsigned int flags);
int apple_cpu_acc_save(void);
int apple_cpu_acc_restore(void);
int apple_cpu_park_power_down_all(void);
void apple_cpu_power_cycled(void);

/* The park loop of an offline CPU, internal to the CPU hotplug code */
u64 apple_cpu_park_wfi(u64 *cmd, u64 cyc, u64 ways, u64 sets, u64 way_shift,
		       u64 line_shift);
#else
static inline int apple_cpu_acc_save(void)
{
	return -ENODEV;
}

static inline int apple_cpu_acc_restore(void)
{
	return -ENODEV;
}

static inline int apple_cpu_park_power_down_all(void)
{
	return 0;
}

static inline void apple_cpu_power_cycled(void)
{
}
#endif

#endif /* __ASM_APPLE_CPU_H */
