// SPDX-License-Identifier: GPL-2.0-only
/*
 * CPU hotplug for Apple SoCs.
 *
 * Secondary CPUs are started by the boot loader (m1n1) and handed to the
 * kernel through a spin table. There is no PSCI. The PMGR can stop and start
 * individual cores, but on the T6020 family a core that was powered down on
 * its own never fetches its reset vector when it is started again: only the
 * first start after its whole cluster was power cycled works. So:
 *
 *  - A CPU going offline parks in the kernel, at a WFI that powers the core
 *    down with state retention, the state cpuidle uses for idle CPUs. Its L1
 *    data cache is cleaned on the way in and the park loop only reads
 *    memory. Bringing it back sends it a fast IPI: it turns its MMU off
 *    through the identity map and re-enters the holding pen.
 *
 *  - Across suspend to RAM, which power cycles every CPU complex, offline
 *    CPUs power down through the PMGR stop control. On resume they are
 *    started through the PMGR: a core comes out of reset at its RVBAR,
 *    which m1n1 pointed at its resident secondary entry path, and m1n1
 *    jumps to the spin table release address the kernel wrote at boot: the
 *    holding pen.
 *
 * The boot CPU is never offlined.
 */

#define pr_fmt(fmt) "apple-cpu: " fmt

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cpu.h>
#include <linux/cpufeature.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kexec.h>
#include <linux/log2.h>
#include <linux/mmu_context.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/suspend.h>

#include <asm/apple_cpu.h>
#include <asm/barrier.h>
#include <asm/cmpxchg.h>
#include <asm/cpu_ops.h>
#include <asm/cputype.h>
#include <asm/smp_plat.h>
#include <asm/sysreg.h>
#include <asm/virt.h>

#include "smp_spin_table.h"

/* Core power control, see apple_cpu_power_down() and apple_cpu_park() */
#define SYS_IMP_APL_CYC_OVRD		sys_reg(3, 5, 15, 5, 0)
#define CYC_OVRD_DISABLE_WFI_RET	BIT_ULL(0)
#define CYC_OVRD_FIQ_MODE		GENMASK_ULL(21, 20)
#define CYC_OVRD_IRQ_MODE		GENMASK_ULL(23, 22)
#define CYC_OVRD_WFI_MODE		GENMASK_ULL(25, 24)
#define CYC_OVRD_INT_MODE_DISABLE	2
#define CYC_OVRD_WFI_MODE_POWER_DOWN	3

/* CPU complex power control, shared by the cores of a cluster */
#define SYS_IMP_APL_ACC_OVRD		sys_reg(3, 5, 15, 6, 0)
#define ACC_OVRD_PWR_DN_SRM		GENMASK_ULL(14, 13)
#define ACC_OVRD_DIS_L2_FLUSH_ACC_SLEEP	GENMASK_ULL(16, 15)
#define ACC_OVRD_TRAIN_DOWN_LINK	GENMASK_ULL(18, 17)
#define ACC_OVRD_POWER_DOWN_CPM		GENMASK_ULL(26, 25)
#define ACC_OVRD_DISABLE_PIO_ON_WFI_CPU	BIT_ULL(32)
#define ACC_OVRD_DEEP_SLEEP		BIT_ULL(34)

/* Fast IPIs, as sent by the AIC driver */
#define SYS_IMP_APL_IPI_RR_LOCAL_EL1	sys_reg(3, 5, 15, 0, 0)
#define SYS_IMP_APL_IPI_RR_GLOBAL_EL1	sys_reg(3, 5, 15, 0, 1)
#define IPI_RR_CPU			GENMASK_ULL(7, 0)
#define IPI_RR_CLUSTER			GENMASK_ULL(23, 16)

/* PMGR CPU start/stop block ("cpu-start") */
#define CPU_START_STOP			0x0
#define CPU_START_ALIVE			0x4
#define CPU_START_CLUSTER(cl)		(0x8 + 4 * (cl))
#define CPU_START_CORE_BIT(cl, core)	BIT(4 * (cl) + (core))
#define CPU_START_MAX_CLUSTERS		8
#define CPU_START_MAX_CORES		4

/* Per-core implementation registers ("apple,cpu-impl-reg") */
#define IMPL_RVBAR			0x0
#define IMPL_RVBAR_LOCK			BIT_ULL(0)
#define IMPL_RVBAR_ADDR			GENMASK_ULL(47, 12)
#define IMPL_CORE_STATUS		0x100
#define IMPL_CORE_STATUS_RUNNING	GENMASK_ULL(7, 0)
#define IMPL_SIZE			0x1000
#define IMPL_STOP_TIMEOUT_US		50000

/* How long a parked core may take to park or to take a command */
#define PARK_TIMEOUT_US			50000

static const struct of_device_id apple_cpu_ctrl_of_match[] = {
	{ .compatible = "apple,t6020-cpu-ctrl" },
	{}
};

struct apple_cpu_ctrl {
	struct device_node *np;
	void __iomem *start;
};

struct apple_cluster {
	struct apple_cpu_ctrl *ctrl;
	unsigned int idx;
};

enum apple_cpu_state {
	/* Online, or never offlined */
	APPLE_CPU_RUNNING,
	/* Parked in the kernel at a retention WFI, see apple_cpu_park() */
	APPLE_CPU_PARKED,
	/*
	 * Powered down through the PMGR for suspend to RAM, which power cycles
	 * its cluster: the core can be started through the PMGR.
	 */
	APPLE_CPU_OFF,
	/* Released from the park or started, not online yet */
	APPLE_CPU_STARTING,
	/* Did not go down or did not come back as expected; left alone */
	APPLE_CPU_STUCK,
};

/* How cpu_die takes the core down, decided by cpu_disable */
enum apple_cpu_die {
	APPLE_DIE_NONE,
	APPLE_DIE_PARK,
	/* PMGR stop and power-down WFI, for suspend to RAM */
	APPLE_DIE_STOP,
};

/* Commands to a parked core (apple_cpu::park_cmd) */
#define PARK_CMD_NONE			0
/* Leave the park through the holding pen */
#define PARK_CMD_RESUME			1
/* The core took the command */
#define PARK_CMD_TAKEN			3

struct apple_cpu {
	struct apple_cluster *cluster;
	unsigned int core;
	void __iomem *impl;
	u64 rvbar;
	enum apple_cpu_state state;
	enum apple_cpu_die die;
	/* Written by the controlling CPU, read by the parked core */
	u64 park_cmd;
	/* Set by the core once it is about to park */
	bool parked;
};

#define APPLE_MAX_CPU_CTRLS		4
#define APPLE_MAX_CLUSTERS		16

static struct apple_cpu_ctrl apple_cpu_ctrls[APPLE_MAX_CPU_CTRLS];
static unsigned int apple_nr_cpu_ctrls;
static struct apple_cluster apple_clusters[APPLE_MAX_CLUSTERS];
static unsigned int apple_nr_clusters;
static struct apple_cpu *apple_cpus[NR_CPUS];

static void __iomem *apple_cpu_ctrl_map(struct device_node *np,
					const char *name, size_t min_size)
{
	struct resource res;
	int idx;

	idx = of_property_match_string(np, "reg-names", name);
	if (idx < 0)
		return NULL;
	if (of_address_to_resource(np, idx, &res) ||
	    resource_size(&res) < min_size)
		return NULL;

	/* of_iomap() honours the non-posted MMIO these blocks require. */
	return of_iomap(np, idx);
}

static struct apple_cpu_ctrl *apple_cpu_ctrl_get(struct device_node *np)
{
	struct apple_cpu_ctrl *ctrl;
	unsigned int i;

	for (i = 0; i < apple_nr_cpu_ctrls; i++)
		if (apple_cpu_ctrls[i].np == np)
			return &apple_cpu_ctrls[i];

	if (!of_match_node(apple_cpu_ctrl_of_match, np) ||
	    !of_device_is_available(np) ||
	    apple_nr_cpu_ctrls >= APPLE_MAX_CPU_CTRLS)
		return NULL;

	ctrl = &apple_cpu_ctrls[apple_nr_cpu_ctrls];
	ctrl->start = apple_cpu_ctrl_map(np, "cpu-start",
					 CPU_START_CLUSTER(CPU_START_MAX_CLUSTERS));
	if (!ctrl->start)
		return NULL;

	ctrl->np = of_node_get(np);
	apple_nr_cpu_ctrls++;
	return ctrl;
}

static struct apple_cluster *apple_cluster_get(struct apple_cpu_ctrl *ctrl,
					       unsigned int idx)
{
	struct apple_cluster *cl;
	unsigned int i;

	for (i = 0; i < apple_nr_clusters; i++)
		if (apple_clusters[i].ctrl == ctrl && apple_clusters[i].idx == idx)
			return &apple_clusters[i];

	if (apple_nr_clusters >= APPLE_MAX_CLUSTERS)
		return NULL;

	cl = &apple_clusters[apple_nr_clusters++];
	cl->ctrl = ctrl;
	cl->idx = idx;

	return cl;
}

static int apple_cpu_setup(unsigned int cpu)
{
	struct of_phandle_args args;
	struct apple_cpu_ctrl *ctrl;
	struct device_node *np;
	struct apple_cpu *ac;
	u64 impl;
	int ret;

	np = of_get_cpu_node(cpu, NULL);
	if (!np)
		return -ENODEV;

	ret = of_parse_phandle_with_fixed_args(np, "apple,cpu-ctrl", 2, 0, &args);
	if (ret)
		goto out_put;

	ret = -EINVAL;
	if (args.args[0] >= CPU_START_MAX_CLUSTERS ||
	    args.args[1] >= CPU_START_MAX_CORES)
		goto out_put_ctrl;
	if (of_property_read_u64(np, "apple,cpu-impl-reg", &impl))
		goto out_put_ctrl;

	ret = -ENODEV;
	ctrl = apple_cpu_ctrl_get(args.np);
	if (!ctrl)
		goto out_put_ctrl;

	ret = -ENOMEM;
	ac = kzalloc_obj(*ac);
	if (!ac)
		goto out_put_ctrl;

	ac->cluster = apple_cluster_get(ctrl, args.args[0]);
	ac->core = args.args[1];
	ac->impl = ioremap_np(impl, IMPL_SIZE);
	if (!ac->cluster || !ac->impl) {
		if (ac->impl)
			iounmap(ac->impl);
		kfree(ac);
		goto out_put_ctrl;
	}

	apple_cpus[cpu] = ac;
	ret = 0;

out_put_ctrl:
	of_node_put(args.np);
out_put:
	of_node_put(np);
	return ret;
}

bool __init apple_spin_table_probe(unsigned int cpu)
{
	struct of_phandle_args args;
	struct device_node *np;
	bool ok = false;

	/* The boot CPU cannot be restarted into the kernel. */
	if (!cpu)
		return false;

	np = of_get_cpu_node(cpu, NULL);
	if (!np)
		return false;

	if (!of_parse_phandle_with_fixed_args(np, "apple,cpu-ctrl", 2, 0, &args)) {
		ok = of_match_node(apple_cpu_ctrl_of_match, args.np) &&
		     of_device_is_available(args.np) &&
		     of_property_present(np, "apple,cpu-impl-reg");
		of_node_put(args.np);
	}

	of_node_put(np);
	return ok;
}

static int apple_spin_table_cpu_prepare(unsigned int cpu)
{
	int ret;

	ret = smp_spin_table_cpu_prepare(cpu);
	if (ret)
		return ret;

	/* Without the power controls the CPU still boots, it just stays online. */
	if (apple_cpu_setup(cpu))
		pr_warn("CPU%u: no PMGR power control, CPU cannot go offline\n", cpu);

	return 0;
}

/* L1 data cache geometry, for set/way maintenance */
struct apple_l1_sw {
	u64 ways;
	u64 sets;
	u64 way_shift;
	u64 line_shift;
};

static void apple_l1_sw(struct apple_l1_sw *sw)
{
	u64 ccsidr;

	write_sysreg(0, csselr_el1);
	isb();
	ccsidr = read_sysreg(ccsidr_el1);
	if (cpuid_feature_extract_unsigned_field(read_sysreg_s(SYS_ID_AA64MMFR2_EL1),
						 ID_AA64MMFR2_EL1_CCIDX_SHIFT)) {
		sw->ways = FIELD_GET(GENMASK_ULL(23, 3), ccsidr) + 1;
		sw->sets = FIELD_GET(GENMASK_ULL(55, 32), ccsidr) + 1;
	} else {
		sw->ways = FIELD_GET(CCSIDR_EL1_Associativity, ccsidr) + 1;
		sw->sets = FIELD_GET(CCSIDR_EL1_NumSets, ccsidr) + 1;
	}
	sw->line_shift = FIELD_GET(CCSIDR_EL1_LineSize, ccsidr) + 4;
	sw->way_shift = 32 - order_base_2(sw->ways);
}

/*
 * Take this core out of the coherent domain and power it down:
 *
 *  1. clean and invalidate the L1 data cache by set/way (set/way operations
 *     do not reach the shared L2, which stays coherent);
 *  2. request the PMGR to stop the core, if @stop is given;
 *  3. optionally keep IRQs and FIQs from waking the core;
 *  4. make WFI power the core down, without state retention;
 *  5. optionally let the CPU complex enter deep sleep, which also flushes
 *     the L2. This affects every core of the cluster, so it must only be
 *     set by the last core going down;
 *  6. WFI forever.
 *
 * Nothing is written to memory after step 1.
 */
static void __noreturn apple_cpu_power_down(void __iomem *stop, u32 stop_bit,
					    unsigned int flags)
{
	struct apple_l1_sw sw;
	u64 cyc, acc;

	cyc = read_sysreg_s(SYS_IMP_APL_CYC_OVRD);
	if (flags & APPLE_CPU_SLEEP_MASK_IRQ) {
		cyc &= ~(CYC_OVRD_IRQ_MODE | CYC_OVRD_FIQ_MODE);
		cyc |= FIELD_PREP(CYC_OVRD_IRQ_MODE, CYC_OVRD_INT_MODE_DISABLE) |
		       FIELD_PREP(CYC_OVRD_FIQ_MODE, CYC_OVRD_INT_MODE_DISABLE);
	}
	cyc &= ~CYC_OVRD_WFI_MODE;
	cyc |= FIELD_PREP(CYC_OVRD_WFI_MODE, CYC_OVRD_WFI_MODE_POWER_DOWN) |
	       CYC_OVRD_DISABLE_WFI_RET;

	acc = read_sysreg_s(SYS_IMP_APL_ACC_OVRD);
	acc &= ~(ACC_OVRD_PWR_DN_SRM | ACC_OVRD_DIS_L2_FLUSH_ACC_SLEEP |
		 ACC_OVRD_TRAIN_DOWN_LINK | ACC_OVRD_POWER_DOWN_CPM);
	acc |= FIELD_PREP(ACC_OVRD_PWR_DN_SRM, 3) |
	       FIELD_PREP(ACC_OVRD_DIS_L2_FLUSH_ACC_SLEEP, 2) |
	       FIELD_PREP(ACC_OVRD_TRAIN_DOWN_LINK, 3) |
	       FIELD_PREP(ACC_OVRD_POWER_DOWN_CPM, 3) |
	       ACC_OVRD_DISABLE_PIO_ON_WFI_CPU | ACC_OVRD_DEEP_SLEEP;

	apple_l1_sw(&sw);

	asm volatile(
	"	sub	%[way], %[way], #1\n"
	"1:	sub	x10, %[sets], #1\n"
	"2:	lslv	x11, %[way], %[way_shift]\n"
	"	lslv	x12, x10, %[line_shift]\n"
	"	orr	x11, x11, x12\n"
	"	dc	cisw, x11\n"
	"	subs	x10, x10, #1\n"
	"	b.ge	2b\n"
	"	subs	%[way], %[way], #1\n"
	"	b.ge	1b\n"
	"	dsb	sy\n"
	"	cbz	%[stop], 3f\n"
	"	str	%w[stop_bit], [%[stop]]\n"
	"	dsb	sy\n"
	"3:	msr	s3_5_c15_c5_0, %[cyc]\n"
	"	cbz	%[acc_deep], 4f\n"
	"	msr	s3_5_c15_c6_0, %[acc]\n"
	"4:	isb\n"
	"5:	dsb	sy\n"
	"	isb\n"
	"	wfi\n"
	"	b	5b\n"
	: [way] "+r" (sw.ways)
	: [sets] "r" (sw.sets), [way_shift] "r" (sw.way_shift),
	  [line_shift] "r" (sw.line_shift), [stop] "r" (stop),
	  [stop_bit] "r" (stop_bit), [cyc] "r" (cyc), [acc] "r" (acc),
	  [acc_deep] "r" ((u64)(flags & APPLE_CPU_SLEEP_ACC_DEEP))
	: "x10", "x11", "x12", "cc", "memory");

	unreachable();
}

/**
 * apple_cpu_deep_sleep() - power down the calling core
 * @flags: APPLE_CPU_SLEEP_* flags
 *
 * Cleans the L1 data cache and powers the core down at WFI without state
 * retention. The core next runs from its RVBAR. Must be called with
 * interrupts masked.
 */
void __noreturn apple_cpu_deep_sleep(unsigned int flags)
{
	apple_cpu_power_down(NULL, 0, flags);
}
EXPORT_SYMBOL_GPL(apple_cpu_deep_sleep);

/*
 * Runs on the dying CPU, with every exception masked. The core parks at a
 * WFI that powers it down with state retention: WFI_MODE(3) without
 * DISABLE_WFI_RET, the state the cpuidle "CPU PD" state uses. Its interrupt
 * modes are left alone so that a fast IPI (a FIQ) wakes it; device
 * interrupts were moved to other CPUs. The PMGR is not involved.
 *
 * The L1 stays powered and coherent in this state, as for an idle CPU, and
 * is cleaned anyway. The park loop only reads memory, so the core holds no
 * dirty line, whatever happens to its power later.
 */
static void __noreturn apple_cpu_park(struct apple_cpu *ac)
{
	typeof(cpu_soft_restart) *restart;
	struct apple_l1_sw sw;
	u64 cyc, park, cmd;

	cyc = read_sysreg_s(SYS_IMP_APL_CYC_OVRD);
	park = (cyc & ~(CYC_OVRD_WFI_MODE | CYC_OVRD_DISABLE_WFI_RET)) |
	       FIELD_PREP(CYC_OVRD_WFI_MODE, CYC_OVRD_WFI_MODE_POWER_DOWN);
	apple_l1_sw(&sw);

	/* Pairs with the acquire in cpu_kill: everything above is done. */
	smp_store_release(&ac->parked, true);

	do {
		cmd = apple_cpu_park_wfi(&ac->park_cmd, park, sw.ways, sw.sets,
					 sw.way_shift, sw.line_shift);
		/* The controlling CPU may have withdrawn it in the meantime. */
	} while (cmpxchg(&ac->park_cmd, cmd, PARK_CMD_TAKEN) != cmd);

	write_sysreg_s(cyc, SYS_IMP_APL_CYC_OVRD);
	isb();

	/* Into the holding pen with the MMU off, as on first boot */
	cpu_install_idmap();
	restart = (void *)__pa_symbol(cpu_soft_restart);
	restart(is_hyp_nvhe(), __pa_symbol(secondary_holding_pen), 0, 0, 0);
	unreachable();
}

static void apple_cpu_send_ipi(unsigned int cpu)
{
	u64 mpidr = cpu_logical_map(cpu);
	u64 val = FIELD_PREP(IPI_RR_CPU, MPIDR_AFFINITY_LEVEL(mpidr, 0));

	preempt_disable();
	if (MPIDR_AFFINITY_LEVEL(read_cpuid_mpidr(), 1) ==
	    MPIDR_AFFINITY_LEVEL(mpidr, 1))
		write_sysreg_s(val, SYS_IMP_APL_IPI_RR_LOCAL_EL1);
	else
		write_sysreg_s(val | FIELD_PREP(IPI_RR_CLUSTER,
						MPIDR_AFFINITY_LEVEL(mpidr, 1)),
			       SYS_IMP_APL_IPI_RR_GLOBAL_EL1);
	isb();
	preempt_enable();
}

/*
 * Give a parked core a command and wait until it took it. If it does not
 * within PARK_TIMEOUT_US the command is withdrawn and the core stays
 * parked. Does not sleep.
 */
static int apple_cpu_park_command(unsigned int cpu, struct apple_cpu *ac,
				  u64 cmd)
{
	u64 val;
	int ret;

	/*
	 * The pen release written before is visible before the command, and
	 * the command before the IPI arrives.
	 */
	smp_store_release(&ac->park_cmd, cmd);
	dsb(sy);
	apple_cpu_send_ipi(cpu);

	ret = read_poll_timeout_atomic(READ_ONCE, val, val == PARK_CMD_TAKEN,
				       1, PARK_TIMEOUT_US, false, ac->park_cmd);
	if (ret && cmpxchg(&ac->park_cmd, cmd, PARK_CMD_NONE) != cmd)
		ret = 0;
	if (ret)
		pr_err("CPU%u: parked core did not respond\n", cpu);

	return ret;
}

/*
 * Suspend to RAM power cycles every CPU complex, so the secondaries it takes
 * offline power down through the PMGR and can be started again on resume.
 */
static bool apple_cpu_suspending(void)
{
	return cpuhp_tasks_frozen && pm_suspend_target_state == PM_SUSPEND_MEM;
}

static bool apple_spin_table_cpu_can_disable(unsigned int cpu)
{
	return !!apple_cpus[cpu];
}

/* Runs on the dying CPU. */
static int apple_spin_table_cpu_disable(unsigned int cpu)
{
	struct apple_cpu *ac = apple_cpus[cpu];

	if (!ac)
		return -EOPNOTSUPP;

	ac->rvbar = readq(ac->impl + IMPL_RVBAR);
	ac->park_cmd = PARK_CMD_NONE;
	ac->parked = false;
	ac->die = apple_cpu_suspending() ? APPLE_DIE_STOP : APPLE_DIE_PARK;

	return 0;
}

static void apple_spin_table_cpu_die(unsigned int cpu)
{
	struct apple_cpu *ac = apple_cpus[cpu];
	struct apple_cluster *cl;

	/* Early boot failures and crash stops keep parking in the kernel. */
	if (!ac || ac->die == APPLE_DIE_NONE)
		return;

	if (ac->die == APPLE_DIE_PARK)
		apple_cpu_park(ac);

	cl = ac->cluster;
	apple_cpu_power_down(cl->ctrl->start + CPU_START_STOP,
			     CPU_START_CORE_BIT(cl->idx, ac->core),
			     APPLE_CPU_SLEEP_MASK_IRQ);
}

static int apple_spin_table_cpu_kill(unsigned int cpu)
{
	struct apple_cpu *ac = apple_cpus[cpu];
	enum apple_cpu_die die;
	bool parked;
	u64 val;
	int ret;

	if (!ac || ac->die == APPLE_DIE_NONE)
		return 0;

	die = ac->die;
	ac->die = APPLE_DIE_NONE;
	/* Nothing may leave the holding pen until the next cpu_boot. */
	smp_spin_table_write_pen_release(INVALID_HWID);

	if (die == APPLE_DIE_PARK) {
		ret = read_poll_timeout(smp_load_acquire, parked, parked, 100,
					PARK_TIMEOUT_US, false, &ac->parked);
		if (ret) {
			pr_err("CPU%u: core did not park\n", cpu);
			ac->state = APPLE_CPU_STUCK;
			return ret;
		}
		ac->state = APPLE_CPU_PARKED;
		return 0;
	}

	ret = readq_poll_timeout(ac->impl + IMPL_CORE_STATUS, val,
				 !(val & IMPL_CORE_STATUS_RUNNING), 1000,
				 IMPL_STOP_TIMEOUT_US);
	if (ret) {
		pr_err("CPU%u: core did not power down: %#llx\n", cpu, val);
		ac->state = APPLE_CPU_STUCK;
		return ret;
	}
	/* Suspend to RAM power cycles the cluster before the restart. */
	ac->state = APPLE_CPU_OFF;
	return 0;
}

static int apple_cpu_check_rvbar(unsigned int cpu, struct apple_cpu *ac)
{
	u64 val;

	if (!ac->rvbar)
		return 0;

	val = readq(ac->impl + IMPL_RVBAR);
	if ((val & IMPL_RVBAR_ADDR) == (ac->rvbar & IMPL_RVBAR_ADDR))
		return 0;

	if (val & IMPL_RVBAR_LOCK) {
		pr_err("CPU%u: RVBAR locked to %#llx, expected %#llx\n",
		       cpu, val & IMPL_RVBAR_ADDR, ac->rvbar & IMPL_RVBAR_ADDR);
		return -EIO;
	}

	writeq(ac->rvbar, ac->impl + IMPL_RVBAR);
	return 0;
}

static int apple_cpu_unpark(unsigned int cpu, struct apple_cpu *ac)
{
	int ret;

	smp_spin_table_write_pen_release(cpu_logical_map(cpu));

	ret = apple_cpu_park_command(cpu, ac, PARK_CMD_RESUME);
	if (ret) {
		smp_spin_table_write_pen_release(INVALID_HWID);
		return ret;
	}

	ac->state = APPLE_CPU_STARTING;
	return 0;
}

/* Start a powered down core through the PMGR. */
static int apple_cpu_start(unsigned int cpu, struct apple_cpu *ac)
{
	struct apple_cluster *cl = ac->cluster;
	int ret;

	ret = apple_cpu_check_rvbar(cpu, ac);
	if (ret)
		return ret;

	smp_spin_table_write_pen_release(cpu_logical_map(cpu));

	writel(CPU_START_CORE_BIT(cl->idx, ac->core),
	       cl->ctrl->start + CPU_START_ALIVE);
	writel(BIT(ac->core), cl->ctrl->start + CPU_START_CLUSTER(cl->idx));

	sev();

	ac->state = APPLE_CPU_STARTING;
	return 0;
}

static int apple_spin_table_cpu_boot(unsigned int cpu)
{
	struct apple_cpu *ac = apple_cpus[cpu];

	if (!ac)
		return smp_spin_table_cpu_boot(cpu);

	switch (ac->state) {
	case APPLE_CPU_RUNNING:
		/* First boot */
		return smp_spin_table_cpu_boot(cpu);
	case APPLE_CPU_PARKED:
		return apple_cpu_unpark(cpu, ac);
	case APPLE_CPU_OFF:
		return apple_cpu_start(cpu, ac);
	case APPLE_CPU_STARTING:
		pr_err("CPU%u: did not come online after its last start, not starting it again\n",
		       cpu);
		return -EIO;
	case APPLE_CPU_STUCK:
		pr_err("CPU%u: did not go down as expected, not starting it\n",
		       cpu);
		return -EIO;
	}

	return -EINVAL;
}

/* Runs on the CPU that just came up. */
static void apple_spin_table_cpu_postboot(void)
{
	struct apple_cpu *ac = apple_cpus[smp_processor_id()];

	if (ac)
		ac->state = APPLE_CPU_RUNNING;
}

const struct cpu_operations apple_spin_table_ops = {
	.name		= "spin-table",
	.cpu_init	= smp_spin_table_cpu_init,
	.cpu_prepare	= apple_spin_table_cpu_prepare,
	.cpu_boot	= apple_spin_table_cpu_boot,
	.cpu_postboot	= apple_spin_table_cpu_postboot,
	.cpu_can_disable = apple_spin_table_cpu_can_disable,
	.cpu_disable	= apple_spin_table_cpu_disable,
	.cpu_die	= apple_spin_table_cpu_die,
	.cpu_kill	= apple_spin_table_cpu_kill,
};
