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
 *  - When the last online CPU of a cluster goes offline, the cluster can be
 *    power gated (apple_cpu.cluster_gating, on by default, see below). The
 *    parked CPUs of the cluster are told to power down without retention
 *    (L1 clean, PMGR stop, power-down WFI), the CPU complex (ACC) is
 *    quiesced, the last CPU powers down with the CPU complex in deep sleep,
 *    and the cluster's PMGR power state target is set to off.
 *
 *  - The first CPU of a gated cluster to come back powers the cluster up,
 *    re-applies the ACC configuration and starts its core through the PMGR.
 *    The core comes out of reset at its RVBAR, which m1n1 pointed at its
 *    resident secondary entry path, and m1n1 jumps to the spin table release
 *    address the kernel wrote at boot: the holding pen. The other cores of
 *    the cluster are started the same way when their CPUs come back. Each
 *    core is started at most once per cluster power cycle.
 *
 *  - m1n1's re-entry path prints to the console and dockchannel UARTs,
 *    which Linux may have powered down, and an access to a powered down
 *    block hangs the machine. So cluster gating is only used when that path
 *    is known to stay quiet: m1n1 publishes the warm boot mailbox
 *    (/chosen/asahi,s2r-mailbox), whose QUIET flag is set before each start,
 *    or the resident image is positively identified as m1n1 v1.6.1, whose
 *    two UART base pointers are then cleared. Any other boot loader keeps
 *    every offline CPU parked.
 *
 *  - Suspend to RAM takes the secondaries offline the same way, so their
 *    clusters are gated and the boot CPU's cluster keeps its secondaries
 *    parked. Those are powered down first, before any SoC power is taken
 *    down, and can be started through the PMGR once resume has reported
 *    the power cycle.
 *
 * The boot CPU is never offlined.
 */

#define pr_fmt(fmt) "apple-cpu: " fmt

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cacheflush.h>
#include <linux/cpu.h>
#include <linux/cpufeature.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/kexec.h>
#include <linux/log2.h>
#include <linux/mfd/syscon.h>
#include <linux/minmax.h>
#include <linux/mm.h>
#include <linux/mmu_context.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/printk.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/string.h>

#include <asm/apple_cpu.h>
#include <asm/barrier.h>
#include <asm/cmpxchg.h>
#include <asm/cpu_ops.h>
#include <asm/cputype.h>
#include <asm/smp_plat.h>
#include <asm/sysreg.h>
#include <asm/virt.h>

#include "smp_spin_table.h"

#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "apple_cpu."

static bool cluster_gating = true;
module_param(cluster_gating, bool, 0644);
MODULE_PARM_DESC(cluster_gating,
		 "Power gate a CPU cluster once all of its CPUs are offline, if the boot loader allows it (default: on)");

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

/* PMGR CPU cluster power states, in the PMGR syscon ("apple,pmgr") */
#define CLUSTER_PS(data, cl)		((data)->cluster_ps + 4 * (cl))
#define CLUSTER_PS_TARGET		GENMASK(3, 0)
#define CLUSTER_PS_BUSY			BIT(4)
#define CLUSTER_PS_ON			0xf
#define CLUSTER_PS_OFF			0x0
#define CLUSTER_PS_TIMEOUT_US		10000

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

/*
 * Once the last core of a cluster enters deep sleep the cluster's registers
 * stop responding, so its power-down cannot be polled.
 */
#define CLUSTER_DOWN_DELAY_MS		10

/* CPU complex (ACC) registers, relative to the cluster base ("accN") */
#define ACC_PSTATE			0x20020
#define ACC_PSTATE_FIXED_FREQ_PLL_RELOCK BIT_ULL(42)
#define ACC_PSTATE_BUSY			BIT_ULL(31)
#define ACC_PSTATE_SET			BIT_ULL(25)
#define ACC_PSTATE_APSC_DIS		BIT_ULL(23)
#define ACC_PSTATE_APSC_BUSY		BIT_ULL(7)
#define ACC_PSTATE_DESIRED		GENMASK_ULL(4, 0)
#define ACC_APSC_SNOOZE_CTL		0x200f8
#define ACC_APSC_SNOOZE			BIT_ULL(40)
#define ACC_AMX_THROTTLE		0x40250
#define ACC_LLC_THROTTLE		0x40270
#define ACC_INIT_CTL			0x440f8
#define ACC_PPT_THROTTLE		0x48400
#define ACC_THROTTLE_EN			BIT_ULL(63)
#define ACC_SIZE			0x48408
#define ACC_PSTATE_TIMEOUT_US		400

struct apple_acc_feature {
	u32 offset;
	u64 mask;
	u64 busy;
	/* Turn the feature off (mask bits = @off) before powering down */
	bool quiesce;
	u64 off;
};

#define APPLE_ACC_MAX_FEATURES		8

struct apple_cpu_ctrl_data {
	unsigned int apsc_pstate;
	unsigned int cluster_ps;
	const struct apple_acc_feature *features;
	unsigned int nr_features;
};

/*
 * CPU complex feature bits. The values in effect when a cluster goes down
 * are restored, in this order, when it comes back up. Before it goes down,
 * the features marked quiesce are turned off, in reverse order.
 */
static const struct apple_acc_feature t6020_acc_features[] = {
	{ ACC_PSTATE, ACC_PSTATE_APSC_DIS, ACC_PSTATE_APSC_BUSY, true,
	  ACC_PSTATE_APSC_DIS },
	{ ACC_PPT_THROTTLE, ACC_THROTTLE_EN, 0, true, 0 },
	{ ACC_LLC_THROTTLE, ACC_THROTTLE_EN, 0, true, 0 },
	{ ACC_AMX_THROTTLE, ACC_THROTTLE_EN, 0, true, 0 },
	{ ACC_PSTATE, ACC_PSTATE_FIXED_FREQ_PLL_RELOCK, 0, false, 0 },
};

static const struct apple_cpu_ctrl_data t6020_cpu_ctrl_data = {
	.apsc_pstate = 1,
	.cluster_ps = 0x8000,
	.features = t6020_acc_features,
	.nr_features = ARRAY_SIZE(t6020_acc_features),
};

static const struct of_device_id apple_cpu_ctrl_of_match[] = {
	{ .compatible = "apple,t6020-cpu-ctrl", .data = &t6020_cpu_ctrl_data },
	{}
};

struct apple_cpu_ctrl {
	struct device_node *np;
	const struct apple_cpu_ctrl_data *data;
	void __iomem *start;
	/* PMGR syscon holding the cluster power states, NULL if absent */
	struct regmap *pmgr;
};

struct apple_cluster {
	struct apple_cpu_ctrl *ctrl;
	unsigned int idx;
	/* NULL if this cluster cannot be power gated */
	void __iomem *acc;
	/* Lowest p-state of the cluster's OPP table, 0 if unknown */
	unsigned int min_pstate;

	/* ACC state captured before the cluster lost power */
	bool saved;
	u64 features[APPLE_ACC_MAX_FEATURES];
	unsigned int pstate;

	/* PMGR target is off; power up and re-init before starting a core */
	bool gated;
	bool reinit;
	/* ACC_OVRD value to restore once a core runs in the cluster again */
	bool acc_ovrd_dirty;
	u64 acc_ovrd;
};

enum apple_cpu_state {
	/* Online, or never offlined */
	APPLE_CPU_RUNNING,
	/* Parked in the kernel at a retention WFI, see apple_cpu_park() */
	APPLE_CPU_PARKED,
	/*
	 * Powered down through the PMGR, and its cluster not power cycled
	 * since: the core cannot be started.
	 */
	APPLE_CPU_STOPPED,
	/*
	 * Powered down, and its cluster power cycled since: the core can be
	 * started through the PMGR.
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
	/*
	 * PMGR stop and power-down WFI, with the CPU complex in deep sleep;
	 * the cluster is gated
	 */
	APPLE_DIE_GATE,
};

/* Commands to a parked core (apple_cpu::park_cmd) */
#define PARK_CMD_NONE			0
/* Leave the park through the holding pen */
#define PARK_CMD_RESUME			1
/* Power down without state retention, through the PMGR */
#define PARK_CMD_POWER_DOWN		2
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

/*
 * Boot loader re-entry: what keeps m1n1's secondary entry path from
 * touching its console UARTs when a core is started through the PMGR.
 */
enum apple_reentry_mode {
	/* Unknown boot loader: CPU clusters are not gated */
	APPLE_REENTRY_UNKNOWN,
	/* Warm boot mailbox: set its QUIET flag */
	APPLE_REENTRY_MAILBOX,
	/* m1n1 v1.6.1: clear its console and dockchannel UART bases */
	APPLE_REENTRY_M1N1_V161,
};

/* Warm boot mailbox (magic, entry, arg, flags) and its QUIET flag */
#define S2R_MAILBOX_SIZE		(4 * sizeof(__le64))
#define S2R_MAILBOX_FLAGS		3
#define S2R_FLAG_QUIET			BIT_ULL(0)

static struct {
	enum apple_reentry_mode mode;
	__le64 *mailbox_flags;
	u64 *uart_base;
	u64 *dockchannel_base;
} apple_reentry;

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
	const struct of_device_id *match;
	struct apple_cpu_ctrl *ctrl;
	unsigned int i;

	for (i = 0; i < apple_nr_cpu_ctrls; i++)
		if (apple_cpu_ctrls[i].np == np)
			return &apple_cpu_ctrls[i];

	match = of_match_node(apple_cpu_ctrl_of_match, np);
	if (!match || !of_device_is_available(np) ||
	    apple_nr_cpu_ctrls >= APPLE_MAX_CPU_CTRLS)
		return NULL;

	ctrl = &apple_cpu_ctrls[apple_nr_cpu_ctrls];
	ctrl->data = match->data;
	ctrl->start = apple_cpu_ctrl_map(np, "cpu-start",
					 CPU_START_CLUSTER(CPU_START_MAX_CLUSTERS));
	ctrl->pmgr = syscon_regmap_lookup_by_phandle(np, "apple,pmgr");
	if (IS_ERR(ctrl->pmgr))
		ctrl->pmgr = NULL;
	if (!ctrl->start)
		return NULL;

	ctrl->np = of_node_get(np);
	apple_nr_cpu_ctrls++;
	return ctrl;
}

static unsigned int apple_cpu_min_pstate(struct device_node *cpu_np)
{
	struct device_node *opp_np;
	unsigned int min = UINT_MAX;
	u32 level;

	opp_np = of_parse_phandle(cpu_np, "operating-points-v2", 0);
	if (!opp_np)
		return 0;

	for_each_available_child_of_node_scoped(opp_np, child)
		if (!of_property_read_u32(child, "opp-level", &level) && level)
			min = min(min, level);

	of_node_put(opp_np);
	return min == UINT_MAX ? 0 : min;
}

static struct apple_cluster *apple_cluster_get(struct apple_cpu_ctrl *ctrl,
					       unsigned int idx,
					       struct device_node *cpu_np)
{
	struct apple_cluster *cl;
	char name[8];
	unsigned int i;

	for (i = 0; i < apple_nr_clusters; i++)
		if (apple_clusters[i].ctrl == ctrl && apple_clusters[i].idx == idx)
			return &apple_clusters[i];

	if (apple_nr_clusters >= APPLE_MAX_CLUSTERS)
		return NULL;

	cl = &apple_clusters[apple_nr_clusters++];
	cl->ctrl = ctrl;
	cl->idx = idx;

	/* Cluster power gating needs the cluster PMGR control and its ACC. */
	snprintf(name, sizeof(name), "acc%u", idx);
	if (ctrl->pmgr && ctrl->data->nr_features <= APPLE_ACC_MAX_FEATURES)
		cl->acc = apple_cpu_ctrl_map(ctrl->np, name, ACC_SIZE);
	cl->min_pstate = apple_cpu_min_pstate(cpu_np);

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

	ac->cluster = apple_cluster_get(ctrl, args.args[0], np);
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

/*
 * m1n1 v1.6.1, resident at its stage 2 base P, which is page aligned. Each
 * secondary's cpu-release-addr is the target word of its slot in m1n1's
 * spin table. A core started through the PMGR runs P+0 -> cpu_reset ->
 * _cpu_reset_c, which prints through uart_putbyte (console UART, base
 * pointer at P+0xe2d78) and the dockchannel UART (base pointer at
 * P+0xd8030), polling their status registers, before it reaches the spin
 * table. Both put functions return at once if their base is 0. These
 * offsets only hold for this exact build, so the image is identified by
 * its version string and the code that loads both pointers before either
 * is written.
 */
#define M1N1_V161_VERSION		"v1.6.1"
#define M1N1_V161_SIZE			0x110000
#define M1N1_V161_SPIN_TABLE		0xe0240
#define M1N1_V161_SLOT_SIZE		0x40
#define M1N1_V161_SLOTS			24
#define M1N1_V161_SLOT_TARGET		0x10
#define M1N1_V161_UART_BASE		0xe2d78
#define M1N1_V161_DOCKCHANNEL_BASE	0xd8030
/* The dockchannel UART m1n1 uses on the T6020 family */
#define M1N1_DOCKCHANNEL_UART		0x29e528000ULL

static const struct {
	u32 offset;
	u32 insn[4];
} m1n1_v161_code[] = {
	/* _vectors_start: mov x9, #'0'; b cpu_reset; nop; nop */
	{ 0x0, { 0xd2800609, 0x1400023e, 0xd503201f, 0xd503201f } },
	/* uart_putbyte: adrp x3; and w0, w0, #0xff; ldr x1, [x3, #0xd78]; cbz x1 */
	{ 0x2c840, { 0xd00005a3, 0x12001c00, 0xf946bc61, 0xb40000e1 } },
	/* dockchannel putbyte: adrp x3; and w0, w0, #0xff; ldr x1, [x3, #0x30]; cbz x1 */
	{ 0xe640, { 0xd0000643, 0x12001c00, 0xf9401861, 0xb4000181 } },
};

/* P from every secondary's cpu-release-addr, 0 if they disagree */
static phys_addr_t __init apple_m1n1_v161_base(void)
{
	phys_addr_t base = 0;
	unsigned int cpu;

	/* Rounding down to the page recovers P from any slot. */
	BUILD_BUG_ON((M1N1_V161_SPIN_TABLE + M1N1_V161_SLOT_TARGET) % SZ_4K +
		     (M1N1_V161_SLOTS - 1) * M1N1_V161_SLOT_SIZE >= SZ_4K);

	for_each_possible_cpu(cpu) {
		struct device_node *np;
		phys_addr_t p, table;
		u64 release = 0;

		if (!apple_cpus[cpu])
			continue;

		np = of_get_cpu_node(cpu, NULL);
		if (np)
			of_property_read_u64(np, "cpu-release-addr", &release);
		of_node_put(np);

		if (release < M1N1_V161_SPIN_TABLE + M1N1_V161_SLOT_TARGET)
			return 0;
		p = round_down(release - M1N1_V161_SPIN_TABLE -
			       M1N1_V161_SLOT_TARGET, SZ_4K);
		table = p + M1N1_V161_SPIN_TABLE;
		if (release >= table + M1N1_V161_SLOTS * M1N1_V161_SLOT_SIZE ||
		    (release - table - M1N1_V161_SLOT_TARGET) % M1N1_V161_SLOT_SIZE)
			return 0;
		if (base && p != base)
			return 0;
		base = p;
	}

	return base;
}

static bool __init apple_m1n1_v161_probe(const char **why)
{
	struct device_node *np;
	struct resource res;
	u64 uart = 0, *ptr;
	const char *version;
	phys_addr_t base;
	unsigned int i, j;
	void *m1n1;

	if (of_property_read_string(of_chosen, "asahi,m1n1-stage2-version",
				    &version) ||
	    strcmp(version, M1N1_V161_VERSION)) {
		*why = "not m1n1 " M1N1_V161_VERSION;
		return false;
	}

	base = apple_m1n1_v161_base();
	if (!base) {
		*why = "cpu-release-addr is not in its spin table";
		return false;
	}

	/* The /memreserve/'d image reaches Linux as reserved, unmapped memory. */
	if (region_intersects(base, M1N1_V161_SIZE, IORESOURCE_SYSTEM_RAM,
			      IORES_DESC_NONE) != REGION_DISJOINT) {
		*why = "the image overlaps System RAM";
		return false;
	}

	m1n1 = memremap(base, M1N1_V161_SIZE, MEMREMAP_WB);
	if (!m1n1) {
		*why = "cannot map the image";
		return false;
	}
	/* Cores that ran m1n1 with their caches off wrote straight to memory. */
	dcache_clean_inval_poc((unsigned long)m1n1,
			       (unsigned long)m1n1 + M1N1_V161_SIZE);

	for (i = 0; i < ARRAY_SIZE(m1n1_v161_code); i++) {
		const u32 *code = m1n1 + m1n1_v161_code[i].offset;

		for (j = 0; j < ARRAY_SIZE(m1n1_v161_code[i].insn); j++) {
			if (code[j] == m1n1_v161_code[i].insn[j])
				continue;
			pr_info("m1n1 at %pa: P+%#x holds %08x, not %08x\n",
				&base, m1n1_v161_code[i].offset + 4 * j, code[j],
				m1n1_v161_code[i].insn[j]);
			*why = "unknown m1n1 " M1N1_V161_VERSION " build";
			goto out_unmap;
		}
	}

	/* Both pointers must hold what m1n1 put there, or 0. */
	np = of_find_node_by_path("serial0");
	if (np && !of_address_to_resource(np, 0, &res))
		uart = res.start;
	of_node_put(np);

	ptr = m1n1 + M1N1_V161_UART_BASE;
	if (*ptr && *ptr != uart) {
		pr_info("m1n1 at %pa: console UART base %#llx, serial0 at %#llx\n",
			&base, *ptr, uart);
		*why = "unexpected console UART base";
		goto out_unmap;
	}
	apple_reentry.uart_base = ptr;

	ptr = m1n1 + M1N1_V161_DOCKCHANNEL_BASE;
	if (*ptr && *ptr != M1N1_DOCKCHANNEL_UART) {
		pr_info("m1n1 at %pa: dockchannel UART base %#llx\n", &base, *ptr);
		*why = "unexpected dockchannel UART base";
		goto out_unmap;
	}
	apple_reentry.dockchannel_base = ptr;

	pr_info("m1n1 %s at %pa identified\n", version, &base);
	return true;

out_unmap:
	apple_reentry.uart_base = NULL;
	memunmap(m1n1);
	return false;
}

static int __init apple_cpu_reentry_init(void)
{
	const char *why = NULL;
	unsigned int cpu;
	__le64 *mb;
	u64 mailbox;

	for_each_possible_cpu(cpu)
		if (apple_cpus[cpu])
			break;
	if (cpu >= nr_cpu_ids)
		return 0;

	if (!of_property_read_u64(of_chosen, "asahi,s2r-mailbox", &mailbox)) {
		mb = memremap(mailbox, S2R_MAILBOX_SIZE, MEMREMAP_WB);
		if (mb) {
			apple_reentry.mailbox_flags = &mb[S2R_MAILBOX_FLAGS];
			apple_reentry.mode = APPLE_REENTRY_MAILBOX;
			pr_info("boot loader warm boot mailbox at %#llx\n", mailbox);
		} else {
			why = "cannot map the warm boot mailbox";
		}
	} else if (apple_m1n1_v161_probe(&why)) {
		apple_reentry.mode = APPLE_REENTRY_M1N1_V161;
	}

	if (apple_reentry.mode == APPLE_REENTRY_UNKNOWN)
		pr_info("boot loader re-entry path not known to be quiet (%s): CPU clusters are not power gated\n",
			why);

	return 0;
}
late_initcall(apple_cpu_reentry_init);

/*
 * Keep m1n1's re-entry path off its console UARTs. The started core runs it
 * with its caches off.
 */
static void apple_reentry_quiet(void)
{
	__le64 *flags = apple_reentry.mailbox_flags;
	u64 *uart = apple_reentry.uart_base;
	u64 *dock = apple_reentry.dockchannel_base;

	switch (apple_reentry.mode) {
	case APPLE_REENTRY_MAILBOX:
		dcache_clean_inval_poc((unsigned long)flags,
				       (unsigned long)(flags + 1));
		WRITE_ONCE(*flags, cpu_to_le64(le64_to_cpu(READ_ONCE(*flags)) |
					       S2R_FLAG_QUIET));
		dcache_clean_poc((unsigned long)flags, (unsigned long)(flags + 1));
		break;
	case APPLE_REENTRY_M1N1_V161:
		WRITE_ONCE(*uart, 0);
		WRITE_ONCE(*dock, 0);
		dcache_clean_poc((unsigned long)uart, (unsigned long)(uart + 1));
		dcache_clean_poc((unsigned long)dock, (unsigned long)(dock + 1));
		break;
	case APPLE_REENTRY_UNKNOWN:
		break;
	}
}

static int apple_acc_set_pstate(struct apple_cluster *cl, unsigned int pstate)
{
	void __iomem *reg = cl->acc + ACC_PSTATE;
	u64 val = readq(reg);

	if (FIELD_GET(ACC_PSTATE_DESIRED, val) == pstate)
		return 0;

	val &= ~ACC_PSTATE_DESIRED;
	val |= ACC_PSTATE_SET | FIELD_PREP(ACC_PSTATE_DESIRED, pstate);
	writeq(val, reg);

	return readq_poll_timeout_atomic(reg, val, !(val & ACC_PSTATE_BUSY), 2,
					 ACC_PSTATE_TIMEOUT_US);
}

static void apple_acc_save(struct apple_cluster *cl)
{
	const struct apple_cpu_ctrl_data *data = cl->ctrl->data;
	unsigned int i;

	for (i = 0; i < data->nr_features; i++)
		cl->features[i] = readq(cl->acc + data->features[i].offset) &
				  data->features[i].mask;
	cl->pstate = FIELD_GET(ACC_PSTATE_DESIRED, readq(cl->acc + ACC_PSTATE));
	cl->saved = true;
}

static int apple_acc_set_feature(struct apple_cluster *cl,
				 const struct apple_acc_feature *f, u64 bits)
{
	void __iomem *reg = cl->acc + f->offset;
	u64 val = (readq(reg) & ~f->mask) | bits;

	writeq(val, reg);
	if (!f->busy)
		return 0;

	return readq_poll_timeout_atomic(reg, val, !(val & f->busy), 2,
					 ACC_PSTATE_TIMEOUT_US);
}

/* Turn the CPU complex features off before the cluster powers down. */
static int apple_acc_quiesce(struct apple_cluster *cl)
{
	const struct apple_cpu_ctrl_data *data = cl->ctrl->data;
	unsigned int i = data->nr_features;
	int ret;

	while (i--) {
		const struct apple_acc_feature *f = &data->features[i];

		if (!f->quiesce)
			continue;
		ret = apple_acc_set_feature(cl, f, f->off);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * Bring a freshly powered CPU complex back to its configuration: APSC
 * p-state, the saved feature bits, the ACC init and APSC snooze controls,
 * then the saved p-state.
 */
static int apple_acc_reinit(struct apple_cluster *cl)
{
	const struct apple_cpu_ctrl_data *data = cl->ctrl->data;
	unsigned int i;
	u64 val;
	int ret;

	if (!cl->saved)
		return -ENODATA;

	ret = apple_acc_set_pstate(cl, data->apsc_pstate);
	if (ret)
		return ret;

	for (i = 0; i < data->nr_features; i++) {
		ret = apple_acc_set_feature(cl, &data->features[i],
					    cl->features[i]);
		if (ret)
			return ret;
	}

	writeq(1, cl->acc + ACC_INIT_CTL);
	val = readq(cl->acc + ACC_APSC_SNOOZE_CTL);
	writeq(val | ACC_APSC_SNOOZE, cl->acc + ACC_APSC_SNOOZE_CTL);

	return apple_acc_set_pstate(cl, cl->pstate);
}

static int apple_cluster_set_ps(struct apple_cluster *cl, u32 target)
{
	return regmap_update_bits(cl->ctrl->pmgr,
				  CLUSTER_PS(cl->ctrl->data, cl->idx),
				  CLUSTER_PS_TARGET,
				  FIELD_PREP(CLUSTER_PS_TARGET, target));
}

static int apple_cluster_power_up(struct apple_cluster *cl)
{
	unsigned int val;
	int ret;

	if (cl->gated) {
		ret = apple_cluster_set_ps(cl, CLUSTER_PS_ON);
		if (!ret)
			ret = regmap_read_poll_timeout(cl->ctrl->pmgr,
						       CLUSTER_PS(cl->ctrl->data, cl->idx),
						       val, !(val & CLUSTER_PS_BUSY),
						       10, CLUSTER_PS_TIMEOUT_US);
		if (ret) {
			pr_err("cluster %u: power-up failed: %d\n", cl->idx, ret);
			return ret;
		}
		cl->gated = false;
		cl->reinit = true;
	}

	if (cl->reinit) {
		ret = apple_acc_reinit(cl);
		if (ret) {
			pr_err("cluster %u: CPU complex re-init failed: %d\n",
			       cl->idx, ret);
			return ret;
		}
		cl->reinit = false;
	}

	return 0;
}

static bool apple_cpu_same_cluster(unsigned int a, unsigned int b)
{
	return !((cpu_logical_map(a) ^ cpu_logical_map(b)) &
		 ~(u64)MPIDR_LEVEL_MASK);
}

static bool apple_cpu_last_in_cluster(unsigned int cpu)
{
	unsigned int other;

	for_each_online_cpu(other)
		if (other != cpu && apple_cpu_same_cluster(cpu, other))
			return false;

	return true;
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
 * dirty line, whatever happens to its power later. Every command but
 * PARK_CMD_RESUME takes it down for good.
 */
static void __noreturn apple_cpu_park(struct apple_cpu *ac)
{
	struct apple_cluster *cl = ac->cluster;
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

	if (cmd == PARK_CMD_RESUME) {
		write_sysreg_s(cyc, SYS_IMP_APL_CYC_OVRD);
		isb();

		/* Into the holding pen with the MMU off, as on first boot */
		cpu_install_idmap();
		restart = (void *)__pa_symbol(cpu_soft_restart);
		restart(is_hyp_nvhe(), __pa_symbol(secondary_holding_pen),
			0, 0, 0);
		unreachable();
	}

	apple_cpu_power_down(cl->ctrl->start + CPU_START_STOP,
			     CPU_START_CORE_BIT(cl->idx, ac->core),
			     APPLE_CPU_SLEEP_MASK_IRQ);
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
 * Power a parked core down without state retention, through the PMGR stop
 * control, so that its cluster can be gated. Does not sleep.
 *
 * A parked core already reads run state 0, so that alone does not show it
 * went down. The core sets its stop bit after cleaning its L1 and right
 * before its final WFI; the stop control reads back the stopped cores.
 */
static int apple_cpu_park_power_down(unsigned int cpu, struct apple_cpu *ac)
{
	struct apple_cluster *cl = ac->cluster;
	void __iomem *stop = cl->ctrl->start + CPU_START_STOP;
	u32 bit = CPU_START_CORE_BIT(cl->idx, ac->core);
	u32 stopped;
	u64 val;
	int ret;

	ret = apple_cpu_park_command(cpu, ac, PARK_CMD_POWER_DOWN);
	if (ret)
		return ret;

	ret = readl_poll_timeout_atomic(stop, stopped, stopped & bit, 10,
					IMPL_STOP_TIMEOUT_US);
	if (!ret)
		ret = readq_poll_timeout_atomic(ac->impl + IMPL_CORE_STATUS, val,
						!(val & IMPL_CORE_STATUS_RUNNING),
						10, IMPL_STOP_TIMEOUT_US);
	if (ret) {
		pr_err("CPU%u: core did not power down: stop %#x, status %#llx\n",
		       cpu, readl(stop), readq(ac->impl + IMPL_CORE_STATUS));
		ac->state = APPLE_CPU_STUCK;
		return ret;
	}

	ac->state = APPLE_CPU_STOPPED;
	return 0;
}

/**
 * apple_cpu_acc_save() - prepare the CPU complexes for losing power
 *
 * Captures the CPU complex configuration of every powered cluster that can
 * be re-initialised, for apple_cpu_acc_restore(). Parked CPUs must already
 * be powered down: see apple_cpu_park_power_down_all(). Must be called
 * with interrupts masked, with every secondary CPU offline.
 *
 * Return: 0, or -ENODEV if no cluster supports it.
 */
int apple_cpu_acc_save(void)
{
	bool found = false;
	unsigned int i;

	for (i = 0; i < apple_nr_clusters; i++) {
		struct apple_cluster *cl = &apple_clusters[i];

		/* A cluster awaiting re-init still holds its capture. */
		if (!cl->acc || cl->gated || cl->reinit)
			continue;
		apple_acc_save(cl);
		found = true;
	}

	return found ? 0 : -ENODEV;
}
EXPORT_SYMBOL_GPL(apple_cpu_acc_save);

/**
 * apple_cpu_park_power_down_all() - power down every parked CPU
 *
 * A parked core would not survive its cluster losing power. Once powered
 * down, it can only be started again after its cluster was power cycled:
 * if suspend to RAM is aborted afterwards, the CPUs of the boot CPU's
 * cluster stay offline until reboot. Call it before any other SoC power is
 * taken down, as a parked core must not be woken after that. Must be
 * called with interrupts masked.
 *
 * Return: 0, or an error if a parked CPU did not power down.
 */
int apple_cpu_park_power_down_all(void)
{
	unsigned int cpu;
	int ret;

	for_each_possible_cpu(cpu) {
		struct apple_cpu *ac = apple_cpus[cpu];

		if (!ac || ac->state != APPLE_CPU_PARKED)
			continue;
		ret = apple_cpu_park_power_down(cpu, ac);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(apple_cpu_park_power_down_all);

/**
 * apple_cpu_power_cycled() - every CPU complex lost power
 *
 * Called on resume from suspend to RAM: each powered-down CPU can now be
 * started once.
 */
void apple_cpu_power_cycled(void)
{
	unsigned int cpu;

	for_each_possible_cpu(cpu) {
		struct apple_cpu *ac = apple_cpus[cpu];

		if (ac && ac->state == APPLE_CPU_STOPPED)
			ac->state = APPLE_CPU_OFF;
	}
}
EXPORT_SYMBOL_GPL(apple_cpu_power_cycled);

/**
 * apple_cpu_acc_restore() - re-apply the CPU complex configuration
 *
 * Re-initialises every powered cluster from its last apple_cpu_acc_save()
 * capture, after the CPU complexes lost their state. Clusters that are
 * power gated are re-initialised when their first CPU is started.
 *
 * Return: 0, the first error, or -ENODEV if no cluster supports it.
 */
int apple_cpu_acc_restore(void)
{
	bool found = false;
	unsigned int i;
	int ret = 0;

	for (i = 0; i < apple_nr_clusters; i++) {
		struct apple_cluster *cl = &apple_clusters[i];
		int err;

		if (!cl->acc || cl->gated || !cl->saved)
			continue;
		found = true;
		err = apple_acc_reinit(cl);
		if (err) {
			pr_err("cluster %u: CPU complex re-init failed: %d\n",
			       cl->idx, err);
			if (!ret)
				ret = err;
		} else {
			cl->reinit = false;
		}
	}

	return found ? ret : -ENODEV;
}
EXPORT_SYMBOL_GPL(apple_cpu_acc_restore);

/*
 * Can @cpu, the last online CPU of its cluster, take the cluster down with
 * it? Every other core of the cluster must be parked or powered down, and a
 * core started later must find a quiet boot loader.
 */
static bool apple_cluster_can_gate(unsigned int cpu, struct apple_cluster *cl)
{
	unsigned int other;

	if (!READ_ONCE(cluster_gating) || !cl->acc || !cl->min_pstate ||
	    apple_reentry.mode == APPLE_REENTRY_UNKNOWN)
		return false;

	/* Neither online nor powered down: never booted, or in an unknown state */
	for_each_possible_cpu(other) {
		struct apple_cpu *ac = apple_cpus[other];

		if (other == cpu || !apple_cpu_same_cluster(cpu, other))
			continue;
		if (!ac || ac->cluster != cl)
			return false;
		if (ac->state != APPLE_CPU_PARKED &&
		    ac->state != APPLE_CPU_STOPPED &&
		    ac->state != APPLE_CPU_OFF)
			return false;
	}

	return true;
}

/*
 * Runs on the last CPU of the cluster, with interrupts masked: power down
 * the parked cores, then quiesce the CPU complex. The CPU complex is only
 * quiesced once every other core of the cluster is down, never under a
 * core that still has to wake from its park.
 *
 * If a parked core does not power down, the cluster is not gated and the
 * calling core parks too. Cores already powered down then stay down until
 * the cluster is next power cycled. Once every core is down, the cluster
 * is gated even if the CPU complex does not settle: it loses its
 * configuration either way, and is re-initialised from the saved copy
 * when it is powered up again.
 */
static int apple_cluster_prepare_gating(unsigned int cpu,
					struct apple_cluster *cl)
{
	unsigned int other;
	int ret;

	for_each_possible_cpu(other) {
		struct apple_cpu *ac = apple_cpus[other];

		if (other == cpu || !ac || ac->cluster != cl ||
		    ac->state != APPLE_CPU_PARKED)
			continue;
		ret = apple_cpu_park_power_down(other, ac);
		if (ret) {
			pr_warn("cluster %u: CPU%u did not power down, not gating it\n",
				cl->idx, other);
			return ret;
		}
	}

	apple_acc_save(cl);
	ret = apple_acc_set_pstate(cl, cl->min_pstate);
	if (!ret)
		ret = apple_acc_quiesce(cl);
	if (ret)
		pr_warn("cluster %u: CPU complex did not settle: %d\n", cl->idx,
			ret);

	return 0;
}

static bool apple_spin_table_cpu_can_disable(unsigned int cpu)
{
	return !!apple_cpus[cpu];
}

/* Runs on the dying CPU, after cpufreq has let go of its cluster. */
static int apple_spin_table_cpu_disable(unsigned int cpu)
{
	struct apple_cpu *ac = apple_cpus[cpu];
	struct apple_cluster *cl;

	if (!ac)
		return -EOPNOTSUPP;

	cl = ac->cluster;
	ac->rvbar = readq(ac->impl + IMPL_RVBAR);
	ac->park_cmd = PARK_CMD_NONE;
	ac->parked = false;

	if (apple_cpu_last_in_cluster(cpu) &&
	    apple_cluster_can_gate(cpu, cl) &&
	    !apple_cluster_prepare_gating(cpu, cl))
		ac->die = APPLE_DIE_GATE;
	else
		ac->die = APPLE_DIE_PARK;

	return 0;
}

static void apple_spin_table_cpu_die(unsigned int cpu)
{
	struct apple_cpu *ac = apple_cpus[cpu];
	unsigned int flags = APPLE_CPU_SLEEP_MASK_IRQ;
	struct apple_cluster *cl;

	/* Early boot failures and crash stops keep parking in the kernel. */
	if (!ac || ac->die == APPLE_DIE_NONE)
		return;

	if (ac->die == APPLE_DIE_PARK)
		apple_cpu_park(ac);

	cl = ac->cluster;
	if (ac->die == APPLE_DIE_GATE) {
		cl->acc_ovrd = read_sysreg_s(SYS_IMP_APL_ACC_OVRD);
		cl->acc_ovrd_dirty = true;
		flags |= APPLE_CPU_SLEEP_ACC_DEEP;
	}

	apple_cpu_power_down(cl->ctrl->start + CPU_START_STOP,
			     CPU_START_CORE_BIT(cl->idx, ac->core), flags);
}

/* The cluster lost power: each of its offline cores can be started once. */
static void apple_cluster_gated(struct apple_cluster *cl)
{
	unsigned int cpu;

	cl->gated = true;
	for_each_possible_cpu(cpu) {
		struct apple_cpu *ac = apple_cpus[cpu];

		if (ac && ac->cluster == cl && ac->state == APPLE_CPU_STOPPED)
			ac->state = APPLE_CPU_OFF;
	}
}

static int apple_spin_table_cpu_kill(unsigned int cpu)
{
	struct apple_cpu *ac = apple_cpus[cpu];
	enum apple_cpu_die die;
	struct apple_cluster *cl;
	bool parked;
	int ret;

	if (!ac || ac->die == APPLE_DIE_NONE)
		return 0;

	die = ac->die;
	ac->die = APPLE_DIE_NONE;
	/* Nothing may leave the holding pen until the next cpu_boot. */
	smp_spin_table_write_pen_release(INVALID_HWID);

	cl = ac->cluster;
	switch (die) {
	case APPLE_DIE_PARK:
		ret = read_poll_timeout(smp_load_acquire, parked, parked, 100,
					PARK_TIMEOUT_US, false, &ac->parked);
		if (ret) {
			pr_err("CPU%u: core did not park\n", cpu);
			ac->state = APPLE_CPU_STUCK;
			return ret;
		}
		ac->state = APPLE_CPU_PARKED;
		return 0;

	case APPLE_DIE_GATE:
		msleep(CLUSTER_DOWN_DELAY_MS);
		ret = apple_cluster_set_ps(cl, CLUSTER_PS_OFF);
		if (ret)
			pr_warn("cluster %u: power-down request failed: %d\n",
				cl->idx, ret);
		/* Lost its state either way: power up and re-init on next use. */
		ac->state = APPLE_CPU_STOPPED;
		apple_cluster_gated(cl);
		return 0;

	default:
		return 0;
	}
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

/* Start a powered down core through the PMGR, powering its cluster up first. */
static int apple_cpu_start(unsigned int cpu, struct apple_cpu *ac)
{
	struct apple_cluster *cl = ac->cluster;
	int ret;

	ret = apple_cluster_power_up(cl);
	if (ret)
		return ret;

	ret = apple_cpu_check_rvbar(cpu, ac);
	if (ret)
		return ret;

	apple_reentry_quiet();
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
	case APPLE_CPU_STOPPED:
		pr_err("CPU%u: powered down while its cluster stayed up, cannot start until the cluster is power gated\n",
		       cpu);
		return -EIO;
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
	struct apple_cluster *cl;

	if (!ac)
		return;

	ac->state = APPLE_CPU_RUNNING;

	cl = ac->cluster;
	if (cl->acc_ovrd_dirty) {
		write_sysreg_s(cl->acc_ovrd, SYS_IMP_APL_ACC_OVRD);
		isb();
		cl->acc_ovrd_dirty = false;
	}
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
