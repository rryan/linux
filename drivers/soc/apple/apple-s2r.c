// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SoC suspend-to-RAM
 *
 * Copyright The Asahi Linux Contributors
 *
 * Provides the "mem" (deep) system sleep state. By the time ->enter() runs,
 * every device has been suspended, the secondary CPUs are offline and the
 * syscore hooks (AIC state save and mask-all) have run. ->enter() then
 * performs, on the boot CPU with interrupts off:
 *
 *  1. the parked secondaries of the boot CPU's cluster powered down (the
 *     other clusters were gated when their last CPU went offline). This
 *     comes first, as for cluster gating: a parked core must not be woken
 *     once SoC power is being taken down. From here on an aborted suspend
 *     leaves them offline until reboot,
 *  2. CPU complex state snapshot for the resume path,
 *  3. CPM power domain gating,
 *  4. media fabric (AFR) off,
 *  5. CPU cluster power targets to off (no wait), then the non-CPU SoC
 *     clusters off (with waits),
 *  6. SIO / SIO_CPU power state auto-enable off,
 *  7. AP wake sources: every source masked except the apple,ap-wake-sources
 *     of the PMGR sleep node; suspend-to-RAM is refused without them,
 *  8. SoC clock generators switched (in order 0x160, 0x168, 0x164, 0x158,
 *     0x16c), then PLL off mode,
 *  9. PMGR suspend-to-RAM timer start, written again as the final step,
 * 10. warm boot mailbox armed with cpu_resume, then a non-retention WFI with
 *     CPU complex deep sleep allowed.
 *
 * The fabric and DCS performance states are staged by apple-pmgr-misc in its
 * noirq suspend callback, before this point.
 *
 * The SoC wakes the boot CPU at its reset vector. Boot firmware finds the
 * armed mailbox, redoes the boot CPU setup and branches to cpu_resume. After
 * the AIC syscore resume, this driver's syscore resume undoes the sequence:
 * PMGR wake control, CPM, SEP power state, CPU clusters, CPU complex state,
 * SIO, SoC clusters, AFR. The clock generators and the PLL come back by
 * themselves.
 *
 * For bring-up, the wdt_timeout parameter makes the finisher arm the
 * watchdog (apple,watchdog) right after the mailbox, so that a sleep that
 * never resumes ends in a reset instead of a hang. Both the boot firmware's
 * warm boot path and the syscore resume disarm it; the latter puts back what
 * the watchdog driver had programmed.
 *
 * CPM gating, AFR, the non-CPU SoC clusters and SIO are shared with s2idle
 * and live in apple-sleep.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cacheflush.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/soc/apple/sleep.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>

#include <asm/apple_cpu.h>
#include <asm/memory.h>
#include <asm/suspend.h>
#include <asm/virt.h>

/* Busy-wait budget for each register poll */
#define S2R_POLL_TIMEOUT_US	8000

/* SoC cluster power control, in the PMGR syscon */
#define CLUSTER_PS(phys)	(0x8000 + 4 * (phys))
#define CLUSTER_PS_TARGET	GENMASK(3, 0)
#define CLUSTER_PS_BUSY		BIT(4)
#define CLUSTER_PS_ON		0xf
#define S2R_MAX_CLUSTERS	8

/* SoC clock generator, switched in this order */
static const u16 clkgen_regs[] = { 0x160, 0x168, 0x164, 0x158, 0x16c };
#define CLKGEN_SWITCH		BIT(31)
#define CLKGEN_PENDING		BIT(30)
#define CLKGEN_SEL		GENMASK(27, 24)

/* PLL off-mode control */
#define PLL_BUSY		BIT(31)
#define PLL_OFF_MODE		GENMASK(5, 3)
#define PLL_OFF_MODE_S2R	3
#define PLL_CTL_SET		BIT(1)
#define PLL_CTL_CLEAR		BIT(0)
#define PLL_TIMEOUT_US		100000

/* PMGR suspend-to-RAM timer */
#define S2R_TIMER_START		1

/* Power state register (see pmgr-pwrstate) */
#define PS_MIN			GENMASK(19, 16)
#define PS_ACTUAL		GENMASK(7, 4)
#define PS_ACTIVE		0xf

/* Watchdog (apple,wdt), armed as a bring-up safety timer */
#define WDT_COUNT		0x10
#define WDT_ALARM		0x14
#define WDT_CTL			0x1c
#define WDT_CTL_RESET_EN	BIT(2)
#define WDT_CLK_HZ		24000000
#define WDT_MAX_TIMEOUT		(U32_MAX / WDT_CLK_HZ)

/* Warm boot mailbox published by the boot firmware */
struct apple_s2r_mailbox {
	__le64 magic;
	__le64 entry;
	__le64 arg;
	__le64 flags;
};

#define S2R_MAGIC		0x21523253316e316dULL
#define S2R_FLAG_QUIET		BIT_ULL(0)
#define S2R_FLAG_CPUFREQ_INIT	BIT_ULL(1)
#define S2R_FLAG_MCC_INIT	BIT_ULL(2)
/* Bring-up: the firmware reboots through the watchdog instead of resuming */
#define S2R_FLAG_REBOOT_AT_ENTRY	BIT_ULL(3)
#define S2R_FLAG_REBOOT_AT_BRANCH	BIT_ULL(4)
#define S2R_FLAGS_OS		(S2R_FLAG_CPUFREQ_INIT | S2R_FLAG_MCC_INIT | \
				 S2R_FLAG_REBOOT_AT_ENTRY | \
				 S2R_FLAG_REBOOT_AT_BRANCH)
#define S2R_FLAG_RESUMED	BIT_ULL(63)

/*
 * Steps taken by ->enter() that the resume path must undo. The same bits
 * select steps to leave out with the skip parameter.
 */
#define S2R_DONE_ACC		BIT(0)
#define S2R_DONE_CPM		BIT(1)
#define S2R_DONE_AFR		BIT(2)
#define S2R_DONE_CPU_CLUSTERS	BIT(3)
#define S2R_DONE_SOC_CLUSTERS	BIT(4)
#define S2R_DONE_SIO		BIT(5)
#define S2R_DONE_CLOCKS		BIT(6)
#define S2R_SKIP_TIMER		BIT(7)
#define S2R_DONE_WAKE		BIT(8)

/* In the order ->enter() runs them; abort_before takes these numbers */
enum apple_s2r_step {
	S2R_STEP_NONE,
	S2R_STEP_PARKED,
	S2R_STEP_ACC,
	S2R_STEP_CPM,
	S2R_STEP_AFR,
	S2R_STEP_SOC_CLUSTERS,
	S2R_STEP_SIO,
	S2R_STEP_WAKE,
	S2R_STEP_CLKGEN,
	S2R_STEP_PLL,
	S2R_STEP_SUSPEND,
};

static const char *const apple_s2r_step_names[] = {
	[S2R_STEP_NONE]		= "none",
	[S2R_STEP_PARKED]	= "parked CPUs power-down",
	[S2R_STEP_ACC]		= "CPU complex state save",
	[S2R_STEP_CPM]		= "CPM power gating",
	[S2R_STEP_AFR]		= "AFR off",
	[S2R_STEP_SOC_CLUSTERS]	= "SoC clusters off",
	[S2R_STEP_SIO]		= "SIO auto-enable off",
	[S2R_STEP_WAKE]		= "AP wake source masks",
	[S2R_STEP_CLKGEN]	= "clock generator switch",
	[S2R_STEP_PLL]		= "PLL off mode",
	[S2R_STEP_SUSPEND]	= "CPU suspend",
};

struct apple_s2r {
	struct device *dev;
	struct regmap *pmgr;
	struct regmap *sep;
	u32 sep_reg;
	void __iomem *clkgen;
	void __iomem *pll;
	void __iomem *timer;
	void __iomem *wdt;
	struct apple_s2r_mailbox *mailbox;

	u32 cpu_clusters[S2R_MAX_CLUSTERS];
	u32 cpu_cluster_saved[S2R_MAX_CLUSTERS];
	int nr_cpu_clusters;

	/* State of the current sleep cycle, written with IRQs off */
	unsigned long done;
	bool resumed;
	enum apple_s2r_step failed_step;
	int failed_ret;
	int restore_ret;

	/* Watchdog state replaced by the safety timer */
	bool wdt_armed;
	u32 wdt_saved_ctl;
	u32 wdt_saved_alarm;

	struct syscore syscore;
};

static struct apple_s2r *apple_s2r;

static bool enable;
module_param(enable, bool, 0444);
MODULE_PARM_DESC(enable, "Offer suspend-to-RAM (\"deep\" mem sleep); needs firmware warm boot support (default: off)");

static ulong warm_flags;
module_param(warm_flags, ulong, 0644);
MODULE_PARM_DESC(warm_flags, "Warm boot requests to the boot firmware: bit 1 CPU cluster frequency init, bit 2 memory cache init; for bring-up, bit 3 reboot on entering the warm boot path, bit 4 reboot right before resuming (default: 0)");

static uint skip;
module_param(skip, uint, 0644);
MODULE_PARM_DESC(skip, "Steps to leave out, for bring-up: 0x2 CPM gating, 0x4 AFR off, 0x8 CPU clusters off, 0x10 SoC clusters off, 0x20 SIO auto-enable off, 0x40 clock generators and PLL, 0x80 suspend-to-RAM timer, 0x100 AP wake source masks (default: 0)");

static uint abort_before;
module_param(abort_before, uint, 0644);
MODULE_PARM_DESC(abort_before, "Abort suspend-to-RAM before this step, for bring-up: 1 parked CPUs power-down, 2 CPU complex state save, 3 CPM gating, 4 AFR off, 5 SoC clusters off (CPU cluster targets are already off), 6 SIO auto-enable off, 7 AP wake source masks, 8 clock generators, 10 CPU suspend (default: 0, never)");

static uint wdt_timeout;
module_param(wdt_timeout, uint, 0644);
MODULE_PARM_DESC(wdt_timeout, "For bring-up: right before the SoC goes to sleep, arm the watchdog to reset the system after this many seconds (at most 178) unless the resume path disarms it first; needs apple,watchdog (default: 0, off)");

static bool spin_instead_of_sleep;
module_param(spin_instead_of_sleep, bool, 0644);
MODULE_PARM_DESC(spin_instead_of_sleep, "For bring-up: run every step, arm the watchdog, then spin with interrupts off instead of the final power-down, so only the watchdog can end it; needs wdt_timeout (default: off)");

/*
 * regmap_read_poll_timeout_atomic() keeps time with ktime_get(), which must
 * not be used once timekeeping is suspended: poll with a delay count.
 */
static u32 apple_s2r_regmap_read(struct regmap *map, unsigned int reg)
{
	u32 val = 0;

	regmap_read(map, reg, &val);
	return val;
}

static void apple_s2r_cpu_clusters_off(struct apple_s2r *s2r)
{
	int i;

	/*
	 * Only the target is set; each CPU cluster powers down once all its
	 * cores are in a non-retention WFI, the boot CPU's own cluster last.
	 */
	for (i = 0; i < s2r->nr_cpu_clusters; i++) {
		u32 reg = CLUSTER_PS(s2r->cpu_clusters[i]);
		u32 val;

		regmap_read(s2r->pmgr, reg, &val);
		s2r->cpu_cluster_saved[i] = FIELD_GET(CLUSTER_PS_TARGET, val);
		regmap_write(s2r->pmgr, reg, val & ~CLUSTER_PS_TARGET);
	}
}

static void apple_s2r_cpu_clusters_restore(struct apple_s2r *s2r)
{
	int i;

	for (i = s2r->nr_cpu_clusters - 1; i >= 0; i--) {
		u32 reg = CLUSTER_PS(s2r->cpu_clusters[i]);
		u32 target = s2r->cpu_cluster_saved[i];
		u32 val;

		regmap_update_bits(s2r->pmgr, reg, CLUSTER_PS_TARGET,
				   FIELD_PREP(CLUSTER_PS_TARGET, target));
		if (!target)
			continue;
		if (read_poll_timeout_atomic(apple_s2r_regmap_read, val,
					     !(val & CLUSTER_PS_BUSY), 1,
					     S2R_POLL_TIMEOUT_US, false,
					     s2r->pmgr, reg))
			s2r->restore_ret = -ETIMEDOUT;
	}
}

static int apple_s2r_clocks_off(struct apple_s2r *s2r)
{
	u32 val;
	int i, ret;

	s2r->failed_step = S2R_STEP_CLKGEN;
	for (i = 0; i < ARRAY_SIZE(clkgen_regs); i++) {
		void __iomem *reg = s2r->clkgen + clkgen_regs[i];

		val = readl_relaxed(reg);
		val &= ~(CLKGEN_SWITCH | CLKGEN_SEL);
		writel_relaxed(val | CLKGEN_SWITCH, reg);
		ret = readl_relaxed_poll_timeout_atomic(reg, val,
							!(val & CLKGEN_PENDING), 1,
							S2R_POLL_TIMEOUT_US);
		if (ret)
			return ret;
	}

	s2r->failed_step = S2R_STEP_PLL;
	val = readl_relaxed(s2r->pll);
	val &= ~PLL_OFF_MODE;
	writel_relaxed(val | FIELD_PREP(PLL_OFF_MODE, PLL_OFF_MODE_S2R), s2r->pll);
	writel_relaxed(readl_relaxed(s2r->pll) | PLL_CTL_SET, s2r->pll);
	writel_relaxed(readl_relaxed(s2r->pll) & ~PLL_CTL_CLEAR, s2r->pll);

	return readl_relaxed_poll_timeout_atomic(s2r->pll, val, !(val & PLL_BUSY),
						 1, PLL_TIMEOUT_US);
}

static void apple_s2r_sep_on(struct apple_s2r *s2r)
{
	u32 val;

	/* The SEP power domain is off after suspend-to-RAM; force it back on */
	regmap_update_bits(s2r->sep, s2r->sep_reg, PS_MIN,
			   FIELD_PREP(PS_MIN, PS_ACTIVE));
	if (read_poll_timeout_atomic(apple_s2r_regmap_read, val,
				     FIELD_GET(PS_ACTUAL, val) == PS_ACTIVE,
				     1, S2R_POLL_TIMEOUT_US, false,
				     s2r->sep, s2r->sep_reg))
		s2r->restore_ret = -ETIMEDOUT;
	regmap_update_bits(s2r->sep, s2r->sep_reg, PS_MIN, 0);
}

/* Make the watchdog reset the system if the resume path does not come back */
static void apple_s2r_wdt_arm(struct apple_s2r *s2r)
{
	u32 secs = min(wdt_timeout, WDT_MAX_TIMEOUT);

	writel_relaxed(secs * WDT_CLK_HZ, s2r->wdt + WDT_ALARM);
	writel_relaxed(0, s2r->wdt + WDT_COUNT);
	writel(WDT_CTL_RESET_EN, s2r->wdt + WDT_CTL);
}

/* Put back what the watchdog driver had programmed, with a fresh count */
static void apple_s2r_wdt_restore(struct apple_s2r *s2r)
{
	writel_relaxed(0, s2r->wdt + WDT_CTL);
	writel_relaxed(s2r->wdt_saved_alarm, s2r->wdt + WDT_ALARM);
	writel_relaxed(0, s2r->wdt + WDT_COUNT);
	writel(s2r->wdt_saved_ctl, s2r->wdt + WDT_CTL);
}

static int apple_s2r_finish(unsigned long arg)
{
	struct apple_s2r *s2r = (struct apple_s2r *)arg;
	struct apple_s2r_mailbox *mb = s2r->mailbox;
	u64 flags = S2R_FLAG_QUIET | (warm_flags & S2R_FLAGS_OS);

	WRITE_ONCE(mb->entry, cpu_to_le64(__pa_symbol(cpu_resume)));
	WRITE_ONCE(mb->arg, 0);
	WRITE_ONCE(mb->flags, cpu_to_le64(flags));
	WRITE_ONCE(mb->magic, cpu_to_le64(S2R_MAGIC));
	dcache_clean_poc((unsigned long)mb, (unsigned long)(mb + 1));

	if (s2r->wdt_armed)
		apple_s2r_wdt_arm(s2r);

	/* The timer start is written a second time, as the very last step */
	if (!(skip & S2R_SKIP_TIMER))
		writel(S2R_TIMER_START, s2r->timer);

	if (READ_ONCE(spin_instead_of_sleep) && s2r->wdt_armed)
		for (;;)
			cpu_relax();

	apple_cpu_deep_sleep(APPLE_CPU_SLEEP_ACC_DEEP);
}

/*
 * Record the step before running it: its restore undoes partial progress.
 * abort_before stops at the step without running it.
 */
static int apple_s2r_check(struct apple_s2r *s2r, enum apple_s2r_step step)
{
	s2r->failed_step = step;
	return READ_ONCE(abort_before) == step ? -ECANCELED : 0;
}

static int apple_s2r_step(struct apple_s2r *s2r, enum apple_s2r_step step,
			  unsigned long done, int (*fn)(void))
{
	int ret;

	if (skip & done & ~S2R_DONE_ACC)
		return 0;

	ret = apple_s2r_check(s2r, step);
	if (ret)
		return ret;
	s2r->done |= done;

	return fn();
}

static int apple_s2r_enter(suspend_state_t state)
{
	struct apple_s2r *s2r = apple_s2r;
	int ret;

	s2r->done = 0;
	s2r->resumed = false;
	s2r->failed_step = S2R_STEP_NONE;
	s2r->failed_ret = 0;
	s2r->restore_ret = 0;
	s2r->wdt_armed = false;

	/* No printk from here on: the console may lose its clock */
	ret = apple_s2r_step(s2r, S2R_STEP_PARKED, 0,
			     apple_cpu_park_power_down_all);
	if (ret)
		goto fail;
	ret = apple_s2r_step(s2r, S2R_STEP_ACC, S2R_DONE_ACC, apple_cpu_acc_save);
	if (ret)
		goto fail;
	ret = apple_s2r_step(s2r, S2R_STEP_CPM, S2R_DONE_CPM, apple_sleep_cpm_gate);
	if (ret)
		goto fail;
	ret = apple_s2r_step(s2r, S2R_STEP_AFR, S2R_DONE_AFR, apple_sleep_afr_off);
	if (ret)
		goto fail;

	if (!(skip & S2R_DONE_CPU_CLUSTERS)) {
		s2r->done |= S2R_DONE_CPU_CLUSTERS;
		apple_s2r_cpu_clusters_off(s2r);
	}

	ret = apple_s2r_step(s2r, S2R_STEP_SOC_CLUSTERS, S2R_DONE_SOC_CLUSTERS,
			     apple_sleep_soc_clusters_off);
	if (ret)
		goto fail;
	ret = apple_s2r_step(s2r, S2R_STEP_SIO, S2R_DONE_SIO, apple_sleep_sio_off);
	if (ret)
		goto fail;
	ret = apple_s2r_step(s2r, S2R_STEP_WAKE, S2R_DONE_WAKE,
			     apple_sleep_ap_wake_arm);
	if (ret)
		goto fail;

	if (!(skip & S2R_DONE_CLOCKS)) {
		ret = apple_s2r_check(s2r, S2R_STEP_CLKGEN);
		if (ret)
			goto fail;
		s2r->done |= S2R_DONE_CLOCKS;
		ret = apple_s2r_clocks_off(s2r);
		if (ret)
			goto fail;
	}

	ret = apple_s2r_check(s2r, S2R_STEP_SUSPEND);
	if (ret)
		goto fail;
	if (!(skip & S2R_SKIP_TIMER))
		writel(S2R_TIMER_START, s2r->timer);

	/* Armed by the finisher, once nothing is left that could abort */
	if (s2r->wdt && wdt_timeout) {
		s2r->wdt_saved_ctl = readl_relaxed(s2r->wdt + WDT_CTL);
		s2r->wdt_saved_alarm = readl_relaxed(s2r->wdt + WDT_ALARM);
		s2r->wdt_armed = true;
	}

	ret = cpu_suspend((unsigned long)s2r, apple_s2r_finish);
	if (ret)
		goto fail;

	s2r->resumed = true;
	s2r->failed_step = S2R_STEP_NONE;
	return 0;

fail:
	s2r->failed_ret = ret;
	return ret;
}

static void apple_s2r_syscore_resume(void *data)
{
	struct apple_s2r *s2r = data;
	unsigned long done = s2r->done;

	/* First, before anything that could hang until the reset */
	if (s2r->wdt_armed) {
		apple_s2r_wdt_restore(s2r);
		s2r->wdt_armed = false;
	}

	if (!done)
		return;

	if (s2r->resumed)
		apple_sleep_pmgr_wake();
	if (done & S2R_DONE_CPM)
		apple_sleep_cpm_restore();
	if (s2r->resumed) {
		apple_s2r_sep_on(s2r);
		apple_cpu_power_cycled();
	}
	if (done & S2R_DONE_CPU_CLUSTERS)
		apple_s2r_cpu_clusters_restore(s2r);
	if (done & S2R_DONE_ACC)
		apple_cpu_acc_restore();
	if (done & S2R_DONE_SIO)
		apple_sleep_sio_restore();
	if (done & S2R_DONE_SOC_CLUSTERS)
		apple_sleep_soc_clusters_restore();
	if (done & S2R_DONE_AFR)
		apple_sleep_afr_restore();
	if (done & S2R_DONE_WAKE)
		apple_sleep_ap_wake_restore();
}

static const struct syscore_ops apple_s2r_syscore_ops = {
	.resume = apple_s2r_syscore_resume,
};

static void apple_s2r_wake(void)
{
	struct apple_s2r *s2r = apple_s2r;
	struct apple_s2r_mailbox *mb = s2r->mailbox;
	u64 flags;

	/* The firmware updated the mailbox with the caches off */
	dcache_clean_inval_poc((unsigned long)mb, (unsigned long)(mb + 1));
	flags = le64_to_cpu(READ_ONCE(mb->flags));

	/* Disarm, and stop muting the firmware console on CPU restarts */
	WRITE_ONCE(mb->magic, 0);
	WRITE_ONCE(mb->flags, 0);
	dcache_clean_poc((unsigned long)mb, (unsigned long)(mb + 1));

	if (s2r->resumed)
		dev_info(s2r->dev, "resumed from suspend-to-RAM%s\n",
			 flags & S2R_FLAG_RESUMED ? " through the firmware warm boot path" : "");
	else if (s2r->failed_step != S2R_STEP_NONE)
		dev_err(s2r->dev, "suspend-to-RAM aborted at %s: %d\n",
			apple_s2r_step_names[s2r->failed_step], s2r->failed_ret);
	if (s2r->restore_ret)
		dev_warn(s2r->dev, "power state restore timed out\n");
	if ((s2r->done & S2R_DONE_CLOCKS) && !s2r->resumed)
		dev_warn(s2r->dev, "SoC clocks were switched without sleeping\n");

	s2r->done = 0;
	s2r->resumed = false;
	s2r->failed_step = S2R_STEP_NONE;
	s2r->restore_ret = 0;
}

static const struct platform_suspend_ops apple_s2r_ops = {
	.valid = suspend_valid_only_mem,
	.enter = apple_s2r_enter,
	.wake = apple_s2r_wake,
};

/* The watchdog driver owns the registers: map them without claiming them */
static int apple_s2r_map_wdt(struct apple_s2r *s2r)
{
	struct device_node *wdt_np;
	struct resource res;
	int ret;

	wdt_np = of_parse_phandle(s2r->dev->of_node, "apple,watchdog", 0);
	if (!wdt_np)
		return 0;
	ret = of_address_to_resource(wdt_np, 0, &res);
	of_node_put(wdt_np);
	if (ret)
		return ret;

	s2r->wdt = devm_ioremap(s2r->dev, res.start, resource_size(&res));

	return s2r->wdt ? 0 : -ENOMEM;
}

static int apple_s2r_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *sep_np;
	struct apple_s2r *s2r;
	u64 mailbox;
	int ret;

	if (!enable) {
		dev_info(dev, "suspend-to-RAM disabled; enable with apple_s2r.enable=1\n");
		return -ENODEV;
	}

	if (!is_kernel_in_hyp_mode())
		return dev_err_probe(dev, -ENODEV, "warm boot resumes at EL2\n");

	if (of_property_read_u64(of_chosen, "asahi,s2r-mailbox", &mailbox))
		return dev_err_probe(dev, -ENODEV,
				     "boot firmware has no warm boot mailbox\n");

	s2r = devm_kzalloc(dev, sizeof(*s2r), GFP_KERNEL);
	if (!s2r)
		return -ENOMEM;
	s2r->dev = dev;

	s2r->mailbox = devm_memremap(dev, mailbox, sizeof(*s2r->mailbox),
				     MEMREMAP_WB);
	if (IS_ERR(s2r->mailbox))
		return dev_err_probe(dev, PTR_ERR(s2r->mailbox),
				     "failed to map the warm boot mailbox\n");

	s2r->clkgen = devm_platform_ioremap_resource_byname(pdev, "clkgen");
	if (IS_ERR(s2r->clkgen))
		return PTR_ERR(s2r->clkgen);
	s2r->pll = devm_platform_ioremap_resource_byname(pdev, "pll");
	if (IS_ERR(s2r->pll))
		return PTR_ERR(s2r->pll);
	s2r->timer = devm_platform_ioremap_resource_byname(pdev, "s2r-timer");
	if (IS_ERR(s2r->timer))
		return PTR_ERR(s2r->timer);

	s2r->pmgr = syscon_regmap_lookup_by_phandle(np, "apple,pmgr");
	if (IS_ERR(s2r->pmgr))
		return dev_err_probe(dev, PTR_ERR(s2r->pmgr), "no PMGR syscon\n");

	ret = of_property_read_variable_u32_array(np, "apple,cpu-clusters",
						  s2r->cpu_clusters, 1,
						  S2R_MAX_CLUSTERS);
	if (ret < 0)
		return dev_err_probe(dev, ret, "invalid apple,cpu-clusters\n");
	s2r->nr_cpu_clusters = ret;

	sep_np = of_parse_phandle(np, "apple,sep-domain", 0);
	if (!sep_np)
		return dev_err_probe(dev, -EINVAL, "missing apple,sep-domain\n");
	s2r->sep = syscon_node_to_regmap(sep_np->parent);
	ret = of_property_read_u32(sep_np, "reg", &s2r->sep_reg);
	of_node_put(sep_np);
	if (IS_ERR(s2r->sep))
		return dev_err_probe(dev, PTR_ERR(s2r->sep), "no SEP power state syscon\n");
	if (ret)
		return dev_err_probe(dev, ret, "invalid SEP power state node\n");

	ret = apple_s2r_map_wdt(s2r);
	if (ret)
		return dev_err_probe(dev, ret, "failed to map apple,watchdog\n");

	apple_s2r = s2r;

	/* Registered after the AIC: its resume runs once the AIC is restored */
	s2r->syscore.ops = &apple_s2r_syscore_ops;
	s2r->syscore.data = s2r;
	register_syscore(&s2r->syscore);

	suspend_set_ops(&apple_s2r_ops);

	dev_info(dev, "suspend-to-RAM available (mailbox at %#llx)\n", mailbox);

	return 0;
}

static const struct of_device_id apple_s2r_of_match[] = {
	{ .compatible = "apple,t6021-pmgr-s2r" },
	{}
};

static struct platform_driver apple_s2r_driver = {
	.probe = apple_s2r_probe,
	.driver = {
		.name = "apple-s2r",
		.of_match_table = apple_s2r_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(apple_s2r_driver);
