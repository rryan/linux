// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SoC system sleep support
 *
 * The PMGR counts how often, and for how long, the SoC has been in each of
 * its sleep states (DDR self-refresh, suspend-to-RAM, deep wait, ...). This
 * driver enables those counters, logs their values at boot (they are kept
 * across a reset), exposes them in debugfs and logs how much each of them
 * advanced across every system suspend, together with the AP wake sources
 * latched by the always-on fabric and the sleep exit times.
 *
 * It can also power down parts of the SoC that stay up during
 * suspend-to-idle. Each step is opt-in through the "stages" parameter:
 *
 *   BIT(1) A2: clear AUTO_ENABLE on the SIO power states, so that SIO
 *          accesses do not power them back up
 *   BIT(2) A3: turn off the AFR media fabric power state (when the
 *          always-on side allows it and no domain behind it is up)
 *   BIT(3) A4: power down the non-CPU SoC clusters (neural engine,
 *          display, media), filtered by the "soc_clusters" mask
 *   BIT(4) A5: enable CPM power domain gating
 *
 * BIT(0) (A1, staged fabric/DCS pstates) is handled by apple-pmgr-misc
 * ("staged_pstate" parameter) in its noirq callbacks.
 *
 * Ordering: the steps run from the s2idle prepare_late callback, i.e. after
 * every device's noirq suspend (so after the PS domains have been put into
 * their final state and after apple-pmgr-misc has lowered fabric/DCS), and
 * they are undone in reverse order from restore_early, before any noirq
 * resume callback runs. The core always calls restore_early after a
 * prepare_late, also when prepare_late fails, so a failed step is rolled
 * back there.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/soc/apple/sleep.h>
#include <linux/suspend.h>

/*
 * PMGR counter blocks: a global enable at offset 0 and, from 0x100, one
 * 16-byte slot per counter holding a config word and, from +8, the 64-bit
 * value, low word first. A sleep state has an entry counter, configured
 * with APPLE_SLEEP_COUNTER_ENTRIES, and a residency counter.
 */
#define APPLE_SLEEP_COUNTERS_CTL	0x0
#define APPLE_SLEEP_COUNTERS_EN		BIT(0)
#define APPLE_SLEEP_COUNTER_CFG(idx)	(0x100 + 16 * (idx))
#define APPLE_SLEEP_COUNTER_EN		BIT(0)
#define APPLE_SLEEP_COUNTER_ENTRIES	BIT(1)
#define APPLE_SLEEP_COUNTER_LO(idx)	(0x108 + 16 * (idx))
#define APPLE_SLEEP_COUNTER_HI(idx)	(0x10c + 16 * (idx))

/* PS (power state) registers */
#define APPLE_PS_AUTO_ENABLE		BIT(28)
#define APPLE_PS_MIN			GENMASK(19, 16)
#define APPLE_PS_WAS_CLKGATED		BIT(9)
#define APPLE_PS_WAS_PWRGATED		BIT(8)
#define APPLE_PS_ACTUAL			GENMASK(7, 4)
#define APPLE_PS_TARGET			GENMASK(3, 0)
/* Write-one-to-clear status: always written as 0 */
#define APPLE_PS_FLAGS			(APPLE_PS_WAS_CLKGATED | APPLE_PS_WAS_PWRGATED)

/* SoC cluster power control */
#define APPLE_CLUSTER_TARGET		GENMASK(3, 0)
#define APPLE_CLUSTER_BUSY		BIT(4)

/* While both are set, the AFR power state must be left alone */
#define APPLE_AON_GFX_AFR_HOLD		GENMASK(19, 18)

/* Start of the "cpm" range; bit 31 is set on every exit from suspend-to-RAM */
#define APPLE_CPM_CTRL_WAKE		BIT(31)

/* CPM power domain gating, one register per CPU complex */
#define APPLE_CPM_GATING_EN		BIT(31)

#define APPLE_SLEEP_TIMEOUT_US		8000

#define APPLE_SLEEP_A2_SIO		BIT(1)
#define APPLE_SLEEP_A3_AFR		BIT(2)
#define APPLE_SLEEP_A4_SOC_CLUSTERS	BIT(3)
#define APPLE_SLEEP_A5_CPM		BIT(4)

static unsigned int stages;
module_param(stages, uint, 0644);
MODULE_PARM_DESC(stages,
		 "SoC power-down steps applied in suspend-to-idle: 0x2 SIO auto-enable off, 0x4 AFR off, 0x8 non-CPU SoC clusters off, 0x10 CPM gating (default: 0)");

static unsigned int soc_clusters = ~0U;
module_param(soc_clusters, uint, 0644);
MODULE_PARM_DESC(soc_clusters,
		 "Mask over apple,soc-clusters entries powered down by stage 0x8 (default: all)");

#define APPLE_SLEEP_MAX_SIO		4
#define APPLE_SLEEP_MAX_AFR_DEPS	8
#define APPLE_SLEEP_MAX_CLUSTERS	8

struct apple_sleep_event {
	const char *name;
	u8 count;
	u8 time;
};

struct apple_sleep_wake_time {
	const char *name;
	u8 idx;
};

struct apple_sleep_hw {
	const struct apple_sleep_event *events;
	unsigned int n_events;
	const struct apple_sleep_wake_time *wake_times;
	unsigned int n_wake_times;
	unsigned int cluster_base;
	unsigned int n_cpm;
	unsigned int cpm_gating;
	unsigned int wake_mask;
	unsigned int wake_latched;
	unsigned int n_wake;
	/* Sources that exist in each mask word */
	const u32 *wake_valid;
};

struct apple_sleep_ps {
	struct regmap *regmap;
	u32 offset;
	const char *name;
	u32 saved;
};

struct apple_sleep_counts {
	u64 count;
	u64 time;
};

struct apple_sleep {
	struct device *dev;
	const struct apple_sleep_hw *hw;

	void __iomem *events;
	void __iomem *wake_time;
	void __iomem *aon_gfx;
	void __iomem *cpm;
	void __iomem *wake;
	struct regmap *pmgr;

	struct apple_sleep_ps sio[APPLE_SLEEP_MAX_SIO];
	unsigned int n_sio;
	unsigned int sio_done;

	struct apple_sleep_ps afr;
	bool has_afr;
	bool afr_done;
	struct apple_sleep_ps afr_deps[APPLE_SLEEP_MAX_AFR_DEPS];
	unsigned int n_afr_deps;

	u32 clusters[APPLE_SLEEP_MAX_CLUSTERS];
	u32 cluster_saved[APPLE_SLEEP_MAX_CLUSTERS];
	unsigned int n_clusters;
	unsigned int clusters_done;

	u32 cpm_saved[4];
	bool cpm_done;

	struct apple_sleep_counts *before;
	bool have_before;
	u32 wake_last[16];
	bool have_wake_last;

	/* AP wake source masks for suspend-to-RAM: 1 = masked */
	u32 wake_armed[16];
	u32 wake_saved[16];
	bool has_wake_sources;
	bool wake_done;

	struct notifier_block pm_nb;
	struct dentry *debugfs;
};

static struct apple_sleep *apple_sleep;

static u64 apple_sleep_read_counter(void __iomem *base, unsigned int idx)
{
	u32 hi, lo, hi2;

	/* The high word may carry between the two reads: retry until stable */
	hi = readl(base + APPLE_SLEEP_COUNTER_HI(idx));
	do {
		hi2 = hi;
		lo = readl(base + APPLE_SLEEP_COUNTER_LO(idx));
		hi = readl(base + APPLE_SLEEP_COUNTER_HI(idx));
	} while (hi != hi2);

	return ((u64)hi << 32) | lo;
}

static void apple_sleep_read_counts(struct apple_sleep *s,
				    struct apple_sleep_counts *c)
{
	unsigned int i;

	for (i = 0; i < s->hw->n_events; i++) {
		c[i].count = apple_sleep_read_counter(s->events,
						      s->hw->events[i].count);
		c[i].time = apple_sleep_read_counter(s->events,
						     s->hw->events[i].time);
	}
}

static void apple_sleep_counter_enable(struct apple_sleep *s, unsigned int idx,
				       bool entries)
{
	void __iomem *reg = s->events + APPLE_SLEEP_COUNTER_CFG(idx);
	u32 val = readl(reg) & ~APPLE_SLEEP_COUNTER_ENTRIES;

	val |= APPLE_SLEEP_COUNTER_EN;
	if (entries)
		val |= APPLE_SLEEP_COUNTER_ENTRIES;
	writel(val, reg);
}

/*
 * Nothing before the kernel enables the counters. Log what they hold first:
 * the block is always-on, so the values may tell how far an earlier boot got
 * into its last sleep before a reset.
 */
static void apple_sleep_counters_init(struct apple_sleep *s)
{
	const struct apple_sleep_event *ev = s->hw->events;
	u32 ctl = readl(s->events + APPLE_SLEEP_COUNTERS_CTL);
	char buf[512], cfg[256];
	int len = 0, cfg_len = 0;
	unsigned int i;

	for (i = 0; i < s->hw->n_events; i++) {
		len += scnprintf(buf + len, sizeof(buf) - len, " %s %llu/%llu",
				 ev[i].name,
				 apple_sleep_read_counter(s->events, ev[i].count),
				 apple_sleep_read_counter(s->events, ev[i].time));
		cfg_len += scnprintf(cfg + cfg_len, sizeof(cfg) - cfg_len,
				     " %x/%x",
				     readl(s->events + APPLE_SLEEP_COUNTER_CFG(ev[i].count)),
				     readl(s->events + APPLE_SLEEP_COUNTER_CFG(ev[i].time)));
	}
	dev_info(s->dev, "sleep events at boot (count/time):%s\n", buf);
	dev_info(s->dev, "sleep event counter config at boot: ctl %#x%s\n",
		 ctl, cfg);

	for (i = 0; i < s->hw->n_events; i++) {
		apple_sleep_counter_enable(s, ev[i].count, true);
		apple_sleep_counter_enable(s, ev[i].time, false);
	}
	writel(ctl | APPLE_SLEEP_COUNTERS_EN,
	       s->events + APPLE_SLEEP_COUNTERS_CTL);
}

/* How long the SoC took to leave its last DDR self-refresh / suspend-to-RAM */
static void apple_sleep_log_wake_times(struct apple_sleep *s, const char *when)
{
	char buf[160];
	unsigned int i;
	int len = 0;

	if (!s->wake_time)
		return;

	for (i = 0; i < s->hw->n_wake_times; i++) {
		unsigned int idx = s->hw->wake_times[i].idx;

		len += scnprintf(buf + len, sizeof(buf) - len, " %s %llu (cfg %#x)",
				 s->hw->wake_times[i].name,
				 apple_sleep_read_counter(s->wake_time, idx),
				 readl(s->wake_time + APPLE_SLEEP_COUNTER_CFG(idx)));
	}

	dev_info(s->dev, "sleep exit times%s (ctl %#x):%s\n", when,
		 readl(s->wake_time + APPLE_SLEEP_COUNTERS_CTL), buf);
}

static void apple_sleep_log_wake(struct apple_sleep *s)
{
	char buf[160];
	unsigned int i;
	int len = 0;

	if (!s->wake)
		return;

	for (i = 0; i < s->hw->n_wake; i++) {
		s->wake_last[i] = readl(s->wake + s->hw->wake_latched + 4 * i);
		len += scnprintf(buf + len, sizeof(buf) - len, " %08x",
				 s->wake_last[i]);
	}
	s->have_wake_last = true;

	dev_info(s->dev, "AP wake sources:%s\n", buf);
}

static void apple_sleep_log_deltas(struct apple_sleep *s)
{
	struct apple_sleep_counts now;
	char buf[512];
	unsigned int i;
	int len = 0;

	for (i = 0; i < s->hw->n_events; i++) {
		now.count = apple_sleep_read_counter(s->events,
						     s->hw->events[i].count);
		now.time = apple_sleep_read_counter(s->events,
						    s->hw->events[i].time);
		len += scnprintf(buf + len, sizeof(buf) - len, " %s +%llu/+%llu",
				 s->hw->events[i].name,
				 now.count - s->before[i].count,
				 now.time - s->before[i].time);
	}

	dev_info(s->dev, "sleep events (count/time):%s\n", buf);
}

static int apple_sleep_pm_notify(struct notifier_block *nb,
				 unsigned long action, void *data)
{
	struct apple_sleep *s = container_of(nb, struct apple_sleep, pm_nb);

	switch (action) {
	case PM_SUSPEND_PREPARE:
		apple_sleep_read_counts(s, s->before);
		s->have_before = true;
		break;
	case PM_POST_SUSPEND:
		if (s->have_before)
			apple_sleep_log_deltas(s);
		s->have_before = false;
		apple_sleep_log_wake(s);
		apple_sleep_log_wake_times(s, "");
		break;
	}

	return NOTIFY_DONE;
}

static int apple_sleep_ps_read(struct apple_sleep_ps *ps, u32 *val)
{
	return regmap_read(ps->regmap, ps->offset, val);
}

static int apple_sleep_ps_update(struct apple_sleep_ps *ps, u32 mask, u32 val)
{
	return regmap_update_bits(ps->regmap, ps->offset,
				  APPLE_PS_FLAGS | mask, val);
}

/*
 * regmap_read_poll_timeout_atomic() keeps time with ktime_get(), which must
 * not be used once timekeeping is suspended: poll with a delay count.
 */
static u32 apple_sleep_regmap_read(struct regmap *map, unsigned int reg)
{
	u32 val = 0;

	regmap_read(map, reg, &val);
	return val;
}

static int apple_sleep_ps_wait_actual(struct apple_sleep_ps *ps, u32 state)
{
	u32 val;

	return read_poll_timeout_atomic(apple_sleep_regmap_read, val,
					FIELD_GET(APPLE_PS_ACTUAL, val) == state,
					1, APPLE_SLEEP_TIMEOUT_US, false,
					ps->regmap, ps->offset);
}

/* A2 */
int apple_sleep_sio_off(void)
{
	struct apple_sleep *s = apple_sleep;
	unsigned int i;
	int ret;

	if (!s)
		return -ENODEV;

	for (i = 0; i < s->n_sio; i++) {
		struct apple_sleep_ps *ps = &s->sio[i];

		ret = apple_sleep_ps_read(ps, &ps->saved);
		if (ret)
			return ret;

		ret = apple_sleep_ps_update(ps, APPLE_PS_AUTO_ENABLE, 0);
		if (ret)
			return ret;
		s->sio_done = i + 1;

		ret = apple_sleep_ps_wait_actual(ps, FIELD_GET(APPLE_PS_TARGET,
							       ps->saved));
		if (ret) {
			dev_err(s->dev, "%s did not settle with auto-enable off\n",
				ps->name);
			return ret;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(apple_sleep_sio_off);

void apple_sleep_sio_restore(void)
{
	struct apple_sleep *s = apple_sleep;

	if (!s)
		return;

	while (s->sio_done) {
		struct apple_sleep_ps *ps = &s->sio[--s->sio_done];

		apple_sleep_ps_update(ps, APPLE_PS_AUTO_ENABLE,
				      ps->saved & APPLE_PS_AUTO_ENABLE);
	}
}
EXPORT_SYMBOL_GPL(apple_sleep_sio_restore);

/* A3 */
int apple_sleep_afr_off(void)
{
	struct apple_sleep *s = apple_sleep;
	struct apple_sleep_ps *afr;
	unsigned int i;
	u32 val;
	int ret;

	if (!s)
		return -ENODEV;
	if (!s->has_afr)
		return 0;
	afr = &s->afr;

	if (s->aon_gfx) {
		val = readl(s->aon_gfx);
		if ((val & APPLE_AON_GFX_AFR_HOLD) == APPLE_AON_GFX_AFR_HOLD) {
			dev_info(s->dev, "AFR held on (0x%08x), leaving it\n", val);
			return 0;
		}
	}

	for (i = 0; i < s->n_afr_deps; i++) {
		ret = apple_sleep_ps_read(&s->afr_deps[i], &val);
		if (ret)
			return ret;
		if (FIELD_GET(APPLE_PS_ACTUAL, val)) {
			dev_info(s->dev, "%s is on (0x%08x), leaving AFR on\n",
				 s->afr_deps[i].name, val);
			return 0;
		}
	}

	ret = apple_sleep_ps_read(afr, &afr->saved);
	if (ret)
		return ret;

	ret = apple_sleep_ps_update(afr, APPLE_PS_MIN | APPLE_PS_TARGET, 0);
	if (ret)
		return ret;
	s->afr_done = true;

	ret = apple_sleep_ps_wait_actual(afr, 0);
	if (ret)
		dev_err(s->dev, "%s did not power down\n", afr->name);

	return ret;
}
EXPORT_SYMBOL_GPL(apple_sleep_afr_off);

void apple_sleep_afr_restore(void)
{
	struct apple_sleep *s = apple_sleep;
	struct apple_sleep_ps *afr;

	if (!s || !s->afr_done)
		return;
	afr = &s->afr;
	s->afr_done = false;

	apple_sleep_ps_update(afr, APPLE_PS_MIN | APPLE_PS_TARGET,
			      afr->saved & (APPLE_PS_MIN | APPLE_PS_TARGET));
	if (apple_sleep_ps_wait_actual(afr, FIELD_GET(APPLE_PS_TARGET,
						      afr->saved)))
		dev_err(s->dev, "%s did not power back up\n", afr->name);
}
EXPORT_SYMBOL_GPL(apple_sleep_afr_restore);

/* A4 */
static unsigned int apple_sleep_cluster_reg(struct apple_sleep *s,
					    unsigned int i)
{
	return s->hw->cluster_base + 4 * s->clusters[i];
}

static int apple_sleep_cluster_set(struct apple_sleep *s, unsigned int i,
				   u32 target)
{
	unsigned int reg = apple_sleep_cluster_reg(s, i);
	u32 val;
	int ret;

	ret = regmap_update_bits(s->pmgr, reg, APPLE_CLUSTER_TARGET, target);
	if (ret)
		return ret;

	ret = read_poll_timeout_atomic(apple_sleep_regmap_read, val,
				       !(val & APPLE_CLUSTER_BUSY), 1,
				       APPLE_SLEEP_TIMEOUT_US, false,
				       s->pmgr, reg);
	if (ret)
		dev_err(s->dev, "SoC cluster %u stuck busy: 0x%08x\n",
			s->clusters[i], val);

	return ret;
}

int apple_sleep_soc_clusters_off(void)
{
	struct apple_sleep *s = apple_sleep;
	unsigned int mask = READ_ONCE(soc_clusters);
	unsigned int i;
	int ret;

	if (!s)
		return -ENODEV;

	for (i = 0; i < s->n_clusters; i++) {
		if (!(mask & BIT(i)))
			continue;

		ret = regmap_read(s->pmgr, apple_sleep_cluster_reg(s, i),
				  &s->cluster_saved[i]);
		if (ret)
			return ret;

		s->clusters_done |= BIT(i);
		ret = apple_sleep_cluster_set(s, i, 0);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(apple_sleep_soc_clusters_off);

void apple_sleep_soc_clusters_restore(void)
{
	struct apple_sleep *s = apple_sleep;
	unsigned int i;

	if (!s)
		return;

	for (i = s->n_clusters; i-- > 0;) {
		if (!(s->clusters_done & BIT(i)))
			continue;

		s->clusters_done &= ~BIT(i);
		apple_sleep_cluster_set(s, i, s->cluster_saved[i] &
					APPLE_CLUSTER_TARGET);
	}
}
EXPORT_SYMBOL_GPL(apple_sleep_soc_clusters_restore);

/* A5 */
static void __iomem *apple_sleep_cpm_reg(struct apple_sleep *s, unsigned int i)
{
	return s->cpm + s->hw->cpm_gating + 4 * i;
}

int apple_sleep_cpm_gate(void)
{
	struct apple_sleep *s = apple_sleep;
	unsigned int i;

	if (!s)
		return -ENODEV;
	if (!s->cpm)
		return 0;

	for (i = 0; i < s->hw->n_cpm; i++) {
		void __iomem *reg = apple_sleep_cpm_reg(s, i);

		s->cpm_saved[i] = readl(reg);
		writel(s->cpm_saved[i] | APPLE_CPM_GATING_EN, reg);
	}
	s->cpm_done = true;

	return 0;
}
EXPORT_SYMBOL_GPL(apple_sleep_cpm_gate);

void apple_sleep_cpm_restore(void)
{
	struct apple_sleep *s = apple_sleep;
	unsigned int i;

	if (!s || !s->cpm_done)
		return;
	s->cpm_done = false;

	for (i = 0; i < s->hw->n_cpm; i++) {
		void __iomem *reg = apple_sleep_cpm_reg(s, i);
		u32 val = readl(reg) & ~APPLE_CPM_GATING_EN;

		writel(val | (s->cpm_saved[i] & APPLE_CPM_GATING_EN), reg);
	}
}
EXPORT_SYMBOL_GPL(apple_sleep_cpm_restore);

/* Exit from suspend-to-RAM; not used for suspend-to-idle */
void apple_sleep_pmgr_wake(void)
{
	struct apple_sleep *s = apple_sleep;

	if (!s || !s->cpm)
		return;

	writel(readl(s->cpm) | APPLE_CPM_CTRL_WAKE, s->cpm);
}
EXPORT_SYMBOL_GPL(apple_sleep_pmgr_wake);

int apple_sleep_ap_wake_arm(void)
{
	struct apple_sleep *s = apple_sleep;
	unsigned int i;

	if (!s || !s->wake)
		return -ENODEV;
	if (!s->has_wake_sources)
		return -ENOENT;

	for (i = 0; i < s->hw->n_wake; i++) {
		void __iomem *reg = s->wake + s->hw->wake_mask + 4 * i;

		s->wake_saved[i] = readl(reg);
		writel(s->wake_armed[i], reg);
	}
	s->wake_done = true;

	return 0;
}
EXPORT_SYMBOL_GPL(apple_sleep_ap_wake_arm);

void apple_sleep_ap_wake_restore(void)
{
	struct apple_sleep *s = apple_sleep;
	unsigned int i;

	if (!s || !s->wake_done)
		return;
	s->wake_done = false;

	for (i = 0; i < s->hw->n_wake; i++)
		writel(s->wake_saved[i], s->wake + s->hw->wake_mask + 4 * i);
}
EXPORT_SYMBOL_GPL(apple_sleep_ap_wake_restore);

static int apple_sleep_s2idle_prepare_late(void)
{
	unsigned int st = READ_ONCE(stages);
	int ret = 0;

	if (!apple_sleep || !st)
		return 0;

	if (st & APPLE_SLEEP_A2_SIO)
		ret = apple_sleep_sio_off();
	if (!ret && (st & APPLE_SLEEP_A3_AFR))
		ret = apple_sleep_afr_off();
	if (!ret && (st & APPLE_SLEEP_A4_SOC_CLUSTERS))
		ret = apple_sleep_soc_clusters_off();
	if (!ret && (st & APPLE_SLEEP_A5_CPM))
		ret = apple_sleep_cpm_gate();

	return ret;
}

static void apple_sleep_s2idle_restore_early(void)
{
	apple_sleep_cpm_restore();
	apple_sleep_soc_clusters_restore();
	apple_sleep_afr_restore();
	apple_sleep_sio_restore();
}

static const struct platform_s2idle_ops apple_sleep_s2idle_ops = {
	.prepare_late = apple_sleep_s2idle_prepare_late,
	.restore_early = apple_sleep_s2idle_restore_early,
};

static int apple_sleep_events_show(struct seq_file *m, void *v)
{
	struct apple_sleep *s = m->private;
	unsigned int i;

	seq_puts(m, "event count time\n");
	for (i = 0; i < s->hw->n_events; i++)
		seq_printf(m, "%s %llu %llu\n", s->hw->events[i].name,
			   apple_sleep_read_counter(s->events, s->hw->events[i].count),
			   apple_sleep_read_counter(s->events, s->hw->events[i].time));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(apple_sleep_events);

static int apple_sleep_wake_show(struct seq_file *m, void *v)
{
	struct apple_sleep *s = m->private;
	unsigned int i;

	/* Latched at the last resume; reading here would not be a wake event */
	if (!s->have_wake_last)
		return 0;

	for (i = 0; i < s->hw->n_wake; i++)
		seq_printf(m, "%u 0x%08x\n", i, s->wake_last[i]);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(apple_sleep_wake);

static void apple_sleep_show_ps(struct seq_file *m, struct apple_sleep_ps *ps)
{
	u32 val;

	if (!apple_sleep_ps_read(ps, &val))
		seq_printf(m, "%s 0x%08x\n", ps->name, val);
}

static int apple_sleep_regs_show(struct seq_file *m, void *v)
{
	struct apple_sleep *s = m->private;
	unsigned int i;
	u32 val;

	for (i = 0; i < s->n_sio; i++)
		apple_sleep_show_ps(m, &s->sio[i]);
	if (s->has_afr)
		apple_sleep_show_ps(m, &s->afr);
	for (i = 0; i < s->n_afr_deps; i++)
		apple_sleep_show_ps(m, &s->afr_deps[i]);
	if (s->aon_gfx)
		seq_printf(m, "aon-gfx 0x%08x\n", readl(s->aon_gfx));
	for (i = 0; i < s->n_clusters; i++)
		if (!regmap_read(s->pmgr, apple_sleep_cluster_reg(s, i), &val))
			seq_printf(m, "soc-cluster%u 0x%08x\n", s->clusters[i], val);
	if (s->cpm) {
		seq_printf(m, "cpm-ctrl 0x%08x\n", readl(s->cpm));
		for (i = 0; i < s->hw->n_cpm; i++)
			seq_printf(m, "cpm-gating%u 0x%08x\n", i,
				   readl(apple_sleep_cpm_reg(s, i)));
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(apple_sleep_regs);

static void __iomem *apple_sleep_optional_reg(struct platform_device *pdev,
					      const char *name)
{
	if (!platform_get_resource_byname(pdev, IORESOURCE_MEM, name))
		return NULL;

	return devm_platform_ioremap_resource_byname(pdev, name);
}

static int apple_sleep_get_ps(struct device *dev, const char *prop,
			      unsigned int idx, struct apple_sleep_ps *ps)
{
	struct device_node *np;
	int ret;

	np = of_parse_phandle(dev->of_node, prop, idx);
	if (!np)
		return -ENOENT;

	ret = of_property_read_u32(np, "reg", &ps->offset);
	if (ret)
		goto out;

	ps->regmap = syscon_node_to_regmap(np->parent);
	if (IS_ERR(ps->regmap)) {
		ret = PTR_ERR(ps->regmap);
		goto out;
	}

	ps->name = devm_kasprintf(dev, GFP_KERNEL, "%pOFn@%x", np, ps->offset);
	if (!ps->name)
		ret = -ENOMEM;
out:
	of_node_put(np);
	return ret;
}

static int apple_sleep_get_ps_list(struct device *dev, const char *prop,
				   struct apple_sleep_ps *ps, unsigned int max,
				   unsigned int *n)
{
	int count, i, ret;

	count = of_count_phandle_with_args(dev->of_node, prop, NULL);
	if (count < 0) {
		*n = 0;
		return 0;
	}
	if (count > max)
		return dev_err_probe(dev, -EINVAL, "too many %s\n", prop);

	for (i = 0; i < count; i++) {
		ret = apple_sleep_get_ps(dev, prop, i, &ps[i]);
		if (ret)
			return dev_err_probe(dev, ret, "bad %s[%d]\n", prop, i);
	}
	*n = count;

	return 0;
}

/* Turn apple,ap-wake-sources into the masks written for suspend-to-RAM */
static int apple_sleep_get_wake_sources(struct apple_sleep *s)
{
	struct device *dev = s->dev;
	unsigned int n = s->hw->n_wake;
	int count, i, ret;
	u32 *bits;

	count = of_property_count_u32_elems(dev->of_node,
					    "apple,ap-wake-sources");
	if (count == -EINVAL)
		return 0;
	if (count <= 0)
		return dev_err_probe(dev, -EINVAL,
				     "bad apple,ap-wake-sources\n");
	if (!s->wake)
		return dev_err_probe(dev, -EINVAL,
				     "apple,ap-wake-sources without wake registers\n");

	bits = kcalloc(count, sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;
	ret = of_property_read_u32_array(dev->of_node, "apple,ap-wake-sources",
					 bits, count);
	if (ret)
		goto out;

	for (i = 0; i < n; i++)
		s->wake_armed[i] = s->hw->wake_valid[i];
	for (i = 0; i < count; i++) {
		u32 w = bits[i] / 32, b = BIT(bits[i] % 32);

		if (w >= n || !(s->hw->wake_valid[w] & b)) {
			ret = dev_err_probe(dev, -EINVAL,
					    "invalid AP wake source %u\n",
					    bits[i]);
			goto out;
		}
		s->wake_armed[w] &= ~b;
	}
	s->has_wake_sources = true;

out:
	kfree(bits);
	return ret;
}

static int apple_sleep_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_sleep *s;
	int ret;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	s->dev = dev;
	s->hw = of_device_get_match_data(dev);
	if (!s->hw)
		return -ENODEV;

	s->before = devm_kcalloc(dev, s->hw->n_events, sizeof(*s->before),
				 GFP_KERNEL);
	if (!s->before)
		return -ENOMEM;

	s->events = devm_platform_ioremap_resource_byname(pdev, "events");
	if (IS_ERR(s->events))
		return PTR_ERR(s->events);

	s->wake_time = apple_sleep_optional_reg(pdev, "wake-time");
	if (IS_ERR(s->wake_time))
		return PTR_ERR(s->wake_time);

	s->aon_gfx = apple_sleep_optional_reg(pdev, "aon-gfx");
	if (IS_ERR(s->aon_gfx))
		return PTR_ERR(s->aon_gfx);
	s->cpm = apple_sleep_optional_reg(pdev, "cpm");
	if (IS_ERR(s->cpm))
		return PTR_ERR(s->cpm);
	s->wake = apple_sleep_optional_reg(pdev, "wake");
	if (IS_ERR(s->wake))
		return PTR_ERR(s->wake);

	ret = of_property_count_u32_elems(dev->of_node, "apple,soc-clusters");
	if (ret > 0) {
		if (ret > APPLE_SLEEP_MAX_CLUSTERS)
			return dev_err_probe(dev, -EINVAL,
					     "too many apple,soc-clusters\n");
		s->n_clusters = ret;
		ret = of_property_read_u32_array(dev->of_node,
						 "apple,soc-clusters",
						 s->clusters, s->n_clusters);
		if (ret)
			return ret;

		s->pmgr = syscon_regmap_lookup_by_phandle(dev->of_node,
							  "apple,pmgr");
		if (IS_ERR(s->pmgr))
			return dev_err_probe(dev, PTR_ERR(s->pmgr),
					     "no apple,pmgr\n");
	}

	ret = apple_sleep_get_ps_list(dev, "apple,sio-domains", s->sio,
				      APPLE_SLEEP_MAX_SIO, &s->n_sio);
	if (ret)
		return ret;

	ret = apple_sleep_get_ps(dev, "apple,afr-domain", 0, &s->afr);
	if (!ret)
		s->has_afr = true;
	else if (ret != -ENOENT)
		return dev_err_probe(dev, ret, "bad apple,afr-domain\n");

	ret = apple_sleep_get_ps_list(dev, "apple,afr-dependents", s->afr_deps,
				      APPLE_SLEEP_MAX_AFR_DEPS, &s->n_afr_deps);
	if (ret)
		return ret;

	ret = apple_sleep_get_wake_sources(s);
	if (ret)
		return ret;

	s->debugfs = debugfs_create_dir("apple-sleep", NULL);
	debugfs_create_file("events", 0400, s->debugfs, s,
			    &apple_sleep_events_fops);
	debugfs_create_file("wake_sources", 0400, s->debugfs, s,
			    &apple_sleep_wake_fops);
	debugfs_create_file("regs", 0400, s->debugfs, s,
			    &apple_sleep_regs_fops);

	s->pm_nb.notifier_call = apple_sleep_pm_notify;
	ret = register_pm_notifier(&s->pm_nb);
	if (ret) {
		debugfs_remove_recursive(s->debugfs);
		return ret;
	}

	apple_sleep_counters_init(s);
	apple_sleep_log_wake_times(s, " at boot");

	apple_sleep = s;
	s2idle_set_ops(&apple_sleep_s2idle_ops);

	return 0;
}

static const struct apple_sleep_event t6020_events[] = {
	{ "SLP_DDR", 43, 42 },
	{ "AWAKE", 45, 44 },
	{ "SLP_S2R", 47, 46 },
	{ "DEEP_WAIT", 49, 48 },
	{ "SYS_AWAKE", 51, 50 },
	{ "SYS_S2R_AOP", 53, 52 },
	{ "SYS_SLP_DDR", 55, 54 },
};

/* Sleep exit times, in the "wake-time" counter block */
static const struct apple_sleep_wake_time t6020_wake_times[] = {
	{ "DDR_WK_TIME", 40 },
	{ "S2R_WK_TIME", 38 },
};

static const u32 t6020_wake_valid[] = {
	0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
	0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
	0x7f,
};

static const struct apple_sleep_hw t6020_sleep_hw = {
	.events = t6020_events,
	.n_events = ARRAY_SIZE(t6020_events),
	.wake_times = t6020_wake_times,
	.n_wake_times = ARRAY_SIZE(t6020_wake_times),
	.cluster_base = 0x8000,
	.n_cpm = 3,
	.cpm_gating = 0x30,
	.wake_mask = 0xc,
	.wake_latched = 0x30,
	.n_wake = ARRAY_SIZE(t6020_wake_valid),
	.wake_valid = t6020_wake_valid,
};

static const struct of_device_id apple_sleep_of_match[] = {
	{ .compatible = "apple,t6020-pmgr-sleep", .data = &t6020_sleep_hw },
	{}
};

static struct platform_driver apple_sleep_driver = {
	.probe = apple_sleep_probe,
	.driver = {
		.name = "apple-sleep",
		.of_match_table = apple_sleep_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(apple_sleep_driver);
