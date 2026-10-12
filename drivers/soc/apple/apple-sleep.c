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
 */

#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
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
	unsigned int wake_latched;
	unsigned int n_wake;
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
	void __iomem *wake;

	struct apple_sleep_counts *before;
	bool have_before;
	u32 wake_last[16];
	bool have_wake_last;

	struct notifier_block pm_nb;
	struct dentry *debugfs;
};

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

static void __iomem *apple_sleep_optional_reg(struct platform_device *pdev,
					      const char *name)
{
	if (!platform_get_resource_byname(pdev, IORESOURCE_MEM, name))
		return NULL;

	return devm_platform_ioremap_resource_byname(pdev, name);
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

	s->wake = apple_sleep_optional_reg(pdev, "wake");
	if (IS_ERR(s->wake))
		return PTR_ERR(s->wake);

	s->debugfs = debugfs_create_dir("apple-sleep", NULL);
	debugfs_create_file("events", 0400, s->debugfs, s,
			    &apple_sleep_events_fops);
	debugfs_create_file("wake_sources", 0400, s->debugfs, s,
			    &apple_sleep_wake_fops);

	s->pm_nb.notifier_call = apple_sleep_pm_notify;
	ret = register_pm_notifier(&s->pm_nb);
	if (ret) {
		debugfs_remove_recursive(s->debugfs);
		return ret;
	}

	apple_sleep_counters_init(s);
	apple_sleep_log_wake_times(s, " at boot");

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

static const struct apple_sleep_hw t6020_sleep_hw = {
	.events = t6020_events,
	.n_events = ARRAY_SIZE(t6020_events),
	.wake_times = t6020_wake_times,
	.n_wake_times = ARRAY_SIZE(t6020_wake_times),
	.wake_latched = 0x30,
	.n_wake = 9,
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
