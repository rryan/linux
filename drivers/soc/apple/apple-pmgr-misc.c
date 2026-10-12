// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SoC PMGR device power state driver
 *
 * Copyright The Asahi Linux Contributors
 */

#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/module.h>

#define APPLE_CLKGEN_PSTATE 0
#define APPLE_CLKGEN_PSTATE_DESIRED GENMASK(3, 0)
#define APPLE_CLKGEN_PSTATE_BUSY BIT(31)

#define APPLE_CLKGEN_PSTATE_TIMEOUT_US 8000

#define SYS_DEV_PSTATE_SUSPEND 1

/* Placeholder in a pstate sequence for the pstate found at probe */
#define SYS_DEV_PSTATE_BOOT U32_MAX

static bool staged_pstate;
module_param(staged_pstate, bool, 0644);
MODULE_PARM_DESC(staged_pstate,
		 "Walk fabric/DCS through the SoC's staged sleep pstate sequence, waiting for each transition (default: off)");

enum sys_device {
	DEV_FABRIC,
	DEV_DCS,
	DEV_MAX,
};

struct apple_pmgr_pstate_step {
	enum sys_device dev;
	u32 pstate;
};

struct apple_pmgr_misc_hw {
	const struct apple_pmgr_pstate_step *suspend;
	unsigned int n_suspend;
	const struct apple_pmgr_pstate_step *resume;
	unsigned int n_resume;
};

struct apple_pmgr_sys_device {
	void __iomem *base;
	u32 active_state;
	u32 suspend_state;
};

struct apple_pmgr_misc {
	struct device *dev;
	const struct apple_pmgr_misc_hw *hw;
	bool staged;
	struct apple_pmgr_sys_device devices[DEV_MAX];
};

static void apple_pmgr_sys_dev_set_pstate(struct apple_pmgr_misc *misc,
					  enum sys_device dev, bool active)
{
	u32 pstate;
	u32 val;

	if (!misc->devices[dev].base)
		return;

	if (active)
		pstate = misc->devices[dev].active_state;
	else
		pstate = misc->devices[dev].suspend_state;

	printk("set %d ps to pstate %d\n", dev, pstate);

	val = readl_relaxed(misc->devices[dev].base + APPLE_CLKGEN_PSTATE);
	val &= ~APPLE_CLKGEN_PSTATE_DESIRED;
	val |= FIELD_PREP(APPLE_CLKGEN_PSTATE_DESIRED, pstate);
	writel_relaxed(val, misc->devices[dev].base);
}

/*
 * Request a pstate and wait for the clock generator to finish the
 * transition (busy bit clear) before the next one is requested.
 */
static int apple_pmgr_sys_dev_step(struct apple_pmgr_misc *misc,
				   const struct apple_pmgr_pstate_step *step)
{
	struct apple_pmgr_sys_device *sdev = &misc->devices[step->dev];
	void __iomem *reg = sdev->base + APPLE_CLKGEN_PSTATE;
	u32 pstate = step->pstate;
	u32 val;
	int ret;

	if (pstate == SYS_DEV_PSTATE_BOOT)
		pstate = sdev->active_state;

	val = readl_relaxed(reg);
	val &= ~APPLE_CLKGEN_PSTATE_DESIRED;
	val |= FIELD_PREP(APPLE_CLKGEN_PSTATE_DESIRED, pstate);
	writel_relaxed(val, reg);

	ret = readl_poll_timeout_atomic(reg, val,
					!(val & APPLE_CLKGEN_PSTATE_BUSY), 1,
					APPLE_CLKGEN_PSTATE_TIMEOUT_US);
	if (ret)
		dev_err(misc->dev, "%s pstate %u timed out: 0x%08x\n",
			step->dev == DEV_FABRIC ? "fabric" : "dcs", pstate, val);

	return ret;
}

static int apple_pmgr_sys_dev_run(struct apple_pmgr_misc *misc,
				  const struct apple_pmgr_pstate_step *steps,
				  unsigned int n)
{
	unsigned int i;
	int ret;

	for (i = 0; i < n; i++) {
		ret = apple_pmgr_sys_dev_step(misc, &steps[i]);
		if (ret)
			return ret;
	}

	return 0;
}

static bool apple_pmgr_misc_can_stage(struct apple_pmgr_misc *misc)
{
	return misc->hw && misc->devices[DEV_FABRIC].base &&
	       misc->devices[DEV_DCS].base;
}

static int apple_pmgr_misc_staged_resume(struct apple_pmgr_misc *misc)
{
	return apple_pmgr_sys_dev_run(misc, misc->hw->resume,
				      misc->hw->n_resume);
}

static int apple_pmgr_misc_staged_suspend(struct apple_pmgr_misc *misc)
{
	int ret;

	ret = apple_pmgr_sys_dev_run(misc, misc->hw->suspend,
				     misc->hw->n_suspend);
	if (ret) {
		/* The resume callback is not called after a failed suspend */
		misc->staged = false;
		apple_pmgr_misc_staged_resume(misc);
	}

	return ret;
}

static int __maybe_unused apple_pmgr_misc_suspend_noirq(struct device *dev)
{
	struct apple_pmgr_misc *misc = dev_get_drvdata(dev);
	int i;

	misc->staged = staged_pstate && apple_pmgr_misc_can_stage(misc);
	if (misc->staged)
		return apple_pmgr_misc_staged_suspend(misc);

	for (i = 0; i < DEV_MAX; i++)
		apple_pmgr_sys_dev_set_pstate(misc, i, false);

	return 0;
}

static int __maybe_unused apple_pmgr_misc_resume_noirq(struct device *dev)
{
	struct apple_pmgr_misc *misc = dev_get_drvdata(dev);
	int i;

	if (misc->staged) {
		misc->staged = false;
		apple_pmgr_misc_staged_resume(misc);
		return 0;
	}

	for (i = 0; i < DEV_MAX; i++)
		apple_pmgr_sys_dev_set_pstate(misc, i, true);

	return 0;
}

static bool apple_pmgr_init_device(struct apple_pmgr_misc *misc,
				   enum sys_device dev, const char *device_name)
{
	void __iomem *base;
	char name[32];
	u32 val;

	snprintf(name, sizeof(name), "%s-ps", device_name);

	base = devm_platform_ioremap_resource_byname(
		to_platform_device(misc->dev), name);
	if (IS_ERR(base))
		return false;

	val = readl_relaxed(base + APPLE_CLKGEN_PSTATE);

	misc->devices[dev].base = base;
	misc->devices[dev].active_state =
		FIELD_GET(APPLE_CLKGEN_PSTATE_DESIRED, val);
	misc->devices[dev].suspend_state = SYS_DEV_PSTATE_SUSPEND;

	snprintf(name, sizeof(name), "apple,%s-min-ps", device_name);
	of_property_read_u32(misc->dev->of_node, name,
			     &misc->devices[dev].suspend_state);

	return true;
}

static int apple_pmgr_misc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_pmgr_misc *misc;
	int ret = -ENODEV;

	misc = devm_kzalloc(dev, sizeof(*misc), GFP_KERNEL);
	if (!misc)
		return -ENOMEM;

	misc->dev = dev;
	misc->hw = of_device_get_match_data(dev);

	if (apple_pmgr_init_device(misc, DEV_FABRIC, "fabric"))
		ret = 0;

	if (apple_pmgr_init_device(misc, DEV_DCS, "dcs"))
		ret = 0;

	platform_set_drvdata(pdev, misc);

	return ret;
}

/*
 * t602x: step fabric and DCS down together (FAB 5, DCS 7, FAB 3, DCS 5) and
 * bring them back up the same way (FAB 5, DCS 7) before returning to the
 * pstates found at probe.
 */
static const struct apple_pmgr_pstate_step t6020_suspend_steps[] = {
	{ DEV_FABRIC, 5 },
	{ DEV_DCS, 7 },
	{ DEV_FABRIC, 3 },
	{ DEV_DCS, 5 },
};

static const struct apple_pmgr_pstate_step t6020_resume_steps[] = {
	{ DEV_FABRIC, 5 },
	{ DEV_DCS, 7 },
	{ DEV_DCS, SYS_DEV_PSTATE_BOOT },
	{ DEV_FABRIC, SYS_DEV_PSTATE_BOOT },
};

static const struct apple_pmgr_misc_hw t6020_pmgr_misc_hw = {
	.suspend = t6020_suspend_steps,
	.n_suspend = ARRAY_SIZE(t6020_suspend_steps),
	.resume = t6020_resume_steps,
	.n_resume = ARRAY_SIZE(t6020_resume_steps),
};

static const struct of_device_id apple_pmgr_misc_of_match[] = {
	{ .compatible = "apple,t6020-pmgr-misc", .data = &t6020_pmgr_misc_hw },
	{ .compatible = "apple,t6000-pmgr-misc" },
	{}
};

MODULE_DEVICE_TABLE(of, apple_pmgr_misc_of_match);

static const struct dev_pm_ops apple_pmgr_misc_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(apple_pmgr_misc_suspend_noirq,
				      apple_pmgr_misc_resume_noirq)
};

static struct platform_driver apple_pmgr_misc_driver = {
	.probe = apple_pmgr_misc_probe,
	.driver = {
		.name = "apple-pmgr-misc",
		.of_match_table = apple_pmgr_misc_of_match,
		.pm = pm_ptr(&apple_pmgr_misc_pm_ops),
	},
};

MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
MODULE_DESCRIPTION("PMGR misc driver for Apple SoCs");
MODULE_LICENSE("GPL v2");

module_platform_driver(apple_pmgr_misc_driver);
