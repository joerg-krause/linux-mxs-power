/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <asm/cacheflush.h>
#include <asm/io.h>
#include <asm/irqflags.h>
#include <linux/compiler.h>
#include <linux/kernel.h>
#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/genalloc.h>

#include "pm.h"
#include "mxs-sleep.h"

#define MXS_SUSPEND_OCRAM_SIZE		(6 * 1024)

#define HW_CLKCTRL_CLKSEQ		0x000001d0
#define HW_CLKCTRL_XTAL			0x00000080

#define BM_POWER_CTRL_ENIRQ_PSWITCH	0x00020000
#define BM_POWER_CTRL_PSWITCH_IRQ	0x00100000
#define HW_POWER_CTRL			0x00000000
#define BM_POWER_RESET_PWD		0x00000001
#define BM_POWER_RESET_UNLOCK		0xFFFF0000
#define BF_POWER_RESET_UNLOCK(v)	(((v) << 16) & BM_POWER_RESET_UNLOCK)

#define HW_ICOLL_STAT			0x00000070

#define MXS_SET_ADDR			0x4
#define MXS_CLR_ADDR			0x8
#define MXS_TOG_ADDR			0xc

extern int mxs_icoll_suspend(void);
extern void mxs_icoll_resume(void);

struct mxs_virt_addr_t {
	void __iomem *clkctrl_addr;
	void __iomem *power_addr;
	void __iomem *dram_addr;
	void __iomem *pinctrl_addr;
	/* These are not used by mxs_cpu_standby */
	void __iomem *icoll_addr;
	void __iomem *rtc_addr;
} __attribute__((packed));

struct mxs_virt_addr_t mxs_virt_addr;

static unsigned long ocram_base;
static struct gen_pool *ocram_pool;

static inline void __mxs_setl(u32 mask, void __iomem *reg)
{
	__raw_writel(mask, reg + MXS_SET_ADDR);
}

static inline void __mxs_clrl(u32 mask, void __iomem *reg)
{
	__raw_writel(mask, reg + MXS_CLR_ADDR);
}

static void get_virt_addr(char *name, void __iomem **paddr)
{
	struct device_node *np;
	np = of_find_node_by_name(NULL, name);
	*paddr = of_iomap(np, 0);
	WARN_ON(!*paddr);
	of_node_put(np);
	pr_debug("get_virt_addr: address of %s is %p\n", name, *paddr);
}

static inline void do_standby(void)
{
	void (*mxs_cpu_standby_ptr)(int arg1, void *arg2);
	struct clk *cpu_clk;
	struct clk *osc_clk;
	struct clk *hbus_clk;
	struct clk *cpu_parent = NULL;
	int cpu_rate = 0;
	int hbus_rate = 0;
	u32 reg_clkctrl_clkseq, reg_clkctrl_xtal;
	int wakeupirq;
	int suspend_param = MXS_DO_SW_OSC_RTC_TO_BATT;

	//if (of_machine_is_compatible("digi,ccardimx28")) {
		/* Setting this switches the crystal oscillator and RTC to
		 * use the battery.  We don't want to do this on the CCARDIMX28
		 * since it doesn't have a battery. */
		suspend_param = MXS_DONOT_SW_OSC_RTC_TO_BATT;
	//}

	/*
	 * 1) switch clock domains from PLL to 24MHz
	 * 2) lower voltage (TODO)
	 * 3) switch EMI to 24MHz and turn PLL off (done in sleep.S)
	 */

	/* make sure SRAM copy gets physically written into SDRAM.
	 * SDRAM will be placed into self-refresh during power down
	 */
	flush_cache_all();

	/* copy suspend function into SRAM */
	memcpy((void *)ocram_base, mxs_cpu_standby,
	       mxs_standby_alloc_sz);

	/* now switch the CPU to cpu_xtal */
	cpu_clk = clk_get_sys("cpu", NULL);
	osc_clk = clk_get_sys("cpu_xtal", NULL);
	hbus_clk = clk_get_sys("hbus", NULL);

	if (IS_ERR(cpu_clk)) {
		pr_err("i.MX28 pm: failed to get cpu_clk with %ld\n",
			PTR_ERR(cpu_clk));
		goto cpu_clk_err;
	}
	if (IS_ERR(osc_clk)) {
		pr_err("i.MX28 pm: failed to get osc_clk with %ld\n",
			PTR_ERR(osc_clk));
		goto cpu_clk_err;
	}

	cpu_rate = clk_get_rate(cpu_clk);
	cpu_parent = clk_get_parent(cpu_clk);
	hbus_rate = clk_get_rate(hbus_clk);
	if (clk_set_parent(cpu_clk, osc_clk) < 0) {
		pr_err("Failed to switch cpu clocks.");
		goto cpu_clk_err;
	}

	if (cpu_rate == 261818000)
		clk_set_rate(hbus_clk, 8727267);

	local_fiq_disable();

	__mxs_setl(BM_POWER_CTRL_ENIRQ_PSWITCH,
		   mxs_virt_addr.power_addr + HW_POWER_CTRL);

	reg_clkctrl_clkseq = __raw_readl(mxs_virt_addr.clkctrl_addr +
					 HW_CLKCTRL_CLKSEQ);

	reg_clkctrl_xtal = __raw_readl(mxs_virt_addr.clkctrl_addr +
				       HW_CLKCTRL_XTAL);

	/* do suspend */
	mxs_cpu_standby_ptr = (void *)ocram_base;

	mxs_cpu_standby_ptr(suspend_param, &mxs_virt_addr);

	wakeupirq = __raw_readl(mxs_virt_addr.icoll_addr + HW_ICOLL_STAT);

	pr_info("wakeup irq = %d\n", wakeupirq);

	__raw_writel(reg_clkctrl_clkseq, mxs_virt_addr.clkctrl_addr +
		     HW_CLKCTRL_CLKSEQ);
	__raw_writel(reg_clkctrl_xtal, mxs_virt_addr.clkctrl_addr +
		     HW_CLKCTRL_XTAL);
	__mxs_clrl(BM_POWER_CTRL_PSWITCH_IRQ,
		   mxs_virt_addr.power_addr + HW_POWER_CTRL);
	__mxs_setl(BM_POWER_CTRL_ENIRQ_PSWITCH,
		   mxs_virt_addr.power_addr + HW_POWER_CTRL);

	local_fiq_enable();

	if (cpu_parent) {
		if (clk_set_parent(cpu_clk, cpu_parent) < 0)
			pr_err("Failed to switch cpu clock back.");
		clk_set_rate(cpu_clk, cpu_rate);
		clk_set_rate(hbus_clk, hbus_rate);
	}

cpu_clk_err:
	clk_put(hbus_clk);
	clk_put(osc_clk);
	clk_put(cpu_clk);
}

static int mxs_suspend_enter(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_MEM:
	case PM_SUSPEND_STANDBY:
		if (ocram_pool) {
			if (mxs_icoll_suspend() < 0)
				pr_err("No wake-up sources enabled. Suspend aborted.\n");
			else
				do_standby();
			mxs_icoll_resume();
		} else {
			cpu_do_idle();
		}
		break;

	default:
		return -EINVAL;
	}
	return 0;
}
static int mxs_pm_valid(suspend_state_t state)
{
	return ((state == PM_SUSPEND_STANDBY) || (state == PM_SUSPEND_MEM));
}

static struct platform_suspend_ops mxs_suspend_ops = {
	.enter = mxs_suspend_enter,
	.valid = mxs_pm_valid,
};

static void mxs_pm_power_off(void)
{
	/* Power down */
	__mxs_setl(BF_POWER_RESET_UNLOCK(0x3e77) | BM_POWER_RESET_PWD,
		   mxs_virt_addr.power_addr + HW_POWER_RESET);
}

static int __init mxs_suspend_alloc_ocram(void)
{
	struct device_node *node;
	struct platform_device *pdev;
	phys_addr_t phys;
	int ret = 0;

	node = of_find_compatible_node(NULL, NULL, "mmio-sram");
	if (!node) {
		pr_warn("%s: failed to find ocram node!\n", __func__);
		return -ENODEV;
	}

	pdev = of_find_device_by_node(node);
	if (!pdev) {
		pr_warn("%s: failed to find ocram device!\n", __func__);
		ret = -ENODEV;
		goto put_node;
	}

	ocram_pool = gen_pool_get(&pdev->dev, NULL);
	if (!ocram_pool) {
		pr_warn("%s: ocram pool unavailable!\n", __func__);
		ret = -ENODEV;
		goto put_node;
	}

	ocram_base = gen_pool_alloc(ocram_pool, MXS_SUSPEND_OCRAM_SIZE);
	if (!ocram_base) {
		pr_warn("%s: unable to alloc ocram!\n", __func__);
		ret = -ENOMEM;
		goto put_node;
	}

	phys = gen_pool_virt_to_phys(ocram_pool, ocram_base);

	get_virt_addr("clkctrl", &mxs_virt_addr.clkctrl_addr);
	get_virt_addr("power", &mxs_virt_addr.power_addr);
	get_virt_addr("rtc", &mxs_virt_addr.rtc_addr);
	get_virt_addr("emi", &mxs_virt_addr.dram_addr);
	get_virt_addr("interrupt-controller", &mxs_virt_addr.icoll_addr);
	get_virt_addr("pinctrl", &mxs_virt_addr.pinctrl_addr);

put_node:
	of_node_put(node);

	return ret;
}

static int __init mxs_suspend_init(void)
{
	int ret;

	ret = mxs_suspend_alloc_ocram();
	if (ret)
		return ret;

	pm_power_off = mxs_pm_power_off;

	suspend_set_ops(&mxs_suspend_ops);

	return 0;
}

void __init mxs_pm_init(void)
{
	int ret;

	if (IS_ENABLED(CONFIG_SUSPEND)) {
		ret = mxs_suspend_init();
		if (ret)
			pr_warn("%s: No DDR LPM support with suspend %d!\n",
				__func__, ret);
	}

	platform_device_register_simple("cpufreq-dt", -1, NULL, 0);
}
