/*++
	drivers/regulator/wmt-gpio-regulator.c

	Copyright (c) 2012 WonderMedia Technologies, Inc.

	This program is free software: you can redistribute it and/or modify it under the
	terms of the GNU General Public License as published by the Free Software Foundation,
	either version 2 of the License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful, but WITHOUT
	ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
	PARTICULAR PURPOSE.  See the GNU General Public License for more details.
	You should have received a copy of the GNU General Public License along with
	this program.  If not, see <http://www.gnu.org/licenses/>.

	WonderMedia Technologies, Inc.
	10F, 529, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C.
--*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/interrupt.h>
#include <linux/regulator/machine.h>
#include <mach/hardware.h>
#include <linux/slab.h>
#include <linux/earlysuspend.h>

struct wmt_gpio_pmic_info {
	/* resource ID, for resource control state machine */
	u8			id;

	/* regulator specific turn-on delay */
	u16			delay;

	/* State REMAP default configuration */
	u8			remap;

	/* chip constraints on regulator behavior */
	u32			min_uV;
	u32			max_uV;
	u32			cur_uvoltage;

	u8			enable;
	u8			voltage_idx;
	/* used by regulator core */
	struct regulator_desc	desc;
};
DECLARE_WAIT_QUEUE_HEAD(wmt_gpio_pmic_wait);
static unsigned int ostimer_int_pending = 0;
static int suspend_stage = 0;
static int early_suspend_stage = 0;
#define WMT_VOLTAGE_IDX0 1150000
#define WMT_VOLTAGE_IDX1 1250000
#define WMT_VOLTAGE_IDX2 1450000

struct wmt_pmicgpio_set_s {
	volatile int  bitmap;
	volatile int  ctraddr;
	volatile int  ocaddr;
	volatile int  odaddr;
	volatile int  opcaddr;
	volatile int  opdaddr;
};

struct wmt_gpo_params{
	unsigned int name_id;
	unsigned int active;
	unsigned int bitmap;
	volatile int ctraddr;
	volatile int ocaddr;
	volatile int odaddr;
	volatile int opcaddr;
	volatile int opdaddr;
};

struct wmt_voltage_map{
	int voltage;
};

struct wmt_pmic_param_s {
	unsigned char en_bit;
	unsigned char *dev_id;
	unsigned int pin_num;
	unsigned int map_num;
	unsigned int up_time;
	unsigned int down_time;
	struct wmt_gpo_params *pmic_gpio;
	struct wmt_voltage_map *voltage_map;
};

static struct wmt_pmic_param_s g_pmic_param;
#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend wmt_gpio_pmic_early_suspend;
#endif

#ifdef CONFIG_MTD_WMT_SF
extern int wmt_getsyspara(char *varname, unsigned char *varval, int *varlen);
#endif
static void init_gpio(void)
{
	int i = 0;
	/*we need to check gpio status to avoid gpio be reset while system was not power down*/
	for (i = 0; i < g_pmic_param.pin_num; ++i) {
		if (*(volatile unsigned int *)((g_pmic_param.pmic_gpio)[i].odaddr) & (g_pmic_param.pmic_gpio)[i].bitmap)
			return;
	}
	i = 0;
	for (i = 0; i < g_pmic_param.pin_num; ++i) {
		*(volatile unsigned int *)((g_pmic_param.pmic_gpio)[i].odaddr) &= ~((g_pmic_param.pmic_gpio)[i].bitmap);
		*(volatile unsigned int *)((g_pmic_param.pmic_gpio)[i].ocaddr) |= ((g_pmic_param.pmic_gpio)[i].bitmap);
		*(volatile unsigned int *)((g_pmic_param.pmic_gpio)[i].ctraddr) |= ((g_pmic_param.pmic_gpio)[i].bitmap);
	}
}

/* use ostimer 3
 * delay_time:us
 */
static int set_ostimer_irq(unsigned int delay_time)
{
	unsigned int val, sw_count;
	unsigned int trigger_time;
	unsigned int remain_time;
	int ret = 0;
	val = REG32_VAL(OSTC_ADDR);
	sw_count = 300000;
	if ((val&0x02) == 0) {
		val |= 0x02;
		REG32_VAL(OSTC_ADDR) = val;
	}
	while(1) {
		val = REG32_VAL(OSTA_ADDR);
		if ((val&0x20) != 0x20)
			break;
		if (--sw_count == 0) {
			ret = -1;
			printk(KERN_ERR "Read OST Count Request Failed\n");
			break;
		}
	}
	val = REG32_VAL(OSCR_ADDR);
	trigger_time = val;
	sw_count = 300000;
	while(1) {
		val = REG32_VAL(OSTA_ADDR);
		if ((val&0x08) != 0x08)
			break;
		if (--sw_count == 0) {
			ret = -1;
			printk(KERN_ERR "OST Match3 Request Failed\n");
			break;
		}
	}

	/*overrun issue*/
	if ((0xFFFFFFFF - trigger_time < delay_time * 3)) {
		remain_time = 0xFFFFFFFF - trigger_time;
		remain_time = delay_time * 3 - remain_time;
		REG32_VAL(OSM3_ADDR) = remain_time;
	} else
		REG32_VAL(OSM3_ADDR) = trigger_time + delay_time * 3;
	REG32_VAL(OSTS_ADDR) = 0x08;
	REG32_VAL(OSTI_ADDR) |= 0x08;
	return ret;
}

static void clear_ostimer_irq(void)
{
	REG32_VAL(OSTS_ADDR) = 0x08;
	REG32_VAL(OSTI_ADDR) &= ~0x08;
}


/*----------------------------------------------------------------------*/

#if 0
static int wmt_gpio_pmic_list_voltage(struct regulator_dev *rdev, unsigned uV)
{
	int i = 0;
	int selector;
	for (i = 0; i < g_pmic_param.map_num; ++i) {
		if (uV < (g_pmic_param.voltage_map)[i].voltage) {
			selector = (g_pmic_param.voltage_map)[i].voltage;
			break;
		}
	}

	return selector;
}
#endif

static void wmt_set_voltage(int idx, int is_up)
{
	int i = 0;

	for (i = 0; i < g_pmic_param.pin_num; ++i) {
		if (!((*(volatile unsigned int *)(g_pmic_param.pmic_gpio)[i].odaddr) & (g_pmic_param.pmic_gpio)[i].bitmap))
			*(volatile unsigned int *)(g_pmic_param.pmic_gpio)[i].odaddr |= (g_pmic_param.pmic_gpio)[i].bitmap;

	}
	for (i = 0; i < g_pmic_param.pin_num; ++i) {
		if (!(idx & (1 << i)))
			*(volatile unsigned int *)(g_pmic_param.pmic_gpio)[i].odaddr &= ~(g_pmic_param.pmic_gpio)[i].bitmap;
	}

	if (early_suspend_stage == 0) {
		if (is_up == 1) {
			if (g_pmic_param.up_time > 0) {
				set_ostimer_irq(g_pmic_param.up_time);
				wait_event(wmt_gpio_pmic_wait, ostimer_int_pending);
			}
		} else {
			if (g_pmic_param.down_time > 0) {
				set_ostimer_irq(g_pmic_param.down_time);
				wait_event(wmt_gpio_pmic_wait, ostimer_int_pending);
			}
		}
	} else {
		if (is_up == 1) {
			if (g_pmic_param.up_time > 0) {
				if (g_pmic_param.up_time < 1000)
					mdelay(1);
				else
					mdelay(g_pmic_param.up_time/1000 + 1);
			}
		} else {
			if (g_pmic_param.down_time > 0) {
				if (g_pmic_param.down_time < 1000)
					mdelay(1);
				else
					mdelay(g_pmic_param.down_time/1000 + 1);
			}
		}
	}
	ostimer_int_pending  = 0;
}
static int
wmt_gpio_pmic_set_voltage(struct regulator_dev *rdev, int min_uV, int max_uV,
		       unsigned int *selector)
{
	struct wmt_gpio_pmic_info *info = rdev_get_drvdata(rdev);
	int selvol;
	int is_up = 0;
	int i = 0;

	if ((min_uV < info->min_uV) || (max_uV > info->max_uV))
		return -EDOM;

	selvol = min_uV;
	selector = &selvol;
	if (suspend_stage == 1)
		return -EDOM;
	if (selvol > info->cur_uvoltage)
		is_up = 1;
	info->cur_uvoltage = selvol;
	while (i < g_pmic_param.map_num) {
		if ((g_pmic_param.voltage_map)[i].voltage == selvol)
			break;
		++i;
	}
	wmt_set_voltage(i, is_up);
	info->voltage_idx = i;
	return *selector;

}

static int wmt_gpio_pmic_get_voltage(struct regulator_dev *rdev)
{
	struct wmt_gpio_pmic_info *info = rdev_get_drvdata(rdev);
	
	return (g_pmic_param.voltage_map)[info->voltage_idx].voltage;
}

static int wmt_gpio_pmic_enable(struct regulator_dev *rdev)

{
        return 0;
}

static int wmt_gpio_pmic_disable(struct regulator_dev *rdev)
{
	return 0;
}
static int wmt_gpio_pmic_is_enabled(struct regulator_dev *rdev)
{
	struct wmt_gpio_pmic_info *info = rdev_get_drvdata(rdev);
	if (info->enable == 1)
		return 0;
	else
		return -EDOM;
	return 0;
}
static int wmt_gpio_pmic_set_mode(struct regulator_dev *rdev, unsigned mode)
{
	return 0;
}
static int wmt_gpio_pmic_get_status(struct regulator_dev *rdev)
{
	return 0;
}
static struct regulator_ops wmt_gpio_pmicdo_ops = {
	.list_voltage	= NULL,

	.set_voltage	= wmt_gpio_pmic_set_voltage,
	.get_voltage	= wmt_gpio_pmic_get_voltage,

	.enable		= wmt_gpio_pmic_enable,
	.disable	= wmt_gpio_pmic_disable,
	.is_enabled	= wmt_gpio_pmic_is_enabled,

	.set_mode	= wmt_gpio_pmic_set_mode,

	.get_status	= wmt_gpio_pmic_get_status,
};


#define WMT_GPIO_PMIC_LABEL 0
/*----------------------------------------------------------------------*/

#define WMT_GPIO_PMIC_MODE0(label, min_uVolts, max_uVolts, num) { \
	.min_uV = min_uVolts, \
	.max_uV = max_uVolts, \
	.voltage_idx = 0, \
	.cur_uvoltage = 1150000, \
	.enable = 1, \
	.desc = { \
		.name = #label, \
		.id = WMT_GPIO_PMIC_LABEL, \
		.n_voltages = (max_uVolts - min_uVolts), \
		.ops = &wmt_gpio_pmicdo_ops, \
		.type = REGULATOR_VOLTAGE, \
		.owner = THIS_MODULE, \
		}, \
	}

static struct wmt_gpio_pmic_info wmt_pmic_regs[] = {
	WMT_GPIO_PMIC_MODE0(WMT_GPIO_PMIC_LABEL, 1150000, 1450000, 1),
};

static struct regulator_consumer_supply wmt_corepower__supply =
	REGULATOR_SUPPLY("wmt_corepower", NULL);
	/*
	REGULATOR_SUPPLY("wmt_corepower", "wmt_cpufreq");
	*/
static struct regulator_init_data wmt_corepower = {
	.constraints = {
		.min_uV                 = 1150000,
		.max_uV                 = 1450000,
		.apply_uV               = true,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &wmt_corepower__supply,
};

static struct platform_device wmt_gpio_pmic_device = {
	.name	= "wmt_gpio_pmic",
	.id	= 0,
	.num_resources  = 0,
	.resource       = NULL,
};
static irqreturn_t wmt_ostimer_isr(int irq, void *dev_id)
{
        clear_ostimer_irq();
	ostimer_int_pending = 1;
	wake_up(&wmt_gpio_pmic_wait);
	return IRQ_HANDLED;
}
#ifdef CONFIG_HAS_EARLYSUSPEND
static void wmt_gpio_early_suspend(struct early_suspend *h)
{
	early_suspend_stage = 1;
}
static void wmt_gpio_late_resume(struct early_suspend *h)
{
	early_suspend_stage = 0;
}
#endif
static int __devinit wmt_gpio_pmic_probe(struct platform_device *pdev)
{
	int				i;
	struct wmt_gpio_pmic_info		*info;
	struct regulator_init_data	*initdata;
	struct regulation_constraints	*c;
	struct regulator_dev		*rdev;

	init_gpio();
	for (i = 0, info = NULL; i < ARRAY_SIZE(wmt_pmic_regs); i++) {
		if (wmt_pmic_regs[i].desc.id != pdev->id)
			continue;
		info = wmt_pmic_regs + i;
		break;
	}
	if (!info)
		return -ENODEV;

	initdata = &wmt_corepower;

	c = &initdata->constraints;
	c->valid_modes_mask &= REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY;
	c->valid_ops_mask &= REGULATOR_CHANGE_VOLTAGE
				| REGULATOR_CHANGE_MODE
				| REGULATOR_CHANGE_STATUS;
	c->always_on = true;

	rdev = regulator_register(&info->desc, &pdev->dev, initdata, info);
	if (IS_ERR(rdev)) {
		dev_err(&pdev->dev, "can't register %s, %ld\n",
				info->desc.name, PTR_ERR(rdev));
		return PTR_ERR(rdev);
	}
	request_irq(IRQ_OST3, wmt_ostimer_isr,
			IRQF_DISABLED,
			"pmic", NULL); 
#ifdef CONFIG_HAS_EARLYSUSPEND
	wmt_gpio_pmic_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	wmt_gpio_pmic_early_suspend.suspend = wmt_gpio_early_suspend;
	wmt_gpio_pmic_early_suspend.resume = wmt_gpio_late_resume;
	register_early_suspend(&wmt_gpio_pmic_early_suspend);
#endif

	platform_set_drvdata(pdev, rdev);

	return 0;
}

static int __devexit wmt_gpio_pmic_remove(struct platform_device *pdev)
{
	regulator_unregister(platform_get_drvdata(pdev));
	return 0;
}

MODULE_ALIAS("platform:wmt_gpio_pmic");

static int wmt_gpio_pmic_suspend(struct platform_device *pdev,
                                   pm_message_t mesg)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);
	struct wmt_gpio_pmic_info *info = rdev_get_drvdata(rdev);
	info->enable = 0;
	suspend_stage = 1;
	return 0;
}

static int wmt_gpio_pmic_resume(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);
	struct wmt_gpio_pmic_info *info = rdev_get_drvdata(rdev);
	init_gpio();
	mdelay(1);
	info->enable = 1;
	suspend_stage = 0;
	return 0;
}

static struct platform_driver wmt_gpio_pmic_driver = {
	.probe		= wmt_gpio_pmic_probe,
	.remove		= __devexit_p(wmt_gpio_pmic_remove),
	.suspend        = wmt_gpio_pmic_suspend,
	.resume         = wmt_gpio_pmic_resume,
	.driver.name	= "wmt_gpio_pmic",
	.driver.owner	= THIS_MODULE,
};

#ifdef CONFIG_MTD_WMT_SF
#define par_len 240
static int parse_pmic_param(void)
{
	int retval;
	unsigned char buf[par_len];
	unsigned char tmp_buf[par_len];
	int varlen = par_len;
	int i = 0;
	int j = 0;
	int k = 0;
	int t = 0;
	int vol = 0;
	int idx = 0;
	char *varname = "wmt.pmic.param";
	retval = wmt_getsyspara(varname, buf, &varlen);
	if (retval != 0)
		return -1;
	if (buf[0] == 0) /*disable*/
		return -1;
	i += 2;
	j = 0;
	for (; i < par_len; ++i) {
		if (buf[i] != ':')
			tmp_buf[j] = buf[i];
		else
			break;
		++j;
	}
	if (strncmp(tmp_buf, "gpio", 4))
		return -1;
	++i;
	/*parse pin_num, map_num, up_time and down_time*/
	j = sscanf((buf+i), "%d,%d,%d,%d,",
			&g_pmic_param.pin_num,
			&g_pmic_param.map_num,
			&g_pmic_param.up_time,
			&g_pmic_param.down_time);
	k = 0;
	for (j = i; j < par_len; ++j) {
		if (buf[j] == ',')
			++k;
		if (k > 3) {
			++j;
			break;
		}	
	}
	i = j;
	g_pmic_param.pmic_gpio = kzalloc(g_pmic_param.pin_num * sizeof(struct wmt_pmic_param_s), GFP_KERNEL);
	g_pmic_param.voltage_map = kzalloc(g_pmic_param.map_num * sizeof(struct wmt_voltage_map), GFP_KERNEL);
	for (j = 0; j < g_pmic_param.pin_num; ++j) {
		k = sscanf((buf +i), "[%x:%x:%x:%x:%x:%x:%x:%x]",
			&(g_pmic_param.pmic_gpio + j)->name_id,
			&(g_pmic_param.pmic_gpio + j)->active,
			&(g_pmic_param.pmic_gpio + j)->bitmap,
			&(g_pmic_param.pmic_gpio + j)->ctraddr,
			&(g_pmic_param.pmic_gpio + j)->ocaddr,
			&(g_pmic_param.pmic_gpio + j)->odaddr,
			&(g_pmic_param.pmic_gpio + j)->opcaddr,
			&(g_pmic_param.pmic_gpio + j)->opdaddr);
		g_pmic_param.pmic_gpio[j].ctraddr += WMT_MMAP_OFFSET;
		g_pmic_param.pmic_gpio[j].ocaddr += WMT_MMAP_OFFSET;
		g_pmic_param.pmic_gpio[j].odaddr += WMT_MMAP_OFFSET;
		g_pmic_param.pmic_gpio[j].opcaddr += WMT_MMAP_OFFSET;
		g_pmic_param.pmic_gpio[j].opdaddr += WMT_MMAP_OFFSET;
		for (k = i; k < par_len; ++k) {
			if (buf[k] == ']') {
				++k;
				break;
			}
		}
		i = k;
	}
	++i;/*','*/
	for (j = 0; j < g_pmic_param.map_num; ++j) {
		k = sscanf((buf + i), "[%d,%d]",
			&idx, &vol);
		t = 0;
		for (k = i; k < par_len; ++k) {
			if (buf[k] == ']') {
				++k;
				break;
			}
		}
		i = k;
		(g_pmic_param.voltage_map + idx)->voltage = vol * 1000;
	}
	return 0;
}
#endif
static int __init wmt_gpio_pmic_init(void)
{
	int ret = 0;
#ifdef CONFIG_MTD_WMT_SF
	ret = parse_pmic_param();
#else
	return -ENODEV;
#endif
	if (ret != 0)
		return -ENODEV;
	ret = platform_device_register(&wmt_gpio_pmic_device);
	if (ret)
		printk("wmt-ebm:register device failed\n");
	return platform_driver_register(&wmt_gpio_pmic_driver);
}
subsys_initcall(wmt_gpio_pmic_init);

static void __exit wmt_gpio_pmic_exit(void)
{
	platform_driver_unregister(&wmt_gpio_pmic_driver);
}
module_exit(wmt_gpio_pmic_exit)

MODULE_AUTHOR("WonderMedia Technologies, Inc.");
MODULE_DESCRIPTION("WMT GPIO PMIC Driver");
MODULE_LICENSE("GPL");
