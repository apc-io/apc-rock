/**************************************************************
Copyright (c) 2008 WonderMedia Technologies, Inc.

Module Name:
    $Workfile: wmt-lcd-backlight.c $
Abstract:
    This program is the WMT LCD backlight driver for Android 1.6 system.
    Andriod1.6 API adjusts the LCD backlight by writing follwing file:
        /sys/class/leds/lcd-backlight/brightness
    Use WMT PWM to control the LCD backlight
Revision History:
    Jan.08.2010 First Created by HowayHuo
	
**************************************************************/
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/io.h>
#include <linux/slab.h>
#include "../char/wmt-pwm.h"

#define BACKLIGHT_ON 0
#define BACKLIGHT_CLOSE	1

struct wmt_pwm_reg_t {
	unsigned int id;
	unsigned int scalar;
	unsigned int period;
	unsigned int duty;
	unsigned int config;
	unsigned int status;
};

static struct wmt_pwm_reg_t g_pwm_setting;

#define ENV_LCD_BACKLIGHT "wmt.display.pwm"

extern int wmt_getsyspara(char *varname, unsigned char *varval, int *varlen);

struct back_light_limit {
	int max;
	int min;
	int diff;
	int step;
};

struct back_light_limit g_backlight_limit;

void backlight_get_env(void)
{
	unsigned char buf[100];
	int varlen = 100;
	int max = -1, min = -1;
	int val = -1;

	memset((char *)&g_backlight_limit, 0, sizeof(struct back_light_limit));
	if( wmt_getsyspara(ENV_LCD_BACKLIGHT,buf,&varlen) == 0) {
		//printk("\r\n backlight_get_env : %s = %s \n",ENV_LCD_BACKLIGHT,buf);
		sscanf(buf,"%x",&val);
		g_pwm_setting.id = val&0x0F;
		if (g_pwm_setting.id > 1) {
			printk("PWM%d is invaild , using default PWM0\n", g_pwm_setting.id);
			g_pwm_setting.id = 0;
		}		
		min = (val >> 8)&0xff;
		max = (val >> 16)&0xff;
		if ((min >= max) || (max > 100) || (min < 0)) {
			printk("backlight_get_env error : max = %d , min = %d ", max, min);
			goto out;
		}
		g_backlight_limit.max = max;
		g_backlight_limit.min = min;
	} else {
		printk("## Warning: %s not defined\n",ENV_LCD_BACKLIGHT);
		goto out;
	}
	return;
out:
	g_pwm_setting.id = 0;
	g_backlight_limit.max = 90;
	g_backlight_limit.min = 10;
}


/*
 * For simplicity, we use "brightness" as if it were a linear function
 * of PWM duty cycle.  However, a logarithmic function of duty cycle is
 * probably a better match for perceived brightness: two is half as bright
 * as four, four is half as bright as eight, etc
 */
static void lcd_brightness(struct led_classdev *cdev, enum led_brightness b)
{
    unsigned int duty,  pwm_enable;
    int val = 0;

	/*printk(KERN_ALERT "[%s]pwm_no = %d pwm_period = %d b= %d\n", __func__, pwm_no,pwm_period, b);*/
	if (!b) {
		g_pwm_setting.status = BACKLIGHT_CLOSE;
		pwm_set_gpio(g_pwm_setting.id, 0);
		return;
	}

	if (g_pwm_setting.status == BACKLIGHT_CLOSE) {
		pwm_set_gpio(g_pwm_setting.id, 1);
		g_pwm_setting.status = BACKLIGHT_ON;
	}

	val = (b*g_backlight_limit.diff)/255;
	duty = (val * g_backlight_limit.step) + g_backlight_limit.min;
    if(duty) {
		if(--duty > 0xFFF)
	    	duty = 0xFFF;
    }

	//printk("max = %d , min = %d , b= %d , duty = %d\n", g_backlight_limit.max, g_backlight_limit.min, b,duty);

    pwm_set_duty(g_pwm_setting.id, duty);
	pwm_enable = pwm_get_enable(g_pwm_setting.id);
}

/*
 * NOTE:  we reuse the platform_data structure of GPIO leds,
 * but repurpose its "gpio" number as a PWM channel number.
 */
static int lcd_backlight_probe(struct platform_device *pdev)
{
	const struct gpio_led_platform_data	*pdata;
	struct led_classdev	*leds;
	int	i;
	int	status;
	unsigned int brightness;
	unsigned int pwm_period;

	pdata = pdev->dev.platform_data;
	if (!pdata || pdata->num_leds < 1)
		return -ENODEV;

	leds = kcalloc(pdata->num_leds, sizeof(*leds), GFP_KERNEL);
	if (!leds)
		return -ENOMEM;

	auto_pll_divisor(DEV_PWM,CLK_ENABLE,0,0);

	pwm_period  = pwm_get_period(g_pwm_setting.id) + 1;
	backlight_get_env();
	g_backlight_limit.diff = (g_backlight_limit.max - g_backlight_limit.min);
	g_backlight_limit.max = (pwm_period*g_backlight_limit.max)/100;
	g_backlight_limit.min = (pwm_period*g_backlight_limit.min)/100;
	g_backlight_limit.step = (g_backlight_limit.max - g_backlight_limit.min) / g_backlight_limit.diff;

	/*calculate the default brightness*/
	brightness = (pwm_get_duty(g_pwm_setting.id) * 255) / pwm_period;

	for (i = 0; i < pdata->num_leds; i++) {
		struct led_classdev		*led = leds + i;
		const struct gpio_led	*dat = pdata->leds + i;

		led->name = dat->name;
		led->brightness = brightness;
		led->brightness_set = lcd_brightness;
		led->default_trigger = dat->default_trigger;

		/* Hand it over to the LED framework */
		status = led_classdev_register(&pdev->dev, led);
		if (status < 0) {
			goto err;
		}
	}

	platform_set_drvdata(pdev, leds);
	g_pwm_setting.status = BACKLIGHT_ON;
	return 0;

err:
	if (i > 0) {
		for (i = i - 1; i >= 0; i--) {
			led_classdev_unregister(leds + i);
		}
	}
	kfree(leds);

	return status;
}

static int lcd_backlight_remove(struct platform_device *pdev)
{
	const struct gpio_led_platform_data	*pdata;
	struct led_classdev				*leds;
	unsigned				i;

	pdata = pdev->dev.platform_data;
	leds = platform_get_drvdata(pdev);

	for (i = 0; i < pdata->num_leds; i++) {
		struct led_classdev		*led = leds + i;

		led_classdev_unregister(led);
	}

	kfree(leds);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static int lcd_backlight_suspend
(
	struct platform_device *pdev,     /*!<; // a pointer point to struct device */
	pm_message_t state		/*!<; // suspend state */
)
{
	unsigned int addr;

	addr = PWM_CTRL_REG_ADDR + (0x10 * g_pwm_setting.id);
	g_pwm_setting.config = (REG32_VAL(addr) & 0xFFF);
	pwm_set_control(g_pwm_setting.id, 0);
	addr = PWM_PERIOD_REG_ADDR + (0x10 * g_pwm_setting.id);
	g_pwm_setting.period = (REG32_VAL(addr) & 0xFFF);

	addr = PWM_SCALAR_REG_ADDR + (0x10 * g_pwm_setting.id);
	g_pwm_setting.scalar = (REG32_VAL(addr) & 0xFFF);
	//g_pwm_setting.duty = 0; // for android , AP will set duty to 0

	return 0;
}

static int lcd_backlight_resume
(
	struct platform_device *pdev 	/*!<; // a pointer point to struct device */
)
{
	pwm_set_scalar(g_pwm_setting.id, g_pwm_setting.scalar);
	pwm_set_period(g_pwm_setting.id, g_pwm_setting.period);

	pwm_set_duty(g_pwm_setting.id, 5);
	pwm_set_control(g_pwm_setting.id, g_pwm_setting.config);

	return 0;
}


static struct gpio_led lcd_pwm[] = {
	{
		.name			= "lcd-backlight",
	}, 
};


static struct gpio_led_platform_data lcd_backlight_data = {
	.leds		= lcd_pwm,
	.num_leds	= ARRAY_SIZE(lcd_pwm),
};

static struct platform_device lcd_backlight_device = {
	.name           = "lcd-backlight",
	.id             = 0,
	.dev            = 	{	.platform_data = &lcd_backlight_data,			
	},
};

static struct platform_driver lcd_backlight_driver = {
	.driver = {
		.name =		"lcd-backlight",
		.owner =	THIS_MODULE,
	},
	/* REVISIT add suspend() and resume() methods */
	.probe	=	lcd_backlight_probe,
	.remove =	__exit_p(lcd_backlight_remove),
	.suspend        = lcd_backlight_suspend,
	.resume         = lcd_backlight_resume
};

static int __init lcd_backlight_init(void)
{
	int ret;

	ret = platform_device_register(&lcd_backlight_device);
	if(ret) {
		printk("[lcd_backlight_init]Error: Can not register LCD backlight device\n");
		return ret;
	}
	//ret = platform_driver_probe(&lcd_backlight_driver, lcd_backlight_probe);
	ret = platform_driver_register(&lcd_backlight_driver);
	if(ret) {
		printk("[lcd_backlight_init]Error: Can not register LCD backlight driver\n");
		platform_device_unregister(&lcd_backlight_device);
		return ret;
	}
	return 0; 
}
module_init(lcd_backlight_init);

static void __exit lcd_backlight_exit(void)
{
	platform_driver_unregister(&lcd_backlight_driver);
	platform_device_unregister(&lcd_backlight_device);
}
module_exit(lcd_backlight_exit);

MODULE_DESCRIPTION("Driver for LCD with PWM-controlled brightness");
MODULE_LICENSE("GPL");

