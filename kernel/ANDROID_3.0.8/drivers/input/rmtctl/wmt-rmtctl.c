/*++ 
 * linux/drivers/input/rmtctl/wmt-rmtctl.c
 * WonderMedia input remote control driver
 *
 * Copyright c 2010  WonderMedia  Technologies, Inc.
 *
 * This program is free software: you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 2 of the License, or 
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * WonderMedia Technologies, Inc.
 * 4F, 533, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C
--*/

  
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/version.h>
#include <linux/input.h>
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,10))
#include <linux/platform_device.h>
#endif
#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/wmt_pmc.h>
#include <linux/delay.h>
#include "oem-dev.h"

#define DRIVER_DESC   "WMT rmtctl driver"

#define RC_INT 55
/* New remote control code */
#include "wmt-rmtctl.h"

#define RMTCTL_DEBUG 1
#define RMT_CFG_INT_CNT
#define RMT_CFG_REPEAT_KEY
//#define RMT_CFG_FACTORY_NEC
//#define RMT_CFG_WAKUP_BY_ANY_KEY

#define FACTORID   0
  
// Jiun 01/12/2013 for oem remote controller
extern int wmt_getsyspara(char *varname, unsigned char *varval, int *varlen);
unsigned int sus_key, u_debug=0;

static struct input_dev *idev;
int cirisrcnt=0;
unsigned long volatile __jiffy_data prev_jiffies;
int errflag=0;
int repeatcnt=0;
int dev_tot_num=0;
static irqreturn_t rmtctl_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	unsigned int status, ir_data, scancode, vendor;
	unsigned char *key;
	int i;

	/* Get IR status. */
	status = REG32_VAL(IRSTS);

	/* Check 'IR received data' flag. */
	if ((status & 0x1) == 0x0) {
		printk("IR IRQ was triggered without data received. (0x%x)\n",
			status);
		return IRQ_NONE;
	}

	/* Read IR data. */
	ir_data = REG32_VAL(IRDATA(0)) ;
	//printk("orig data 0x%08x \n",ir_data);

	key = (char *) &ir_data;

	/* clear INT status*/
	REG32_VAL(IRSTS)=0x1 ;

	if (RMTCTL_DEBUG){
		printk("ir_data = 0x%08x, status = 0x%x \n", ir_data, status);
		//printk("HZ %d jiffies 0x%08x \n ",HZ,jiffies);
	}
	/* Get vendor ID. */
	vendor = (key[0] << 8) | (key[1]);

	/* Check if key is valid. Key[3] is XORed t o key[2]. */
	if (key[2] & key[3]) { 
		printk("Invalid IR key received. (0x%x, 0x%x)\n", key[2], key[3]);
		return IRQ_NONE;
	}
	
	/* Keycode mapping. */
	scancode = key[2]; 	//default
    // Jiun 01/12/2013 for oem remote controller
    if (u_debug) printk("rmtctl vendor id: 0x%x  scancode:0x%x\n", vendor, scancode);
		
	printk(">> dev_tot_num %d\n",dev_tot_num);
	for (i=0; i < dev_tot_num; i++)
	{
		if (vendor == rmt_dev_tbl[i].vender_id){
			scancode = rmt_dev_tbl[i].key_codes[key[2]];
			printk(" vender_name %s   id 0x%04x  scan code %d \n", rmt_dev_tbl[i].vendor_name,vendor,i);
			break;
		}
	}
	
	if (RMTCTL_DEBUG)
		printk("scancode = 0x%08x \n", scancode);
	/* Check 'IR code repeat' flag. */
	if ((status & 0x2) || (scancode == KEY_RESERVED)) {
		/* Ignore repeated or reserved keys. */
	} else {
		printk(KERN_ERR"%d ---------IR report key 0x%x\n" ,cirisrcnt++,scancode);
		input_report_key(idev, scancode, 1);
		input_report_key(idev, scancode, 0);
		input_sync(idev);
	}

	return IRQ_HANDLED;
}

static void rmtctl_hw_suspend(void)
{

    // Jiun 01/12/2013 for oem remote controller
		//REG32_VAL(WAKEUP_CMD1(0))=0xf30cf708; //0xf10ebf40;
        REG32_VAL(WAKEUP_CMD1(0))=sus_key;
    
        REG32_VAL(WAKEUP_CMD1(1))=0x0;
      		REG32_VAL(WAKEUP_CMD1(2))=0x0;
      		REG32_VAL(WAKEUP_CMD1(3))=0x0;
      		REG32_VAL(WAKEUP_CMD1(4))=0x0;
      		
             REG32_VAL(WAKEUP_CMD2(0))=0xa857fd02;//0xff00ff00;
             REG32_VAL(WAKEUP_CMD2(1))=0x0;
      		REG32_VAL(WAKEUP_CMD2(2))=0x0;
      		REG32_VAL(WAKEUP_CMD2(3))=0x0;
      		REG32_VAL(WAKEUP_CMD2(4))=0x0;
      
      #ifdef RMT_CFG_WAKUP_BY_ANY_KEY
             REG32_VAL(WAKEUP_CTRL) = 0x001;
      #else
             REG32_VAL(WAKEUP_CTRL) = 0x101;
      #endif
     
}
static void rmtctl_hw_init(void)
{
	unsigned int st;

	/* Turn off CIR SW reset. */
	REG32_VAL(IRSWRST) = 1;
	REG32_VAL(IRSWRST) = 0;

	REG32_VAL(PARAMETER(0)) = 0x10a;
	REG32_VAL(PARAMETER(1)) = 0x8E;  		
	REG32_VAL(PARAMETER(2)) = 0x42;
	REG32_VAL(PARAMETER(3)) = 0x55;		
	REG32_VAL(PARAMETER(4)) = 0x9; 		
	REG32_VAL(PARAMETER(5)) = 0x13;
	REG32_VAL(PARAMETER(6)) = 0x13;

#ifdef RMT_CFG_REPEAT_KEY
	REG32_VAL(NEC_REPEAT_TIME_OUT_CTRL) = 0x1;
	REG32_VAL(NEC_REPEAT_TIME_OUT_COUNT) = 17965000;//(107.9ms * 1000000)/(1000/166.5)
	REG32_VAL(IRCTL)        =  0X100;//NEC repeat key
#else
       REG32_VAL(IRCTL)        =  0;//NEC repeat key
#endif

#ifdef RMT_CFG_FACTORY_NEC
	REG32_VAL(IRCTL) |= (0x20<<16) ; //BIT16-23->0x20,  BIT 24,25 -> 0
#else  //NEC
	REG32_VAL(IRCTL) |= (0x0<<16) |(0x1<<25);
#endif

#ifdef RMT_CFG_INT_CNT
	REG32_VAL(INT_MASK_CTRL) = 0x1;
	REG32_VAL(INT_MASK_COUNT) =50*1000000*1/3;//0x47868C0/4;//count for 1 sec 0x47868C0
#endif



	/* Set IR remoter vendor type */ 
	/* BIT[0]: IR Circuit Enable. */
	REG32_VAL(IRCTL) |= 0x1; /*  IR_EN */

	/* Read CIR status to clear IR interrupt. */
	st = REG32_VAL(IRSTS);

	
}

static int rmtctl_probe(struct platform_device *dev)
{
	int i,j;

	printk("rmtctl_probe\n");
	/* Keycode_1 32 - 63 are invalid. */
	
	/* Register an input device. */

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,10))
	if ((idev = input_allocate_device()) == NULL)
		return -1;
#else
	if ((idev = kmalloc(sizeof(struct input_dev), GFP_KERNEL)) == NULL)
		return -1;
	memset(idev, 0, sizeof(struct input_dev));
	init_input_dev(idev);
#endif

	set_bit(EV_KEY, idev->evbit);

	dev_tot_num = sizeof(rmt_dev_tbl) / sizeof(struct rmt_dev);

	for ( j = 0; j < dev_tot_num; j++)
	{
		for (i=0;i<128;i++)
		{
			if (rmt_dev_tbl[j].key_codes[i]) {
				set_bit(rmt_dev_tbl[j].key_codes[i], idev->keybit);
			}

		}
	}
	idev->name = "rmtctl";
	idev->phys = "rmtctl";

	if (!idev) {
		printk("Inavlid input_dev detected.\n");
		return -1;
	}
	input_register_device(idev);

	/* Register an ISR */
	i = request_irq(RC_INT, rmtctl_interrupt, IRQF_SHARED, "rmtctl", idev);

	/* Initial H/W */
	rmtctl_hw_init();

	if (RMTCTL_DEBUG)
		printk("WonderMedia rmtctl driver v0.98 initialized: ok\n");

	return 0;
}

static int rmtctl_remove(struct platform_device *dev)
{
	if (RMTCTL_DEBUG)
		printk("rmtctl_remove\n");

	free_irq(RC_INT, idev);
	input_unregister_device(idev);
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,10))
	input_free_device(idev);
#else
	kfree(idev);
#endif

	return 0;
}

#ifdef CONFIG_PM
static int rmtctl_suspend(struct platform_device *dev, u32 state, u32 level)
{
	/* Nothing to suspend? */
	rmtctl_hw_init();

	rmtctl_hw_suspend();
	printk("wmt cir suspend  \n");
	
	disable_irq(RC_INT);
	return 0;
}

static int rmtctl_resume(struct platform_device *dev, u32 level)
{
	volatile  unsigned int regval;
	int i =0 ;
	printk("wmt cir resume \n");

	/* Initial H/W */
	REG32_VAL(WAKEUP_CTRL) &=~ BIT0;

	for (i=0;i<10;i++)
	{
		regval = REG32_VAL(WAKEUP_STS) ;

		if (regval & BIT0){
			REG32_VAL(WAKEUP_STS) |= BIT4;

		}else{
			break;
		}
		msleep_interruptible(5);
	}
	
	regval = REG32_VAL(WAKEUP_STS) ;
	if (regval & BIT0)
		printk(" 1. CIR resume NG  WAKEUP_STS 0x%08x \n",regval);


	rmtctl_hw_init();
	enable_irq(RC_INT);
	return 0;
}
#else
#define rmtctl_suspend NULL
#define rmtctl_resume NULL
#endif

static struct platform_driver  rmtctl_driver = {
	/* 
	 * Platform bus will compare the driver name 
	 * with the platform device name. 
	 */
	.driver.name = "wmt-rmtctl", 
	//.bus = &platform_bus_type,
	//.probe = rmtctl_probe,
	.remove = rmtctl_remove,
	.suspend = rmtctl_suspend,
	.resume = rmtctl_resume
};

static void rmtctl_release(struct device *dev)
{
	/* Nothing to release? */
	if (RMTCTL_DEBUG)
		printk("rmtctl_release\n");
}

static u64 rmtctl_dmamask = 0xffffffff;

static struct platform_device rmtctl_device = {
	/* 
	 * Platform bus will compare the driver name 
	 * with the platform device name. 
	 */
	.name = "wmt-rmtctl",
	.id = 0,
	.dev = { 
		.release = rmtctl_release,
		.dma_mask = &rmtctl_dmamask,
	},
	.num_resources = 0,     /* ARRAY_SIZE(rmtctl_resources), */
	.resource = NULL,       /* rmtctl_resources, */
};

static int __init rmtctl_init(void)
{
	int ret;
    unsigned int varlen = 160;
    unsigned char varbuf[64];
    int v_id;
    unsigned int param[12];

	if (RMTCTL_DEBUG)
		printk("rmtctl_init\n");

    /* Jiun 01/12/2013 for oem remote controller
     * wmt.cir.param [u_debug]:[vid]:[1power]:[2home]:[3back]:[4enter]:[5menu]:[6up]:[7down]:[8left]:[9right]:[VOLUMEUP]:[VOLUMEDOWN]
     * debug = 1: will show vender id and each key scancode
     * debug = 0: active rest keys
     */
    if (wmt_getsyspara("wmt.cir.param", varbuf, &varlen)) {
        printk("wmt.cir.param not defined!\n");
        sus_key = 0xf10ebf40;   // wondermedia default
    } else {
        sscanf(varbuf, "%d:%x:%x:%x:%x:%x:%x:%x:%x:%x:%x:%x:%x",
            &u_debug, &v_id, &param[0], &param[1], &param[2], &param[3], &param[4],
            &param[5], &param[6], &param[7], &param[8], &param[9], &param[10]);

        if (u_debug) {
            printk("cir: start debug mode\n");
        } else {
            rmt_dev_tbl[0].vender_id = v_id;
            rmt_dev_tbl[0].key_codes[param[0]] = KEY_POWER;         /* power down */
            rmt_dev_tbl[0].key_codes[param[1]] = KEY_HOMEPAGE;      /* home */
            rmt_dev_tbl[0].key_codes[param[2]] = KEY_BACK;          /* back */
            rmt_dev_tbl[0].key_codes[param[3]] = KEY_ENTER;         /* enter */
            rmt_dev_tbl[0].key_codes[param[4]] = KEY_MENU;          /* menu */
            rmt_dev_tbl[0].key_codes[param[5]] = KEY_UP;            /* up */
            rmt_dev_tbl[0].key_codes[param[6]] = KEY_DOWN;          /* down */
            rmt_dev_tbl[0].key_codes[param[7]] = KEY_LEFT;          /* left */
            rmt_dev_tbl[0].key_codes[param[8]] = KEY_RIGHT;         /* right */
            rmt_dev_tbl[0].key_codes[param[9]] = KEY_VOLUMEUP;      /* vol+ */
            rmt_dev_tbl[0].key_codes[param[10]] = KEY_VOLUMEDOWN;   /* vol- */

            sus_key = ((v_id & 0xff)<<8) | ((v_id & 0xff00)>>8) | (param[0]<<16)|(~param[0])<<24;
            printk("cir: %d:0x%x:0x%x:0x%x:0x%x:0x%x:0x%x:0x%x:0x%x:0x%x:0x%x:0x%x:0x%x, 0x%x\n", 
                   u_debug, v_id, param[0], param[1], param[2], param[3], param[4],
                   param[5], param[6], param[7], param[8], param[9], param[10], sus_key);
        }
    }


	if (platform_device_register(&rmtctl_device))//add by jay,for modules support
		return -1;
	ret = platform_driver_probe(&rmtctl_driver, rmtctl_probe);

	return ret;
}

static void __exit rmtctl_exit(void)
{
	if (RMTCTL_DEBUG)
		printk("rmtctl_exit\n");

	(void)platform_driver_unregister(&rmtctl_driver);
	(void)platform_device_unregister(&rmtctl_device);//add by jay,for modules support
}

module_init(rmtctl_init);
module_exit(rmtctl_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);

