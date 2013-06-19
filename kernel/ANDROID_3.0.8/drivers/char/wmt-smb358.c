/*++
	drivers/char/wmt-smb358.c - SMB358 Battery charging driver

	Copyright (c) 2012  WonderMedia Technologies, Inc.

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
/* 2012/07/20 Diable OTG function because SMB358 A1 bug */ 

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <mach/hardware.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <linux/workqueue.h>
#include <mach/wmt_mmap.h>
#include <linux/slab.h>


static int __devinit smb_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id);
#if 0
static unsigned int smb_read(unsigned int reg);
#endif

static int smb_write(unsigned int reg, unsigned int value);
static unsigned int g_battery_charging_en;
static unsigned int g_i2cbus_id = 3;

#ifdef CONFIG_MTD_WMT_SF
extern int wmt_getsyspara(char *varname, unsigned char *varval, int *varlen);
#endif

#define smb_I2C_ADDR    0x6a

#define SUSGPIO0_EVENT_ID 0x10
static struct i2c_client *smb_client;

static struct workqueue_struct *smb_wq;
struct work_struct smb_work;
static int pmc_status=0; 

extern void pmc_enable_wakeup_event(unsigned int wakeup_event, unsigned int type);
extern void pmc_disable_wakeup_event(unsigned int wakeup_event);
extern int pmc_register_callback(unsigned int wakeup_event, void (*callback)(void *), void *callback_data);
extern int pmc_unregister_callback(unsigned int wakeup_event);
extern unsigned int pmc_get_wakeup_status(void);

static void smb_enable_irq(void)
{
	pmc_enable_wakeup_event(SUSGPIO0_EVENT_ID, 4);
}

static void smb_disable_irq(void)
{
	pmc_disable_wakeup_event(SUSGPIO0_EVENT_ID);
}

static int smb_suspend(struct i2c_client *client, pm_message_t mesg)
{
	printk("smb_suspend \n");
	smb_disable_irq();
	cancel_work_sync(&smb_work);
	smb_write(0x30, 0x8a);  // OTG disable charging enable 

	return 0;
}

static int smb_resume(struct i2c_client *client)
{
	unsigned int wakeup_sts;
	wakeup_sts = pmc_get_wakeup_status();
	printk("smb resume \n");

	queue_work(smb_wq, &smb_work);
	smb_enable_irq();

	return 0;
}



static const struct i2c_device_id smb_i2c_id[] = {
	{ "smb358", 1 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, smb_i2c_id);

static struct i2c_driver smb_i2c_driver = {
	.driver = {
		.name = "smb358",
		.owner = THIS_MODULE,
	},
	.probe = smb_i2c_probe,
	.id_table = smb_i2c_id,
	.suspend = smb_suspend,
	.resume = smb_resume,

};

struct smb_conf_struct {
	unsigned char bus_id;	    // 0: bus-0  
	                               // 1: bus-1
	                               // 2: bus-2
	struct i2c_client *client;
	struct work_struct work;

	struct i2c_adapter *i2c_adap;								   
};
static struct smb_conf_struct smb_conf;

static struct i2c_board_info __initdata smb_board_info[] = {
	{
		I2C_BOARD_INFO("smb358", smb_I2C_ADDR),
	},
};

static void smb_irq_callback(void *data)
{
	queue_work(smb_wq, &smb_work);
}


#ifdef CONFIG_MTD_WMT_SF
static void parse_arg(void)
{
	int retval;
	unsigned char buf[80];
	unsigned char tmp_buf[80];
	int varlen = 80;
	char *varname = "wmt.bc.param";
	int i = 0;
	int j = 0;
	retval = wmt_getsyspara(varname, buf, &varlen);
	if (retval == 0) {
		for (i = 0; i < 80; ++i) {
			if (buf[i] == ':')
				break;
			g_battery_charging_en = (buf[i] - '0' == 1)?1:0;
			if (g_battery_charging_en == 0)/*disable*/
				return;
		}
		++i;
		for (; i < 80; ++i) {
			if (buf[i] == ':')
				break;
			tmp_buf[j] = buf[i];
			++j;
		}
		if (!strncmp(tmp_buf,"smb358",6))
			g_battery_charging_en = 1;
		else
			g_battery_charging_en = 0;
		++i;
		g_i2cbus_id = buf[i] - '0';
	} else
		g_battery_charging_en = 0;
}
#endif
static int smb_modinit(void)
{
	int ret = 0;
	struct i2c_adapter *adapter = NULL;
	struct i2c_client *client   = NULL;

	printk(" smb_modinit \n");
#ifdef CONFIG_MTD_WMT_SF
	parse_arg();
#endif

	if (!g_battery_charging_en) {
		printk("NO SMB358\n");
		return -EIO;
	}

	smb_conf.bus_id = g_i2cbus_id;
	adapter  = i2c_get_adapter(smb_conf.bus_id);
	smb_conf.i2c_adap = adapter;
	if (adapter == NULL) {
		printk("can not get smb i2c adapter, client address error");
		return -ENODEV;
	}
	if ((client=i2c_new_device(adapter, smb_board_info)) == NULL) {
		printk("allocate smb i2c client failed");
		return -ENOMEM;
	}
	
	ret = i2c_add_driver(&smb_i2c_driver);
	if(ret ) 
		printk("SMB 358 i2c fail\n");

	return ret;
}
module_init(smb_modinit);

static void smb_work_func(struct work_struct *work)
{
	int status;

	if(pmc_status & 0x4000) // power button 
	{
		printk("smb disable OTG \n");
		smb_write(0x30, 0x8a);  // OTG disable charging enable 
		pmc_status = 0;
	}
	else
	{
		status =REG32_VAL(0xFE110000) & 0x00200000; 
		if(!status ) 
		{
			printk("smb insert 0x30=0x98  (A1 bug, turn off it) \n");
			// smb_write(0x30, 0x98);  // OTG enable charging disbale 
		}
		else	
		{
			printk("smb not insert 0x30=0x8a\n");
			smb_write(0x30, 0x8a);  // OTG disable charging enable 
		}
	}

}
static int __devinit smb_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct smb_conf_struct  *smb;

	int ret = 0;
	int reg;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk("SMB  I2C_FUNC_I2C\n");
	}

	smb = kzalloc(sizeof(*smb), GFP_KERNEL);
	if(smb == NULL)
		printk("SMB  FAIL\n");

	smb->client = client;
	smb_client = client;
	i2c_set_clientdata(client,smb);

	printk(" smb_i2c_probe \n");

	smb_wq = create_workqueue("smb_wq");
	if(smb_wq == NULL)
               printk("smb work fail \n");

	INIT_WORK(&smb_work,smb_work_func);

	reg=REG32_VAL(0xFE110000) & 0x00200000;  //  read sus_gpio0 status 
	if(!reg) 
	{
		printk("smb insert 0x30=0x98  (A1 bug, turn off it) \n");
		//smb_write(0x30, 0x98);  // OTG enable charging disbale 
	}
	else	
	{
		printk("smb not insert\n");
		smb_write(0x30, 0x8a);  // OTG disable charging enable 
	}

	ret = pmc_register_callback(SUSGPIO0_EVENT_ID, smb_irq_callback, client);
	if (ret == 0)
		smb_enable_irq();
	return ret;
}

static int smb_write(unsigned int reg, unsigned int value)
{
	int count;
	struct i2c_msg msg[1];
	unsigned char buf[2];
	count=10;

	while (count--) {
		buf[0] = reg;
		buf[1] = value;
		msg[0].addr = smb_I2C_ADDR;
		msg[0].flags = 0 ;
		msg[0].flags &= ~(I2C_M_RD);
		msg[0].len = 2;
		msg[0].buf = buf;
	
		if (i2c_transfer(smb_conf.i2c_adap, msg, ARRAY_SIZE(msg)) == ARRAY_SIZE(msg))
			return 0;
	} 
	printk(" error: i2c write reg[%02x]=%02x ", reg, value);
	return -1;
}

#if 0
static unsigned int smb_read(unsigned int reg)
{
	unsigned char buf[1];
	struct i2c_msg msg[2];
	
	msg[0].addr = smb_I2C_ADDR;
	msg[0].flags = (2 | I2C_M_NOSTART);
	msg[0].len = 1;
	msg[0].buf = (unsigned char *)&reg;

	msg[1].addr = smb_I2C_ADDR;
	msg[1].flags = (I2C_M_RD | I2C_M_NOSTART);
	msg[1].len = 1;
	msg[1].buf = buf;

	if (i2c_transfer(smb_conf.i2c_adap, msg, ARRAY_SIZE(msg)) == ARRAY_SIZE(msg))
		return buf[0];

	printk(" error: i2c read reg[%02x]", reg);
	return -1;
	
}
#endif

static void __exit smb_exit(void)
{
	i2c_del_driver(&smb_i2c_driver);

}
module_exit(smb_exit);

MODULE_DESCRIPTION("WMT SMB358 driver");
MODULE_AUTHOR("WonderMedia Technologies, Inc.");
MODULE_LICENSE("GPL");
