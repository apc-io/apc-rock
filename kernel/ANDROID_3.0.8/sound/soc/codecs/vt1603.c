/*++ 
 * linux/sound/soc/codecs/vt1603.c
 * WonderMedia audio driver for ALSA
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
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include <mach/hardware.h>
#include "vt1603.h"
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <linux/workqueue.h>
#include <mach/wmt-spi.h>

#define VT1603_VERSION "0.20"
#define VT1603_I2C_ADDR    (0x1a)
#define SPI_BUFSIZ 32
#define VT1603_I2C_RETRY_TIMES (100)



/*
 * Debug
 */
#define AUDIO_NAME "VT1603"
//#define WMT_VT1603_DEBUG 1
//#define WMT_VT1603_DEBUG_DETAIL 1

#ifdef WMT_VT1603_DEBUG
#define DPRINTK(format, arg...) \
	printk(KERN_INFO AUDIO_NAME ": " format "\n" , ## arg)
#else
#define DPRINTK(format, arg...) do {} while (0)
#endif

#ifdef WMT_VT1603_DEBUG_DETAIL
#define DBG_DETAIL(format, arg...) \
	printk(KERN_INFO AUDIO_NAME ": [%s]" format "\n" , __FUNCTION__, ## arg)
#else
#define DBG_DETAIL(format, arg...) do {} while (0)
#endif

#define err(format, arg...) \
	printk(KERN_ERR AUDIO_NAME ": " format "\n" , ## arg)
#define info(format, arg...) \
	printk(KERN_INFO AUDIO_NAME ": " format "\n" , ## arg)
#define warn(format, arg...) \
	printk(KERN_WARNING AUDIO_NAME ": " format "\n" , ## arg)


/* If the work_struct layer weren't so broken, we could pass this kind of 
 * data around.
 */
static struct snd_soc_codec *vt1603_soc_codec;

static unsigned char vt1603_classd_or_hp = 0; // 0: class-d 
                                              // 1: headphone
//static unsigned char vt1603_line_or_mic  = 1; // 0: line    
                                              // 1: mic

#define POLL_PERIOD    500 // ms
struct timer_list vt1603_timer;
static struct work_struct vt1603_timer_work;
static struct work_struct vt1603_hw_mute_work;

extern int wmt_getsyspara(char *varname, unsigned char *varval, int *varlen);
extern int wmt_i2c_xfer_continue_if_4(struct i2c_msg *msg, 
		unsigned int num,int bus_id);

/* codec hw configuration */
static int hw_mute_flag = 0;

struct vt1603_conf_struct {
	unsigned char hp_plugin_level; // 0: plugin low 
	                               // 1: plugin high
	unsigned char stereo_or_mono;  // 0: stereo     
	                               // 1: mono
	unsigned char bus_type; 	   // 0: i2c-bus => 0  
	                               // 1: spi-bus => 1
	unsigned char bus_id;	       // 0: bus-0  
	                               // 1: bus-1
	                               // 2: bus-2
	unsigned char record_src;      // 0: linein     
	                               // 1: analog-mic
	                               // 2: digital-mic
	unsigned char volume_percent;
	struct i2c_adapter *i2c_adap;
	unsigned char headmic_det;	   // 0: disable headset mic detect     
	                               // 1: enable headset mic detect
	unsigned char classD_gain;	   // 0 ~ 7: Class-D AC boost gain
								   // 0xFF: invalid value
	struct snd_soc_codec *codec;
	unsigned char in_record;
	unsigned char trigger_stat;
};
static struct vt1603_conf_struct vt1603_conf;

/* check fm34 */
static int fm34_enable  = 0;
static int fm34_i2c_bus = 0;

/* when wmt.vt1603.debug is set, enable regs dump */
#define VT1603_DBG_INTERVAL (10 * 1000 / POLL_PERIOD) // interval=10s
static int vt1603_dbg_flag = 0;

/*
 * vt1603 register cache
 * We can't read the VT1603 register space when we
 * are using 2 wire for device control, so we cache them instead.
 */
static const u16 vt1603_regs[] = {
	0x0020, 0x00D7, 0x00D7, 0x0004,  /* 0-3 */
	0x0000, 0x0040, 0x0000, 0x00c0,  /* 4-7 */
	0x00c0, 0x0010, 0x0001, 0x0041,  /* 8-11 */
	0x0000, 0x0000, 0x000b, 0x0093,  /* 12-15 */
	0x0004, 0x0016, 0x0026, 0x0060,  /* 16-19 */
	0x001a, 0x0000, 0x0002, 0x0000,  /* 20-23 */
	0x0000, 0x000a, 0x0000, 0x0010,  /* 24-27 */
	0x0000, 0x00e2, 0x0000, 0x0000,  /* 28-31 */
	0x0000, 0x0000, 0x0000, 0x0000,  /* 32-35 */
	0x0000, 0x000c, 0x0054, 0x0000,  /* 36-39 */
	0x0001, 0x000c, 0x000c, 0x000c,  /* 40-43 */
	0x000c, 0x000c, 0x00c0, 0x00d5,  /* 44-47 */
	0x00c5, 0x0012, 0x0085, 0x002b,  /* 48-51 */
	0x00cd, 0x00cd, 0x008e, 0x008d,  /* 52-55 */
	0x00e0, 0x00a6, 0x00a5, 0x0050,  /* 56-59 */
	0x00e9, 0x00cf, 0x0040, 0x0000,  /* 60-63 */
	0x0000, 0x0008, 0x0004, 0x0000,  /* 64-67 */
	0x0040, 0x0000, 0x0040, 0x0000,  /* 68-71 */
	0x0000, 0x0000, 0x0000, 0x0000,  /* 72-75 */
	0x0000, 0x0000, 0x0000, 0x0000,  /* 76-79 */

	0x0000, 0x0000, 0x0000, 0x0000,  /* 80-83 */
	0x0000, 0x0000, 0x0000, 0x0000,  /* 84-87 */
	0x0000, 0x0000, 0x0000, 0x0000,  /* 88-91 */
	0x0000, 0x0000, 0x0000, 0x0000,  /* 92-95 */
	0x00c4, 0x0079, 0x000c, 0x0024,  /* 96-99 */
	0x0016, 0x0016, 0x0060, 0x0002,  /* 100-103 */
	0x005b, 0x0003, 0x0039, 0x0039,  /* 104-107 */
	0x00fe, 0x0012, 0x0034, 0x0000,  /* 108-111 */
	0x0004, 0x00f0, 0x0000, 0x0000,  /* 112-115 */
	0x0000, 0x0000, 0x0003, 0x0000,  /* 116-119 */
	0x0000, 0x0000, 0x0000, 0x0000,  /* 120-123 */
	0x00f2, 0x0020, 0x0023, 0x006e,  /* 124-127 */
	0x0019, 0x0040, 0x0000, 0x0004,  /* 128-131 */
	0x000a, 0x0075, 0x0035, 0x00b0,  /* 132-135 */
	0x0040, 0x0000, 0x0000, 0x0000,  /* 136-139 */
	0x0088, 0x0013, 0x000c, 0x0003,  /* 140-143 */
	0x0000, 0x0000, 0x0008, 0x0000,  /* 144-147 */
	0x0003, 0x0020, 0x0030, 0x0018,  /* 148-151 */
	0x001b,
};


static int vt1603_spi_write_then_read(struct spi_device *spi,
		const u8 *txbuf, unsigned n_tx,
		u8 *rxbuf, unsigned n_rx)
{
	static DEFINE_MUTEX(lock);
	
	int		 status;
	struct spi_message	message;
	struct spi_transfer	x;
	u8		 *local_buf;
	static u8  buf[SPI_BUFSIZ];

	/* Use preallocated DMA-safe buffer.  We can't avoid copying here,
	 * (as a pure convenience thing), but we can keep heap costs
	 * out of the hot path ...
	 */
	if ((n_tx + n_rx) > SPI_BUFSIZ)
		return -EINVAL;
	
	spi_message_init(&message);
	memset(&x, 0, sizeof x);
	x.len = n_tx + n_rx;
	spi_message_add_tail(&x, &message);
	
	/* ... unless someone else is using the pre-allocated buffer */
	if (!mutex_trylock(&lock)) {
		local_buf = kmalloc(SPI_BUFSIZ, GFP_KERNEL);
		if (!local_buf)
			return -ENOMEM;
	} 
	else {
		memset(buf, 0x00, SPI_BUFSIZ);
		local_buf = buf;
	}
	
	memcpy(local_buf, txbuf, n_tx);
	x.tx_buf = local_buf;
	x.rx_buf = local_buf;
	
	/* do the i/o */
	status = spi_sync(spi, &message);
	if (status == 0)
		memcpy(rxbuf, x.rx_buf + n_tx, n_rx);
	
	if (x.tx_buf == buf)
		mutex_unlock(&lock);
	else
		kfree(local_buf);
	
	return status;
}

static int vt1603_write(struct snd_soc_codec *codec, 
		unsigned int reg, unsigned int value)
{
	unsigned char data[3];
	int count;
	struct i2c_msg msg[1];
	unsigned char buf[2];
	
	if (vt1603_conf.bus_type == WMT_SND_SPI_BUS) {
		count = 0;
		
		data[0] = ((reg & 0XFF) | BIT7); 
		data[1] = ((reg & 0XFF) >> 7);
		data[2] = value & 0xff;

		if (!codec->control_data) {
			err(" codec->control_data = NULL");
			return -1;
		}

		//DBG_DETAIL();
	
		while(codec->hw_write(codec->control_data, data, 3)){
			mdelay(1);  
			count++;
			if(count == 50)
				break;
		}
	
		return 0;
	}
	else {
		count = VT1603_I2C_RETRY_TIMES;
		
		while (count--) {
			buf[0] = reg;
			buf[1] = value;
			msg[0].addr = VT1603_I2C_ADDR;
			msg[0].flags = 0 ;
			msg[0].flags &= ~(I2C_M_RD);
			msg[0].len = 2;
			msg[0].buf = buf;
		
			if (i2c_transfer(vt1603_conf.i2c_adap, msg, ARRAY_SIZE(msg)) == ARRAY_SIZE(msg)) {
				return 0;
			}
			
			mdelay(1);
		}

		err(" error: i2c write reg[%02x]=%02x ", reg, value);
		return -1;
	}
}

static unsigned int vt1603_read(struct snd_soc_codec *codec, unsigned int reg)
{
	u8 addr[3];
	u8 data[3];
	int count;
	unsigned char buf[1];
	struct i2c_msg msg[2];
	u16 *cache = codec->reg_cache;
	
	if (vt1603_conf.bus_type == WMT_SND_SPI_BUS) {
		count = 0;
		
		addr[0] = ((reg & 0XFF) & (~ BIT7)); 
		addr[1] = ((reg & 0XFF) >> 7);
		addr[2] = 0xff;

		if (!codec->control_data) {
			err(" codec->control_data = NULL");
			return -1;
		}
	
		while(vt1603_spi_write_then_read(codec->control_data, addr, 2, data, 3)){
			mdelay(1);
			count++;
			if(count == 50)
				break;
		}
	
		data[2]&=0x00ff;  
		return data[2];
	}
	else {
		count = VT1603_I2C_RETRY_TIMES;
		
		while (count--) {
			msg[0].addr = VT1603_I2C_ADDR;
			msg[0].flags = (2 | I2C_M_NOSTART);
			msg[0].len = 1;
			msg[0].buf = (unsigned char *)&reg;

			msg[1].addr = VT1603_I2C_ADDR;
			msg[1].flags = (I2C_M_RD | I2C_M_NOSTART);
			msg[1].len = 1;
			msg[1].buf = buf;
	
			if (i2c_transfer(vt1603_conf.i2c_adap, msg, ARRAY_SIZE(msg)) == ARRAY_SIZE(msg)) {
				return buf[0];
			}
			
			mdelay(1);
		}

		err(" error: i2c read reg[%02x]", reg);
		return cache[reg]; // i2c read error, read cache instead.
	}
}

static int vt1603_codec_write(struct snd_soc_codec *codec, 
		unsigned int reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;

	//DBG_DETAIL();
	
	if (vt1603_write(codec, reg, value)) {
		err("fail!");
		return -1;
	}	
	cache[reg] = value;
	return 0;
}

static unsigned int vt1603_codec_read(struct snd_soc_codec *codec, 
		unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	
	//DBG_DETAIL();
	
	return cache[reg];
}

int vt1603_hwdep_ioctl(u8 rw_flag, u16 offset, u16 value)
{
	//info("rw_flag=%d, offset=0x%x, value=0x%x", rw_flag, offset, value);

	if (rw_flag) {
		/* open write register for DRC control and EQ only */
		if (((offset >= VT1603_R0e) && (offset <= VT1603_R13)) ||
			((offset >= VT1603_R28) && (offset <= VT1603_R3f)) ||
			(offset == VT1603_R06)) {
			snd_soc_write(vt1603_conf.codec, offset, value);
		}
		else {
			info("write to offset 0x%x is not allowed", offset);
		}
		return 0;
	}
	else {
		return snd_soc_read(vt1603_conf.codec, offset);
	}
}
EXPORT_SYMBOL(vt1603_hwdep_ioctl);

static void vt1603_regs_dump(struct snd_soc_codec *codec)
{
	int i = 0;
	
	err(">>>>>>>>>>>>>>>>>>>>>>");
	for(i = 0; i < ARRAY_SIZE(vt1603_regs); i++)
		printk(KERN_INFO "reg[%02x] = %02x\n", i, vt1603_read(codec, i));
	printk(KERN_INFO "<<<<<<<<<<<<<<<<<<<<<<\n\n");
}
 
static int vt1603_reset(struct snd_soc_codec *codec)
{
	DBG_DETAIL();
	
	vt1603_write(codec, VT1603_RC2, 0x01);
	vt1603_write(codec, VT1603_RESET, 0xff);//reset reg
	vt1603_write(codec, VT1603_RESET, 0x00);//reset reg
	vt1603_write(codec, VT1603_R60, 0x04);
	vt1603_write(codec, VT1603_R60, 0xc4);  //reset DAC/ADC analog
	
	//msleep(30);
	return 0;
}

static int vt1603_hp_power_up(struct snd_soc_codec *codec)
{
	u16 reg;

	DBG_DETAIL();

	/* disable mono mixing */
	reg = snd_soc_read(codec, VT1603_R0d);
	reg &= ~BIT4;
	snd_soc_write(codec, VT1603_R0d, reg);

	/* make sure DAC R channel is enable */
	reg = snd_soc_read(codec, VT1603_R62);
	reg |= BIT4;
	snd_soc_write(codec, VT1603_R62, reg);
	
	// set HP power up
	reg = snd_soc_read(codec, VT1603_R68);
	reg &= ~BIT4;
	snd_soc_write(codec, VT1603_R68, reg);
	
	return 0;
}

static int vt1603_classd_power_up(struct snd_soc_codec *codec)
{
	u16 reg;

	DBG_DETAIL();
				
	if (vt1603_conf.stereo_or_mono) {
		/* enable mono mixing */
		reg = snd_soc_read(codec, VT1603_R0d);
		reg |= BIT4;
		snd_soc_write(codec, VT1603_R0d, reg);

		/* disable DAC R channel */
		reg = snd_soc_read(codec, VT1603_R62);
		reg &= ~BIT4;
		snd_soc_write(codec, VT1603_R62, reg);
	} 
	
	// set class-d power up
	reg = snd_soc_read(codec, VT1603_R25);
	reg |= BIT1;
	snd_soc_write(codec, VT1603_R25, reg);
	
	return 0;
}

static int vt1603_hp_power_off(struct snd_soc_codec *codec)
{
	u16 reg;

	DBG_DETAIL();
	
	// set HP power off
	reg = snd_soc_read(codec, VT1603_R68);
	reg |= BIT4;
	snd_soc_write(codec, VT1603_R68, reg);
	
	return 0;
}

static int vt1603_classd_power_off(struct snd_soc_codec *codec)
{
	u16 reg;
	
	DBG_DETAIL();
	
	// set class-d power off
	reg = snd_soc_read(codec, VT1603_R25);
	reg &= ~BIT1;
	snd_soc_write(codec, VT1603_R25, reg);
	
	return 0;
}

static int vt1603_set_default_setting(struct snd_soc_codec *codec)
{
	u16 reg;

	DBG_DETAIL();

	// r4 is reserved, I don't know more about it.
	snd_soc_write(codec, VT1603_R04, 0x00);
	
	// r9[5] is reserved, I don't know more about it.
	reg = snd_soc_read(codec, VT1603_R09);
	reg |= BIT5;
	snd_soc_write(codec, VT1603_R09, reg);

	// r79 is reserved, I don't know more about it.
	reg = snd_soc_read(codec, VT1603_R79);
	reg |= BIT2;
	reg &= ~(BIT0+BIT1);
	snd_soc_write(codec, VT1603_R79, reg);

	// r7a is reserved, I don't know more about it.
	reg = snd_soc_read(codec, VT1603_R7a);
	reg |= BIT3+BIT4+BIT7;
	snd_soc_write(codec, VT1603_R7a, reg);

	// r87 is reserved, I don't know more about it.
	snd_soc_write(codec, VT1603_R87, 0x90);

	// r88 is reserved, I don't know more about it.
	snd_soc_write(codec, VT1603_R88, 0x28);

	// r8a is reserved, I don't know more about it.
	reg = snd_soc_read(codec, VT1603_R8a);
	reg |= BIT1;
	snd_soc_write(codec, VT1603_R8a, reg);

	//-------------------------------------------

	// enable CLK_SYS, CLK_DSP, CLK_AIF
	reg = snd_soc_read(codec, VT1603_R40);
	reg |= BIT4+BIT5+BIT6;
	snd_soc_write(codec, VT1603_R40, reg);

	// set DAC sample rate divier=CLK_SYS/1, DAC clock divider=DAC_CLK_512X/2
	reg = snd_soc_read(codec, VT1603_R41);
		
	if (vt1603_conf.record_src == WMT_SND_DMIC_IN) {
		/*
		R41H
		BIT	Type	Defaut      Description	                 
		7:4	RW	0000	DMIC Sample Rate Divider

		0000 = CLK_SYS / 1
		0011 = CLK_SYS / 4----->0x30 is Not OK. DMIC CLK = 2.8224MHz too high for DMIC?
		0111 = CLK_SYS / 8----->0x70 is OK. DMIC CLK = 11.2896M/8 = 1.4112M
		1111 = CLK_SYS / 16---->0xf0  is OK. DMIC CLK = 720K

		*/
		reg &=~BIT7;
		reg |= (BIT6+BIT5+BIT4);//0x70
	}
	
	snd_soc_write(codec, VT1603_R41, reg);

	// set BCLK rate=CLK_SYS/8
	reg = snd_soc_read(codec, VT1603_R42);
	reg |= BIT0+BIT1+BIT2;
	reg &= ~BIT3;
	snd_soc_write(codec, VT1603_R42, reg);

	if (vt1603_conf.record_src == WMT_SND_DMIC_IN) {
		reg = 0x03;
		snd_soc_write(codec, VT1603_R16, reg);

		reg = 0x00; //All default.0dB Gain on DMIC.
		/*
		//DMIC Gain Shift bit [4:3] 00:0dB,  11:3dB
		*/
		//reg |= (BIT4+BIT3);//3dB Gain on DMIC.
		snd_soc_write(codec, VT1603_R17, reg);
	
		/*
			R24(18h):
			BIT	Type	PWD	Description	Output
			Bit[7:4]	RW	0	Reserved.	N/A
			Bit[3:2]	RW	00	Digital MIC boost gain for L channel.
			00: 0dB;
			01: +12dB;
			10: +24dB;
			11: +36dB.	DMIC_GL[1:0]
			Bit[1:0]	RW	00	Digital MIC boost gain for R channel.
			00: 0dB;
			01: +12dB;
			10: +24dB;
			11: +36dB.	DMIC_GR[1:0]
		*/
		reg = 0x0a;
		snd_soc_write(codec, VT1603_R18, reg);

		reg = 0x04;
		snd_soc_write(codec, VT1603_R25, reg);
	}

	//-------------------------------------------

	// enable VREF_SC_DA buffer
	reg = snd_soc_read(codec, VT1603_R61);
	reg |= BIT7;
	snd_soc_write(codec, VT1603_R61, reg);

	// ADC VREF_SC offset voltage
	snd_soc_write(codec, VT1603_R93, 0x20);

	// set mic bias voltage = 90% * AVDD
	reg = snd_soc_read(codec, VT1603_R92);
	reg |= BIT2;
	snd_soc_write(codec, VT1603_R92, reg); 

	// enable bypass for AOW0 parameterized HPF
	reg = snd_soc_read(codec, VT1603_R0a);
	reg |= BIT6;
	snd_soc_write(codec, VT1603_R0a, reg);

	/*
	 * Let CPVEE=2.2uF, the ripple frequency ON CPVEE above 22kHz, 
	 * that will decrease HP noise. Suggested by Sunday.
	 */
	reg = snd_soc_read(codec, VT1603_R7a);
	reg |= BIT6;
	reg &= ~BIT5;
	snd_soc_write(codec, VT1603_R7a, reg);

	//-------------------------------------------

	return 0;
}

/*
 * capture  : phys mic->micboost->pga->adc
 * playback : i2s->dac->mixer->cld-mixer->spk [not use now]
 *            i2s->dac->cld-mixer->spk   [default: spk not support incall]
 *
 *            i2s->dac->mixer->hp-mixer->hp
 * incall   : [no initialization]
 */
static int vt1603_set_default_route(struct snd_soc_codec *codec)
{
	u16 reg;
	
	DBG_DETAIL();

	// disable EQ
	reg = snd_soc_read(codec, VT1603_R28);
	reg &= ~BIT0;
	snd_soc_write(codec, VT1603_R28, reg);

	// disable DAC soft mute
	reg = snd_soc_read(codec, VT1603_R0b);
	reg |= BIT1+BIT2;
	reg &= ~BIT0;
	snd_soc_write(codec, VT1603_R0b, reg);

	// enable DAC channels
	reg = snd_soc_read(codec, VT1603_R62);
	reg |= BIT7+BIT6;
	reg |= BIT4+BIT5;
	reg &= ~BIT3;
	snd_soc_write(codec, VT1603_R62, reg);

	if (vt1603_conf.record_src != WMT_SND_DMIC_IN) {
		// enable ADC channels 
		reg = snd_soc_read(codec, VT1603_R63);
		reg |= BIT6+BIT7;
	}
	else {	//add for DMIC support.
		reg = 0xe4;
	}
	reg &= ~(BIT6 | BIT7);
	snd_soc_write(codec, VT1603_R63, reg);
	
	//-------------------------------------------

	// enable microphone bias
	reg = snd_soc_read(codec, VT1603_R60);
	reg |= 0x0f;
	snd_soc_write(codec, VT1603_R60, reg);

	// micboost power up, PGA power up
	reg = snd_soc_read(codec, VT1603_R67);
	reg |= BIT7+BIT6; // micboost power up
	reg |= BIT5+BIT4; // PGA power up
	snd_soc_write(codec, VT1603_R67, reg);

	/* record source select */
	reg = snd_soc_read(codec, VT1603_R8e);
	if ((vt1603_conf.headmic_det) ||
		(vt1603_conf.record_src == WMT_SND_MICIN_2)) {
		// Linein-LR and Micin-L disable, Micin-R enable
		reg &= ~(BIT7 + BIT6 + BIT5);
		reg |= BIT4;
	}
	else if (vt1603_conf.record_src == WMT_SND_LINEIN_1) {
		// Micin-LR and Linein-R disable, Linein-L enable
		reg &= ~(BIT6 + BIT5 + BIT4);
		reg |= BIT7;
	}
	else if (vt1603_conf.record_src == WMT_SND_LINEIN_12) {
		// Micin-LR disable, Linein-LR enable
		reg &= ~(BIT5 + BIT4);
		reg |= (BIT7 + BIT6);
	}
	else if (vt1603_conf.record_src == WMT_SND_LINEIN_2) {
		// Micin-LR and Linein-L disable, Linein-R enable
		reg &= ~(BIT7 + BIT5 + BIT4);
		reg |= BIT6;
	}
	else if (vt1603_conf.record_src == WMT_SND_MICIN_12) {
		// Linein-LR disable, Micin-LR enable
		reg &= ~(BIT7 + BIT6);
		reg |= (BIT5 + BIT4);
	}
	else {
		// Linein-LR and Micin-R disable, Micin-L enable
		reg &= ~(BIT7 + BIT6 + BIT4);
		reg |= BIT5;
	}
	reg |= 0x0f;
	snd_soc_write(codec, VT1603_R8e, reg);

	/* ADC_L/ADC_R data source select */
	reg = snd_soc_read(codec, VT1603_R03);
	if ((vt1603_conf.headmic_det) || (vt1603_conf.record_src == WMT_SND_LINEIN_2)
		|| (vt1603_conf.record_src == WMT_SND_MICIN_2)) {
		// Right ADC outputs left/right interface data
		reg |= (BIT2 | BIT3);
	}
	else if ((vt1603_conf.record_src == WMT_SND_LINEIN_12) || 
		(vt1603_conf.record_src == WMT_SND_MICIN_12)) {
		// Left ADC outputs left interface data
		// Right ADC outputs right interface data
		reg = BIT2;
	}
	else {
		// Left ADC outputs left/right interface data
		reg &= ~(BIT2 | BIT3);
	}
	snd_soc_write(codec, VT1603_R03, reg);

	// set PGA
	reg = snd_soc_read(codec, VT1603_R66);
	reg &= ~(BIT6+BIT5); // unmute PGA channels
	reg |= BIT1+BIT2+BIT3+BIT4; //
	snd_soc_write(codec, VT1603_R66, reg);

	//-------------------------------------------

	// mixer: select input from DAC to output path
	reg = snd_soc_read(codec, VT1603_R71);
	reg &= ~(BIT7+BIT6); // unmute AA path to mixer
	reg |= BIT5+BIT4;    // mute right(left) input from AA to left(right) output
	reg &= ~(BIT3+BIT2); // AA nc
	reg |= BIT1+BIT0;    // DAC connect
	snd_soc_write(codec, VT1603_R71, reg);

	// mixer: enable
	reg = snd_soc_read(codec, VT1603_R72);
	reg |= BIT6;
	snd_soc_write(codec, VT1603_R72, reg);
	
	reg = snd_soc_read(codec, VT1603_R73);
	reg |= BIT6;
	snd_soc_write(codec, VT1603_R73, reg);

	//-------------------------------------------

	// hp mixer: select input from mixer
	reg = snd_soc_read(codec, VT1603_R69);
	reg &= ~(BIT7+BIT4);
	reg |= BIT5+BIT2;
	snd_soc_write(codec, VT1603_R69, reg);

	// cld mixer: select input from mixer
	/*reg = snd_soc_read(codec, VT1603_R90);
	reg |= BIT5+BIT3;
	reg &= ~(BIT4+BIT2);
	snd_soc_write(codec, VT1603_R90, reg);*/
	
	// cld mixer: select input from ADC
	reg = snd_soc_read(codec, VT1603_R90);
	reg &= ~(BIT5+BIT3);
	reg |= BIT4+BIT2;
	snd_soc_write(codec, VT1603_R90, reg);

	// cld mixer: unmute
	reg = snd_soc_read(codec, VT1603_R91);
	reg |= BIT1+BIT0;
	snd_soc_write(codec, VT1603_R91, reg);

	//-------------------------------------------
	
	// enable HP
	reg = snd_soc_read(codec, VT1603_R68);
	reg &= ~BIT4; // HP power up
	reg |= BIT2;  // enable output mode
	reg &= ~(BIT0+BIT1); // unmute HP channels
	snd_soc_write(codec, VT1603_R68, reg);

	// enable class-d, unmute channels
	snd_soc_write(codec, VT1603_R7c, 0xe0);

	// set class-d power up
	reg = snd_soc_read(codec, VT1603_R25);
	reg |= BIT1;
	snd_soc_write(codec, VT1603_R25, reg);

	//-------------------------------------------

	return 0;
}

static int vt1603_set_default_volume(struct snd_soc_codec *codec)
{
	u16 reg;

	DBG_DETAIL();
	
	/* original: set Mic Gain Boost to 20dB, PGA Gain to 0dB, 0x97 */
	/* set Mic Gain Boost to 20dB, PGA Gain to 7.5dB, 0xa1 */
	snd_soc_write(codec, VT1603_R64, 0xa1);
	snd_soc_write(codec, VT1603_R65, 0xa1);

	// if fm34 is enable, decrease the capture gain, otherwise appear 
	// echo sound
	if (fm34_enable) {
		// set ADC gain to be 0x43
		snd_soc_write(codec, VT1603_R01, 0x43);
		snd_soc_write(codec, VT1603_R02, 0x43);
	}
	else {
		if (vt1603_conf.record_src != WMT_SND_DMIC_IN) {
			/* original: set ADC Digital Gain to 12.5dB, 0x70 */
			/* set ADC Digital Gain to 20dB, 0x7f */
			snd_soc_write(codec, VT1603_R01, 0x7f);
			snd_soc_write(codec, VT1603_R02, 0x7f);

			// set ADC gain update, enable ADCDAT output, Hi-Fi mode
			/*reg = snd_soc_read(codec, VT1603_R00);
			reg |= BIT7+BIT6+BIT5;
			snd_soc_write(codec, VT1603_R00, reg);*/
			// set ADC gain update, enable ADCDAT output, voice mode 2
			snd_soc_write(codec, VT1603_R00, 0xf0);
		}
		else {
			snd_soc_write(codec, VT1603_R01, 0x57);
			snd_soc_write(codec, VT1603_R02, 0x57);
			snd_soc_write(codec, VT1603_R00, 0xe0);
		}
	}
	
	// set AA to mixer path gain to be 0dB,
	reg = snd_soc_read(codec, VT1603_R72);
	reg = (reg & (~0x07)) + 0x07; //bit[2:0]=111: -21dB [step: -3dB/]
	snd_soc_write(codec, VT1603_R72, reg);
	
	reg = snd_soc_read(codec, VT1603_R73);
	reg = (reg & (~0x07)) + 0x07; //bit[2:0]=111: -21dB [step: -3dB/]
	snd_soc_write(codec, VT1603_R73, reg);

	// set DAC gain. max:0xc0
	reg = 0xC0 * vt1603_conf.volume_percent / 100;
	snd_soc_write(codec, VT1603_R07, reg);
	snd_soc_write(codec, VT1603_R08, reg);

	// set DAC gain update
	reg = snd_soc_read(codec, VT1603_R05);
	reg |= BIT6+BIT7;
	snd_soc_write(codec, VT1603_R05, reg);

	// set LOUT1 & ROUT1 gain from 0x39 to 0x3b
	snd_soc_write(codec,VT1603_R6a, 0x3b);
	snd_soc_write(codec,VT1603_R6b, 0x3b);

	// set class-d AC boost gain=1.6
	reg = snd_soc_read(codec, VT1603_R97);
	
	if (vt1603_conf.classD_gain == 0xFF) {
		reg += 0x04;
	}
	else {
		reg += vt1603_conf.classD_gain;
	}
	
	//info("VT1603_R97=0x%x",	reg);
	snd_soc_write(codec, VT1603_R97, reg);

	return 0;
}

static void vt1603_irq_init(struct snd_soc_codec *codec)
{
	DBG_DETAIL();
	
	/* vt1603 touch uses irq low active. */
	if (vt1603_conf.hp_plugin_level) {
		//hp high
		snd_soc_write(codec, VT1603_R21, 0xfd);
	} 
	else {
		//hp low
		snd_soc_write(codec, VT1603_R21, 0xff);
	}
	
	//snd_soc_write(codec, VT1603_R23,  0x00);//mic high
	snd_soc_write(codec, VT1603_R1b, 0xff);//for test
	snd_soc_write(codec, VT1603_R1d, 0xff);//for test
	snd_soc_write(codec, VT1603_R24, 0x04);//GPIO1 for IRQ

	snd_soc_write(codec, VT1603_R1b, 0xfd);//open HP detect
	if (vt1603_read(codec, VT1603_R1b) != 0xfd)
		err("*** vt1603 register R/W test error ***");
	
	//snd_soc_write(codec, VT1603_R1d, 0xf7);//open mic bias detect
	snd_soc_write(codec, VT1603_R1c, 0xff);
	
	snd_soc_write(codec, VT1603_R95, 0x00);//for mic bias detect
	snd_soc_write(codec, VT1603_R60, 0xc7);//for mic bias detect
	snd_soc_write(codec, VT1603_R93, 0x02);//for mic bias detect
	snd_soc_write(codec, VT1603_R7b, 0x10);//for mic bias detect
	snd_soc_write(codec, VT1603_R92, 0x2c);//for mic bias detect       
}

#define VT1603_INT_HP_DETECT     0x01 // headphone plugin detect
#define VT1603_INT_LINEIN_DETECT 0x02 // linein plugin detect
#define VT1603_INT_CLASSD_LOCP   0x04 // class-d left channel over-current
#define VT1603_INT_CLASSD_ROCP   0x08 // class-d right channel over-current

static void vt1603_irq_mask_reset(struct snd_soc_codec *codec,int flag)
{
	unsigned int value = 0;

	DBG_DETAIL();
	
	switch (flag) {
	case VT1603_INT_HP_DETECT:	
		value = vt1603_read(codec,VT1603_R1b);
		value |= BIT1;
		snd_soc_write(codec, VT1603_R1b, value);
		value &= ~BIT1;
		snd_soc_write(codec, VT1603_R1b, value);
		break;
	
	case VT1603_INT_LINEIN_DETECT:	
		value = vt1603_read(codec,VT1603_R1d);
		value |= BIT3;
		snd_soc_write(codec, VT1603_R1d, value);
		value &= ~BIT3;
		snd_soc_write(codec, VT1603_R1d, value);
		break;
	
	case VT1603_INT_CLASSD_LOCP:	
		value = vt1603_read(codec,VT1603_R1c);
		value |= BIT4;
		snd_soc_write(codec, VT1603_R1c, value);
		value &= ~BIT4;
		snd_soc_write(codec, VT1603_R1c, value);
		break;
	
	case VT1603_INT_CLASSD_ROCP:	
		value = vt1603_read(codec,VT1603_R1c);
		value |= BIT3;
		snd_soc_write(codec, VT1603_R1c, value);
		value &= ~BIT3;
		snd_soc_write(codec, VT1603_R1c, value);
		break;
	}
}

static unsigned int vt1603_irq_flag_detect(struct snd_soc_codec *codec)
{
	u16 reg;
	unsigned int flag = 0;

	//DBG_DETAIL();
	
	reg = vt1603_read(codec, VT1603_R51);
	if (reg & BIT1)
		flag |= VT1603_INT_HP_DETECT;
		
	reg = vt1603_read(codec, VT1603_R53);
	if (reg & BIT3)
		flag |= VT1603_INT_LINEIN_DETECT;
	
	reg = vt1603_read(codec, VT1603_R52);
	if (reg & BIT4)
		flag |= VT1603_INT_CLASSD_LOCP;
	if (reg & BIT3)
		flag |= VT1603_INT_CLASSD_ROCP;
		
	return flag;
}

static void vt1603_switch_to_hp(struct snd_soc_codec *codec)
{	
	u16 reg;
	
	DBG_DETAIL();
	
	vt1603_classd_power_off(codec);

	if ((vt1603_conf.headmic_det) && (!(GPIO_ID_GPIO_27_20_2BYTE_VAL & BIT6))) {
		info("Headset Mic (4Rings)");

		// select input from Micin-L
		reg = snd_soc_read(codec, VT1603_R03);
		reg &= ~(BIT2 | BIT3);
		snd_soc_write(codec, VT1603_R03, reg);

		// Linein-LR and Micin-R disable, Micin-L enable
		reg = snd_soc_read(codec, VT1603_R8e);
		reg &= ~(BIT7 + BIT6 + BIT4);
		reg |= BIT5;
		snd_soc_write(codec, VT1603_R8e, reg);
	}
	
	if (codec->card->rtd->codec_dai->playback_active == 1)
		vt1603_hp_power_up(codec);
}

static void vt1603_switch_to_classd(struct snd_soc_codec *codec)
{	
	u16 reg;
	
	DBG_DETAIL();
	
	vt1603_hp_power_off(codec);

	if (vt1603_conf.headmic_det) {
		//info("HP Plug-Out");
		// select input from Micin-R
		reg = snd_soc_read(codec, VT1603_R03);
		reg |= (BIT2 | BIT3);
		snd_soc_write(codec, VT1603_R03, reg);

		// Linein-LR and Micin-L disable, Micin-R enable
		reg = snd_soc_read(codec, VT1603_R8e);
		reg &= ~(BIT7 + BIT6 + BIT5);
		reg |= BIT4;
		snd_soc_write(codec, VT1603_R8e, reg);
	}	
	
	if (codec->card->rtd->codec_dai->playback_active == 1)
		vt1603_classd_power_up(codec);
}

static void vt1603_hp_classd_switch(struct snd_soc_codec *codec)
{
	unsigned int value=0;
	value=vt1603_read(codec, VT1603_R21);

	DBG_DETAIL();
	
	if (vt1603_conf.hp_plugin_level) {
		if (value & BIT1) {
			value &= ~BIT1;
			vt1603_classd_or_hp = 0;
			snd_soc_write(codec, VT1603_R21, value);
			vt1603_switch_to_classd(codec);
		} 
		else {
			value |= BIT1;
			vt1603_classd_or_hp = 1;
			snd_soc_write(codec, VT1603_R21, value);
			vt1603_switch_to_hp(codec);
		}
	} 
	else {
		if ((value & BIT1)) {
			value &= ~BIT1;
			vt1603_classd_or_hp = 1;
			snd_soc_write(codec, VT1603_R21, value);
			vt1603_switch_to_hp(codec);
		}
		else {
			value |= BIT1;
			vt1603_classd_or_hp = 0;
			snd_soc_write(codec, VT1603_R21, value);
			vt1603_switch_to_classd(codec);
		}
	}
}

#if 0
static void vt1603_mic_line_switch(struct snd_soc_codec *codec)
{
	unsigned int count = 0;
	unsigned int flag = 0;
	unsigned int value = vt1603_read(codec, VT1603_R23);
	
	if (value & BIT3) {
		for (count = 0; count < 200; count++) {
			flag = vt1603_read(codec, VT1603_R5f);
			if (flag & BIT3)
				break;
		}
		if (count == 200) {
			value &= ~BIT3;
			vt1603_line_or_mic = 0;
			snd_soc_write(codec, VT1603_R23, value);
		}
	} 
	else {
		value |= BIT3;
		vt1603_line_or_mic = 1;
		vt1603_read(codec, VT1603_R5f);
		snd_soc_write(codec, VT1603_R23, value);
	}
}
#endif

static void vt1603_do_timer_work(struct work_struct *work)
{
	static int dbg_cnt = 0;
	struct snd_soc_codec *codec = vt1603_soc_codec;
	unsigned int flag = vt1603_irq_flag_detect(codec);

	//DBG_DETAIL();
	
	if (flag & VT1603_INT_HP_DETECT) {
		printk("VT1603_INT_HP_DETECT\n");
		vt1603_hp_classd_switch(codec);
		vt1603_irq_mask_reset(codec, VT1603_INT_HP_DETECT);
	}
	
	if (flag & VT1603_INT_LINEIN_DETECT) {
		printk("VT1603_INT_LINEIN_DETECT\n");
		//vt1603_mic_line_switch(codec);
		vt1603_irq_mask_reset(codec, VT1603_INT_LINEIN_DETECT);
	}
	
	if (flag & VT1603_INT_CLASSD_LOCP) {
		printk("VT1603_INT_CLASSD_LOCP\n");
		vt1603_irq_mask_reset(codec, VT1603_INT_CLASSD_LOCP);
	}
	
	if (flag & VT1603_INT_CLASSD_ROCP) {
		printk("VT1603_INT_CLASSD_ROCP\n");
		vt1603_irq_mask_reset(codec, VT1603_INT_CLASSD_ROCP);
	}

	if (vt1603_dbg_flag) {
		if (dbg_cnt == VT1603_DBG_INTERVAL) {
			vt1603_regs_dump(codec);
			dbg_cnt = 0;
		}
		dbg_cnt++;
	}
}

static void vt1603_timer_handler(unsigned long data)
{
	//DBG_DETAIL();
	
	schedule_work(&vt1603_timer_work);
	mod_timer(&vt1603_timer, jiffies + POLL_PERIOD*HZ/1000);
}

static const DECLARE_TLV_DB_SCALE(digital_tlv, -9550, 50, 1);
static const DECLARE_TLV_DB_SCALE(digital_clv, -4350, 50, 1);
static const DECLARE_TLV_DB_SCALE(adc_boost_clv, 0, 1000, 1);
static const DECLARE_TLV_DB_SCALE(dac_boost_tlv, 0, 600, 0);
static const DECLARE_TLV_DB_SCALE(adc_pga_clv, -1650, 150, 1);
static const DECLARE_TLV_DB_SCALE(aa_tlv, -2100, 300, 1);

static const struct snd_kcontrol_new vt1603_snd_controls[] = {	
SOC_DOUBLE_R_TLV("Capture ADC Volume", VT1603_R01, VT1603_R02, 0, 0x70, 0, digital_clv),
SOC_DOUBLE_R_TLV("Capture Boost Volume", VT1603_R64, VT1603_R65, 6, 4, 0, adc_boost_clv),
SOC_DOUBLE_R_TLV("Capture PGA Volume", VT1603_R64, VT1603_R65, 1, 0x1f, 0, adc_pga_clv),

SOC_DOUBLE_R_TLV("Playback AA Volume", VT1603_R72, VT1603_R73, 0, 0x07, 1, aa_tlv),
SOC_DOUBLE_R_TLV("Playback DAC Volume", VT1603_R07, VT1603_R08, 0, 0xc0, 0, digital_tlv),
SOC_DOUBLE_TLV("Playback Boost Volume", VT1603_R06, 0, 3, 8, 0, dac_boost_tlv),

SOC_SINGLE("Playback Switch", VT1603_R0b, 0, 1, 1),
SOC_DOUBLE_R("Capture Switch", VT1603_R01, VT1603_R02, 7, 1, 1),
SOC_DOUBLE("SPK Switch", VT1603_R82, 3, 4, 1, 1),
SOC_SINGLE("HP Switch", VT1603_R68, 4, 1, 1),
};

/*
 * DAPM Controls
 */

static const struct snd_kcontrol_new linmix_controls[] = {
SOC_DAPM_SINGLE("Micin Switch", VT1603_R8e, 5, 1, 0),
SOC_DAPM_SINGLE("Linein Switch", VT1603_R8e, 7, 1, 0),
};

static const struct snd_kcontrol_new rinmix_controls[] = {
SOC_DAPM_SINGLE("Micin Switch", VT1603_R8e, 4, 1, 0),
SOC_DAPM_SINGLE("Linein Switch", VT1603_R8e, 6, 1, 0),
};

static const struct snd_kcontrol_new loutmix_controls[] = {
SOC_DAPM_SINGLE("AA Switch", VT1603_R71, 3, 1, 0),
SOC_DAPM_SINGLE("DAC Switch", VT1603_R71, 1, 1, 0),
};

static const struct snd_kcontrol_new routmix_controls[] = {
SOC_DAPM_SINGLE("AA Switch", VT1603_R71, 2, 1, 0),
SOC_DAPM_SINGLE("DAC Switch", VT1603_R71, 0, 1, 0),
};

static const struct snd_kcontrol_new lcld_controls[] = {
SOC_DAPM_SINGLE("MIX Switch", VT1603_R90, 5, 1, 0),
SOC_DAPM_SINGLE("DAC Switch", VT1603_R90, 4, 1, 0),
};

static const struct snd_kcontrol_new rcld_controls[] = {
SOC_DAPM_SINGLE("MIX Switch", VT1603_R90, 3, 1, 0),
SOC_DAPM_SINGLE("DAC Switch", VT1603_R90, 2, 1, 0),
};

static const struct snd_kcontrol_new lhp_controls[] = {
SOC_DAPM_SINGLE("MIX Switch", VT1603_R69, 5, 1, 0),
SOC_DAPM_SINGLE("DAC Switch", VT1603_R69, 7, 1, 0),
};

static const struct snd_kcontrol_new rhp_controls[] = {
SOC_DAPM_SINGLE("MIX Switch", VT1603_R69, 2, 1, 0),
SOC_DAPM_SINGLE("DAC Switch", VT1603_R69, 4, 1, 0),
};

static const struct snd_soc_dapm_widget vt1603_dapm_widgets[] = {
SND_SOC_DAPM_INPUT("Micin"),
SND_SOC_DAPM_INPUT("Linein"),
SND_SOC_DAPM_MIXER("Left In Mixer", VT1603_R66, 6, 0,
		linmix_controls, ARRAY_SIZE(linmix_controls)),
SND_SOC_DAPM_MIXER("Right In Mixer", VT1603_R66, 5, 0,
		rinmix_controls, ARRAY_SIZE(rinmix_controls)),

SND_SOC_DAPM_DAC("DACL", "Left Playback", VT1603_NULL, 5, 0),
SND_SOC_DAPM_DAC("DACR", "Right Playback", VT1603_NULL, 4, 0),

SND_SOC_DAPM_MIXER("Left Out Mixer", VT1603_NULL, 6, 0,
		loutmix_controls, ARRAY_SIZE(loutmix_controls)),
SND_SOC_DAPM_MIXER("Right Out Mixer", VT1603_NULL, 6, 0,
		routmix_controls, ARRAY_SIZE(routmix_controls)),

SND_SOC_DAPM_MIXER("Left CLD", VT1603_NULL, 1, 0,
		lcld_controls, ARRAY_SIZE(lcld_controls)),
SND_SOC_DAPM_MIXER("Right CLD", VT1603_NULL, 0, 0,
		rcld_controls, ARRAY_SIZE(rcld_controls)),

SND_SOC_DAPM_MIXER("Left HP", VT1603_NULL, 1, 1,
		lhp_controls, ARRAY_SIZE(lhp_controls)),
SND_SOC_DAPM_MIXER("Right HP", VT1603_NULL, 0, 1,
		rhp_controls, ARRAY_SIZE(rhp_controls)),
};

/*
 * Note: define audio path, such as: 
 *  1. linein -> inmixer -> outmixer -> spk/hp
 *  2. dac -> outmixer -> spk/hp
 *  3. dac -> spk/hp
 *  4. etc...
 * 'linein -> inmixer -> outmixer -> hp' seems some
 * problem exist: can't disable this path!!!
 */
static const struct snd_soc_dapm_route audio_map[] = {
	{ "Left In Mixer", "Linein Switch", "Linein" },
	{ "Left In Mixer", "Micin Switch", "Micin" },
	{ "Right In Mixer", "Linein Switch", "Linein" },
	{ "Right In Mixer", "Micin Switch", "Micin" },
	
	{ "Left Out Mixer", "AA Switch", "Left In Mixer" },
	{ "Left Out Mixer", "DAC Switch", "DACL" },
	{ "Right Out Mixer", "AA Switch", "Right In Mixer" },
	{ "Right Out Mixer", "DAC Switch", "DACR" },

	{ "Left CLD", "MIX Switch", "Left Out Mixer" },
	{ "Left CLD", "DAC Switch", "DACL" },
	{ "Right CLD", "MIX Switch", "Right Out Mixer" },
	{ "Right CLD", "DAC Switch", "DACR" },

	{ "Left HP", "MIX Switch", "Left Out Mixer" },
	{ "Left HP", "DAC Switch", "DACL" },
	{ "Right HP", "MIX Switch", "Right Out Mixer" },
	{ "Right HP", "DAC Switch", "DACR" },
};

struct _coeff_div {
	u32 mclk;
	u32 rate;
	u16 fs;
	u8 sr;
	u8 bclk_div;
};

/* codec hifi mclk clock divider coefficients */
static const struct _coeff_div coeff_div[] = {
	/* 8k */
	{12288000, 8000, 1536, 0x4, 0x0},
	/* 11.025k */
	{11289600, 11025, 1024, 0x8, 0x0},
	/* 16k */
	{12288000, 16000, 768, 0x5, 0x0},
	/* 22.05k */
	{11289600, 22050, 512, 0x9, 0x0},
	/* 32k */
	{12288000, 32000, 384, 0x7, 0x0},
	/* 44.1k */
	{11289600, 44100, 256, 0x6, 0x07},
	/* 48k */
	{12288000, 48000, 256, 0x0, 0x07},
	/* 96k */
	{12288000, 96000, 128, 0x1, 0x04},
};

/* codec private data */
struct vt1603_priv {
	unsigned int sysclk;
	enum snd_soc_control_type control_type;
	void *control_data;
	struct spi_device *spi;
};

static inline int get_coeff(int mclk, int rate)
{
	int i;

	DBG_DETAIL();

	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].rate == rate && coeff_div[i].mclk == mclk)
			return i;
	}

	err("vt1603: could not get coeff for mclk %d @ rate %d",
			mclk, rate);
	
	return -EINVAL;
}

static int vt1603_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct vt1603_priv *vt1603 = snd_soc_codec_get_drvdata(codec);
	u16 reg;
	u16 iface = snd_soc_read(codec, VT1603_R19) & 0x73;
	int coeff = get_coeff(vt1603->sysclk, params_rate(params));

	DBG_DETAIL();
	
	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= 0x04;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:	
		iface |= 0x08;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		iface |= 0x0c;
		break;
	}
	
	/* set iface & srate */
	snd_soc_write(codec, VT1603_R19, iface);
	
	if (coeff >= 0) {
		reg = snd_soc_read(codec, VT1603_R42);
		reg &= 0xf0;
		reg |= coeff_div[coeff].bclk_div;
		snd_soc_write(codec, VT1603_R42, reg);

		if(codec->card->rtd->codec_dai->playback_active == 1) {
			reg = snd_soc_read(codec, VT1603_R05);
			reg &= 0xf0;
			reg |= coeff_div[coeff].sr;
			snd_soc_write(codec, VT1603_R05, reg);
			
			if(vt1603_classd_or_hp)
				vt1603_switch_to_hp(codec);
			else
				vt1603_switch_to_classd(codec);
		}
               
		if(codec->card->rtd->codec_dai->capture_active == 1) {
			reg = snd_soc_read(codec, VT1603_R03);
			reg &= 0x0f;
			reg |= ((coeff_div[coeff].sr<<4) & 0xf0);
			snd_soc_write(codec, VT1603_R03, reg);
		}
	}

	return 0;
}

static int vt1603_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct vt1603_priv *vt1603 = snd_soc_codec_get_drvdata(codec);

	DBG_DETAIL("freq=%d", freq);

	switch (freq) {
	case 11289600:
	case 12000000:
	case 12288000:
	case 16934400:
	case 18432000:
		vt1603->sysclk = freq;
		return 0;
	}
	return -EINVAL;
}

static int vt1603_set_dai_fmt(struct snd_soc_dai *codec_dai, 
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface = snd_soc_read(codec, VT1603_R19);

	DBG_DETAIL();
	
	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface |= BIT5 ;
		break;	
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= 0x0002;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x0001;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface |= 0x0003;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		iface |= 0x0013;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x0090;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= 0x0080;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface |= 0x0010;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_write(codec, VT1603_R19, iface);
	
	return 0;
}

static int vt1603_mute(struct snd_soc_dai *dai, int mute)
{ 
	struct snd_soc_codec *codec = dai->codec;
	u16 reg = snd_soc_read(codec, VT1603_R0b);

	DBG_DETAIL("mute=%d", mute);
	
	if (mute) {
		reg |= BIT0;
	} 
	else {
		reg &= ~BIT0;
	}
	snd_soc_write(codec, VT1603_R0b, reg);
	
	return 0;
}

/*
 * The defferent between vt1603_hw_mute and vt1603_mute is that 
 * vt1603_mute only do software mute, while vt1603_hw_mute 
 * let L/R channel power down. 
 */
static int vt1603_hw_mute(struct snd_soc_codec *codec, int mute)
{
	u16 reg;

	DBG_DETAIL("mute=%d", mute);
	
	/* If current output mixer connect to AA, means that it's in incall mode,  
	 * don't power down class-d channels.
	 */
	/*reg = snd_soc_read(codec, VT1603_R71);
	if (reg & BIT3 || reg & BIT2)
		return 0;*/
	
	reg = snd_soc_read(codec, VT1603_R82);
	if (mute) {
		reg |= BIT4+BIT3;
	} 
	else {
		reg &= ~(BIT4+BIT3);
	}
	snd_soc_write(codec, VT1603_R82, reg);

	return 0;
}

static void vt1603_do_hw_mute_work(struct work_struct *work )
{
	struct snd_soc_codec *codec = vt1603_soc_codec;
	u16 reg;

	DBG_DETAIL();
	vt1603_hw_mute(codec, hw_mute_flag);

	if (vt1603_conf.in_record) {
		if (vt1603_conf.trigger_stat == SNDRV_PCM_TRIGGER_START) {
			reg = snd_soc_read(codec, VT1603_R63);
			reg |= (BIT6 | BIT7);
			snd_soc_write(codec, VT1603_R63, reg);
		}
		else if (vt1603_conf.trigger_stat == SNDRV_PCM_TRIGGER_STOP) {
			reg = snd_soc_read(codec, VT1603_R63);
			reg &= ~(BIT6 | BIT7);
			snd_soc_write(codec, VT1603_R63, reg);

			vt1603_conf.in_record = 0;
		}
	}
}

/*
 * vt1603_trigger to disable/enable channels output while pcm stream is 
 * pause/playing. Avoid sharp noise while pcm stream is not playing.
 * Note that don't add printk that will cause a loud POP.
 */
static int vt1603_trigger(struct snd_pcm_substream *substream, 
		int cmd, struct snd_soc_dai *codec_dai)
{
	DBG_DETAIL();
	
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			hw_mute_flag = 0;
		}
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			vt1603_conf.in_record = 1;
			vt1603_conf.trigger_stat = SNDRV_PCM_TRIGGER_START;
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			hw_mute_flag = 1;
		}
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			vt1603_conf.trigger_stat = SNDRV_PCM_TRIGGER_STOP;
		}
		break;
	}
	
	schedule_work(&vt1603_hw_mute_work);
	return 0;
}

#define VT1603_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
	SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 | \
	SNDRV_PCM_RATE_32000)

#define VT1603_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_U8)

static struct snd_soc_dai_ops vt1603_dai_ops = {
	.hw_params    = vt1603_pcm_hw_params,
	.digital_mute = vt1603_mute,
	.trigger      = vt1603_trigger, 
	.set_fmt      = vt1603_set_dai_fmt,
	.set_sysclk   = vt1603_set_dai_sysclk,
};

struct snd_soc_dai_driver vt1603_dai = {
	.name     = "VT1603",
	.playback = {
		.stream_name  = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates        = VT1603_RATES,
		.formats      = VT1603_FORMATS,
	},
	.capture  = {
		.stream_name  = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates        = VT1603_RATES,
		.formats      = VT1603_FORMATS,
	},
	.ops      = &vt1603_dai_ops,
};

static int vt1603_set_bias_level(struct snd_soc_codec *codec, 
		enum snd_soc_bias_level level)
{
	u16 reg;

	DBG_DETAIL();
	
	switch (level) {
	case SND_SOC_BIAS_ON:
		if(codec->card->rtd->codec_dai->playback_active == 1){
			if(vt1603_classd_or_hp == 0)
				vt1603_classd_power_up(codec);
			else
				vt1603_hp_power_up(codec);
		}
		break;
		
	case SND_SOC_BIAS_PREPARE:
		reg = snd_soc_read(codec, VT1603_R96);
		reg |= BIT4+BIT5;
		snd_soc_write(codec, VT1603_R96, reg);
		break;
		
	case SND_SOC_BIAS_STANDBY:
		break;
	
	case SND_SOC_BIAS_OFF:
		vt1603_hp_power_off(codec);
		vt1603_classd_power_off(codec);
		break;
	}
	
	codec->dapm.bias_level = level;
	return 0;
}

static void vt1603_do_set_bias_work(struct work_struct *work)
{
	struct snd_soc_codec *codec =
			container_of(work, struct snd_soc_codec, dapm.delayed_work.work);

	DBG_DETAIL();
	vt1603_set_bias_level(codec, codec->dapm.bias_level);
}

static int vt1603_suspend_prepare(struct snd_soc_codec *codec)
{
	u16 reg;

	DBG_DETAIL();

	// output mode disable
	reg = snd_soc_read(codec, VT1603_R68);
	reg &= ~BIT2;
	vt1603_write(codec, VT1603_R68, reg);

	// mute both channels.
	reg = snd_soc_read(codec, VT1603_R68);
	reg |= BIT0+BIT1;
	vt1603_write(codec, VT1603_R68, reg);

	// DAC software mute enable
	reg = snd_soc_read(codec, VT1603_R0b);
	reg |= BIT0+BIT2;
	vt1603_write(codec, VT1603_R0b, reg);

	// headphone power down 
	reg = snd_soc_read(codec, VT1603_R68);
	reg &= ~BIT4;
	vt1603_write(codec, VT1603_R68, reg);

	reg = snd_soc_read(codec, VT1603_R7a);
	reg &= ~(BIT3+BIT4);
	vt1603_write(codec, VT1603_R7a, reg);

	// class-d power down
	reg = snd_soc_read(codec, VT1603_R25);
	reg &= ~BIT1;
	vt1603_write(codec, VT1603_R25, reg);

	// disable AA path to mixer
	reg = snd_soc_read(codec, VT1603_R71);
	reg &= ~(BIT2+BIT3);
	reg |= BIT6+BIT7; 
	vt1603_write(codec, VT1603_R71, reg);

	// left channel mixer disable
	reg = snd_soc_read(codec, VT1603_R72);
	reg &= ~BIT6;
	vt1603_write(codec, VT1603_R72, reg);

	// right channel mixer disable
	reg = snd_soc_read(codec, VT1603_R73);
	reg &= ~BIT6;
	vt1603_write(codec, VT1603_R73, reg);

	// reset PGA channels register
	reg = snd_soc_read(codec, VT1603_R8e);
	reg &= ~(BIT1+BIT0);
	vt1603_write(codec, VT1603_R8e, reg);

	// PGA channels disable
	reg = snd_soc_read(codec, VT1603_R66);
	reg &= ~(BIT2+BIT1);
	vt1603_write(codec, VT1603_R66, reg);

	// micboost power down, PGA power down
	vt1603_write(codec, VT1603_R67,0x02);

	//mdelay(30);
	
	return 0;
}

static int vt1603_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	DBG_DETAIL();
	
	del_timer_sync(&vt1603_timer);
	vt1603_suspend_prepare(codec);
	
	return 0;
}

static int vt1603_resume(struct snd_soc_codec *codec)
{
	int i;
	u16 *cache = codec->reg_cache;

	DBG_DETAIL();
	
	/*
	 * Resume from suspend, should reset the codec chip, otherwise
	 * record operation will cause codec working abnormally. 
	 */
	vt1603_reset(codec);
	
	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(vt1603_regs); i++) {
		if (i == VT1603_RESET)
			continue;
		snd_soc_write(codec, i, cache[i]);
	}

	mod_timer(&vt1603_timer, jiffies + POLL_PERIOD*HZ/1000);
	
	return 0;
}

static int vt1603_init(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret = 0;

	DBG_DETAIL();

	vt1603_conf.codec = codec;
	vt1603_conf.in_record = 0;

	if (vt1603_conf.headmic_det) {
		GPIO_PULL_CTRL_GPIO_23_20_BYTE_VAL |= BIT6; //GPIO22 enable PU
		GPIO_PULL_EN_GPIO_23_20_BYTE_VAL |= BIT3; //GPIO22 enable PU/PD
		GPIO_OC_GPIO_27_20_2BYTE_VAL &= ~BIT6; //set GPIO22 to GPIO input enable
		GPIO_CTRL_GPIO_23_20_BYTE_VAL |= BIT6; //enable GPIO22 to gpio mode
	}
	
	INIT_WORK(&vt1603_timer_work, vt1603_do_timer_work);
	INIT_WORK(&vt1603_hw_mute_work, vt1603_do_hw_mute_work);

	init_timer(&vt1603_timer);
	vt1603_timer.data = (unsigned long)codec;
	vt1603_timer.function = vt1603_timer_handler;
	mod_timer(&vt1603_timer, jiffies + POLL_PERIOD*HZ/1000);

	vt1603_reset(codec);
	vt1603_irq_init(codec);

	/* charge output caps */
	vt1603_set_bias_level(codec, SND_SOC_BIAS_PREPARE);
	codec->dapm.bias_level = SND_SOC_BIAS_STANDBY;  
	schedule_delayed_work(&codec->dapm.delayed_work, msecs_to_jiffies(1000));      
	
	vt1603_set_default_setting(codec);
	vt1603_set_default_route(codec);
	vt1603_set_default_volume(codec);
	
	vt1603_hw_mute(codec, 1);

	ret = snd_soc_add_controls(codec, vt1603_snd_controls,
			ARRAY_SIZE(vt1603_snd_controls));
	if (ret) {
		info("snd_soc_add_controls fail");
		goto vt1603_init_err;
	}

	ret = snd_soc_dapm_new_controls(dapm, vt1603_dapm_widgets,
			ARRAY_SIZE(vt1603_dapm_widgets));
	if (ret) {
		info("snd_soc_dapm_new_controls fail");
		goto vt1603_init_err;
	}

	ret = snd_soc_dapm_add_routes(dapm, audio_map,
			ARRAY_SIZE(audio_map));
	if (ret) {
		info("snd_soc_dapm_add_routes fail");
		goto vt1603_init_err;
	}

	return ret;
	
vt1603_init_err:
	del_timer_sync(&vt1603_timer);
	return ret;
}

static int vt1603_probe(struct snd_soc_codec *codec)
{
	struct vt1603_priv *vt1603 = snd_soc_codec_get_drvdata(codec);
	int ret ;
	
	pr_info("VT1603 Audio Codec %s\n", VT1603_VERSION);
	
	/*ret = snd_soc_codec_set_cache_io(codec, 7, 9, vt1603->control_type);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}*/

	if (vt1603_conf.bus_type == WMT_SND_SPI_BUS) { 
		codec->control_data = vt1603->spi;
		codec->hw_write = (hw_write_t)spi_write;
	}

	vt1603_soc_codec = codec;
	INIT_DELAYED_WORK(&codec->dapm.delayed_work, vt1603_do_set_bias_work);

	ret = vt1603_init(codec);
	if (ret < 0) {
		err(" failed to initialise VT1603");
	}

	return ret;
}

static int vt1603_remove(struct snd_soc_codec *codec)
{
	int ret;

	DBG_DETAIL();
	
	vt1603_set_bias_level(codec, SND_SOC_BIAS_OFF);
	
	/* forces any delayed work to be queued and run. */
	ret = cancel_delayed_work(&codec->dapm.delayed_work);
	/* if there was any work waiting then we run it now and
	 * wait for it's completion 
	 */
	if (ret) {
		schedule_delayed_work(&codec->dapm.delayed_work, 0);
		flush_scheduled_work();
	}
	
	del_timer_sync(&vt1603_timer);
	
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_vt1603 = {
	.probe = 	vt1603_probe,
	.remove = 	vt1603_remove,
	.suspend = 	vt1603_suspend,
	.resume =	vt1603_resume,
	.set_bias_level = vt1603_set_bias_level,
	.read = vt1603_codec_read,
	.write = vt1603_codec_write,
	.reg_cache_size = ARRAY_SIZE(vt1603_regs),
	.reg_word_size = sizeof(u16),
	.reg_cache_default = vt1603_regs,

	/*.controls = vt1603_snd_controls,
	.num_controls = ARRAY_SIZE(vt1603_snd_controls),
	.dapm_widgets = vt1603_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(vt1603_dapm_widgets),
	.dapm_routes = audio_map,
	.num_dapm_routes = ARRAY_SIZE(audio_map),*/
};

static int __devinit vt1603_spi_probe(struct spi_device *spi)
{
	struct vt1603_priv *vt1603;
	int ret = 0;

	DBG_DETAIL();

	vt1603 = kzalloc(sizeof(struct vt1603_priv), GFP_KERNEL);
	if (vt1603 == NULL) {
		info("vt1603_spi_probe kzalloc fail");
		return -ENOMEM;
	}

	spi_set_drvdata(spi, vt1603);
	vt1603->control_type = SND_SOC_SPI;
	vt1603->spi = spi;

	ret = snd_soc_register_codec(&spi->dev,
			&soc_codec_dev_vt1603, &vt1603_dai, 1);
	
	if (ret < 0) {
		kfree(vt1603);
		info("snd_soc_register_codec fail");
	}

	return ret;
}

static int __devexit vt1603_spi_remove(struct spi_device *spi)
{
	DBG_DETAIL();
	
	snd_soc_unregister_codec(&spi->dev);
	kfree(spi_get_drvdata(spi));
	
	return 0;
}

/* add by SparkSun for hardware independence */
static struct wmt_spi_slave vt1603_codec_info = {
	.dma_en        = SPI_DMA_DISABLE,
	.bits_per_word = 8,
};

static struct spi_board_info vt1603_codec_spi_bi = { 
	.modalias        = "vt1603",
	.controller_data = &vt1603_codec_info, /* vt1603 spi bus config info */
	.irq             = -1,                 /* use gpio irq               */
	.max_speed_hz    = 12000000,           /* same as spi master         */ 
	.bus_num         = 0,                  /* use spi master 0           */
	.mode            = SPI_CLK_MODE3,      /* phase1, polarity1          */ 
	.chip_select     = 0,                  /* as slave 0, CS=0           */
};

static int vt1603_codec_spi_init(void)
{
	struct spi_master *master = NULL;
	struct spi_device *slave  = NULL;

	DBG_DETAIL();
	
	master = spi_busnum_to_master(vt1603_codec_spi_bi.bus_num);
	if (!master)
		return -ENODEV;
	
	slave = spi_new_device(master, &vt1603_codec_spi_bi);
	if (!slave)
		return -EAGAIN;
	
	return 0;
}

static const struct spi_device_id vt1603_codec_spi_dev_ids[] = {
	{ "vt1603", 0},
	{ },
};
MODULE_DEVICE_TABLE(spi, vt1603_codec_spi_dev_ids);

static struct spi_driver vt1603_spi_driver = {
	.driver = {
		.name	= "vt1603",
		.owner	= THIS_MODULE,
	},
	.probe		= vt1603_spi_probe,
	.remove		= __devexit_p(vt1603_spi_remove),
	.id_table = vt1603_codec_spi_dev_ids,
};

static int __devinit vt1603_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct vt1603_priv *vt1603;
	int ret;

	DBG_DETAIL();

	vt1603 = kzalloc(sizeof(struct vt1603_priv), GFP_KERNEL);
	if (vt1603 == NULL) {
		info("vt1603_i2c_probe kzalloc fail");
		return -ENOMEM;
	}

	i2c_set_clientdata(i2c, vt1603);
	vt1603->control_type = SND_SOC_I2C;

	ret = snd_soc_register_codec(&i2c->dev,
			&soc_codec_dev_vt1603, &vt1603_dai, 1);
	
	if (ret < 0) {
		kfree(vt1603);
		info("snd_soc_register_codec fail");
	}
	
	return ret;
}

static int __devexit vt1603_i2c_remove(struct i2c_client *client)
{
 	DBG_DETAIL();
	
	snd_soc_unregister_codec(&client->dev);
	kfree(i2c_get_clientdata(client));
	
	return 0;
}

static const struct i2c_device_id vt1603_i2c_id[] = {
	{ "vt1603", 1 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, vt1603_i2c_id);

/* corgi i2c codec control layer */
static struct i2c_driver vt1603_i2c_driver = {
	.driver = {
		.name = "vt1603",
		.owner = THIS_MODULE,
	},
	.probe = vt1603_i2c_probe,
	.remove = __devexit_p(vt1603_i2c_remove),
	.id_table = vt1603_i2c_id,
};

static int vt1603_configure(void)
{
	int ret = 0;
	unsigned char buf[80];
	int varlen = sizeof(buf);
	unsigned int hpdetect =0;
	unsigned int channels =0;
	unsigned int bustype = 0;
	unsigned int recordsrc = 0;
	char codec_name[6];
	char *recsrc_name[7] = {"LINEIN_1", "MICIN_1", "DMIC_IN",
							"LINEIN_12", "LINEIN_2", "MICIN_2", "MICIN_12"}; 

	ret = wmt_getsyspara("wmt.audio.i2s", buf, &varlen);
	if (ret == 0) {
		sscanf(buf, "%6s", codec_name);
		
		if (strcmp(codec_name, "vt1603")) {
			info("vt1603 string not found");
			return -EINVAL;
		}
		
		sscanf(buf, "vt1603:%x:%x:%x:%x:%d", 
				&bustype, &channels, &recordsrc, &hpdetect, (int *)&vt1603_conf.volume_percent);
		info("wmt.audio.i2s = vt1603:%x:%x:%x:%x:%d",
				bustype, channels, recordsrc, hpdetect, vt1603_conf.volume_percent);
		
		if (hpdetect == 0xf1) {
			/* hp plug-in is low level */
			vt1603_conf.hp_plugin_level = 0;
		} 
		else if (hpdetect == 0xf2) {
			/* hp plug-in is high level */
			vt1603_conf.hp_plugin_level = 1;
		} 
		else {
			info("[hpdetect] invalid. set hp_plugin_level->low");
			vt1603_conf.hp_plugin_level = 0;
		}

		/* channel select */
		if (channels == 0xf1) {
			/* mono */
			vt1603_conf.stereo_or_mono = 1;
		} 
		else if (channels == 0xf2) {
			/* stereo */
			vt1603_conf.stereo_or_mono = 0;
		} 
		else {
			info("[mono/stereo] invalid. set channels->stereo");
			vt1603_conf.stereo_or_mono = 0;
		}

		/* bus type select */
		if (bustype == 0xf0) {
			vt1603_conf.bus_type = WMT_SND_I2C_BUS;
			vt1603_conf.bus_id = 0;
		} 
		else if (bustype == 0xf1) {
			vt1603_conf.bus_type = WMT_SND_I2C_BUS;
			vt1603_conf.bus_id = 1;
		}
		else if (bustype == 0xf2) {
			vt1603_conf.bus_type = WMT_SND_SPI_BUS;
			vt1603_conf.bus_id = 0;
		}
		else if (bustype == 0xf3) {
			vt1603_conf.bus_type = WMT_SND_I2C_BUS;
			vt1603_conf.bus_id = 2;
		}
		else {
			info("invalid bus type. set to SPI Bus0");
			vt1603_conf.bus_type = WMT_SND_SPI_BUS;
			vt1603_conf.bus_id = 0;
		}

		if (vt1603_conf.bus_type) {
			info("Bus_type=SPI-Bus, Bus_id=%d",	vt1603_conf.bus_id);
		}
		else {
			info("Bus_type=I2C-Bus, Bus_id=%d", vt1603_conf.bus_id);
		}

		/* record source select */
		if (recordsrc == 0xf0) {
			vt1603_conf.record_src = WMT_SND_LINEIN_1;
		} 
		else if (recordsrc == 0xf1) {
			vt1603_conf.record_src = WMT_SND_MICIN_1;
		}
		else if ((recordsrc == 0xf2) && (vt1603_conf.bus_type == WMT_SND_I2C_BUS)) {
			/* record via digital mic */
			vt1603_conf.record_src = WMT_SND_DMIC_IN;
		}
		else if (recordsrc == 0xf3) {
			vt1603_conf.record_src = WMT_SND_LINEIN_12;
		}
		else if (recordsrc == 0xf4) {
			vt1603_conf.record_src = WMT_SND_LINEIN_2;
		}
		else if (recordsrc == 0xf5) {
			vt1603_conf.record_src = WMT_SND_MICIN_2;
		}
		else if (recordsrc == 0xf6) {
			vt1603_conf.record_src = WMT_SND_MICIN_12;
		}
		else {
			info("[record source] invalid. set to MICIN_1");
			vt1603_conf.record_src = WMT_SND_MICIN_1;
		}

		info("Record_src=%s", recsrc_name[vt1603_conf.record_src]);
	} 
	else {
		info(" wmt.audio.i2s not set");
		info(" set default->hp-plugin-low-level/stereo/spi-bus-0/a-mic");
		vt1603_conf.hp_plugin_level = 0;
		vt1603_conf.stereo_or_mono = 0;
		vt1603_conf.bus_type = WMT_SND_SPI_BUS;
		vt1603_conf.bus_id = 0;
		vt1603_conf.record_src = WMT_SND_MICIN_1;
	}
	
	if (vt1603_conf.volume_percent > 100)
		vt1603_conf.volume_percent = 100;
	info("playback volume percent[%d%%]", 
			vt1603_conf.volume_percent);

	// get debug parameter
	memset(buf, 0x0, sizeof(buf));
	varlen = sizeof(buf);
	ret = wmt_getsyspara("wmt.vt1603.debug", buf, &varlen);
	if (ret == 0) {
		sscanf(buf, "%d", &vt1603_dbg_flag);
	}

	// has fm34 for echo cancellation?
	memset(buf, 0x0, sizeof(buf));
	varlen = sizeof(buf);
	ret = wmt_getsyspara("wmt.audio.fm34", buf, &varlen);
	if (ret == 0) {
		sscanf(buf, "%d:%d", &fm34_enable, &fm34_i2c_bus);
		if (fm34_enable != 0)
			fm34_enable = 1;
		if (fm34_i2c_bus != 0)
			fm34_i2c_bus = 1;
	}

	// get customization parameter
	memset(buf, 0x0, sizeof(buf));
	varlen = sizeof(buf);
	ret = wmt_getsyspara("wmt.audio.custm", buf, &varlen);
	if (ret == 0) {
		sscanf(buf, "%d:%d", (int *)&vt1603_conf.headmic_det,
			(int *)&vt1603_conf.classD_gain);
		if (vt1603_conf.headmic_det != 0)
			vt1603_conf.headmic_det = 1;
		if ((vt1603_conf.classD_gain < 0) || (vt1603_conf.classD_gain > 7))
			vt1603_conf.classD_gain = 0xFF;
	}
	else {
		vt1603_conf.headmic_det = 0;
		vt1603_conf.classD_gain = 0xFF;
	}

	/*info("headmic_det=%d, classD_gain=%d",
		vt1603_conf.headmic_det, vt1603_conf.classD_gain);*/

	return 0;
}

static struct i2c_board_info __initdata wmt_vt1603_board_info[] = {
	{
		I2C_BOARD_INFO("vt1603", 0x1a),
	},
};

static int __init vt1603_modinit(void)
{
	int ret = 0;

	DBG_DETAIL();

	if (vt1603_configure())
		return -ENODEV;
	
	if (vt1603_conf.bus_type == WMT_SND_SPI_BUS) { 
		ret = vt1603_codec_spi_init();
		if (ret) {
			err("VT1603 codec add spi device failed, err %d", ret);
			return ret;
		}
	
		ret = spi_register_driver(&vt1603_spi_driver);
		if (ret != 0) {
			err("can't add spi driver");
		}
	}
	else {
		vt1603_conf.i2c_adap = i2c_get_adapter(vt1603_conf.bus_id);
		if (vt1603_conf.i2c_adap == NULL) {
			err("can not get i2c adapter, client address error");
			return -ENODEV;
		}
	
		if (i2c_new_device(vt1603_conf.i2c_adap, wmt_vt1603_board_info) == NULL) {
			err("allocate i2c client failed");
			return -ENOMEM;
		}
	
		i2c_put_adapter(vt1603_conf.i2c_adap);

		ret = i2c_add_driver(&vt1603_i2c_driver);
		if (ret)
			info("i2c_add_driver fail");
	}
	
	return ret;
}
module_init(vt1603_modinit);

static void __exit vt1603_exit(void)
{
	DBG_DETAIL();
	
	if (vt1603_conf.bus_type == WMT_SND_SPI_BUS) { 
		spi_unregister_driver(&vt1603_spi_driver);
	}
	else {
		i2c_del_driver(&vt1603_i2c_driver);
	}
}
module_exit(vt1603_exit);

MODULE_DESCRIPTION("WMT [ALSA SoC] driver");
MODULE_AUTHOR("WonderMedia Technologies, Inc.");
MODULE_LICENSE("GPL");
