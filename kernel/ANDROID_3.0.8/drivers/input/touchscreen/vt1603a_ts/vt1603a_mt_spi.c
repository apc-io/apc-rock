/*++
	Copyright (c) 2008  WonderMedia Technologies, Inc.

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

    History: 2011.Jan.21st, version: 1.00, single touch
    History: 2011.Mar.28th, version: 2.00, add dual touch interface, battery alarm support
    History: 2011.Mar.31st, version: 2.01, add wm8710 support
    History: 2011.Jun.7th, version: 2.02, modify calibration interface

--*/

#include <linux/init.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/input.h>
#include <linux/random.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <asm/uaccess.h>
#include <mach/hardware.h>
#include <mach/wmt_gpio.h>
#include <linux/version.h>
#include <linux/spi/spi.h>
#include <mach/wmt-spi.h>
#include "vt1603a_mt.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend vt1603_early_suspend;

static void vt1603_ts_early_suspend(struct early_suspend *h);
static void vt1603_ts_late_resume(struct early_suspend *h);
static unsigned early_suspend_stage;
#endif
 
struct wmt_gpt_s {
    char name[10];
    unsigned int bitmap;
    unsigned int ctraddr;
    unsigned int ocaddr;
    unsigned int idaddr;
    unsigned int peaddr;
    unsigned int pcaddr;
    unsigned int itbmp;
    unsigned int itaddr;
    unsigned int isbmp;
    unsigned int isaddr;
    unsigned int irq;
};

static struct wmt_gpt_s wmt_ts_gpt = {
    .name = "vt1603",
    .bitmap = 0x04,/*GPIO 2*/
    .ctraddr = 0xd8110040 + WMT_MMAP_OFFSET,
    .ocaddr = 0xd8110080 + WMT_MMAP_OFFSET,
    .idaddr = 0xd8110000 + WMT_MMAP_OFFSET,
    .peaddr = 0xd8110480 + WMT_MMAP_OFFSET,
    .pcaddr = 0xd81104c0 + WMT_MMAP_OFFSET,
    .itbmp = 0x00800000, /* low level */
    .itaddr = 0xd8110300 + WMT_MMAP_OFFSET,
    .isbmp = 0x4,
    .isaddr = 0xd8110360 + WMT_MMAP_OFFSET,
    .irq = 5,
};

//#define DEBUG
#ifdef  DEBUG
static int dbg_mask = 0;
module_param(dbg_mask, int, S_IRUGO | S_IWUSR);
#define ts_dbg(fmt, args...) \
        do {\
            if (dbg_mask) \
                printk(KERN_ERR "[%s]_%d: " fmt, __func__ , __LINE__, ## args);\
        } while(0)
#define ts_trace() \
        do {\
            if (dbg_mask) \
                printk(KERN_ERR "trace in %s %d\n", __func__, __LINE__);\
        } while(0)
#else
#define ts_dbg(fmt, args...)
#define ts_trace()
#endif

static void parse_gpt_arg(void)
{
	char *varname = "wmt.gpt.ts";
	unsigned int varlen = 160;
	unsigned char buf[160];
	int retval;
	int i = 0;
	retval = wmt_getsyspara(varname, buf, &varlen);
	if (retval != 0)
		return;
	if (buf[i] != 'v')
		return;
	++i;
	if (buf[i] != 't')
		return;
	++i;
	if (buf[i] != '1')
		return;
	++i;
	if (buf[i] != '6')
		return;
	++i;
	if (buf[i] != '0')
		return;
	++i;
	if (buf[i] != '3' && buf[i] != '9')
		return;
	i += 2;
	sscanf((buf + i), "%x:%x:%x:%x:%x:%x:%x:%x:%x:%x:%x",
			&wmt_ts_gpt.bitmap,
			&wmt_ts_gpt.ctraddr,
			&wmt_ts_gpt.ocaddr,
			&wmt_ts_gpt.idaddr,
			&wmt_ts_gpt.peaddr,
			&wmt_ts_gpt.pcaddr,
			&wmt_ts_gpt.itbmp,
			&wmt_ts_gpt.itaddr,
			&wmt_ts_gpt.isbmp,
			&wmt_ts_gpt.isaddr,
			&wmt_ts_gpt.irq);

	wmt_ts_gpt.ctraddr += WMT_MMAP_OFFSET;
	wmt_ts_gpt.ocaddr += WMT_MMAP_OFFSET;
	wmt_ts_gpt.idaddr += WMT_MMAP_OFFSET;
	wmt_ts_gpt.peaddr += WMT_MMAP_OFFSET;
	wmt_ts_gpt.pcaddr += WMT_MMAP_OFFSET;
	wmt_ts_gpt.itaddr += WMT_MMAP_OFFSET;
	wmt_ts_gpt.isaddr += WMT_MMAP_OFFSET;
	/*
	printk("wmt.gpt.ts = %x:%x:%x:%x:%x:%x:%x:%x:%x:%x:%x\n",
		wmt_ts_gpt.bitmap,
		wmt_ts_gpt.ctraddr,
		wmt_ts_gpt.ocaddr,
		wmt_ts_gpt.idaddr,
		wmt_ts_gpt.peaddr,
		wmt_ts_gpt.pcaddr,
		wmt_ts_gpt.itbmp,
		wmt_ts_gpt.itaddr,
		wmt_ts_gpt.isbmp,
		wmt_ts_gpt.isaddr,
		wmt_ts_gpt.irq);
	*/
}
/* 
 * vt1603_ts_spi_fix_cs - fix vt1603 spi chipselect, this function MUST
 *    be invoked before seting/getting vt1603's register via spi bus
 * @spi: proxy for vt1603 (as spi slave)
 */
static inline void vt1603_ts_spi_fix_cs(struct spi_device *spi)
{
    spi->chip_select = VT1603_SPI_FIX_CS;
}

/*
 * vt1603_ts_spi_fake_cs - fake vt1603 spi chipselect, this function MUST
 *    be invoked after setting/getting vt1603's register via spi bus
 * @spi: proxy for vt1603 (as spi slave)
 */
static inline void vt1603_ts_spi_fake_cs(struct spi_device *spi)
{
    spi->chip_select = VT1603_SPI_FAKE_CS;
}

/*
 * vt1603_ts_spi_write - write a u8 data to vt1603 via spi bus
 * @spi: proxy for vt1603 (as spi slave)
 * @addr: vt1603 register address
 * @data: data write to register
 */
static int vt1603_ts_spi_reg_op(struct spi_device *spi,
                                const u8 *tx, u8 *rx, int op_rw)
{
    struct spi_message m;
    struct spi_transfer t;

    spi_message_init(&m);
    memset(&t, 0x00, sizeof(t));
    if (VT1603_REG_OP_W == op_rw)
        t.len = 3;
    else
        t.len = 5;
    spi_message_add_tail(&t, &m);
    t.tx_buf = tx;
    t.rx_buf = rx;

    return spi_sync(spi, &m);
}

static int vt1603_ts_spi_write(struct spi_device *spi, u8 addr, const u8 *data)
{
    int ret = 0;
    u8 w_cmd[3] = {0};

    w_cmd[0] = ((addr & 0xFF) | BIT7); 
    w_cmd[1] = ((addr & 0xFF) >> 7);
    w_cmd[2] = data[0];
    vt1603_ts_spi_fix_cs(spi);
    ret = vt1603_ts_spi_reg_op(spi, w_cmd, NULL, VT1603_REG_OP_W);
    if (ret != 0)
        ts_dbg("ts spi write failed, err:%d\n", ret);
    vt1603_ts_spi_fake_cs(spi);
    return ret;
}

/*
 * vt1603_ts_spi_read - read a u8 data from vt1603 via spi bus
 * @spi: proxy for vt1603 (as spi slave)
 * @addr: vt1603 register address
 * @data: data read from vt1603
 */
static int vt1603_ts_spi_read(struct spi_device *spi, u8 addr, u8 *data)
{
    int ret;
    u8 r_cmd[IDLE_DATA_NUM] = {0};
    u8 r_buf[IDLE_DATA_NUM] = {0};

    r_cmd[0] = ((addr & 0xFF) & (~BIT7)); 
    r_cmd[1] = ((addr & 0xFF) >> 7);
    vt1603_ts_spi_fix_cs(spi);
    ret = vt1603_ts_spi_reg_op(spi, r_cmd, r_buf, VT1603_REG_OP_R);
    if (ret != 0)
        ts_dbg("ts spi read failed, err:%d\n", ret);
    data[0] = r_buf[IDLE_DATA_NUM - 1];
    vt1603_ts_spi_fake_cs(spi);
    return ret;
}

/*
 * vt1603_set_reg8 - set register value of vt1603
 * @ts_drv: vt1603 driver data
 * @reg: vt1603 register address
 * @val: value register will be set
 */
int vt1603_set_reg8(struct vt1603_ts_drvdata *ts_drv, u8 reg, u8 val)
{
    return vt1603_ts_spi_write(ts_drv->spi, reg, &val);
}
EXPORT_SYMBOL_GPL(vt1603_set_reg8);

/*
 * vt1603_get_reg8 - get register value of vt1603
 * @ts_drv: vt1603 driver data
 * @reg: vt1603 register address
 */
u8 vt1603_get_reg8(struct vt1603_ts_drvdata *ts_drv, u8 reg)
{
    u8  val = 0;
    int ret = 0;

    ret = vt1603_ts_spi_read(ts_drv->spi, reg, &val);
    if (ret)
        ts_dbg("vt1603 read register failed with error:%d\n", ret);
    return val;
}
EXPORT_SYMBOL_GPL(vt1603_get_reg8);

/*
 * vt1603_spi_device_dump - dubug function, for dump spi device status
 * @spi: spi device (slave)
 */
static void vt1603_spi_device_dump(struct spi_device * spi_dev)
{
    ts_dbg("spi_device: modalias %s\n", spi_dev->modalias);
    ts_dbg("spi_device: bus_num %d\n", spi_dev->master->bus_num);    
    ts_dbg("spi_device: max_speed_hz %d\n", spi_dev->max_speed_hz);
    ts_dbg("spi_device: chip_select %d\n", spi_dev->chip_select);
    ts_dbg("spi_device: mode 0x%x\n", spi_dev->mode);
    ts_dbg("spi_device: bits_per_word %d\n", spi_dev->bits_per_word);
    ts_dbg("spi_device: irq %d\n", spi_dev->irq);
}

/*
 * vt1603_reg_dump - dubug function, for dump vt1603 related registers
 * @ts_drv: vt1603 driver data
 */
static void vt1603_reg_dump(struct vt1603_ts_drvdata *ts_drv, u8 addr, int len)
{
    u8 i;
    for (i = addr; i < addr + len; i += 2)
        ts_dbg("reg[%d]:0x%02X,  reg[%d]:0x%02X\n", 
                i, vt1603_get_reg8(ts_drv, i), 
                i + 1, vt1603_get_reg8(ts_drv, i + 1));
}

/*
 * vt1603_setbits - write bit1 to related register's bit
 * @ts_drv: vt1603 driver data
 * @reg: vt1603 register address
 * @mask: bit setting mask
 */
void vt1603_setbits(struct vt1603_ts_drvdata *ts_drv, u8 reg, u8 mask)
{
    u8 tmp = vt1603_get_reg8(ts_drv, reg) | mask;
    vt1603_set_reg8(ts_drv, reg, tmp);
}
EXPORT_SYMBOL_GPL(vt1603_setbits);

/*
 * vt1603_clrbits - write bit0 to related register's bit
 * @ts_drv: vt1603 driver data
 * @reg: vt1603 register address
 * @mask:bit setting mask
 */
void vt1603_clrbits(struct vt1603_ts_drvdata *ts_drv, u8 reg, u8 mask)
{
    u8 tmp = vt1603_get_reg8(ts_drv, reg) & (~mask);
    vt1603_set_reg8(ts_drv, reg, tmp);
}
EXPORT_SYMBOL_GPL(vt1603_clrbits);

/*
 * vt1603_clr_ts_irq -  clear touch panel pen down/up and 
 *    conversion end/timeout interrupts
 * @ts_drv: vt1603 driver data
 * @mask: which interrupt will be cleared
 */
int vt1603_clr_ts_irq(struct vt1603_ts_drvdata *ts_drv, u8 mask)
{
    vt1603_setbits(ts_drv, VT1603_INTS_REG, mask);
    return 0;
}
EXPORT_SYMBOL_GPL(vt1603_clr_ts_irq);

static int soc_gpio_num_to_irq_num(struct vt1603_ts_drvdata *ts_drv)
{
    return wmt_ts_gpt.irq; 
}

static void soc_gpio_irq_disable(struct vt1603_ts_drvdata *ts_drv)
{
    if (ts_drv->gpio_num < 0)
        return ;

    if (wmt_ts_gpt.itbmp & 0xff)
            REG32_VAL(wmt_ts_gpt.itaddr) &= ~0x80;
    else if (wmt_ts_gpt.itbmp & 0xff00)
            REG32_VAL(wmt_ts_gpt.itaddr) &= ~0x8000;
    else if (wmt_ts_gpt.itbmp & 0xff0000)
            REG32_VAL(wmt_ts_gpt.itaddr) &= ~0x800000;
    else if (wmt_ts_gpt.itbmp & 0xff000000)
            REG32_VAL(wmt_ts_gpt.itaddr) &= ~0x80000000;
}

static void soc_gpio_irq_enable(struct vt1603_ts_drvdata *ts_drv)
{
    if (ts_drv->gpio_num < 0)
        return ;

#ifdef CONFIG_HAS_EARLYSUSPEND
    if (early_suspend_stage == 0) {
#endif
        if (wmt_ts_gpt.itbmp & 0xff)
                REG32_VAL(wmt_ts_gpt.itaddr) |= 0x80;
        else if (wmt_ts_gpt.itbmp & 0xff00)
                REG32_VAL(wmt_ts_gpt.itaddr) |= 0x8000;
        else if (wmt_ts_gpt.itbmp & 0xff0000)
                REG32_VAL(wmt_ts_gpt.itaddr) |= 0x800000;
        else if (wmt_ts_gpt.itbmp & 0xff000000)
                REG32_VAL(wmt_ts_gpt.itaddr) |= 0x80000000;
#ifdef CONFIG_HAS_EARLYSUSPEND
    }
#endif
}

static void soc_gpio_irq_clear(struct vt1603_ts_drvdata *ts_drv)
{
    if (ts_drv->gpio_num < 0)
        return ;

    REG32_VAL(wmt_ts_gpt.isaddr) = wmt_ts_gpt.isbmp; //clear interrupt status
    return;
}

#define SET_GPIO_TS_INT() {\
	REG32_VAL(wmt_ts_gpt.ctraddr) |= wmt_ts_gpt.bitmap; \
	REG32_VAL(wmt_ts_gpt.ocaddr) &= ~wmt_ts_gpt.bitmap; \
	REG32_VAL(wmt_ts_gpt.peaddr) |= ~wmt_ts_gpt.bitmap; \
	REG32_VAL(wmt_ts_gpt.pcaddr) |= wmt_ts_gpt.bitmap; \
	REG32_VAL(wmt_ts_gpt.isaddr) = wmt_ts_gpt.isbmp; \
}

static void soc_gpio_irq_init(struct vt1603_ts_drvdata *ts_drv)
{
    if (ts_drv->gpio_num < 0)
        return ;

    if (wmt_ts_gpt.itbmp & 0xff) {
            REG32_VAL(wmt_ts_gpt.itaddr) &= ~0xff;
            REG32_VAL(wmt_ts_gpt.itaddr) |= (wmt_ts_gpt.itbmp & ~BIT7);
    } else if (wmt_ts_gpt.itbmp & 0xff00) {
            REG32_VAL(wmt_ts_gpt.itaddr) &= ~0xff00;
            REG32_VAL(wmt_ts_gpt.itaddr) |= (wmt_ts_gpt.itbmp & ~BIT15);
    } else if (wmt_ts_gpt.itbmp & 0xff0000) {
            REG32_VAL(wmt_ts_gpt.itaddr) &= ~0xff0000;
            REG32_VAL(wmt_ts_gpt.itaddr) |= (wmt_ts_gpt.itbmp & ~BIT23);
    } else if (wmt_ts_gpt.itbmp & 0xff000000) {
            REG32_VAL(wmt_ts_gpt.itaddr) &= ~0xff000000;
            REG32_VAL(wmt_ts_gpt.itaddr) |= (wmt_ts_gpt.itbmp & ~BIT31);
    }
    SET_GPIO_TS_INT();
    return;
}

static int soc_check_int_status_gpio(void)
{
	return (REG32_VAL(wmt_ts_gpt.isaddr) & (wmt_ts_gpt.isbmp)); // interrupt status
}

static int soc_check_int_enable_gpio(void)
{
    if (wmt_ts_gpt.itbmp & 0xff)
            return REG32_VAL(wmt_ts_gpt.itaddr) & 0x80;
    else if (wmt_ts_gpt.itbmp & 0xff00)
            return REG32_VAL(wmt_ts_gpt.itaddr) & 0x8000;
    else if (wmt_ts_gpt.itbmp & 0xff0000)
            return REG32_VAL(wmt_ts_gpt.itaddr) & 0x800000;
    else if (wmt_ts_gpt.itbmp & 0xff000000)
            return REG32_VAL(wmt_ts_gpt.itaddr) & 0x80000000;
    else
            return 0;
}


static void vt1603_ts_soc_gpio_handle_init(struct vt1603_ts_drvdata * ts_drv)
{
    ts_drv->soc_gpio_irq_init    = soc_gpio_irq_init;
    ts_drv->soc_gpio_irq_enable  = soc_gpio_irq_enable;
    ts_drv->soc_gpio_irq_disable = soc_gpio_irq_disable;
    ts_drv->soc_gpio_irq_clear   = soc_gpio_irq_clear;
}

/*
 * vt1603_ts_clk_enable - open mclk for vt1603 touch-panel 
 *    and sar-adc module
 */
static void vt1603_ts_clk_enable(void)
{
    return ;
}

/*
 * vt1603_gpio1_reset - vt1603 gpio1 configuration, 
 *  gpio1 as interrupt output, low active
 * @ts_drvdata_addr: address of vt1603 driver data
 */
static void vt1603_gpio1_reset(struct vt1603_ts_drvdata *ts_drv)
{
#ifndef MODULE
    /* mask other module interrupts      */
    vt1603_set_reg8(ts_drv, VT1603_IMASK_REG27, 0xFD);
    vt1603_set_reg8(ts_drv, VT1603_IMASK_REG28, 0xFF);
    vt1603_set_reg8(ts_drv, VT1603_IMASK_REG29, 0xFF);
    /* irq output form gpio1 high active */
    if (ts_drv->pdata->irq_type == HIGH_ACTIVE
        || ts_drv->pdata->irq_type == RISING_EDGE_ACTIVE)
        vt1603_set_reg8(ts_drv, VT1603_IPOL_REG33, 0xDF);
    else
        vt1603_set_reg8(ts_drv, VT1603_IPOL_REG33, 0xFF);
    /* vt1603 gpio1 as IRQ output        */
    vt1603_set_reg8(ts_drv, VT1603_ISEL_REG36, 0x04);
#endif
    return ;
}

static unsigned int is_vt1609 = 0;
vt1603_dual_touch_support_fn vt1603_ts_dual = NULL;
EXPORT_SYMBOL_GPL(vt1603_ts_dual);
/*
 * vt1603_ts_report_penup - report touch panel pen up event to
 *    kernel input system
 * @ts_drv: vt1603 driver data
 */
void vt1603_ts_report_penup(struct vt1603_ts_drvdata *ts_drv)
{
    if (ts_drv->ts_state == TS_PENUP_STATE)
        return ;

    input_report_key(ts_drv->input, BTN_TOUCH, 0);
    //input_report_abs(ts_drv->input, ABS_MT_TOUCH_MAJOR, 0);
    input_mt_sync(ts_drv->input);
    input_sync(ts_drv->input);
}
EXPORT_SYMBOL_GPL(vt1603_ts_report_penup);

/*
 * vt1603_ts_report_pos - report touch panel touched position to
 *     kernel input system
 * @ts_drv: vt1603 driver data
 * @pos: vt1603 touch panel touched point conversion data
 */

static int panelres_x = 800;
static int panelres_y = 480;

static int g_bCalibrating = false;
static struct vt1603_ts_cal_info g_CalcParam;
static int vt1603_ts_get_resolvY(void)
{
	return panelres_y;
}

static int vt1603_ts_get_resolvX(void)
{
	return panelres_x;
}

static int vt1603_ts_is_calibration(void)
{
    return g_bCalibrating;
}

int vt1603_ts_pos_calibration(struct vt1603_ts_pos *to_cal)
{
	int x, y, delta;

    if (g_CalcParam.delta == 0)
        delta = 1;
    else
        delta = g_CalcParam.delta;

    x = (g_CalcParam.a1 * to_cal->xpos + g_CalcParam.b1 * to_cal->ypos +
         g_CalcParam.c1) / delta;
    y = (g_CalcParam.a2 * to_cal->xpos + g_CalcParam.b2 * to_cal->ypos +
         g_CalcParam.c2) / delta;

    /* pos check */
    if (x < 0)
        x = 0;
    if (y < 0)
        y = 0;
    if (x > vt1603_ts_get_resolvX())
        x = vt1603_ts_get_resolvX();
    if (y > vt1603_ts_get_resolvY())
        y = vt1603_ts_get_resolvY();

    to_cal->xpos = x;
    to_cal->ypos = y;
	return 0;
}
EXPORT_SYMBOL_GPL(vt1603_ts_pos_calibration);

static struct vt1603_ts_event g_evLast;
static void vt1603_ts_set_rawdata(struct vt1603_ts_pos *pos)
{
    g_evLast.x = pos->xpos;
    g_evLast.y = pos->ypos;
}

void vt1603_ts_report_pos(struct vt1603_ts_drvdata *ts_drv,
                                 struct vt1603_ts_pos *pos)
{
    struct vt1603_ts_pos p = *pos;

    if (unlikely(vt1603_get_reg8(ts_drv, VT1603_INTS_REG) & BIT4)) {
        ts_dbg("vt1603 touch already pen up when report pos!\n");
        return ;
    }
    if (vt1603_ts_is_calibration()) {
        vt1603_ts_set_rawdata(&p);
        p.xpos = vt1603_ts_get_resolvX() / 2;
        p.ypos = vt1603_ts_get_resolvY() / 2;
    } else {
        vt1603_ts_pos_calibration(&p);
    }
    ts_dbg("pos caled:x-pos[%d], y-pos[%d]\n", p.xpos, p.ypos);
    input_report_key(ts_drv->input, BTN_TOUCH, 1);
    input_report_abs(ts_drv->input, ABS_MT_POSITION_X, p.xpos);
    input_report_abs(ts_drv->input, ABS_MT_POSITION_Y, p.ypos);
    //input_report_abs(ts_drv->input, ABS_MT_TOUCH_MAJOR, 15);
    input_mt_sync(ts_drv->input);
    input_sync(ts_drv->input);
}
EXPORT_SYMBOL_GPL(vt1603_ts_report_pos);

/*
 * vt1603_ts_get_pos - get touch panel touched position from vt1603
 *     conversion register
 * @ts_drv: vt1603 driver data
 * @pos: vt1603 touch panel touched point conversion data
 */
void
vt1603_ts_get_pos(struct vt1603_ts_drvdata *ts_drv, struct vt1603_ts_pos *pos)
{
    u8 datal, datah;

    /* get x-position */
    datal = vt1603_get_reg8(ts_drv, VT1603_XPL_REG);
    datah = vt1603_get_reg8(ts_drv, VT1603_XPH_REG);
    pos->xpos = ADC_DATA(datal, datah);
    /* get y-positin */
    datal = vt1603_get_reg8(ts_drv, VT1603_YPL_REG);
    datah = vt1603_get_reg8(ts_drv, VT1603_YPH_REG);
    pos->ypos = ADC_DATA(datal, datah);
}
EXPORT_SYMBOL_GPL(vt1603_ts_get_pos);

/*
 * vt1603_ts_work - vt1603 touch-panel/sar-adc interrupt isr bottom half,
 *    all interrupts will processed here
 *
 *  TODO: We don't do battery/temperature detect in ts_work any more, so if 
 *  received a battery/temperature related irq in ts_work, that means
 *  something is wrong. Please invoke vt1603_work_mode_switch to switch
 *  vt1603 into touch-mode to resolve this issue.
 *  TODO: we just enable pendown and timeout irq generate
 * @work: vt1603 driver work struct
 */
static void vt1603_ts_work(struct work_struct *work)
{
    u8 int_sts;
    unsigned long flags;
    struct vt1603_ts_drvdata *ts_drv;
    int delay_ms = 0;

    ts_drv = container_of(work, struct vt1603_ts_drvdata, work);
    spin_lock_irqsave(&ts_drv->spinlock, flags);

    ts_dbg("Enter\n");
    int_sts = vt1603_get_reg8(ts_drv, VT1603_INTS_REG);
    ts_dbg("int_sts 0x%02x\n", int_sts);

    /* this interrupt not belongs to sar-adc */
    if (unlikely(((int_sts & 0x0F) == 0) && ((int_sts & BIT4) != 0))) {
        ts_dbg("vt1603 other module interrupt?\n");
        goto out;
    }
    /* timeout interrupt, reset sar-adc     */
    if (unlikely((int_sts & BIT3) != 0)) {
        ts_dbg("vt1603 ts timeout irq, hardware reset now!\n");
        vt1603_ts_report_penup(ts_drv);
        ts_drv->ts_state = TS_PENUP_STATE;
        if (ts_drv->ts_type == PANEL_TYPE_4WIRED)
            vt1603_set_reg8(ts_drv, VT1603_CR_REG, BIT1);
        else
            vt1603_set_reg8(ts_drv, VT1603_CR_REG, BIT1 | BIT0);
        goto out;
    }
    /* already pendown, but received pendown event again */
    if (((int_sts & BIT4) == 0) && (ts_drv->ts_state == TS_PENDOWN_STATE)) {
        delay_ms = ts_drv->ts_delay;
        ts_dbg("vt1603 ts already pendown!, why pendown again?\n");
    }
    /* already penup, but received penup event again     */
    if (((int_sts & BIT4) != 0) && (ts_drv->ts_state == TS_PENUP_STATE)) {
        delay_ms = ts_drv->ts_delay;
        ts_dbg("vt1603 ts already penup!, why penup again?\n");
    }
#ifdef CONFIG_HAS_EARLYSUSPEND
    if (early_suspend_stage == 0)
#endif
        /* handle touch event in workqueue                   */
        queue_delayed_work(ts_drv->workqueue, &ts_drv->read_work,
                            msecs_to_jiffies(delay_ms));
    
out:
    vt1603_clr_ts_irq(ts_drv, int_sts & 0x0F);
    ts_drv->soc_gpio_irq_clear(ts_drv);
    spin_unlock_irqrestore(&ts_drv->spinlock, flags);

    ts_dbg("Exit\n\n\n");
    ts_drv->soc_gpio_irq_enable(ts_drv);
    return ;
}

static void pos_buf_clean(struct pos_buf *p_buf)
{
    memset(p_buf, 0x00, sizeof(*p_buf));
}

static void pos_buf_fill(struct pos_buf *p_buf, struct vt1603_ts_pos *pos)
{
    p_buf->pos_fifo[p_buf->len % AVG_POS_BUF_LEN].xpos = pos->xpos;
    p_buf->pos_fifo[p_buf->len % AVG_POS_BUF_LEN].ypos = pos->ypos;
    p_buf->len++;
}

static void pos_buf_avg(struct pos_buf *p_buf, struct vt1603_ts_pos *pos)
{
    int i, num;
    u32 avg_x = 0;
    u32 avg_y = 0;

    if (p_buf->len < AVG_POS_BUF_LEN)
        num = p_buf->len;
    else
        num = AVG_POS_BUF_LEN;

    for (i = 0; i < num; i++) {
        avg_x += p_buf->pos_fifo[i].xpos;
        avg_y += p_buf->pos_fifo[i].ypos;
    }

    pos->xpos = (u16)(avg_x / num);
    pos->ypos = (u16)(avg_y / num);

    return ;
}

struct vt1603_ts_pos pre_pos = {0, 0};
EXPORT_SYMBOL_GPL(pre_pos);
struct pos_buf pos_buff = {.len = 0};
EXPORT_SYMBOL_GPL(pos_buff);
static void vt1603_ts_pure_single_read_loop(struct vt1603_ts_drvdata *ts_drv)
{
    unsigned long flags = 0;
    struct vt1603_ts_pos pos;
    u8 int_sts;

    ts_dbg("Enter\n");
    spin_lock_irqsave(&ts_drv->spinlock, flags);
    if (ts_drv->mode != VT1603_TS_MODE) {
        printk("vt1603 not touch mode now?\n");
        goto next_loop;
    }

    int_sts = vt1603_get_reg8(ts_drv, VT1603_INTS_REG);
    ts_dbg("int_sts:0x%02x\n", int_sts);
    /* already penup now */
    if (int_sts & BIT4) {
        if (jiffies_to_msecs(jiffies - ts_drv->ts_stamp) < TS_DEBOUNCE) {
            ts_dbg("vt1603 ts debouncing?...\n");
            vt1603_clr_ts_irq(ts_drv, int_sts & 0x0F);
            goto next_loop;
        }
        ts_dbg("vt1603 ts penup now!\n");
        vt1603_clr_ts_irq(ts_drv, int_sts & 0x0F);
        vt1603_ts_report_penup(ts_drv);
        ts_drv->ts_state = TS_PENUP_STATE;
        ts_drv->pos_num = 0;
        pre_pos.xpos = 0;
        pre_pos.ypos = 0;
        pos_buf_clean(&pos_buff);
        goto out;
    }
    /* read and report pos if conversion end */
    if ((int_sts & BIT0) == 0) {
        ts_dbg("no conversion end?!\n");
        goto next_loop;
    }
    ts_drv->pos_num++;
    if (ts_drv->pos_num <= VT1603_FILTER_FIRST_N_POS) {
        ts_dbg("first pos, do not report it!\n");
        vt1603_ts_get_pos(ts_drv, &pos);
        ts_dbg("filter: pos.x = %d, pos.y = %d\n", pos.xpos, pos.ypos);
        vt1603_clr_ts_irq(ts_drv, int_sts & 0x0F);
        goto next_loop;
    }
    vt1603_ts_get_pos(ts_drv, &pos);
    ts_dbg("org: pos.x = %d, pos.y = %d\n", pos.xpos, pos.ypos);

    if (((pre_pos.xpos != 0) && (pre_pos.ypos != 0)) &&
           ((abs(pre_pos.xpos - pos.xpos) > VT1603_JITTER_THRESHOLD)
        || (abs(pre_pos.ypos - pos.ypos) > VT1603_JITTER_THRESHOLD))) {
        ts_dbg("jitter pos, touchscreen pressure unstable?\n");
        vt1603_clr_ts_irq(ts_drv, int_sts & 0x0F);
        goto next_loop;
    }
    ts_drv->ts_state = TS_PENDOWN_STATE;
    ts_drv->ts_stamp = jiffies;
    pos_buf_fill(&pos_buff, &pos);
    pos_buf_avg(&pos_buff, &pos);
    pre_pos.xpos = pos.xpos;
    pre_pos.ypos = pos.ypos;
    ts_dbg("avg: pos.x = %d, pos.y = %d\n", pos.xpos, pos.ypos);
    vt1603_ts_report_pos(ts_drv, &pos);
    vt1603_clr_ts_irq(ts_drv, int_sts & 0x0F);
next_loop:
#ifdef CONFIG_HAS_EARLYSUSPEND
    if (early_suspend_stage == 0)
#endif
        queue_delayed_work(ts_drv->workqueue, &ts_drv->read_work,
                        msecs_to_jiffies(ts_drv->ts_delay));
out:
    spin_unlock_irqrestore(&ts_drv->spinlock, flags);
    ts_dbg("Exit\n");
    return ;
}

static void vt1603_ts_read_loop(struct work_struct *dwork)
{
    struct vt1603_ts_drvdata *ts_drv = NULL;
    struct delayed_work *dwk = (struct delayed_work *)dwork;

    ts_dbg("Enter\n");
    ts_drv = container_of(dwk, struct vt1603_ts_drvdata, read_work);

    if (is_vt1609 == 0)
        vt1603_ts_pure_single_read_loop(ts_drv);
    else {
        if (unlikely(vt1603_ts_dual == NULL))
            vt1603_ts_pure_single_read_loop(ts_drv);
        else
            vt1603_ts_dual(ts_drv);
    }

    ts_dbg("Exit\n\n\n");
    return ;
}

/*
 * vt1603_ts_isr - vt1603 ts/bat/temp module interrupt routine
 *
 * TODO: we just close soc gpio interrupt input and schedule the
 *    workqueue here, the soc gpio interrupt will open in workqueue
 *    routine function, so we can ensure every intrrupt 
 *    will be handled 
 *
 * @irq: irq number
 * @dev_id: irq handler data, actually is vt1603 driver data
 */
static irqreturn_t vt1603_ts_isr(int irq, void *dev_id)
{
    struct vt1603_ts_drvdata *ts_drv = dev_id;
/*
    int irq_mask = 1 << ts_drv->gpio_num;

    if ((GPIO_INT_REQ_STS_VAL & irq_mask) == 0) {
        ts_dbg("Other gpio irq!, status:0x%08x, mask:%d\n",
                        GPIO_INT_REQ_STS_VAL, irq_mask);
        return IRQ_NONE;
    }
*/
    /*check whether GPIO interrupt turn on or not?*/
    if(!soc_check_int_enable_gpio())
        return IRQ_NONE;

    /*check interrupt status*/
    if(!soc_check_int_status_gpio())
        return IRQ_NONE;

    ts_drv->soc_gpio_irq_disable(ts_drv);
    ts_drv->soc_gpio_irq_clear(ts_drv);
    schedule_work(&ts_drv->work);

    return IRQ_HANDLED;
}

static int vt1603_ts_input_dev_init(struct vt1603_ts_drvdata * ts_drv)
{
    ts_drv->input->name = "vt1603_ts";
	set_bit(EV_SYN, ts_drv->input->evbit);
	set_bit(EV_KEY, ts_drv->input->evbit);
	set_bit(EV_ABS, ts_drv->input->evbit);
	set_bit(BTN_TOUCH, ts_drv->input->keybit);
    set_bit(INPUT_PROP_DIRECT, ts_drv->input->propbit);

#if 0
    input_set_abs_params(ts_drv->input, ABS_X, 0,
                              vt1603_ts_get_resolvX(), 2, 0);
    input_set_abs_params(ts_drv->input, ABS_Y, 0,
                              vt1603_ts_get_resolvY(), 2, 0);
#endif
    input_set_abs_params(ts_drv->input, ABS_MT_POSITION_X, 0,
                              vt1603_ts_get_resolvX(), 2, 0);
    input_set_abs_params(ts_drv->input, ABS_MT_POSITION_Y, 0,
                              vt1603_ts_get_resolvY(), 2, 0);
//    input_set_abs_params(ts_drv->input, ABS_MT_TOUCH_MAJOR, 0, 15, 0, 0);
    input_register_device(ts_drv->input);

    return 0;
}

/*
 * vt1603_ts_reset - reset vt1603, auto postition conversion mode,
 *     do self calibration if enable
 * @ts_drv: vt1603 driver data
 */
static void vt1603_ts_reset(struct vt1603_ts_drvdata * ts_drv)
{
    /* power control enable  */
    vt1603_set_reg8(ts_drv, VT1603_PWC_REG, 0x08);
    /* auto position conversion mode and panel type config */
    if (ts_drv->ts_type == PANEL_TYPE_4WIRED)
        vt1603_set_reg8(ts_drv, VT1603_CR_REG, BIT1);
    else
        vt1603_set_reg8(ts_drv, VT1603_CR_REG, BIT1 | BIT0);
    /* interrupt control, pen up/down detection disable    */
    vt1603_set_reg8(ts_drv, VT1603_INTCR_REG, 0x7F);
    /* interrupt enable register, timeout and pen up / down irq  */
    vt1603_set_reg8(ts_drv, VT1603_INTEN_REG, BIT1 | BIT2 | BIT3);
    /* clock divider */
    vt1603_set_reg8(ts_drv, VT1603_CDPR_REG, ts_drv->sclk_div); 
    /* precharge time set to 0x10 to support dual touch */
    vt1603_set_reg8(ts_drv, VT1603_TSPC_REG, 0x10);
    /* clear penup, timeout, conversion_end irq firest */
    vt1603_clr_ts_irq(ts_drv, BIT0 | BIT2 | BIT3);
}

struct vt1603_ts_device vt1603_ts_dev;
EXPORT_SYMBOL_GPL(vt1603_ts_dev);
static int __devinit vt1603_ts_spi_probe(struct spi_device *spi)
{
    int ret = 0;
    struct vt1603_ts_drvdata *ts_drv = NULL;
    struct vt1603_ts_platform_data *ts_pdata = NULL;

    parse_gpt_arg();

    ts_pdata = spi->dev.platform_data;
    if (NULL == ts_pdata) {
        ts_dbg("vt1603_ts: platform data NULL\n");
        ret = -ENODATA;
        goto out;
    }
    if (spi->bits_per_word != 8) {
        ts_dbg("vt1603_ts: bits_per_word can't be %d\n", spi->bits_per_word);
        spi->bits_per_word = 8;
    }
    if (spi->max_speed_hz > VT1603_MAX_SPI_CLK) {
        ts_dbg("vt1603_ts: spi clk can't be %dhz\n", spi->max_speed_hz);
        spi->max_speed_hz = SPI_DEFAULT_CLK;
    }
    if (spi->mode != SPI_MODE_3) {
        ts_dbg("vt1603_ts: spi mode can not be 0x%x\n", spi->mode);
        spi->mode = SPI_MODE_3;
    }
    ret = spi_setup(spi);
    if (ret) {
        ts_dbg("vt1603_ts: spi_setup failed\n");
        goto out;
    }
    /* touch panel controller driver data allocation */
    ts_drv = kzalloc(sizeof(*ts_drv), GFP_KERNEL);
    if (!ts_drv) {
        ts_dbg("vt1603_ts: alloc driver data failed\n");
        ret = -ENOMEM;
        goto out;
    }
    /* touch panel controller driver data initialization */
    ts_drv->spi        = spi;
    ts_drv->pdata      = ts_pdata;
    ts_drv->gpio_num   = ts_pdata->gpio_num;
    ts_drv->sclk_div   = ts_pdata->sclk_div;
    ts_drv->mode       = VT1603_TS_MODE;
    ts_drv->ts_state   = TS_PENUP_STATE;
    ts_drv->gpio_irq   = soc_gpio_num_to_irq_num(ts_drv);

    spin_lock_init(&ts_drv->spinlock);
    vt1603_ts_dev.ts_drv = ts_drv;
    dev_set_drvdata(&spi->dev, ts_drv);
    /* soc-side gpio handle function init */
    vt1603_ts_soc_gpio_handle_init(ts_drv);
    ts_drv->soc_gpio_irq_init(ts_drv);
    ts_drv->ts_delay   = TS_READ_DELAY;
    ts_drv->ts_type    = ts_pdata->panel_type;
    ts_drv->ts_stamp   = 0;
    ts_drv->mt_stage   = NO_FINGER;
    /* GPIO IRQ request */
    if (ts_drv->gpio_irq <= 0) {
        ts_dbg("vt1603_ts: IRQ number error\n");
        ret = -ENODEV;
        goto release_driver_data;
    }
    INIT_WORK(&ts_drv->work, vt1603_ts_work);
    INIT_DELAYED_WORK(&ts_drv->read_work, vt1603_ts_read_loop);
    ts_drv->workqueue = create_singlethread_workqueue(DRV_NAME);
    if (!ts_drv->workqueue) {
        ts_dbg("vt1603 create singlethread workqueue failed!\n");
        goto release_driver_data;
    }
    /* we disable gpio irq to avoid interrupt in vt1603 initialization */
    ts_drv->soc_gpio_irq_disable(ts_drv);
    if (request_irq(ts_drv->gpio_irq, vt1603_ts_isr, IRQF_SHARED,
                    DRV_NAME, ts_drv)) {
        ts_dbg("vt1603_ts: request IRQ %d failed\n",
                    ts_drv->gpio_irq);
        ret = -ENODEV;
        goto release_workqueue;
    }
    /* iuput device routine */
    ts_drv->input = input_allocate_device();
    if (!ts_drv->input) {
        ts_dbg("vt1603_ts: alloc input device failed");
        ret = -ENOMEM;
        goto release_gpio_irq;
    }
    vt1603_ts_input_dev_init(ts_drv);

#ifdef CONFIG_HAS_EARLYSUSPEND
    vt1603_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    vt1603_early_suspend.suspend = vt1603_ts_early_suspend;
    vt1603_early_suspend.resume = vt1603_ts_late_resume;
    register_early_suspend(&vt1603_early_suspend);
#endif
    /* hardware prepare here */
    /* 1. mclk enable        */
    vt1603_ts_clk_enable();
    /* 2.vt1603 touch-panel and sar-adc module reset */
    vt1603_ts_reset(ts_drv);
    /* 3. vt1603 gpio1 reset, as irq output */
    vt1603_gpio1_reset(ts_drv);
    /* 4. dump vt1603 to ensure setting ok  */
    vt1603_reg_dump(ts_drv, 0xc0, 12);
    /* enable soc gpio irq */
    printk(KERN_INFO "VT1603 Touch Panel Driver OK!\n");
    ts_drv->soc_gpio_irq_clear(ts_drv);
    ts_drv->soc_gpio_irq_enable(ts_drv);
    goto out;

release_gpio_irq:
    free_irq(ts_drv->gpio_irq, ts_drv);
release_workqueue:
    cancel_work_sync(&ts_drv->work);
    cancel_delayed_work(&ts_drv->read_work);
    flush_workqueue(ts_drv->workqueue);
    destroy_workqueue(ts_drv->workqueue);
release_driver_data:
    kfree(ts_drv);
    ts_drv = NULL;
out:
    return ret;
}

static int __devexit vt1603_ts_spi_remove(struct spi_device *spi)
{
    struct vt1603_ts_drvdata *ts_drv;
    ts_drv = dev_get_drvdata(&spi->dev);

    ts_dbg("Enter\n");

    /* disable gpio irq first         */
    ts_drv->soc_gpio_irq_disable(ts_drv);
    ts_drv->soc_gpio_irq_clear(ts_drv);
    /* disable sar adc interrupt       */
    vt1603_set_reg8(ts_drv, VT1603_CR_REG, 0x00);
    vt1603_set_reg8(ts_drv, VT1603_INTEN_REG, 0x00);
    vt1603_clr_ts_irq(ts_drv, 0x0F);
    /* clock disable and power down     */
    vt1603_set_reg8(ts_drv, VT1603_PWC_REG, 0x21);
    cancel_work_sync(&ts_drv->work);
    cancel_delayed_work(&ts_drv->read_work);
    flush_workqueue(ts_drv->workqueue);
    destroy_workqueue(ts_drv->workqueue);
    /* gpio irq free first              */
    free_irq(ts_drv->gpio_irq, ts_drv);
    /* input unregister                 */
    input_unregister_device(ts_drv->input);
    /* enable gpio irq, codec maybe use it */
    ts_drv->soc_gpio_irq_clear(ts_drv);
    ts_drv->soc_gpio_irq_enable(ts_drv);
    /* free vt1603 driver data          */
    dev_set_drvdata(&spi->dev, NULL);
    kfree(ts_drv);   
    ts_drv = NULL;
    ts_dbg("Exit\n");
    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void vt1603_ts_early_suspend(struct early_suspend *h)
{
    early_suspend_stage = 1;
    vt1603_ts_dev.ts_drv->soc_gpio_irq_disable(vt1603_ts_dev.ts_drv);
    cancel_work_sync(&vt1603_ts_dev.ts_drv->work);
    cancel_delayed_work(&vt1603_ts_dev.ts_drv->read_work);
    flush_workqueue(vt1603_ts_dev.ts_drv->workqueue);
    vt1603_ts_dev.ts_drv->ts_state   = TS_PENUP_STATE;
    vt1603_ts_dev.ts_drv->mt_stage   = NO_FINGER;
}
void vt1603_ts_late_resume(struct early_suspend *h)
{
    early_suspend_stage = 0;
    vt1603_ts_dev.ts_drv->soc_gpio_irq_disable(vt1603_ts_dev.ts_drv);
    vt1603_ts_dev.ts_drv->ts_state = TS_PENUP_STATE;
    vt1603_ts_dev.ts_drv->mode = VT1603_TS_MODE;
    vt1603_ts_dev.ts_drv->mt_stage = NO_FINGER;
    vt1603_ts_clk_enable();
    /* vt1603 ts hardware resume     */
    vt1603_ts_reset(vt1603_ts_dev.ts_drv);
    /* vt1603 gpio1 reset            */
    vt1603_gpio1_reset(vt1603_ts_dev.ts_drv);

    /* enable gpio irq last          */
    /* FIXME: it's no need init gpio irq? */
    vt1603_ts_dev.ts_drv->soc_gpio_irq_init(vt1603_ts_dev.ts_drv);
    vt1603_ts_dev.ts_drv->soc_gpio_irq_clear(vt1603_ts_dev.ts_drv);
    vt1603_ts_dev.ts_drv->soc_gpio_irq_enable(vt1603_ts_dev.ts_drv);
}
#endif
#ifdef CONFIG_PM
static int
vt1603_ts_spi_suspend(struct spi_device *spi, pm_message_t message)
{
    struct vt1603_ts_drvdata *ts_drv;

    ts_dbg("Enter\n");
    ts_drv = dev_get_drvdata(&spi->dev);
    /* disable and clear gpio irq first      */
#ifndef CONFIG_HAS_EARLYSUSPEND
    ts_drv->soc_gpio_irq_disable(ts_drv);
    cancel_work_sync(&ts_drv->work);
    cancel_delayed_work(&ts_drv->read_work);
    flush_workqueue(ts_drv->workqueue);
    ts_drv->ts_state   = TS_PENUP_STATE;
    ts_drv->mt_stage   = NO_FINGER;
#endif
    /* FIXME: we do not enable soc gpio irq when vt1603 suspend */
    //ts_drv->soc_gpio_irq_enable(ts_drv);

    ts_dbg("Exit\n");
    return 0;
}

static int vt1603_ts_spi_resume(struct spi_device *spi)
{
#ifndef CONFIG_HAS_EARLYSUSPEND
    struct vt1603_ts_drvdata *ts_drv = dev_get_drvdata(&spi->dev);

    ts_dbg("Enter\n");

    ts_drv->soc_gpio_irq_disable(ts_drv);
    ts_drv->ts_state = TS_PENUP_STATE;
    ts_drv->mode = VT1603_TS_MODE;
    ts_drv->mt_stage = NO_FINGER;
    vt1603_ts_clk_enable();
    /* vt1603 ts hardware resume     */
    vt1603_ts_reset(ts_drv);
    /* vt1603 gpio1 reset            */
    vt1603_gpio1_reset(ts_drv);

    /* enable gpio irq last          */
    /* FIXME: it's no need init gpio irq? */
    ts_drv->soc_gpio_irq_init(ts_drv);
    ts_drv->soc_gpio_irq_clear(ts_drv);
    ts_drv->soc_gpio_irq_enable(ts_drv);

    ts_dbg("Exit\n");
#endif
    return 0;
}

#else
#define vt1603_ts_spi_suspend NULL
#define vt1603_ts_spi_resume  NULL
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32))
static const struct spi_device_id vt1603_ts_spi_dev_ids[] = {
        { DRV_NAME, 0},
        { },
};
MODULE_DEVICE_TABLE(spi, vt1603_ts_spi_dev_ids);
#endif

struct spi_driver vt1603_ts_spi_driver = {
    .driver    = {
        .name  = DRV_NAME,
        .bus   = &spi_bus_type,
        .owner = THIS_MODULE,
        },
    .probe   = vt1603_ts_spi_probe,
    .remove  = vt1603_ts_spi_remove,
    .suspend = vt1603_ts_spi_suspend,
    .resume  = vt1603_ts_spi_resume,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32))
    .id_table = vt1603_ts_spi_dev_ids,
#endif
};

static struct wmt_spi_slave vt1603_ts_info = {
    .dma_en        = SPI_DMA_DISABLE,
    .bits_per_word = 8,
};

/*TODO: Here is the vt1603 driver configuration area, if you want to
 *      drive vt1603 on your platform, you MUST ensure this platform
 *      data is right 
 *TODO: For VT3445(wm8710), irq_type just support edge active, but 
        VT3465(wm3465) support all the four irq trigger type */
static struct vt1603_ts_platform_data vt1603_ts_pdata = {
    .sclk_div     = 0x04,
    .irq_type     = FALLING_EDGE_ACTIVE,
    .gpio_num     = 2,
    .panel_type   = PANEL_TYPE_4WIRED,
};

static struct spi_board_info vt1603_ts_spi_bi = { 
    .modalias           = DRV_NAME,
    .platform_data      = &vt1603_ts_pdata,   /* vt1603 ts platform data */
    .controller_data    = &vt1603_ts_info,    /* vt1603 spi config info  */
    .irq                = -1,                 /* use gpio irq            */
    .max_speed_hz       = SPI_DEFAULT_CLK,    /* same as spi master      */ 
    .bus_num            = VT1603_SPI_BUS_0,   /* use spi master 0        */
    .mode               = SPI_CLK_MODE3,      /* phase1, polarity1       */ 
    .chip_select        = VT1603_SPI_FAKE_CS, /* as slave 0, CS=0        */
};

static int vt1603_ts_dev_open(struct inode *inode, struct file *filp)
{
    struct vt1603_ts_device *ts_dev;

    ts_dbg("Enter\n");

    ts_dev = container_of(inode->i_cdev, struct vt1603_ts_device, cdev);
    if (ts_dev->ts_drv == NULL) {
        ts_dbg("can not get vt1603_ts driver data\n");
        return -ENODATA;
    }
    filp->private_data = ts_dev;

    ts_dbg("Exit\n");
    return 0;
}

static int vt1603_ts_dev_close(struct inode *inode, struct file *filp)
{
    struct vt1603_ts_device *ts_dev;

    ts_dbg("Enter\n");

    ts_dev = container_of(inode->i_cdev, struct vt1603_ts_device, cdev);
    if (ts_dev->ts_drv == NULL) {
        ts_dbg("can not get vt1603_ts driver data\n");
        return -ENODATA;
    }

    ts_dbg("Exit\n");
    return 0;
}

static long
vt1603_ts_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    int err = 0;
	int nBuff[8] = { 0 };
	char env_val[96] = { 0 };
    struct vt1603_ts_device *ts_dev;
    struct vt1603_ts_drvdata *ts_drv;
    struct vt1603_reg_ioc ts_ioc;

    ts_dbg("Enter\n");
    /* check type and command number */
    if (_IOC_TYPE(cmd) != VT1603_TS_IOC_MAGIC)
        return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
        err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
        err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    if (err)
        return -EFAULT;

    ts_dev = filp->private_data;
    ts_drv = ts_dev->ts_drv;

    switch (cmd) {
    /* get/set register value        */
    case VT1603_IOC_RD_REG:
        ret = copy_from_user(&ts_ioc, (void __user *)arg, sizeof(ts_ioc));
        if (ret == 0) {
            ts_ioc.reg_val = vt1603_get_reg8(ts_drv, ts_ioc.reg_addr);
            ret = copy_to_user((void __user *)arg, &ts_ioc, sizeof(ts_ioc));
        }
        break;
    case VT1603_IOC_WR_REG:
        ret = copy_from_user(&ts_ioc, (void __user *)arg, sizeof(ts_ioc));
        if (ret == 0)
            vt1603_set_reg8(ts_drv, ts_ioc.reg_addr, ts_ioc.reg_val);
        break;
    /* vt1603 touch calibration      */
    case VT1603_TS_IOC_CAL_START:
        g_bCalibrating = true;
        ts_dbg("vt1603 ts ioctl cal start\n");
		ts_dbg("cal g_CalcParam = %d, %d, %d, %d, %d, %d, %d\n",
				g_CalcParam.a1, g_CalcParam.b1, g_CalcParam.c1,
				g_CalcParam.a2, g_CalcParam.b2, g_CalcParam.c2, g_CalcParam.delta);
        break;
    /* CAL_DONE means calibration success */
    case VT1603_TS_IOC_CAL_DONE:
        g_bCalibrating = false;
        ts_dbg("vt1603 ts ioctl cal done\n");
        copy_from_user(nBuff, (unsigned int *)arg, 8 * sizeof(int));
		if (nBuff[7] == 0) {
			printk("calibration failed! Dont set\n");
			return 0;
		}
		g_CalcParam.a1 = nBuff[0];
		g_CalcParam.b1 = nBuff[1];
		g_CalcParam.c1 = nBuff[2];
		g_CalcParam.a2 = nBuff[3];
		g_CalcParam.b2 = nBuff[4];
		g_CalcParam.c2 = nBuff[5];
		g_CalcParam.delta = nBuff[6];
		if (g_CalcParam.delta == 0)
			g_CalcParam.delta = 1;
		if (is_vt1609 == 0)
			sprintf(env_val, "1:vt1603:spi:%d:%d:%d %d %d %d %d %d %d",panelres_x, panelres_y,
				nBuff[0], nBuff[1], nBuff[2], nBuff[3], nBuff[4], nBuff[5], nBuff[6]);
		else
			sprintf(env_val, "1:vt1609:spi:%d:%d:%d %d %d %d %d %d %d",panelres_x, panelres_y,
				nBuff[0], nBuff[1], nBuff[2], nBuff[3], nBuff[4], nBuff[5], nBuff[6]);
		ts_dbg("TSC calibrate done data: [%s]\n", env_val);
		wmt_setsyspara("wmt.io.ts", env_val);
		break;
    /* CAL_QUIT means calibration failed */
    case VT1603_TS_IOC_CAL_QUIT:
		g_bCalibrating = false;
		ts_dbg("vt1603 ts ioctl cal quit\n");
		ts_dbg("cal_quit g_CalcParam = %d, %d, %d, %d, %d, %d, %d\n",
				g_CalcParam.a1, g_CalcParam.b1, g_CalcParam.c1,
				g_CalcParam.a2, g_CalcParam.b2, g_CalcParam.c2, g_CalcParam.delta);
		break;
    case VT1603_TS_IOC_CAL_RAWDATA:
		ts_dbg("vt1603 ts ioctl cal raw data\n");
		if (!g_bCalibrating) {
            ts_dbg("g_bCalibrating is false\n");
		    return -EINVAL;
        }
		nBuff[0] = g_evLast.x;
		nBuff[1] = g_evLast.y;
		copy_to_user((unsigned int *)arg, nBuff, 2 * sizeof(int));
		ts_dbg("raw data: x=%d, y=%d\n", nBuff[0], nBuff[1]);
        break;
#ifdef DEBUG
    case VT1603_IOC_OPEN_DBG:
        dbg_mask = 1;
        ts_dbg("vt1603 debug info enable\n");
        break;
    case VT1603_IOC_CLOSE_DBG:
        ts_dbg("vt1603 debug info disable\n");
        dbg_mask = 0;
        break;
#endif
    default:
        ts_dbg("command unknow");
		ret = -EINVAL;
        break;
    }

    ts_dbg("Exit\n");
    return ret;
}

static struct file_operations vt1603_ts_fops = {
    .owner          = THIS_MODULE,
    .open           = vt1603_ts_dev_open,
    .unlocked_ioctl = vt1603_ts_dev_ioctl,
    .release        = vt1603_ts_dev_close,
};

static int vt1603_ts_dev_major = VT1603_TS_DEV_MAJOR;
static int vt1603_ts_dev_minor = 0;
module_param(vt1603_ts_dev_major, int, S_IRUGO);
module_param(vt1603_ts_dev_minor, int, S_IRUGO);

static struct class *vt1603_ts_class;
static int vt1603_ts_dev_setup(void)
{
    dev_t dev_no = 0;
    int ret = 0;
    struct device *dev = NULL;

    ts_dbg("Enter\n");
    if (vt1603_ts_dev_major) {
        dev_no = MKDEV(vt1603_ts_dev_major, vt1603_ts_dev_minor);
        ret = register_chrdev_region(dev_no, VT1603_TS_NR_DEVS, DEV_NAME);
    } else {
        ret = alloc_chrdev_region(&dev_no, vt1603_ts_dev_minor, 
                                VT1603_TS_NR_DEVS, DEV_NAME);
        vt1603_ts_dev_major = MAJOR(dev_no);
        vt1603_ts_dev_minor = MINOR(dev_no);
        ts_dbg("vt1603_ts device major is %d\n", vt1603_ts_dev_major);
    }

    if (ret < 0) {
        ts_dbg("can not get major %d\n", vt1603_ts_dev_major);
        goto out;
    }

    cdev_init(&vt1603_ts_dev.cdev, &vt1603_ts_fops);
    vt1603_ts_dev.ts_drv     = NULL;
    vt1603_ts_dev.cdev.owner = THIS_MODULE;
    vt1603_ts_dev.cdev.ops   = &vt1603_ts_fops;
    ret = cdev_add(&vt1603_ts_dev.cdev, dev_no, VT1603_TS_NR_DEVS);
    if (ret) {
        ts_dbg("add char dev for vt1603 ts failed\n");
        goto release_region;
    }

    vt1603_ts_class = class_create(THIS_MODULE, "touch_panel");
    if (IS_ERR(vt1603_ts_class)) {
        ts_dbg("create vt1603_ts class failed\n");
        ret = PTR_ERR(vt1603_ts_class);
        goto release_cdev;
    }

    /* FIXME: parent should be spi->dev */
    dev = device_create(vt1603_ts_class, NULL, dev_no, NULL, DEV_NAME);
	if (IS_ERR(dev)) {
        ts_dbg("create device for vt1603 ts failed\n");
		ret = PTR_ERR(dev);
		goto release_class;
	}
    goto out;

release_class:
    class_destroy(vt1603_ts_class);
    vt1603_ts_class = NULL;
release_cdev:
    cdev_del(&vt1603_ts_dev.cdev);
release_region:
    unregister_chrdev_region(dev_no, VT1603_TS_NR_DEVS);
out:
    ts_dbg("Exit\n");
    return ret;
}

static void vt1603_ts_dev_cleanup(void)
{
    dev_t dev_no = MKDEV(vt1603_ts_dev_major, vt1603_ts_dev_minor);

    ts_dbg("Enter\n");
    cdev_del(&vt1603_ts_dev.cdev);
    unregister_chrdev_region(dev_no, VT1603_TS_NR_DEVS);
    device_destroy(vt1603_ts_class, dev_no);
    class_destroy(vt1603_ts_class);
    ts_dbg("Exit\n");
}

static int __init vt1603_ts_uboot_env_check(void)
{
	int ret = 0;
	int varlen = 128;
    int ts_en = 0;
    unsigned char buf[128] = {0};

    /* TODO: uboot env should be 
     * wmt.io.ts = 1:vt1603: a1 b1 c1 a2 b2 c2 delta */
    ts_dbg("Enter\n");
    ret = wmt_getsyspara("wmt.io.ts", buf, &varlen);
    if (ret) {
        ts_dbg("vt1603_ts: cann't find para [wmt.io.ts]\n");
        ret = -ENODATA;
        goto out;
    }
    if (!strncmp(buf + 2, "vt1603", 6))
        is_vt1609 = 0;
    else if (!strncmp(buf + 2, "vt1609", 6))
        is_vt1609 = 1;
    else {
        ts_dbg("vt1603_ts: touchscreen is not vt1603\n");
        ret = -ENODEV;
        goto out;
    }
    if (strncmp(buf + 9, "spi", 3)) {
        ret = -ENODEV;
        goto out;
    } 

    if (is_vt1609 == 0)
        sscanf(buf, "%d:vt1603:spi:%d:%d:%d %d %d %d %d %d %d",
                    &ts_en,
                    &panelres_x,
                    &panelres_y,
                    &g_CalcParam.a1,
                    &g_CalcParam.b1,
                    &g_CalcParam.c1,
                    &g_CalcParam.a2,
                    &g_CalcParam.b2,
                    &g_CalcParam.c2,
                    &g_CalcParam.delta);
    else
        sscanf(buf, "%d:vt1609:spi:%d:%d:%d %d %d %d %d %d %d",
                    &ts_en,
                    &panelres_x,
                    &panelres_y,
                    &g_CalcParam.a1,
                    &g_CalcParam.b1,
                    &g_CalcParam.c1,
                    &g_CalcParam.a2,
                    &g_CalcParam.b2,
                    &g_CalcParam.c2,
                    &g_CalcParam.delta);
    if (!ts_en) {
        ts_dbg("vt1603_ts: touchscreen is disabled now\n");
        ret = -ENODEV;
        goto out;
    }
	ts_dbg("vt1603_ts calibration param:[%d %d %d %d %d %d %d]\n",
        g_CalcParam.a1, g_CalcParam.b1, g_CalcParam.c1, 
        g_CalcParam.a2, g_CalcParam.b2, g_CalcParam.c2, g_CalcParam.delta);

out:
    ts_dbg("Exit\n");
    return ret;
}

static int __init vt1603_ts_spi_init(void)
{
    int ret = 0;
    struct spi_master *master = NULL;
    struct spi_device *slave  = NULL;

    ts_dbg("Enter\n");
    ret = vt1603_ts_uboot_env_check();
    if (ret) {
        ts_dbg("vt1603_ts uboot env check failed\n");
        goto out;
    }
    ret = vt1603_ts_dev_setup();
    if (ret) {
        ts_dbg("vt1603_ts create device node failed\n");
        goto out;
    }
    master = spi_busnum_to_master(vt1603_ts_spi_bi.bus_num);
    if (NULL == master) {
        ts_dbg("can't find master in bus%d\n", vt1603_ts_spi_bi.bus_num);
        ret = -ENODEV;
        goto release_dev;
    }
    slave = spi_new_device(master, &vt1603_ts_spi_bi);
    if (NULL == slave) {
        ts_dbg("vt1603_ts add spi device failed\n");
        ret = -ENOMEM;
        goto release_dev;
    }
    vt1603_spi_device_dump(slave);
    ret = spi_register_driver(&vt1603_ts_spi_driver);
    if (ret) {
        ts_dbg("vt1603_ts register spi driver failed\n");
        goto release_slave;
    }
    goto out;

release_slave:
    spi_unregister_device(slave);
    slave = NULL;
release_dev:
    vt1603_ts_dev_cleanup();
out:
    ts_dbg("Exit\n");
    return ret;
}
late_initcall(vt1603_ts_spi_init);

static void __exit vt1603_ts_spi_exit(void)
{
    ts_dbg("Enter\n");
    spi_unregister_device(vt1603_ts_dev.ts_drv->spi);
    spi_unregister_driver(&vt1603_ts_spi_driver);
    vt1603_ts_dev_cleanup();
    ts_dbg("Exit\n");
}
module_exit(vt1603_ts_spi_exit);

MODULE_AUTHOR("WonderMedia Technologies, Inc");
MODULE_DESCRIPTION("VT1603A Touch-Panel Controller and SAR-ADC Driver");
MODULE_LICENSE("Dual BSD/GPL");
