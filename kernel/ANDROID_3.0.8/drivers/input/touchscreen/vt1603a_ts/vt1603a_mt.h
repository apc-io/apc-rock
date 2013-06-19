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
--*/

#ifndef __VT1603A_MT_H__
#define __VT1603A_MT_H__

#define DEV_NAME  "wmtts"
#define DRV_NAME  "vt1603_ts"

#define POLLING_BITS_TOUT     0x20000
#define ADC_DATA(low, high)  ((((high) & 0x0F) << 8) + (low))

/* touch panel type config   */
#define PANEL_TYPE_4WIRED     0x10
#define PANEL_TYPE_5WIRED     0x11

/* VT1603 working mode       */
#define VT1603_TS_MODE      BIT1
#define VT1603_TEMP_MODE    BIT2
#define VT1603_BAT_MODE     BIT3

/* VT1603 touch panel state  */
#define TS_PENDOWN_STATE     0x00
#define TS_PENUP_STATE       0x01


/* vt1603 bus type config    */
#ifdef  CONFIG_VT1603_TS_SPI
#define VT1603_TS_USE_SPI
#define VT1603_SPI_FIX_CS   0x00
#define VT1603_SPI_FAKE_CS  0x7F
#define VT1603_SPI_BUS_0    0x00
#define VT1603_SPI_BUS_1    0x01
#define VT1603_MAX_SPI_CLK    (20*1000*1000)
#define SPI_DEFAULT_CLK       (12*1000*1000)  /* 12Mhz, Max 20Mhz */
#define IDLE_DATA_NUM         5
#define VT1603_REG_OP_R       0x00
#define VT1603_REG_OP_W       0x01
#endif

#ifdef  CONFIG_VT1603_TS_I2C
#define VT1603_TS_USE_I2C
#define VT1603_I2C_FIX_ADDR  0x1A
#define VT1603_I2C_FAKE_ADDR 0xFF
#define VT1603_TS_I2C_WCMD   0x00
#define VT1603_TS_I2C_RCMD   0x01
#define VT1603_TS_I2C_RWCMD  0x02
#define VT1603_I2C_BUS_0     0x00
#define VT1603_I2C_BUS_1     0x01
#endif

/*
 * struct vt1603_ts_pos - vt1603 position conversion data
 * @xpos:                 x position conversion data
 * @ypos:                 y position conversion data
 */
struct vt1603_ts_pos {
    u16 xpos;
    u16 ypos;
};

#define AVG_POS_BUF_LEN  5
struct pos_buf {
    int len;
    struct vt1603_ts_pos pos_fifo[AVG_POS_BUF_LEN];
};

/*
 * struct vt1603_ts_drvdata - vt1603 driver data
 * @spi:                      spi device(slave) for vt1603
 * @input:                    touch panel as an input device 
 * @spinlock:                 spinlock
 * @work:                     work struct
 * @workqueue:                workqueue for interrupts processing
 * @gpio_irq:                 gpio interrupts, input is vt1603 gpio1
 * @ts_type:                  touch panel type, get from platform data
 * @sclk_div:                 SCLK dividor, get from platform data
 * @mode:                     vt1603 working mode, bat\temp\touch-panel
 * @ts_state:                 touch panel state, pen down or pen up
 */
/* FIXME: is it better to use a union instead of macro defination? */
struct vt1603_ts_drvdata {
#ifdef VT1603_TS_USE_SPI
    struct spi_device *spi;
#endif
#ifdef VT1603_TS_USE_I2C
    struct i2c_client *i2c;
#endif
    spinlock_t spinlock;
    struct vt1603_ts_platform_data *pdata;
    int  gpio_irq;
    char gpio_num;
    char ts_state;
    u8   sclk_div;
    u8   mode;
    void (*soc_gpio_irq_init)(struct vt1603_ts_drvdata *);
    void (*soc_gpio_irq_enable)(struct vt1603_ts_drvdata *);
    void (*soc_gpio_irq_disable)(struct vt1603_ts_drvdata *);
    void (*soc_gpio_irq_clear)(struct vt1603_ts_drvdata *);
    int ts_type;
    int ts_stamp;
    int ts_delay;
    unsigned int pos_num;
    struct input_dev *input;
    struct workqueue_struct *workqueue;
    struct work_struct work;
    struct delayed_work read_work;
    /* those members for dual touch support, 
     * you should never touch them */
    u32 rt;
    int vx_max;
    int vy_max;
    int vx;
    int vy;
    u8 mt_stage;
    int dual_num;
    int dual_to_single;

    struct vt1603_ts_pos fst_pos;
    u16 xx_t1;
    u16 xx_t2;
    u16 yy_t1;
    u16 yy_t2;
    int rt_prev;
    int ux;
    int uy;
    u64 slope;
};

enum gpio_irq_trigger_type {
      HIGH_ACTIVE         = 0,
      LOW_ACTIVE          = 1,
      RISING_EDGE_ACTIVE  = 3,
      FALLING_EDGE_ACTIVE = 4,
      UNKOWN_TYPE         = 0xFF
};

#define TS_DEBOUNCE    50
#define TS_READ_DELAY  10
#define VT1603_FILTER_FIRST_N_POS  1
#define VT1603_JITTER_THRESHOLD    800

/*
 * vt1603_ts_platform_data - vt1603 configuration data
 * @panel_type:              touch panel type: 4-wired or 5-wired
 * @sclk_div:                initial value of sclk dividor
 *    if mclk = 12.288MHZ 0x04 = 200ksps 0x08 = 100ksps
 * @soc_gpio_irq:            soc gpio interrupts, connect with vt1603 gpio1
 */
struct vt1603_ts_platform_data {
    u8  panel_type;
    u8  sclk_div;
    int i2c_bus_id;
    int soc_gpio_irq;
    int gpio_num;
    enum gpio_irq_trigger_type irq_type;
    u16 abs_x_min;
    u16 abs_x_max;
    u16 abs_y_min;
    u16 abs_y_max;
};

/* VT1603 Register address */
#define VT1603_BTHD_REG       0x78
#define VT1603_BCLK_REG       0x88
#define VT1603_BAEN_REG       0x04

#define VT1603_PWC_REG        0xC0
#define VT1603_CR_REG         0xC1
#define VT1603_CCCR_REG       0xC2
#define VT1603_CDPR_REG       0xC3
#define VT1603_TSPC_REG       0xC4
#define VT1603_AMCR_REG       0xC7
#define VT1603_INTCR_REG      0xC8
#define VT1603_INTEN_REG      0xC9
#define VT1603_INTS_REG       0xCA
#define VT1603_DCR_REG        0xCB

#define VT1603_TODCL_REG      0xCC
#define VT1603_TODCH_REG      0xCD

#define VT1603_DATL_REG       0xCE
#define VT1603_DATH_REG       0xCF

#define VT1603_XPL_REG        0xD0
#define VT1603_XPH_REG        0xD1
#define VT1603_YPL_REG        0xD2
#define VT1603_YPH_REG        0xD3

#define VT1603_BATL_REG       0xD4
#define VT1603_BATH_REG       0xD5

#define VT1603_TEMPL_REG      0xD6
#define VT1603_TEMPH_REG      0xD7

#define VT1603_ERR8_REG       0xD8
#define VT1603_ERR7_REG       0xD9
#define VT1603_ERR6_REG       0xDA
#define VT1603_ERR5_REG       0xDB
#define VT1603_ERR4_REG       0xDC
#define VT1603_ERR3_REG       0xDD
#define VT1603_ERR2_REG       0xDE
#define VT1603_ERR1_REG       0xDF

#define VT1603_DBG8_REG       0xE0
#define VT1603_DBG7_REG       0xE1
#define VT1603_DBG6_REG       0xE2
#define VT1603_DBG5_REG       0xE3
#define VT1603_DBG4_REG       0xE4
#define VT1603_DBG3_REG       0xE5
#define VT1603_DBG2_REG       0xE6
#define VT1603_DBG1_REG       0xE7

/* for VT1603 GPIO1 interrupt setting */
#define VT1603_IMASK_REG27    27
#define VT1603_IMASK_REG28    28
#define VT1603_IMASK_REG29    29
#define VT1603_IPOL_REG33     33
#define VT1603_ISEL_REG36     36

#define  VT1603_TS_NR_DEVS     1
#define  VT1603_TS_DEV_MAJOR   160

struct vt1603_ts_device {
    struct cdev cdev;
    struct vt1603_ts_drvdata *ts_drv;
};

struct vt1603_ts_cal_info {
    int   a1;
    int   b1;
    int   c1;
    int   a2;
    int   b2;
    int   c2;
    int   delta;
};

struct vt1603_ts_event {
    u16 x;
    u16 y;
};

/* for vt1603 register ioctl     */
struct vt1603_reg_ioc {
    u8  reg_addr;
    u8  reg_val;
    u16 reserve;
};

/* VT1603 TS and SAR-ADC IOCTL   */
#define VT1603_TS_IOC_MAGIC  't'

/* for touch screen calibration  */
#define VT1603_TS_IOC_CAL_START      _IO(VT1603_TS_IOC_MAGIC,  1)
#define VT1603_TS_IOC_CAL_DONE       _IOW(VT1603_TS_IOC_MAGIC, 2, int *)
#define VT1603_TS_IOC_CAL_RAWDATA    _IOR(VT1603_TS_IOC_MAGIC, 3, int *)
#define VT1603_TS_IOC_CAL_QUIT       _IOW(VT1603_TS_IOC_MAGIC, 4, int *)

/* read/write vt1603 register    */
#define VT1603_IOC_RD_REG            _IOW(VT1603_TS_IOC_MAGIC, 10, struct vt1603_reg_ioc)
#define VT1603_IOC_WR_REG            _IOW(VT1603_TS_IOC_MAGIC, 11, struct vt1603_reg_ioc)

/* get bat/temp conversion value */
#define VT1603_IOC_GET_BAT_VAL       _IOR(VT1603_TS_IOC_MAGIC, 10, u16)
#define VT1603_IOC_GET_TEMP_VAL      _IOR(VT1603_TS_IOC_MAGIC, 11, u16)

/* get/set bat detect interval   */
#define VT1603_IOC_GET_BAT_INTERVAL  _IOR(VT1603_TS_IOC_MAGIC, 13, u16)
#define VT1603_IOC_SET_BAT_INTERVAL  _IOW(VT1603_TS_IOC_MAGIC, 13, u16)

/* get/set temp detect interval  */
#define VT1603_IOC_GET_TEMP_INTERVAL _IOR(VT1603_TS_IOC_MAGIC, 14, u16)
#define VT1603_IOC_SET_TEMP_INTERVAL _IOW(VT1603_TS_IOC_MAGIC, 14, u16)

/* get/set conversion data shift */
#define VT1603_IOC_GET_SHIFT         _IOR(VT1603_TS_IOC_MAGIC, 15, u8)
#define VT1603_IOC_SET_SHIFT         _IOW(VT1603_TS_IOC_MAGIC, 15, u8)

/* get/set sclk dividor          */
#define VT1603_IOC_GET_CLK_DIV       _IOR(VT1603_TS_IOC_MAGIC, 16, u8)
#define VT1603_IOC_SET_CLK_DIV       _IOW(VT1603_TS_IOC_MAGIC, 16, u8)

/* get/set calibration capacitor */
#define VT1603_IOC_GET_CAL_SEL       _IOR(VT1603_TS_IOC_MAGIC, 17, u8)
#define VT1603_IOC_SET_CAL_SEL       _IOW(VT1603_TS_IOC_MAGIC, 17, u8)

/* diable/enable vt1603 debug info */
#define VT1603_IOC_OPEN_DBG         _IO(VT1603_TS_IOC_MAGIC, 18)
#define VT1603_IOC_CLOSE_DBG        _IO(VT1603_TS_IOC_MAGIC, 19)

#define NO_FINGER  0x00
#define ONE_FINGER 0x01
#define TWO_FINGER 0x02

typedef int (* vt1603_dual_touch_support_fn)(struct vt1603_ts_drvdata *);
extern int wmt_setsyspara(char *varname, unsigned char *varval);
extern int wmt_getsyspara(char *varname, unsigned char *varval, int *varlenex);
extern int vt1603_set_reg8(struct vt1603_ts_drvdata *ts_drv, u8 reg, u8 val);
extern u8 vt1603_get_reg8(struct vt1603_ts_drvdata *ts_drv, u8 reg);
extern void vt1603_setbits(struct vt1603_ts_drvdata *ts_drv, u8 reg, u8 mask);
extern void vt1603_clrbits(struct vt1603_ts_drvdata *ts_drv, u8 reg, u8 mask);
extern int vt1603_clr_ts_irq(struct vt1603_ts_drvdata *ts_drv, u8 mask);
extern void vt1603_ts_get_pos(struct vt1603_ts_drvdata *ts_drv, struct vt1603_ts_pos *pos);
extern void vt1603_ts_report_pos(struct vt1603_ts_drvdata *ts_drv, struct vt1603_ts_pos *pos);
extern int vt1603_ts_pos_calibration(struct vt1603_ts_pos *to_cal);
extern void vt1603_ts_report_penup(struct vt1603_ts_drvdata *ts_drv);

#endif  /* __VT1603A_MT_H__ */
