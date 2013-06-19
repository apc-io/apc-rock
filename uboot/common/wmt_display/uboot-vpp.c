/*++ 
Copyright (c) 2010 WonderMedia Technologies, Inc.

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details. You
should have received a copy of the GNU General Public License along with this
program. If not, see http://www.gnu.org/licenses/>.

WonderMedia Technologies, Inc.
4F, 531, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C.
--*/

#include <config.h>
#include <common.h>
#include <command.h>
#include <version.h>
#include <stdarg.h>
#include <linux/types.h>
#include <devices.h>
#include <linux/stddef.h>
#include <asm/byteorder.h>

//#define DEBUG
#include "vpp.h"
#include "vout.h"
#include "minivgui.h"

#define WMT_DISPLAY
//#define WMT_DUAL_DISPLAY_LOGO	// dual display : but firmware install osd not work

#include "../../board/wmt/include/wmt_clk.h"

// extern int auto_pll_divisor(enum dev_id dev, enum clk_cmd cmd, int unit, int freq);

#include "hw_devices.h"
#include "wmt_display.h"
#include "vout.h"

struct wmt_display_param_t g_display_param;
struct wmt_display_param_t g_display_param2;
struct wmt_pwm_setting_t g_pwm_setting;
vpp_timing_t g_display_tmr;
struct gpio_operation_t g_lcd_pw_pin;
int g_display_vaild;
unsigned int g_fb_phy;
unsigned int g_img_phy;
int g_logo_x;
int g_logo_y;
int g_direct_path;

int vpp_dac_sense_enable;

struct fb_var_screeninfo vfb_var = {
	.xres           = 800,
	.yres           = 480,
	.xres_virtual   = 800,
	.yres_virtual   = 480,
	.bits_per_pixel = 16,
	.red            = {11, 5, 0},
	.green          = {5, 6, 0},
	.blue           = {0, 5, 0},
	.transp         = {0, 0, 0},
	//.activate       = FB_ACTIVATE_NOW | FB_ACTIVATE_FORCE,
	.height         = -1,
	.width          = -1,
	.pixclock       = (VPP_HD_DISP_RESX*VPP_HD_DISP_RESY*VPP_HD_DISP_FPS),
	.left_margin    = 40,
	.right_margin   = 24,
	.upper_margin   = 32,
	.lower_margin   = 11,
	.hsync_len      = 96,
	.vsync_len      = 2,
	//.vmode          = FB_VMODE_NONINTERLACED
};

char *video_str[] = {
	"SDA", "SDD" , "LCD", "DVI" , "HDMI",	"DVO2HDMI", "LVDS", "VGA"
};

extern int vo_dvi_initial(void);
extern int vo_hdmi_initial(void);
extern int vo_lvds_initial(void);
extern int vo_virtual_initial(void);
extern int vt1632_module_init(void);
extern int sil902x_module_init(void);
extern void vpp_init(void);
extern vout_dev_t *lcd_get_dev(void);
extern void vpp_config(vout_info_t *info);
extern int cs8556_module_init(void);    // Jiun 04/02/2013

static int alignmentMemory(unsigned int *memtotal)
{
	unsigned int memmax = 512; //should get from MC4

	if (*memtotal > 256) {         /* 512M */
		*memtotal = 512;
	} else if (*memtotal > 128) {  /* 256M */
		*memtotal = 256;
	} else if (*memtotal > 64) {   /* 128M */
		*memtotal = 128;
	} else if (*memtotal > 32) {   /* 64M */
		*memtotal = 64;
	} else if (*memtotal > 16) {   /* 32M */
		*memtotal = 32;
	} else {
		*memtotal = 0;
	}
	if ((*memtotal == 0) || (*memtotal > memmax)) {
		printf("error : memtotal = %dMByte , can not calculate framebuffer address\n", memtotal);
		return -1;
	}
	return 0;
}


static int getwmtfb(unsigned int memtotal)
{
	unsigned int tmp, ofs;

	tmp = *(unsigned int *)0xd8120000;
	tmp >>= 16;

	switch(tmp) {
	case 0x3429:
	case 0x3465:
		ofs = 8;
		break;
	case 0x3451:
		ofs = 12;
		break;
	case 0x3445: // pmem has been disable
	case 0x3481:
		g_fb_phy = memtotal;
		return 0;
	default: 
		printf("error : unknow chip %x\n", tmp);
		return -1;
	}

	tmp = memtotal;
	if (alignmentMemory(&tmp))
		return -1;
	g_fb_phy = tmp - ofs;
	return 0;
}

static int wmt_init_check(void)
{
	char *tmp;
	unsigned int memtotal = 0;
//	long ps[6];

	g_fb_phy = 0;
	if ((tmp = getenv (ENV_DIRECTFB)) != NULL) {
		g_fb_phy = (unsigned int)simple_strtoul (tmp, NULL, 16);
	}

	if ((tmp = getenv (ENV_DIRECTPATH)) != NULL) {
		g_direct_path = (unsigned int)simple_strtoul (tmp, NULL, 10);
	} else {
		g_direct_path = 0;
	}

	if (g_fb_phy == 0)	{
		// we must check "memtotal" because we get framebuffer from it
		if ((tmp = getenv (ENV_MEMTOTAL)) != NULL) {
			memtotal = (unsigned int)simple_strtoul (tmp, NULL, 10);
		} else {
			printf("error : we need %s to calculate framebuffer address\n", ENV_MEMTOTAL);
			return -1;
		}
		if (g_direct_path != 0) {
			g_fb_phy = memtotal;
		} else {
			if (getwmtfb(memtotal))
				return -1;
		}
		g_fb_phy <<= 20;
	}

	if ((tmp = getenv (ENV_IMGADDR)) != NULL) {
		g_img_phy = (unsigned int)simple_strtoul (tmp, NULL, 16);
	} else {
		printf("%s is not found , command (display show) is invaild\n", ENV_IMGADDR);
		g_img_phy = 0xFFFF;
	}

	return 0;

// checkfailed :
	g_display_vaild = 0;
	return -1;	
}

#if 0
int uboot_vpp_alloc_framebuffer(unsigned int resx,unsigned int resy)
{
	return 0;
}
#endif

void vpp_mod_init(void)
{
//	vppm_mod_init();

#ifdef WMT_FTBLK_GOVRH	
	govrh_mod_init();
#endif


#ifdef WMT_FTBLK_LCDC
	lcdc_mod_init();
#endif

#ifdef WMT_FTBLK_GOVW
	govw_mod_init();
#endif
/*
#ifdef WMT_FTBLK_SCL
	scl_mod_init();
#endif
*/
#ifdef WMT_FTBLK_GOVM
	govm_mod_init();
#endif
/*
#ifdef WMT_FTBLK_VPU
	vpu_mod_init();
#endif
*/

#ifdef WMT_FTBLK_DISP	
	disp_mod_init();
#endif

	// register vout interface
	vo_dvi_initial();
	vo_hdmi_initial();
	vo_lvds_initial();
}

void vpp_device_init(void)
{
	auto_pll_divisor(DEV_PWM,CLK_ENABLE,0,0);
	lcd_module_init();
	lcd_oem_init();
	lcd_lw700at9003_init();
	lcd_at070tn83_init();
	lcd_a080sn01_init();
	lcd_ek08009_init();
	lcd_HSD101PFW2_init();
	
	vt1632_module_init();
	sil902x_module_init();
    cs8556_module_init();   // Jiun 04/02/2013
}

int wmt_graphic_init(void)
{
	vout_info_t *info;
	int i;
	int y_bpp,c_bpp;
	vout_t *vo;

	g_direct_path = 0;
	if (wmt_init_check())
		return -1;

	if (g_direct_path) {
		vfb_var.bits_per_pixel = 32;
	}

	vpp_mod_init();
	vpp_device_init();
	
	g_vpp.govr = p_govrh;
	g_vpp.govrh_preinit = 0;
	g_vpp.alloc_framebuf = 0; // uboot_vpp_alloc_framebuffer;
	g_vpp.mb[0] = g_fb_phy;
	g_vpp.mb[1] = g_vpp.mb[0];
	vpp_init();
	vfb_var.bits_per_pixel = (g_vpp.mb_colfmt == VDO_COL_FMT_ARGB)? 32:16;
	info = &vout_info[0];
	
	vout_check_info((g_vpp.dual_display)? info->vo_mask:VPP_VOUT_ALL,info);
#ifdef WMT_DUAL_DISPLAY_LOGO
	vfb_var.xres = 1920;
	vfb_var.yres = 1080;
#else
	vfb_var.xres = info->resx;
	vfb_var.yres = info->resy;
	vpp_get_colfmt_bpp(g_vpp.govr->fb_p->fb.col_fmt,&y_bpp,&c_bpp);
	y_bpp /= 8;
	vfb_var.xres_virtual = vpp_calc_align(vfb_var.xres,VPP_FB_WIDTH_ALIGN/y_bpp);
#endif
	
//	g_display_param.vout = ( lcd_get_dev() )? VPP_VOUT_LCD:0;
	g_display_param.vout = VPP_VOUT_LCD;
	if (g_display_param.vout == VPP_VOUT_LCD) {
		parse_pwm_params(ENV_DISPLAY_PWM, NULL);
		parse_gpio_operation(ENV_LCD_POWER);
		
		if ((g_display_vaild&PWMDEFMASK) == PWMDEFTP) {
			lcd_blt_set_pwm(g_pwm_setting.pwm_no, g_pwm_setting.duty, g_pwm_setting.period);
		} else {
			// fan : may need to check PWM power ..
			pwm_set_period(g_pwm_setting.pwm_no, g_pwm_setting.period-1);
			pwm_set_duty(g_pwm_setting.pwm_no, g_pwm_setting.duty-1);
			pwm_set_control(g_pwm_setting.pwm_no, (g_pwm_setting.duty-1)? 0x34:0x8);
			pwm_set_gpio(g_pwm_setting.pwm_no, g_pwm_setting.duty-1);
			pwm_set_scalar(g_pwm_setting.pwm_no, g_pwm_setting.scalar-1);
		}
	}

	for(i=0;i<VPP_VOUT_INFO_NUM;i++){
		info = &vout_info[i];
		if( info->vo_mask ){
			vdo_framebuf_t *fb;
			unsigned int pixel_size,line_size;

			if( info->govr == 0 )
				continue;

			fb = &info->govr->fb_p->fb;
			fb->img_w = info->resx;
			fb->img_h = info->resy;
#ifdef WMT_DUAL_DISPLAY_LOGO
			fb->fb_w = 1920;
			fb->fb_h = 1080;
#else
			fb->fb_w = vpp_calc_align(info->resx,VPP_FB_WIDTH_ALIGN/y_bpp);
			fb->fb_h = info->resy;
#endif
			DBG_MSG("[UBOOT] img(%d,%d),fb(%d,%d)\n",fb->img_w,fb->img_h,fb->fb_w,fb->fb_h);

			info->resx_virtual = fb->fb_w;
			pixel_size = y_bpp / 8;
			line_size = fb->fb_w * pixel_size;
#ifdef WMT_DUAL_DISPLAY_LOGO
			fb->y_addr = g_vpp.mb[0] + ((fb->fb_h - fb->img_h)/2 * line_size) + ((fb->fb_w - fb->img_w)/2 * pixel_size);
#else
			fb->y_addr = g_vpp.mb[0];
#endif
			fb->c_addr = 0;
		}
	}

	DBG_MSG("vout config\n");
	for(i=0;i<VPP_VOUT_INFO_NUM;i++){
		info = &vout_info[i];
		if( info->vo_mask ){
			vpp_config(info);
			if( g_vpp.dual_display == 0 ){
				break;
			}
		}
	}

	// clear frame buffer
	i = vfb_var.xres * vfb_var.yres * vfb_var.bits_per_pixel / 8;
	memset((void *)g_vpp.mb[0],0,i);

	// enable vout
	if( g_vpp.virtual_display ){
		extern int hdmi_cur_plugin;
		int plugin;

		plugin = hdmi_cur_plugin;	// use plug status in config to avoid reconfig TV reset signal
		// plugin = vout_chkplug(VPP_VOUT_NUM_HDMI);
		vout_change_status(vout_get_entry(VPP_VOUT_NUM_HDMI),VPP_VOUT_STS_BLANK,(plugin)?0:1);
		vout_change_status(vout_get_entry(VPP_VOUT_NUM_DVI),VPP_VOUT_STS_BLANK,(plugin)?1:0);
	}
	
	for(i=0;i<VPP_VOUT_NUM;i++){
		if( (vo = vout_get_entry(i)) ){
			vout_set_blank((0x1 << i),(vo->status & VPP_VOUT_STS_BLANK)?1:0);
		}
	}
	
#if 0
	p_govrh->dump_reg();
	p_govrh2->dump_reg();
	hdmi_reg_dump();
#endif
	g_display_vaild |= DISPLAY_ENABLE;
	return 0;
}
#undef WMT_DISPLAY

