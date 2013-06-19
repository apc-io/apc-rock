/*++
linux/arch/arm/mach-wmt/wmt_cpufreq.c

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

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/cpufreq.h>
#include <mach/hardware.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>

#include <mach/wmt_env.h>

#define WM3481_A2_ID 0x34810103

//#define DEBUG
#ifdef  DEBUG
static int dbg_mask = 1;
module_param(dbg_mask, int, S_IRUGO | S_IWUSR);
#define fq_dbg(fmt, args...) \
        do {\
            if (dbg_mask) \
                printk(KERN_ERR "[%s]_%d: " fmt, __func__ , __LINE__, ## args);\
        } while(0)
#define fq_trace() \
        do {\
            if (dbg_mask) \
                printk(KERN_ERR "trace in %s %d\n", __func__, __LINE__);\
        } while(0)
#else
#define fq_dbg(fmt, args...)
#define fq_trace()
#endif

#define DVFS_TABLE_NUM_MAX  20
#define PMC_BASE            PM_CTRL_BASE_ADDR
#define PLLA_FREQ_KHZ       (24 * 1000)
#define PMC_PLLA            (PM_CTRL_BASE_ADDR + 0x200)
#define ARM_DIV_OFFSET      (PM_CTRL_BASE_ADDR + 0x300)

struct wmt_dvfs_table {
    unsigned int freq;
    unsigned int vol;
    unsigned int l2c_div;
    unsigned int l2c_tag;
    unsigned int l2c_data;
    unsigned int axi;
    int index;
    struct list_head node;
};

struct wmt_dvfs_driver_data {
    unsigned int tbl_num;
    unsigned int sample_rate;
    struct list_head wmt_dvfs_list;
    struct wmt_dvfs_table *dvfs_table;
    struct cpufreq_frequency_table *freq_table;
};
static struct wmt_dvfs_driver_data wmt_dvfs_drvdata;
static struct regulator *re;

extern int wmt_getsyspara(char *varname, unsigned char *varval, int *varlenex);

spinlock_t	wmt_cpufreq_lock;
char	use_dvfs;
char	use_dvfs_debug;

static int wmt_init_cpufreq_table(struct wmt_dvfs_driver_data *drv_data)
{
    int i = 0;
    struct wmt_dvfs_table *dvfs_tbl = NULL;
    struct cpufreq_frequency_table *freq_tbl = NULL;
    
//    freq_tbl = kzalloc(sizeof(struct wmt_dvfs_table) * (drv_data->tbl_num + 1),
//                                                                    GFP_KERNEL);
    freq_tbl = kzalloc(sizeof(struct cpufreq_frequency_table) * (drv_data->tbl_num + 1),
                                                                    GFP_KERNEL);    
    if (freq_tbl == NULL) {
	    printk(KERN_ERR "%s: failed to allocate frequency table\n", __func__);
        return -ENOMEM;
    }

    drv_data->freq_table = freq_tbl;
    list_for_each_entry(dvfs_tbl, &drv_data->wmt_dvfs_list, node) {
        fq_dbg("freq_table[%d]:freq->%dKhz", i, dvfs_tbl->freq);
        freq_tbl[i].index = i;
        freq_tbl[i].frequency = dvfs_tbl->freq;
        i++;
    }
    /* the last element must be initialized to CPUFREQ_TABLE_END */
	freq_tbl[i].index = i;
	freq_tbl[i].frequency = CPUFREQ_TABLE_END;

    return 0;
}

static void wmt_exit_cpufreq_table(struct wmt_dvfs_driver_data *drv_data)
{
    if (!drv_data)
        return;
    if (drv_data->freq_table)
        kfree(drv_data->freq_table);
    if (drv_data->dvfs_table)
        kfree(drv_data->dvfs_table);

    return ;
}

static unsigned int wmt_getspeed(unsigned int cpu)
{
    int freq = 0;
 
    if (cpu)
        return 0;

    //spin_lock(&wmt_cpufreq_lock);
    freq = auto_pll_divisor(DEV_ARM, GET_FREQ, 0, 0) / 1000;
    //spin_unlock(&wmt_cpufreq_lock);

    if (freq < 0)
        freq = 0;

    return freq;
}

static struct wmt_dvfs_table *find_freq_ceil(unsigned int *target)
{
    struct wmt_dvfs_table *dvfs_tbl = NULL;
    unsigned int freq = *target;

    list_for_each_entry(dvfs_tbl, &wmt_dvfs_drvdata.wmt_dvfs_list, node) {
        if (dvfs_tbl->freq >= freq) {
            *target = dvfs_tbl->freq;
            return dvfs_tbl;
        }
    }

    return NULL;
}

static struct wmt_dvfs_table *find_freq_floor(unsigned int *target)
{
    struct wmt_dvfs_table *dvfs_tbl = NULL;
    unsigned int freq = *target;

    list_for_each_entry_reverse(dvfs_tbl, &wmt_dvfs_drvdata.wmt_dvfs_list, node) {
        if (dvfs_tbl->freq <= freq) {
            *target = dvfs_tbl->freq;
            return dvfs_tbl;
        }
    }

    return NULL;
}

static struct wmt_dvfs_table *
wmt_recalc_target_freq(unsigned *target_freq, unsigned relation)
{
    struct wmt_dvfs_table *dvfs_tbl = NULL;

    if (!target_freq)
        return NULL;

    switch (relation) {
        case CPUFREQ_RELATION_L:
			/* Try to select a new_freq higher than or equal target_freq */
            dvfs_tbl = find_freq_ceil(target_freq);
            break;
        case CPUFREQ_RELATION_H:
			/* Try to select a new_freq lower than or equal target_freq */
            dvfs_tbl = find_freq_floor(target_freq);
            break;
        default:
            dvfs_tbl = NULL;
            break;
    }

    return dvfs_tbl;
}

static int get_arm_plla_param(void)
{
    unsigned ft, df, dr, dq, div, tmp;

    tmp =  *(volatile unsigned int *)PMC_PLLA;
    fq_dbg("PMC PLLA REG IS 0x%08x\n", tmp);

    ft = (tmp >> 24) & 0x03; // bit24 ~ bit26
    df = (tmp >> 16) & 0xFF; // bit16 ~ bit23
    dr = (tmp >>  8) & 0x1F; // bit8  ~ bit12  
    dq = tmp & 0x03;
    
    tmp = *(volatile unsigned int *)ARM_DIV_OFFSET;
    div = tmp & 0x1F;

    fq_dbg("ft:%d, df:%d, dr:%d, dq:%d, div:%d\n", ft, df, dr, dq, div);
    return 0;
}

static int wmt_change_plla_freq(unsigned target)
{
    int ret = 0;

    ret = auto_pll_divisor(DEV_ARM, SET_PLL, 1, target);
    return ret;
}

extern void FAN5365_set_pmic_volt(unsigned int volt);

static int wmt_change_plla_table(struct wmt_dvfs_table *plla_table, unsigned relation)
{
    int ret = 0;
    struct plla_param plla_env;

    switch (relation) {
        case CPUFREQ_RELATION_L:
            plla_env.plla_clk = (plla_table->freq/1000);
            plla_env.arm_div = 1;    
            plla_env.l2c_div = plla_table->l2c_div;   
            plla_env.l2c_tag_div= plla_table->l2c_tag;
            plla_env.l2c_data_div= plla_table->l2c_data;
            plla_env.axi_div= plla_table->axi;
            
	    if (use_dvfs_debug)
	      printk("change to L %dKhz\n", plla_table->freq);   
	    
            ret = set_plla_divisor(&plla_env);
            /*
            if (plla_table->vol) {
//				printk("change to L %dmV\n", plla_table->vol);
            	FAN5365_set_pmic_volt(plla_table->vol*10);
		    }
            */
            if (plla_table->vol) {
		if (use_dvfs_debug)
		  printk("pllal_table = %d\n", plla_table->vol*1000);
                regulator_set_voltage(re, plla_table->vol*1000, plla_table->vol*1000);
	    }

            break;
        case CPUFREQ_RELATION_H:
            /*
		    if (plla_table->vol) {
//				printk("change to H %dmV\n", plla_table->vol);
            	FAN5365_set_pmic_volt(plla_table->vol*10);
		    }
            */
            if (plla_table->vol) {
		if (use_dvfs_debug)
		  printk("pllah_table = %d\n", plla_table->vol*1000);
                regulator_set_voltage(re, plla_table->vol*1000, plla_table->vol*1000);
		}
		
						if (regulator_is_enabled(re) < 0) {
							if (use_dvfs_debug)
							  printk("xxx regulator is disabled\n");
														
							ret = -EINVAL;
						} else {
            
            plla_env.plla_clk = (plla_table->freq/1000);
            plla_env.arm_div = 1;    
            plla_env.l2c_div = plla_table->l2c_div;   
            plla_env.l2c_tag_div= plla_table->l2c_tag;
            plla_env.l2c_data_div= plla_table->l2c_data;
            plla_env.axi_div= plla_table->axi;
	    if (use_dvfs_debug)
	      printk("change to H %dKhz\n", plla_table->freq);
            ret = set_plla_divisor(&plla_env);
          	}

            break;
        default:
            break;
    }
    
    return ret;
}

/* target is calculated by cpufreq governor, unit Khz */
int wmt_lock_dvfs=0;
int wmt_during_dvfs = 0;

int
wmt_suspend_target(unsigned target, unsigned relation)
{
    int ret = 0;
    struct wmt_dvfs_table *dvfs_tbl = NULL;
  
	while(wmt_during_dvfs)
		msleep(0);
//    spin_lock(&wmt_cpufreq_lock);
    
    /* find out (freq, voltage) pair to do dvfs */
    dvfs_tbl= wmt_recalc_target_freq(&target, relation);
    if (dvfs_tbl == NULL) {
        fq_dbg("Can not change to target_freq:%dKhz", target);
        ret = -EINVAL;
        goto out;
    }
    fq_dbg("recalculated target freq is %dMhz\n", target / 1000);

    get_arm_plla_param();
    ret = wmt_change_plla_freq(target);
//    ret = wmt_change_plla_table(dvfs_tbl, relation);
    
    fq_dbg("change to %dKhz\n", ret / 1000);    
    if (use_dvfs_debug)
      printk("change to %dKhz\n", ret / 1000);        
    
    regulator_set_voltage(re, dvfs_tbl->vol*1000, dvfs_tbl->vol*1000);
    get_arm_plla_param();

    if (ret < 0) {
        ret = -ENODATA;
        fq_dbg("wmt_cpufreq: auto_pll_divisor failed\n");
    }
    ret = 0;

out:
//    spin_unlock(&wmt_cpufreq_lock);
    return ret;
}


/* target is calculated by cpufreq governor, unit Khz */
static int
wmt_target(struct cpufreq_policy *policy, unsigned target, unsigned relation)
{
    int ret = 0;
    struct cpufreq_freqs freqs;
    struct wmt_dvfs_table *dvfs_tbl = NULL;
 
    fq_dbg("cpu freq:%dMhz now, need %s to %dMhz\n", wmt_getspeed(0) / 1000,
              (relation == CPUFREQ_RELATION_L) ? "DOWN" : "UP", target / 1000);

		wmt_during_dvfs = 1;
    //spin_lock(&wmt_cpufreq_lock);
    if (policy->cpu) {
        ret = -EINVAL;
        goto out;
    }

    if (wmt_lock_dvfs) {
        ret = -EINVAL;
        goto out;
    }	
	
    if (regulator_is_enabled(re) < 0) {
        ret = -EINVAL;
        goto out;
    }

    /* Ensure desired rate is within allowed range.  Some govenors
     * (ondemand) will just pass target_freq=0 to get the minimum. */
    if (target < policy->min)
        target = policy->min;
    if (target > policy->max)
        target = policy->max;

    /* find out (freq, voltage) pair to do dvfs */
    dvfs_tbl= wmt_recalc_target_freq(&target, relation);
    if (dvfs_tbl == NULL) {
        fq_dbg("Can not change to target_freq:%dKhz", target);
        ret = -EINVAL;
        goto out;
    }
    fq_dbg("recalculated target freq is %dMhz\n", target / 1000);

    freqs.cpu = policy->cpu;
    freqs.new = target;
    freqs.old = wmt_getspeed(policy->cpu); 

    if (freqs.new == freqs.old) {
        ret = 0;
        goto out;
    }
    else if (freqs.new > freqs.old)
		relation = CPUFREQ_RELATION_H;
    else
		relation = CPUFREQ_RELATION_L;

    /* actually we just scaling CPU frequency here */
    cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

    get_arm_plla_param();
//    ret = wmt_change_plla_freq(target);
    ret = wmt_change_plla_table(dvfs_tbl, relation);
    get_arm_plla_param();
    fq_dbg("change to %dKhz\n", ret / 1000);

    if (ret < 0) {
        ret = -ENODATA;
        fq_dbg("wmt_cpufreq: auto_pll_divisor failed\n");
    }
    ret = 0;
    cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

out:
    //	spin_unlock(&wmt_cpufreq_lock);
    fq_dbg("cpu freq scaled to %dMhz now\n\n", wmt_getspeed(0) / 1000);
    wmt_during_dvfs = 0;
    return ret;
}

static int wmt_verify_speed(struct cpufreq_policy *policy)
{
    int ret = 0;
    struct cpufreq_frequency_table *freq_tbl = wmt_dvfs_drvdata.freq_table;

    if (policy->cpu)
        return -EINVAL;

    if (NULL == freq_tbl)
        ret = -EINVAL;
    else
        ret = cpufreq_frequency_table_verify(policy, freq_tbl);

    return ret;
}

static int __init wmt_cpu_init(struct cpufreq_policy *policy)
{
    int ret = 0;
    struct cpufreq_frequency_table *wmt_freq_tbl = NULL;
    
    if (policy->cpu)
		return -EINVAL;

    policy->cur = wmt_getspeed(policy->cpu);
    policy->min = wmt_getspeed(policy->cpu);
    policy->max = wmt_getspeed(policy->cpu);
    
    /* generate cpu frequency table here */
    wmt_init_cpufreq_table(&wmt_dvfs_drvdata);
    wmt_freq_tbl = wmt_dvfs_drvdata.freq_table;
    if (NULL == wmt_freq_tbl) {
        printk(KERN_ERR "wmt_cpufreq create wmt_freq_tbl failed\n");
        return -EINVAL;
    }

    /* 
     * check each frequency and find max_freq 
     * min_freq in the table, then set:
     *  policy->min = policy->cpuinfo.min_freq = min_freq;
     *  policy->max = policy->cpuinfo.max_freq = max_freq; 
     */
    ret = cpufreq_frequency_table_cpuinfo(policy, wmt_freq_tbl);
    if (0 == ret)
        cpufreq_frequency_table_get_attr(wmt_freq_tbl, policy->cpu);

    /*
     * 1. make sure current frequency be covered in cpufreq_table
     * 2. change cpu frequency to policy-max for fast booting
     */
    wmt_target(policy, policy->max, CPUFREQ_RELATION_H);

    policy->cur = wmt_getspeed(policy->cpu);
    policy->cpuinfo.transition_latency = wmt_dvfs_drvdata.sample_rate;
    
    fq_dbg("p->max:%d, p->min:%d, c->max=%d, c->min=%d, current:%d\n",
            policy->max, policy->min, policy->cpuinfo.max_freq, 
            policy->cpuinfo.min_freq, wmt_getspeed(0)); 

    return ret;
}

static int wmt_cpu_exit(struct cpufreq_policy *policy)
{
    wmt_exit_cpufreq_table(&wmt_dvfs_drvdata);
    memset(&wmt_dvfs_drvdata, sizeof(wmt_dvfs_drvdata), 0x00);
    cpufreq_frequency_table_put_attr(policy->cpu);

    return 0;
}

static struct freq_attr *wmt_cpufreq_attr[] = {
        &cpufreq_freq_attr_scaling_available_freqs,
        NULL,
};

#ifdef CONFIG_PM


static int wmt_cpufreq_suspend(struct cpufreq_policy *policy)
{

	printk("xxxx wmt_cpufreq_suspend\n");
	return 0;
}

static int wmt_cpufreq_resume(struct cpufreq_policy *policy)
{
	printk("xxxx wmt_cpufreq_resume\n");
	return 0;
}
#else
#define wmt_cpufreq_suspend NULL
#define wmt_cpufreq_resume NULL
#endif

static struct cpufreq_driver wmt_cpufreq_driver = {
    .name       = "wmt_cpufreq",
    .owner      = THIS_MODULE,
    .flags		= CPUFREQ_STICKY,
    .init       = wmt_cpu_init,
    .verify     = wmt_verify_speed,
    .target     = wmt_target,
    .get        = wmt_getspeed,
    .exit       = wmt_cpu_exit,
    .attr       = wmt_cpufreq_attr,
		.suspend	= wmt_cpufreq_suspend,
		.resume		= wmt_cpufreq_resume,    
};

static int __init wmt_cpufreq_check_env(void)
{
    int i = 0;
    int ret = 0;
    int varlen = 512;
    char *ptr = NULL;
    unsigned int tbl_num = 0;
    unsigned int drv_en = 0;
    unsigned int sample_rate = 0;
    unsigned int freq = 0;
    unsigned int voltage = 0;
    unsigned int l2c_div = 0;
    unsigned int l2c_tag = 0;
    unsigned int l2c_data = 0;
    unsigned int axi = 0;
    struct wmt_dvfs_table *wmt_dvfs_tbl = NULL;
    unsigned char buf[512] = {0};

    /* uboot env name is: wmt.cpufreq.param/wmt.dvfs.param, format is:                
        <enable>:<sample_rate>:<table_number>:<[freq,voltage,l2c_div,l2c_tag,l2c_data,axi]*>
        >        
        Example:        
         <enable>:<sample_rate>:<table_number>:<[freq,voltage,l2c_div,l2c_tag,l2c_data,axi]*> 
    */
    
    use_dvfs = 0;
    use_dvfs_debug = 0;
    
    ret = wmt_getsyspara("wmt.cpufreq.param", buf, &varlen);                                                                    
    if (ret) {
        printk(KERN_INFO "Can not find uboot env wmt.cpufreq.param\n");
        ret = -ENODATA;
        goto out;
    }
    fq_dbg("wmt.cpufreq.param:%s\n", buf);

    sscanf(buf, "%x:%d:%d", &drv_en, &sample_rate, &tbl_num);
    if (!drv_en) {
        printk(KERN_INFO "wmt cpufreq disaled\n");
        ret = -ENODEV;
        goto out;
    }

    /* 2 dvfs table at least */
    if (tbl_num < 2) {
        printk(KERN_INFO "No frequency information found\n");
        ret = -ENODATA;
        goto out;
    }
    if (tbl_num > DVFS_TABLE_NUM_MAX)
        tbl_num = DVFS_TABLE_NUM_MAX;

    wmt_dvfs_tbl = kzalloc(sizeof(struct wmt_dvfs_table) * tbl_num, GFP_KERNEL);
    if (!wmt_dvfs_tbl) {
        ret = -ENOMEM;
        goto out;
    }
    wmt_dvfs_drvdata.tbl_num = tbl_num;
    wmt_dvfs_drvdata.sample_rate = sample_rate;
    wmt_dvfs_drvdata.dvfs_table = wmt_dvfs_tbl;
    INIT_LIST_HEAD(&wmt_dvfs_drvdata.wmt_dvfs_list);

    /* copy freq&vol info from uboot env to wmt_dvfs_table */
    ptr = buf;
    for (i = 0; i < tbl_num; i++) {
        strsep(&ptr, "[");
        sscanf(ptr, "%d,%d,%d,%d,%d,%d]:[", &freq, &voltage, &l2c_div, &l2c_tag, &l2c_data, &axi);
        wmt_dvfs_tbl[i].freq = freq*1000;
        wmt_dvfs_tbl[i].vol  = voltage;
        wmt_dvfs_tbl[i].l2c_div = l2c_div;
        wmt_dvfs_tbl[i].l2c_tag  = l2c_tag;
        wmt_dvfs_tbl[i].l2c_data = l2c_data;
        wmt_dvfs_tbl[i].axi  = axi;        
        wmt_dvfs_tbl[i].index = i;
        INIT_LIST_HEAD(&wmt_dvfs_tbl[i].node);
        list_add_tail(&wmt_dvfs_tbl[i].node, &wmt_dvfs_drvdata.wmt_dvfs_list);
        fq_dbg("dvfs_table[%d]: freq %dMhz, voltage %dmV l2c_div %d l2c_tag %d l2c_data %d axi %d\n", i, freq, voltage, l2c_div, l2c_tag, l2c_data, axi);
    }
    use_dvfs = 1;
	wmt_lock_dvfs = 0;
	wmt_during_dvfs = 0;

    if (drv_en & 0x10)
      use_dvfs_debug = 1;    
    
out:
    return ret;
}

static int __init wmt_cpufreq_driver_init(void)
{
    int ret = 0;
		unsigned int chip_id = 0;
		unsigned int bondingid = 0;

    /* if cpufreq disabled, cpu will always run at current frequency
     * which defined in wmt.plla.param */
    spin_lock_init(&wmt_cpufreq_lock);
    ret = wmt_cpufreq_check_env();
    if (ret) {
        printk(KERN_WARNING "wmt_cpufreq check env failed, current cpu "
                                    "frequency is %dKhz\n", wmt_getspeed(0));
        goto out;
    }
    
    wmt_getsocinfo(&chip_id, &bondingid);
    
    if (((chip_id & 0xffff0000) == (WM3481_A2_ID & 0xffff0000)) && ((chip_id & 0xffff) < (WM3481_A2_ID & 0xffff))) {
      ret = -ENODEV;
      goto out;
    }
    re = regulator_get(NULL, "wmt_corepower");
    ret = cpufreq_register_driver(&wmt_cpufreq_driver);

out:
    return ret;
}
late_initcall(wmt_cpufreq_driver_init);

static void __exit wmt_cpufreq_driver_exit(void)
{
    cpufreq_unregister_driver(&wmt_cpufreq_driver);
}
module_exit(wmt_cpufreq_driver_exit);

MODULE_AUTHOR("WonderMedia Technologies, Inc");
MODULE_DESCRIPTION("WMT CPU frequency driver");
MODULE_LICENSE("Dual BSD/GPL");
