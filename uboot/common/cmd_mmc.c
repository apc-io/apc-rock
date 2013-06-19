/*
 * (C) Copyright 2003
 * Kyle Harris, kharris@nexus-tech.net
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <command.h>


#if (CONFIG_COMMANDS & CFG_CMD_MMC)

#include <mmc.h>
#include <environment.h>
#include <fastboot.h>
#include <part.h>

static   block_dev_desc_t *mmc_cur_dev = NULL;
static  unsigned long mmc_part_offset =0;
static  int mmc_cur_part =1;
int mmc_register_device(block_dev_desc_t *dev_desc, int part_no);


static int sd_check_ctrlc(void)
{
    extern int ctrlc (void);
    if( ctrlc()){
        printf("Abort\n");
        return 1;
    }
    return 0;
}

int do_mmc_wait_insert(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    SD_Controller_Powerup();
	int insert = simple_strtoul(argv[1], NULL, 10);
	if(insert){
		printf("waiting insert SD card\n");
		while(SD_card_inserted()!=1){
			if( sd_check_ctrlc())
                    return -1;
		}
		return 0;

	}else{
		printf("wainting remove SD card\n");
		while(SD_card_inserted()!=0){
			if( sd_check_ctrlc())
                    return -1;
            udelay(500000);//delay 500ms
		}
		
		return 0;
	}
}

int do_mmc (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int device_num = 1;
	ulong dev_id = 0;
	
	if (argc <= 1) {
		printf("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}
	
	if (argc == 2)
		dev_id = simple_strtoul(argv[1], NULL, 16);
	if (dev_id == 0 || dev_id == 1 || dev_id == 2 || dev_id == 3) //add dev_id == 3 by eason 2012/3/27
		device_num = dev_id;

	if (mmc_init (1, (int)device_num) != 0) {
		printf ("No MMC card found\n");
		return 1;
	}
	return 0;
}

int do_mmc_read (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int device_num = 1;
	ulong dev_id = 0;
	ulong addr,block_num,bytes,blk_cnt;
	ulong ret;
		
	if (argc < 4) {
		printf("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}
	
	/*get device id*/
	dev_id = simple_strtoul(argv[1], NULL, 16);
	if (dev_id == 0 || dev_id == 1 || dev_id == 2 || dev_id == 3) //add dev_id == 3 by eason 2012/3/27
		device_num = dev_id;
	else {
		printf("dev_id Invalid\n");
		return 1;
	}

	/*get memory address*/
	addr = simple_strtoul (argv[2], NULL, 16);
	if (addr < 0) {
		printf("addr Invalid\n");
		return 1;
	}

	/*get card block address*/
	block_num = simple_strtoul (argv[3], NULL, 16);
	if (block_num < 0) {
		printf("block_num Invalid\n");
		return 1;
	}

	/*get transfer size is bytes*/
	bytes = simple_strtoul (argv[4], NULL, 16);
	if (bytes < 0) {
		printf("bytes Invalid\n");
		return 1;
	}

	if (bytes == 0)
		return 0;

	/*calculate transfer block count*/
	blk_cnt = (bytes / 512);
	if (bytes % 512)
		blk_cnt++;
	
	//printf("device_num = %x block_num = %x addr =  %x bytes = %x blk_cnt =%x\n",device_num,block_num,addr,bytes,blk_cnt);

	ret = mmc_bread(device_num,block_num,blk_cnt,(ulong *)addr);

	if (ret != blk_cnt) {
		printf("Read Data Fail\n");
		return 1;
	} else {
		printf("Read Data Success\n");
	}
		

	return 0;
}

int do_mmc_write (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int device_num = 1;
	ulong dev_id = 0;
	ulong addr,block_num,bytes,blk_cnt;
	ulong ret;
		
	if (argc < 4) {
		printf("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}
	
	/*get device id*/
	dev_id = simple_strtoul(argv[1], NULL, 16);
	if (dev_id == 0 || dev_id == 1 || dev_id == 2 || dev_id == 3)//add dev_id == 3 by eason 2012/3/27
		device_num = dev_id;
	else {
		printf("dev_id Invalid\n");
		return 1;
	}

	/*get memory address*/
	addr = simple_strtoul (argv[2], NULL, 16);
	if (addr < 0) {
		printf("addr Invalid\n");
		return 1;
	}

	/*get card block address*/
	block_num = simple_strtoul (argv[3], NULL, 16);
	if (block_num < 0) {
		printf("block_num Invalid\n");
		return 1;
	}

	/*get transfer size is bytes*/
	bytes = simple_strtoul (argv[4], NULL, 16);
	if (bytes < 0) {
		printf("bytes Invalid\n");
		return 1;
	}
	
	if (bytes == 0)
		return 0;

	/*calculate transfer block count*/
	blk_cnt = (bytes / 512);
	if (bytes % 512)
		blk_cnt++;
	
	//printf("device_num = %x block_num = %x addr =  %x bytes = %x blk_cnt =%x\n",device_num,block_num,addr,bytes,blk_cnt);
	ret = mmc_bwrite(device_num,block_num,blk_cnt,(ulong *)addr);

	if (ret != blk_cnt) {
		printf("Write Data Fail\n");
		return 1;
	} else {
		printf("Write Data Success\n");
	}
		

	return 0;
}

 void memdump (void *pv, int num)
{
	unsigned int  tmp,ba;
	ba =(unsigned int) pv;
	for (tmp = 0;tmp < num/16; tmp++)
	printf("[%8.8x]%8.8x %8.8x %8.8x %8.8x\n",(0x10*tmp+ba),*(volatile unsigned int *)(ba+(tmp*0x10)),*(volatile unsigned int *)(ba+(tmp*0x10)+0x4),*(volatile unsigned int *)(ba+(tmp*0x10)+0x8),*(volatile unsigned int *)(ba+(tmp*0x10)+0xc));
}
extern int do_setenv ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
int do_saveenv (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
extern block_dev_desc_t *get_dev (char*, int);

int do_mmc_read_img (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int device_num = 1;
	int dev_id = 0;
	ulong kernel_addr,ramdisk_addr;
	ulong blk_cnt_kenel, blk_cnt_ramdisk, page_cnt_kenel, page_cnt_ramdisk;
	ulong ret;
	static unsigned char data[512];
	block_dev_desc_t *dev_desc=NULL;
	int dev=0;
	ulong part=1;
	char *ep;
	unsigned int  page_size, kernel_size, ramdisk_size, total_size;
	
	char var[64], val[32];
	char *setenv[4]  = { "setenv", NULL, NULL, NULL, };
	char *saveenv[2] = { "setenv", NULL, };
	setenv[1] = var;
	setenv[2] = val;


	/*printf("%s\n", __FUNCTION__);*/

	if (argc < 4) {
		printf("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}
	/*get device id*/
	dev_id = (int)simple_strtoul (argv[2], &ep, 16);
	if (dev_id == 0 || dev_id == 1 || dev_id == 2 || dev_id == 3) 
		device_num = dev_id;
	else {
		printf("dev_id Invalid\n");
		return 1;
	}
	/*get mmc dev descriptor*/
	dev_desc=get_dev(argv[1], dev_id);
	if (dev_desc==NULL) {
		puts ("\n** Invalid boot device **\n");
		return 1;
	}
	if (*ep) {
		if (*ep != ':') {
			puts ("\n** Invalid boot device, use `dev[:part]' **\n");
			return 1;
		}
		part = (int)simple_strtoul(++ep, NULL, 16);
	}


	/*printf("dev_id:%x,part:%x\n",dev_id,part);*/
	/* init mmc controller */	
	if (mmc_init(1, device_num)) {
		printf("mmc init failed?\n");
		return 1;
	}
	if (mmc_register_device(dev_desc, part) != 0) {
		printf ("\n** Unable to use %s %d:%d for fatload **\n", argv[1], dev_id, part);
		return 1;
	}

	/*get kernel memory address*/
	kernel_addr = simple_strtoul (argv[3], NULL, 16);
	if (kernel_addr < 0) {
		printf("kernel addr Invalid\n");
		return 1;
	}

	/*get ramdisk memory address*/
	ramdisk_addr = simple_strtoul (argv[4], NULL, 16);
	if (ramdisk_addr < 0) {
		printf("ramdisk addr  Invalid\n");
		return 1;
	}
	/*mmc block read partition offset sectors*/
	ret = mmc_bread(device_num, mmc_part_offset, 1,  (void *)data);
	if (ret == -1  ) {
		printf("Read Data Fail\n");
		return 1;
	} else {
		printf("Read Data Success\n");
	}

	/*memdump ((void *) data, 512);*/
	struct fastboot_boot_img_hdr *fb_hdr = (struct fastboot_boot_img_hdr *)data;
	
	if (memcmp(data, "ANDROID!", 8)) {
		printf(" boot.img  partition table not found\n");
		return -1;
	}
	/*kernel size change to sectors*/
	kernel_size = fb_hdr->kernel_size ;
	ramdisk_size = fb_hdr->ramdisk_size;
	page_size = fb_hdr->page_size ; 
	total_size = 1*page_size + kernel_size + ramdisk_size;

	/* calculate transfer block count */
	blk_cnt_kenel= (fb_hdr->kernel_size / 512);
	if (fb_hdr->kernel_size % 512)
		blk_cnt_kenel++;
	
	page_cnt_kenel= (fb_hdr->kernel_size / page_size);
	if (fb_hdr->kernel_size % page_size)
		page_cnt_kenel++;

	/* load kernel image from mmc boot image to kernel_addr */
	/*printf(" mmc_part_offset + page_size:0x%x, fb_hdr->kernel_size:0x%x, mem kernel_addr:0x%x\n", 
		mmc_part_offset + page_size / 512, fb_hdr->kernel_size, kernel_addr);*/
    	ret = mmc_bread(device_num, mmc_part_offset + page_size / 512, blk_cnt_kenel, kernel_addr);
	if (ret == -1  ) {
		printf("Read Kernel Data Fail\n");
		return 1;
	} else {
		printf("Read Kernel Data Success\n");
	}
	blk_cnt_ramdisk= (fb_hdr->ramdisk_size / 512);
	if (fb_hdr->ramdisk_size % 512)
		blk_cnt_ramdisk++;
	
	page_cnt_ramdisk= (fb_hdr->kernel_size / page_size);
	if (fb_hdr->kernel_size % page_size)
		page_cnt_ramdisk++;

	/* load ramdisk image from mmc boot image to ramdisk_addr */
	/*printf("mmc_part_offset + page_size + blk_cnt_kenel:0x%x, fb_hdr->ramdisk_size:0x%x, mem ramdisk_addr:0x%x\n",
			mmc_part_offset + page_size /512 + page_cnt_kenel *(page_size/512), fb_hdr->ramdisk_size, ramdisk_addr);*/
    	ret = mmc_bread(device_num, mmc_part_offset + page_size /512 + page_cnt_kenel *(page_size /512), blk_cnt_ramdisk, ramdisk_addr);
	if (ret == -1  ) {
		printf("Read Ramdisk Data Fail\n");
		return 1;
	} else {
		printf("Read Ramdisk Data Success\n");
	}
	
	/*save ramdisk size to env argv[4]*/
	sprintf (var, "%s", argv[5]);
	sprintf (val, "%x", fb_hdr->ramdisk_size);
	do_setenv (NULL, 0, 3, setenv);

	printf("%s %s %s\n", setenv[0], setenv[1], setenv[2]);
	/*do_saveenv (NULL, 0, 1, saveenv);*/

	return 0;
}


int
mmc_register_device(block_dev_desc_t *dev_desc, int part_no)
{
	unsigned char buffer[0x200];
	disk_partition_t info;

	if (!dev_desc->block_read)
		return -1;
	mmc_cur_dev=dev_desc;
	/* check if we have a MBR (on floppies we have only a PBR) */
	if (dev_desc->block_read (dev_desc->dev, 0, 1, (ulong *) buffer) != 1) {
		printf ("** Can't read from device %d **\n", dev_desc->dev);
		return -1;
	}

	if(!get_partition_info(dev_desc, part_no, &info)) {
			mmc_part_offset = info.start;
			mmc_cur_part = part_no;
			printf("part_offset : %x, cur_part : %x\n", mmc_part_offset, mmc_cur_part);
	} else {
#if 1
		printf ("** Partition %d not valid on device %d **\n",part_no,dev_desc->dev);
		return -1;
#else

		/* FIXME we need to determine the start block of the
		 * partition where the boot.img partition resides. This can be done
		 * by using the get_partition_info routine. For this
		 * purpose the libpart must be included.
		 */
		part_offset=32;
		cur_part = 1;
#endif
	}
	return 0;
}


U_BOOT_CMD(
	mmcinit,	2,	1,	do_mmc,
	"mmcinit - init mmc card\n"
	"  mmcinit 0 -- init mmc device 0 \n"
	"  mmcinit 1 -- init mmc device 1 \n"
	"  mmcinit 2 -- init mmc device 2 \n"
	"  mmcinit 3 -- init mmc device 3 \n",
	"mmcinit - init mmc card\n"
	"  mmcinit 0 -- init mmc device 0 \n"
	"  mmcinit 1 -- init mmc device 1 \n"
	"  mmcinit 2 -- init mmc device 2 \n"
	"  mmcinit 2 -- init mmc device 3 \n"
);

U_BOOT_CMD(
	mmcread,	5,	1,	do_mmc_read,
	"mmcread - read data from SD/MMC card\n"
	"  <dev_id> <addr> <block_num> <bytes>\n"
	"   -read data from SD/MMC card block address 'block_num' on 'dev_id'\n"
	"    to memory address 'addr' size is 'bytes'\n",
	"mmcread - read data from SD/MMC card\n"
	"  <dev_id> <addr> <block_num> <bytes>\n"
	"   -read data from SD/MMC card block address 'block_num' on 'dev_id'\n"
	"    to memory address 'addr' size is 'bytes'\n"
);

U_BOOT_CMD(
	mmcwrite,	5,	1,	do_mmc_write,
	"mmcwrite - write data to SD/MMC card\n"
	"  <dev_id> <addr> <block_num> <bytes>\n"
	"   -write data to SD/MMC card block address 'block_num' on 'dev_id'\n"
	"    from memory address 'addr' size is 'bytes'\n",
	"mmcwrite - write data to SD/MMC card\n"
	"  <dev_id> <addr> <block_num> <bytes>\n"
	"   -write data to SD/MMC card block address 'block_num' on 'dev_id'\n"
	"    from memory address 'addr' size is 'bytes'\n"
);


U_BOOT_CMD(
	sdwaitins,	2,	1,	do_mmc_wait_insert,
	"sdwaitins - wait sd card inserted or removed\n"
	"sdwaitins 0 -- waiting removed\n"
	"sdwaitins 1 -- waiting inserted\n",
	"sdwaitins - wait sd card inserted or removed\n"
	"sdwaitins 0 -- waiting removed\n"
	"sdwaitins 1 -- waiting inserted\n"
);
//Charles
U_BOOT_CMD(
	mmcreadimg,	6,	1,	do_mmc_read_img,
	"mmcreadimg - read boot.img or recovery.img from SD/MMC card\n"
	"  <dev_id[:partition_no]> <kernel_addr> <ramdisk_addr> <ramdisk_sizes>\n"
	"   -read boot.img from SD/MMC card partition on 'dev_id'\n"
	"    to memory address 'addr' size is 'bytes'\n",
	"mmcreadimg - read boot.img or recovery.img from SD/MMC card\n"
	"  <dev_id[:partition_no]> <kernel_addr> <ramdisk_addr> <ramdisk_sizes>\n"
	"   -read boot.img from SD/MMC card partition on 'dev_id'\n"
	"    to memory address 'addr' size is 'bytes'\n"
);

#endif	/* CFG_CMD_MMC */
