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
10F, 529, Chung-Cheng Road, Hsin-Tien, Taipei 231, R.O.C.
--*/
/*
 *  u-boot/include/linux/mtd/nand_ids.h
 *
 *  Copyright (c) 2000 David Woodhouse <dwmw2@mvhi.com>
 *                     Steven J. Hill <sjhill@cotw.com>
 *
 * $Id: nand_ids.h,v 1.1 2000/10/13 16:16:26 mdeans Exp $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Info:
 *   Contains standard defines and IDs for NAND flash devices
 *
 *  Changelog:
 *   01-31-2000 DMW     Created
 *   09-18-2000 SJH     Moved structure out of the Disk-On-Chip drivers
 *			so it can be used by other NAND flash device
 *			drivers. I also changed the copyright since none
 *			of the original contents of this file are specific
 *			to DoC devices. David can whack me with a baseball
 *			bat later if I did something naughty.
 *   10-11-2000 SJH     Added private NAND flash structure for driver
 *   2000-10-13 BE      Moved out of 'nand.h' - avoids duplication.
 */

#ifndef __LINUX_MTD_NAND_IDS_H
#define __LINUX_MTD_NAND_IDS_H
#if 0
static struct nand_flash_dev nand_flash_ids[] = {
	{"Toshiba TC5816BDC",     NAND_MFR_TOSHIBA, 0x64, 21, 1, 2, 0x1000, 0},
	{"Toshiba TC5832DC",      NAND_MFR_TOSHIBA, 0x6b, 22, 0, 2, 0x2000, 0},
	{"Toshiba TH58V128DC",    NAND_MFR_TOSHIBA, 0x73, 24, 0, 2, 0x4000, 0},
	{"Toshiba TC58256FT/DC",  NAND_MFR_TOSHIBA, 0x75, 25, 0, 2, 0x4000, 0},
	{"Toshiba TH58512FT",     NAND_MFR_TOSHIBA, 0x76, 26, 0, 3, 0x4000, 0},
	{"Toshiba TC58V32DC",     NAND_MFR_TOSHIBA, 0xe5, 22, 0, 2, 0x2000, 0},
	{"Toshiba TC58V64AFT/DC", NAND_MFR_TOSHIBA, 0xe6, 23, 0, 2, 0x2000, 0},
	{"Toshiba TC58V16BDC",    NAND_MFR_TOSHIBA, 0xea, 21, 1, 2, 0x1000, 0},
	{"Toshiba TH58100FT",     NAND_MFR_TOSHIBA, 0x79, 27, 0, 3, 0x4000, 0},
	{"Samsung KM29N16000",    NAND_MFR_SAMSUNG, 0x64, 21, 1, 2, 0x1000, 0},
	{"Samsung unknown 4Mb",   NAND_MFR_SAMSUNG, 0x6b, 22, 0, 2, 0x2000, 0},
	{"Samsung KM29U128T",     NAND_MFR_SAMSUNG, 0x73, 24, 0, 2, 0x4000, 0},
	{"Samsung KM29U256T",     NAND_MFR_SAMSUNG, 0x75, 25, 0, 2, 0x4000, 0},
	{"Samsung unknown 64Mb",  NAND_MFR_SAMSUNG, 0x76, 26, 0, 3, 0x4000, 0},
	{"Samsung KM29W32000",    NAND_MFR_SAMSUNG, 0xe3, 22, 0, 2, 0x2000, 0},
	{"Samsung unknown 4Mb",   NAND_MFR_SAMSUNG, 0xe5, 22, 0, 2, 0x2000, 0},
	{"Samsung KM29U64000",    NAND_MFR_SAMSUNG, 0xe6, 23, 0, 2, 0x2000, 0},
	{"Samsung KM29W16000",    NAND_MFR_SAMSUNG, 0xea, 21, 1, 2, 0x1000, 0},
	{"Samsung K9F5616Q0C",    NAND_MFR_SAMSUNG, 0x45, 25, 0, 2, 0x4000, 1},
	{"Samsung K9K1216Q0C",    NAND_MFR_SAMSUNG, 0x46, 26, 0, 3, 0x4000, 1},
	{0,}
};
#endif
#define NAND_TYPE_MLC 1
#define NAND_TYPE_SLC 0
#define WIDTH_8 0
static struct WMT_nand_flash_dev WMT_nand_flash_ids[] = {
	//Hynix
	{0xADD314A5, 4096, 2048,  64, 0x40000, 5, 125, 127, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC,  4, 0x140A0C46,  0x46, 0, 0, 0, "HY27UT088G2M-T(P)", 0},
	{0xADF1801D, 1024, 2048,  64, 0x20000, 4,   0,   1, 0, WIDTH_8, 4, 0, 1, NAND_TYPE_SLC,  4, 0x140A0F64,  0x64, 0, 0, 0, "HY27UF081G2A", 0},
	{0xADF1001D, 1024, 2048,  64, 0x20000, 4,   0,   1, 0, WIDTH_8, 4, 0, 1, NAND_TYPE_SLC,  4, 0x140A0C46,  0x46, 0, 0, 0, "H27U1G8F2BFR", 0},
	{0xADD59425, 4096, 4096, 218, 0x80000, 5, 125, 127, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 12, 0x140A0F64,  0x64, 0, 0, 0, "HY27UAG8T2A", 0},
	{0xADD7949A, 2048, 8192, 448,0x200000, 5,   0, 255, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 24, 0x140A0C64,  0x64, 0, 0, 0, "HY27UBG8T2ATR", 0},
	{0xADD5949A, 1024, 8192, 448,0x200000, 5,   0, 255, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 24, 0x140A0C64,  0x64, 0, 0, 0, "H27UAG8T2BTR", 0},
	{0xADD794DA, 2048, 8192, 640,0x200000, 5,   0, 255, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 24, 0x10080AC8,  0xC8, 0, 1, 1, "H27UBG8T2BTR", 0},
	{0xADD79491, 2048, 8192, 640,0x200000, 5,   0, 255, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 40, 0x100608C8,  0xC8, 0, 1, 1, "H27UBG8T2CTR-F20", 0},
	{0xADDE94DA, 4096, 8192, 640,0x200000, 5,   0, 255, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 40, 0x100608C8,  0xC8, 1, 1, 1, "H27UCG8T2ATR-F20", 0},
                                                                                                                       
	//Samsung
	{0xECD314A5, 4096, 2048,  64, 0x40000, 5, 127,   0, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC,  4, 0x140A0C64,  0x64, 0, 0, 0, "K9G8G08X0A", 0},
	{0xECD59429, 4096, 4096, 218, 0x80000, 5, 127,   0, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 12, 0x140A0F64,  0x64, 0, 0, 0, "K9GAG08UXD", 0},
	{0xECF10095, 1024, 2048,  64, 0x20000, 4,   0,   1, 0, WIDTH_8, 4, 0, 1, NAND_TYPE_SLC,  4, 0x140a1464,  0x64, 0, 0, 0, "K9F1G08U0B", 0},
	{0xEC75A5BD, 2048,  512,  16,  0x4000, 4,   0,   1, 5, WIDTH_8, 1, 1, 0, NAND_TYPE_SLC,  1, 0x230F1964,  0x64, 0, 0, 0, "K95608U0D", 0},
	{0xECD514B6, 4096, 4096, 128, 0x80000, 5, 127,   0, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC,  4, 0x140A0C64,  0x64, 0, 0, 0, "K9GAG08U0M", 0},
	{0xECD755B6, 8192, 4096, 128, 0x80000, 5, 127,   0, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC,  4, 0x140A0C64,  0x64, 0, 0, 0, "K9LBG08U0M", 0},
	{0xECD58472, 2048, 8192, 436,0x100000, 5,   0, 127, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 24, 0x190A0FFF, 0x12C, 0, 0, 0, "K9GAG08U0E", 0},
	{0xECD7947A, 4096, 8192, 448,0x100000, 5,   0, 127, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 24, 0x190A0FFF, 0x12C, 0, 0, 1, "K9GBG08U0A", 0},
	{0xECD59476, 2048, 8192, 448,0x100000, 5,   0, 127, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 24, 0x140A0CFF, 0x12C, 0, 0, 0, "K9GAG08U0F", 0},
	{0xECD7947E, 4096, 8192,1024,0x100000, 5,   0, 127, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 40, 0x140B0BFF, 0x12C, 0, 0, 1, "K9GBG08U0B", 0},
	{0xECDED57A, 8192, 8192, 640,0x100000, 5,   0, 127, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 24, 0x140A0CFF, 0x12C, 0, 0, 1, "K9LCG08U0A", 0},
                                                                                                                       
	//Toshiba
	{0x98D594BA, 4096, 4096, 218, 0x80000, 5,   0, 127, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 12, 0x190F0F70,  0x70, 0, 0, 0, "TC58NVG4D1DTG0", 0},
	{0x98D19015, 1024, 2048,  64, 0x20000, 4,   0,   1, 0, WIDTH_8, 4, 0, 1, NAND_TYPE_SLC,  4, 0x140A0C11,  0x11, 0, 0, 0, "TC58NVG0S3ETA00", 0},
	{0x98D59432, 2048, 8192, 448,0x100000, 5,   0, 127, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 24, 0x140A0C84,  0x84, 0, 0, 0, "TC58NVG4D2FTA00", 0},
	{0x98D58432, 2048, 8192, 640,0x100000, 5,   0, 127, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 40, 0x190A0FFF, 0x12C, 0, 1, 1, "TC58NVG4D2HTA00", 0},
	{0x98DE8493, 2048,16384,1280,0x400000, 5,   0, 255, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 40, 0x190A0FFF, 0x12C, 0, 1, 1, "TC58NVG6DCJTA00", 0},
	{0x98D79432, 4096, 8192, 448,0x100000, 5,   0, 127, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 24, 0x140A0C78,  0x78, 0, 0, 0, "TC58NVG5D2FTAI0", 0x76550000},
	{0x98D79432, 4096, 8192, 640,0x100000, 5,   0, 127, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 40, 0x10070AFF, 0x12C, 0, 1, 1, "TC58NVG5D2HTA00", 0x76560000},
	
	//Miron
	{0x2C88044B, 4096, 8192, 448,0x200000, 5,   0, 255, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 24, 0x281E32C8,  0xC8, 0, 0, 0, "MT29F64G08CBAAA", 0},
	{0x2C68044A, 4096, 4096, 224,0x100000, 5,   0,   1, 0, WIDTH_8, 1, 0, 1, NAND_TYPE_MLC, 12, 0x321E32C8,  0xC8, 0, 0, 0, "MT29F32G08CBACA", 0},
	{0,}
	/*add new product item here.*/
};
/* Nand Product Table          We take HYNIX_NF_HY27UF081G2A for example.
struct NFC_info{
 DWORD dwFlashID;            //composed by 4 bytes of ID. For example:0xADF1801D
 DWORD dwBlockCount;      //block count of one chip. For example: 1024
 DWORD dwPageSize;       //page size. For example:2048(other value can be 512 or 4096)
 DWORD dwSpareSize;       //spare area size. For example:16(almost all kinds of nand is 16)
 DWORD dwBlockSize;       //block size = dwPageSize * PageCntPerBlock. For example:131072
 DWORD dwAddressCycle;      //address cycle 4 or 5
 DWORD dwBI0Position;      //BI0 page postion in block
 DWORD dwBI1Position;      //BI1 page postion in block
 DWORD dwBIOffset;       //BI offset in page 0 or 2048
 DWORD dwDataWidth;      //data with X8 = 0 or X16 = 1
 DWORD dwPageProgramLimit;     //chip can program PAGE_PROGRAM_LIMIT times within the same page
 DWORD dwSeqRowReadSupport;    //whether support sequential row read, 1 = support, 0 = not support
 DWORD dwSeqPageProgram;     //chip need sequential page program in a block. 1 = need
 DWORD dwNandType;       //MLC = 1 or SLC = 0
 DWORD dwECCBitNum;      //ECC bit number needed
 DWORD dwRWTimming;     //NFC Read/Write Pulse width and Read/Write hold time. default =0x12121010
 char cProductName[MAX_PRODUCT_NAME_LENGTH]; //product name. for example "HYNIX_NF_HY27UF081G2A"
};
*/

int hynix_set_parameter(struct nand_chip *nand, int mode, int def_mode);
int hynix_get_parameter(struct nand_chip *nand, int mode);
int hynix_get_otp(struct nand_chip *nand);
int toshiba_get_parameter(struct nand_chip *nand, int mode);
int toshiba_set_parameter(struct nand_chip *nand, int mode, int def);
struct nand_read_retry_param chip_table[] = {
	//Hynix
    [0] = {
        .magic = "readretry", 
        .nand_id = 0xADD794DA,
		.nand_id_5th = 0,
        .eslc_reg_num = 5, 
		.eslc_offset = {0xa0, 0xa1, 0xb0, 0xb1, 0xc9},
		.eslc_set_value = {0x26, 0x26, 0x26, 0x26, 0x1}, 
		.retry_reg_num = 4,
		.retry_offset = {0xa7, 0xad, 0xae, 0xaf},
		.retry_value = {0, 0x6,0xa, 0x6, 0x0, 0x3, 0x7, 0x8, 0, 0x6, 0xd, 0xf, 0x0, 0x9, 0x14, 0x17, 0x0, 0x0, 0x1a, 0x1e, 0x0, 0x0, 0x20, 0x25}, 
		.total_try_times = 6, 
		.cur_try_times = -1,
		.set_parameter = hynix_set_parameter, 
		.get_parameter = hynix_get_parameter,
	},
	[1] = {
		.magic = "readretry", 
		.nand_id = 0xADDE94DA, 
		.nand_id_5th = 0,
		.eslc_reg_num = 4, 
		.eslc_offset = {0xb0, 0xb1, 0xa0, 0xa1}, 
		.eslc_set_value = {0xa, 0xa, 0xa, 0xa}, 
		.retry_reg_num = 8, 
		.retry_offset = {0xcc, 0xbf, 0xaa, 0xab, 0xcd, 0xad, 0xae, 0xaf}, 
		.otp_len = 2,
		.otp_offset = {0xff, 0xcc},
		.otp_data = {0x40, 0x4d},
		.total_try_times = 7, 
		.cur_try_times = -1, 
		.set_parameter = hynix_set_parameter, 
		.get_parameter = hynix_get_parameter, 
		.get_otp_table = hynix_get_otp, 
	},
	[2] = {
		.magic = "readretry", 
		.nand_id = 0xADDE94EB, 
		.nand_id_5th = 0x0,
		.eslc_reg_num = 4, 
		.eslc_offset = {0xa0, 0xa1, 0xa7, 0xa8}, 
		.eslc_set_value = {0xa, 0xa, 0xa, 0xa}, 
		.retry_reg_num = 8, 
		.retry_offset = {0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7}, 
		.otp_len = 2,
		.otp_offset = {0xae, 0xb0},
		.otp_data = {0x00, 0x4d},
		.total_try_times = 7, 
		.cur_try_times = -1, 
		.set_parameter = hynix_set_parameter, 
		.get_parameter = hynix_get_parameter, 
		.get_otp_table = hynix_get_otp, 
	},
	[3] = {
		.magic = "readretry", 
		.nand_id = 0xADD79491, 
		.nand_id_5th = 0x0,
		.eslc_reg_num = 4, 
		.eslc_offset = {0xa0, 0xa1, 0xa7, 0xa8}, 
		.eslc_set_value = {0xa, 0xa, 0xa, 0xa}, 
		.retry_reg_num = 8, 
		.retry_offset = {0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7}, 
		.otp_len = 2,
		.otp_offset = {0xae, 0xb0},
		.otp_data = {0x00, 0x4d},
		.total_try_times = 7, 
		.cur_try_times = -1, 
		.set_parameter = hynix_set_parameter, 
		.get_parameter = hynix_get_parameter, 
		.get_otp_table = hynix_get_otp, 
	},
	//Toshiba
	[4] = {
		.magic = "readretry", 
		.nand_id = 0x98de8493, 
		.nand_id_5th = 0,
		.retry_reg_num = 4, 
		.retry_offset = {4, 5, 6, 7},
		.retry_value = {0, 0, 0, 0, 4, 4, 4, 4, 0x7c, 0x7c, 0x7c, 0x7c, 0x78, 0x78, 0x78, 0x78, 0x74, 0x74, 0x74, 0x74, 0x8, 0x8, 0x8, 0x8}, 
		.total_try_times = 6, 
		.cur_try_times = 0, 
		.set_parameter = toshiba_set_parameter, 
		.get_parameter = toshiba_get_parameter,
	},
	[5] = {
		.magic = "readretry", 
		.nand_id = 0x98DE9482, 
		.nand_id_5th = 0,
		.retry_reg_num = 4, 
		.retry_offset = {4, 5, 6, 7},
		.retry_value = {0, 0, 0, 0, 4, 4, 4, 4, 0x7c, 0x7c, 0x7c, 0x7c, 0x78, 0x78, 0x78, 0x78, 0x74, 0x74, 0x74, 0x74, 0x8, 0x8, 0x8, 0x8}, 
		.total_try_times = 6, 
		.cur_try_times = 0, 
		.set_parameter = toshiba_set_parameter, 
		.get_parameter = toshiba_get_parameter,
	}
};  
#endif /* __LINUX_MTD_NAND_IDS_H */
