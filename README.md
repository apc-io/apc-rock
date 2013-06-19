apc-rock
========

Kernel and Bootloader codes for Rock (&amp; Paper)

----------------------------------------

Copyright © 2013 VIA Technologies, Inc.

Update: 2013/05/02
----------------------------------------

This application is Android BSP for APC board.

----------------------------------------

Linux kernel is distributed under the GNU General Public License.
http://www.kernel.org/pub/linux/kernel/COPYING

----------------------------------------

DEVELOP ENVIRONMENT:
OS: Ubuntu 10.04 x64
Linux kernel toolchain: Sourcery G++ Lite (2011.03-41) 4.5.2

INSTALLATION:
1. Unpack APC-8950.tgz

BUILD SOURCE CODE
1. Build Linux kernel
   a. Go to apc-8950/kernel
   b. Using "make Android_defconfig" command to set config
   c. Using "make ubin" command to build Linux kernel

2. Build U-boot
   a. Go to apc-8950/uboot
   b. make wmt_config
   c. make

----------------------------------------
APC-8950.tgz v1.01.00

Info:
------
1. New source from original

Change log:
------------
- Add Share PIN default setting
- Add Support CS8556 transmitter
- Add Support CIR remote control by uboot param

--------------------------------------------------------------------------------
APC-8950.tgz v1.01.01

Info:
------

Change log:
------------
- Add Disable edid by uboot parameter
- Add HDMI and DAC audio at same time when wmt.display.dual=1

--------------------------------------------------------------------------------
APC-8950.tgz v1.01.02

Info:
------

Change log:
------------
- Chg CS8556 register table

--------------------------------------------------------------------------------
