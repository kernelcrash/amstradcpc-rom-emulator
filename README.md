amstradcpc-rom-emulator
---------------------------------
kernel at kernelcrash dot com

UPDATE: 20180613 - Now uses interrupts and should load whole disks track by 
track. More technical details on kernelcrash.com

Sort of a proof of concept for using a cheap STM32F407 board directly attached to an Amstrad to emulate
a few 'hardware components'

- Can emulate a few ROMS. They appear as normal Amstrad 'external ROMs'
- Can load floppy DSK image files from an SD card attached to the STM32F407
- Simulates the NEC upd765 Floppy controller, so you think you are loading from a normal floppy drive
- As it simulates the upd765, this is really just for a CPC464, unless you want to remove the upd765
  from your 6128 (and you might need to remove the AMSDOS rom too).

Overview
========

- Get a cheap STM32F407 board off ebay/aliexpress/taobao. Look for STM32F407VGT6 or STM32F407VET6 for
  the cheaper boards. Also look for 2.54mm pin spacing. You can wire in an SD card adapter, or just
  get a board that has one already. So long as its wired in the standard way to use SDIO for
  a STM32F4 microcontroller, it should work.
- Get a 50 pin edge connector and somehow just wire it directly in;

 - PD15 - PD8 - connect to D7 to D0 on Amstrad
 - PE15 - PE0 - connect to A15 to A0 on Amstrad
 - PC0 - Amstrad /WR
 - PC1 - Amstrad /IORQ
 - PC2 - Amstrad /ROMEN
 - PC3 - Amstrad ROMDIS
 - PC4 - Amstrad /MREQ

 - PA0 is a debug output for hooking up to a logic analyser to check timings, but is not connected 
   normally

 - As mentioned, most STM32F4 boards with an SD card adapter are wired the same way. The important
   pins are:
```
   /CS    - PC11
   MOSI   - PD2
   SCK    - PC12

   MISO   - PC8
```
 - GND - Amstrad GND
 - +5V - Amstrad +5V

- Get some ROMS. It can be reconfigured to have more than two roms, but to keep things simple, it
  will load two roms. One at ROM 6 , the other at ROM 7. The fact that it loads ROM 7, means you can't
  use this with a 6128 at the moment, as supposedly you cannot disable ROM 7 on a 6128. The other
  fact is ; that it emulates the upd765 floppy controller chip also means its limited to the CPC 464.

  I put the Maxam assembler as ROM 6 and either AMSDOS or Parados as ROM 7. ROM 7 has to be a Disk OS
  ROM. ROM 6 can be anything, but Maxam is quite handy for being able to type |HELP 

  Just make sure you have two .incbin lines uncommented in the poller.S file referencing your roms. eg
```
   rom_base:
   //.incbin "roms/Moon_Buggy.rom"
   .incbin "roms/maxam15.rom"
   .incbin "roms/parados12.rom"
   //.incbin "roms/AMSDOS_0.7.ROM"
```
  The first .incbin line becomes ROM 6, the 2nd one becomes ROM 7

- compile the code. I am using the STM32F4 Discovery std periph libraries from ST. The DSP one
  will probably work as well, so long as you update the STM_COMMON reference in the Makefile.
  I generally do this;
```
   export PATH=/usr/local/gcc-arm-none-eabi-7-2017-q4-major/bin:$PATH
   make
```
 You will end up with an elf and bin file. There is a transfer.sh file to help you use dfu-util
 to flash it to your board. You will need to set your BOOT0/1 jumpers appropriately first.

- Format a micro SD card with FAT32. Make one partition (preferably under 4GB) at the start of the SD
  card. I just juse linux mkfs.vfat and its fine. 

- Get some Amstrad CPC DSK files and put them in the root of the SD card using this naming scheme
```
    CPC000.DSK
    CPC001.DSK
    CPC002.DSK
```
 Yep, I know its pretty ugly. This is 'proof of concept' territory.
 The older 'polling_version' branch could only load the first 96K of a disk. The current interrupt
 driven version can load whole disks track by track.

 Just try out some different games and see if they work.

Using it
========

If you have say MAXAM and ParaDOS as the ROMs, then you should see 'MAXAM' and 'ParaDOS' in the 
Amstrad main screen when you power on. If you don't get the banners, check your wiring, try reseating
the edge connector.

Assuming you have your SD card plugged in and you have put at least one DSK file on it called CPC000.DSK,
try doing this
```
|A
CAT
```
If it's all working, you'll get a listing of whatever is on the CPC000.DSK image.

If its a game, you generally see one file. You can generally use the one filename to load and run it.
For example:
```
 RUN "ELITE64E"
```
Now, if you want to change to using CPC001.DSK , or CPC002.DSK, do this
```
OUT &FB7E,1
```
Basically, write the DSK image number you want to FB7E. If you want CPC008.DSK, then ;
```
OUT &FB7E,8
```
You generally don't have to power off/on the Amstrad. Just do the usual to check the contents of A: 
have changed
```
|A
CAT
```
For the informed, FB7E is usually used for reading data and status info from the upd765 floppy chip,
but if you write to it, my code picks up the write and swaps to a new disk image.

The DSK id (ie 0,1,2,3 etc) is stored in the 'backup SRAM' in the STM32F407, so if your board has a 
battery attached, then it should remember what image was last selected each time the board is 
powered on. If you don't have a battery, it will probably reset to id 0 (ie. CPC000.DSK). It is
probably not very hard to either attach a battery or a large capacitor to VBAT . If you don't have
a battery and you are sick of it resetting to CPC000.DSK after every power off/on, try powering the 
STM32F4 board from a USB cable (make sure +5V from the Amstrad is no longer connected to the STM32F4 
board). 

Note, if you are just trying to access ROMS, I'd suggest having MAXAM as one of the ROMS you load
so you can type
```
  |HELP
```
to list out the ROMS and their number, then you can do something like 
```
  |HELP,7
```
to show info on using ROM 7 etc.


What doesn't work?
==================

I haven't tested this greatly. If you are trying to read something with a lot of copy protection, then
give up. I have only implemented 'some' of the functions of the upd765. There is a lot missing. I have
pretty much only implemented the 'read data' function (which reads sectors), and a bunch of related
functions. I have not implemented read track. There is write support ('WRITE DATA'), but I have not 
tested it greatly.

But a lot of games do load. Just try stuff.














