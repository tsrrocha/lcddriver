LCD I2C Driver
===============

Module Driver for LCD 16x2 and 16x4



Install in Beaglebone Black:
-----------------
**1 - Copy the folder "lcd" to path [source-code-kernel-beaglebone]/drivers/misc/**

**2 - Edit the file [source-code-kernel-beaglebone]/drivers/misc/Makefile**

- Add in new line the folder name of lcd folder: 'obj-y   += lcd/'
  
- Save the ..../misc/Makefile
  
**3 - Edit the file [source-code-kernel-beaglebone]/drivers/misc/Kconfig**

- Add in new line in the end of file: 'source "drivers/misc/lcd/Kconfig"'
  
- Save the ..../misc/Kconfig

**4 - Edit the file [source-code-kernel-beaglebone]/arch/arm/boot/dts/am335x-bone-common.dtsi**

- Add the following code before the SLOT@3:

  >&#09;slot@4 {
  
  >&#09;lcd16x2 = &#60;&amp;cape_lcd16x2_1&#62;;
  
  >};
  
- Add the following code before the CAPE_EEPROM3:

  cape_lcd16x2_1: cape_lcd16x2_1@27 {
    
    compatible = "at,lcd16x2";
    
    reg = &#60;27&#62;;
  
  };
  

**5 - **

6 - Compile the kernel source.




Install in Raspberry Pi:
-------------
[...]
