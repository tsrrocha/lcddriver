LCD 16x2 and 16x4 Driver
===========================================
Copyright (C) 2013 RS Tecnologia.

<h2>The Project</h2>

This project aims to create a driver for communication and manipulation of a 16x2 and 16x4 lcd display via I2C protocol. Using a PCF8574 IO expander (IC)..

-------------------

1. **Use** - This driver creates a file structure to facilitate its use as described below:
    1. **line1**
        1. **/sys/class/display/lcdxy/device/line1** - Write only. 
        1. Writing a text on line 1 of the display. Command: *echo "Text" > /sys/class/display/lcdxy/device/line1*.
    1. **line2**
        1. **/sys/class/display/lcdxy/device/line2** - Write only.
        1. Writing a text on line 2 of the display. Command: *echo "Text" > /sys/class/display/lcdxy/device/line2*.
    1. **line3** - Only LCD16x4
        1. **/sys/class/display/lcdxy/device/line3** - Write only.
        1. Writing a text on line 2 of the display. Command: *echo "Text" > /sys/class/display/lcdxy/device/line3*.
    1. **line4** - Only LCD16x4
        1. **/sys/class/display/lcdxy/device/line4** - Write only.
        1. Writing a text on line 2 of the display. Command: *echo "Text" > /sys/class/display/lcdxy/device/line4*.
    1. **clear**
        1. **/sys/class/display/lcdxy/device/clear** - Write only.
        1. Clears all text display. Command: *echo 0 > /sys/class/display/lcdxy/device/clear*.
    1. **backlight**
        1. **/sys/class/display/lcdxy/device/backlightr** - Write only.
        1. Turns off the Backlight. Command: *echo 0 > /sys/class/display/lcdxy/device/backlight*.
        1. Turns on the Backlight. Command: *echo 1 > /sys/class/display/lcdxy/device/backlight*.
    1. **command**
        1. **/sys/class/display/lcdxy/device/command** - Write only.
        1. Sends a byte of command to the display. Command: *echo 192 > /sys/class/display/lcdxy/device/command*.
    1. **data**
        1. **/sys/class/display/lcdxy/device/data** - Write only.
        1. Sends a byte of data to the display. Command: *echo 192 > /sys/class/display/lcdxy/device/data*.
    1. **new_caracter**
        1. **/sys/class/display/lcdxy/device/new_caracter** - Write only.
        1. Sends a new character into CGRAM of the display. Command: *echo char_file.chr > /sys/class/display/lcdxy/device/new_caracter*.
        1. The file should contain 9 bytes. The first is the character position in the CGRAM and the next eight bytes form the character.
        1. To print the character set, the cursor must be positioned using **COMMAND** file and then sent to the **CGRAM** position to be printed on the display using the **DATA** file.

<h2>**Install the Module in Beaglebone Black or Raspberry Pi**</h2>

1. Download the lcd folder and copy the path to the kernel source code BEAGLEBONE or Raspberry Pi: /usr/src/beagle-3.8/KERNEL/drivers/misc/
1. Change the Makefile that is on the way: /usr/src/beagle-3.8/KERNEL/drivers/misc/Makefile 
    1. Add: "obj-y + = lcd/"
1. Change the Makefile that is on the way: /usr/src/beagle-3.8/KERNEL/drivers/misc/Kconfig 
    1. Add: source "drivers/misc/lcd/Kconfig"
1. Compile the kernel, generating modules **lcd16x2.ko** and **lcd16x4.ko**

--------------

<h2>**Instantiating the device in user space**</h2>

To instantiate the device lcd16x2 or lcd16x4, just run the following command respecting the device address, coforme below:

`LCD 16x2 - address 0x27:`

      echo lcd16x2 0x27 > /sys/bus/i2c/devices/i2c-1/new_device

`LCD 16x4 - address 0x27:`

      echo lcd16x4 0x27 > /sys/bus/i2c/devices/i2c-1/new_device

------------------------
 
 
<h2>**Instantiating the device in kernel space on Beaglebone Black with Device Tree**</h2>

Edit the file: ***/usr/src/beagle-3.8/KERNEL/arch/arm/boot/dts/am335x-bone-common.dtsi*** and add the following code before the SLOT@3:

    slot@4 {
    lcd16x2 = <&cape_lcd16x2_1>; 
    };

Edit the file: ***/usr/src/beagle-3.8/KERNEL/arch/arm/boot/dts/am335x-bone-common.dtsi*** and add the following code before the CAPE_EEPROM3:

    cape_lcd16x2_1: cape_lcd16x2_1@27 {
    compatible = "at,lcd16x2";
    reg = <0x27>; 
    };

Compile the kernel






----------------------------


<h2>Support</h2>

    Developer: Tiago Sousa Rocha - tsrrocha@gmail.com - +558396541382 and +558381115793
