/*###########################################################################
 *#	LCD 16x2 Driver
 *#
 *#	Author:	Tiago Sousa Rocha
 *#	Desc:	This driver will do the lcd control.
 *#
 *###########################################################################
 * lcd16x2.c - Linux kernel modules
 *
 * Copyright (C) 2003-2010  Tiago Sousa Rocha <tsrrocha@gmail.com>
 *
 *###########################################################################
 *
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *###########################################################################
 */
#include <asm/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kdev_t.h>

//###########################################################################
//#	Types and Variables:
//###########################################################################
// Addresses to scan.
static const unsigned short normal_i2c[] = { 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, I2C_CLIENT_END };
// Types of LCD
enum chips { lcd16x2, lcd16x4 };
// define type BYTE
typedef unsigned char BYTE;
// Flag for backlight
static u8 backlightFlag 	= 0x01;	// Flag de in
// LCD data
static u8 lcd_data 		= 0x00;
// Mutex para controle de acesso concorrente
static DEFINE_MUTEX(lcd16x2_mutex);
// Classe do device
static struct class *lcd_class;
//###########################################################################
//#	Defines:
//###########################################################################
//#define DEBUG
//###########################################################################
#define DRIVER_AUTHOR		"Tiago Sousa Rocha <tsrrocha@gmail.com>"
#define DRIVER_DESCRIPTION	"LCD 16x2 & 16x4 display driver"
#define DRIVER_VERSION		"1.4.r02"		// Versão do driver
//###########################################################################
#define LCD_REG_MAN_ID		0x27
#define LCD_REG_CHIP_ID		0xFF
//###########################################################################
#define LCD_BL			0x08
#define LCD_EN			0x04  // Enable bit
#define LCD_RW			0x02  // Read/Write bit
#define LCD_RS			0x01  // Register select bit
#define LCD_MASK_DISABLE_EN	0xFB  // MASK of control
#define STROBE_EN_DELAY 	1		// nanosegundos
#define CLEAR_DELAY 		1		//
#define COMMAND_INIT_DELAY 	1		// ms
//###########################################################################
// LCD Commands
#define LCD_CMD_CLEARDISPLAY    0x01
#define LCD_CMD_RETURNHOME      0x02
#define LCD_CMD_ENTRYMODESET    0x04
#define LCD_CMD_DISPLAYCONTROL  0x08
#define LCD_CMD_CURSORSHIFT     0x10
#define LCD_CMD_FUNCTIONSET     0x20
#define LCD_CMD_SETCGRAMADDR    0x40
#define LCD_CMD_SETDDRAMADDR    0x80
#define LCD_CLEARDISPLAY    	0x01
#define LCD_RETURNHOME      	0x02
#define LCD_ENTRYMODESET    	0x04
#define LCD_DISPLAYCONTROL  	0x08
#define LCD_CURSORSHIFT     	0x10
#define LCD_FUNCTIONSET     	0x20
#define LCD_SETCGRAMADDR    	0x40
#define LCD_SETDDRAMADDR    	0x80
// FLAGS de controle do LCD
#define LCD_CTRL_BLINK_ON	0x01
#define LCD_CTRL_BLINK_OFF	0x00
#define LCD_CTRL_DISPLAY_ON	0x04
#define LCD_CTRL_DISPLAY_OFF	0x00
#define LCD_CTRL_CURSOR_ON	0x02
#define LCD_CTRL_CURSOR_OFF	0x00
#define LCD_DISPLAYON   	0x04
#define LCD_DISPLAYOFF  	0x00
#define LCD_CURSORON    	0x02
#define LCD_CURSOROFF   	0x00
#define LCD_BLINKON     	0x01
#define LCD_BLINKOFF    	0x00
#define LCD_ENTRYRIGHT           0x00
#define LCD_ENTRYLEFT            0x02
#define LCD_ENTRYSHIFTINCREMENT  0x01
#define LCD_ENTRYSHIFTDECREMENT  0x00
#define LCD_DISPLAYMOVE  	0x08
#define LCD_CURSORMOVE   	0x00
#define LCD_MOVERIGHT    	0x04
#define LCD_MOVELEFT     	0x00
#define LCD_8BITMODE  		0x10
#define LCD_4BITMODE  		0x00
#define LCD_2LINE     		0x08
#define LCD_1LINE     		0x00
#define LCD_5x10DOTS  		0x04
#define LCD_5x8DOTS   		0x00
//###########################################################################


//###########################################################################
//#	Driver data (common to all clients)
//###########################################################################
static const struct i2c_device_id lcd_id[] = {
	{ "lcd16x2", lcd16x2 },
	{ "lcd16x4", lcd16x4 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lcd_id);

//###########################################################################
//#	chip type specific parameters
//###########################################################################
struct lcd_params {
	u8  qtylines;
	u8  qtycolumns;
	u8  address;
};
static const struct lcd_params lcd_params[] = {
	[lcd16x2] = {
		.qtylines = 2,
		.qtycolumns = 16,
		.address = 0x27,
	},
	[lcd16x4] = {
		.qtylines = 4,
		.qtycolumns = 16,
		.address = 0x26,
	},
};

//###########################################################################
//#	Client data (each client gets its own)
//###########################################################################
struct lcd_data {
	struct device *hwmon_dev;
	struct mutex update_lock;
	int kind;
};

//###########################################################################
//#	Functions of lcd control
//###########################################################################
/**   Function:	static int lcd_i2c_write_byte(struct i2c_client *client, u8 *data)
 **   Author:	Tiago Sousa Rocha
 **   Desc:	This function will go write an byte in i2c protocol.
 */
static int lcd_i2c_write_byte(struct i2c_client *client, u8 *data) {
  int ret;
  ret = i2c_master_send(client, data, 1);
  if (ret < 0)
    dev_warn(&client->dev, "Write byte in i2c ['0x%02X'] failed.\n", ((int)*data));
  return ret;
}

/**   Function:	static int lcd_i2c_read_byte(struct i2c_client *client)
 **   Author:	Tiago Sousa Rocha
 **   Desc:	This function will go read an byte of i2c protocol.
 */
static int lcd_i2c_read_byte(struct i2c_client *client) {
  unsigned char i2c_data[1];
  int ret = 0;
  ret = i2c_master_recv(client, i2c_data, 1);
  if (ret < 0) {
    dev_warn(&client->dev, "LCD 16x2 i2c read data failed\n");
    return ret;
  }
  return (i2c_data[0]);
}

/**   Function:	
 **   Author:	Tiago Sousa Rocha
 **   Desc:	
 */
static void lcd_en_strobe(struct i2c_client *client, int msdelay) {
  int ret = 0;
  lcd_data = lcd_i2c_read_byte(client);
  lcd_data = (lcd_data | LCD_EN | (backlightFlag == 1 ? LCD_BL : 0x00));
  ret = lcd_i2c_write_byte(client, &lcd_data);
  ndelay(msdelay);	
  lcd_data &= LCD_MASK_DISABLE_EN | (backlightFlag == 1 ? LCD_BL : 0x00);
  ret = lcd_i2c_write_byte(client, &lcd_data);
}

/**   Function:	static int lcd_send_cmd(struct i2c_client *client, u8 cmd, int msdelay)
 **   Author:	Tiago Sousa Rocha
 **   Desc:	This function will go to send the command for lcd.
 */
static int lcd_send_cmd(struct i2c_client *client, u8 cmd, int msdelay) {
  BYTE d;
  int ret = 0;
  mutex_lock(&lcd16x2_mutex);
  lcd_data = cmd;
  d = (cmd & 0xF0) | (backlightFlag == 1 ? LCD_BL : 0x00) ;
  lcd_i2c_write_byte(client, &d);
  lcd_en_strobe(client, STROBE_EN_DELAY);
  d = (cmd << 4) | (backlightFlag == 1 ? LCD_BL : 0x00) ;
  lcd_i2c_write_byte(client, &d);
  lcd_en_strobe(client, STROBE_EN_DELAY);
  msleep(msdelay);
  mutex_unlock(&lcd16x2_mutex);
  if (ret < 0)
    dev_warn(&client->dev, "command '%c' failed.\n", cmd);
  return ret;
}

/**   Function:	
 **   Author:	Tiago Sousa Rocha
 **   Desc:	
 */
static int lcd_send_data(struct i2c_client *client, u8 data) {
  BYTE d;
  int ret = 0;
  mutex_lock(&lcd16x2_mutex);
  lcd_data = data;
  d = (data & 0xF0) | LCD_RS | (backlightFlag == 1 ? LCD_BL : 0x00) ;
  lcd_i2c_write_byte(client, &d);
  lcd_en_strobe(client, STROBE_EN_DELAY);
  d = (data << 4) | LCD_RS | (backlightFlag == 1 ? LCD_BL : 0x00) ;
  lcd_i2c_write_byte(client, &d);
  lcd_en_strobe(client, STROBE_EN_DELAY);
  mutex_unlock(&lcd16x2_mutex);
  if (ret < 0)
    dev_warn(&client->dev, "data '%c' failed.\n", data);
  return ret;
}

/**   Function:	
 **   Author:	Tiago Sousa Rocha
 **   Desc:	
 */
static void _setDDRAMAddress(struct i2c_client *client, int line, int col){
  //we write to the Data Display RAM (DDRAM)
  if(line == 1)
    lcd_send_cmd(client, LCD_SETDDRAMADDR | (0x00 + col), CLEAR_DELAY);
  if(line == 2)
    lcd_send_cmd(client, LCD_SETDDRAMADDR | (0x40 + col), CLEAR_DELAY);
  if(line == 3)
    lcd_send_cmd(client, LCD_SETDDRAMADDR | (0x10 + col), CLEAR_DELAY);
  if(line == 4)
    lcd_send_cmd(client, LCD_SETDDRAMADDR | (0x50 + col), CLEAR_DELAY);
}

/**   Function:	
 **   Author:	Tiago Sousa Rocha
 **   Desc:	
 */
static void lcd_puts(struct i2c_client *client, char *string, u8 line, u8 col, u8 count){
  u8 i;
  _setDDRAMAddress(client, line, col);
  for(i = 0; i < count-1; i++){
      lcd_send_data(client, string[i]);
  }
}

/**   Function:	
 **   Author:	Tiago Sousa Rocha
 **   Desc:	
 */
static int lcd_init(struct i2c_client *client){
  BYTE msg[15] = {"CARREGANDO ..."};
  int displayshift 	= (LCD_CURSORMOVE | LCD_MOVERIGHT);
  int displaymode 	= (LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);
  int displaycontrol 	= (LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);

  // inicialização
  lcd_send_cmd(client, 0x02, COMMAND_INIT_DELAY);
  lcd_send_cmd(client, LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS , COMMAND_INIT_DELAY);
  lcd_send_cmd(client, LCD_DISPLAYCONTROL | displaycontrol , COMMAND_INIT_DELAY);
  lcd_send_cmd(client, LCD_ENTRYMODESET | displaymode , COMMAND_INIT_DELAY);
  lcd_send_cmd(client, LCD_CLEARDISPLAY , COMMAND_INIT_DELAY);
  lcd_send_cmd(client, LCD_CURSORSHIFT | displayshift , COMMAND_INIT_DELAY);
  //lcd_send_cmd(client, LCD_RETURNHOME , COMMAND_INIT_DELAY);
  lcd_send_cmd(client, LCD_RETURNHOME , COMMAND_INIT_DELAY);

  lcd_puts(client, msg, 1, 0, 15);
  return 0;
}

/**   Function:	
 **   Author:	Tiago Sousa Rocha
 **   Desc:	
 */
static void lcd_clear(struct i2c_client *client){
  lcd_send_cmd(client, LCD_CLEARDISPLAY , CLEAR_DELAY);
}

/**   Function:	
 **   Author:	Tiago Sousa Rocha
 **   Desc:	
 */
static int lcd_set_backlight(struct device *dev, const char *buf, size_t count)
{
  struct i2c_client *client = to_i2c_client(dev);
  unsigned long val = 0;
  BYTE data;
	
  if (strict_strtoul(buf, 10, &val))
    return -EINVAL;
  if (val == 0) {
    backlightFlag = 0x00;
  } else if (val == 1) {
    backlightFlag = 0x01;
  }

  data = (backlightFlag == 1 ? LCD_BL : 0x00);
  mutex_lock(&lcd16x2_mutex);
  i2c_master_send(client, &data, 1);
  mutex_unlock(&lcd16x2_mutex);
  return count;
}

//###########################################################################
//#	Sysfs stuff
//###########################################################################
/**   Function:	
 **   Author:	Tiago Sousa Rocha
 **   Desc:	
 */
static ssize_t lcd_set_backlight_state_store(struct device *dev, struct device_attribute *attr, const  char *buf, size_t count)
{
  return lcd_set_backlight(dev, buf, count);
}

/**   Function:	
 **   Author:	Tiago Sousa Rocha
 **   Desc:	
 */
static ssize_t lcd_line_1_store(struct device *dev, struct device_attribute *attr, const  char *buf, size_t count)
{
  struct i2c_client *client = to_i2c_client(dev);
  lcd_puts(client, buf, 1, 0, count);
  return count;
}

/**   Function:	
 **   Author:	Tiago Sousa Rocha
 **   Desc:	
 */
static ssize_t lcd_line_2_store(struct device *dev, struct device_attribute *attr, const  char *buf, size_t count)
{
  struct i2c_client *client = to_i2c_client(dev);
  lcd_puts(client, buf, 2, 0, count);
  return count;
}

/**   Function:	
 **   Author:	Tiago Sousa Rocha
 **   Desc:	
 */
static ssize_t lcd_line_3_store(struct device *dev, struct device_attribute *attr, const  char *buf, size_t count)
{
  struct i2c_client *client = to_i2c_client(dev);
  lcd_puts(client, buf, 3, 0, count);
  return count;
}

/**   Function:	
 **   Author:	Tiago Sousa Rocha
 **   Desc:	
 */
static ssize_t lcd_line_4_store(struct device *dev, struct device_attribute *attr, const  char *buf, size_t count)
{
  struct i2c_client *client = to_i2c_client(dev);
  lcd_puts(client, buf, 4, 0, count);
  return count;
}


/**   Function:	
 **   Author:	Tiago Sousa Rocha
 **   Desc:	
 */
static ssize_t lcd_clear_store(struct device *dev, struct device_attribute *attr, const  char *buf, size_t count)
{
  struct i2c_client *client = to_i2c_client(dev);
  lcd_clear(client);
  return count;
}

/**   Function:	
 **   Author:	Tiago Sousa Rocha
 **   Desc:	
 */
static ssize_t lcd_command_store(struct device *dev, struct device_attribute *attr, const  char *buf, size_t count)
{
  struct i2c_client *client = to_i2c_client(dev);
  unsigned long val = 0;
	
  if (strict_strtoul(buf, 10, &val))
    return -EINVAL;
  val &= 0xFF;
  lcd_send_cmd(client, val, CLEAR_DELAY);

  return count;
}

/**   Function:	
 **   Author:	Tiago Sousa Rocha
 **   Desc:	
 */
static ssize_t lcd_data_store(struct device *dev, struct device_attribute *attr, const  char *buf, size_t count)
{
  struct i2c_client *client = to_i2c_client(dev);
  unsigned long val = 0;
	
  if (strict_strtoul(buf, 10, &val))
    return -EINVAL;
  val &= 0xFF;
  lcd_send_data(client, val);

  return count;
}

/**   Function:	
 **   Author:	Tiago Sousa Rocha
 **   Desc:	
 */
static ssize_t lcd_new_caracter_store(struct device *dev, struct device_attribute *attr, const  char *buf, size_t count)
{
  struct i2c_client *client = to_i2c_client(dev);
  int i = 0;

  if ((count == 9) & (buf[0] < 8)){
    lcd_send_cmd(client, LCD_SETCGRAMADDR|(buf[0] * 8), CLEAR_DELAY);
    for (i=1; i<9; i++) {
      lcd_send_data(client, buf[i]);
    }
  }

  return count;
}

//###########################################################################
//#	Attributes
//###########################################################################
static DEVICE_ATTR(backlight, S_IWUSR, NULL, lcd_set_backlight_state_store);
static DEVICE_ATTR(clear, S_IWUSR, NULL, lcd_clear_store);
static DEVICE_ATTR(command, S_IWUSR, NULL, lcd_command_store);
static DEVICE_ATTR(data, S_IWUSR, NULL, lcd_data_store);
static DEVICE_ATTR(new_caracter, S_IWUSR, NULL, lcd_new_caracter_store);
static DEVICE_ATTR(line1, S_IWUSR, NULL, lcd_line_1_store);
static DEVICE_ATTR(line2, S_IWUSR, NULL, lcd_line_2_store);
static DEVICE_ATTR(line3, S_IWUSR, NULL, lcd_line_3_store);
static DEVICE_ATTR(line4, S_IWUSR, NULL, lcd_line_4_store);

static struct attribute *lcd_16x2_attributes[] = {
  &dev_attr_line1.attr,
  &dev_attr_line2.attr,
  &dev_attr_clear.attr,
  &dev_attr_backlight.attr,
  &dev_attr_command.attr,
  &dev_attr_data.attr,
  &dev_attr_new_caracter.attr,
  NULL
};
static const struct attribute_group lcd_16x2_group = {
  .attrs = lcd_16x2_attributes
};

static struct attribute *lcd_16x4_attributes[] = {
  &dev_attr_line1.attr,
  &dev_attr_line2.attr,
  &dev_attr_line3.attr,
  &dev_attr_line4.attr,
  &dev_attr_clear.attr,
  &dev_attr_backlight.attr,
  &dev_attr_command.attr,
  &dev_attr_data.attr,
  &dev_attr_new_caracter.attr,
  NULL
};
static const struct attribute_group lcd_16x4_group = {
  .attrs = lcd_16x4_attributes
};

//###########################################################################
//#	Real code
//###########################################################################
/* Return 0 if detection is successful, -ENODEV otherwise */
static int lcd_detect(struct i2c_client *client, struct i2c_board_info *info) 
{
  struct i2c_adapter *adapter = client->adapter;
  int address = client->addr;
  const char *name = NULL;
  int man_id = 0, chip_id = 0;

  if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
    return -ENODEV;

  /* detection and identification */
  man_id = i2c_smbus_read_byte_data(client, LCD_REG_MAN_ID);
  chip_id = i2c_smbus_read_byte_data(client, LCD_REG_CHIP_ID);
  if (man_id < 0 || chip_id < 0)
    return -ENODEV;

  if ( (address == 0x21) | (address == 0x23) | (address == 0x25) | (address == 0x27) ) { /* National Semiconductor */
    name = "lcd16x2";
    dev_info(&adapter->dev, "Assuming LCD 16x2 chip at 0x%02x\n", address);
    dev_info(&adapter->dev, "If it is an LCD16x2, instantiate it with the new_device sysfs interface\n");
  } else if ( (address == 0x20) | (address == 0x22) | (address == 0x24) | (address == 0x26) ) { /* National Semiconductor */
    name = "lcd16x4";
    dev_info(&adapter->dev, "Assuming LCD 16x4 chip at 0x%02x\n", address);
    dev_info(&adapter->dev, "If it is an LCD16x4, instantiate it with the new_device sysfs interface\n");
  }

  if (!name) {
    dev_dbg(&adapter->dev, "Unsupported chip at 0x%02x (man_id=0x%02X, chip_id=0x%02X)\n", address, man_id, chip_id);
    return -ENODEV;
  }

  strlcpy(info->type, name, I2C_NAME_SIZE);
  return 0;
}

static void lcd_remove_files(struct i2c_client *client, struct lcd_data *data) 
{
  struct device *dev = &client->dev;
  if (data->kind == lcd16x2)
    sysfs_remove_group(&dev->kobj, &lcd_16x2_group);
  else if (data->kind == lcd16x4)
    sysfs_remove_group(&dev->kobj, &lcd_16x4_group);
}

static void lcd_init_client(struct i2c_client *client) 
{
  lcd_init(client);
}

static int lcd_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{
  struct device *dev = &client->dev;
  struct lcd_data *data;
  int err;

  data = devm_kzalloc(&client->dev, sizeof(struct lcd_data), GFP_KERNEL);
  if (!data)
    return -ENOMEM;

  i2c_set_clientdata(client, data);
  mutex_init(&data->update_lock);

  /* Initialize the LCD chip */
  lcd_init_client(client);



  lcd_class = class_create(THIS_MODULE, "display");
  if (IS_ERR(lcd_class)) {
    printk(KERN_ERR "couldn't create sysfs class display\n");
    return PTR_ERR(lcd_class);
  }

  /* Set the device type */
  data->kind = id->driver_data;
  if (data->kind == lcd16x2) {
    printk(KERN_INFO "(DEBUG) Create sysfs group file for lcd 16x2.\n");
    err = sysfs_create_group(&dev->kobj, &lcd_16x2_group);
    if (err)
      return err;
    data->hwmon_dev = device_create(lcd_class, dev, MKDEV(0, 0), NULL, "lcdxy");
  } else if (data->kind == lcd16x4) {
    printk(KERN_INFO "(DEBUG) Create sysfs group file for lcd 16x4.\n");
    err = sysfs_create_group(&dev->kobj, &lcd_16x4_group);
    if (err)
      return err;

    data->hwmon_dev = device_create(lcd_class, dev, MKDEV(0, 0), NULL, "lcdxy");
  }

  if (IS_ERR(data->hwmon_dev)) {
    printk(KERN_INFO "(ERROR) Fail to register device.\n");
    err = PTR_ERR(data->hwmon_dev);
    goto exit_remove_files;
  }
  return 0;
exit_remove_files:
  lcd_remove_files(client, data);
  return err;
}

static int lcd_remove(struct i2c_client *client) 
{
  struct lcd_data *data = i2c_get_clientdata(client);
  printk(KERN_INFO "(DEBUG) lcd_remove()\n");

  lcd_send_cmd(client, LCD_CLEARDISPLAY , 5);

  device_unregister(data->hwmon_dev);
  class_destroy(lcd_class);
  lcd_remove_files(client, data);
  return 0;
}

static struct i2c_driver lcd_driver = {
	.driver = {
		.name = "lcd_driver",
    		.owner  = THIS_MODULE,	
	},
	.probe		= lcd_probe,
	.remove		= lcd_remove,
	.detect		= lcd_detect,
	.id_table	= lcd_id,
	.address_list	= normal_i2c,
};

static int __init _lcd_init(void) {
  return i2c_add_driver(&lcd_driver);
};
//subsys_initcall(_lcd_init);
module_init(_lcd_init);

static void __exit _lcd_exit(void) {
	i2c_del_driver(&lcd_driver);
};
module_exit(_lcd_exit);


MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

