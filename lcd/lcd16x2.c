/*##################################################################################################
 *#	LCD 16x2 Driver
 *#
 *#	Author:	Tiago Sousa Rocha
 *#	Desc:	This driver will do the lcd control.
 *#
 *##################################################################################################
 * lcd16x2.c - Linux kernel modules
 *
 * Copyright (C) 2003-2010  Tiago Sousa Rocha <tsrrocha@gmail.com>
 *
 *##################################################################################################
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
 *################################################################################################*/
  #include <linux/kernel.h>
  #include <linux/init.h>
  #include <linux/module.h>
  #include <linux/slab.h>
  #include <linux/delay.h>
  #include <linux/mutex.h>
  #include <linux/sysfs.h>
  #include <linux/mod_devicetable.h>
  #include <linux/log2.h>
  #include <linux/bitops.h>
  #include <linux/jiffies.h>
  #include <linux/of.h>
  #include <linux/i2c.h>
  #include <linux/i2c/lcd16x2.h>
  #include <linux/i2c/eeprom.h>
//##################################################################################################
//#	Defines:
//##################################################################################################
  #define DRV_AUTHOR			"Tiago Sousa Rocha <tsrrocha@gmail.com>"
  #define DRV_DESCRIPTION		"LCD 16x2 & 16x4 display driver"
  #define DRV_VERSION			"1.00.r001"		// Versão do driver
  #define DRV_NAME				"lcd16x2"		// 
  #define DRV_LICENSE			"GPL"			// 
//##################################################################################################
  #define LCD_REG_MAN_ID		0x27
  #define LCD_REG_CHIP_ID		0xFF
//##################################################################################################
  #define LCD_BL			0x08
  #define LCD_EN			0x04  // Enable bit
  #define LCD_RW			0x02  // Read/Write bit
  #define LCD_RS			0x01  // Register select bit
  #define LCD_MASK_DISABLE_EN	0xFB  // MASK of control
  #define STROBE_EN_DELAY 	1		// nanosegundos
  #define CLEAR_DELAY 		1		//
  #define COMMAND_INIT_DELAY 	1		// ms
//##################################################################################################
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
  #define AT24_SIZE_BYTELEN 5
  #define AT24_SIZE_FLAGS 8

  #define AT24_BITMASK(x) (BIT(x) - 1)

  // create non-zero magic value for given eeprom parameters 
  #define AT24_DEVICE_MAGIC(_len, _flags) 		\
	((1 << AT24_SIZE_FLAGS | (_flags)) 		\
	    << AT24_SIZE_BYTELEN | ilog2(_len))
//##################################################################################################



//##################################################################################################
//#	Types and Variables:
//##################################################################################################
  // define type BYTE
  typedef unsigned char BYTE;
  // Flag for backlight
  //static u8 backlightFlag 	= 0x01;	// Flag de in
  // LCD data
  //static u8 lcd_data 		= 0x00;
  // Mutex para controle de acesso concorrente
  //static DEFINE_MUTEX(lcd16x2_mutex);
  // Classe do device
  static struct class *lcd_class;
//##################################################################################################




//##################################################################################################
//#	Struct:
//##################################################################################################
struct lcd16x2_data {
	struct lcd16x2_platform_data chip;
	struct memory_accessor macc;
	int use_smbus;

	/*
	 * Lock protects against activities from other Linux tasks,
	 * but not from changes by other I2C masters.
	 */
	struct mutex lock;
	struct bin_attribute bin;

	u8 *writebuf;
	unsigned write_max;
	unsigned num_addresses;

	/*
	 * Some chips tie up multiple I2C addresses; dummy devices reserve
	 * them for us, and we'll use them with SMBus calls.
	 */
	struct i2c_client *client[];
};
//##################################################################################################





//##################################################################################################
//#	Module Params:
//##################################################################################################
/*
 * This parameter is to help this driver avoid blocking other drivers out
 * of I2C for potentially troublesome amounts of time. With a 100 kHz I2C
 * clock, one 256 byte read takes about 1/43 second which is excessive;
 * but the 1/170 second it takes at 400 kHz may be quite reasonable; and
 * at 1 MHz (Fm+) a 1/430 second delay could easily be invisible.
 *
 * This value is forced to be a power of two so that writes align on pages.
 */
static unsigned io_limit = 128;
module_param(io_limit, uint, 0);
MODULE_PARM_DESC(io_limit, "Maximum bytes per I/O (default 128)");

/*
 * Specs often allow 5 msec for a page write, sometimes 20 msec;
 * it's important to recover from write timeouts.
 */
static unsigned write_timeout = 25;
module_param(write_timeout, uint, 0);
MODULE_PARM_DESC(write_timeout, "Time (in ms) to try writes (default 25)");

//##################################################################################################




//##################################################################################################
//#	I2C Device ID:
//##################################################################################################
static const struct i2c_device_id lcd16x2_ids[] = {
	{ DRV_NAME, 0 },
	{ /* END OF LIST */ }
};
MODULE_DEVICE_TABLE(i2c, lcd16x2_ids);
//##################################################################################################










/*-------------------------------------------------------------------------*/

/*
 * This routine supports chips which consume multiple I2C addresses. It
 * computes the addressing information to be used for a given r/w request.
 * Assumes that sanity checks for offset happened at sysfs-layer.
 */
static struct i2c_client *at24_translate_offset(struct lcd16x2_data *at24, unsigned *offset) {
	unsigned i;

	if (at24->chip.flags & AT24_FLAG_ADDR16) {
		i = *offset >> 16;
		*offset &= 0xffff;
	} else {
		i = *offset >> 8;
		*offset &= 0xff;
	}

	return at24->client[i];
}

static ssize_t at24_eeprom_read(struct lcd16x2_data *at24, char *buf, unsigned offset, size_t count) {
	struct i2c_msg msg[2];
	u8 msgbuf[2];
	struct i2c_client *client;
	unsigned long timeout, read_time;
	int status, i;

	memset(msg, 0, sizeof(msg));

	/*
	 * REVISIT some multi-address chips don't rollover page reads to
	 * the next slave address, so we may need to truncate the count.
	 * Those chips might need another quirk flag.
	 *
	 * If the real hardware used four adjacent 24c02 chips and that
	 * were misconfigured as one 24c08, that would be a similar effect:
	 * one "eeprom" file not four, but larger reads would fail when
	 * they crossed certain pages.
	 */

	/*
	 * Slave address and byte offset derive from the offset. Always
	 * set the byte address; on a multi-master board, another master
	 * may have changed the chip's "current" address pointer.
	 */
	client = at24_translate_offset(at24, &offset);

	if (count > io_limit)
		count = io_limit;

	switch (at24->use_smbus) {
	case I2C_SMBUS_I2C_BLOCK_DATA:
		/* Smaller eeproms can work given some SMBus extension calls */
		if (count > I2C_SMBUS_BLOCK_MAX)
			count = I2C_SMBUS_BLOCK_MAX;
		break;
	case I2C_SMBUS_WORD_DATA:
		count = 2;
		break;
	case I2C_SMBUS_BYTE_DATA:
		count = 1;
		break;
	default:
		/*
		 * When we have a better choice than SMBus calls, use a
		 * combined I2C message. Write address; then read up to
		 * io_limit data bytes. Note that read page rollover helps us
		 * here (unlike writes). msgbuf is u8 and will cast to our
		 * needs.
		 */
		i = 0;
		if (at24->chip.flags & AT24_FLAG_ADDR16)
			msgbuf[i++] = offset >> 8;
		msgbuf[i++] = offset;

		msg[0].addr = client->addr;
		msg[0].buf = msgbuf;
		msg[0].len = i;

		msg[1].addr = client->addr;
		msg[1].flags = I2C_M_RD;
		msg[1].buf = buf;
		msg[1].len = count;
	}

	/*
	 * Reads fail if the previous write didn't complete yet. We may
	 * loop a few times until this one succeeds, waiting at least
	 * long enough for one entire page write to work.
	 */
	timeout = jiffies + msecs_to_jiffies(write_timeout);
	do {
		read_time = jiffies;
		switch (at24->use_smbus) {
		case I2C_SMBUS_I2C_BLOCK_DATA:
			status = i2c_smbus_read_i2c_block_data(client, offset,
					count, buf);
			break;
		case I2C_SMBUS_WORD_DATA:
			status = i2c_smbus_read_word_data(client, offset);
			if (status >= 0) {
				buf[0] = status & 0xff;
				buf[1] = status >> 8;
				status = count;
			}
			break;
		case I2C_SMBUS_BYTE_DATA:
			status = i2c_smbus_read_byte_data(client, offset);
			if (status >= 0) {
				buf[0] = status;
				status = count;
			}
			break;
		default:
			status = i2c_transfer(client->adapter, msg, 2);
			if (status == 2)
				status = count;
		}
		dev_dbg(&client->dev, "read %zu@%d --> %d (%ld)\n",
				count, offset, status, jiffies);

		if (status == count)
			return count;

		/* REVISIT: at HZ=100, this is sloooow */
		msleep(1);
	} while (time_before(read_time, timeout));

	return -ETIMEDOUT;
}

static ssize_t at24_read(struct lcd16x2_data *at24, char *buf, loff_t off, size_t count) {
	ssize_t retval = 0;

	if (unlikely(!count))
		return count;

	/*
	 * Read data from chip, protecting against concurrent updates
	 * from this host, but not from other I2C masters.
	 */
	mutex_lock(&at24->lock);

	while (count) {
		ssize_t	status;

		status = at24_eeprom_read(at24, buf, off, count);
		if (status <= 0) {
			if (retval == 0)
				retval = status;
			break;
		}
		buf += status;
		off += status;
		count -= status;
		retval += status;
	}

	mutex_unlock(&at24->lock);

	return retval;
}

static ssize_t at24_bin_read(struct file *filp, struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t off, size_t count) {
	struct lcd16x2_data *at24;

	at24 = dev_get_drvdata(container_of(kobj, struct device, kobj));
	return at24_read(at24, buf, off, count);
}


/*
 * Note that if the hardware write-protect pin is pulled high, the whole
 * chip is normally write protected. But there are plenty of product
 * variants here, including OTP fuses and partial chip protect.
 *
 * We only use page mode writes; the alternative is sloooow. This routine
 * writes at most one page.
 */
static ssize_t at24_eeprom_write(struct lcd16x2_data *at24, const char *buf, unsigned offset, size_t count) {
	struct i2c_client *client;
	struct i2c_msg msg;
	ssize_t status;
	unsigned long timeout, write_time;
	unsigned next_page;

	/* Get corresponding I2C address and adjust offset */
	client = at24_translate_offset(at24, &offset);

	/* write_max is at most a page */
	if (count > at24->write_max)
		count = at24->write_max;

	/* Never roll over backwards, to the start of this page */
	next_page = roundup(offset + 1, at24->chip.page_size);
	if (offset + count > next_page)
		count = next_page - offset;

	/* If we'll use I2C calls for I/O, set up the message */
	if (!at24->use_smbus) {
		int i = 0;

		msg.addr = client->addr;
		msg.flags = 0;

		/* msg.buf is u8 and casts will mask the values */
		msg.buf = at24->writebuf;
		if (at24->chip.flags & AT24_FLAG_ADDR16)
			msg.buf[i++] = offset >> 8;

		msg.buf[i++] = offset;
		memcpy(&msg.buf[i], buf, count);
		msg.len = i + count;
	}

	/*
	 * Writes fail if the previous one didn't complete yet. We may
	 * loop a few times until this one succeeds, waiting at least
	 * long enough for one entire page write to work.
	 */
	timeout = jiffies + msecs_to_jiffies(write_timeout);
	do {
		write_time = jiffies;
		if (at24->use_smbus) {
			status = i2c_smbus_write_i2c_block_data(client,
					offset, count, buf);
			if (status == 0)
				status = count;
		} else {
			status = i2c_transfer(client->adapter, &msg, 1);
			if (status == 1)
				status = count;
		}
		dev_dbg(&client->dev, "write %zu@%d --> %zd (%ld)\n",
				count, offset, status, jiffies);

		if (status == count)
			return count;

		/* REVISIT: at HZ=100, this is sloooow */
		msleep(1);
	} while (time_before(write_time, timeout));

	return -ETIMEDOUT;
}

static ssize_t at24_write(struct lcd16x2_data *at24, const char *buf, loff_t off,
			  size_t count)
{
	ssize_t retval = 0;

	if (unlikely(!count))
		return count;

	/*
	 * Write data to chip, protecting against concurrent updates
	 * from this host, but not from other I2C masters.
	 */
	mutex_lock(&at24->lock);

	while (count) {
		ssize_t	status;

		status = at24_eeprom_write(at24, buf, off, count);
		if (status <= 0) {
			if (retval == 0)
				retval = status;
			break;
		}
		buf += status;
		off += status;
		count -= status;
		retval += status;
	}

	mutex_unlock(&at24->lock);

	return retval;
}

static ssize_t at24_bin_write(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr,
		char *buf, loff_t off, size_t count)
{
	struct lcd16x2_data *at24;

	at24 = dev_get_drvdata(container_of(kobj, struct device, kobj));
	return at24_write(at24, buf, off, count);
}

/*-------------------------------------------------------------------------*/

/*
 * This lets other kernel code access the eeprom data. For example, it
 * might hold a board's Ethernet address, or board-specific calibration
 * data generated on the manufacturing floor.
 */

static ssize_t at24_macc_read(struct memory_accessor *macc, char *buf, off_t offset, size_t count) {
	struct lcd16x2_data *at24 = container_of(macc, struct lcd16x2_data, macc);

	return at24_read(at24, buf, offset, count);
}

static ssize_t at24_macc_write(struct memory_accessor *macc, const char *buf,
			  off_t offset, size_t count)
{
	struct lcd16x2_data *at24 = container_of(macc, struct lcd16x2_data, macc);

	return at24_write(at24, buf, offset, count);
}

/*-------------------------------------------------------------------------*/












//##################################################################################################
//#	Platform Data:
//##################################################################################################
#ifdef CONFIG_OF
static void lcd16x2_get_ofdata(struct i2c_client *client, struct lcd16x2_platform_data *chip) {
	const __be32 *val;
	struct device_node *node = client->dev.of_node;

	if (node) {
		if (of_get_property(node, "read-only", NULL))
			chip->flags |= AT24_FLAG_READONLY;
		val = of_get_property(node, "pagesize", NULL);
		if (val)
			chip->page_size = be32_to_cpup(val);
	}
}
#else
static void lcd16x2_get_ofdata(struct i2c_client *client,
		struct lcd16x2_platform_data *chip)
{ }
#endif /* CONFIG_OF */
//##################################################################################################






//##################################################################################################
//#	Drive:	Probe and Remove
//##################################################################################################
static int lcd16x2_drv_probe(struct i2c_client *client, const struct i2c_device_id *id) {
  struct lcd16x2_platform_data chip;
  bool writable;
  int use_smbus = 0;
  struct lcd16x2_data *at24;
  int err;
  unsigned i, num_addresses;
  kernel_ulong_t magic;
  printk(KERN_INFO "[DEBUG-LCD] %s \n", __func__);

  if (client->dev.platform_data) {
    chip = *(struct lcd16x2_platform_data *)client->dev.platform_data;
  } else {
    if (!id->driver_data) {
      err = -ENODEV;
      goto err_out;
    }
    magic = id->driver_data;
    chip.byte_len = BIT(magic & AT24_BITMASK(AT24_SIZE_BYTELEN));
    magic >>= AT24_SIZE_BYTELEN;
    chip.flags = magic & AT24_BITMASK(AT24_SIZE_FLAGS);
    /*
    * This is slow, but we can't know all eeproms, so we better
    * play safe. Specifying custom eeprom-types via platform_data
    * is recommended anyhow.
    */
    chip.page_size = 1;

    // update chipdata if OF is present 
    lcd16x2_get_ofdata(client, &chip);

    chip.setup = NULL;
    chip.context = NULL;
  }

  if (!is_power_of_2(chip.byte_len))
    dev_warn(&client->dev, "byte_len looks suspicious (no power of 2)!\n");
  if (!chip.page_size) {
    dev_err(&client->dev, "page_size must not be 0!\n");
    err = -EINVAL;
    goto err_out;
  }
  if (!is_power_of_2(chip.page_size))
    dev_warn(&client->dev, "page_size looks suspicious (no power of 2)!\n");

    // Use I2C operations unless we're stuck with SMBus extensions.
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
      if (chip.flags & AT24_FLAG_ADDR16) {
        err = -EPFNOSUPPORT;
        goto err_out;
      }
      if (i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_READ_I2C_BLOCK)) {
        use_smbus = I2C_SMBUS_I2C_BLOCK_DATA;
      } else if (i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_READ_WORD_DATA)) {
        use_smbus = I2C_SMBUS_WORD_DATA;
      } else if (i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_READ_BYTE_DATA)) {
        use_smbus = I2C_SMBUS_BYTE_DATA;
      } else {
        err = -EPFNOSUPPORT;
        goto err_out;
      }
  }

  if (chip.flags & AT24_FLAG_TAKE8ADDR)
    num_addresses = 8;
  else
    num_addresses =	DIV_ROUND_UP(chip.byte_len, (chip.flags & AT24_FLAG_ADDR16) ? 65536 : 256);

  at24 = kzalloc(sizeof(struct lcd16x2_data) + num_addresses * sizeof(struct i2c_client *), GFP_KERNEL);
  if (!at24) {
    err = -ENOMEM;
    goto err_out;
  }

  mutex_init(&at24->lock);
  at24->use_smbus = use_smbus;
  at24->chip = chip;
  at24->num_addresses = num_addresses;

  /*
   * Export the EEPROM bytes through sysfs, since that's convenient.
   * By default, only root should see the data (maybe passwords etc)
   */
  sysfs_bin_attr_init(&at24->bin);
  at24->bin.attr.name = "eeprom";
  at24->bin.attr.mode = chip.flags & AT24_FLAG_IRUGO ? S_IRUGO : S_IRUSR;
  at24->bin.read = at24_bin_read;
  at24->bin.size = chip.byte_len;
  at24->macc.read = at24_macc_read;

  writable = !(chip.flags & AT24_FLAG_READONLY);
  if (writable) {
    if (!use_smbus || i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WRITE_I2C_BLOCK)) {
      unsigned write_max = chip.page_size;
      at24->macc.write = at24_macc_write;
      at24->bin.write = at24_bin_write;
      at24->bin.attr.mode |= S_IWUSR;

      if (write_max > io_limit)
        write_max = io_limit;
      if (use_smbus && write_max > I2C_SMBUS_BLOCK_MAX)
        write_max = I2C_SMBUS_BLOCK_MAX;
      at24->write_max = write_max;

      // buffer (data + address at the beginning) 
      at24->writebuf = kmalloc(write_max + 2, GFP_KERNEL);
      if (!at24->writebuf) {
        err = -ENOMEM;
        goto err_struct;
      }
    } else {
      dev_warn(&client->dev, "cannot write due to controller restrictions.");
    }
  }

  at24->client[0] = client;

  // use dummy devices for multiple-address chips 
  for (i = 1; i < num_addresses; i++) {
    at24->client[i] = i2c_new_dummy(client->adapter, client->addr + i);
    if (!at24->client[i]) {
      dev_err(&client->dev, "address 0x%02x unavailable\n", client->addr + i);
      err = -EADDRINUSE;
      goto err_clients;
    }
  }

  err = sysfs_create_bin_file(&client->dev.kobj, &at24->bin);
  if (err)
    goto err_clients;

  i2c_set_clientdata(client, at24);

  dev_info(&client->dev, "%zu byte %s EEPROM, %s, %u bytes/write\n", at24->bin.size, client->name, writable ? "writable" : "read-only", at24->write_max);
  if (use_smbus == I2C_SMBUS_WORD_DATA ||
      use_smbus == I2C_SMBUS_BYTE_DATA) {
    dev_notice(&client->dev, "Falling back to %s reads, performance will suffer\n", use_smbus == I2C_SMBUS_WORD_DATA ? "word" : "byte");
  }

  // export data to kernel code 
  if (chip.setup)
    chip.setup(&at24->macc, chip.context);

  return 0;

err_clients:
  for (i = 1; i < num_addresses; i++)
    if (at24->client[i])
      i2c_unregister_device(at24->client[i]);

  kfree(at24->writebuf);
err_struct:
  kfree(at24);
err_out:
  dev_dbg(&client->dev, "probe error %d\n", err);
  return err;
}


static int lcd16x2_drv_remove(struct i2c_client *client) {
  struct lcd16x2_data *lcd;
  int i;
  printk(KERN_INFO "[DEBUG][%s] %s \n", DRV_VERSION, __func__);

  lcd = i2c_get_clientdata(client);
  sysfs_remove_bin_file(&client->dev.kobj, &lcd->bin);

  for (i = 1; i < lcd->num_addresses; i++)
    i2c_unregister_device(lcd->client[i]);

  kfree(lcd->writebuf);
  kfree(lcd);
  return 0;
}
//##################################################################################################





static int lcd16x2_drv_command(struct i2c_client *client, unsigned int cmd, void *arg) {
  struct lcd16x2_data *lcd;
  const struct memory_accessor **maccp;
  printk(KERN_INFO "[DEBUG][%s] %s \n", DRV_VERSION, __func__);

  // only supporting a single command 
  if (cmd != I2C_EEPROM_GET_MEMORY_ACCESSOR)
    return -ENOTSUPP;

  // rudimentary check 
  if (arg == NULL)
    return -EINVAL;

  lcd = i2c_get_clientdata(client);

  maccp = arg;
  *maccp = &lcd->macc;

  return 0;
}

/*-------------------------------------------------------------------------*/

static struct i2c_driver lcd_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe = lcd16x2_drv_probe,
	.remove = lcd16x2_drv_remove,
	.id_table = lcd16x2_ids,
	.command = lcd16x2_drv_command,
};

static int __init lcd16x2_driver_init(void)
{
  printk(KERN_INFO "[DEBUG][%s] %s \n", DRV_VERSION, __func__);
	if (!io_limit) {
		pr_err("at24: io_limit must not be 0!\n");
		return -EINVAL;
	}

	io_limit = rounddown_pow_of_two(io_limit);
	return i2c_add_driver(&lcd_driver);
}
module_init(lcd16x2_driver_init);

static void __exit lcd16x2_driver_exit(void)
{
  printk(KERN_INFO "[DEBUG][%s] %s \n", DRV_VERSION, __func__);
	i2c_del_driver(&lcd_driver);
}
module_exit(lcd16x2_driver_exit);

MODULE_AUTHOR(DRV_AUTHOR);
MODULE_DESCRIPTION(DRV_DESCRIPTION);
MODULE_VERSION(DRV_VERSION);
MODULE_LICENSE(DRV_LICENSE);
