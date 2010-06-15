/*-------------------------------------------------------------
 * Linux on RoboX project - ASL ETHZ
 *-------------------------------------------------------------
 *
 * PCore user LED linux driver
 *
 *-------------------------------------------------------------
 *
 *-------------------------------------------------------------*/

#include "userled.h"

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <asm/io.h>

/*
==============================================================================
        Constant definitions
==============================================================================
*/

// Names
#define USERLED_NAME            "PCore user LED"
#define USERLED_DRVNAME         "userled"

// I/O
#define USERLED_IO              0xfe000300

// Character device settings
#define USERLED_MAJOR           211
#define USERLED_MINOR           0
#define USERLED_DEVCLASS        "userled"
#define USERLED_DEVNAME         "userled"

// Character device states
#define USERLED_COFF            '0'
#define USERLED_CGREEN          'g'
#define USERLED_CRED            'r'

/*
==============================================================================
        Kernel messages and debugging
==============================================================================
*/

static int debug;

#define userled_printk(fmt, arg...) {\
  char message[256]; \
  sprintf(message, fmt, ## arg); \
  printk("%s: %s", USERLED_DRVNAME, message); \
}
#define userled_alertk(fmt, arg...) \
  userled_printk(KERN_ALERT fmt, ## arg)
#define userled_debugk(fmt, arg...) \
  if (debug) userled_printk(KERN_DEBUG fmt, ## arg)

/*
==============================================================================
	Driver structures and declarations
==============================================================================
*/

int userled_device_open(struct inode *inode, struct file *file);
int userled_device_release(struct inode *inode, struct file *file);
int userled_device_read(struct file *file, char *buff, size_t len,
  loff_t *f_pos);
int userled_device_write(struct file *file, const char *buff, size_t len,
  loff_t *off);

static struct file_operations userled_fops = {
  .open = userled_device_open,
  .release = userled_device_release,
  .read = userled_device_read,
  .write = userled_device_write,
};

typedef struct {
  /* char device */
  struct class* dev_class;              // device class
  struct cdev cdev;                     // char device
} userled_device;

static userled_device uled_dev;

/*
==============================================================================
       Kernel driver code
==============================================================================
*/

MODULE_AUTHOR("Ralf Kaestner");
MODULE_DESCRIPTION("Linux driver for PCore user LED");
MODULE_LICENSE("GPL");
module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug enabled or not");

/*
==============================================================================
        Device initialization and removal
==============================================================================
*/

static int __init userled_init(void) {
  // create device class
  uled_dev.dev_class = class_create(THIS_MODULE, USERLED_DEVCLASS);

  userled_printk("Registering char device at %s\n", USERLED_DEVNAME);

  // create char device
  cdev_init(&uled_dev.cdev, &userled_fops);

  if (!cdev_add(&uled_dev.cdev, MKDEV(USERLED_MAJOR, USERLED_MINOR), 1)) {
    class_device_create(uled_dev.dev_class, uled_dev.cdev.dev, NULL,
      USERLED_DEVNAME);
  }
  else userled_alertk("Could not register char device\n");

  return 0;
}

static void __exit userled_exit(void) {
  // destroy char device
  if (uled_dev.cdev.count) {
    class_device_destroy(uled_dev.dev_class, uled_dev.cdev.dev);

    cdev_del(&uled_dev.cdev);

    userled_printk("Unregistered char device\n");
  }

  // destroy device class
  class_destroy(uled_dev.dev_class);
}

module_init(userled_init);
module_exit(userled_exit);

/*
==============================================================================
        Read/write the device
==============================================================================
*/

unsigned char userled_read(void) {
  unsigned char value = 0x0F;

  value &= readb((void __iomem*)USERLED_IO);

  userled_debugk("RD value=%d\n", value);

  return value;
}

int userled_write(unsigned char value) {
  userled_debugk("WR value=%d\n", value);

  writeb(value, (void __iomem*)USERLED_IO);

  return 1;
}

/*
==============================================================================
        Read/write the char device
==============================================================================
*/

int userled_device_open(struct inode *inode, struct file *file) {
  return 0;
}

int userled_device_release(struct inode *inode, struct file *file) {
  return 0;
}

int userled_device_read(struct file *file, char *buff, size_t len,
  loff_t *f_pos) {
  int result = -EFAULT;
  unsigned char value;
  char cvalue = USERLED_COFF;

  userled_debugk("EDBG: BGN: userled_device_read(...)\n");

  if (len >= sizeof(cvalue)) {
    value = userled_read();

    switch (value) {
      case USERLED_GREEN:
        cvalue = USERLED_CGREEN;
      break;
      case USERLED_RED:
        cvalue = USERLED_CRED;
      break;
    }

    if (!copy_to_user(buff, &cvalue, sizeof(cvalue)))
      result = sizeof(cvalue);
  }

  userled_debugk("EDBG: END: userled_device_read(...)\n");

  return result;
}

int userled_device_write(struct file *file, const char *buff, size_t len,
  loff_t *off) {
  int result = -EFAULT;
  unsigned char value = USERLED_OFF;
  char cvalue;

  userled_debugk("EDBG: BGN: userled_device_write(...)\n");

  if (len >= sizeof(cvalue)) {
     if (!copy_from_user(&cvalue, buff, sizeof(cvalue))) {
      switch (cvalue) {
        case USERLED_CGREEN:
          value = USERLED_GREEN;
        break;
        case USERLED_CRED:
          value = USERLED_RED;
        break;
      }

      result = userled_write(value);
    }
  }

  userled_debugk("EDBG: END: userled_device_write(...)\n");

  return result;
}
