/*-------------------------------------------------------------
 * Linux on RoboX project - ASL ETHZ
 *-------------------------------------------------------------
 *
 * SBS Technologies IP Quadrature linux driver for kernel 2.6
 *
 *-------------------------------------------------------------
 * $Id: ipquad.c,v 1.2 2008/04/27 08:25:15 kralf Exp $
 *-------------------------------------------------------------*/

#include <linux/version.h>
#include "config.h"

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <asm/semaphore.h>

#include "tpmodule.c"
#include "ipquad.h"
#include "ipac.h"

/*
==============================================================================
        Constant definitions
==============================================================================
*/

// IDs and names
#define IPQUAD_VENDORID               0xF0  // Vendor ID
#define IPQUAD_DEVICEID               0x41  // Device ID
#define IPQUAD_NAME                   "SBS Technologies IP Quadrature decoder"
#define IPQUAD_DRVNAME                "ipquad"
#define IPQUAD_DRVBINVERSION          0x00001

// Hardware limits
#define IPQUAD_NUM_CHANNELS           4

// Character devices
#define IPQUAD_MAJOR_BASE             212
#define IPQUAD_DEVCLASS               "ipquad"
#define IPQUAD_DEVNAME                "ipquad"
#define IPQUAD_FORMAT                 "%d"

// Register offsets and access masks
#define IPQUAD_CTRLSTAT_OFFSET        0x31
#define IPQUAD_INTVECT_OFFSET         0x35

#define IPQUAD_CHAN_SPAN              4

#define IPQUAD_CHAN_OLPCR_OFFSET      0x01
#define IPQUAD_CHAN_CTRLSTAT_OFFSET   0x03
#define IPQUAD_CHAN_COUNTERCTRL_MASK  0x00
#define IPQUAD_CHAN_INCTRL_MASK       0x40
#define IPQUAD_CHAN_OUTCTRL_MASK      0x80
#define IPQUAD_CHAN_QUADCTRL_MASK     0xC0
#define IPQUAD_CHAN_CONF_OFFSET       0x21

// Control chars
#define IPQUAD_TERM1_ENABLE           0xF9
#define IPQUAD_TERM1_DISABLE          0xF8
#define IPQUAD_TERM2_ENABLE           0xFA
#define IPQUAD_TERM2_DISABLE          0xF8
#define IPQUAD_INTVEC_INIT            0x0F

#define IPQUAD_CHAN_MASTER_RESET      0x20

#define IPQUAD_CHAN_READ_OL           0x03
#define IPQUAD_CHAN_WRITE_PCR         0x01
#define IPQUAD_CHAN_TRANSFER_PC       0x08

#define IPQUAD_CHAN_COUNTER_DISABLE   0x00
#define IPQUAD_CHAN_COUNTER_ENABLE    0x08

#define IPQUAD_CHAN_INT_BORROW        0x10
#define IPQUAD_CHAN_INT_MATCH         0x30

#define IPQUAD_CHAN_QUAD_DISABLE      0x00
#define IPQUAD_CHAN_QUAD_X1           0x01
#define IPQUAD_CHAN_QUAD_X2           0x02
#define IPQUAD_CHAN_QUAD_X4           0x03

#define IPQUAD_CHAN_X_INVERT          0x01
#define IPQUAD_CHAN_Y_INVERT          0x02
#define IPQUAD_CHAN_Z_INVERT          0x04

/*
==============================================================================
        Kernel messages and debugging
==============================================================================
*/

static int debug;

#define ipquad_printk(fmt, arg...) {\
  char message[256]; \
  sprintf(message, fmt, ## arg); \
  printk("%s: %s", IPQUAD_DRVNAME, message); \
}
#define ipquad_alertk(fmt, arg...) \
  ipquad_printk(KERN_ALERT fmt, ## arg)
#define ipquad_debugk(fmt, arg...) \
  if (debug) ipquad_printk(KERN_DEBUG fmt, ## arg)

/*
==============================================================================
        Driver structures and declarations
==============================================================================
*/

static struct class* ipquad_class;    // device class

int ipquad_device_open(struct inode *inode, struct file *file);
int ipquad_device_release(struct inode *inode, struct file *file);
int ipquad_device_read(struct file *file, char *buff, size_t len,
  loff_t *f_pos);
int ipquad_device_write(struct file *file, const char *buff, size_t len,
  loff_t *off);

static struct file_operations ipquad_fops = {
  .open = ipquad_device_open,
  .release = ipquad_device_release,
  .read = ipquad_device_read,
  .write = ipquad_device_write,
};

struct ipac_module_id ipquad_id[] = {
  {
    manufacturer: IPQUAD_VENDORID,
    model_number: IPQUAD_DEVICEID,
    slot_config:  IPAC_INT0_EN | IPAC_INT1_EN | IPAC_LEVEL_SENS | IPAC_CLK_8MHZ,
    mem_size:     0,
    private_data: 0,
  },
  {
    manufacturer: 0,
    model_number: 0,
  }
};

typedef struct {
  struct addr_space_desc *io_space;

  struct semaphore sem;                 // semaphore

  // char device
  struct class* dev_class;              // device class
  struct cdev cdev;                     // char device
} ipquad_device;

static ipquad_device* ipquad_dev = 0;

/*
==============================================================================
       Kernel driver code
==============================================================================
*/

MODULE_AUTHOR("Frederic Pont and Ralf Kaestner");
MODULE_DESCRIPTION("Linux driver for SBS Technologies IP Quadrature decoder");
MODULE_LICENSE("GPL");
module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug enabled or not");

/*
==============================================================================
  More forward declarations
==============================================================================
*/

int ipquad_init_device();
int ipquad_init_channel(int channel);

/*
==============================================================================
        Char device registration and release
==============================================================================
*/

static void ipquad_register_cdevs(struct class* cdev_class, struct cdev*
  reg_cdev, unsigned int cdev_major, unsigned int device, const char* 
  cdev_bname) {
  unsigned int cdev_minor = reg_cdev->count;
  int i;

  if (!cdev_add(reg_cdev, MKDEV(cdev_major, cdev_minor),
    reg_cdev->count+IPQUAD_NUM_CHANNELS)) {
    ipquad_printk("Registering %d channel char devices at %s[%d-%d]\n",
      IPQUAD_NUM_CHANNELS, cdev_bname, 0, IPQUAD_NUM_CHANNELS-1);

    for(i = 0; i < IPQUAD_NUM_CHANNELS; i++) {
      char cdev_fname[256];
      sprintf(cdev_fname, "%s%d", cdev_bname, i);

      class_device_create(cdev_class, MKDEV(cdev_major, cdev_minor+i),
        NULL, cdev_fname);

      ipquad_debugk("Channel %d at %s (major %d, minor %d)\n", 
        i, cdev_fname, cdev_major, cdev_minor+i);
    }
  }
  else ipquad_alertk("Could not register channel char devices\n");
}

static void ipquad_unregister_cdevs(struct class* cdev_class, struct cdev*
  unreg_cdev) {
  int i;

  if (unreg_cdev->count) {
    for(i = 0; i < unreg_cdev->count; i++)
    class_device_destroy(cdev_class, unreg_cdev->dev+i);

    cdev_del(unreg_cdev);

    ipquad_printk("Unregistered all char devices\n");
  }
}

/*
==============================================================================
        Device initialization and removal
==============================================================================
*/

static int ipquad_probe(struct ipac_module *ipac, const struct ipac_module_id 
  *module_id) {
  int result = 0;
  struct addr_space_desc *io_space;
  int i;

  if (ipquad_dev) {
      ipquad_printk("This driver supports a single device only\n");
      return -1;
  }

  ipquad_printk("Probe new IPQuad decoder mounted on <%s> at slot %c\n",
    ipac->carrier_drv->name, 'A'+ipac->slot.slot_index);

  //  first try to map the IPAC IO space
  if ((io_space = ipac_map_space(ipac, IPAC_IO_SPACE)) == 0) {
      ipquad_printk("Unable to map IPAC IO space\n");
      return -1;
  }

  // init driver structure
  ipquad_dev = kmalloc(sizeof(ipquad_device), GFP_KERNEL);

  ipquad_dev->io_space = io_space;
  ((struct ipac_module_id*)module_id)->private_data = (unsigned long)ipquad_dev;
  ipquad_debugk("Device at 0x%x enabled\n", io_space->physical_address);

  // init mutex
  sema_init(&ipquad_dev->sem, 1);

  // init device and channels
  if (ipquad_init_device())
    return -1;

  for (i = 0; i < IPQUAD_NUM_CHANNELS; ++i)
    if (ipquad_init_channel(i))
    return -1;

  // register char devices
  ipquad_dev->dev_class = ipquad_class;
  cdev_init(&ipquad_dev->cdev, &ipquad_fops);

  ipquad_register_cdevs(ipquad_dev->dev_class, &ipquad_dev->cdev,
    IPQUAD_MAJOR_BASE+IPQUAD_NUM_CHANNELS, IPQUAD_NUM_CHANNELS, 
    IPQUAD_DEVNAME);

  return 0;
}

struct ipac_driver ipquad_drv = {
    name:       IPQUAD_DRVNAME,
    version:    IPQUAD_DRVBINVERSION,
    id_table:   ipquad_id,
    probe:      ipquad_probe,
};

static int __init ipquad_init(void) {
  // create device class
  ipquad_class = class_create(THIS_MODULE, IPQUAD_DEVCLASS);

  // find devices and register driver
  if (ipac_register_driver(&ipquad_drv) == -1)
    return -ENODEV;

  return 0;
}

static void ipqad_cleanup(void) {
  // unregister char devices
  ipquad_unregister_cdevs(ipquad_dev->dev_class, &ipquad_dev->cdev);

  // unregister driver
  ipac_unregister_driver(&ipquad_drv);

  // destroy device class
  class_destroy(ipquad_class);

  // free memory
  kfree(ipquad_dev);
  ipquad_dev = 0;  
}

module_init(ipquad_init);
module_exit(ipqad_cleanup);

/*
==============================================================================

        Name:   ipquad_init_device

        Description:
                Initialize decoder device.

==============================================================================
*/

int ipquad_init_device() {
  ipquad_debugk("Initializing device\n");

  down(&ipquad_dev->sem);
  ipac_write_uchar(ipquad_dev->io_space, IPQUAD_CTRLSTAT_OFFSET, 
    IPQUAD_TERM1_DISABLE | IPQUAD_TERM2_DISABLE);
  ipac_write_uchar(ipquad_dev->io_space, IPQUAD_INTVECT_OFFSET, 
    IPQUAD_INTVEC_INIT);
  up(&ipquad_dev->sem);

  return 0;
}

/*
==============================================================================

        Name:   ipquad_init_channel

        Description:
                Initializes decoder channel.

==============================================================================
*/

int ipquad_init_channel(int channel) {
  if ((channel < 0) || (channel >= IPQUAD_NUM_CHANNELS)) 
    return -ENODEV;

  ipquad_debugk("Initializing channel %d\n", channel);

  unsigned int chan_ctrlstat_offset = IPQUAD_CHAN_CTRLSTAT_OFFSET+
    IPQUAD_CHAN_SPAN*channel;

  down(&ipquad_dev->sem);
  ipac_write_uchar(ipquad_dev->io_space, chan_ctrlstat_offset, 
    IPQUAD_CHAN_COUNTERCTRL_MASK | IPQUAD_CHAN_MASTER_RESET);

  ipac_write_uchar(ipquad_dev->io_space, chan_ctrlstat_offset, 
    IPQUAD_CHAN_OUTCTRL_MASK | IPQUAD_CHAN_INT_BORROW);
  ipac_write_uchar(ipquad_dev->io_space, chan_ctrlstat_offset, 
    IPQUAD_CHAN_QUADCTRL_MASK | IPQUAD_CHAN_QUAD_X4);
  ipac_write_uchar(ipquad_dev->io_space, chan_ctrlstat_offset, 
    IPQUAD_CHAN_INCTRL_MASK | IPQUAD_CHAN_COUNTER_ENABLE);

  up(&ipquad_dev->sem);

  return 0;
}

/*
==============================================================================

        Name:   ipquad_read_channel

        Description:
                Reads decoder channel.

==============================================================================
*/

int ipquad_read_channel(int channel, int* value) {
  if ((channel < 0) || (channel >= IPQUAD_NUM_CHANNELS)) 
    return -ENODEV;

  ipquad_debugk("Reading channel %d\n", channel);

  unsigned int chan_ctrlstat_offset = IPQUAD_CHAN_CTRLSTAT_OFFSET+
    IPQUAD_CHAN_SPAN*channel;
  unsigned int chan_olpcr_offset = IPQUAD_CHAN_OLPCR_OFFSET+
    IPQUAD_CHAN_SPAN*channel;
  unsigned char *uchar_val = (unsigned char*)value;
  
  down(&ipquad_dev->sem);
  ipac_write_uchar(ipquad_dev->io_space, chan_ctrlstat_offset, 
    IPQUAD_CHAN_COUNTERCTRL_MASK | IPQUAD_CHAN_READ_OL);

  uchar_val[3] = ipac_read_uchar(ipquad_dev->io_space, chan_olpcr_offset);
  uchar_val[2] = ipac_read_uchar(ipquad_dev->io_space, chan_olpcr_offset);
  uchar_val[1] = ipac_read_uchar(ipquad_dev->io_space, chan_olpcr_offset);
  uchar_val[0] = 0;

  up(&ipquad_dev->sem);

  return 0;
}

/*
==============================================================================

        Name:   ipquad_write_channel

        Description:
                Writes decoder channel.

==============================================================================
*/

int ipquad_write_channel(int channel, int value) {
  if ((channel < 0) || (channel >= IPQUAD_NUM_CHANNELS)) 
    return -ENODEV;

  ipquad_debugk("Writing channel %d\n", channel);

  unsigned int chan_ctrlstat_offset = IPQUAD_CHAN_CTRLSTAT_OFFSET+
    IPQUAD_CHAN_SPAN*channel;
  unsigned int chan_olpcr_offset = IPQUAD_CHAN_OLPCR_OFFSET+
    IPQUAD_CHAN_SPAN*channel;
  unsigned char *uchar_val = (unsigned char*)&value;
  
  down(&ipquad_dev->sem);
  ipac_write_uchar(ipquad_dev->io_space, chan_ctrlstat_offset, 
    IPQUAD_CHAN_COUNTERCTRL_MASK | IPQUAD_CHAN_WRITE_PCR);

  ipac_write_uchar(ipquad_dev->io_space, chan_olpcr_offset, uchar_val[3]);
  ipac_write_uchar(ipquad_dev->io_space, chan_olpcr_offset, uchar_val[2]);
  ipac_write_uchar(ipquad_dev->io_space, chan_olpcr_offset, uchar_val[1]);

  ipac_write_uchar(ipquad_dev->io_space, chan_ctrlstat_offset, 
    IPQUAD_CHAN_COUNTERCTRL_MASK | IPQUAD_CHAN_TRANSFER_PC);

  up(&ipquad_dev->sem);

  return 0;
}

/*
==============================================================================

        Name:   ipquad_invert_channel

        Description:
                Inverts decoder channel.

==============================================================================
*/

int ipquad_invert_channel(int channel, int invert) {
  if ((channel < 0) || (channel >= IPQUAD_NUM_CHANNELS)) 
    return -ENODEV;

  ipquad_debugk("Inverting channel %d\n", channel);

  unsigned int chan_conf_offset = IPQUAD_CHAN_CONF_OFFSET+
    IPQUAD_CHAN_SPAN*channel;
  
  down(&ipquad_dev->sem);
  ipac_write_uchar(ipquad_dev->io_space, chan_conf_offset, 
    (invert) ? IPQUAD_CHAN_X_INVERT | IPQUAD_CHAN_Y_INVERT : 0x00);
  up(&ipquad_dev->sem);
  
  return 0;
}

/*
==============================================================================
        Read/write char devices
==============================================================================
*/

int ipquad_device_open(struct inode *inode, struct file *file) {
  ipquad_device *ipquad_dev;
  unsigned int chan = iminor(inode);

  if (chan < IPQUAD_NUM_CHANNELS) {
    // find and associate device with file
    ipquad_dev = container_of(inode->i_cdev, ipquad_device, cdev);
    file->private_data = ipquad_dev;
  
    return 0;
  }
  else
    return -EINVAL;
}

int ipquad_device_release(struct inode *inode, struct file *file) {
  // nothing to do here

  return 0;
}

int ipquad_device_read(struct file *file, char *buff, size_t len, 
  loff_t *f_pos) {
  int result = 0;
  ipquad_device *ipquad_dev =  file->private_data;
  unsigned int chan = iminor(file->f_dentry->d_inode);
  int val;
  char out[256];
  unsigned int out_len;

  if (!ipquad_read_channel(chan, &val))
    sprintf(out, IPQUAD_FORMAT, val);
  else
    result = -EFAULT;

  if (!result) {
    mdelay(500);
    strcat(out, "\n");
    out_len = strlen(out);

    if (len >= out_len)
      if (!copy_to_user(buff, out, out_len)) result = out_len;
  }

  return result;
}

int ipquad_device_write(struct file *file, const char *buff, size_t len,
  loff_t *off) {
  ipquad_device *ipquad_dev =  file->private_data;
  unsigned int chan = iminor(file->f_dentry->d_inode);
  int val;
  char in[len+1];

  if (!copy_from_user(in, buff, len)) {
    if ((len == 2) && ((buff[0] == '+') || (buff[0] == '-'))) {
      if (!ipquad_invert_channel(chan, (buff[0] == '-')))
        return len;
      else 
        return -EFAULT;
    }
    else {
      if ((sscanf(in, IPQUAD_FORMAT, &val) == 1) &&
        !ipquad_write_channel(chan, val))
        return len;
      else 
        return -EFAULT;
    }
  }
  else
    return -EFAULT;
}
