/*-------------------------------------------------------------
 * Linux on RoboX project - ASL EPFL
 *-------------------------------------------------------------
 *
 * icpmulti linux driver.
 *
 * WARNING:
 * ELROB version: adapted from RTAI version from
 * robox-linux. removed all RTAI stuff, no longer safe,
 * no mutual exclusion...
 *
 * very much inspired by the comedi ICP-multi driver, but
 * taken out of comedi.
 *
 * char device implementation and non-realtime mutual exclusion
 * by Ralf Kaestner
 *
 *-------------------------------------------------------------
 * $Id: icpmulti.c,v 1.5 2004/01/12 12:54:08 fred Exp $
 *-------------------------------------------------------------*/

#include "icpmulti.h"

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

/*
==============================================================================
        Constant definitions
==============================================================================
*/

// IDs and names
#define ICPMULTI_VENDORID       0x104c  // Vendor ID
#define ICPMULTI_DEVICEID	      0x8000	// Device ID
#define ICPMULTI_NAME           "ICP-multi composite A&D I/O board"
#define ICPMULTI_DRVNAME        "icpmulti"

// Hardware types of the cards
#define ICPMULTI_TYPE	        0

#define ICPMULTI_IORANGE 	32

#define ICPMULTI_ADC_CSR	0	// R/W:	ADC command/status register
#define ICPMULTI_AI		2	// R:	Analog input data
#define ICPMULTI_DAC_CSR	4	// R/W:	DAC command/status register
#define ICPMULTI_AO		6	// R/W:	Analog output data
#define ICPMULTI_DI		8	// R/W:	Digital inputs
#define ICPMULTI_DO		0x0A	// R/W:	Digital outputs
#define ICPMULTI_INT_EN	        0x0C	// R/W:	Interrupt enable register
#define ICPMULTI_INT_STAT	0x0E	// R/W:	Interrupt status register
#define ICPMULTI_CNTR0		0x10	// R/W:	Counter 0
#define ICPMULTI_CNTR1		0x12	// R/W:	counter 1
#define ICPMULTI_CNTR2		0x14	// R/W:	Counter 2
#define ICPMULTI_CNTR3		0x16	// R/W:	Counter 3

#define ICPMULTI_SIZE		0x20	// 32 bytes

// Define bits from ADC command/status register
#define	ADC_ST		0x0001    // Start ADC
#define	ADC_BSY		0x0001	  // ADC busy
#define ADC_BI		0x0010    // Bipolar input range 1 = bipolar
#define ADC_RA		0x0020	  // Input range 0 = 5V, 1 = 10V
#define	ADC_DI		0x0040	  // Differential input mode 1 = differential

// Define bits from DAC command/status register
#define	DAC_ST		0x0001    // Start DAC
#define DAC_BSY		0x0001    // DAC busy
#define	DAC_BI		0x0010    // Bipolar input range 1 = bipolar
#define	DAC_RA		0x0020    // Input range 0 = 5V, 1 = 10V

// Define bits from interrupt enable/status registers
#define	ADC_READY	0x0001    // A/d conversion ready interrupt
#define	DAC_READY	0x0002    // D/a conversion ready interrupt
#define	DOUT_ERROR	0x0004    // Digital output error interrupt
#define	DIN_STATUS	0x0008    // Digital input status change interrupt
#define	CIE0		0x0010    // Counter 0 overrun interrupt
#define	CIE1		0x0020    // Counter 1 overrun interrupt
#define	CIE2		0x0040    // Counter 2 overrun interrupt
#define	CIE3		0x0080    // Counter 3 overrun interrupt

// Useful definitions
#define	Status_IRQ	0x00ff    // All interrupts

// Character devices
#define ICPMULTI_MAJOR_BASE 210
#define ICPMULTI_DEVCLASS "icp"
#define ICPMULTI_DEVNAME "icp"
#define ICPMULTI_DEVNAME_AI "ai"
#define ICPMULTI_DEVNAME_AO "ao"
#define ICPMULTI_DEVNAME_DI "di"
#define ICPMULTI_DEVNAME_DO "do"
#define ICPMULTI_FORMAT "%d"

/*
==============================================================================
        Kernel messages and debugging
==============================================================================
*/

static int debug;

#define icpmulti_printk(fmt, arg...) {\
  char message[256]; \
  sprintf(message, fmt, ## arg); \
  printk("%s: %s", ICPMULTI_DRVNAME, message); \
}
#define icpmulti_alertk(fmt, arg...) \
  icpmulti_printk(KERN_ALERT fmt, ## arg)
#define icpmulti_debugk(fmt, arg...) \
  if (debug) icpmulti_printk(KERN_DEBUG fmt, ## arg)

/*
==============================================================================
	Driver structures and declarations
==============================================================================
*/

static struct class* icpmulti_class;    // device class

int icpmulti_device_open(struct inode *inode, struct file *file);
int icpmulti_device_release(struct inode *inode, struct file *file);
int icpmulti_device_read(struct file *file, char *buff, size_t len,
  loff_t *f_pos);
int icpmulti_device_write(struct file *file, const char *buff, size_t len,
  loff_t *off);

static struct file_operations icpmulti_fops = {
  .open = icpmulti_device_open,
  .release = icpmulti_device_release,
  .read = icpmulti_device_read,
  .write = icpmulti_device_write,
};

typedef struct {
  struct pci_dev *pdev;                 // PCI device
  unsigned char irq;                    // Interrupt request
  unsigned long phys_iobase;		// Physical I/O address
  void __iomem *iobase;                 // Kernel space I/O address
  unsigned int AdcCmdStatus;		// ADC Command/Status register
  unsigned int DacCmdStatus;		// DAC Command/Status register
  unsigned int IntEnable;		// Interrupt Enable register
  unsigned int IntStatus;		// Interrupt Status register
  unsigned int act_chanlist[32];	// list of scaned channel
  unsigned char act_chanlist_len;	// len of scanlist
  unsigned char act_chanlist_pos;	// actual position in MUX list
  unsigned int *ai_chanlist;		// actual chanlist
  unsigned char *ai_data;		// data buffer
  unsigned short ao_data[4];		// Analog data output buffer
  unsigned char di_data;		// Digital input data
  unsigned char do_data[8];	        // Digital data output buffer
  unsigned char do_state;

  int n_aichan;	                        // num of A/D chans
  int n_aochan;	                        // num of D/A chans
  int n_dichan;	                        // num of DI chans
  int n_dochan;	                        // num of DO chans

  // mutexes
  struct semaphore ai_sem;              // analog in semaphore
  struct semaphore ao_sem;              // analog out semaphore
  struct semaphore di_sem;              // digital in semaphore
  struct semaphore do_sem;              // digital out semaphore

  // char device
  struct class* dev_class;              // device class
  struct cdev cdev;                     // char device
} icpmulti_device;

static unsigned int icpmulti_num_devs;          // the number of devices
static icpmulti_device* icpmulti_default;       // the default device

/*
==============================================================================
       Kernel driver code
==============================================================================
*/

MODULE_AUTHOR("Frederic Pont and Ralf Kaestner");
MODULE_DESCRIPTION("Linux driver for ICP-multi composite A&D I/O board");
MODULE_LICENSE("GPL");
module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug enabled or not");

static struct pci_device_id icpmulti_pci_table[] = {
  { PCI_DEVICE(ICPMULTI_VENDORID, ICPMULTI_DEVICEID) },
  { 0, } // Terminating entry
};
MODULE_DEVICE_TABLE(pci, icpmulti_pci_table);

int icpmulti_probe(struct pci_dev *pdev, const struct pci_device_id *ent);
void icpmulti_remove(struct pci_dev *pdev);

static struct pci_driver icpmulti_driver = {
  .name = ICPMULTI_DRVNAME,
  .owner = THIS_MODULE,
  .id_table = icpmulti_pci_table,
  .probe = icpmulti_probe,
  .remove = icpmulti_remove
};

/*
==============================================================================
	More forward declarations
==============================================================================
*/

static int icpmulti_reset(icpmulti_device* icp_dev);
static int init_ao_channels(icpmulti_device* icp_dev);
static int init_do_channels(icpmulti_device* icp_dev);
static int reset_ao_channels(icpmulti_device* icp_dev);
static int reset_do_channels(icpmulti_device* icp_dev);
static void setup_channel_list(icpmulti_device* icp_dev);

/*
==============================================================================
	Helper functions
==============================================================================
*/

int icpmulti_busy_sleep(int delay) {
  unsigned long j = jiffies + delay/10000000;

  while(jiffies < j)
    schedule();

  return 0;
};

/*
==============================================================================
        Char device registration and release
==============================================================================
*/

static void icpmulti_register_cdevs(struct class* cdev_class, struct cdev*
  reg_cdev, unsigned int cdev_major, unsigned int num_cdevs, unsigned int
  device, const char* cdev_bname, const char* cdev_name) {
  unsigned int cdev_minor = reg_cdev->count;
  int i;

  if (!cdev_add(reg_cdev, MKDEV(cdev_major, cdev_minor),
    reg_cdev->count+num_cdevs)) {
    icpmulti_printk("Registering %d %s char devices at %s[%d-%d]\n",
      num_cdevs, cdev_name, cdev_bname, 0, num_cdevs-1);

    for(i = 0; i < num_cdevs; i++) {
      char cdev_fname[256];
      sprintf(cdev_fname, "%s%d%s%d", ICPMULTI_DEVNAME, device, cdev_bname, i);

      class_device_create(cdev_class, MKDEV(cdev_major, cdev_minor+i),
        NULL, cdev_fname);

      icpmulti_debugk("%s %d at %s (major %d, minor %d)\n", cdev_name, i,
        cdev_fname, cdev_major, cdev_minor+i);
    }
  }
  else icpmulti_alertk("Could not register %s char devices\n", cdev_name);
}

static void icpmulti_unregister_cdevs(struct class* cdev_class, struct cdev*
  unreg_cdev) {
  int i;

  if (unreg_cdev->count) {
    for(i = 0; i < unreg_cdev->count; i++)
    class_device_destroy(cdev_class, unreg_cdev->dev+i);

    cdev_del(unreg_cdev);

    icpmulti_printk("Unregistered all char devices\n");
  }
}

/*
==============================================================================
        Device initialization and removal
==============================================================================
*/

int icpmulti_probe(struct pci_dev *pdev, const struct pci_device_id *ent) {
  int result = -ENODEV;

  if ((pdev->vendor == ICPMULTI_VENDORID) &&
    (pdev->device == ICPMULTI_DEVICEID)) {
    icpmulti_printk("%s detected\n", ICPMULTI_NAME);

    result = pci_enable_device(pdev);

    if (!result) {
      // this is our device
      icpmulti_device *icp_dev = kmalloc(sizeof(icpmulti_device),
        GFP_KERNEL);
      pci_set_drvdata(pdev, icp_dev);

      // save PCI device
      icp_dev->pdev = pdev;

      // query interrupt line
      pci_read_config_byte(icp_dev->pdev, PCI_INTERRUPT_LINE, &icp_dev->irq);
      // enable PCI 16 bits
      pci_write_config_dword(icp_dev->pdev, 0xb0, 0x00050000);
      // enable memory
      pci_write_config_dword(icp_dev->pdev, 0x04, 0x00000002);

      // init mutexes
      sema_init(&icp_dev->ai_sem, 1);
      sema_init(&icp_dev->ao_sem, 1);
      sema_init(&icp_dev->di_sem, 1);
      sema_init(&icp_dev->do_sem, 1);

      // setup channel information
      icp_dev->n_aichan = 16;
      icp_dev->n_aochan = 4;
      icp_dev->n_dichan = 16;
      icp_dev->n_dochan = 8;
      icp_dev->do_state = 0;

      // request first I/O address of region 2
      icp_dev->phys_iobase = 0xb7fd0000;//pci_resource_start(icp_dev->pdev, 2);

      icpmulti_debugk("Device at 0x%x (IRQ = %d) enabled\n",
        (unsigned int)icp_dev->phys_iobase, icp_dev->irq);

      // remap physical I/O to kernel space I/O
      icp_dev->iobase = ioremap(icp_dev->phys_iobase, ICPMULTI_SIZE);
      if (!icp_dev->iobase) {
        icpmulti_alertk("I/O remap failed\n");
        result = -ENOMEM;
      }
      else {
        icpmulti_debugk("Physical I/O remapped to 0x%x\n",
          (unsigned int)icp_dev->iobase);

        // reset and initialize
        icpmulti_reset(icp_dev);
        init_ao_channels(icp_dev);
        init_do_channels(icp_dev);

        // register char devices
        icp_dev->dev_class = icpmulti_class;
        cdev_init(&icp_dev->cdev, &icpmulti_fops);

        icpmulti_register_cdevs(icp_dev->dev_class, &icp_dev->cdev,
          ICPMULTI_MAJOR_BASE+icpmulti_num_devs, icp_dev->n_aichan,
          icpmulti_num_devs, ICPMULTI_DEVNAME_AI, "analog input channel");
        icpmulti_register_cdevs(icp_dev->dev_class, &icp_dev->cdev,
          ICPMULTI_MAJOR_BASE+icpmulti_num_devs, icp_dev->n_aochan,
          icpmulti_num_devs, ICPMULTI_DEVNAME_AO, "analog output channel");
        icpmulti_register_cdevs(icp_dev->dev_class, &icp_dev->cdev,
          ICPMULTI_MAJOR_BASE+icpmulti_num_devs, icp_dev->n_dichan,
          icpmulti_num_devs, ICPMULTI_DEVNAME_DI, "digital input channel");
        icpmulti_register_cdevs(icp_dev->dev_class, &icp_dev->cdev,
          ICPMULTI_MAJOR_BASE+icpmulti_num_devs, icp_dev->n_dochan,
          icpmulti_num_devs, ICPMULTI_DEVNAME_DO, "digital output channel");

        mdelay(1000);
      }

      // Save the first device as default
      if (!icpmulti_default) icpmulti_default = icp_dev;
      ++icpmulti_num_devs;
    }
    else icpmulti_alertk("Failed to enable device\n");
  }

  return result;
}

void icpmulti_remove(struct pci_dev *pdev) {
  icpmulti_device *icp_dev = pci_get_drvdata(pdev);

  if (icp_dev) {
    // reset
    reset_ao_channels(icp_dev);
    reset_do_channels(icp_dev);

    // unregister char devices
    icpmulti_unregister_cdevs(icp_dev->dev_class, &icp_dev->cdev);

    // unmap kernel space I/O
    if (icp_dev->iobase) iounmap(icp_dev->iobase);

    // forget about the default device
    if (icpmulti_default == icp_dev) icpmulti_default = 0;

    // free memory
    kfree(icp_dev);

    icpmulti_printk("%s removed\n", ICPMULTI_NAME);
  }
  else icpmulti_alertk("Failed to remove device\n");

}

static int __init icpmulti_init(void) {
  // create device class
  icpmulti_class = class_create(THIS_MODULE, ICPMULTI_DEVCLASS);

  // find devices and register driver
  icpmulti_num_devs = 0;
  icpmulti_default = 0;
  return pci_module_init(&icpmulti_driver);
}

static void __exit icpmulti_exit(void) {
  // unregister driver
  pci_unregister_driver(&icpmulti_driver);

  // destroy device class
  class_destroy(icpmulti_class);
}

module_init(icpmulti_init);
module_exit(icpmulti_exit);

/*
==============================================================================

        Name:   icpmulti_read_dev_di/icpmulti_read_di

        Description:
                This function reads the digital inputs.

==============================================================================
*/

int icpmulti_read_dev_di(icpmulti_device* icp_dev, int channel, unsigned char
  *value) {
  unsigned short status;

  icpmulti_debugk("EDBG: BGN: icpmulti_read_dev_di(...)\n");

  down(&icp_dev->di_sem);

  status = readw(icp_dev->iobase+ICPMULTI_DI);
  status >>= channel;
  status &= 0x0001;

  if (status) (*value) = 1;
  else (*value) = 0;

  icpmulti_debugk("DI chan=%d value=%d\n", channel, *value);

  up(&icp_dev->di_sem);

  icpmulti_debugk("EDBG: END: icpmulti_read_dev_di(...)\n");

  return 0;
}

int icpmulti_read_di(int channel, unsigned char *value) {
  if (!icpmulti_default) return -ENODEV;
  else return icpmulti_read_dev_di(icpmulti_default, channel, value);
}

/*
==============================================================================

        Name:   icpmulti_read_dev_do/icpmulti_read_do

        Description:
                This function reads a single digital output.

==============================================================================
*/

int icpmulti_read_dev_do(icpmulti_device* icp_dev, int channel, unsigned char
  *value) {
  down(&icp_dev->do_sem);

  *value = icp_dev->do_data[channel];

  up(&icp_dev->do_sem);
  return 0;
}

int icpmulti_read_do(int channel, unsigned char *value) {
  if (!icpmulti_default) return -ENODEV;
  else return icpmulti_read_dev_do(icpmulti_default, channel, value);
}

/*
==============================================================================

        Name:   icpmulti_write_dev_do/icpmulti_write_do

        Description:
                This function writes the appropriate digital outputs.

==============================================================================
*/

int icpmulti_write_dev_do(icpmulti_device* icp_dev, int channel, unsigned char
  value) {
  unsigned short bit = 0;

  icpmulti_debugk("EDBG: BGN: icpmulti_write_dev_do(...)\n");

  down(&icp_dev->do_sem);

  bit = 0x01 << channel;
  icp_dev->do_state &= ~bit;
  if (value) icp_dev->do_state |= bit;

  writew(icp_dev->do_state, icp_dev->iobase+ICPMULTI_DO);

  icpmulti_debugk("DO chan=%d value=%d\n", channel, value);

  // Save digital output data
  icp_dev->do_data[channel] = value;

  up(&icp_dev->do_sem);

  icpmulti_debugk("EDBG: END: icpmulti_write_dev_do(...)\n");

  return 0;
}

int icpmulti_write_do(int channel, unsigned char value) {
  if (!icpmulti_default) return -ENODEV;
  else return icpmulti_write_dev_do(icpmulti_default, channel, value);
}

/*
==============================================================================

	Name:	icpmulti_read_dev_ai/icpmulti_read_ai

	Description:
		This function reads a single analog input.

	Parameters:
		comedi_device *dev	Pointer to current device structure
		comedi_subdevice *s	Pointer to current subdevice structure
		comedi_insn *insn	Pointer to current comedi instruction
		lsampl_t *data		Pointer to analog input data

	Returns:int			Nmuber of instructions executed

==============================================================================
*/

int icpmulti_read_dev_ai(icpmulti_device* icp_dev, int channel, short *value) {
  int n, timeout;
  int range = 0x30; // range -10 10

  icpmulti_debugk("EDBG: BGN: icpmulti_read_dev_ai(...)\n");

  down(&icp_dev->ai_sem);

  // Disable A/D conversion ready interrupt
  icp_dev->IntEnable &= ~ADC_READY;
  writew(icp_dev->IntEnable, icp_dev->iobase+ICPMULTI_INT_EN);

  // Clear interrupt status
  icp_dev->IntStatus |= ADC_READY;
  writew(icp_dev->IntStatus, icp_dev->iobase+ICPMULTI_INT_STAT);

  // Set up appropriate channel, mode and range data, for specified channel
  icp_dev->AdcCmdStatus &= 0xf00f;
  icp_dev->AdcCmdStatus |= range;
  icp_dev->AdcCmdStatus |= (channel << 8);

  /* Output channel, range, mode to ICP Multi */
  writew(icp_dev->AdcCmdStatus, icp_dev->iobase+ICPMULTI_ADC_CSR);

  for (n = 0; n < 1; n++) {
    // Set start ADC bit
    icp_dev->AdcCmdStatus |= ADC_ST;
    writew(icp_dev->AdcCmdStatus, icp_dev->iobase+ICPMULTI_ADC_CSR);
    icp_dev->AdcCmdStatus &= ~ADC_ST;

    // Wait for conversion to complete, or get fed up waiting
    timeout=100;
    while (timeout--) {
      if (!(readw(icp_dev->iobase+ICPMULTI_ADC_CSR) & ADC_BSY))
      goto conv_finish;

      if (debug) if (!(timeout%10))
        icpmulti_debugk("ERROR: AI chan=%d n=%d tm=%d ST=0x%x\n", channel, n,
        timeout, readw(icp_dev->iobase+ICPMULTI_ADC_CSR));
    }

    // If we reach here, a timeout has occurred
    icpmulti_printk("A/D conversion timeout\n");

    // Disable interrupt
    icp_dev->IntEnable &= ~ADC_READY;
    writew(icp_dev->IntEnable, icp_dev->iobase+ICPMULTI_INT_EN);

    // Clear interrupt status
    icp_dev->IntStatus |= ADC_READY;
    writew(icp_dev->IntStatus, icp_dev->iobase+ICPMULTI_INT_STAT);

    // Clear data received
    *value = 0;

    icpmulti_debugk("EDBG: END: icpmulti_read_dev_ai(...) n=%d\n", n);

    up(&icp_dev->ai_sem);
    return -ETIME;

conv_finish:
    *value = (readw(icp_dev->iobase+ICPMULTI_AI) >> 4 ) & 0x0fff;
  }

  // Disable interrupt
  icp_dev->IntEnable &= ~ADC_READY;
  writew(icp_dev->IntEnable, icp_dev->iobase+ICPMULTI_INT_EN);

  // Clear interrupt status
  icp_dev->IntStatus |= ADC_READY;
  writew(icp_dev->IntStatus, icp_dev->iobase+ICPMULTI_INT_STAT);

  icpmulti_debugk("AI chan=%d value=%d\n", channel, *value);
  icpmulti_debugk("EDBG: END: icpmulti_read_dev_ai(...) n=%d\n", n);

  up(&icp_dev->ai_sem);
  return 0;
}

int icpmulti_read_ai(int channel, short *value) {
  if (!icpmulti_default) return -ENODEV;
  else return icpmulti_read_dev_ai(icpmulti_default, channel, value);
}

/*
==============================================================================

        Name:   icpmulti_read_dev_ao/icpmulti_read_ao

        Description:
                This function reads a single analog output.

==============================================================================
*/

int icpmulti_read_dev_ao(icpmulti_device* icp_dev, int channel, short *value) {
  down(&icp_dev->ao_sem);

  *value = icp_dev->ao_data[channel];

  up(&icp_dev->ao_sem);
  return 0;
}

int icpmulti_read_ao(int channel, short *value) {
  if (!icpmulti_default) return -ENODEV;
  else return icpmulti_read_dev_ao(icpmulti_default, channel, value);
}

/*
==============================================================================

	Name:	icpmulti_write_dev_ao/icpmulti_write_ao

	Description:
		This function writes a single analog output.

==============================================================================
*/

int icpmulti_write_dev_ao(icpmulti_device* icp_dev, int channel, short value) {
  int n, timeout;
  int range = 0x30; /* range -10 10 */

  icpmulti_debugk("EDBG: BGN: icpmulti_write_dev_ao(...)\n");

  down(&icp_dev->ao_sem);

  // Disable D/A conversion ready interrupt
  icp_dev->IntEnable &= ~DAC_READY;
  writew(icp_dev->IntEnable, icp_dev->iobase+ICPMULTI_INT_EN);

  // Clear interrupt status
  icp_dev->IntStatus |= DAC_READY;
  writew(icp_dev->IntStatus, icp_dev->iobase+ICPMULTI_INT_STAT);

  // Set up range and channel data
  // Bit 4 = 1 : Bipolar
  // Bit 5 = 0 : 5V
  // Bit 5 = 1 : 10V
  // Bits 8-9 : Channel number
  icp_dev->DacCmdStatus &= 0xfccf;
  icp_dev->DacCmdStatus |= range;
  icp_dev->DacCmdStatus |= (channel << 8);

  writew(icp_dev->DacCmdStatus, icp_dev->iobase+ICPMULTI_DAC_CSR);

  for (n = 0; n < 1; n++) {
    /* Wait for analog output data register to be ready for new data,
       or get fed up waiting */
    timeout=100;
    while (timeout--) {
      if (!(readw(icp_dev->iobase+ICPMULTI_DAC_CSR) & DAC_BSY))
        goto dac_ready;

      if (debug) if (!(timeout%10))
        icpmulti_debugk("ERROR: AO chan=%d n=%d tm=%d ST=0x%x\n", channel, n,
        timeout, readw(icp_dev->iobase+ICPMULTI_DAC_CSR));

      icpmulti_busy_sleep(1000);
    }

    // If we reach here, a timeout has occurred
    icpmulti_printk("D/A conversion timed out\n");

    // Disable interrupt
    icp_dev->IntEnable &= ~DAC_READY;
    writew(icp_dev->IntEnable, icp_dev->iobase+ICPMULTI_INT_EN);

    // Clear interrupt status
    icp_dev->IntStatus |= DAC_READY;
    writew(icp_dev->IntStatus, icp_dev->iobase+ICPMULTI_INT_STAT);

    // Clear data received
    icp_dev->ao_data[channel] = 0;

    icpmulti_debugk("EDBG: END: icpmulti_write_dev_ao(...) n=%d\n", n);

    up(&icp_dev->ao_sem);
    return -ETIME;

dac_ready:
    // Write data to analog output data register
    writew(value*0x10, icp_dev->iobase+ICPMULTI_AO);

    icpmulti_debugk("AO chan=%d, range=%d: 0x%x at 0x%x=%d\n", channel, range,
      value*0x10, (unsigned int)icp_dev->iobase, ICPMULTI_AO);

    // Set DAC_ST bit to write the data to selected channel
    icp_dev->DacCmdStatus |= DAC_ST;
    writew(icp_dev->DacCmdStatus, icp_dev->iobase+ICPMULTI_DAC_CSR);
    icp_dev->DacCmdStatus &= ~DAC_ST;

    // Save analog output data
    icp_dev->ao_data[channel] = value;
  }

  icpmulti_debugk("EDBG: END: icpmulti_write_dev_ao(...) n=%d\n", n);

  up(&icp_dev->ao_sem);
  return 0;
}

int icpmulti_write_ao(int channel, short value) {
  if (!icpmulti_default) return -ENODEV;
  else return icpmulti_write_dev_ao(icpmulti_default, channel, value);
}

/*
==============================================================================

	Name:	setup_channel_list

	Description:
		This function sets the appropriate channel selection,
		differential input mode and range bits in the ADC Command/
		Status register.

		HACK for now: do [-10 10] and stop
==============================================================================
*/

static void setup_channel_list(icpmulti_device* icp_dev) {
  unsigned int i, range, chanprog;
  unsigned int diff;

  icpmulti_debugk("EDBG: setup_channel_list()\n");

  /* FIXME: clean this up */
  for (i = 0; i < icp_dev->n_aichan; i++) {
    diff = 0;
    chanprog &= 0x000f;

    // Clear channel, range and input mode bits in A/D command/status register
    icp_dev->AdcCmdStatus &= 0xf00f;

    // Set channel number and differential mode status bit
    if (diff) {
      // Set channel number, bits 9-11 & mode, bit 6
      icp_dev->AdcCmdStatus |= (chanprog << 9);
      icp_dev->AdcCmdStatus |= ADC_DI;
    }
    else
      // Set channel number, bits 8-11
      icp_dev->AdcCmdStatus |= (chanprog << 8);

    // Get range for current channel
    range = 0x30;
    // Set range. bits 4-5
    icp_dev->AdcCmdStatus |= range;

    /* Output channel, range, mode to ICP Multi */
    writew(icp_dev->AdcCmdStatus, icp_dev->iobase+ICPMULTI_ADC_CSR);

    icpmulti_debugk("GS: %2d. [0x%x]=0x%x 0x%x\n", i, chanprog, range,
      icp_dev->act_chanlist[i]);
  }
}

/*
==============================================================================

	Name:	icpmulti_reset

	Description:
		This function resets the icp multi device to a 'safe' state

	Parameters:
		comedi_device *dev	Pointer to current sevice structure

	Returns:int	0 = success

==============================================================================
*/

int icpmulti_reset(icpmulti_device* icp_dev) {
  unsigned int i;

  icpmulti_debugk("EDBG: BGN: icpmulti_reset(...)\n");

  // Clear INT enables and requests
  writew(0, icp_dev->iobase+ICPMULTI_INT_EN);
  writew(0x00ff, icp_dev->iobase+ICPMULTI_INT_STAT);

  // Set DACs to 0 5 range and 0V output
  for (i = 0; i < icp_dev->n_aochan; i++) {
    icp_dev->DacCmdStatus &= 0xfcce;

    // Set channel number
    icp_dev->DacCmdStatus |= (i << 8);

    // Output 0V
    //writew(2048, icp_dev->iobase+ICPMULTI_AO);
    writew(0, icp_dev->iobase+ICPMULTI_AO);

    // Set start conversion bit
    icp_dev->DacCmdStatus |= DAC_ST;

    // Output to command / status register
    writew(icp_dev->DacCmdStatus, icp_dev->iobase+ICPMULTI_DAC_CSR);

    // Delay to allow DAC time to recover
    mdelay(1);
  }

  // Digital outputs to 0
  writew(0, icp_dev->iobase+ICPMULTI_DO);

  icpmulti_debugk("EDBG: END: icpmulti_reset(...)\n");

  return 0;
}

static int init_ao_channels(icpmulti_device* icp_dev) {
  int chan;
  int init_value = 2048;
  int range = 0x30;

  icpmulti_debugk("EDBG: BGN: init_ao_channels(...)\n");

  // Disable D/A conversion ready interrupt
  icp_dev->IntEnable &= ~DAC_READY;
  writew(icp_dev->IntEnable, icp_dev->iobase+ICPMULTI_INT_EN);

  // Clear interrupt status
  icp_dev->IntStatus |= DAC_READY;
  writew(icp_dev->IntStatus, icp_dev->iobase+ICPMULTI_INT_STAT);

  for (chan = 0; chan < icp_dev->n_aochan; chan++) {
    icp_dev->DacCmdStatus &= 0xfccf;
    icp_dev->DacCmdStatus |= range;
    icp_dev->DacCmdStatus |= (chan << 8);

    writew(icp_dev->DacCmdStatus, icp_dev->iobase+ICPMULTI_DAC_CSR);

    // Write data to analog output data register
    writew(init_value*0x10, icp_dev->iobase+ICPMULTI_AO);

    // Set DAC_ST bit to write the data to selected channel
    icp_dev->DacCmdStatus |= DAC_ST;

    writew(icp_dev->DacCmdStatus, icp_dev->iobase+ICPMULTI_DAC_CSR);
    icp_dev->DacCmdStatus &= ~DAC_ST;

    // Save analog output data
    icp_dev->ao_data[chan] = init_value;

    mdelay(1);
  }

  icpmulti_debugk("EDBG: END: init_ao_channels(...)\n");

  return 0;
}

static int init_do_channels(icpmulti_device* icp_dev) {
  int chan;
  int init_value = 0;
  unsigned short bit = 0;

  icpmulti_debugk("EDBG: BGN: init_do_channels(...)\n");

  for (chan = 0; chan < icp_dev->n_dochan; chan++) {
    bit = 0x01 << chan;
    icp_dev->do_state &= ~bit;
    if (init_value) icp_dev->do_state |= bit;

    writew(icp_dev->do_state, icp_dev->iobase+ICPMULTI_DO);

    // Save digital output data
    icp_dev->do_data[chan] = init_value;

    mdelay(1);
  }

  icpmulti_debugk("EDBG: END: init_do_channels(...)\n");

  return 0;
}

static int reset_ao_channels(icpmulti_device* icp_dev) {
  int chan;
  int reset_value = 2048;
  int range = 0x30;

  icpmulti_debugk("EDBG: BGN: reset_ao_channels(...)\n");

  for (chan = 0; chan < icp_dev->n_aochan; chan++) {
    icp_dev->DacCmdStatus &= 0xfccf;
    icp_dev->DacCmdStatus |= range;
    icp_dev->DacCmdStatus |= (chan << 8);

    writew(icp_dev->DacCmdStatus, icp_dev->iobase+ICPMULTI_DAC_CSR);

    // Write data to analog output data register
    writew(reset_value*0x10, icp_dev->iobase+ICPMULTI_AO);

    // Set DAC_ST bit to write the data to selected channel
    icp_dev->DacCmdStatus |= DAC_ST;

    writew(icp_dev->DacCmdStatus, icp_dev->iobase+ICPMULTI_DAC_CSR);
    icp_dev->DacCmdStatus &= ~DAC_ST;

    // Save analog output data
    icp_dev->ao_data[chan] = reset_value;

    mdelay(1);
  }

  icpmulti_debugk("EDBG: END: reset_ao_channels(...)\n");

  return 0;
}

static int reset_do_channels(icpmulti_device* icp_dev) {
  int chan;
  int reset_value = 0;
  unsigned short bit = 0;

  icpmulti_debugk("EDBG: BGN: reset_do_channels(...)\n");

  for (chan = 0; chan < icp_dev->n_dochan; chan++) {
    bit = 0x01 << chan;
    icp_dev->do_state &= ~bit;
    if (reset_value) icp_dev->do_state |= bit;

    writew(icp_dev->do_state, icp_dev->iobase+ICPMULTI_DO);

    // Save digital output data
    icp_dev->do_data[chan] = reset_value;

    mdelay(1);
  }

  icpmulti_debugk("EDBG: END: reset_do_channels(...)\n");

  return 0;
}

/*
==============================================================================
        Read/write char devices
==============================================================================
*/

int icpmulti_device_open(struct inode *inode, struct file *file) {
  int result = 0;
  icpmulti_device *icp_dev;
  unsigned int chan = iminor(inode);

  icpmulti_debugk("EDBG: BGN: icpmulti_device_open(...)\n");

  // find the device
  icp_dev = container_of(inode->i_cdev, icpmulti_device, cdev);

  if (file->f_mode & FMODE_WRITE) {
    if (chan < icp_dev->n_aichan) result = -EINVAL;
    else {
      chan -= icp_dev->n_aichan;

      if (chan >= icp_dev->n_aochan) {
        chan -= icp_dev->n_aochan;
        if (chan < icp_dev->n_dichan) result = -EINVAL;
      }
    }
  }

  // associate device with file
  file->private_data = icp_dev;

  icpmulti_debugk("EDBG: END: icpmulti_device_open(...)\n");

  return result;
}

int icpmulti_device_release(struct inode *inode, struct file *file) {
  icpmulti_debugk("EDBG: BGN: icpmulti_device_release(...)\n");

  // nothing to do here

  icpmulti_debugk("EDBG: END: icpmulti_device_release(...)\n");

  return 0;
}

int icpmulti_device_read(struct file *file, char *buff, size_t len,
  loff_t *f_pos) {
  int result = -EFAULT;
  icpmulti_device *icp_dev =  file->private_data;
  unsigned int chan = iminor(file->f_dentry->d_inode);
  short short_val;
  unsigned char char_val;
  char out[256];
  unsigned int out_len;

  icpmulti_debugk("EDBG: BGN: icpmulti_device_read(...)\n");

  if (chan < icp_dev->n_aichan) {
    result = icpmulti_read_dev_ai(icp_dev, chan, &short_val);
    if (!result) sprintf(out, ICPMULTI_FORMAT, short_val);

    goto device_read;
  }
  else chan -= icp_dev->n_aichan;

  if (chan < icp_dev->n_aochan) {
    result = icpmulti_read_dev_ao(icp_dev, chan, &short_val);
    if (!result) sprintf(out, ICPMULTI_FORMAT, short_val);

    goto device_read;
  }
  else chan -= icp_dev->n_aochan;

  if (chan < icp_dev->n_dichan) {
    result = icpmulti_read_dev_di(icp_dev, chan, &char_val);
    if (!result) sprintf(out, ICPMULTI_FORMAT, char_val);

    goto device_read;
  }
  else chan -= icp_dev->n_dichan;

  if (chan < icp_dev->n_dochan) {
    result = icpmulti_read_dev_do(icp_dev, chan, &char_val);
    if (!result) sprintf(out, ICPMULTI_FORMAT, char_val);

    goto device_read;
  }

device_read:
  if (!result) {
    mdelay(500);
    strcat(out, "\n");
    out_len = strlen(out);

    if (len >= out_len)
      if (!copy_to_user(buff, out, out_len)) result = out_len;
  }

  icpmulti_debugk("EDBG: END: icpmulti_device_read(...)\n");

  return result;
}

int icpmulti_device_write(struct file *file, const char *buff, size_t len,
  loff_t *off) {
  int result = -EFAULT;
  icpmulti_device *icp_dev =  file->private_data;
  unsigned int chan = iminor(file->f_dentry->d_inode);
  char in[len+1];
  int in_val;

  icpmulti_debugk("EDBG: BGN: icpmulti_device_write(...)\n");


  if (!copy_from_user(in, buff, len)) {
    in[len] = 0;

    chan -= icp_dev->n_aichan;
    if (chan < icp_dev->n_aochan) {
      if (sscanf(in, ICPMULTI_FORMAT, &in_val) > 0)
        result = icpmulti_write_dev_ao(icp_dev, chan, (short)in_val);

      goto device_written;
    }
    else chan -= icp_dev->n_aochan+icp_dev->n_dichan;

    if (chan < icp_dev->n_dochan) {
      if (sscanf(in, ICPMULTI_FORMAT, &in_val) > 0)
        result = icpmulti_write_dev_do(icp_dev, chan, (unsigned char)in_val);

      goto device_written;
    }
  }

device_written:
  if (!result) result = len;

  icpmulti_debugk("EDBG: END: icpmulti_device_write(...)\n");

  return result;
}
