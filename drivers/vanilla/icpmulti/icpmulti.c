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
 * very much inspired by the comedi icp_multi driver, but
 * taken out of comedi.
 *
 * char device implementation and none-realtime mutual exclusion
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

static int debug;

// IDs and names
#define ICPMULTI_VENDORID       0x104c  // Vendor ID
#define ICPMULTI_DEVICEID	0x8000	// Device ID
#define ICPMULTI_FULLNAME       "ICP-multi composite A&D I/O board"
#define ICPMULTI_NAME           "icpmulti"

// Hardware types of the cards
#define TYPE_ICP_MULTI	0

#define IORANGE_ICP_MULTI 	32

#define ICP_MULTI_ADC_CSR	0	// R/W:	ADC command/status register
#define ICP_MULTI_AI		2	// R:	Analogue input data
#define ICP_MULTI_DAC_CSR	4	// R/W:	DAC command/status register
#define ICP_MULTI_AO		6	// R/W:	Analogue output data
#define ICP_MULTI_DI		8	// R/W:	Digital inouts
#define ICP_MULTI_DO		0x0A	// R/W:	Digital outputs
#define ICP_MULTI_INT_EN	0x0C	// R/W:	Interrupt enable register
#define ICP_MULTI_INT_STAT	0x0E	// R/W:	Interrupt status register
#define ICP_MULTI_CNTR0		0x10	// R/W:	Counter 0
#define ICP_MULTI_CNTR1		0x12	// R/W:	counter 1
#define ICP_MULTI_CNTR2		0x14	// R/W:	Counter 2
#define ICP_MULTI_CNTR3		0x16	// R/W:	Counter 3

#define ICP_MULTI_SIZE		0x20	// 32 bytes

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

// Character device operations
#define ICPMULTI_AIMAJOR 210
#define ICPMULTI_AOMAJOR 211
#define ICPMULTI_DIMAJOR 212
#define ICPMULTI_DOMAJOR 213
#define ICPMULTI_DEVCLASS "icp"

int icpmulti_device_open(struct inode *inode, struct file *file);
int icpmulti_device_release(struct inode *inode, struct file *file);
int icpmulti_device_read(struct file *filp, char *buff, size_t len,
  loff_t *f_pos);
int icpmulti_device_write(struct file *filp, const char *buff, size_t len,
  loff_t *off);

static struct file_operations icpmulti_ai_fops = {
  .open = icpmulti_device_open,
  .release = icpmulti_device_release,
  .read = icpmulti_device_read,
};

static struct file_operations icpmulti_ao_fops = {
  .open = icpmulti_device_open,
  .release = icpmulti_device_release,
  .read = icpmulti_device_read,
  .write = icpmulti_device_write,
};

static struct file_operations icpmulti_di_fops = {
  .open = icpmulti_device_open,
  .release = icpmulti_device_release,
  .read = icpmulti_device_read,
};

static struct file_operations icpmulti_do_fops = {
  .open = icpmulti_device_open,
  .release = icpmulti_device_release,
  .read = icpmulti_device_read,
  .write = icpmulti_device_write,
};

// Kernel messages
#define icpmulti_printk(fmt, arg...) {\
  char message[256]; \
  sprintf(message, fmt, ## arg); \
  printk("%s: %s", ICPMULTI_NAME, message); \
}
#define icpmulti_alertk(fmt, arg...) \
  icpmulti_printk(KERN_ALERT fmt, ## arg)
#define icpmulti_debugk(fmt, arg...) \
  if (debug) icpmulti_printk(KERN_DEBUG fmt, ## arg)

/*
==============================================================================
	Data & Structure declarations
==============================================================================
*/

typedef struct {
  unsigned long phys_iobase;		// Physical io address
  unsigned char *iobase;
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

  /* mutexes */
  struct semaphore ai_sem;              // analog in semaphore
  struct semaphore ao_sem;              // analog out semaphore
  struct semaphore di_sem;              // digital in semaphore
  struct semaphore do_sem;              // digital out semaphore

  /* char devices */
  struct class* dev_class;              // device class
  struct cdev ai_cdev;                  // analog in char device
  struct cdev ao_cdev;                  // analog out char device
  struct cdev di_cdev;                  // digital in char device
  struct cdev do_cdev;                  // digital out char device
} icpmulti_private;

static icpmulti_private private;

MODULE_AUTHOR("Frederic Pont and Ralf Kaestner");
MODULE_DESCRIPTION("Linux driver for ICP-multi composite A&D I/O board");
MODULE_LICENSE("GPL");
module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug enabled or not");
static struct pci_device_id icpmulti_pci_table[] __devinitdata = {
  { ICPMULTI_VENDORID, ICPMULTI_DEVICEID, PCI_ANY_ID, PCI_ANY_ID, },
  { 0, } // Terminating entry
};
MODULE_DEVICE_TABLE(pci, icpmulti_pci_table);

/*
==============================================================================
	More forward declarations
==============================================================================
*/

int icpmulti_reset(void);
static int init_ao_channels(void);
static int init_do_channels(void);
static void setup_channel_list(void);

/*
==============================================================================
	Functions
==============================================================================
*/

int icpmulti_busy_sleep(int delay) {
  unsigned long j = jiffies + delay/10000000;

  while(jiffies < j)
    schedule();

  return 0;
};

static void icpmulti_register_cdevs(struct class* cdev_class, struct cdev*
  reg_cdev, struct file_operations* cdev_fops, unsigned int cdev_major,
  unsigned int cdev_minor, unsigned int num_cdevs, const char* cdev_bname,
  const char* cdev_name) {
  int i;

  cdev_init(reg_cdev, cdev_fops);
  reg_cdev->owner = THIS_MODULE;
  if (!cdev_add(reg_cdev, MKDEV(cdev_major, cdev_minor), num_cdevs)) {
    icpmulti_printk("Registering %d %s char devices\n", num_cdevs, cdev_name);

    for(i = 0; i < num_cdevs; i++) {
      char cdev_fname[256];
      sprintf(cdev_fname, "%s%d", cdev_bname, i);

      class_device_create(cdev_class, MKDEV(cdev_major, cdev_minor)+i, NULL,
        cdev_fname);

      icpmulti_printk("%s %d at %s\n", cdev_name, i, cdev_fname);
    }
  }
  else icpmulti_printk("Could not register %s char devices\n", cdev_name);
}

static void icpmulti_unregister_cdevs(struct class* cdev_class, struct cdev*
  unreg_cdev, const char* cdev_name) {
  int i;

  if (unreg_cdev->count) {
    for(i = 0; i < unreg_cdev->count; i++)
    class_device_destroy(cdev_class, unreg_cdev->dev+i);

    cdev_del(unreg_cdev);

    icpmulti_printk("Unregistered %s char devices\n", cdev_name);
  }
}

static int icpmulti_init(void) {
  struct pci_dev *dev;

  dev = pci_find_device(ICPMULTI_VENDORID, ICPMULTI_DEVICEID, NULL);

  if (!dev) {
    icpmulti_alertk("%s not found\n", ICPMULTI_FULLNAME);
    return -ENODEV;
  }

  icpmulti_printk("%s detected\n", ICPMULTI_FULLNAME);

  if (pci_enable_device(dev)) return -ENODEV;

  /* configure PCI */
  pci_write_config_dword(dev, 0xb0, 0x00050000); // enable pci 16 bits
  pci_write_config_dword(dev, 0x04, 0x00000002); // enable memory

  private.n_aichan = 16;
  private.n_aochan = 4;
  private.n_dichan = 16;
  private.n_dochan = 8;
  private.do_state = 0;

  /* init mutexes */
  sema_init(&private.ai_sem, 1);
  sema_init(&private.ao_sem, 1);
  sema_init(&private.di_sem, 1);
  sema_init(&private.do_sem, 1);

  private.phys_iobase = pci_resource_start(dev, 2);

  private.iobase = ioremap(private.phys_iobase, ICP_MULTI_SIZE);

  if (!private.iobase) {
    icpmulti_printk("I/O remap failed\n");
    return -ENOMEM;
  }

  icpmulti_reset();
  init_ao_channels();
  init_do_channels();

  icpmulti_debugk("do_state = %x\n", private.do_state);

  // create device class
  private.dev_class = class_create(THIS_MODULE, ICPMULTI_DEVCLASS);

  // register char devices
  icpmulti_register_cdevs(private.dev_class, &private.ai_cdev,
    &icpmulti_ai_fops, ICPMULTI_AIMAJOR, 0, private.n_aichan, "ai",
    "analog input channel");
  icpmulti_register_cdevs(private.dev_class, &private.ao_cdev,
    &icpmulti_ao_fops, ICPMULTI_AOMAJOR, 0, private.n_aochan, "ao",
    "analog output channel");
  icpmulti_register_cdevs(private.dev_class, &private.di_cdev,
    &icpmulti_di_fops, ICPMULTI_DIMAJOR, 0, private.n_dichan, "di",
    "digital input channel");
  icpmulti_register_cdevs(private.dev_class, &private.do_cdev,
    &icpmulti_do_fops, ICPMULTI_DOMAJOR, 0, private.n_dochan, "do",
    "digital output channel");

  mdelay(1000);

  return 0;
}

static void icpmulti_exit(void) {
  // unregister char devices
  icpmulti_unregister_cdevs(private.dev_class, &private.ai_cdev,
    "analog input channel");
  icpmulti_unregister_cdevs(private.dev_class, &private.ao_cdev,
    "analog output channel");
  icpmulti_unregister_cdevs(private.dev_class, &private.di_cdev,
    "digital input channel");
  icpmulti_unregister_cdevs(private.dev_class, &private.do_cdev,
    "digital output channel");

  // destroy device class
  class_destroy(private.dev_class);

  if (private.iobase) iounmap(private.iobase);
}

module_init(icpmulti_init);
module_exit(icpmulti_exit);

/*
==============================================================================

        Name:   icp_multi_read_di

        Description:
                This function reads the digital inputs.

==============================================================================
*/

int icpmulti_read_di(int channel, unsigned char *value) {
  unsigned short status;

  down(&private.di_sem);

  status = readw(private.iobase+ICP_MULTI_DI);
  status >>= channel;
  status &= 0x0001;

  if (status) (*value) = 1;
  else (*value) = 0;

  up(&private.di_sem);
  return 1;
}

/*
==============================================================================

        Name:   icpmulti_read_do

        Description:
                This function reads a single digital output.

==============================================================================
*/

int icpmulti_read_do(int channel, unsigned char *value) {
  down(&private.do_sem);

  *value = private.do_data[channel];

  up(&private.do_sem);
  return 1;
}

/*
==============================================================================

        Name:   icpmulti_write_do

        Description:
                This function writes the appropriate digital outputs.

==============================================================================
*/

int icpmulti_write_do(int channel, unsigned char value) {
  unsigned short bit = 0;

  down(&private.do_sem);

  bit = 0x01 << channel;
  private.do_state &= ~bit;
  if (value) private.do_state |= bit;

  writew(private.do_state, private.iobase+ICP_MULTI_DO);

  // Save digital output data
  private.do_data[channel] = value;

  up(&private.do_sem);
  return 1;
}

/*
==============================================================================

	Name:	icp_multi_read_ai

	Description:
		This function reads a single analogue input.

	Parameters:
		comedi_device *dev	Pointer to current device structure
		comedi_subdevice *s	Pointer to current subdevice structure
		comedi_insn *insn	Pointer to current comedi instruction
		lsampl_t *data		Pointer to analogue input data

	Returns:int			Nmuber of instructions executed

==============================================================================
*/

int icpmulti_read_ai(int channel, short *value) {
  int n, timeout;
  int range = 0x30; // range -10 10

  icpmulti_debugk("EDBG: BGN: icp_multi_insn_read_ai(...)\n");

  down(&private.ai_sem);

  // Disable A/D conversion ready interrupt
  private.IntEnable &= ~ADC_READY;
  writew(private.IntEnable, private.iobase+ICP_MULTI_INT_EN);

  // Clear interrupt status
  private.IntStatus |= ADC_READY;
  writew(private.IntStatus, private.iobase+ICP_MULTI_INT_STAT);

  // Set up appropriate channel, mode and range data, for specified channel
  private.AdcCmdStatus &= 0xf00f;
  private.AdcCmdStatus |= range;
  private.AdcCmdStatus |= (channel << 8);

  /* Output channel, range, mode to ICP Multi */
  writew(private.AdcCmdStatus, private.iobase+ICP_MULTI_ADC_CSR);

  icpmulti_debugk("A ST=%4x IO=%x\n", readw(private.iobase+ICP_MULTI_ADC_CSR),
    private.iobase+ICP_MULTI_ADC_CSR);

  for (n = 0; n < 1; n++) {
    // Set start ADC bit
    private.AdcCmdStatus |= ADC_ST;
    writew(private.AdcCmdStatus, private.iobase+ICP_MULTI_ADC_CSR);
    private.AdcCmdStatus &= ~ADC_ST;

    icpmulti_debugk("B n=%d ST=%4x\n", n,
      readw(private.iobase+ICP_MULTI_ADC_CSR));
    icpmulti_debugk("C n=%d ST=%4x\n", n,
      readw(private.iobase+ICP_MULTI_ADC_CSR));

    // Wait for conversion to complete, or get fed up waiting
    timeout=100;
    while (timeout--) {
      if (!(readw(private.iobase+ICP_MULTI_ADC_CSR) & ADC_BSY))
      goto conv_finish;

      if (debug) if (!(timeout%10))
        icpmulti_debugk("D n=%d tm=%d ST=%4x\n", n, timeout,
        readw(private.iobase+ICP_MULTI_ADC_CSR));
    }

    // If we reach here, a timeout has occurred
    icpmulti_printk("A/D insn timeout");

    // Disable interrupt
    private.IntEnable &= ~ADC_READY;
    writew(private.IntEnable, private.iobase+ICP_MULTI_INT_EN);

    // Clear interrupt status
    private.IntStatus |= ADC_READY;
    writew(private.IntStatus, private.iobase+ICP_MULTI_INT_STAT);

    // Clear data received
    *value = 0;

    icpmulti_debugk("EDBG: END: icp_multi_insn_read_ai(...) n=%d\n", n);

    up(&private.ai_sem);
    return -ETIME;

conv_finish:
    *value = (readw(private.iobase+ICP_MULTI_AI) >> 4 ) & 0x0fff;
  }

  // Disable interrupt
  private.IntEnable &= ~ADC_READY;
  writew(private.IntEnable, private.iobase+ICP_MULTI_INT_EN);

  // Clear interrupt status
  private.IntStatus |= ADC_READY;
  writew(private.IntStatus, private.iobase+ICP_MULTI_INT_STAT);

  icpmulti_debugk("Read AI chan=%d value=%d\n", channel, *value);
  icpmulti_debugk("EDBG: END: icp_multi_insn_read_ai(...) n=%d\n", n);

  up(&private.ai_sem);
  return 1;
}

/*
==============================================================================

        Name:   icpmulti_read_ao

        Description:
                This function reads a single analogue output.

==============================================================================
*/

int icpmulti_read_ao(int channel, short *value) {
  down(&private.ao_sem);

  *value = private.ao_data[channel];

  up(&private.ao_sem);
  return 1;
}

/*
==============================================================================

	Name:	icpmulti_write_ao

	Description:
		This function writes a single analogue output.

==============================================================================
*/

int icpmulti_write_ao(int channel, short value) {
  int n, timeout;
  int range = 0x30; /* range -10 10 */

  icpmulti_debugk("EDBG: BGN: icp_multi_insn_write_ao(...)\n");

  down(&private.ao_sem);

  // Disable D/A conversion ready interrupt
  private.IntEnable &= ~DAC_READY;
  writew(private.IntEnable, private.iobase+ICP_MULTI_INT_EN);

  // Clear interrupt status
  private.IntStatus |= DAC_READY;
  writew(private.IntStatus, private.iobase+ICP_MULTI_INT_STAT);

  // Set up range and channel data
  // Bit 4 = 1 : Bipolar
  // Bit 5 = 0 : 5V
  // Bit 5 = 1 : 10V
  // Bits 8-9 : Channel number
  private.DacCmdStatus &= 0xfccf;
  private.DacCmdStatus |= range;
  private.DacCmdStatus |= (channel << 8);

  writew(private.DacCmdStatus, private.iobase+ICP_MULTI_DAC_CSR);

  for (n = 0; n < 1; n++) {
    /* Wait for analogue output data register to be ready for new data,
       or get fed up waiting */
    timeout=100;
    while (timeout--) {
      if (!(readw(private.iobase+ICP_MULTI_DAC_CSR) & DAC_BSY))
        goto dac_ready;

      if (debug) if (!(timeout%10))
        icpmulti_debugk("ERROR: AO chan=%d n=%d tm=%d ST=%4x\n", channel, n,
        timeout, readw(private.iobase+ICP_MULTI_DAC_CSR));

      icpmulti_busy_sleep(1000);
    }

    // If we reach here, a timeout has occurred
    icpmulti_printk("D/A insn timeout");

    // Disable interrupt
    private.IntEnable &= ~DAC_READY;
    writew(private.IntEnable, private.iobase+ICP_MULTI_INT_EN);

    // Clear interrupt status
    private.IntStatus |= DAC_READY;
    writew(private.IntStatus, private.iobase+ICP_MULTI_INT_STAT);

    // Clear data received
    private.ao_data[channel] = 0;

    icpmulti_debugk("EDBG: END: icp_multi_insn_write_ao(...) n=%d\n", n);

    up(&private.ao_sem);
    return -ETIME;

dac_ready:
    // Write data to analogue output data register
    writew(value*0x10, private.iobase+ICP_MULTI_AO);

    icpmulti_debugk("AO chan=%d, range=%d: %x at %x=%d\n", channel, range,
      value*0x10, private.iobase, ICP_MULTI_AO);

    // Set DAC_ST bit to write the data to selected channel
    private.DacCmdStatus |= DAC_ST;
    writew(private.DacCmdStatus, private.iobase+ICP_MULTI_DAC_CSR);
    private.DacCmdStatus &= ~DAC_ST;

    // Save analogue output data
    private.ao_data[channel] = value;
  }

  icpmulti_debugk("EDBG: END: icp_multi_insn_write_ao(...) n=%d\n", n);

  up(&private.ao_sem);
  return 1;
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

static void setup_channel_list(void) {
  unsigned int i, range, chanprog;
  unsigned int diff;

  icpmulti_debugk("EDBG: setup_channel_list()\n");

  /* FIXME: clean this up */
  for (i = 0; i < private.n_aichan; i++) {
    diff = 0;
    chanprog &= 0x000f;

    // Clear channel, range and input mode bits in A/D command/status register
    private.AdcCmdStatus &= 0xf00f;

    // Set channel number and differential mode status bit
    if (diff) {
      // Set channel number, bits 9-11 & mode, bit 6
      private.AdcCmdStatus |= (chanprog << 9);
      private.AdcCmdStatus |= ADC_DI;
    }
    else
      // Set channel number, bits 8-11
      private.AdcCmdStatus |= (chanprog << 8);

    // Get range for current channel
    range = 0x30;
    // Set range. bits 4-5
    private.AdcCmdStatus |= range;

    /* Output channel, range, mode to ICP Multi */
    writew(private.AdcCmdStatus, private.iobase+ICP_MULTI_ADC_CSR);

    icpmulti_debugk("GS: %2d. [%4x]=%4x %4x\n", i, chanprog, range,
      private.act_chanlist[i]);
  }
}

/*
==============================================================================

	Name:	icp_multi_reset

	Description:
		This function resets the icp multi device to a 'safe' state

	Parameters:
		comedi_device *dev	Pointer to current sevice structure

	Returns:int	0 = success

==============================================================================
*/

int icpmulti_reset(void) {
  unsigned int i;

  icpmulti_debugk("EDBG: BGN: icp_multi_reset(...)\n");

  // Clear INT enables and requests
  writew(0, private.iobase+ICP_MULTI_INT_EN);
  writew(0x00ff, private.iobase+ICP_MULTI_INT_STAT);

  // Set DACs to 0 5 range and 0V output
  for (i = 0; i < private.n_aochan; i++) {
    private.DacCmdStatus &= 0xfcce;

    // Set channel number
    private.DacCmdStatus |= (i << 8);

    // Output 0V
    //writew(2048, private.iobase+ICP_MULTI_AO);
    writew(0, private.iobase+ICP_MULTI_AO);

    // Set start conversion bit
    private.DacCmdStatus |= DAC_ST;

    // Output to command / status register
    writew(private.DacCmdStatus, private.iobase+ICP_MULTI_DAC_CSR);

    // Delay to allow DAC time to recover
    mdelay(1);
  }

  // Digital outputs to 0
  writew(0, private.iobase+ICP_MULTI_DO);

  icpmulti_debugk("EDBG: END: icp_multi_reset(...)\n");

  return 0;
}

static int init_ao_channels(void) {
  int chan;
  int init_value = 2048;
  int range = 0x30;

  icpmulti_debugk("EDBG: BGN: init_ao_channels(...)\n");

  // Disable D/A conversion ready interrupt
  private.IntEnable &= ~DAC_READY;
  writew(private.IntEnable, private.iobase+ICP_MULTI_INT_EN);

  // Clear interrupt status
  private.IntStatus |= DAC_READY;
  writew(private.IntStatus, private.iobase+ICP_MULTI_INT_STAT);

  for (chan = 0; chan < private.n_aochan; chan++) {
    private.DacCmdStatus &= 0xfccf;
    private.DacCmdStatus |= range;
    private.DacCmdStatus |= (chan << 8);

    writew(private.DacCmdStatus, private.iobase+ICP_MULTI_DAC_CSR);

    // Write data to analogue output data register
    writew(init_value*0x10, private.iobase+ICP_MULTI_AO);

    // Set DAC_ST bit to write the data to selected channel
    private.DacCmdStatus |= DAC_ST;

    writew(private.DacCmdStatus, private.iobase+ICP_MULTI_DAC_CSR);
    private.DacCmdStatus &= ~DAC_ST;

    // Save analogue output data
    private.ao_data[chan] = init_value;

    mdelay(1);
  }

  icpmulti_debugk("EDBG: END: init_ao_channels(...)\n");

  return 0;
}

static int init_do_channels(void) {
  int chan;
  int init_value = 0;
  unsigned short bit = 0;

  icpmulti_debugk("EDBG: BGN: init_do_channels(...)\n");

  for (chan = 0; chan < private.n_dochan; chan++) {
    bit = 0x01 << chan;
    private.do_state &= ~bit;
    if (init_value) private.do_state |= bit;

    writew(private.do_state, private.iobase+ICP_MULTI_DO);

    // Save digital output data
    private.do_data[chan] = init_value;

    mdelay(1);
  }

  icpmulti_debugk("EDBG: END: init_do_channels(...)\n");

  return 0;
}

/*
==============================================================================
        Read/write char devices
==============================================================================
*/

int icpmulti_device_open(struct inode *inode, struct file *file) {
  unsigned int major = imajor(inode);

  if ((file->f_mode & FMODE_WRITE) &&
    ((major == ICPMULTI_AIMAJOR) || (major == ICPMULTI_DIMAJOR)))
    return -EINVAL;
  else return 0;
}

int icpmulti_device_release(struct inode *inode, struct file *file) {
  return 0;
}

int icpmulti_device_read(struct file *filp, char *buff, size_t len,
  loff_t *f_pos) {
  int result = -EFAULT;
  unsigned int major = imajor(filp->f_dentry->d_inode);
  unsigned int minor = iminor(filp->f_dentry->d_inode);
  short short_val;
  unsigned char char_val;

  switch (major) {
    case ICPMULTI_AIMAJOR:
      if (len >= sizeof(short_val)) {
        result = icpmulti_read_ai(minor, &short_val);
        if (result > 0)
          if (!copy_to_user(buff, (char*)&short_val, sizeof(short_val)))
          result = sizeof(short_val);
      }
    break;
    case ICPMULTI_AOMAJOR:
      if (len >= sizeof(short_val)) {
        result = icpmulti_read_ao(minor, &short_val);
        if (result > 0)
          if (!copy_to_user(buff, (char*)&short_val, sizeof(short_val)))
          result = sizeof(short_val);
      }
    break;
    case ICPMULTI_DIMAJOR:
      if (len >= sizeof(char_val)) {
        result = icpmulti_read_di(minor, &char_val);
//         if (result > 0)
//           if (!copy_to_user(buff, (char*)&char_val, sizeof(char_val)))
//           result = sizeof(char_val);

        if (result > 0) {
          mdelay(10);
          if (char_val > 0) copy_to_user(buff, "1", 1);
          else copy_to_user(buff, "0", 1);

          result = 1;
        }
      }
    break;
    case ICPMULTI_DOMAJOR:
      if (len >= sizeof(char_val)) {
        result = icpmulti_read_do(minor, &char_val);
//         if (result > 0)
//           if (!copy_to_user(buff, (char*)&char_val, sizeof(char_val)))
//           result = sizeof(char_val);

        if (result > 0) {
          mdelay(10);
          if (char_val > 0) copy_to_user(buff, "1", 1);
          else copy_to_user(buff, "0", 1);

          result = 1;
        }
      }
    break;
  }

  return result;
}

int icpmulti_device_write(struct file *filp, const char *buff, size_t len,
  loff_t *off) {
  int result = -EFAULT;
  unsigned int major = imajor(filp->f_dentry->d_inode);
  unsigned int minor = iminor(filp->f_dentry->d_inode);
  short short_val;
  unsigned char char_val;

  switch (major) {
    case ICPMULTI_AOMAJOR:
      if (len >= sizeof(short_val)) {
        if (!copy_from_user((char*)&short_val, buff, sizeof(short_val)))
          result = icpmulti_write_ao(minor, short_val);
      }
    break;
    case ICPMULTI_DOMAJOR:
      if (len >= sizeof(char_val)) {
        if (!copy_from_user((char*)&char_val, buff, sizeof(char_val))) {
//           result = icpmulti_write_do(minor, char_val);

          if (char_val == '0') result = icpmulti_write_do(minor, 0);
          else result = icpmulti_write_do(minor, 1);
        }
      }
    break;
  }

  return result;
}
