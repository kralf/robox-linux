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
 *-------------------------------------------------------------
 * $Id: icpmulti.c,v 1.5 2004/01/12 12:54:08 fred Exp $
 *-------------------------------------------------------------*/

/** \file icpmulti.c
 * \brief ICPMULTI (digital and analog I/O) device driver (kernel module)
 */

#include "icpmulti.h"

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>

#define ICPMULTI_MAJOR 210
int icpmulti_read_di0(struct file *filp, char *buf, size_t count,
		      loff_t *f_pos);

static struct file_operations icpmulti_fops = {
  read:   icpmulti_read_di0
};

static int debug;

#define ICPMULTI_DEVICEID	0x8000	// Device ID
#define ICPMULTI_VENDORID	0x104c	// Device ID
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

#define ICPMULTI_NORTAI
#ifdef ICPMULTI_NORTAI
typedef int SEM;
#define BIN_SEM 0
// DUMMY RTAI FUNCTIONS for non-RTAI system
int rt_sem_wait(SEM *sem){return 0;}
int rt_sem_signal(SEM *sem){return 0;}
int rt_sem_delete(SEM *sem){return 0;}
int rt_typed_sem_init(SEM *sem,int i,int type){return 0;}
int rt_busy_sleep(int delay){
  unsigned long j = jiffies + delay/10000000;
  while(jiffies < j)
    schedule();
  return 0;
};
#endif

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
  unsigned int *ai_chanlist;		// actaul chanlist
  unsigned char *ai_data;		// data buffer
  unsigned int ao_data[4];		// data output buffer
  unsigned char di_data;		// Digital input data
  unsigned int do_data;	                // Remember digital output data
  unsigned char do_state;

  int n_aichan;	                        // num of A/D chans
  int n_aochan;	                        // num of D/A chans
  int n_dichan;	                        // num of DI chans
  int n_dochan;	                        // num of DO chans

  /* mutexes */
  SEM ai_sem;                           // analog in semaphore
  SEM ao_sem;                           // analog out semaphore
  SEM di_sem;                           // digital in semaphore
  SEM do_sem;                           // digital out semaphore
} icpmulti_private;

static icpmulti_private private;

MODULE_AUTHOR ("Frederic Pont");
MODULE_DESCRIPTION ("Linux driver for ICP-Multi Composite A&D I/O Board");
MODULE_LICENSE ("GPL");
module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug enabled or not");

/*
==============================================================================
	More forward declarations
==============================================================================
*/

int icpmulti_reset(void);
static int init_ao_channels(void);
static void setup_channel_list(void);

/*
==============================================================================
	Functions
==============================================================================
*/

static int icpmulti_init(void) {
  struct pci_dev *dev;
  int result;

  printk("%s: module_init()\n", ICPMULTI_NAME);
  dev = pci_find_device(ICPMULTI_VENDORID,ICPMULTI_DEVICEID,NULL);

  if (!dev) {
    printk(KERN_ALERT "%s: device not found\n", ICPMULTI_NAME);
    return -ENODEV;
  }

  printk("%s: board detected\n",ICPMULTI_NAME);

  if (pci_enable_device(dev))
    return -ENODEV;

  /* configure PCI */
  pci_write_config_dword(dev,0xb0,0x00050000); // enable pci 16 bits
  pci_write_config_dword(dev,0x04,0x00000002); // enable memory

  private.n_aichan = 16;
  private.n_aochan = 4;
  private.n_dichan = 16;
  private.n_dochan = 8;
  private.do_state = 0;

  /* init mutexes */
  rt_typed_sem_init(&private.ai_sem,1,BIN_SEM);
  rt_typed_sem_init(&private.ao_sem,1,BIN_SEM);
  rt_typed_sem_init(&private.di_sem,1,BIN_SEM);
  rt_typed_sem_init(&private.do_sem,1,BIN_SEM);


  private.phys_iobase = pci_resource_start(dev,2);

  private.iobase = ioremap(private.phys_iobase, ICP_MULTI_SIZE);

  if (!private.iobase) {
    printk("%s: ioremap failed.\n",ICPMULTI_NAME);
    return -ENOMEM;
  }
  {
    int i, test;

    icpmulti_reset();
    init_ao_channels();

    printk("%s: do_state=%x\n",ICPMULTI_NAME,private.do_state);

    icpmulti_write_do(6,1);

    for(i=0;i<private.n_dichan;i++) {
      icpmulti_read_di(i,&test);
      printk("channel %d: %d\n",i,test);
    }

#ifdef NO
    icpmulti_write_do(0,1);
    printk("%s: do_state=%x\n",ICPMULTI_NAME,private.do_state);
    icpmulti_write_do(0,0);
    printk("%s: do_state=%x\n",ICPMULTI_NAME,private.do_state);

    i=1000000000;
    while(i--);
#endif
    rt_busy_sleep(1000000000);

    /* turn off flash */
    icpmulti_write_do(6,0);
  }

  /* register char driver */
  result = register_chrdev(ICPMULTI_MAJOR,ICPMULTI_NAME,&icpmulti_fops);
  if (result < 0)
    printk("%s: could not register major number %d\n",ICPMULTI_NAME,
    ICPMULTI_MAJOR);

  return 0;
}


static void icpmulti_exit(void) {
  int result;

  result = unregister_chrdev(ICPMULTI_MAJOR,ICPMULTI_NAME);

  /* delete mutexes */
  rt_sem_delete(&private.ai_sem);
  rt_sem_delete(&private.ao_sem);
  rt_sem_delete(&private.di_sem);
  rt_sem_delete(&private.do_sem);

  if(private.iobase) iounmap(private.iobase);

  return;
}

module_init(icpmulti_init);
module_exit(icpmulti_exit);

/*
==============================================================================

	Name:	icp_multi_insn_read_ai

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

int icpmulti_read_ai(int channel, int *value) {
  int n,timeout;
  int range = 0x30; // range -10 10

  if (debug)
    printk("%s: EDBG: BGN: icp_multi_insn_read_ai(...)\n",
    ICPMULTI_NAME);

  rt_sem_wait(&private.ai_sem);

  // Disable A/D conversion ready interrupt
  private.IntEnable &= ~ADC_READY;
  writew(private.IntEnable,private.iobase + ICP_MULTI_INT_EN);

  // Clear interrupt status
  private.IntStatus |= ADC_READY;
  writew(private.IntStatus,private.iobase + ICP_MULTI_INT_STAT);

  // Set up appropriate channel, mode and range data, for specified channel
  private.AdcCmdStatus &= 0xf00f;
  private.AdcCmdStatus |= range;
  private.AdcCmdStatus |= (channel << 8);

  /* Output channel, range, mode to ICP Multi */
  writew(private.AdcCmdStatus, private.iobase+ICP_MULTI_ADC_CSR);

  if (debug)
    printk("%s: A ST=%4x IO=%x\n",ICPMULTI_NAME,
    readw(private.iobase+ICP_MULTI_ADC_CSR),
    private.iobase+ICP_MULTI_ADC_CSR);

  for (n=0; n<1; n++) {
    // Set start ADC bit
    private.AdcCmdStatus |= ADC_ST;
    writew(private.AdcCmdStatus, private.iobase+ICP_MULTI_ADC_CSR);
    private.AdcCmdStatus &= ~ADC_ST;

  if (debug)
    printk("%s: B n=%d ST=%4x\n",ICPMULTI_NAME,n,
    readw(private.iobase+ICP_MULTI_ADC_CSR));

  if (debug)
    printk("%s: C n=%d ST=%4x\n",ICPMULTI_NAME,n,
    readw(private.iobase+ICP_MULTI_ADC_CSR));

    // Wait for conversion to complete, or get fed up waiting
    timeout=100;
    while (timeout--) {
      if (!(readw(private.iobase+ICP_MULTI_ADC_CSR) & ADC_BSY))
      goto conv_finish;

      if (debug) if (!(timeout%10))
        printk("%s: D n=%d tm=%d ST=%4x\n",ICPMULTI_NAME, n,timeout,
        readw(private.iobase+ICP_MULTI_ADC_CSR));
    }

    // If we reach here, a timeout has occurred
    printk("%s: A/D insn timeout", ICPMULTI_NAME);

    // Disable interrupt
    private.IntEnable &= ~ADC_READY;
    writew(private.IntEnable,private.iobase + ICP_MULTI_INT_EN);

    // Clear interrupt status
    private.IntStatus |= ADC_READY;
    writew(private.IntStatus,private.iobase + ICP_MULTI_INT_STAT);

    // Clear data received
    *value=0;

    if (debug)
      printk("%s: EDBG: END: icp_multi_insn_read_ai(...) n=%d\n",
      ICPMULTI_NAME,n);

    rt_sem_signal(&private.ai_sem);
    return -ETIME;

conv_finish:
    *value = (readw(private.iobase+ICP_MULTI_AI) >> 4 ) & 0x0fff;
  }

  // Disable interrupt
  private.IntEnable &= ~ADC_READY;
  writew(private.IntEnable,private.iobase + ICP_MULTI_INT_EN);

  // Clear interrupt status
  private.IntStatus |= ADC_READY;
  writew(private.IntStatus,private.iobase + ICP_MULTI_INT_STAT);

  if (debug) {
    printk("%s: read AI chan=%d value=%d\n",ICPMULTI_NAME,
      channel,*value);
    printk("%s: EDBG: END: icp_multi_insn_read_ai(...) n=%d\n",
      ICPMULTI_NAME,n);
  }

  rt_sem_signal(&private.ai_sem);
  return 0;
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

  if (debug)
    printk("%s: EDBG: BGN: icp_multi_insn_write_ao(...)\n",
    ICPMULTI_NAME);

  rt_sem_wait(&private.ao_sem);

  // Disable D/A conversion ready interrupt
  private.IntEnable &= ~DAC_READY;
  writew(private.IntEnable,private.iobase + ICP_MULTI_INT_EN);

  // Clear interrupt status
  private.IntStatus |= DAC_READY;
  writew(private.IntStatus,private.iobase + ICP_MULTI_INT_STAT);

  // Set up range and channel data
  // Bit 4 = 1 : Bipolar
  // Bit 5 = 0 : 5V
  // Bit 5 = 1 : 10V
  // Bits 8-9 : Channel number
  private.DacCmdStatus &= 0xfccf;
  private.DacCmdStatus |= range;
  private.DacCmdStatus |= (channel << 8);

  writew(private.DacCmdStatus, private.iobase+ICP_MULTI_DAC_CSR);

  for (n=0; n<1; n++) {
    /* Wait for analogue output data register to be ready for new data,
       or get fed up waiting */
    timeout=100;
    while (timeout--) {
      if (!(readw(private.iobase+ICP_MULTI_DAC_CSR) & DAC_BSY))
        goto dac_ready;

      if (!(timeout%10))
        printk("%s: ERROR :AO channel%d n=%d tm=%d ST=%4x\n",
        ICPMULTI_NAME,channel,n,timeout,
        readw(private.iobase+ICP_MULTI_DAC_CSR));

      rt_busy_sleep(1000);
    }

    // If we reach here, a timeout has occurred
    printk("%s: D/A insn timeout", ICPMULTI_NAME);

    // Disable interrupt
    private.IntEnable &= ~DAC_READY;
    writew(private.IntEnable,private.iobase + ICP_MULTI_INT_EN);

    // Clear interrupt status
    private.IntStatus |= DAC_READY;
    writew(private.IntStatus,private.iobase + ICP_MULTI_INT_STAT);

    // Clear data received
    private.ao_data[channel]=0;

    if (debug)
      printk("%s: EDBG: END: icp_multi_insn_write_ao(...) n=%d\n",
      ICPMULTI_NAME,n);

    rt_sem_signal(&private.ao_sem);
    return -ETIME;

dac_ready:
    // Write data to analogue output data register
    writew(value*0x10, private.iobase + ICP_MULTI_AO);

    if (debug)
      printk("%s: AO chan+%d, range=%d: %x at %x+%d\n",ICPMULTI_NAME,
      channel,range,value*0x10,private.iobase,ICP_MULTI_AO);

    // Set DAC_ST bit to write the data to selected channel
    private.DacCmdStatus |= DAC_ST;
    writew(private.DacCmdStatus, private.iobase+ICP_MULTI_DAC_CSR);
    private.DacCmdStatus &= ~DAC_ST;

    // Save analogue output data
    private.ao_data[channel]=value;
  }

  if (debug)
    printk("%s: EDBG: END: icp_multi_insn_write_ao(...) n=%d\n",
    ICPMULTI_NAME,n);

  rt_sem_signal(&private.ao_sem);
  return n;
}

/*
==============================================================================

	Name:	icpmulti_read_ao

	Description:
		This function reads a single analogue output.

==============================================================================
*/

int icpmulti_read_ao(int channel, int *value) {
  *value=private.ao_data[channel];

  return 1;
}

/*
==============================================================================

	Name:	icp_multi_read_di

	Description:
		This function reads the digital inputs.

==============================================================================
*/

int icpmulti_read_di(int channel, int *value) {
  unsigned short status;

  rt_sem_wait(&private.di_sem);

  status = readw(private.iobase + ICP_MULTI_DI);
  status >>= channel;
  status &= 0x0001;

  if (status)
    (*value) = 1;
  else
    (*value) = 0;

  return 0;
}

/*
==============================================================================

	Name:	icpmulti_write_do

	Description:
		This function writes the appropriate digital outputs.

==============================================================================
*/

int icpmulti_write_do(int channel, unsigned char value) {
  unsigned short bit = 0;

  rt_sem_wait(&private.do_sem);

  bit = 0x01 << channel;
  private.do_state &= ~bit;
  if(value) private.do_state |= bit;

  writew(private.do_state, private.iobase + ICP_MULTI_DO);

  rt_sem_signal(&private.do_sem);
  return 0;
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

  if (debug)
    printk("%s: EDBG:  setup_channel_list()\n",ICPMULTI_NAME);

  /* FIXME: clean this up */
  for (i=0; i<private.n_aichan; i++) {
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

    if (debug)
      printk("%s: GS: %2d. [%4x]=%4x %4x\n", ICPMULTI_NAME, i, chanprog,
      range, private.act_chanlist[i]);
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
  unsigned int    i;

  if (debug)
    printk("%s: EDBG: BGN: icp_multi_reset(...)\n", ICPMULTI_NAME);

  // Clear INT enables and requests
  writew(0, private.iobase + ICP_MULTI_INT_EN);
  writew(0x00ff, private.iobase + ICP_MULTI_INT_STAT);

  // Set DACs to 0 5 range and 0V output
  for (i =0; i < private.n_aochan; i++) {
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
    rt_busy_sleep(100000);
  }

  // Digital outputs to 0
  writew(0, private.iobase + ICP_MULTI_DO);

  if (debug)
    printk("%s: EDBG: END: icp_multi_reset(...)\n", ICPMULTI_NAME);

  return 0;
}

static int init_ao_channels(void) {
  int chan;
  int init_value = 2048;
  int range = 0x30;

  if (debug)
    printk("%s: EDBG: BGN: init_ao_channels(...)\n", ICPMULTI_NAME);

  // Disable D/A conversion ready interrupt
  private.IntEnable &= ~DAC_READY;
  writew(private.IntEnable,private.iobase + ICP_MULTI_INT_EN);

  // Clear interrupt status
  private.IntStatus |= DAC_READY;
  writew(private.IntStatus,private.iobase + ICP_MULTI_INT_STAT);

  for (chan =0; chan < private.n_aochan; chan++) {

    private.DacCmdStatus &= 0xfccf;
    private.DacCmdStatus |= range;
    private.DacCmdStatus |= (chan << 8);

    writew(private.DacCmdStatus, private.iobase+ICP_MULTI_DAC_CSR);

    // Write data to analogue output data register
    writew(init_value*0x10, private.iobase + ICP_MULTI_AO);

    // Set DAC_ST bit to write the data to selected channel
    private.DacCmdStatus |= DAC_ST;

    writew(private.DacCmdStatus, private.iobase+ICP_MULTI_DAC_CSR);
    private.DacCmdStatus &= ~DAC_ST;

    // Save analogue output data
    private.ao_data[chan]=init_value;

    rt_busy_sleep(100000);
  }

  if (debug)
    printk("%s: EDBG: END: init_ao_channels(...)\n", ICPMULTI_NAME);

  return 0;
}

int icpmulti_read_di0(struct file *filp, char *buf, size_t count,
  loff_t *f_pos) {
  int value;

  if (debug)
    printk("%s: read_di0\n", ICPMULTI_NAME);

  if(count < sizeof(value))
    return -EFAULT;

  icpmulti_read_di(0,&value);

  if (copy_to_user(buf,(char *)&value,sizeof(value)))
    return -EFAULT;

  return sizeof(value);
}
