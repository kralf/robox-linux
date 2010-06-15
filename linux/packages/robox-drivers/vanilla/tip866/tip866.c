/*-------------------------------------------------------------
 * Linux on RoboX project - ASL EPFL
 *-------------------------------------------------------------
 *
 * tip866 linux driver for kernel 2.6
 *
 * modifications by Ralf Kaestner
 *
 *-------------------------------------------------------------
 * $Id: tip866.c,v 1.15 2004/02/03 08:25:15 fred Exp $
 *-------------------------------------------------------------*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/config.h>
#include <linux/pci.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/smp_lock.h>

#include <linux/devfs_fs_kernel.h>
#include <linux/devfs_fs.h>
#include <linux/proc_fs.h>
#include <asm/semaphore.h>

#include "tip866.h"

/*
==============================================================================
        Constant definitions
==============================================================================
*/

// IDs and names
#define TIP866_VENDORID         0xb3
#define TIP866_DEVICEID         0x1d
#define TIP866_NAME             "TEWS TIP866-20 serial interface"
#define TIP866_DRVNAME          "tip866"

// Slot A on the carrier board
#define TIP866_CARRIER_IOBASE   0xb8000000
#define TIP866_IOOFFSET         0x0fff

#define TIP866_VENDORIDOFFSET   0x09
#define TIP866_DEVICEIDOFFSET   0x0b

#define TIP866_PROM_OFFSET      0x100
#define TIP866_PROM_LEN         0x40

#define TIP866_MEM_LEN          256

// Useful definitions
#define TIP866_CHANNELS         2
#define TIP866_RXBUFLEN         2048

// Read registers
#define TIP866_RHR              0x01
#define TIP866_IER              0x03
#define TIP866_ISR              0x05
#define TIP866_LCR              0x07
#define TIP866_MSR              0x0d

// Read/Write registers
#define TIP866_LSB              0x01
#define TIP866_MSB              0x03
#define TIP866_MCR              0x09
#define TIP866_LSR              0x0b
#define TIP866_INTVEC           0x0f
#define TIP866_EFR              0x05

// Write registers
#define TIP866_THR              0x01
#define TIP866_IER              0x03
#define TIP866_FCR              0x05

#define TIP866_RXREADY          0x01
#define TIP866_RXTRIGGER        0x04
#define TIP866_RXTIMEOUT        0x0c
#define TIP866_TXREADY          0x20
#define TIP866_BUFOVERFLOW      0x02

// Character devices
#define TIP866_MAJOR            0

// ProcFS
#define TIP866_PROCNAME         "tip866"

/*
==============================================================================
        Kernel messages and debugging
==============================================================================
*/

static int debug;

#define tip866_printk(fmt, arg...) {\
  char message[256]; \
  sprintf(message, fmt, ## arg); \
  printk("%s: %s", TIP866_DRVNAME, message); \
}
#define tip866_alertk(fmt, arg...) \
  tip866_printk(KERN_ALERT fmt, ## arg)
#define tip866_debugk(fmt, arg...) \
  if (debug) tip866_printk(KERN_DEBUG fmt, ## arg)

/*
==============================================================================
        Driver structures and declarations
==============================================================================
*/

typedef struct {
  unsigned int line;
  unsigned char *miobase;
  atomic_t available_bytes;
  int roffset;
  int woffset;
  unsigned char rxbuffer[TIP866_RXBUFLEN];

  atomic_t rxbytes;
  atomic_t txbytes;
  atomic_t overflows;
  atomic_t reqbytes;
} tip866_channel;

typedef struct {
  unsigned int iobase;
  unsigned char *miobase;
  int baud;
  int parity;
  int wordlen;
  int stopbit;
  unsigned char *prom;
  int major;

  // mutexes
  struct semaphore rd_sem;              // read semaphore
  struct semaphore wr_sem;              // write semaphore

  tip866_channel channel[TIP866_CHANNELS];
} tip866_device;

static tip866_device tip866_dev;

// Parameters default values
static int iobase = TIP866_CARRIER_IOBASE+TIP866_IOOFFSET;
static int baud = 500000;
static int parity = 0;
static int wordlen = 8;
static int stopbit = 1;
static int major = 0; // auto

/*
==============================================================================
       Kernel driver code
==============================================================================
*/

MODULE_AUTHOR("Frederic Pont and Ralf Kaestner");
MODULE_DESCRIPTION("Linux driver for TEWS TIP866-20 serial interface");
MODULE_LICENSE("GPL");
module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug enabled or not");
module_param(iobase, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(iobase, "Base I/O memory of module");
module_param(baud, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(baud, "Serial interface baud rate");
module_param(parity, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(parity, "Serial interface parity");
module_param(wordlen, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(wordlen, "Serial interface data word length");
module_param(stopbit, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(stopbit, "Serial interface stop bit");

/*
==============================================================================
        Helper functions
==============================================================================
*/

int tip866_busy_sleep(int delay) {
  unsigned long j = jiffies + delay/10000000;

  while(jiffies < j)
    schedule();

  return 0;
};

/*
==============================================================================
        More forward declarations
==============================================================================
*/

int tip866_open(struct inode *inode, struct file *filp);
int tip866_release(struct inode *inode, struct file *filp);
int tip866_proc_read(char *buf, char **start, off_t offset, int count,
  int *eof, void *data);

/*
==============================================================================
        Device initialization and removal
==============================================================================
*/

static int __init tip866_init(void) {
  int channel = 0;
  unsigned char *channel_base;
  unsigned char lcr;

  // store parameters
  tip866_dev.iobase = iobase;
  tip866_dev.stopbit = stopbit;
  tip866_dev.major = major;

  if (check_mem_region(tip866_dev.iobase, TIP866_MEM_LEN)) {
    tip866_alertk("Memory already in use\n");
    return -EBUSY;
  }

  request_mem_region(tip866_dev.iobase, TIP866_MEM_LEN, TIP866_DRVNAME);
  tip866_dev.miobase = ioremap(tip866_dev.iobase, TIP866_MEM_LEN);

  if(tip866_dev.miobase) {
    tip866_dev.prom = tip866_dev.miobase+TIP866_PROM_OFFSET;

    // Check for device
    if ((readb(tip866_dev.prom+TIP866_VENDORIDOFFSET) == TIP866_VENDORID) &&
      (readb(tip866_dev.prom+TIP866_DEVICEIDOFFSET) == TIP866_DEVICEID)) {
      tip866_printk("%s detected\n", TIP866_NAME);

      // init channels
      for (channel = 0; channel < TIP866_CHANNELS; channel++) {
	channel_base = tip866_dev.miobase+channel*0x10;
	tip866_dev.channel[channel].miobase = channel_base;
	tip866_init_channel(channel);

	// setup baudrate
	writeb(0x80, channel_base+TIP866_LCR);

	switch(baud) {
	case 9600:
	  tip866_dev.baud = baud;
	  writeb(0x34, channel_base+TIP866_LSB);
	  writeb(0x00, channel_base+TIP866_MSB);
	  break;
	case 38400:
	  tip866_dev.baud = baud;
	  writeb(0x0d, channel_base+TIP866_LSB);
	  writeb(0x00, channel_base+TIP866_MSB);
	  break;
	default: // 500000
	  tip866_dev.baud = baud;
	  writeb(0x01, channel_base+TIP866_LSB);
	  writeb(0x00, channel_base+TIP866_MSB);
	  break;
	}

 	// reset MCR as we do not use the divider
	writeb(0xbf, channel_base+TIP866_LCR);
	writeb(0x10, channel_base+TIP866_EFR);
	writeb(0x00, channel_base+TIP866_LCR);
	writeb(0x01, channel_base+TIP866_IER);
	writeb(0x00, channel_base+TIP866_MCR);

	// disable HW flow control in EFR
	writeb(0xbf, channel_base+TIP866_LCR);
	writeb(0x00, channel_base+TIP866_EFR);
	writeb(0x00, channel_base+TIP866_LCR);

	// reset and program LCR
	writeb(0, channel_base+TIP866_LCR);
	lcr = 0;

	switch(wordlen) {
	case 5:
	  tip866_dev.wordlen = wordlen;
	  break;
	case 6:
	  lcr |= 1;
	  tip866_dev.wordlen = wordlen;
	  break;
	case 7:
	  lcr |= 2;
	  tip866_dev.wordlen = wordlen;
	  break;
	default:
	  lcr |= 3;
	  tip866_dev.wordlen = 8;
	  break;
	}

	switch(stopbit) {
	case 1:
	  tip866_dev.stopbit = stopbit;
	  break;
	case 2:
	  lcr |= 4;
	  tip866_dev.stopbit = stopbit;
	  break;
	default:
	  tip866_dev.stopbit = 1;
	  break;
	}

	switch(parity) {
	case 0:
	  tip866_dev.parity = parity;
	  break;
	case 1:
	  lcr |= 8;
	  tip866_dev.parity = parity;
	  break;
	case 2:
	  lcr |= 0x18;
	  tip866_dev.parity = parity;
	  break;
	case 3:
	  lcr |= 0x28;
	  tip866_dev.parity = parity;
	  break;
	case 4:
	  lcr |= 0x38;
	  tip866_dev.parity = parity;
	  break;
	default:
	  tip866_dev.parity = 0;
	  break;
	}

        writeb(lcr, channel_base+TIP866_LCR);

        tip866_debugk("wordlength=%d, stopbit=%d, parity=%d\n",
          tip866_dev.wordlen, tip866_dev.stopbit, tip866_dev.parity);

	tip866_printk("Channel %i baudrate set to %d\n", channel,
          tip866_dev.baud);

        // init mutexes
        sema_init(&tip866_dev.rd_sem, 1);
        sema_init(&tip866_dev.wr_sem, 1);
      }
    }
    else {
      tip866_alertk("%s not found\n", TIP866_NAME);

      iounmap((void*)tip866_dev.miobase);
      release_mem_region(tip866_dev.iobase, TIP866_MEM_LEN);

      return -ENODEV;
    }
  }
  else {
    tip866_alertk("I/O remap from 0x%x failed\n", tip866_dev.iobase);

    release_mem_region(tip866_dev.iobase, TIP866_MEM_LEN);

    return -ENODEV;
  }

  // ProcFS entries
  create_proc_read_entry(TIP866_PROCNAME, 0, NULL, tip866_proc_read, NULL);

  return 0;
}

static void __exit tip866_exit(void) {
  remove_proc_entry(TIP866_PROCNAME, NULL);

  iounmap((void*)tip866_dev.miobase);
  release_mem_region(tip866_dev.iobase, TIP866_MEM_LEN);

  tip866_printk("%s removed\n", TIP866_NAME);
}

module_init(tip866_init);
module_exit(tip866_exit);

/*
==============================================================================
        Channel initialization
==============================================================================
*/

void tip866_init_channel(int channel) {
  tip866_dev.channel[channel].roffset = 0;
  tip866_dev.channel[channel].woffset = 0;

  atomic_set(&tip866_dev.channel[channel].available_bytes, 0);
  atomic_set(&tip866_dev.channel[channel].rxbytes, 0);
  atomic_set(&tip866_dev.channel[channel].txbytes, 0);
  atomic_set(&tip866_dev.channel[channel].reqbytes, 0);

  // Enable FIFO, reset RX and TX FIFOs, set TX trigger to 16
  writeb(0x47, tip866_dev.channel[channel].miobase+TIP866_FCR);
}

/*
==============================================================================
        Read/write operations
==============================================================================
*/

int tip866_read_channel(int channel, unsigned char *buf) {
  int len = 0;
  unsigned char lsr;
  tip866_channel *chan = &tip866_dev.channel[channel];

  down(&tip866_dev.rd_sem);

  lsr = readb(chan->miobase+TIP866_LSR);

  // Get all available bytes
  while (lsr & TIP866_RXREADY) {
    if (lsr & TIP866_BUFOVERFLOW) {
      atomic_inc(&chan->overflows);
      return -1;
    }

    buf[len] = readb(chan->miobase+TIP866_RHR);
    len++;

    lsr = readb(chan->miobase+TIP866_LSR);
  }

  up(&tip866_dev.rd_sem);

  if (len) tip866_debugk("%d bytes received on channel %d\n", len, channel);

  return len;
}

int tip866_write_channel(int channel, const unsigned char *buf, int count) {
  int len = 0;
  unsigned int status = 0;
  unsigned char *channel_base;

  down(&tip866_dev.wr_sem);

  channel_base = tip866_dev.miobase+channel*0x10;

  status = readb(channel_base+TIP866_LSR);

  while (len < count) {
    writeb(buf[len], channel_base+TIP866_RHR);

    len++;
    atomic_inc(&tip866_dev.channel[channel].txbytes);

    // Slow down for SICK devices
    tip866_busy_sleep(100000);

    status = readb(channel_base+TIP866_LSR);
  }

  up(&tip866_dev.wr_sem);

  if (len) tip866_debugk("%d bytes sent on channel %d\n", len, channel);

  return len;
}

/*
==============================================================================
        ProcFS operations
==============================================================================
*/

int tip866_proc_read(char *buf, char **start, off_t offset, int count,
  int *eof, void *data) {
  int i, len=0;

  for (i=0; i < TIP866_CHANNELS; i++) {
    len += sprintf(buf+len, "\nChannel %d:\n", i);
    len += sprintf(buf+len, "   overflows: %d\n",
      atomic_read(&tip866_dev.channel[i].overflows));
    len += sprintf(buf+len, "   rx:        %d\n",
      atomic_read(&tip866_dev.channel[i].rxbytes));
    len += sprintf(buf+len, "   tx:        %d\n",
      atomic_read(&tip866_dev.channel[i].txbytes));
    len += sprintf(buf+len, "   woffset:   %d\n",
      tip866_dev.channel[i].woffset);
    len += sprintf(buf+len, "   roffset:   %d\n",
      tip866_dev.channel[i].roffset);
    len += sprintf(buf+len, "   available: %d\n",
      atomic_read(&tip866_dev.channel[i].available_bytes));
  }
  *eof = 1;

  return len;
}
