/*-------------------------------------------------------------
 * Linux on RoboX project - ASL EPFL
 *-------------------------------------------------------------
 *
 * tip866 linux driver for kernel 2.6
 *
 *-------------------------------------------------------------
 * $Id: tip866.c,v 1.15 2004/02/03 08:25:15 fred Exp $
 *-------------------------------------------------------------*/

/** \file tip866.c
 * \brief TIP866 (RS422) device driver (kernel module)
 */


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

#include <rtai_sched.h>
#include <rtai_sem.h>



#ifndef CONFIG_PCI
#error "**"
#error "This driver needs PCI support to be available"
#error "**"
#endif

#ifndef CONFIG_DEVFS_FS
#error "**"
#error "This driver needs DEVFS support to be available"
#error "**"
#endif


MODULE_AUTHOR ("Frederic Pont");
MODULE_DESCRIPTION ("Linux driver for TIP866-20");
MODULE_LICENSE ("EPFL");

/* Turn debug messages on/off here */
//#define TIP866_DEBUG

#ifdef TIP866_DEBUG
#define PDEBUG(fmt, args...) printk(KERN_WARNING "tip866: " fmt, ##args)
#warning 
#warning ********************** 
#warning TIP866 debugging is ON 
#warning ********************** 
#warning  
#else 
#define PDEBUG(fmt, args...)
#endif

#define TIP866_NAME "tip866"

/* slot A on the carrier board */
#define IPCARRIER_IOBASE    0xb8000000
#define TIP866_OFFSET       0x0fff

#define TIP866_MANUF_ID     0xb3
#define TIP866_MODEL_ID     0x1d
#define TIP866_MANUF_OFFSET 0x09
#define TIP866_MODEL_OFFSET 0x0b

#define TIP866_PROM_OFFSET  0x100
#define TIP866_PROM_LEN     0x40

#define TIP866_MEM_LEN      256

/* only 2 channels used on RoboX */
#define TIP866_CHANNELS     2
#define RXBUFLEN            2048


/* Read registers */
#define TIP866_RHR    0x01
#define TIP866_IER    0x03
#define TIP866_ISR    0x05
#define TIP866_LCR    0x07
#define TIP866_MSR    0x0d

/* Read Write registers */
#define TIP866_LSB    0x01
#define TIP866_MSB    0x03
#define TIP866_MCR    0x09
#define TIP866_LSR    0x0b
#define TIP866_INTVEC 0x0f
#define TIP866_EFR    0x05


/* Write registers */
#define TIP866_THR    0x01
#define TIP866_IER    0x03
#define TIP866_FCR    0x05

#define RXREADY       0x01
#define RXTRIGGER     0x04
#define RXTIMEOUT     0x0c
#define TXREADY       0x20
#define BUFOVERFLOW   0x02
#define TIP866_MAJOR  0



/* channel */
struct tip866_port {
  unsigned int line;
  unsigned char *miobase;
  atomic_t available_bytes;
  int roffset;
  int woffset;
  unsigned char rxbuffer[RXBUFLEN];
  atomic_t rxbytes;
  atomic_t txbytes;
  atomic_t overflows;
  atomic_t reqbytes;
  struct semaphore read_sem;
  SEM sem;
};



#define HDRLEN 5
#define BUFLEN 2048

#define RX_ACK  0
#define RX_HDR  1
#define RX_DATA 2
#define RX_DONE 3
#define RX_ERROR 4

#define ACK_OK 0x06

struct sick_telegram {
  unsigned char ack;
  unsigned char hdr[HDRLEN];
  unsigned char data[BUFLEN];
  int offset;
  unsigned short datalen;
  unsigned int state;
};

struct sick_response {
  unsigned char status;
  unsigned char cmd;
  unsigned short len;
  unsigned char data[BUFLEN];
};

struct tip866_private {
  unsigned int iobase;
  unsigned char *miobase;
  int baud;
  int parity;
  int wordlen;
  int stopbit;
  unsigned char *prom;
  int major;
  struct tip866_port channel[TIP866_CHANNELS];
  struct sick_telegram tmp_telegram[TIP866_CHANNELS];
  struct sick_response response[TIP866_CHANNELS];

  RT_TASK rt_task;

};


static struct tip866_private private;

MODULE_PARM(iobase, "i");
MODULE_PARM_DESC(iobase, "Base I/O memory for tip866");
MODULE_PARM(baud, "i");
MODULE_PARM_DESC(baud, "Baud rate ()");
MODULE_PARM(parity, "i");
MODULE_PARM_DESC(parity, "Parity ()");
MODULE_PARM(wordlen, "i");
MODULE_PARM_DESC(wordlen, "Data word length ()");
MODULE_PARM(stopbit, "i");
MODULE_PARM_DESC(stopbit, "Stop bit()");


/* parameters default values */
static int iobase  = IPCARRIER_IOBASE+TIP866_OFFSET;
static int baud    = 500000;
static int parity  = 0;
static int wordlen = 8;
static int stopbit = 1;
static int major   = 0; /* autodetect */


int tip866_open(struct inode *inode, struct file *filp);
int tip866_release(struct inode *inode, struct file *filp);
int tip866_print_registers(unsigned char *channel_base);
void tip866_rx_task(int arg);
void tip866_sick_rx_task(int arg);
int tip866_proc_read(char *buf,char **start, off_t offset,
		     int count, int *eof, void *data);     



static int tip866_init(void)
{
  
  int channel = 0;
  unsigned char *channel_base;
  unsigned char lcr;  


  /* store parameters in private struct */
  private.iobase  = iobase;
  private.stopbit = stopbit;
  private.major   = major;
  
  printk(KERN_ALERT "%s: driver init (iobase=0x%8x)\n",
	 TIP866_NAME,private.iobase);
  
  if (check_mem_region(private.iobase,TIP866_MEM_LEN)) {
    printk(KERN_ALERT "%s: memory already in use\n", TIP866_NAME);
    return -EBUSY;
  }
  
  request_mem_region(private.iobase,TIP866_MEM_LEN,TIP866_NAME);
  
  private.miobase = ioremap(private.iobase,TIP866_MEM_LEN);
  
  if(private.miobase) {
  
    private.prom = private.miobase+TIP866_PROM_OFFSET;
    
    /* check if there is a tip866 there */
    if ((readb(private.prom+TIP866_MANUF_OFFSET)
	 == TIP866_MANUF_ID) && 
	(readb(private.prom+TIP866_MODEL_OFFSET)
	 == TIP866_MODEL_ID)) {
      
      printk(KERN_ALERT "%s: TIP866-20 device found\n", TIP866_NAME);
      
      /* init channels */
      for (channel=0;channel<TIP866_CHANNELS;channel++) {
	
	channel_base = private.miobase + channel*0x10;
	private.channel[channel].miobase = channel_base;
	tip866_init_channel(channel);

	/* enable receiver ready interrupt */
	//writeb(0x00,channel_base+TIP866_IER);
	
	
	/* setup baudrate */
	writeb(0x80,channel_base+TIP866_LCR);
	
	switch(baud) {
	case 9600:
	  private.baud = baud;
	  writeb(0x34,channel_base+TIP866_LSB);
	  writeb(0x00,channel_base+TIP866_MSB);
	  break;


	case 38400:
	  private.baud = baud;
	  writeb(0x0d,channel_base+TIP866_LSB);
	  writeb(0x00,channel_base+TIP866_MSB);
	  break;

	default:
	  /* 500000 */
	  private.baud = baud;
	  writeb(0x01,channel_base+TIP866_LSB);
	  writeb(0x00,channel_base+TIP866_MSB);
	  break;
	}
	printk("%s: baud_rate=%d\n", TIP866_NAME, private.baud);
	
	
 	/* reset MCR as we don't use the divider (bit7) */
	writeb(0xbf,channel_base+TIP866_LCR);
	writeb(0x10,channel_base+TIP866_EFR);
	writeb(0x00,channel_base+TIP866_LCR);
	writeb(0x01,channel_base+TIP866_IER);
	writeb(0x00,channel_base+TIP866_MCR);
	
	/* disable HW flow control in EFR */
	writeb(0xbf,channel_base+TIP866_LCR);
	writeb(0x00,channel_base+TIP866_EFR);
	writeb(0x00,channel_base+TIP866_LCR);

	/* reset and program LCR */
	writeb(0,channel_base+TIP866_LCR);
	lcr = 0;
	switch(wordlen) {
	case 5:
	  private.wordlen = wordlen;
	  break;	
	case 6:
	  lcr |= 1;
	  private.wordlen = wordlen;
	  break;
	case 7:
	  lcr |= 2;
	  private.wordlen = wordlen;
	  break;
	default:	
	  lcr |= 3;
	  private.wordlen = 8;
	  break;
	}
	printk("%s: wordlen=%d\n", TIP866_NAME, private.wordlen);

	switch(stopbit) {
	case 1:
	  private.stopbit = stopbit;
	  break;
	case 2:
	  lcr |= 4;
	  private.stopbit = stopbit;
	  break;
	default:	
	  private.stopbit = 1;
	  break;
	}
	printk("%s: stopbit=%d\n", TIP866_NAME, private.stopbit);
	
	switch(parity) {
	case 0:
	  private.parity = parity;
	  break;	
	case 1:
	  lcr |= 8;
	  private.parity = parity;
	  break;
	case 2:
	  lcr |= 0x18;
	  private.parity = parity;
	  break;
	case 3:
	  lcr |= 0x28;
	  private.parity = parity;
	  break;
	case 4:
	  lcr |= 0x38;
	  private.parity = parity;
	  break;
	default:	
	  private.parity = 0;
	  break;
	}
	printk("%s: parity=%d\n", TIP866_NAME, private.parity);

	writeb(lcr,channel_base+TIP866_LCR);
	printk("%s: lcr=0x%x\n", TIP866_NAME, lcr);


	printk(KERN_ALERT "%s: channel %i configured\n\n", TIP866_NAME, channel);

      }
      
      
    } else {
      
      printk(KERN_ALERT "%s: TIP866-20 device not found\n", TIP866_NAME);
      iounmap((void *)private.miobase);
      release_mem_region(private.iobase,TIP866_MEM_LEN);

      return -ENODEV;
    
    }
    
  } else {
    
    printk(KERN_ALERT "%s: could not ioremap() iobase=0x%8x ",
	   TIP866_NAME,private.iobase);
    release_mem_region(private.iobase,TIP866_MEM_LEN);

    return -ENODEV;
  }

  //rt_sem_init(&private.channel[0].sem,0);
  //rt_sem_init(&private.channel[1].sem,0);


  /* init and start real-time task */
  //rt_task_init(&private.rt_task,tip866_rx_task,0,2000,0,0,0);
  //rt_task_init(&private.rt_task,tip866_sick_rx_task,0,10000,0,0,0);
  //rt_task_make_periodic_relative_ns(&private.rt_task,5000,500000);
  
  
  /* proc */
  create_proc_read_entry("tip866",0,NULL,tip866_proc_read,NULL);
  
  printk("%s: driver loaded\n\n",TIP866_NAME);

  /* success */
  return 0;
}


static void tip866_exit(void)
{

  //rt_task_delete(&private.rt_task);

  remove_proc_entry("tip866",NULL);

  iounmap((void *)private.miobase);
  release_mem_region(private.iobase,TIP866_MEM_LEN);

  printk(KERN_ALERT "%s: driver exit\n",TIP866_NAME);

}


module_init(tip866_init);
module_exit(tip866_exit);



int tip866_read_channel(int chan, unsigned char *buf)
{
  int len = 0;
  unsigned char lsr;
  struct tip866_port *channel = &private.channel[chan]; 
  
  lsr = readb(channel->miobase+TIP866_LSR);
  
  /* get all available bytes */
  while (lsr & RXREADY) {

    if (lsr & BUFOVERFLOW) {
      //printk("%s: BUFFER OVERFLOW on channel %d\n",TIP866_NAME,chan);
      //printk(" lsr=%x\n",lsr);
      atomic_inc(&channel->overflows);
      return -1;
    }
    
    buf[len] = readb(channel->miobase+TIP866_RHR);
    len++;
    
    lsr = readb(channel->miobase+TIP866_LSR);	
    
  }

  if (len) PDEBUG("%d bytes read on channel %d\n", len, chan);
  return len;
}


int tip866_write_channel(int channel, const unsigned char *buf, int count)
{
  
  int len = 0;
  unsigned int status = 0;
  unsigned char *channel_base;
  
  channel_base = private.miobase + channel*0x10;

  status = readb(channel_base+TIP866_LSR);
  /* the TXREADY check should not be necessary. It is there to
   * slow down transmission to ensure  Sick is able to receive
   * data when we use high bit rate like 500kb/s 
   * This is inspired by the Oberon driver */
  //while ((status & TXREADY) && (len < count)) {
  while (len < count) {
    writeb(buf[len],channel_base+TIP866_RHR);
    //printk("%s: w 0x%x channel %d\n", TIP866_NAME,buf[len],channel);
    len++;
    atomic_inc(&private.channel[channel].txbytes);
    
    /* be cool with Sick. slow down. ugly but works */
    rt_busy_sleep(100000);
  
    status = readb(channel_base+TIP866_LSR);
  }

  PDEBUG("%d bytes sent on channel %i\n", len, channel);
  return len;
  
}


/* to be used in case of buffer overflow */
void tip866_init_channel(int channel)
{
  private.channel[channel].roffset = 0;
  private.channel[channel].woffset = 0;
  atomic_set(&private.channel[channel].available_bytes, 0);
  atomic_set(&private.channel[channel].rxbytes, 0);
  atomic_set(&private.channel[channel].txbytes, 0);
  atomic_set(&private.channel[channel].reqbytes, 0);
  init_MUTEX_LOCKED(&private.channel[channel].read_sem);

  /* enable FIFO, reset RX and TX FIFOs, 
   * set TX trigger to 16 */
  writeb(0x47,private.channel[channel].miobase+TIP866_FCR);
  
  //printk("%s: channel %d reset done\n",TIP866_NAME,channel);

}


EXPORT_SYMBOL(tip866_read_channel);
EXPORT_SYMBOL(tip866_write_channel);
EXPORT_SYMBOL(tip866_init_channel);


int tip866_proc_read(char *buf,char **start, off_t offset,
		     int count, int *eof, void *data)
{
  int i,len=0;
  
  for (i=0;i<TIP866_CHANNELS;i++) {
    len += sprintf(buf+len,"\nChannel %d:\n",i);
    len += sprintf(buf+len,"   overflows: %d\n",
		   atomic_read(&private.channel[i].overflows));
    len += sprintf(buf+len,"   rx:        %d\n",
		   atomic_read(&private.channel[i].rxbytes));
    len += sprintf(buf+len,"   tx:        %d\n",
		   atomic_read(&private.channel[i].txbytes));
    len += sprintf(buf+len,"   woffset:   %d\n",private.channel[i].woffset);
    len += sprintf(buf+len,"   roffset:   %d\n",private.channel[i].roffset);
    len += sprintf(buf+len,"   available: %d\n",
		   atomic_read(&private.channel[i].available_bytes));

  }

  len += sprintf(buf+len,"\nprivate.response0:status=%x, cmd=%x, len=%d, data=%s\n",
		 private.response[0].status,
		 private.response[0].cmd,
		 private.response[0].len,
		 private.response[0].data);
  
  len += sprintf(buf+len,"private.response1:status=%x, cmd=%x, len=%d, data=%s\n",
		 private.response[1].status,
		 private.response[1].cmd,
		 private.response[1].len,
		 private.response[1].data);
  
  *eof = 1;
  return len;
}     



/* Debug stuff follows */

int tip866_print_registers(unsigned char *channel_base)
{
  
  int i;
  unsigned char lcr;
  
  lcr = readb(channel_base+TIP866_LCR);
  
  printk("\n%s: register set 1\n", TIP866_NAME);  
  for (i=3;i<16;i=i+2) {
    printk("%s: 0x%x 0x%x\n",TIP866_NAME,i,readb(channel_base+i));
  }

  printk("\n%s: register set 2\n", TIP866_NAME);
  writeb(0xbf,channel_base+TIP866_LCR);
  for (i=1;i<16;i=i+2) {
    printk("%s: 0x%x 0x%x\n",TIP866_NAME,i,readb(channel_base+i));
  }

  /* reset lcr value */
  writeb(lcr,channel_base+TIP866_LCR);  

  printk("\n%s: normal operation\n", TIP866_NAME);  
  for (i=3;i<16;i=i+2) {
    printk("%s: 0x%x 0x%x\n",TIP866_NAME,i,readb(channel_base+i));
  }

  return 0;
}


int print_prom(void)
{
  
  int i;
  
  for (i=1;i<TIP866_PROM_LEN;i=i+2) 
    printk(KERN_ALERT "%d: 0x%2x\n",i,readb(private.prom+i));
  
  return 0;
}

#ifdef USE_ISR    
#define TIP866_RX_TRIGGER 0x04
   
    if (isr & TIP866_RX_TRIGGER) {
      lsr = readb(channel_base+TIP866_LSR);
      printk("%s: channel %d: lsr=%x\n", TIP866_NAME,channel,lsr);
      
      while(count<16) {

	c = readb(channel_base+TIP866_RHR);
	lsr = readb(channel_base+TIP866_LSR);
	isr = readb(channel_base+TIP866_ISR);

	printk("%s: channel %d: %x (ISR=%x,LSR=%x)\n", TIP866_NAME,channel,c,isr,lsr);
	
	count++;
      }
    }
#endif

























#ifdef USE_OLD_STUFF



void tip866_rx_task(int arg)
{

  unsigned char lsr;
  struct tip866_port *channel; 
  int i;

  printk("tip866_rx_task started\n");

  for(;;) {

    //printk("tip866_rx_task invoked\n");
  
    
    for (i=0;i<TIP866_CHANNELS;i++) {
      
      channel = &private.channel[i];
      
      /* we do just polling for now */
      lsr = readb(channel->miobase+TIP866_LSR);
    
      while (lsr & RXREADY) {
	if (lsr & BUFOVERFLOW) {
	  //printk("%s: BUFFER OVERFLOW on channel %d\n",TIP866_NAME,i);
	  //printk(" lsr=%x\n",lsr);
	  atomic_inc(&channel->overflows);
	}
	
	channel->rxbuffer[channel->woffset]
	  = readb(channel->miobase+TIP866_RHR);

	channel->woffset = (channel->woffset+1) % RXBUFLEN;
	atomic_inc(&channel->available_bytes);
	atomic_inc(&channel->rxbytes);
	
#ifdef BLOCK_ON_READ
	if(atomic_read(&channel->reqbytes)) {
	  if(atomic_read(&channel->available_bytes) >= 
	     atomic_read(&channel->reqbytes))
	    up(&channel->read_sem);
	}
#endif

	lsr = readb(channel->miobase+TIP866_LSR);	
      }
      	
    }
    
    rt_task_wait_period();
  } 



 
  return;
}



void tip866_sick_rx_task(int arg)
{

  unsigned char lsr;
  struct tip866_port *channel; 
  int i,j;
  struct sick_telegram *telegram;
  struct sick_response *response;

  printk("tip866_sick_rx_task started\n");

  for(;;) {
    
    
    for (i=0;i<TIP866_CHANNELS;i++) {
      
      channel = &private.channel[i];
  
      /* we do just polling for now */
      lsr = readb(channel->miobase+TIP866_LSR);
    
      while (lsr & RXREADY) {
	if (lsr & BUFOVERFLOW) {
	  printk("%s: BUFFER OVERFLOW on channel %d\n",TIP866_NAME,i);
	  printk(" lsr=%x\n",lsr);
	  atomic_inc(&channel->overflows);
	}
	
	telegram = &private.tmp_telegram[i];

	switch (telegram->state) {
	case RX_ACK:
	  /* receive ACK */
	  telegram->ack = readb(channel->miobase+TIP866_RHR);
	  //PDEBUG("Ax%x ", telegram->ack);
	  if(telegram->ack == ACK_OK) {
	    telegram->offset = 0;
	    telegram->state  = RX_HDR;
	    PDEBUG("%d: ACK received\n",i);
	  } else {
	    telegram->state = RX_ERROR;
	    printk("tip866: NACK received (%x) on channel %i\n",telegram->ack,i);
	  }
	  break;;
	  
	case RX_HDR:
	  /* receive HDR */
	  telegram->hdr[telegram->offset] = readb(channel->miobase+TIP866_RHR);
	  //PDEBUG("Hx%x ", telegram->hdr[telegram->offset]);
	  telegram->offset++;
	  if (telegram->offset == HDRLEN) {
	    /* extract data len */
	    telegram->datalen = *(&telegram->hdr[2]) -1; /* remove 1 (cmd byte received) */
	    telegram->offset = 0;
	    telegram->state = RX_DATA;
	    PDEBUG("%d: HDR received\n",i);
	  }
	  break;
	  
	case RX_DATA:
	  /* receive DATA */
	  telegram->data[telegram->offset] = readb(channel->miobase+TIP866_RHR);
	  telegram->offset++;
	  if (telegram->offset == telegram->datalen+2) { /* add 2 for CRC */
	    /* do something */
	    PDEBUG("%d: DATA received\n",i);
	    telegram->state = RX_DONE;
	  }
	  break;
	}
	
	
	if (telegram->state == RX_DONE || telegram->state == RX_ERROR) {
	  /* telegram done, make it available, cleanup */
	  response = &private.response[i];
	  
	  if (telegram->state == RX_DONE) response->status = 0;
	  else response->status = 1; /* error */
	  
	  response->cmd = telegram->hdr[4];
	  for(j=0;j<telegram->datalen;j++)
	    response->data[j] = telegram->data[j];
	  response->len = telegram->datalen;
	  
	  /* if call was blocking */
	  PDEBUG("calling rt_sem_signal() for channel %d\n",i);
	  //rt_sem_signal(&channel->sem);
	  //rt_sem_wait(&channel->sem);
	  /* wait for next message */
	  telegram->state = RX_ACK;
	}
	
	lsr = readb(channel->miobase+TIP866_LSR);	
      }
      	
    }

    rt_task_wait_period();
  } 



 
  return;
}



ssize_t tip866_read_channel(int channel, char *buf, size_t count)
{
  int offset,len;
  
  offset =  private.channel[channel].roffset;
  len = 0;

#ifdef BLOCK_ON_READ
  if(count > atomic_read(&private.channel[channel].available_bytes)) {
    atomic_set(&private.channel[channel].reqbytes,count);
    down(&private.channel[channel].read_sem);
  }
#endif

  while( (len<atomic_read(&private.channel[channel].available_bytes) ) 
	 && 
	 (len<count) ){ 
    buf[len] = private.channel[channel].rxbuffer[(offset+len)%RXBUFLEN];
    len++;
  }
  
  /* increment offset of read offset */
  private.channel[channel].roffset = 
    (private.channel[channel].roffset+len)%RXBUFLEN;

  /* decrement number of available bytes */
  atomic_add(-len,&private.channel[channel].available_bytes);

  return len;

}


int tip866_read_response(int channel, unsigned char *buf)
{
  
  int i;
  struct sick_response *response;

  /* wait for response to be ready */
  //down(&private.channel[channel].read_sem);
  
  PDEBUG("rt_sem_wait() on channel %d\n",channel); 
  rt_sem_wait(&private.channel[channel].sem);

  PDEBUG("response ready\n");
  
  response = &private.response[channel];
  
  if(response->status != 0)
    return -1;
  
  for(i=0;i<response->len;i++)
    buf[i] = response->data[i];
  
  return 0;

}

#endif
