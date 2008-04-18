/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://asl.epfl.ch/
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA
 */

/** \file sicklms200.c
 * \brief sicklms2xx device driver (kernel module, RTAI task)
 *  \bug If the system is stopped abruptly (e.g. kernel crash) and rebooted
 *  wihout completely powering off, the driver is not able to recover, as 
 *  continious scans arrive from Sick. 
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/config.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>

#include <linux/devfs_fs_kernel.h>
#include <linux/devfs_fs.h>


#include <asm/semaphore.h>
#include <linux/workqueue.h>

#include <rtai_sched.h>
#include <rtai_sem.h>

#include "../tip866/tip866.h"

#ifndef CONFIG_DEVFS_FS
#error "**"
#error "This driver needs DEVFS support to be available"
#error "**"
#endif


MODULE_AUTHOR ("frederic.pont@epfl.ch");
MODULE_DESCRIPTION ("Linux driver for Sick LMS200 (RTAI task)");
MODULE_LICENSE ("GPL");


#define SICKLMS200_NAME "sicklms"
#define SICKLMS200_MAJOR 0
#define BUFLEN           2048
#define CHANNELS         2
#define NSPERIOD 500000

#define MAKEUINT16(lo, hi) ((((unsigned short) (hi)) << 8) | ((unsigned short) (lo)))  
#define LOBYTE(w) ((unsigned char) (w & 0xFF))
#define HIBYTE(w) ((unsigned char) ((w >> 8) & 0xFF))


//#define SICKLMS200_DEBUG

#ifdef SICKLMS200_DEBUG
#define PDEBUG(fmt, args...) printk(KERN_WARNING "sicklms: " fmt, ##args)
#warning 
#warning *************************** 
#warning SICKLMS200 debugging is ON 
#warning *************************** 
#warning  
#else 
#define PDEBUG(fmt, args...)
#endif


#define STX     0x02
#define ACK     0x06
#define NACK    0x15
#define CRC16_GEN_POL 0x8005

#define MODE_REQUEST   0x25
#define MODE_CONTINOUS 0x24


int sicklms200_open(struct inode *inode, struct file *filp);
int sicklms200_release(struct inode *inode, struct file *filp);
int sicklms200_read(struct file *filp, char *buf, size_t count,
		loff_t *f_pos);
int sicklms200_write(struct file *filp, const char *buf, size_t count,
		 loff_t *f_pos);
int sicklms200_ioctl(struct inode *inode, struct file *filep, 
		 unsigned int cmd, unsigned long arg);
unsigned short sicklms200_createcrc(unsigned char* data, ssize_t len);

void sicklms200_read_scan(unsigned long arg);
int  sicklms200_proc_read(char *buf,char **start, off_t offset,
		     int count, int *eof, void *data);     
int  sicklms200_read_response(int channel, unsigned char *buf);
void sick_rx_task(int arg);
int  init_channel(int channel);

struct sick_telegram *get_tmp_telegram(int channel);
struct sick_telegram *get_telegram(int channel);
int make_telegram_available(int channel);
int sicklms200_set_mode(int channel, int mode);
unsigned short sicklms200_createcrc(unsigned char* data, ssize_t len);
int sicklms200_request_type(int channel);
int sicklms200_configure(int channel);
int sicklms200_enter_config_mode(int channel);
int sicklms200_select_variant(int channel);



struct file_operations sicklms200_fops = {
  read:    sicklms200_read,
  write:   sicklms200_write,
  open:    sicklms200_open,
  release: sicklms200_release,
  ioctl:   sicklms200_ioctl,
};


/* Telegram states */
#define RX_ACK  0
#define RX_HDR  1
#define RX_DATA 2
#define RX_DONE 3
#define RX_ERROR 4


#define ACK_OK 0x06

#define HDRLEN 5
#define BUFLEN 2048
#define CRCLEN 2

struct sick_telegram {
  unsigned int   id;           /* telegram id, incremented for each rx telegram */
  unsigned char  ack;       
  unsigned char  hdr[HDRLEN];  /* hdr buffer */
  unsigned short datalen;      /* data length (without cmd, crc) */
  unsigned char  cmd;         
  unsigned char  data[BUFLEN]; /* data buffer, without cmd, crc */
  unsigned char  crc[CRCLEN];  /* crc bytes */
  unsigned int   state;

  int hdroffset;
  int dataoffset;
};


struct sick_channel {
  int id;
  int telegramid;
  struct sick_telegram telegram[2];  /* telegram being received */
  int current;  /* telegram being received */
  int complete; /* last complete telegram */ 
  unsigned char scanmode; /* request or continous scanning */
};

struct sicklms200_private {
  int major;
  devfs_handle_t dev;
  struct sick_channel channel[CHANNELS];

  int nsperiod; /* task period in ns */
  RT_TASK rt_task;
};


static struct sicklms200_private private;




static int sicklms200_init(void)
{
  RTIME now, tick_period;
  int i;

  printk(KERN_ALERT "%s: driver init\n",
	 SICKLMS200_NAME);
 
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)      
  /* get major number for device */
  private.major = register_chrdev(SICKLMS200_MAJOR, "sicklms200", &sicklms200_fops);
  if (private.major < 0) {
    printk(KERN_WARNING "%s: can not get major number\n", SICKLMS200_NAME);
    return -ENODEV;
  }

  /* devfs */
  devfs_mk_cdev(MKDEV(private.major,0),
		S_IFCHR | S_IRUGO | S_IWUGO,
		"sicklms200");
#else
  private.dev = devfs_register(NULL,"sicklms200",
			       DEVFS_FL_AUTO_DEVNUM,
			       0, 0, S_IFCHR | S_IRUGO | S_IWUGO,
			       &sicklms200_fops, NULL);
#endif

  /* proc */
  create_proc_read_entry("sicklms200",0,NULL,sicklms200_proc_read,NULL);
  
  /* init tip866 channels */
  for(i=0;i<CHANNELS;i++) {
    init_channel(i);
    tip866_init_channel(i);
  }

  private.nsperiod = NSPERIOD;
  
  /* start real-time task */
  tick_period = nano2count(private.nsperiod);
  now = rt_get_time();
  rt_task_init(&private.rt_task,sick_rx_task,0,10000,0,0,0);
  rt_task_make_periodic(&private.rt_task, now+tick_period, tick_period);
  
  
  for(i=0;i<CHANNELS;i++) {
    /* get sick type */
    sicklms200_request_type(i);
    /* configure SICK */
    sicklms200_enter_config_mode(i);
    sicklms200_select_variant(i);
    sicklms200_configure(i);
  }
  
  for(i=0;i<CHANNELS;i++)
    /* set continous scan mode */
    sicklms200_set_mode(i, MODE_CONTINOUS);
  
  return 0;
}


static void sicklms200_exit(void)
{
  int i;

  /* stop continous scan 
   * (we don't read the answer..) */
  for(i=0;i<CHANNELS;i++)
    sicklms200_set_mode(i, MODE_REQUEST);
  
  rt_busy_sleep(100000000);
  rt_task_delete(&private.rt_task);

  remove_proc_entry("sicklms200",NULL);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)        
  unregister_chrdev(private.major, "sicklms200");
  devfs_remove("sicklms200");
#else
  devfs_unregister(private.dev);
#endif

  printk(KERN_ALERT "%s exit\n",SICKLMS200_NAME);
  
}


module_init(sicklms200_init);
module_exit(sicklms200_exit);


int sicklms200_open(struct inode *inode, struct file *filp)
{
  
  return 0;
  
}

int sicklms200_release(struct inode *inode, struct file *filp)
{
  
  return 0;
  
}

ssize_t sicklms200_read(struct file *filp, char *buf, size_t count,
		    loff_t *f_pos)
{
  return -1;
}

ssize_t sicklms200_write(struct file *filp, const char *buf, size_t count,
		    loff_t *f_pos)
{
  return -1;
}

int sicklms200_ioctl(struct inode *inode, struct file *filep, 
			  unsigned int cmd, unsigned long arg)
{
  return -1;
}


unsigned short sicklms200_createcrc(unsigned char* data, ssize_t len)
{
  unsigned short uCrc16;
  unsigned char abData[2];
  
  uCrc16 = 0;
  abData[0] = 0;
  
  while(len-- )
  {
    abData[1] = abData[0];
    abData[0] = *data++;
    
    if( uCrc16 & 0x8000 )
    {
      uCrc16 = (uCrc16 & 0x7fff) << 1;
      uCrc16 ^= CRC16_GEN_POL;
    }
    else
    {    
      uCrc16 <<= 1;
    }
    uCrc16 ^= MAKEUINT16(abData[0],abData[1]);
  }
  return (uCrc16); 
}


int sicklms200_check_response(int channel, unsigned char cmd)
{
  struct sick_telegram *response;
  response = &private.channel[channel].telegram[private.channel[channel].complete];
 
  if(response->cmd != cmd+0x80) {
    printk("error (expecting cmd 0x%x, got 0x%x)\n",
	   cmd+0x80,response->cmd);
    return -1;
  }	
  
  return 0;
}

int sicklms200_request_type(int channel)
{
  int len;
  unsigned char req[256];
  unsigned short crc;
  struct sick_telegram *response;

  /* build type request */
  req[0] = 0x02;
  req[1] = 0;
  req[2] = 1;
  req[3] = 0;
  req[4] = 0x3a;
  len = 5;
  
  crc = sicklms200_createcrc(req,len);

  req[len]   = LOBYTE(crc);
  req[len+1] = HIBYTE(crc);

  tip866_write_channel(channel,req,len+2);
  
  rt_busy_sleep(300000000);

  response = &private.channel[channel].telegram[private.channel[channel].complete];
  printk("sicklms%d: type request: ",channel);
  if(sicklms200_check_response(channel,req[4]))
    return 0;
  response->data[response->datalen] = 0x0;
  printk("'%s'\n", response->data);
  

  return 0;
}


int sicklms200_enter_config_mode(int channel)
{
  int len;
  unsigned char req[256];
  unsigned short crc;
  struct sick_telegram *response;
  
  req[0]  = 0x02;
  req[1]  = 0x00;
  req[2]  = 0x0A;
  req[3]  = 0x00;
  req[4]  = 0x20;
  req[5]  = 0x00;
  req[6]  = 'S';
  req[7]  = 'I';
  req[8]  = 'C';
  req[9]  = 'K';
  req[10] = '_';
  req[11] = 'L';
  req[12] = 'M';
  req[13] = 'S';
  
  len = 14;
  crc = sicklms200_createcrc(req,len);
  req[len]   = LOBYTE(crc);
  req[len+1] = HIBYTE(crc);

  tip866_write_channel(channel,req,len+2);
  
  /* wait for sick to send response. how long? */
  rt_busy_sleep(100000000);

  
  response = &private.channel[channel].telegram[private.channel[channel].complete];
  printk("sicklms%d: enter config mode: ",channel);
  if(sicklms200_check_response(channel,req[4]))
    return 0;
  switch(response->data[0]) {
  case 0:
    printk("OK.\n");
    break;
  case 1:
    printk("incorrect password.\n");
    break;
  default:	
    printk("unknown status (%x).\n",response->data[0]);
    break;
  }
  return 0;
  
}

int sicklms200_configure(int channel)
{
  int len;
  unsigned char req[256];
  unsigned short crc;
  struct sick_telegram *response;

  req[0]=0x02;
  req[1]=0x00;
  req[2]=0x21;
  req[3]=0x00;
  req[4]=0x77;
  req[5]=0x00;
  req[6]=0x00;
  req[7]=0x46;
  req[8]=0x00;  
  req[9]=0x00;  
  req[10]= 0x01; /* measurement mode */
  req[11]= 0x01; /* unit 0=cm, 1=mm */
  req[12]=0x00;
  req[13]=0x00;
  req[14]=0x02;
  req[15]=0x02;
  req[16]=0x05;
  req[17]=0x00;
  req[18]=0x00;
  req[19]=0x02;
  req[20]=0x03;
  req[21]=0x0A;
  req[22]=0x14;
  req[23]=0x00;
  req[24]=0x04;
  req[25]=0x05;
  req[26]=0x14;
  req[27]=0x1E;
  req[28]=0x00;
  req[29]=0x06;
  req[30]=0x07;
  req[31]=0x5A;
  req[32]=0x78;
  req[33]=0x00;
  req[34]=0x00;
  req[35]=0x00;
  req[36]=0x00;
  
  len = 37;
  crc = sicklms200_createcrc(req,len);
  req[len]   = LOBYTE(crc);
  req[len+1] = HIBYTE(crc);

  tip866_write_channel(channel,req,len+2);
  
  /* wait for sick to send response. how long? */
  rt_busy_sleep(500000000);

  
  response = &private.channel[channel].telegram[private.channel[channel].complete];
  printk("sicklms%d: apply configuration: ",channel);
  if(sicklms200_check_response(channel,req[4]))
    return 0;
  switch(response->data[0]) {
  case 0:
    printk("configuration not accepted (0x%x).\n",response->data[0]);
    break;
  case 1:
    printk("OK.\n");
    break;
  default:	
    printk("unknown status (%x).\n",response->data[0]);
    break;
  }
  return 0;
  
}



int sicklms200_select_variant(int channel)
{
  int len;
  unsigned char req[256];
  unsigned short crc;
  struct sick_telegram *response;
  
  /* build type request */
  req[0] = 0x02;
  req[1] = 0x00;
  req[2] = 0x05;
  req[3] = 0x00;
  req[4] = 0x3b;
  req[5] = 180;
  req[6] = 0x00;
  req[7] = 50;
  req[8] = 0x00;

  len = 9;
  
  crc = sicklms200_createcrc(req,len);

  req[len]   = LOBYTE(crc);
  req[len+1] = HIBYTE(crc);

  tip866_write_channel(channel,req,len+2);
  
  rt_busy_sleep(300000000);
  
  response = &private.channel[channel].telegram[private.channel[channel].complete];
  printk("sicklms%d: change variant: ",channel);
  if(sicklms200_check_response(channel,req[4]))
    return 0;
  switch(response->data[0]) {
  case 0:
    printk("aborted.\n");
    break;
  case 1:
    printk("OK (0x%x, 0x%x).\n",response->data[1],response->data[3]);
    break;
  default:	
    printk("unknown status 0x%x.\n",response->data[0]);
    break;
  }
  

  return 0;
}



int sicklms200_proc_read(char *buf,char **start, off_t offset,
		     int count, int *eof, void *data)
{
  int len=0;
  int i,j;


  len += sprintf(buf+len,"Sick LMS 200\n");

  for(i=0;i<CHANNELS;i++) {
    len += sprintf(buf+len,"channel %d: current=%d complete=%d id=%d, mode=0x%x\n",i,
		   private.channel[i].current,
		   private.channel[i].complete,
		   private.channel[i].telegramid,
		   private.channel[i].scanmode);
    len += sprintf(buf+len," current  HDR: ");
    for(j=0;j<HDRLEN;j++)
      len += sprintf(buf+len," 0x%x",private.channel[i].telegram[private.channel[i].current].hdr[j]);
     len += sprintf(buf+len,"\n");
    len += sprintf(buf+len," complete HDR: ");
    for(j=0;j<HDRLEN;j++)
      len += sprintf(buf+len," 0x%x",private.channel[i].telegram[private.channel[i].complete].hdr[j]);
     len += sprintf(buf+len,"\n");
   
  }
  

  
  
  *eof = 1;
  return len;
}     


int init_channel(int channel)
{
  struct sick_telegram *telegram;
  int i;
  
  for(i=0;i<2;i++) {
    telegram = &private.channel[channel].telegram[i];
    telegram->state = RX_ACK;
  }
  
  private.channel[channel].telegramid = 0;
  private.channel[channel].current    = 0;
  private.channel[channel].complete   = 1;
  private.channel[channel].scanmode   = MODE_REQUEST; /* default mode is request based */  
  return 0; 

}


void sick_rx_task(int arg)
{

  int i, offset, len;
  unsigned char buf[2048];
  struct sick_channel *channel;
  struct sick_telegram *telegram;
  
  PDEBUG("rx_task started\n");
    
  for(;;) {
    
    
    for(i=0;i<CHANNELS;i++) {
      
      channel  = &private.channel[i];
      telegram = get_tmp_telegram(i);

      /* read bytes on channel */
	len = tip866_read_channel(i,buf);
      
      if(len < 0) {
	
	printk("error on channel %d, EXIT.\n",i);
	return;  /* FIXME: handle this better */
	
      } else if (len > 0) {
	
	offset = 0;
	
	while (offset < len) {
	  
	  switch (telegram->state) {

	  case RX_ACK:
	    /* receive ACK */
	    telegram->ack = buf[offset]; offset++;
	    PDEBUG("A 0x%x\n", telegram->ack);
	    if(telegram->ack == ACK_OK) {
	      telegram->state = RX_HDR;
	      PDEBUG("%d: ACK received\n",i);
	    } else {
	      telegram->state = RX_ERROR;
	      printk("tip866: NACK(?) received (%x) on channel %i.exit\n",telegram->ack,i);
	      return;
	    }
	    break;
	     
	  case RX_HDR:
	    /* receive HDR */
	    telegram->hdr[telegram->hdroffset] = buf[offset]; offset++;
	    PDEBUG("H 0x%x\n", telegram->hdr[telegram->hdroffset]);
	    telegram->hdroffset++;
	    if (telegram->hdroffset == HDRLEN) {
	      /* extract data len */
	      telegram->datalen = telegram->hdr[2]+256*telegram->hdr[3]-1; /* cmd byte already there */
	      PDEBUG("%d: datalen=%d \n",i,telegram->datalen);
	      telegram->cmd = telegram->hdr[4];
	      telegram->hdroffset = 0;
	      telegram->state = RX_DATA;
	      PDEBUG("%d: HDR received\n",i);
	    }
	    break;
	    
	  case RX_DATA:
	    /* receive DATA */
	    telegram->data[telegram->dataoffset] = buf[offset]; offset++;
	    PDEBUG("D 0x%x\n", telegram->data[telegram->dataoffset]);
	    telegram->dataoffset++;
	    if (telegram->dataoffset == telegram->datalen+2) { /* add 2 for CRC */
	      PDEBUG("dataoffset=%d, offset=%d, len=%d \n",telegram->dataoffset, offset, len);
	      /* do something */
	      telegram->dataoffset = 0;
	      telegram->state      = RX_DONE;
	      PDEBUG("%d: DATA received\n",i);
	    }
	    break;
	  }
	  
	  
	  /* FIXME: protect access to all this */
	  if (telegram->state == RX_DONE || telegram->state == RX_ERROR) {
	    
	    PDEBUG("telegram complete\n");
	    
	    /* FIXME: this should be just a swap of pointers.. */
	    make_telegram_available(i);
	    
	    /* wait for next message */
	    if(private.channel[i].scanmode == MODE_CONTINOUS)
	      private.channel[i].telegram[private.channel[i].current].state = RX_HDR;
	    else private.channel[i].telegram[private.channel[i].current].state = RX_ACK; 
	  
	  }
	  
	}
	
	
      }
    } /* for(channel) */

    
    rt_task_wait_period();
  } /* for(;;) */
  
  return;
}

int sicklms200_read_channel(int channel, unsigned char *buf) {
  int len = 0;
  int i;
  int complete;
  struct sick_telegram *last;
    
  //sicklms200_request_scan(channel);
  
  complete = private.channel[channel].complete;
  last     = &private.channel[channel].telegram[complete];
  
  /* transfer data to user*/
  for(i=2;i<last->datalen-1;i++,len++) {
    buf[len] = last->data[i];
  }
  
  return len;
}


int sicklms200_get_scanlen(int channel)
{
  int complete;
  struct sick_telegram *last;

  /* FIXME: we probably need some locking here */

  complete = private.channel[channel].complete;
  last     = &private.channel[channel].telegram[complete];

  /* FIXME: clean this mess */
  return (last->datalen-3)/2;

}

struct sick_telegram *get_tmp_telegram(int channel)
{
  
  return &private.channel[channel].telegram[private.channel[channel].current];
  
}

struct sick_telegram *get_telegram(int channel)
{
  return &private.channel[channel].telegram[private.channel[channel].complete];
  
}


int make_telegram_available(int channel)
{
  
  if(private.channel[channel].current == 0 &&
     private.channel[channel].complete == 1) {
    private.channel[channel].current   = 1;
    private.channel[channel].complete = 0;
  } else if(private.channel[channel].current == 1 &&
	    private.channel[channel].complete == 0) {
    private.channel[channel].current   = 0;
    private.channel[channel].complete = 1;
  } else {
    printk("sicklms200: error, current=%d, complete=%d, EXIT.\n",
	   private.channel[channel].current,
	   private.channel[channel].complete);
    return -1;
  }
  

  private.channel[channel].telegram[private.channel[channel].complete].id =
	private.channel[channel].telegramid++;
  return 0;
}

int sicklms200_set_mode(int channel, int mode)
{

  unsigned char req[256];
  unsigned short crc;
  int len;
  
  private.channel[channel].scanmode = mode;

  req[0] = 2;
  req[1] = 0;
  req[2] = 2;
  req[3] = 0;
  req[4] = 0x20;
  req[5] = mode;
    
  len = 6;
  
  crc = sicklms200_createcrc(req,len);
  
  req[len]   = LOBYTE(crc);
  req[len+1] = HIBYTE(crc);
  
  tip866_write_channel(channel,req,len+2);

  return 0;

}






int sicklms200_request_scan(int channel)
{
  int len,t;
  unsigned char req[256];
  unsigned short crc;
  int complete;
  int state,id;
  
  //printk("%s: request_scan on channel %d\n",SICKLMS200_NAME,channel);

  /* get one scan */
  req[0] = 2;
  req[1] = 0;
  req[2] = 2;
  req[3] = 0;
  req[4] = 0x30;
  req[5] = 0x01;
  len = 6;
  
  crc = sicklms200_createcrc(req,len);

  req[len]   = LOBYTE(crc);
  req[len+1] = HIBYTE(crc);

  tip866_write_channel(channel,req,len+2);

  /* wait for sick to send response. how long? */
  t = 100000000;
  while(t--)
    /* do nothing */;

  //ret = tip866_read_response(channel, buf);
  complete = private.channel[channel].complete;
  state = private.channel[channel].telegram[complete].state;
  id    = private.channel[channel].telegram[complete].id;
  if(state == RX_DONE) {
    printk("sicklms200: scan received on channel %d state=%d (id=%d)\n",channel,state,id);
  } else {
    printk("could not get scan reponse from sick (%d)\n",state);
  }
  
  
  return 0;
}

