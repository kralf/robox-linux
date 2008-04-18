/*-------------------------------------------------------------
 * Linux on RoboX project - ASL EPFL
 *-------------------------------------------------------------
 *
 * IP-Quadrature linux driver for kernel 2.6
 *
 *-------------------------------------------------------------
 * $Id: ipquad.c,v 1.10 2004/03/25 14:44:41 cvs Exp $
 *-------------------------------------------------------------*/

/** \file ipquad.c
 * \brief ipquad (encoders) device driver (kernel module)
 */

#include "ipquad.h"

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/config.h>
#include <linux/pci.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>

#include <linux/devfs_fs_kernel.h>
#include <linux/devfs_fs.h>

#include <asm/uaccess.h>

#include <rtai_sched.h>
#include <rtai_sem.h>


MODULE_AUTHOR ("Frederic Pont");
MODULE_DESCRIPTION ("Linux driver for SBS IPQuadCPCI");
MODULE_LICENSE ("EPFL");

#define IPQUAD_NAME "ipquad"

/* slot B on the carrier board */
#define IP_IOBASE 0xb8000000
#define IPQUAD_OFFSET 0x1fff

#define IPQUAD_MANUF_ID     0xf0
#define IPQUAD_MODEL_ID     0x41
#define IPQUAD_MANUF_OFFSET 0x09
#define IPQUAD_MODEL_OFFSET 0x0b
#define STAT_CTRL_OFFSET      49
#define INT_VECT_OFFSET       53

#define CHAN_STAT_CTRL_OFFSET  3
#define CHAN_CONF_OFFSET      33
#define OLPR_COMP_OFFSET       1

#define IPQUAD_PROM_OFFSET 0x100
#define IPQUAD_PROM_LEN    0x40

#define TERM12ENABLE 0xf8
#define UNINITINT    0x0f
#define OUT_CTRL     0x90
#define QUAD         0xc3
#define IN_CTRL      0x48
#define XY_INVERT    0x03

#define READ_OL      0x03

#define IPQUAD_MAX_CHANNELS 4

/* FIXME this is wrong */
#define IPQUAD_MEM_LEN 512

#define IPQUAD_MAJOR 0

//#define IPQUAD_DEBUG

MODULE_PARM(iobase, "i");
MODULE_PARM_DESC(iobase, "Base I/O memory for IPQuad");

int ipquad_open(struct inode *inode, struct file *filp);
int ipquad_release(struct inode *inode, struct file *filp);
int ipquad_read(struct file *filp, char *buf, size_t count,
		loff_t *f_pos);
int ipquad_proc_read(char *buf,char **start, off_t offset,
		     int count, int *eof, void *data);
     

struct file_operations ipquad_fops = {
  read:    ipquad_read,
  open:    ipquad_open,
  release: ipquad_release
};

struct ipquad_private {
  unsigned int iobase;
  unsigned char *miobase;
  unsigned int prom;
  unsigned int major;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)      
  devfs_handle_t devdir;
  devfs_handle_t dev[IPQUAD_MAX_CHANNELS];
#endif
  SEM sem;
};

static struct ipquad_private private;

/* parameters default values */
static int iobase  = IP_IOBASE+IPQUAD_OFFSET;


static int ipquad_init(void)
{
  
  unsigned int channel_base = 0;
  int err = 0;
  int i = 0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)      
  char devname[16];
#endif

  private.iobase = iobase;
  private.major  = 0;

  printk(KERN_ALERT "%s: driver init at iobase=0x%8x\n",
	 IPQUAD_NAME,private.iobase);
  
  if (check_mem_region(private.iobase,IPQUAD_MEM_LEN)) {
    printk(KERN_ALERT "%s: memory already in use\n", IPQUAD_NAME);
    return -EBUSY;
  }
  
  request_mem_region(private.iobase,IPQUAD_MEM_LEN,IPQUAD_NAME);
  
  private.miobase = ioremap(private.iobase,IPQUAD_MEM_LEN);
  
  if(private.miobase) {
  
    private.prom = (unsigned int)private.miobase+IPQUAD_PROM_OFFSET;
  
#ifdef IPQUAD_DEBUG    
    for (i=1;i<IPQUAD_PROM_LEN;i=i+2) 
      printk(KERN_ALERT "%d: 0x%2x\n",i,readb(private.prom+i));
#endif
    
    if ((readb(private.prom+IPQUAD_MANUF_OFFSET)
	 == IPQUAD_MANUF_ID) && 
	(readb(private.prom+IPQUAD_MODEL_OFFSET)
	 == IPQUAD_MODEL_ID)) {
      
      printk(KERN_ALERT "%s: SBS IP Quad device found\n", IPQUAD_NAME);
      
      /* init device */
      writeb(TERM12ENABLE,private.miobase+STAT_CTRL_OFFSET);
      writeb(UNINITINT,private.miobase+INT_VECT_OFFSET);
     
      for (i=0;i<IPQUAD_MAX_CHANNELS;i++){
	
	printk(KERN_ALERT "%s: initializing channel %d\n", IPQUAD_NAME,i);
	
	channel_base = (unsigned int)private.miobase+4*i+CHAN_STAT_CTRL_OFFSET;
	writeb(OUT_CTRL,channel_base);
	writeb(QUAD,channel_base);
	writeb(IN_CTRL,channel_base);
	
	writeb(XY_INVERT,private.miobase+4*1+CHAN_CONF_OFFSET);
	
	/* FIXME add hard/soft count stuff */
  
#ifdef IPQUAD_DEBUG
	printk("%s: 0x%x\n", IPQUAD_NAME, ipquad_read_channel(i));
#endif
	
      }
      
      printk(KERN_ALERT "%s: IPQuad initialized\n", IPQUAD_NAME);
      
      
    } else {
      
      printk(KERN_ALERT "%s: device not found\n", IPQUAD_NAME);
      err = -ENODEV;
    
    }
    
    
    
  } else {
    
    printk(KERN_ALERT "%s: could not ioremap() iobase=0x%8x ",
	   IPQUAD_NAME,private.iobase);
    err = -ENODEV;
  }

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)      
  /* get major number for device */
  private.major = register_chrdev(IPQUAD_MAJOR, "ipquad", &ipquad_fops);
  if (private.major < 0) {
    printk(KERN_WARNING "%s: can not get major number\n", IPQUAD_NAME);
    err = -ENODEV;
    
  } else {
    
    /* devfs */
    for (i=0;i<IPQUAD_MAX_CHANNELS;i++)
      devfs_mk_cdev(MKDEV(private.major,i),
		    S_IFCHR | S_IRUGO | S_IWUGO,
		    "ipquad/channel%i",i);
    
    printk("%s: channels available through /dev/ipquad/channelx\n", IPQUAD_NAME);
  }
#else
  private.devdir = devfs_mk_dir(NULL,"ipquad",NULL);
  
  for(i=0;i<IPQUAD_MAX_CHANNELS;i++) {
    sprintf(devname,"channel%d",i);
    private.dev[i] = devfs_register(private.devdir,devname,
				    DEVFS_FL_AUTO_DEVNUM,
				    0, 0, S_IFCHR | S_IRUGO | S_IWUGO,
				    &ipquad_fops, NULL);
  }
#endif

  /* init mutexes */
  rt_typed_sem_init(&private.sem,1,BIN_SEM);
  //rt_typed_sem_init(&private.sem,1,RES_SEM);
  
  
  /* proc */
  create_proc_read_entry("ipquad",0,NULL,ipquad_proc_read,NULL);
  
  
  if (err) {
    release_mem_region(private.iobase,IPQUAD_MEM_LEN);
    if(private.miobase) iounmap((void *)private.miobase);
  }
  
  

  return err;
}


static void ipquad_exit(void)
{
  
  int channel;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)        
  unregister_chrdev(private.major, "ipquad");
  
  
  for (channel=0;channel<IPQUAD_MAX_CHANNELS;channel++)
    devfs_remove("ipquad/channel%i",channel);
  
  devfs_remove("ipquad");
#else
  for (channel=0;channel<IPQUAD_MAX_CHANNELS;channel++)
    devfs_unregister(private.dev[channel]);
  
  devfs_unregister(private.devdir);
  
#endif  
  
  remove_proc_entry("ipquad",NULL);

  
  iounmap((void *)private.miobase);
  release_mem_region(private.iobase,IPQUAD_MEM_LEN);
  
  rt_sem_delete(&private.sem);

  printk(KERN_ALERT "%s: driver exit\n",IPQUAD_NAME);
  
}


module_init(ipquad_init);
module_exit(ipquad_exit);


int ipquad_open(struct inode *inode, struct file *filp)
{
  
  return 0;
}


int ipquad_release(struct inode *inode, struct file *filp)
{

  return 0;

}


int ipquad_read(struct file *filp, char *buf, size_t count,
		loff_t *f_pos)
{
  
  int result;
  int channel;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)        
  channel =  iminor(filp->f_dentry->d_inode);
#else
  channel = MINOR(filp->f_dentry->d_inode->i_rdev);
#endif

  /* we return 4B long values */
  if(count < 4) return -1;
  
  result = ipquad_read_channel(channel);
  
  if (copy_to_user(buf,(char *)&result,4))
    return -EFAULT;
  
  printk("ipquad channel %d: %x\n",channel,result);
  return 0;

}

int ipquad_read_channel(int channel)
{

  int result = 0;
  unsigned char *ptr;
  
  if ((channel<0) || (channel>IPQUAD_MAX_CHANNELS)) return -ENODEV;
  
  rt_sem_wait(&private.sem);
  
  ptr = (unsigned char *) &result;
  
  writeb(READ_OL,private.miobase+4*channel+CHAN_STAT_CTRL_OFFSET);
  
  *(ptr+3) = readb(private.miobase+4*channel+OLPR_COMP_OFFSET);
  *(ptr+2) = readb(private.miobase+4*channel+OLPR_COMP_OFFSET);
  *(ptr+1) = readb(private.miobase+4*channel+OLPR_COMP_OFFSET);
  *(ptr)   = 0;
  
  rt_sem_signal(&private.sem);
  
  return result;

}


int ipquad_proc_read(char *buf,char **start, off_t offset,
		       int count, int *eof, void *data)
{
  int len=0;
  int i;
  
  len += sprintf(buf+len,"Linux Robox IPQUAD:\n");
  for(i=0;i<IPQUAD_MAX_CHANNELS;i++)
    len += sprintf(buf+len," channel%d: %d\n",
		   i,ipquad_read_channel(i));
  
  *eof = 1;
  return len;
}     

