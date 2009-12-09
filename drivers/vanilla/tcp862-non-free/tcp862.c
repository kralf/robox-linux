/* $Header: /Tews/Device Driver/Linux/TDRV001/Code/tdrv001.c 8     21.02.05 9:08 Hesse $ */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @   T D R V 0 0 1     @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          Linux - TDRV001 Driver                                **
**                                                                           **
**    File             tdrv001.c                                             **
**                                                                           **
**    Function         Driver source for TEWS TPMC862/362, TCP862 module     **
**                                                                           **
**                                                                           **
**    Owner            TEWS TECHNOLOGIES                                     **
**                     Am Bahnhof 7                                          **
**                     D-25469 Halstenbek                                    **
**                     Germany                                               **
**                                                                           **
**                                                                           **
**                     Tel.: +49(0)4101/4058-0                               **
**                     Fax.: +49(0)4101/4058-19                              **
**                     e-mail: support@tews.com                              **
**                                                                           **
**                                                                           **
**                     Copyright (c) 2004-2005                               **
**                     TEWS TECHNOLOGIES GmbH                                **
**                                                                           **
**                                                                           **
**                                                                           **
**                                                                           **
**                                                                           **
*******************************************************************************
*******************************************************************************/

#ifndef __KERNEL__
#define __KERNEL__
#endif


#undef TDRV001_DEBUG_VIEW
#undef TDRV001_DEBUG_INTR

#define DEBUG_NAME          "tcp862:"

#define DRIVER_NAME         "TCP862 - 4 Channel Serial IP"
#define DRIVER_VERSION      "2.0.0"
#define DRIVER_BIN_VERSION  0x20000
#define DRIVER_REVDATE      "2005-02-18"

#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/kmod.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/signal.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/sysrq.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/bitops.h>
#include <asm/byteorder.h>
#include <asm/delay.h>
#include <asm/semaphore.h>
#include <asm/uaccess.h>


#include "tcp862.h"
#include "tcp862def.h"
#include "pef20534.h"


#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE !FALSE
#endif

// Character devices
#define TCP862_DEVCLASS "tcp862"
#define TCP862_DEVNAME "ttySTCP862"

#if defined(MODULE_DESCRIPTION)
MODULE_DESCRIPTION("TCP862 - 4 Channel Serial IP");
#endif
#if defined(MODULE_AUTHOR)
MODULE_AUTHOR("TEWS Technologies <support@tews.com>");
#endif
#if defined(MODULE_LICENSE)
MODULE_LICENSE("GPL");
#endif



/*****************************************************************************
definitions of configurable module parameters
*****************************************************************************/
static unsigned long rx_timeout = TP862_DEFAULT_RX_TIMEOUT;
module_param(rx_timeout, ulong, 1);
MODULE_PARM_DESC(rx_timeout, "receive timeout");

/*****************************************************************************
definitions of device access functions
*****************************************************************************/
static int      tp862_open  (struct inode *inode, struct file *filp);
static int      tp862_close (struct inode *inode, struct file *filp);
static int      tp862_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
static ssize_t  tp862_write (struct file *filp, const char *buff, size_t count, loff_t *offp);
static ssize_t  tp862_read  (struct file *filp, char *buff, size_t count, loff_t *offp);

/*****************************************************************************
definitions of our own helper functions
*****************************************************************************/
void            set_cpld_clocksource            ( TP862_CCB* pCCB, unsigned long option );
void            set_cpld_transceivermode        ( TP862_CCB* pCCB, TRANSCEIVER_MODE mode );
void            set_cpld_dcedte                 ( TP862_CCB* pCCB, DCEDTE dce_dte  );
int             set_osc_source                  ( TP862_CCB* pCCB, OSC_SOURCE option );
void            channel_reset_rxtx              ( TP862_CCB* pCCB );
void            channel_init_rxtx               ( TP862_CCB* pCCB );
void            channel_reset                   ( TP862_CCB* pCCB );
int             init_hardware                   ( TP862_DCB* pDCB );
int             evaluate_buildoption_register   ( TP862_DCB* pDCB );
int             setup_channel_tx_descriptor_list( TP862_CCB* pCCB );
int             setup_channel_rx_descriptor_list( TP862_CCB* pCCB );
int             tp862_reset_dscc4               ( TP862_DCB* pDCB );
int             set_operation_mode              ( TP862_CCB* pCCB, TP862_OPERATION_MODE_STRUCT* pOperationMode );
int             set_baudrate                    ( TP862_CCB* pCCB, unsigned long bps );
int             remove_descriptor               ( TP862_CCB* pCCB, unsigned long rempos );
void            timeout_function                ( unsigned long data );
void            start_receiver                  ( TP862_CCB* pCCB );
void            stop_receiver                   ( TP862_CCB* pCCB );
static void     cleanup_device                  ( TP862_DCB* pDCB );
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
void            timeout_remove                  ( void* data );
#else
void            timeout_remove                  ( struct work_struct* ws );
#endif

/*****************************************************************************
definitions of global module varialbes
*****************************************************************************/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
static struct class* tcp862_class;
#endif

/*****************************************************************************
File operations supported by the TPMC862 driver
*****************************************************************************/
struct file_operations tp862_fops = {

    owner:      THIS_MODULE,
    write:      tp862_write,
    ioctl:      tp862_ioctl,
    open:       tp862_open,
    release:    tp862_close,
    read:       tp862_read,
};




/*****************************************************************************
some global variable definitions used for device management
*****************************************************************************/
static struct list_head tp862_board_root;
static int              modules_found = 0;
static int              tcp862_major = TDRV001_MAJOR;
static int              minor_count = 0;



/******************************************************************************
** interrupt handling and misc for different Kernel versions ...
**
******************************************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
#define TP_IRQ_RETURN_T				void
#undef  IRQ_NONE
#define IRQ_NONE (0)
#undef  IRQ_HANDLED
#define IRQ_HANDLED (1)
#define TP_IRQ_RETURN(x)      return
#else
#define TP_IRQ_RETURN_T       irqreturn_t
#define TP_IRQ_RETURN(x)      return x
#endif

#define TP_MOD_INC_USE_COUNT		MOD_INC_USE_COUNT
#define TP_MOD_DEC_USE_COUNT		MOD_DEC_USE_COUNT

#else

#define TP_IRQ_RETURN_T				irqreturn_t
#define TP_IRQ_RETURN(x)			return x;

#define TP_MOD_INC_USE_COUNT
#define TP_MOD_DEC_USE_COUNT

#endif


/**************************************************************************
  I/O space read and write functions (Windows NT style function).
  The READ/WRITE_REGISTER_* calls manipulate I/O registers in MEMORY space.

  Note. Linux expect that the PCI bus is inherently Little-Endian. Therefor
        all PCI access functions swaps on PowerPC systems and don't swap on
        Intel x86 systems.
        Unfortunately the CAN controller registers are Big-Endian (usualy
        all TEWS PMC module register spaces are Big-Endian) and therefor
        we have to swap 16 and 32 bit values.

***************************************************************************/

static unsigned char READ_REGISTER_UCHAR(void *pReg)
{
    return readb(pReg);
}


static void WRITE_REGISTER_UCHAR(void *pReg, unsigned char value)
{
    writeb(value, pReg);
}


/*---- ULONG ----*/
static unsigned long READ_REGISTER_ULONG(void *pReg)
{
    return readl( pReg );
}

static void WRITE_REGISTER_ULONG(void *pReg, unsigned long value)
{
    writel( value, pReg );
}



/*
**---------------------------------------------------------------------
** Open and close
** ---------------------------------------------------------------------
*/
static int tp862_open (struct inode *inode, struct file *filp)
{
#ifndef CONFIG_DEVFS_FS
    int                 minor = MINOR(inode->i_rdev);
    int                 channel;
    TP862_DCB*          pDCB;
    struct list_head    *ptr;
#endif /* CONFIG_DEVFS_FS */
    TP862_CCB*          pCCB = (TP862_CCB*)filp->private_data;


#ifndef CONFIG_DEVFS_FS
    /* check range of minor device number */
    if (minor >= minor_count) return -ENODEV;

    /*
    ** Loop through the list of minor devices to get a pointer to the
    ** requested device.
    */
    list_for_each(ptr, &tp862_board_root) {
        /*
        **  map the list_head structure pointer back into a pointer
        **  to structure that contains it
        */
        pDCB = list_entry(ptr, TP862_DCB, node);
        /* search for specified device's control block */
        for (channel=0; channel<TP862_NUM_CHANS; channel++)
        {
            pCCB = pDCB->pCCB[channel];
            if ( pCCB->minor == minor) break;
        }
        if ( pCCB->minor == minor) break;
    }

    /* be sure that this minor device is the requested */
    if (pCCB->minor != minor) return -ENODEV;

    /* and use filp->private_data to point to the device data */
    filp->private_data = pCCB;

#endif /* CONFIG_DEVFS_FS */

    if (!pCCB)
    {
        printk(KERN_WARNING "%s Open Error! Device not available!\n", DEBUG_NAME);
        return -ENODEV;
    }

    if (pCCB->OpenCount == 0)
    {
        pCCB->timer.expires = jiffies + HZ; /* start the timeout check in 1 second  */
        add_timer( &pCCB->timer );
    }

    pCCB->OpenCount++;
    TP_MOD_INC_USE_COUNT;
    /*printk("%s open successful (%d,%d)\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);*/
    return 0;          /* success */
}


static int tp862_close (struct inode *inode, struct file *filp)
{
#ifndef CONFIG_DEVFS_FS
    int  minor = MINOR(inode->i_rdev);
#endif /* CONFIG_DEVFS_FS */
    TP862_CCB* pCCB = (TP862_CCB*)filp->private_data;

#ifndef CONFIG_DEVFS_FS

    /* check range of minor device number */
    if (minor >= minor_count) return -ENODEV;

#endif /* CONFIG_DEVFS_FS */

    if (!pCCB)
    {
        printk(KERN_WARNING "%s Close Error! Device not available!\n", DEBUG_NAME);
        return -ENODEV;
    }

    if (pCCB->OpenCount) pCCB->OpenCount--;
    TP_MOD_DEC_USE_COUNT;
    if (pCCB->OpenCount == 0)
    {
        del_timer_sync( &pCCB->timer );
    }

    /*printk("%s close successful (%d,%d)\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);*/
    return 0;          /* success */
}




static ssize_t tp862_read (struct file *filp, char *buff, size_t count, loff_t *offp)
{
    TP862_CCB* pCCB = (TP862_CCB*)filp->private_data;
    int nbytes = 0;
    int result;
    long remaining = 0;
    unsigned char* pRead;
    long  timeout;
    wait_queue_t  wait;

    printk("tp862_read()\n");
    timeout = pCCB->RxTimeout;

    if (timeout)
    {
        /*printk("%s tp862_read(%d,%d): timeout=%ld\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, timeout);*/
        if (!pCCB->pInternalRingBuffer->entry[pCCB->pInternalRingBuffer->get_idx].Valid)
        {
            /*
            ** go to sleep until new data arrives (or timeout)
            */
            init_waitqueue_entry(&wait, current);
            add_wait_queue(&pCCB->rx_waitqueue, &wait);

            while(1) {
              set_current_state(TASK_INTERRUPTIBLE);

              if (pCCB->pInternalRingBuffer->entry[pCCB->pInternalRingBuffer->get_idx].Valid) break;

              if (timeout) {
                timeout = schedule_timeout(timeout);
                if (timeout == 0) {
                  result = -ETIME;    /* value is not returned because it is no error here.*/
                  break;
                }
              }
              else {
                schedule();
              }
              /* check for received signals */
              if (signal_pending(current)) {
                result = -ERESTARTSYS;
                break;
              }
            }
            set_current_state(TASK_RUNNING);
            remove_wait_queue(&pCCB->rx_waitqueue, &wait);
        }
    } else {
        /*printk("tp862_read: non-blocking IO\n");*/
    }



    if (pCCB->pInternalRingBuffer->entry[pCCB->pInternalRingBuffer->get_idx].Valid)
    {

        /* update remaining bytes*/
        remaining = pCCB->pInternalRingBuffer->entry[pCCB->pInternalRingBuffer->get_idx].NumberOfBytes - pCCB->ReadOffset;
        /* set read-pointer*/
        pRead = &pCCB->pInternalRingBuffer->entry[pCCB->pInternalRingBuffer->get_idx].pData[pCCB->ReadOffset];

        nbytes = remaining;
        if (nbytes > count)
        {
            nbytes = count;
        }
        remaining -= nbytes;

        /* copy the current read buffer from kernel to user space */
        if (copy_to_user(buff, pRead, nbytes)) {
            printk(KERN_WARNING "\n%s **tp862_read: Unable to copy necessary data for READ to user!**\n", DEBUG_NAME);
            return -EFAULT;
        }

        printk("tp862_read: %dB '%p'\n",nbytes,buff);

        if (remaining > 0)
        {
            pCCB->ReadOffset += nbytes;
        }

        /*
        ** jump to the next receive descriptor only if all data has been read.
        */
        if (remaining <= 0)
        {
            pCCB->ReadOffset = 0;
            pCCB->pInternalRingBuffer->entry[pCCB->pInternalRingBuffer->get_idx].Valid = FALSE;
            pCCB->pInternalRingBuffer->get_idx = (pCCB->pInternalRingBuffer->get_idx + 1) % pCCB->rx_data_queue_size;
            if (pCCB->pInternalRingBuffer->buffer_overrun)
            {
                pCCB->pInternalRingBuffer->buffer_overrun = FALSE;
                /* start receiver*/
                start_receiver( pCCB );
            }
        }
    } else {
        /*
        ** no need to copy anything because there is no data available
        */
        nbytes = 0;
    }
    return nbytes;
}



static ssize_t tp862_write (struct file *filp, const char *buff, size_t count, loff_t *offp)
{
    TP862_CCB* pCCB = (TP862_CCB*)filp->private_data;
    TP862_TxFD*     pTxFD;
    unsigned char* pBuffer;
    dma_addr_t   Buffer_dma;
    unsigned long nbytes;
    unsigned long id;
    int result;
    long  timeout;
    wait_queue_t  wait;


    nbytes = count + (count % 4);   /* multiple of ULONG*/

    pBuffer = (unsigned char*)pci_alloc_consistent( pCCB->dev, nbytes+1, &Buffer_dma );
    if (!pBuffer)
    {
        printk(KERN_WARNING "%s tp862_write(%d,%d): Error getting memory!!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
        return -ENOMEM;
    }

    /* copy the write buffer from user to kernel space */
    if (copy_from_user(pBuffer, buff, count))
    {
        printk(KERN_WARNING "%s tp862_write(%d,%d): Error copying from userspace!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
        return -EFAULT;
    }

    timeout = pCCB->TxTimeout;

    if (pCCB->tx_free < 2)
    {
        printk("tp862_write: we have to wait for a descriptor\n");
        /*
        ** let's go to sleep until a descriptor is free for us
        */
        init_waitqueue_entry(&wait, current);
        add_wait_queue(&pCCB->tx_waitqueue, &wait);

        while(1) {
          set_current_state(TASK_INTERRUPTIBLE);

          if (pCCB->tx_free > 1) break;

          if (timeout) {
            timeout = schedule_timeout(timeout);
            if (timeout == 0) {
              result = -ETIME;
              break;
            }
          }
          else {
            schedule();
          }

          /* check for received signals */
          if (signal_pending(current)) {
            result = -ERESTARTSYS;
            break;
          }
        }

        set_current_state(TASK_RUNNING);
        remove_wait_queue(&pCCB->tx_waitqueue, &wait);

        if ( pCCB->tx_free < 2 )
        {
            /*
            ** no free descriptor after specified timeout !!
            */
            printk(KERN_INFO "%s tp862_write(%d,%d): No descriptor available after timeout, we are busy!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
            /*
            ** update error-count
            */
            pCCB->tx_count_error++;

            /* free allocated memory*/
            pci_free_consistent(pCCB->dev, nbytes+1, pBuffer, Buffer_dma);
            return -EBUSY;
        } else {
            printk("tp862_write: Ok, let's go. Time remaining = %ld\n", timeout);
        }

    }



    //pBuffer[count] = '\0';
    printk("tp862_write: %dB '%p'\n", count, buff);

    /*
    ** get pointer to current insert-descriptor
    */
    pTxFD = &pCCB->tx_fd[pCCB->tx_insert];
    id = pCCB->tx_insert;

    /*
    ** insert data buffer into descriptor list
    */
    pTxFD->data = __le32_to_cpu( (unsigned long)Buffer_dma );
    pTxFD->txbuf_virt = (unsigned long)pBuffer;
    pTxFD->txbuf_dma = (unsigned long)Buffer_dma;
    pTxFD->size = (unsigned long)(nbytes+1);        /* used to free the buffer*/
    pTxFD->finished = FALSE;

    /*
    ** update descriptor information
    */
    pTxFD->state = __le32_to_cpu( ((count << 16) | FrameEnd | Hold) );
    pTxFD->complete = 0xffffffff;

    /*
    ** update queue management
    */
    pCCB->tx_free--;
    pCCB->tx_insert = (pCCB->tx_insert + 1) % TX_DATA_QUEUE_SIZE;

    /*
    ** set LAST Tx pointer and start the dma transfer
    */
    pCCB->tx_fd_last_dma = (dma_addr_t)__cpu_to_le32( pTxFD->next );;
    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->dscc4_space + CH0LTDA + 4*pCCB->ChannelNumber), __cpu_to_le32( pTxFD->next ) );

    if (pCCB->TxTimeRemaining == -1)
    {
        pCCB->TxTimeRemaining = pCCB->DeviceWriteTimeout;
    }

    printk("tp862_write: count=%d\n",count);
    return count;
}




static int tp862_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
    TP862_CCB* pCCB = NULL;
    TP862_OPERATION_MODE_STRUCT* pOperationMode;
    TP862_OPERATION_MODE_STRUCT OperationMode;
    TP862_REGISTER_STRUCT RegStruct;
    TP862_REGISTER_STRUCT* pRegStruct;
    unsigned long* pUlValue;
    unsigned long baudrate, xtalExt, ReceiverState, ulValue;
    OSC_SOURCE oscSource;
    int channel, result, i;

    if (filp == NULL)
    {
        printk(KERN_WARNING "%s IOCTL: argument filp is NULL!\n", DEBUG_NAME);
        return -EINVAL;
    }
    if (filp->private_data == NULL)
    {
        printk(KERN_WARNING "%s IOCTL: argument filp->private_data is NULL!\n", DEBUG_NAME);
        return -EINVAL;
    }

    pCCB = filp->private_data;


    printk("%s IOCTL called (0x%X)\n", DEBUG_NAME, cmd);

    /*
    * extract the type and number bitfields, and don't decode
    * wrong cmds: return EINVAL before verify_area()
    */
    if (_IOC_TYPE(cmd) != TP862_IOC_MAGIC) return -EINVAL;


    switch (cmd)
    {

    case TP862_IOCQ_GET_TX_COUNT_ERROR:

        pUlValue = (unsigned long*)arg;
        if (copy_to_user(pUlValue, &pCCB->tx_count_error, sizeof(unsigned long))) {
            printk(KERN_WARNING "%s Unable to copy necessary data for get_tx_count_error to user!\n", DEBUG_NAME);
            return -EFAULT;
        }
        pCCB->tx_count_error = 0;
        break;
    case TP862_IOCQ_GET_TX_COUNT_OK:
        pUlValue = (unsigned long*)arg;
        if (copy_to_user(pUlValue, &pCCB->tx_count_ok, sizeof(unsigned long))) {
            printk(KERN_WARNING "%s Unable to copy necessary data for get_tx_count_ok to user!\n", DEBUG_NAME);
            return -EFAULT;
        }
        pCCB->tx_count_ok = 0;
        break;
    case TP862_IOC_DSCC4_RESET:
        tp862_reset_dscc4( pCCB->pDCB );
        /* configure module-hardware parameters */
        for (channel=0; channel<TP862_NUM_CHANS; channel++)
        {
            pCCB->pDCB->pCCB[channel]->is_in_init_state = FALSE;
            pCCB->pDCB->pCCB[channel]->is_in_reset_state = TRUE;
        }
        init_hardware( pCCB->pDCB );
        /*printk(KERN_INFO "%s reset done for Board #%d\n", DEBUG_NAME, pCCB->BoardNumber);*/
        break;

    case TP862_IOCS_SET_OPERATION_MODE:
        pOperationMode = (TP862_OPERATION_MODE_STRUCT*)arg;
        /* copy data from user to kernel space */
        if (copy_from_user(&OperationMode, pOperationMode, sizeof(TP862_OPERATION_MODE_STRUCT))) {
            printk(KERN_WARNING "%s Unable to copy necessary data for SetOperationMode from user!\n", DEBUG_NAME);
            return -EFAULT;
        }
        result = set_operation_mode( pCCB, &OperationMode );
        /*
        ** evaluate return value
        */
        switch (result)
        {
        case 0:
            /* everything fine */
            break;
        case TP862_ERROR_COMMTYPE:
            printk(KERN_WARNING "%s TP862_IOCS_SET_OPERATION_MODE(%d,%d): COMMTYPE error.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
            result = -EINVAL;
            break;
        case TP862_ERROR_CLOCKMODE:
            printk(KERN_WARNING "%s TP862_IOCS_SET_OPERATION_MODE(%d,%d): CLOCKMODE error.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
            result = -EINVAL;
            break;
        case TP862_ERROR_TXCLOCK_OUT:
            printk(KERN_WARNING "%s TP862_IOCS_SET_OPERATION_MODE(%d,%d): TXCLOCK_OUT error.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
            result = -EINVAL;
            break;
        case TP862_ERROR_TRANSCEIVER:
            printk(KERN_WARNING "%s TP862_IOCS_SET_OPERATION_MODE(%d,%d): TRANSCEIVER error.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
            result = -EINVAL;
            break;
        case TP862_ERROR_DCEDTE:
            printk(KERN_WARNING "%s TP862_IOCS_SET_OPERATION_MODE(%d,%d): DCEDTE error.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
            result = -EINVAL;
            break;
        case TP862_ERROR_OVERSAMPLING:
            printk(KERN_WARNING "%s TP862_IOCS_SET_OPERATION_MODE(%d,%d): OVERSAMPLING error.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
            result = -EINVAL;
            break;
        case TP862_ERROR_USETERMCHAR:
            printk(KERN_WARNING "%s TP862_IOCS_SET_OPERATION_MODE(%d,%d): USETERMCHAR error.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
            result = -EINVAL;
            break;
        case TP862_ERROR_BAUDRATE:
            printk(KERN_WARNING "%s TP862_IOCS_SET_OPERATION_MODE(%d,%d): BAUDRATE error.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
            result = -EINVAL;
            break;
        default:
            /* unknown return value. should never happen. */
            printk(KERN_WARNING "%s TP862_IOCS_SET_OPERATION_MODE(%d,%d): unknown error.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
            result = -EINVAL;
            break;
        }
        return result;
        break;
    case TP862_IOCT_SET_BAUDRATE:
        baudrate = (unsigned long)arg;
        result = set_baudrate( pCCB, baudrate );
        if (result > 0)
        {
            /* error setting baudrate */
            printk(KERN_WARNING "%s TP862_IOCT_SET_BAUDRATE(%d,%d): Error setting baudrate!!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
            return -EINVAL;
        } else {
            /* store new baudrate in OperationMode structure */
            pCCB->OperationMode.baudrate = baudrate;
        }
        break;
    case TP862_IOCG_GET_OPERATION_MODE:
        pOperationMode = (TP862_OPERATION_MODE_STRUCT*)arg;
        if (copy_to_user(pOperationMode, &pCCB->OperationMode, sizeof(TP862_OPERATION_MODE_STRUCT))) {
            printk(KERN_WARNING "%s Unable to copy necessary data for GetOperationMode to user!\n", DEBUG_NAME);
            return -EFAULT;
        }
        break;

    case TP862_IOCT_SET_RECEIVER_STATE:
        ReceiverState = (unsigned long)arg;
        switch (ReceiverState)
        {
        case 0:
            /* switch off receiver*/
#ifdef TDRV001_DEBUG_VIEW
            printk(KERN_INFO "%s Receiver(%d,%d) switched off.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
#endif
            stop_receiver( pCCB );
            break;
        case 1:
            /* switch on receiver*/
#ifdef TDRV001_DEBUG_VIEW
            printk(KERN_INFO "%s Receiver(%d,%d) switched on.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
#endif
            start_receiver( pCCB );
            break;
        default:
            return -EINVAL;
            break;
        }
        break;
    case TP862_IOC_CLEAR_RX_BUFFER:
        /* stop receiver*/
        stop_receiver( pCCB );
        for (i=0; i<RX_DATA_QUEUE_SIZE; i++)
        {
            memset(pCCB->pInternalRingBuffer->entry[i].pData, 0, TP862_RX_BUFFER_SIZE);
            pCCB->pInternalRingBuffer->entry[i].Valid = FALSE;
            pCCB->pInternalRingBuffer->entry[i].NumberOfBytes = 0;
        }
        pCCB->pInternalRingBuffer->get_idx = pCCB->pInternalRingBuffer->put_idx;
        pCCB->pInternalRingBuffer->buffer_overrun = FALSE;
#ifdef TDRV001_DEBUG_VIEW
        printk(KERN_INFO "%s clearing rx buffer(%d,%d) done.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
#endif
        /* start receiver*/
        start_receiver( pCCB );
        break;

    case TP862_IOCT_SET_EXT_XTAL:
        xtalExt = (unsigned long)arg;
#ifdef TDRV001_DEBUG_VIEW
        printk(KERN_INFO "%s TP862_IOCT_SET_EXT_XTAL(%d,%d): xtal=%ld\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, xtalExt);
#endif
        pCCB->xtalExt_hz = xtalExt;
        break;

    case TP862_IOCT_SET_READ_TIMEOUT:
        ulValue = (unsigned long)arg;
        pCCB->RxTimeout = ulValue;
#ifdef TDRV001_DEBUG_VIEW
        printk(KERN_INFO "%s Read-Timeout(%d,%d) set to %ld.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, pCCB->RxTimeout);
#endif
        break;
    /*
    ** direct register read/write functions
    */
    case TP862_IOCS_SCC_REG_WRITE:
        pRegStruct = (TP862_REGISTER_STRUCT*)arg;
        /* copy data from user to kernel space */
        if (copy_from_user(&RegStruct, pRegStruct, sizeof(TP862_REGISTER_STRUCT))) {
            printk(KERN_WARNING "%s Unable to copy necessary data for TP862_IOCS_SCC_REG_WRITE from user!\n", DEBUG_NAME);
            return -EFAULT;
        }
        /* perform register range check*/
        if (RegStruct.offset > SCC_MAX_REG)
        {
            printk(KERN_WARNING "%s TP862_IOCS_SCC_REG_WRITE(%d,%d): offset too big! (offset=0x%lX max=0x%X)\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, RegStruct.offset, CPLD_MAX_REG);
            return -EINVAL;
        }
        pCCB->scc_regs_local[ RegStruct.offset >> 2 ] = RegStruct.value;
        WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->scc_regstart + RegStruct.offset), RegStruct.value );
        break;
    case TP862_IOCG_SCC_REG_READ:
        pRegStruct = (TP862_REGISTER_STRUCT*)arg;
        if (copy_from_user(&RegStruct, pRegStruct, sizeof(TP862_REGISTER_STRUCT))) {
            printk(KERN_WARNING "%s Unable to copy necessary data for TP862_IOCG_SCC_REG_READ from user!\n", DEBUG_NAME);
            return -EFAULT;
        }
        /* perform register range check*/
        if (RegStruct.offset > SCC_MAX_REG)
        {
            printk(KERN_WARNING "%s TP862_IOCG_SCC_REG_READ(%d,%d): offset too big! (offset=0x%lX max=0x%X)\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, RegStruct.offset, CPLD_MAX_REG);
            return -EINVAL;
        }
        RegStruct.value = READ_REGISTER_ULONG( (unsigned long*)(pCCB->scc_regstart + RegStruct.offset) );
        if (copy_to_user(pRegStruct, &RegStruct, sizeof(TP862_REGISTER_STRUCT))) {
            printk(KERN_WARNING "%s Unable to copy necessary data for TP862_IOCG_SCC_REG_READ to user!\n", DEBUG_NAME);
            return -EFAULT;
        }
        break;
    case TP862_IOCS_GLOB_REG_WRITE:
        pRegStruct = (TP862_REGISTER_STRUCT*)arg;
        /* copy data from user to kernel space */
        if (copy_from_user(&RegStruct, pRegStruct, sizeof(TP862_REGISTER_STRUCT))) {
            printk(KERN_WARNING "%s Unable to copy necessary data for TP862_IOCS_GLOB_REG_WRITE from user!\n", DEBUG_NAME);
            return -EFAULT;
        }
        /* perform register range check*/
        if (RegStruct.offset > GLOB_MAX_REG)
        {
            printk(KERN_WARNING "%s TP862_IOCS_GLOB_REG_WRITE(%d,%d): offset too big! (offset=0x%lX max=0x%X)\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, RegStruct.offset, CPLD_MAX_REG);
            return -EINVAL;
        }
        WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->dscc4_space + RegStruct.offset), RegStruct.value );
        break;
    case TP862_IOCG_GLOB_REG_READ:
        pRegStruct = (TP862_REGISTER_STRUCT*)arg;
        /* copy data from user to kernel space */
        if (copy_from_user(&RegStruct, pRegStruct, sizeof(TP862_REGISTER_STRUCT))) {
            printk(KERN_WARNING "%s Unable to copy necessary data for TP862_IOCG_GLOB_REG_READ from user!\n", DEBUG_NAME);
            return -EFAULT;
        }
        /* perform register range check*/
        if (RegStruct.offset > GLOB_MAX_REG)
        {
            printk(KERN_WARNING "%s TP862_IOCG_GLOB_REG_READ(%d,%d): offset too big! (offset=0x%lX max=0x%X)\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, RegStruct.offset, CPLD_MAX_REG);
            return -EINVAL;
        }
        RegStruct.value = READ_REGISTER_ULONG( (unsigned long*)(pCCB->dscc4_space + RegStruct.offset) );
        if (copy_to_user(pRegStruct, &RegStruct, sizeof(TP862_REGISTER_STRUCT))) {
            printk(KERN_WARNING "%s Unable to copy necessary data for TP862_IOCG_GLOB_REG_READ to user!\n", DEBUG_NAME);
            return -EFAULT;
        }
        break;
    case TP862_IOCS_CPLD_REG_WRITE:
        pRegStruct = (TP862_REGISTER_STRUCT*)arg;
        /* copy data from user to kernel space */
        if (copy_from_user(&RegStruct, pRegStruct, sizeof(TP862_REGISTER_STRUCT))) {
            printk(KERN_WARNING "%s Unable to copy necessary data for TP862_IOCS_CPLD_REG_WRITE from user!\n", DEBUG_NAME);
            return -EFAULT;
        }
        /* perform register range check*/
        if (RegStruct.offset > CPLD_MAX_REG)
        {
            printk(KERN_WARNING "%s TP862_IOCS_CPLD_REG_WRITE(%d,%d): offset too big! (offset=0x%lX max=0x%X)\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, RegStruct.offset, CPLD_MAX_REG);
            return -EINVAL;
        }
        WRITE_REGISTER_UCHAR( (unsigned char*)(pCCB->cpld_space + RegStruct.offset), (unsigned char)RegStruct.value );
        break;
    case TP862_IOCG_CPLD_REG_READ:
        pRegStruct = (TP862_REGISTER_STRUCT*)arg;
        if (copy_from_user(&RegStruct, pRegStruct, sizeof(TP862_REGISTER_STRUCT))) {
            printk(KERN_WARNING "%s Unable to copy necessary data for TP862_IOCG_CPLD_REG_READ to user!\n", DEBUG_NAME);
            return -EFAULT;
        }
        /* perform register range check*/
        if (RegStruct.offset > CPLD_MAX_REG)
        {
            printk(KERN_WARNING "%s TP862_IOCG_CPLD_REG_READ(%d,%d): offset too big! (offset=0x%lX max=0x%X)\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, RegStruct.offset, CPLD_MAX_REG);
            return -EINVAL;
        }
        RegStruct.value = (unsigned long)READ_REGISTER_UCHAR( (unsigned char*)(pCCB->cpld_space + RegStruct.offset) );
        if (copy_to_user(pRegStruct, &RegStruct, sizeof(TP862_REGISTER_STRUCT))) {
            printk(KERN_WARNING "%s Unable to copy necessary data for TP862_IOCG_CPLD_REG_READ to user!\n", DEBUG_NAME);
            return -EFAULT;
        }
        break;

    case TP862_IOCT_SET_OSC_SRC:
        oscSource = (OSC_SOURCE)arg;
        switch (pCCB->Version)
        {
        case ID_VERSION_1_0:
            printk(KERN_WARNING "%s IOCTL_TP862_SET_OSC_SOURCE not supported for v1.0 !\n", DEBUG_NAME);
            return -EINVAL;
            break;
        case ID_VERSION_1_1:
            return set_osc_source( pCCB, oscSource );
            break;
        default:
            printk(KERN_WARNING "%s IOCTL_TP862_SET_OSC_SOURCE: Unknown version!\n", DEBUG_NAME);
            return -EINVAL;
            break;
        }
        break;

    default:
        printk(KERN_WARNING "%s unknown IOCTL!\n", DEBUG_NAME);
        break;
    }

    return 0;
}



void tp862_cfg_irq( TP862_DCB* pDCB )
{
    u32 state;
    unsigned long index;

    /*
    **  perform irq-index syncing
    **  ( search for a vector != 0 )
    */
    if (pDCB->cfg_irq_index_outofsync)
    {
        for (index=0; index<IRQ_RING_SIZE; index++)
        {
            state = __cpu_to_le32( pDCB->cfg_irq[index] );
            if (state != 0)
            {
                /* found a vector! we are sync'ed again!*/
#ifdef TDRV001_DEBUG_INTR
                printk(KERN_INFO "%s cfg_irq(%d): synchronized at index = %ld.\n", DEBUG_NAME, pDCB->BoardNumber, index);
#endif
                pDCB->cfg_irq_index_outofsync = FALSE;
                pDCB->cfg_irq_index = index;
                break;
            }
        }
    }

    /*
    ** check if a correct interrupt queue index was found.
    ** if not, don't serve this interrupt (it was not for this channel)
    */
    if (pDCB->cfg_irq_index_outofsync)
    {
        printk(KERN_WARNING "%s cfg_irq(%d): not synchronized yet. bad error.\n", DEBUG_NAME, pDCB->BoardNumber);
        pDCB->CfgIntOk = FALSE;
        return;
    }

    /* get current interrupt vector from queue */
    state = __cpu_to_le32( pDCB->cfg_irq[pDCB->cfg_irq_index] );

#ifdef TDRV001_DEBUG_INTR
    printk( KERN_INFO "%s cfg_irq(%d): vector = 0x%.8lX\n", DEBUG_NAME, pDCB->BoardNumber, (unsigned long)state );
#endif

    /* check if we have a cfg interrupt */
    if ((state & 0xa0000000) == 0xa0000000)
    {
        /* entry found, serve it */
        if ( state & Arf )
        {
            /* action request failure */
#ifdef TDRV001_DEBUG_INTR
            printk( KERN_INFO "%s cfg_irq(%d): Arf!\n", DEBUG_NAME, pDCB->BoardNumber);
#endif
        }
        if (state & ArAck)
        {
            /* action request acknowledge */
#ifdef TDRV001_DEBUG_INTR
            printk( KERN_INFO "%s cfg_irq(%d): Arack\n", DEBUG_NAME, pDCB->BoardNumber);
#endif

        }

        pDCB->cfg_irq[pDCB->cfg_irq_index] = 0x00000000;  /* mark vector as served */
        /* set index to next space */
        pDCB->cfg_irq_index = (pDCB->cfg_irq_index + 1) % IRQ_RING_SIZE;

        /* signal that the cfg int vector was ok (for startup check) */
        pDCB->CfgIntOk = TRUE;

    } else {
        /* error! no interrupt vector available, but interrupt received! */
        printk("%s cfg_irq(%d): no vector found! (state=0x%08lX)\n", DEBUG_NAME, pDCB->BoardNumber, (unsigned long)state);
        pDCB->CfgIntOk = FALSE;
    }

    return;
}

void ChannelIsrRx( TP862_CCB* pCCB )
{
    unsigned long sourceId;
    unsigned long state, index, state2;
    int served = FALSE;
    TP862_RxFD*  pRxFD;
#ifdef TDRV001_DEBUG_INTR
    unsigned char tmp[250];
    unsigned char* src;
#endif


    /* build sourceId for this channel */
    sourceId = (pCCB->ChannelNumber << 28);

    /*
    **  perform irq-index syncing
    **  ( search for a vector != 0 )
    */
    if (pCCB->rx_irq_index_outofsync)
    {
        for (index=0; index<IRQ_RING_SIZE; index++)
        {
            state = __cpu_to_le32( pCCB->rx_irq[index] );
            if (state != 0)
            {
                /* found a vector! we are sync'ed again!*/
#ifdef TDRV001_DEBUG_INTR
                printk(KERN_INFO "%s rx_irq(%d,%d): synchronized at index = %ld.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, index);
#endif
                pCCB->rx_irq_index_outofsync = FALSE;
                pCCB->rx_irq_index = index;
                break;
            }
        }
    }

    /*
    ** check if a correct interrupt queue index was found.
    ** if not, don't serve this interrupt (it was not for this channel)
    */
    if (pCCB->rx_irq_index_outofsync)
    {
        return;
    }

    /* get current interrupt vector from queue */
    state = __cpu_to_le32( pCCB->rx_irq[pCCB->rx_irq_index] );
    while (state != 0x00000000)
    {

        /* check if the received interrupt was for us */
        if ( ((state & sourceId) == sourceId) && (state != 0x00000000) )
        {
            /* there is a new vector for us. */
#ifdef TDRV001_DEBUG_INTR
            printk(KERN_INFO "%s rx_irq(%d,%d): vector = 0x%lx\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, state);
#endif
            if (state & SccInt)
            {
                /* we have an scc interrupt. serve it. */
                /*printk("rx_irq(%d): scc interrupt.", pCCB->ChannelNumber);*/
            } else {

                if (state & Err)
                {
                    /*printk("rx_irq(%d): Err!\n", pCCB->ChannelNumber);*/
                } else {

                    /* we have a dma interrupt. serve it. */
                    /*printk("rx_irq(%d): dma interrupt.\n", pCCB->ChannelNumber);*/

                    /*
                    ** no WHILE loop necessary because we are generating an interrupt
                    ** for each passed descriptor. Finally that is the sollution to all my receive-problems!!!
                    */
                    pRxFD = &pCCB->rx_fd[pCCB->rx_fd_index];
                    state2 = __cpu_to_le32( pRxFD->state2 );

#ifdef TDRV001_DEBUG_INTR
                    if ((state2 & FrameEnd) )
                    {
                        printk("%s rx_irq(%d,%d): Frame with Frame-End.(%ld)\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, pCCB->rx_fd_index);
                    } else {
                        printk("%s rx_irq(%d,%d): Frame without Frame-End.(%ld)\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, pCCB->rx_fd_index);
                    }
                    if ((state2 & DataComplete) )
                    {
                        printk("%s rx_irq(%d,%d): Frame with DataComplete. (%ld)\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, pCCB->rx_fd_index);
                    } else {
                        printk("%s rx_irq(%d,%d): Frame without DataComplete (%ld).\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, pCCB->rx_fd_index);
                    }

                    printk("%s rx_irq(%d,%d): length=%ld (%ld)\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, ((state2 & RxBNO) >> 16), pCCB->rx_fd_index);

                    src = (unsigned char*)pCCB->pInternalRingBuffer->entry[pCCB->rx_fd_index].pData;
                    for (index=0; index<((state2 & RxBNO) >> 16); index++)
                    {
                        tmp[index] = src[index];
                        if (tmp[index] == '\0') tmp[index]=' ';
                    }
                    tmp[((state2 & RxBNO) >> 16)] = '\0';
                    printk("%s rx_irq(%d,%d): data='%s'\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, tmp);
#endif

                    /*
                    ** update buffer information
                    */
                    pCCB->pInternalRingBuffer->entry[pCCB->rx_fd_index].NumberOfBytes = ((state2 & RxBNO) >> 16);
                    if ( (pCCB->OperationMode.commtype == COMMTYPE_HDLC) && (state2 & FrameEnd) )
                    {
                        if ( pCCB->discardCrcValue && (pCCB->pInternalRingBuffer->entry[pCCB->rx_fd_index].NumberOfBytes>1) )
                        {
                            pCCB->pInternalRingBuffer->entry[pCCB->rx_fd_index].NumberOfBytes--;
                        }
                    }
                    pCCB->pInternalRingBuffer->entry[pCCB->rx_fd_index].Valid = TRUE;
                    pCCB->pInternalRingBuffer->put_idx = (pCCB->rx_fd_index + 1) % pCCB->rx_data_queue_size;
                    /* re-init rx-descriptor for next usage*/
                    pRxFD->state1 = __le32_to_cpu( (RECEIVE_LENGTH_LIMIT << 16) | HiDesc );

                    /* signal that new data is present*/
                    wake_up( &pCCB->rx_waitqueue );

                    pRxFD->state2 = 0x00000000;
                    pCCB->rx_fd_index = (pCCB->rx_fd_index + 1) % pCCB->rx_data_queue_size;

                    /*
                    ** check if we had a buffer overrun
                    */
                    if ( pCCB->pInternalRingBuffer->entry[(pCCB->rx_fd_index + 1) % pCCB->rx_data_queue_size].Valid )
                    {
                        pCCB->pInternalRingBuffer->buffer_overrun = TRUE;
                        /* stop receiver*/
                        stop_receiver( pCCB );
#ifdef TDRV001_DEBUG_INTR
                        printk(KERN_INFO "%s rx_irq(%d,%d): internal buffer overrun!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
#endif
                    }
                    /*
                    ** no WHILE loop necessary because we are generating an interrupt
                    ** for each passed descriptor. Finally that is the sollution to all my receive-problems!!!
                    */
                }
            }

            /* mark vector as served */
            pCCB->rx_irq[pCCB->rx_irq_index] = 0x00000000;
            /* set index to next space */
            pCCB->rx_irq_index = (pCCB->rx_irq_index+1) % IRQ_RING_SIZE;

            served = TRUE;
        } else {
            /*printk("rx_irq(%d,%d): This vector was not for us.\n", pCCB->BoardNumber, pCCB->ChannelNumber);*/
        }
        /* get next interrupt vector from queue (look forward) */
        state = __cpu_to_le32( pCCB->rx_irq[pCCB->rx_irq_index] );
        if (state)
        {
            /*printk("rx_irq(%d,%d): next vector already available (0x%.8lX)\n", pCCB->BoardNumber, pCCB->ChannelNumber, state);*/
        }
    }
    return;
}

void ChannelIsrTx( TP862_CCB* pCCB )
{
    unsigned long state;
    unsigned long sourceId;
    TP862_TxFD*     pTxFD;
    int served = FALSE;
    unsigned long index;


    /* build sourceId for this channel */
    sourceId = (0x04 + pCCB->ChannelNumber) << 28;

    /*
    **  perform irq-index syncing
    **  ( search for a vector != 0 )
    */
    if (pCCB->tx_irq_index_outofsync)
    {
        for (index=0; index<IRQ_RING_SIZE; index++)
        {
            state = __cpu_to_le32( pCCB->tx_irq[index] );
            if (state != 0)
            {
                /* found a vector! we are sync'ed again!*/
#ifdef TDRV001_DEBUG_INTR
                printk(KERN_INFO "%s tx_irq(%d,%d): synchronized at index = %ld.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, index);
#endif
                pCCB->tx_irq_index_outofsync = FALSE;
                pCCB->tx_irq_index = index;
                break;
            }
        }
    }
    /*
    ** check if a correct interrupt queue index was found.
    ** if not, don't serve this interrupt (it was not for this channel)
    */
    if (pCCB->tx_irq_index_outofsync)
    {
        /*printk("tx_irq(%d): not synchronized yet.", pExtension->ChannelNumber);*/
        return;
    }

    /* get current interrupt vector from queue */
    state = __cpu_to_le32( pCCB->tx_irq[pCCB->tx_irq_index] );
    while (state != 0x00000000)
    {

        /* check if the received interrupt was for us */
        if ( (state & sourceId) == sourceId)
        {
            /* there is a new vector for us. */
#ifdef TDRV001_DEBUG_INTR
            printk("%s tx_irq(%d,%d): vector = 0x%lx, idx=%ld\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, state, pCCB->tx_irq_index);
#endif
            /* determine type of interrupt (dma or scc) */
            if (state & SccInt)
            {
                /*
                **  we have an SCC interrupt. serve it.
                */
#ifdef TDRV001_DEBUG_INTR
                printk(KERN_INFO "%s tx_irq(%d,%d): scc interrupt.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
#endif

                if (state & ALLS)
                {
#ifdef TDRV001_DEBUG_INTR
                    printk(KERN_INFO "%s tx_irq(%d,%d): ALLS\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
#endif


                    pTxFD = &pCCB->tx_fd[pCCB->tx_fd_index];
                    if (__cpu_to_le32(pTxFD->complete) == DataComplete)
                    {
#ifdef TDRV001_DEBUG_INTR
                        printk(KERN_INFO "%s tx_irq(%d,%d): completed desc found (queue = %ld).\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber, pCCB->tx_fd_index);
#endif
                        pCCB->tx_fd_index = (pCCB->tx_fd_index + 1) % TX_DATA_QUEUE_SIZE;
                        if (pCCB->tx_free < TX_DATA_QUEUE_SIZE)
                        {
                            pCCB->tx_free++;
                        }
#ifdef TDRV001_DEBUG_INTR
                        printk(KERN_INFO "%s tx_irq(%d,%d): %d descriptors free again.\n", DEBUG_NAME, pCCB->pDCB->BoardNumber, pCCB->ChannelNumber, pCCB->tx_free);
#endif
                        /* signal that a descriptor is free again*/
                        wake_up( &pCCB->tx_waitqueue );
                        pTxFD->finished = TRUE;

                        /*
                        ** free the allocated buffer
                        */
                        /*printk("tx_irq(%d,%d): freeing buffer (data='%s')\n", pCCB->BoardNumber, pCCB->ChannelNumber, (unsigned char*)pTxFD->data_virt);*/
                        if (pTxFD->size && pTxFD->txbuf_virt && pTxFD->txbuf_dma)
                        {
                            /*printk(KERN_INFO "%s tx_irq(%d,%d): freeing buffer\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);*/
                            pci_free_consistent(pCCB->dev, pTxFD->size, (unsigned char*)pTxFD->txbuf_virt, (dma_addr_t)pTxFD->txbuf_dma);
                            pTxFD->txbuf_virt = 0;
                            pTxFD->txbuf_dma = 0;
                            pTxFD->size = 0;
                            /*
                            ** update ok-count
                            */
                            pCCB->tx_count_ok++;
                            if (pCCB->tx_free == TX_DATA_QUEUE_SIZE)
                            {
                                pCCB->TxTimeRemaining = -1;
                            } else {
                                pCCB->TxTimeRemaining = pCCB->DeviceWriteTimeout;
                            }
                        } else {
                            /*printk("%s tx_irq(%d,%d): not freeing buffer, no size or pointer NULL!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);*/
                        }
                    } else {
                        /*printk("%s tx_irq(%d,%d): descriptor not completed.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);*/
                    }
                }
                if (state & XPR)
                {
#ifdef TDRV001_DEBUG_INTR
                    printk(KERN_INFO "%s tx_irq(%d,%d): XPR interrupt.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
#endif
                }

            } else {
                /*
                **  we have a DMA interrupt. serve it.
                */
#ifdef TDRV001_DEBUG_INTR
                printk(KERN_INFO "%s tx_irq(%d,%d): dma interrupt.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
#endif
            }

            /* mark vector as served */
            pCCB->tx_irq[pCCB->tx_irq_index] = 0x00000000;
            /* set index to next space */
            pCCB->tx_irq_index = (pCCB->tx_irq_index+1) % IRQ_RING_SIZE;


            served = TRUE;
        }
        /* get next interrupt vector from queue (look forward) */
        state = __cpu_to_le32( pCCB->tx_irq[pCCB->tx_irq_index] );
        if (state)
        {
            /*printk("tx_irq(%d,%d): next vector already available (0x%.8lX)\n", pCCB->pDCB->boardNumber, pCCB->channelNumber, state);*/
        }
    }
    return;
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
static TP_IRQ_RETURN_T tp862_isr(int irq, void *token, struct pt_regs *ptregs)
#else
static TP_IRQ_RETURN_T tp862_isr(int irq, void *token)
#endif
{
    unsigned long state = 0;
    TP862_DCB* pDCB = (TP862_DCB*)token;
    int channel;
  int					retval = IRQ_NONE;


    /* read interrupt status */
    state = READ_REGISTER_ULONG( (unsigned long*)(pDCB->dscc4_space + GSTAR) );

    if (state != 0)
    {
    retval = IRQ_HANDLED;

        /* clear interrupt status */
        WRITE_REGISTER_ULONG( (unsigned long*)(pDCB->dscc4_space + GSTAR), state );

#ifdef TDRV001_DEBUG_INTR
    printk(KERN_INFO "%s interrupt entry (board #%d)\n", DEBUG_NAME, pDCB->BoardNumber);
#endif

        if (state & Cfg)
        {
            /* we have received a "Configuration" interrupt */
            /*printk("%s isr: CFG-Int\n", DEBUG_NAME);*/
            tp862_cfg_irq( pDCB );
        }
        /* rx interrupt?    */
      if (state & RxEvt) {
            /* we have received a "Receive" interrupt */
            /*printk("%s isr: Rx-Int\n", DEBUG_NAME); */
            for (channel=0; channel<TP862_NUM_CHANS; channel++)
            {
                ChannelIsrRx( pDCB->pCCB[channel] );
            }
        }

        /* tx interrupt? */
      if (state & TxEvt) {
            /* we have received a "Transmit" interrupt */
            /*printk("%s isr: Tx-Int\n", DEBUG_NAME);*/
            for (channel=0; channel<TP862_NUM_CHANS; channel++)
            {
                ChannelIsrTx( pDCB->pCCB[channel] );
            }
        }
    }
  TP_IRQ_RETURN(retval);
}





int tp862_reset_dscc4( TP862_DCB* pDCB )
{
    int retval;
    u32 status_command, bar0, bar1, cachelinesize, interruptline;


    /****************************************/
    /* save current PCI initialization data */
    /****************************************/
    if ( (retval= pci_read_config_dword( pDCB->dev, 0x04, &status_command)) != 0)
    {
        printk(KERN_WARNING "%s reset_dscc4: read error for Status/Command\n", DEBUG_NAME);
        return retval;
    }
    if ( (retval= pci_read_config_dword( pDCB->dev, 0x10, &bar0)) != 0)
    {
        printk(KERN_WARNING "%s reset_dscc4: read error for BAR0\n", DEBUG_NAME);
        return retval;
    }
    if ( (retval= pci_read_config_dword( pDCB->dev, 0x14, &bar1)) != 0)
    {
        printk(KERN_WARNING "%s reset_dscc4: read error for BAR1\n", DEBUG_NAME);
        return retval;
    }
    if ( (retval= pci_read_config_dword( pDCB->dev, 0x0C, &cachelinesize)) != 0)
    {
        printk(KERN_WARNING "%s reset_dscc4: read error for Cache Line Size\n", DEBUG_NAME);
        return retval;
    }
    if ( (retval= pci_read_config_dword( pDCB->dev, 0x3C, &interruptline)) != 0)
    {
        printk(KERN_WARNING "%s reset_dscc4: read error for Interrupt Line\n", DEBUG_NAME);
        return retval;
    }

#ifdef TDRV001_DEBUG_VIEW
    printk(KERN_INFO "%s reset_dscc4: pci-header stored successfully.\n", DEBUG_NAME);
#endif

    /*****************/
    /* perform reset */
    /*****************/
#ifdef TDRV001_DEBUG_VIEW
    printk(KERN_INFO "%s reset_dscc4: performing reset.\n", DEBUG_NAME);
#endif
    WRITE_REGISTER_UCHAR( (unsigned char*)(pDCB->cpld_space + TP862_CPLD_RESET_REG), 0x01);
    mdelay(200);
#ifdef TDRV001_DEBUG_VIEW
    printk(KERN_INFO "%s reset_dscc4: reset done.\n", DEBUG_NAME);
#endif

    /***********************************************************/
    /* write saved PCI initialization data to device (re-init) */
    /***********************************************************/
    if ( (retval= pci_write_config_dword( pDCB->dev, 0x04, status_command)) != 0)
    {
        printk(KERN_WARNING "%s reset_dscc4: write error for Status/Command\n", DEBUG_NAME);
        return retval;
    }
    if ( (retval= pci_write_config_dword( pDCB->dev, 0x10, bar0)) != 0)
    {
        printk(KERN_WARNING "%s reset_dscc4: write error for BAR0\n", DEBUG_NAME);
        return retval;
    }
    if ( (retval= pci_write_config_dword( pDCB->dev, 0x14, bar1)) != 0)
    {
        printk(KERN_WARNING "%s reset_dscc4: write error for BAR1\n", DEBUG_NAME);
        return retval;
    }
    if ( (retval= pci_write_config_dword( pDCB->dev, 0x0C, cachelinesize)) != 0)
    {
        printk(KERN_WARNING "%s reset_dscc4: write error for Cache Line Size\n", DEBUG_NAME);
        return retval;
    }
    if ( (retval= pci_write_config_dword( pDCB->dev, 0x3C, interruptline)) != 0)
    {
        printk(KERN_WARNING "%s reset_dscc4: write error for Interrupt Line\n", DEBUG_NAME);
        return retval;
    }

#ifdef TDRV001_DEBUG_VIEW
    printk(KERN_INFO "%s reset_dscc4: pci-header written successfully.\n", DEBUG_NAME);
#endif


    /************************************************************/
    /* re-enable local bus interface to access the onboard CPLD */
    /************************************************************/
    WRITE_REGISTER_ULONG( (unsigned long*)(pDCB->dscc4_space + LCONF), 0x0040001F);
    /* set local bus clock to CLK/2; little endian; LTDA mode */
    WRITE_REGISTER_ULONG( (unsigned long*)(pDCB->dscc4_space + GMODE), 0x00080001 );

    return 0;
}


int set_osc_source( TP862_CCB* pCCB, OSC_SOURCE option )
{
    OSC_SOURCE oscsrc = (OSC_SOURCE)option;

    switch (oscsrc)
    {
    case OSCSOURCE_XTAL1:
#ifdef TDRV001_DEBUG_VIEW
        printk("Setting Xtal1 as Global Oscillator Source.\n");
#endif
        pCCB->osc_hz = pCCB->xtal1_hz;
        WRITE_REGISTER_UCHAR( (unsigned char*)(pCCB->cpld_space + TP862_CPLD_OSC_SEL_REG), 0x00 );
        break;
    case OSCSOURCE_XTAL2:
#ifdef TDRV001_DEBUG_VIEW
        printk("Setting Xtal2 as Global Oscillator Source.\n");
#endif
        pCCB->osc_hz = pCCB->xtal2_hz;
        WRITE_REGISTER_UCHAR( (unsigned char*)(pCCB->cpld_space + TP862_CPLD_OSC_SEL_REG), 0x01 );
        break;
    default:
        printk("OscSource: Illegal parameter!\n");
        return -EFAULT;
        break;
    }
    return 0;
}



int set_clock_inversion( TP862_CCB* pCCB, unsigned long option )
{
    unsigned char ucValue = 0x00;


    if (pCCB->Version == ID_VERSION_1_0)
    {
        /* this feature is not supported! */
        return -1;
    }

    ucValue = READ_REGISTER_UCHAR( (unsigned char*)(pCCB->cpld_space + TP862_CPLD_0_SPECIAL_REG_A + 2*pCCB->ChannelNumber) );

    /* clear inversion bits */
    ucValue &= (~TP862_TXC_INV);
    ucValue &= (~TP862_RXC_INV);

    if ( option & (~CLOCKINV_TXC) & (~CLOCKINV_RXC) )
    {
        printk("%s: Clockinversion: Illegal parameter!\n", DEBUG_NAME);
        return -1;
    }

    if ( option & CLOCKINV_TXC )
    {
        /* invert Tx Clock */
        ucValue |= TP862_TXC_INV;
    }

    if ( option & CLOCKINV_RXC )
    {
        /* invert Rx Clock */
        ucValue |= TP862_RXC_INV;
    }

    WRITE_REGISTER_UCHAR( (unsigned char*)(pCCB->cpld_space + TP862_CPLD_0_SPECIAL_REG_A + 2*pCCB->ChannelNumber), ucValue );
    return 0;
}



int set_baudrate( TP862_CCB* pCCB, unsigned long bps )
{
    unsigned long xtal;
  unsigned long n = 0;
    unsigned long m = 0;
    unsigned long divider;
    unsigned long brr;
    unsigned long state;


    switch (pCCB->OperationMode.clockmode)
    {
    case CLOCKMODE_CM0B:
    case CLOCKMODE_CM7B:
        xtal = pCCB->osc_hz;
        break;
    case CLOCKMODE_CM8A:
        xtal = pCCB->xtal1_hz;
        break;
    case CLOCKMODE_CM8:
    case CLOCKMODE_CM8B:
        xtal = pCCB->xtal2_hz;
        break;
    case CLOCKMODE_CM3B:
        xtal = pCCB->xtalExt_hz;
        break;
    case CLOCKMODE_CM0A:
    case CLOCKMODE_CM4:
    case CLOCKMODE_CM9:
        /* no baudrate calculation necessary */
        return 0; /* everything is fine */
        break;
    default:
        /* should never happen */
        printk(KERN_WARNING "%s set_baudrate: Error! Wrong Clockmode!\n", DEBUG_NAME);
        return TP862_ERROR_CLOCKMODE;
        break;
    }

    /*printk("set_baudrate: xtal = %ld, bps = %ld\n", xtal, bps);*/

    if (bps && xtal)
    {
        if ( (pCCB->OperationMode.commtype == COMMTYPE_ASYNC) && (pCCB->OperationMode.oversampling == ENABLED) )
        {
            if ( (bps*16) <= xtal)
            {
            /* multiply baudrate with 16 (oversampling) */
                bps = 16*bps;
            } else {
                printk(KERN_WARNING "%s 16-times oversampling not possible! Baudrate too high.\n", DEBUG_NAME);
                return TP862_ERROR_BAUDRATE;
            }
        }

    divider = xtal / bps;
    if (divider > BRR_DIVIDER_MAX) {
            printk(KERN_WARNING "%s Bad Error! baudrate divider too big!\n", DEBUG_NAME);
            return TP862_ERROR_BAUDRATE;
        }

        if (divider >> 22) {
      n = 63;
      m = 15;
    } else if (divider) {
      /* Extraction of the 6 highest weighted bits */
      m = 0;
      while (0xffffffc0 & divider) {
        m++;
        divider >>= 1;
      }
      n = divider-1;
    }
    brr = (m << 8) | n;
    divider = (n+1) << m;

        /*
        ** check if desired baudrate can be set properly
        */
        if ((xtal/divider) != bps)
        {
            if ((pCCB->OperationMode.commtype == COMMTYPE_ASYNC) && (pCCB->OperationMode.oversampling == ENABLED) )
            {
                if ( (bps/16) != ((xtal / divider)/16) )
                {
                    printk(KERN_WARNING "%s baudrate not valid: %ld (possible: %ld)\n", DEBUG_NAME, bps/16, (xtal / divider)/16);
                } else {
                    printk(KERN_WARNING "%s baudrate not valid with oversampling!\n", DEBUG_NAME);
                }
            } else {
                printk(KERN_WARNING "%s baudrate not valid: %ld (possible: %ld)\n", DEBUG_NAME, bps, (xtal / divider));
            }
            return TP862_ERROR_BAUDRATE;
        }

        WRITE_REGISTER_ULONG((unsigned long*)(pCCB->scc_regstart + BRR), brr);
        pCCB->scc_regs_local[BRR>>2] = brr;

        if (pCCB->OperationMode.commtype == COMMTYPE_ASYNC)
        {
            state = pCCB->scc_regs_local[CCR0>>2];
            if (pCCB->OperationMode.oversampling == ENABLED)
            {
                state |= (1<<7);
            } else {
                state &= ~(1<<7);
            }
            WRITE_REGISTER_ULONG((unsigned long*)(pCCB->scc_regstart + CCR0), state);
            pCCB->scc_regs_local[CCR0>>2] = state;
        }
        return 0;   /* everything ok */
    }
    printk(KERN_WARNING "%s baudrate=0 or xtal=0 not valid\n", DEBUG_NAME);
    return TP862_ERROR_BAUDRATE;       /* error */
}


void tp862_set_channel_dma_memory_addresses( TP862_CCB* pCCB )
{
    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->dscc4_space + CH0BRDA + pCCB->ChannelNumber*0x0C), pCCB->rx_fd_dma );
    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->dscc4_space + CH0FRDA + pCCB->ChannelNumber*0x04), pCCB->rx_fd_dma );
    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->dscc4_space + CH0BTDA + pCCB->ChannelNumber*0x0C), pCCB->tx_fd_first_dma );
    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->dscc4_space + CH0FTDA + pCCB->ChannelNumber*0x04), pCCB->tx_fd_first_dma );
    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->dscc4_space + CH0LRDA + pCCB->ChannelNumber*0x04), 0x00000000 );
    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->dscc4_space + CH0LTDA + pCCB->ChannelNumber*0x04), pCCB->tx_fd_last_dma );
    return;
}


int tp862_channel_init( TP862_CCB* pCCB )
{

    setup_channel_tx_descriptor_list( pCCB );
    setup_channel_rx_descriptor_list( pCCB );

    /* set CPLD registers */
    /* 14.7456MHz clock source, standard baudrate, needed for channel init !! */
    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->scc_regstart + CCR0), STANDARD_CCR0 );
    pCCB->scc_regs_local[CCR0>>2] = STANDARD_CCR0;
    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->scc_regstart + CCR1), STANDARD_CCR1 );
    pCCB->scc_regs_local[CCR1>>2] = STANDARD_CCR1;
    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->scc_regstart + CCR2), STANDARD_CCR2 );
    pCCB->scc_regs_local[CCR2>>2] = STANDARD_CCR2;
    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->scc_regstart + IMR),  STANDARD_IMR );
    pCCB->scc_regs_local[IMR>>2] = STANDARD_IMR;

    /* stop receiver*/
    stop_receiver( pCCB );

    set_baudrate( pCCB, STANDARD_BAUDRATE );

    tp862_set_channel_dma_memory_addresses( pCCB );

    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->dscc4_space + IQSCC0RXBAR + pCCB->ChannelNumber*0x04), pCCB->rx_irq_dma );
    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->dscc4_space + IQSCC0TXBAR + pCCB->ChannelNumber*0x04), pCCB->tx_irq_dma );
/*  ATTENTION: the index is not reset to 0 by dscc4! */

    /* trigger dscc4 configuration */
    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->dscc4_space + GCMDR), ((0x11000000 << pCCB->ChannelNumber) | 0x01) );

    return 0;
}

int set_operation_mode( TP862_CCB* pCCB, TP862_OPERATION_MODE_STRUCT* pOperationMode )
{
    int store_commtype    = TRUE;
    int store_clockmode   = TRUE;
    int store_txclkout    = TRUE;
    int store_transceiver = TRUE;
    int store_dcedte      = TRUE;
    int store_oversampling= TRUE;
    int store_usetermchar = TRUE;
    int store_clockinv    = TRUE;
    int result = 0;
    unsigned long state;


    /*
    ** set commtype (HDLC/ASYNC)
    */
    switch (pOperationMode->commtype)
    {
    case COMMTYPE_HDLC:
        /*printk("COMMTYPE_HDLC\n");*/
        break;
    case COMMTYPE_ASYNC:
        /*printk("COMMTYPE_ASYNC\n");*/
        break;
    case COMMTYPE_NO_CHANGES:
        /*printk("COMMTYPE_NO_CHANGES\n");*/
        store_commtype = FALSE;
        break;
    default:
        printk(KERN_WARNING "%s OperationMode(%d,%d): unknown COMMTYPE!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
        return TP862_ERROR_COMMTYPE;
        break;
    }

    /*
    ** set clockmode
    */
    switch (pOperationMode->clockmode)
    {
    case CLOCKMODE_NO_CHANGES:
        store_clockmode= FALSE;
        break;
    case CLOCKMODE_CM0A:
        break;
    case CLOCKMODE_CM0B:
        break;
    case CLOCKMODE_CM3B:
        break;
    case CLOCKMODE_CM4:
        break;
    case CLOCKMODE_CM9:
        /* only supported for v1.1 boards */
        switch (pCCB->Version)
        {
        case ID_VERSION_1_0:
            printk(KERN_WARNING "%s OperationMode(%d,%d): CLOCKMODE_CM9 unsupported!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
            store_clockmode= FALSE;
            return TP862_ERROR_CLOCKMODE;
            break;
        case ID_VERSION_1_1:
            break;
        default:
            printk(KERN_WARNING "%s OperationMode(%d,%d): unable to determine board version!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
            store_clockmode= FALSE;
            break;
        }
        break;
    case CLOCKMODE_CM7B:
        break;
    case CLOCKMODE_CM8A:
        /* only supported for v1.1 boards */
        switch (pCCB->Version)
        {
        case ID_VERSION_1_0:
            printk(KERN_WARNING "%s OperationMode(%d,%d): CLOCKMODE_CM8A unsupported!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
            store_clockmode= FALSE;
            return TP862_ERROR_CLOCKMODE;
            break;
        case ID_VERSION_1_1:
            /*printf("ok.\n");*/
            break;
        default:
            printk(KERN_WARNING "%s OperationMode(%d,%d): unable to determine board version!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
            store_clockmode= FALSE;
            break;
        }
        break;
    case CLOCKMODE_CM8:
    case CLOCKMODE_CM8B:
        break;
    default:
        printk(KERN_WARNING "%s OperationMode(%d,%d): unknown CLOCKMODE!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
        return TP862_ERROR_CLOCKMODE;
        break;
    }

    /*
    ** set txclock monitor output
    */
    switch (pOperationMode->txclk_out)
    {
    case ENABLED:
        /*printk("txclock enabled.\n");*/
        break;
    case DISABLED:
        /*printk("txclock disabled.\n");*/
        break;
    case NO_CHANGES:
        /*printk("txclock unchanged.\n");*/
        store_txclkout = TRUE;
        break;
    default:
        printk(KERN_WARNING "%s OperationMode(%d,%d): unknown value for txclk_out!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
        return TP862_ERROR_TXCLOCK_OUT;
        break;
    }

    /*
    ** determine transceiver mode
    */
    switch (pOperationMode->transceivermode)
    {
    case TRANSCEIVER_NO_CHANGES:
        /*printk("transceivermode: unchanged.\n");*/
        store_transceiver = FALSE;
        break;
    case TRANSCEIVER_NOT_USED:
        /*printk("transceivermode: TRANSCEIVER_NOT_USED.\n");*/
        break;
    case TRANSCEIVER_RS530A:
        /*printk("transceivermode: TRANSCEIVER_RS530A.\n");*/
        break;
    case TRANSCEIVER_RS530:
        /*printk("transceivermode: TRANSCEIVER_RS530.\n");*/
        break;
    case TRANSCEIVER_X21:
        /*printk("transceivermode: TRANSCEIVER_X21.\n");*/
        break;
    case TRANSCEIVER_V35:
        /*printk("transceivermode: TRANSCEIVER_V35.\n");*/
        break;
    case TRANSCEIVER_RS449:
        /*printk("transceivermode: TRANSCEIVER_RS449.\n");*/
        break;
    case TRANSCEIVER_V36:
        /*printk("transceivermode: TRANSCEIVER_V36.\n");*/
        break;
    case TRANSCEIVER_RS232:
        /*printk("transceivermode: TRANSCEIVER_RS232.\n");*/
        break;
    case TRANSCEIVER_V28:
        /*printk("transceivermode: TRANSCEIVER_V28.\n");*/
        break;
    case TRANSCEIVER_NO_CABLE:
        /*printk("transceivermode: TRANSCEIVER_NO_CABLE.\n");*/
        break;
    default:
        printk(KERN_WARNING "%s OperationMode(%d,%d): unknown value for transceivermode!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
        return TP862_ERROR_TRANSCEIVER;
        break;
    }

    switch (pOperationMode->dce_dte)
    {
    case DCEDTE_NO_CHANGES:
        /*printk("dce_dte: unchanged.\n");*/
        store_dcedte = FALSE;
        break;
    case DCEDTE_DCE:
        /*printk("dce_dte: dce.\n");*/
        break;
    case DCEDTE_DTE:
        /*printk("dce_dte: dte.\n");*/
        break;
    default:
        printk(KERN_WARNING "%s OperationMode(%d,%d): unknown value for dce_dte!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
        return TP862_ERROR_DCEDTE;
        break;
    }

    switch (pOperationMode->oversampling)
    {
    case ENABLED:
        /*printk("async oversampling enabled.\n");*/
        break;
    case DISABLED:
        /*printk("async oversampling disabled.\n");*/
        break;
    case NO_CHANGES:
        /*printk("async oversampling unchanged.\n");*/
        store_oversampling = FALSE;
        break;
    default:
        printk(KERN_WARNING "%s OperationMode(%d,%d): unknown value for async oversampling!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
        return TP862_ERROR_OVERSAMPLING;
        break;
    }

    /*
    ** check clock inversion
    */
    if (pOperationMode->clockinversion != CLOCKINV_NONE)
    {
        switch (pCCB->Version)
        {
        case ID_VERSION_1_0:
            printk(KERN_WARNING "%s OperationMode(%d,%d): Clockinversion not supported for v1.0 !\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
            store_clockinv = FALSE;
            return TP862_ERROR_CLOCKINVERSION;
            break;
        case ID_VERSION_1_1:
            break;
        default:
            printk(KERN_WARNING "%s OperationMode(%d,%d): Unable to determine version!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
            store_clockinv = FALSE;
            return TP862_ERROR_CLOCKINVERSION;
            break;
        }
    }

    switch (pOperationMode->usetermchar)
    {
    case ENABLED:
        break;
    case DISABLED:
        break;
    case NO_CHANGES:
        store_usetermchar = FALSE;
        break;
    default:
        printk(KERN_WARNING "%s OperationMode(%d,%d): unknown value for async termination character usage!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
        return TP862_ERROR_USETERMCHAR;
        break;
    }

    /*
    ** store data in CCB
    */
    if (store_commtype)     pCCB->OperationMode.commtype        = pOperationMode->commtype;
    if (store_clockmode)    pCCB->OperationMode.clockmode       = pOperationMode->clockmode;
    if (store_txclkout)     pCCB->OperationMode.txclk_out       = pOperationMode->txclk_out;
    if (store_transceiver)  pCCB->OperationMode.transceivermode = pOperationMode->transceivermode;
    if (store_dcedte)       pCCB->OperationMode.dce_dte         = pOperationMode->dce_dte;
    if (store_oversampling) pCCB->OperationMode.oversampling    = pOperationMode->oversampling;
    if (store_clockinv)     pCCB->OperationMode.clockinversion  = pOperationMode->clockinversion;
    if (store_usetermchar)
    {
        pCCB->OperationMode.usetermchar     = pOperationMode->usetermchar;
        pCCB->OperationMode.termchar        = pOperationMode->termchar;
    }
    pCCB->OperationMode.baudrate  = pOperationMode->baudrate;


    /*
    ** update dce_dte
    */
    if (store_dcedte) set_cpld_dcedte( pCCB, pOperationMode->dce_dte );

    /*
    ** update transceiver mode
    */
    if (store_transceiver) set_cpld_transceivermode( pCCB, pOperationMode->transceivermode );

    /*
    ** update communication type (HDLC/ASYNC)
    */
    state = pCCB->scc_regs_local[CCR0>>2];
    state &= ~(3<<16);  /* mask out serial port mode bits */
    state &= ~(1<<7);   /* mask out BCR bit (oversampling) */
    switch (pCCB->OperationMode.commtype)
    {
    case COMMTYPE_HDLC:
        state |= (0<<16);
        break;
    case COMMTYPE_ASYNC:
        state |= (3<<16);
        /* update oversampling */
        switch (pCCB->OperationMode.oversampling)
        {
        case ENABLED:
            state |= (1<<7);
            break;
        case DISABLED:
            state &= ~(1<<7);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->scc_regstart + CCR0), state );
    pCCB->scc_regs_local[CCR0>>2] = state;

    /*
    ** update clockmode + txclk_out, set clockswitch
    */
    state = pCCB->scc_regs_local[CCR0>>2];
    state &= ~(Ccr0ClockMask);
    switch (pCCB->OperationMode.clockmode)
    {
    case CLOCKMODE_CM0A:
        set_cpld_clocksource( pCCB, CLOCKSOURCE_EXTERNAL );
        break;
    case CLOCKMODE_CM0B:
        state |= 0x10;      /* clockmode 0b */
        /* set CPLD's clockswitch to 'external clock' */
        set_cpld_clocksource( pCCB, CLOCKSOURCE_EXTERNAL );
        break;
    case CLOCKMODE_CM3B:
        state |= 0x13;      /* clockmode 3b */
        pCCB->OperationMode.baudrate = pCCB->xtal2_hz;
        /* set CPLD's clockswitch to 'external clock' */
        set_cpld_clocksource( pCCB, CLOCKSOURCE_EXTERNAL );
        break;
    case CLOCKMODE_CM4:
        state |= 0x0C;      /* clockmode 4 + HS */
        set_cpld_clocksource( pCCB, CLOCKSOURCE_EXTERNAL );
        break;
    case CLOCKMODE_CM9:
        state |= 0x0C;      /* clockmode 4 + HS */
        WRITE_REGISTER_UCHAR( (unsigned char*)(pCCB->cpld_space + 2*pCCB->ChannelNumber + TP862_CPLD_0_SPECIAL_REG_B),
            TP862_CM9_EN | 0x02 | 0x08 | 0x10 );
        break;
    case CLOCKMODE_CM7B:
        state |= 0x17;      /* clockmode 7b */
        break;
    case CLOCKMODE_CM8A:
        state |= 0x13;      /* clockmode 3b with special clock source */
        /* set CPLD's clockswitch to 'onboard clock' */
        set_cpld_clocksource( pCCB, CLOCKSOURCE_XTAL1 );
        break;
    case CLOCKMODE_CM8:
    case CLOCKMODE_CM8B:
        state |= 0x13;      /* clockmode 3b with special clock source */
        /* set CPLD's clockswitch to 'onboard clock' */
        set_cpld_clocksource( pCCB, CLOCKSOURCE_XTAL2 );
        break;

    default:
        break;
    }
    if ( (pCCB->OperationMode.txclk_out == ENABLED) && (pCCB->OperationMode.clockmode != CLOCKMODE_CM0A) )
    {
        state |= 0x20;
    }
    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->scc_regstart + CCR0), state );
    pCCB->scc_regs_local[CCR0>>2] = state;

    /*
    ** set termination character for ASYNC operation mode
    */
    if (pCCB->OperationMode.usetermchar == ENABLED)
    {
        state = TCDE;
        state |= pCCB->OperationMode.termchar;
        /*printk("Using TerminationCharacter: '%c'\n", pCCB->OperationMode.termchar);*/
    } else {
        state = 0;
        /*printk("No TerminationCharacter is used.\n");*/
    }
    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->scc_regstart + TCR), state );
    pCCB->scc_regs_local[TCR>>2] = state;

    /*
    ** set clock-inversion
    */
    set_clock_inversion( pCCB, pCCB->OperationMode.clockinversion );

    /*
    ** set baudrate
    */
    result = set_baudrate( pCCB, pCCB->OperationMode.baudrate );

    pCCB->tx_fd_first_dma = pCCB->tx_fd_dma;
    pCCB->tx_fd_last_dma = pCCB->tx_fd_dma;
    pCCB->tx_fd_last_index = 1;

    channel_reset( pCCB );

    return result;
}


void set_cpld_clocksource( TP862_CCB* pCCB, unsigned long option )
{
    unsigned char ucOldValue, ucNewValue;

    ucOldValue = READ_REGISTER_UCHAR( (unsigned char*)(pCCB->cpld_space + TP862_CPLD_TX_CLK_SEL_REG) );
    switch (option)
    {
    case CLOCKSOURCE_EXTERNAL:
        ucNewValue = 1;
        break;
    case CLOCKSOURCE_ONBOARD:
    case CLOCKSOURCE_XTAL2:
        ucNewValue = 2;
        break;
    case CLOCKSOURCE_XTAL1:
        /* only available for version >= 1.1 */
        ucNewValue = 3;
        break;
    default:
        ucNewValue = 0;
        break;
    }
    ucOldValue = ucOldValue & ~(0x03 << 2*pCCB->ChannelNumber);
    ucNewValue = ucOldValue | (ucNewValue << 2*pCCB->ChannelNumber);
    /*printk("Clock-Sel-Reg = 0x%x\n", ucNewValue);*/
    WRITE_REGISTER_UCHAR( (unsigned char*)(pCCB->cpld_space + TP862_CPLD_TX_CLK_SEL_REG), ucNewValue );
    return;
}


void set_cpld_transceivermode( TP862_CCB* pCCB, TRANSCEIVER_MODE mode )
{
    unsigned char ucOldValue, ucNewValue;
    unsigned char value;
    unsigned char regoffset;

    if (pCCB->ChannelNumber < 2)
    {
        regoffset = TP862_CPLD_MODE_REG0;
    } else {
        regoffset = TP862_CPLD_MODE_REG1;
    }

    /* get current register content */
    ucOldValue = READ_REGISTER_UCHAR( (unsigned char*)(pCCB->cpld_space + regoffset) );

    switch (mode)
    {
    case TRANSCEIVER_NO_CHANGES:
        return;
        break;
    case TRANSCEIVER_NOT_USED:
        /*printk("transceivermode: TRANSCEIVER_NOT_USED.\n");*/
        value = TP862_TRANSCEIVER_NOT_USED;
        break;
    case TRANSCEIVER_RS530A:
        /*printk("transceivermode: TRANSCEIVER_RS530A.\n");*/
        value = TP862_TRANSCEIVER_RS530A;
        break;
    case TRANSCEIVER_RS530:
        /*printk("transceivermode: TRANSCEIVER_RS530.\n");*/
        value = TP862_TRANSCEIVER_RS530;
        break;
    case TRANSCEIVER_X21:
        /*printk("transceivermode: TRANSCEIVER_X21.\n");*/
        value = TP862_TRANSCEIVER_X21;
        break;
    case TRANSCEIVER_V35:
        /*printk("transceivermode: TRANSCEIVER_V35.\n");*/
        value = TP862_TRANSCEIVER_V35;
        break;
    case TRANSCEIVER_RS449:
        /*printk("transceivermode: TRANSCEIVER_RS449.\n");*/
        value = TP862_TRANSCEIVER_RS449;
        break;
    case TRANSCEIVER_V36:
        /*printk("transceivermode: TRANSCEIVER_V36.\n");*/
        value = TP862_TRANSCEIVER_V36;
        break;
    case TRANSCEIVER_RS232:
        /*printk("transceivermode: TRANSCEIVER_RS232.\n");*/
        value = TP862_TRANSCEIVER_RS232;
        break;
    case TRANSCEIVER_V28:
        /*printk("transceivermode: TRANSCEIVER_V28.\n");*/
        value = TP862_TRANSCEIVER_V28;
        break;
    case TRANSCEIVER_NO_CABLE:
        /*printk("transceivermode: TRANSCEIVER_NO_CABLE.\n");*/
        value = TP862_TRANSCEIVER_NO_CABLE;
        break;
    default:
        printk(KERN_WARNING "%s OperationMode(%d,%d): unknown value for transceivermode!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
        return;
        break;
    }
    value <<= 1;
    ucOldValue &= ~(0x0E << 4*(pCCB->ChannelNumber % 2));
    ucNewValue = ucOldValue | ( value << 4*(pCCB->ChannelNumber % 2) );

    WRITE_REGISTER_UCHAR( (unsigned char*)(pCCB->cpld_space + regoffset), ucNewValue );
    return;
}


void set_cpld_dcedte( TP862_CCB* pCCB, DCEDTE dce_dte  )
{
    unsigned char ucOldValue, ucNewValue;
    unsigned char value;
    unsigned char regoffset;

    if (pCCB->ChannelNumber < 2)
    {
        regoffset = TP862_CPLD_MODE_REG0;
    } else {
        regoffset = TP862_CPLD_MODE_REG1;
    }
    /* get current register content*/
    ucOldValue = READ_REGISTER_UCHAR( (unsigned char*)(pCCB->cpld_space + regoffset) );

    switch (dce_dte)
    {
    case DCEDTE_NO_CHANGES:
        /*printk("dce_dte: unchanged.\n");*/
        return;
        break;
    case DCEDTE_DCE:
        /*printk("dce_dte: dce.\n");*/
        value = TP862_DCEDTE_DCE;
        break;
    case DCEDTE_DTE:
        /*printk("dce_dte: dte.\n");*/
        value = TP862_DCEDTE_DTE;
        break;
    default:
        printk(KERN_WARNING "%s OperationMode(%d,%d): unknown value for dce_dte!\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
        return;
        break;
    }

    ucOldValue &= ~(0x01 << 4*(pCCB->ChannelNumber % 2) );
    ucNewValue = ucOldValue | (value << 4*(pCCB->ChannelNumber % 2) );
    WRITE_REGISTER_UCHAR( (unsigned char*)(pCCB->cpld_space + regoffset), ucNewValue );
    return;
}



void channel_reset_rxtx( TP862_CCB* pCCB )
{
    unsigned long timeout = CHIP_TIMEOUT;

    /*printk("channel_reset_rxtx (ch %d)\n", pCCB->ChannelNumber);*/

    /* switch off receiver (inactive: RAC=0, CCR2)*/
    stop_receiver( pCCB );

    /* wait */
    set_current_state( TASK_INTERRUPTIBLE );
    schedule_timeout( timeout );

    /* reset dma controller */
    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->dscc4_space + CH0CFG + pCCB->ChannelNumber*0x0c), 0x00600000 );

    /* wait */
    set_current_state( TASK_INTERRUPTIBLE );
    schedule_timeout( timeout );


    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->dscc4_space + GCMDR), 0x00000001 );

    /* wait */
    set_current_state( TASK_INTERRUPTIBLE );
    schedule_timeout( timeout );

    pCCB->is_in_reset_state = TRUE;
    pCCB->is_in_init_state = FALSE;

    return;
}


void channel_init_rxtx( TP862_CCB* pCCB )
{
    unsigned long i;
    unsigned long timeout = CHIP_TIMEOUT;

    /*printk("channel_init_rxtx (ch %d)\n", pCCB->ChannelNumber);*/


    tp862_set_channel_dma_memory_addresses( pCCB );
    pCCB->tx_fd_index = 0;
    pCCB->tx_fd_index_dpc = 0;

    pCCB->rx_irq_index_outofsync = TRUE;
    pCCB->tx_irq_index_outofsync = TRUE;

    pCCB->rx_fd_index = 0;
    pCCB->pInternalRingBuffer->get_idx = 0;
    pCCB->pInternalRingBuffer->put_idx = 0;


    /* clear internal ringbuffer*/
    for (i=0; i<RX_DATA_QUEUE_SIZE; i++)
    {
        memset(pCCB->pInternalRingBuffer->entry[i].pData, 0, TP862_RX_BUFFER_SIZE);
        pCCB->pInternalRingBuffer->entry[i].Valid = FALSE;
    }

    /* init dma */
    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->dscc4_space + CH0CFG + pCCB->ChannelNumber*0x0c), 0x00180000 );

    /* wait */
    set_current_state( TASK_INTERRUPTIBLE );
    schedule_timeout( timeout );

    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->dscc4_space + GCMDR), 0x00000001 );

    /* wait */
    set_current_state( TASK_INTERRUPTIBLE );
    schedule_timeout( timeout );

    /* switch on receiver */
    start_receiver( pCCB );

    /* RRES/XRES  (self-clearing bits. do not store locally.) */
    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->scc_regstart + CMDR), RRES | XRES );

    /* wait */
    set_current_state( TASK_INTERRUPTIBLE );
    schedule_timeout( timeout );

    pCCB->is_in_reset_state = FALSE;
    pCCB->is_in_init_state = TRUE;

    return;
}


void channel_reset( TP862_CCB* pCCB )
{
    if (!pCCB->is_in_reset_state)
    {
        /*printk("#####CHANNEL_RESET\n"); */
        channel_reset_rxtx( pCCB );
    }
    pCCB->tx_fd_first_dma = pCCB->tx_fd_dma;
    pCCB->tx_fd_last_dma = pCCB->tx_fd_dma;
    pCCB->tx_fd_index = 0;
    pCCB->tx_insert = 1;
    if (!pCCB->is_in_init_state)
    {
        /*printk("#####CHANNEL_INIT\n"); */
        channel_init_rxtx( pCCB );
    }
    return;
}


int init_hardware( TP862_DCB* pDCB )
{
    int channel;
    unsigned char ucValue;
    TP862_OPERATION_MODE_STRUCT OperationMode;

    /* init internal bus */
    /* enable local bus interface to access the onboard CPLD */
    WRITE_REGISTER_ULONG( (unsigned long*)(pDCB->dscc4_space + LCONF), 0x0040001F );

    /* set local bus clock to CLK/2; little endian; LTDA mode */
    WRITE_REGISTER_ULONG( (unsigned long*)(pDCB->dscc4_space + GMODE), 0x00080001 );

    /* set configuration interrupt queue lengths and base addresses*/
    /* set IntQueue length Rx/Tx */
    WRITE_REGISTER_ULONG( (unsigned long*)(pDCB->dscc4_space + IQLENR0),
            ( (((IRQ_RING_SIZE>>5)-1)<<28) | (((IRQ_RING_SIZE>>5)-1)<<24) |
              (((IRQ_RING_SIZE>>5)-1)<<20) | (((IRQ_RING_SIZE>>5)-1)<<16) |
              (((IRQ_RING_SIZE>>5)-1)<<12) | (((IRQ_RING_SIZE>>5)-1)<< 8) |
              (((IRQ_RING_SIZE>>5)-1)<< 4) | (((IRQ_RING_SIZE>>5)-1)<< 0) ) );
    /* set IntQueue length Cfg/Per */
    WRITE_REGISTER_ULONG( (unsigned long*)(pDCB->dscc4_space + IQLENR1),
            ( (((IRQ_RING_SIZE>>5)-1)<<20) | (((IRQ_RING_SIZE>>5)-1)<<16) ) );

    WRITE_REGISTER_ULONG( (unsigned long*)(pDCB->dscc4_space + IQCFGBAR), pDCB->cfg_irq_dma );
    /* reset config irq index */
    pDCB->cfg_irq_index = 0;

    for (channel=0; channel<TP862_NUM_CHANS; channel++)
    {
        /* reset interrupt queue indices */
        pDCB->pCCB[channel]->rx_irq_index = 0;
        pDCB->pCCB[channel]->tx_irq_index = 0;
    }

    /* set FIFO size */
    WRITE_REGISTER_ULONG( (unsigned long*)(pDCB->dscc4_space + FIFOCR1), STANDARD_FIFOCR1 );
    WRITE_REGISTER_ULONG( (unsigned long*)(pDCB->dscc4_space + FIFOCR2), STANDARD_FIFOCR2 );
    WRITE_REGISTER_ULONG( (unsigned long*)(pDCB->dscc4_space + FIFOCR3), STANDARD_FIFOCR3 );
    WRITE_REGISTER_ULONG( (unsigned long*)(pDCB->dscc4_space + FIFOCR4), STANDARD_FIFOCR4 );

    /* initiate dscc4 configuration of cfg irq */
    WRITE_REGISTER_ULONG( (unsigned long*)(pDCB->dscc4_space + GCMDR), 0x00200001 );
    /* wait */
    set_current_state( TASK_INTERRUPTIBLE );
    schedule_timeout( CHIP_TIMEOUT );
    if (!pDCB->CfgIntOk)
    {
        printk(KERN_WARNING "%s Bad misconfiguration error! No Config interrupt vector!\n", DEBUG_NAME);
        return -1;
    }

#ifdef TDRV001_DEBUG_VIEW
    printk(KERN_INFO "%s ID-Register: 0x%.2X\n", DEBUG_NAME, READ_REGISTER_UCHAR( (unsigned char*)(pDCB->cpld_space + TP862_CPLD_ID_REG) ) );
    printk(KERN_INFO "%s BuildOption: 0x%.2X\n", DEBUG_NAME, READ_REGISTER_UCHAR( (unsigned char*)(pDCB->cpld_space + TP862_CPLD_BUILDOPTION_REG) ) );
#endif

    /*
    ** evaluate build-option register to fetch xtal frequencies
    ** Note: must be _after_ hardware init because of bus access to CPLD register.
    */
    evaluate_buildoption_register( pDCB );

    /*
    ** evaluate ID register
    */
    ucValue = READ_REGISTER_UCHAR( (unsigned char*)(pDCB->cpld_space + TP862_CPLD_ID_REG) );
    switch (ucValue)
    {
    case ID_VERSION_1_0:
        pDCB->Version = ID_VERSION_1_0;
        break;
    case ID_VERSION_1_1:
        pDCB->Version = ID_VERSION_1_1;
        break;
    default:
#ifdef TDRV001_DEBUG_VIEW
        printk("version unknown (%d)\n", ucValue);
#endif
        pDCB->Version = 0xFF;
        break;
    }

    for (channel=0; channel<TP862_NUM_CHANS; channel++)
    {
        pDCB->pCCB[channel]->osc_hz   = pDCB->xtal1_hz;     /* Xtal1 is connected to OscIn by default */
        pDCB->pCCB[channel]->xtal1_hz = pDCB->xtal1_hz;
        pDCB->pCCB[channel]->xtal2_hz = pDCB->xtal2_hz;
        pDCB->pCCB[channel]->Version  = pDCB->Version;

        memset( pDCB->pCCB[channel]->scc_regs_local, 0, 23*sizeof(unsigned long) );
        pDCB->pCCB[channel]->scc_regs_local[IMR>>2] = 0xFFFFFFFF;
    }


    /*
    ** configure channels
    */
    OperationMode.baudrate = STANDARD_BAUDRATE;
    OperationMode.commtype = STANDARD_COMMTYPE;
    OperationMode.clockmode = STANDARD_CLOCKMODE;
    OperationMode.dce_dte = STANDARD_DCEDTE;
    OperationMode.transceivermode = STANDARD_TRANSCEIVERMODE;
    OperationMode.txclk_out = STANDARD_TXCLOCK;
    OperationMode.oversampling = STANDARD_OVERSAMPLING;
    OperationMode.usetermchar = DISABLED;
    OperationMode.clockinversion = CLOCKINV_NONE;

    for (channel=0; channel<TP862_NUM_CHANS; channel++)
    {
        tp862_channel_init( pDCB->pCCB[channel] );
        set_operation_mode( pDCB->pCCB[channel], &OperationMode );
    }

    return 0;
}


int free_channel_memory( TP862_CCB* pCCB )
{

    if (pCCB->rx_irq)
    {
        pci_free_consistent(pCCB->dev, IRQ_RING_SIZE*sizeof(u32), pCCB->rx_irq, pCCB->rx_irq_dma);
        pCCB->rx_irq = NULL;
    }
    if (pCCB->tx_irq)
    {
        pci_free_consistent(pCCB->dev, IRQ_RING_SIZE*sizeof(u32), pCCB->tx_irq, pCCB->tx_irq_dma);
        pCCB->tx_irq = NULL;
    }

    if (pCCB->rx_fd)
    {
        pci_free_consistent(pCCB->dev, RX_DATA_QUEUE_SIZE*sizeof(TP862_RxFD), pCCB->rx_fd, pCCB->rx_fd_dma);
        pCCB->rx_fd = NULL;
    }
    if (pCCB->tx_fd)
    {
        pci_free_consistent(pCCB->dev, TX_DATA_QUEUE_SIZE*sizeof(TP862_TxFD), pCCB->tx_fd, pCCB->tx_fd_dma);
        pCCB->tx_fd = NULL;
    }

    if (pCCB->pInternalRingBuffer)
    {
        pci_free_consistent(pCCB->dev, sizeof(TP862_INTERNAL_RINGBUFFER), pCCB->pInternalRingBuffer, pCCB->InternalRingBuffer_dma);
        pCCB->pInternalRingBuffer = NULL;
    }

    if (pCCB->tx_buf)
    {
        pci_free_consistent(pCCB->dev, TX_DATA_QUEUE_SIZE*TP862_TX_BUFFER_SIZE*sizeof(unsigned char), pCCB->tx_buf, pCCB->tx_buf_dma);
        pCCB->tx_buf = NULL;
    }
    return 0;
}



int free_board_memory( TP862_DCB* pDCB )
{
    int channel;

    if (pDCB->cfg_irq)
    {
        pci_free_consistent(pDCB->dev, IRQ_RING_SIZE*sizeof(u32), pDCB->cfg_irq, pDCB->cfg_irq_dma);
        /*printk("board #%d: cfg_irq free.\n", pDCB->BoardNumber);*/
        pDCB->cfg_irq = NULL;
    }

    for (channel=0; channel<TP862_NUM_CHANS; channel++)
    {
        free_channel_memory( pDCB->pCCB[channel] );
    }

#ifdef TDRV001_DEBUG_VIEW
    printk(KERN_INFO "%s memory cleanup done (board #%d)\n", DEBUG_NAME, pDCB->BoardNumber);
#endif

    return 0;
}




static int tp862_init_one(struct pci_dev *dev, const struct pci_device_id  *id)
{
    unsigned long dscc4_base;
    unsigned long locbus_base;
    TP862_DCB*  pDCB;
    TP862_CCB*  pCCB;
    int i, channel, result, enable_result;
    static int BoardNumber = 0;
#if defined CONFIG_DEVFS_FS
    char dev_name[DEV_NAME_LEN];
#endif

    printk(KERN_INFO "\n%s Probe new device (vendor=0x%04X, device=0x%04X)\n",
        DEBUG_NAME, id->vendor, id->device);

    /*
    ** allocate kernel memory for device control block DCB
    */
    if (!(pDCB = kmalloc( sizeof(TP862_DCB), GFP_KERNEL ))) {
        printk(KERN_WARNING "\n%s unable to allocate memory for DCB\n", DEBUG_NAME);
        return -1;
    }
    memset( pDCB, 0, sizeof(TP862_DCB) );

    modules_found++;
#ifdef TDRV001_DEBUG_VIEW
    printk(KERN_INFO "%s modules found: %d\n", DEBUG_NAME, modules_found);
#endif

    /* add DCB structure to the list of known boards */
    list_add_tail(&pDCB->node, &tp862_board_root);

    /* make pci device available for access */
    enable_result = pci_enable_device(dev);

    /*
    ** try to occupy memory resource of the tpmc862 board
    */
    for (i=0; i<TP862_NUM_BAR; i++) {
        if ((pDCB->bar[i] = request_mem_region(pci_resource_start(dev, i), pci_resource_len(dev, i), "TPMC862")) == 0) {
            printk(KERN_WARNING "\n%s BAR[%d] memory resource already occupied!?\n", DEBUG_NAME, i);
            list_del(&pDCB->node);
            cleanup_device(pDCB);
            return -1;
        }
    }

  /* Cf errata DS5 p.2 */
  pci_write_config_byte(dev, PCI_LATENCY_TIMER, 0xf8);
  pci_set_master(dev);

    /* get physical addresses of used base address registers */
    dscc4_base = pci_resource_start(dev, 0);
    locbus_base = pci_resource_start(dev, 1);

    pDCB->dscc4_space  = (unsigned long)ioremap(pci_resource_start(dev, 0), pci_resource_len(dev, 0));
    pDCB->cpld_space = (unsigned long)ioremap(pci_resource_start(dev, 1), pci_resource_len(dev, 1));

    /* check memory space pointer */
    if (pDCB->cpld_space == 0) printk(KERN_WARNING "%s error: pDCB->cpld_space == NULL\n", DEBUG_NAME);
    if (pDCB->dscc4_space == 0) printk(KERN_WARNING "%s error: pDCB->dscc4_space == NULL\n", DEBUG_NAME);


    /*
    ** initialize DCB-structure
    */
    pDCB->BoardNumber = BoardNumber;
    pDCB->cfg_irq_index = 0;
    pDCB->cfg_irq_index_outofsync = TRUE;
    pDCB->dev = dev;
    pDCB->CfgIntOk = FALSE;


    /*
    ** create channel devices
    */
    for (channel=0; channel<TP862_NUM_CHANS; channel++)
    {

        /*
        ** allocate kernel memory for channel control block CCB
        */
        if (!(pCCB = kmalloc( sizeof(TP862_CCB), GFP_KERNEL ))) {
            printk(KERN_WARNING "\n%s unable to allocate memory for CCB #%d\n", DEBUG_NAME, channel);
            return -1;
        }
        pDCB->pCCB[channel] = pCCB;     /* store channel control block in device control block (link back) */

        /*
        ** create channel device (devfs or character device)
        */
#if defined CONFIG_DEVFS_FS
    sprintf(dev_name, "tdrv001_%i_%i", pDCB->BoardNumber, channel);
    pCCB->devfs_handle = devfs_register(NULL, dev_name, DEVFS_FL_AUTO_DEVNUM, 0, 0,
                                                  S_IFCHR | S_IRUGO | S_IWUGO, &tp862_fops,
                                                  (void *)pCCB);
        if (!pCCB->devfs_handle)
        {
            /* error during devfs register! */
            printk(KERN_WARNING "%s Board #%d: Error during devfs register!\n", DEBUG_NAME, BoardNumber);
            return -1;
        } else {
#ifdef TDRV001_DEBUG_VIEW
            printk( "%s device registered: %s\n", DEBUG_NAME, dev_name);
#endif
        }
#endif

        /*
        ** init CCB structure
        */
        pCCB->BoardNumber = BoardNumber;
        pCCB->ChannelNumber = channel;
        pCCB->dev = pDCB->dev;
        pCCB->minor = minor_count;
        pCCB->pDCB = pDCB;                  /* link back the other way round */
        pCCB->cpld_space = pDCB->cpld_space;
        pCCB->dscc4_space = pDCB->dscc4_space;
        pCCB->scc_regstart = pDCB->dscc4_space + SCC_START + channel*SCC_OFFSET;
        pCCB->rx_fd_index = 0;
        pCCB->tx_fd_index = 0;
        pCCB->rx_irq_index = 0;
        pCCB->tx_irq_index = 0;
        pCCB->tx_insert = 1;
        pCCB->tx_fd_last_index = 1;
        pCCB->tx_free = TX_DATA_QUEUE_SIZE - 1;
        pCCB->is_in_init_state = FALSE;
        pCCB->is_in_reset_state = TRUE;

        pCCB->tx_irq_index_outofsync = TRUE;
        pCCB->rx_irq_index_outofsync = TRUE;

        pCCB->rx_irq_index = 0;
        pCCB->tx_irq_index = 0;
        pCCB->rx_irq_index_outofsync = TRUE;
        pCCB->tx_irq_index_outofsync = TRUE;

        pCCB->tx_count_ok = 0;      /* statistics */
        pCCB->tx_count_error = 0;   /* statistics */

        pCCB->rx_data_queue_size = RX_DATA_QUEUE_SIZE;

        pCCB->OperationMode.baudrate = STANDARD_BAUDRATE;
        pCCB->OperationMode.commtype = STANDARD_COMMTYPE;
        pCCB->OperationMode.clockmode = STANDARD_CLOCKMODE;
        pCCB->OperationMode.dce_dte = STANDARD_DCEDTE;
        pCCB->OperationMode.transceivermode = STANDARD_TRANSCEIVERMODE;
        pCCB->OperationMode.txclk_out = STANDARD_TXCLOCK;
        pCCB->OperationMode.oversampling = STANDARD_OVERSAMPLING;
        pCCB->OperationMode.usetermchar = DISABLED;

        pCCB->DeviceWriteTimeout = TP862_TRANSMIT_TIMEOUT;
        pCCB->TxTimeRemaining = -1;

        pCCB->lock = SPIN_LOCK_UNLOCKED;
        sema_init( &pCCB->sema, 1 );

        init_waitqueue_head( &pCCB->rx_waitqueue );
        init_waitqueue_head( &pCCB->tx_waitqueue );

        /*
        ** init queue structure for kernels 2.4.x or  2.6.x
        */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
        /* Kernel 2.4.x */
        INIT_TQUEUE( &pCCB->taskQ, timeout_remove, (void*)pCCB );
#else
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
        /* Kernel 2.6.x */
        INIT_WORK( &pCCB->Work, timeout_remove, (void*)pCCB );
#else
        INIT_WORK( &pCCB->Work, timeout_remove );
#endif
#endif

        init_timer( &pCCB->timer );
        pCCB->timer.function = timeout_function;
        pCCB->timer.data = (unsigned long)pCCB;

        pCCB->tx_count_error = 0;
        pCCB->tx_count_ok    = 0;

        pCCB->TxTimeout = TP862_DEFAULT_TX_TIMEOUT;
        pCCB->RxTimeout = rx_timeout;             /* module parameter */

        pCCB->ReadOffset = 0;

        pCCB->discardCrcValue = DISCARD_CRCVALUE_IN_HDLC_MODE;

        pCCB->OpenCount = 0;

        minor_count++;

    }

    /*
    ** allocate memory for CFG interupt queue
    */
    pDCB->cfg_irq = (u32*)pci_alloc_consistent( pDCB->dev, IRQ_RING_SIZE*sizeof(u32), &pDCB->cfg_irq_dma );

    if (!pDCB->cfg_irq)
    {
        printk(KERN_WARNING "%s Error getting memory for CFG-IRQ\n", DEBUG_NAME);
        return -1;
    }

    /*
    ** allocate dma-memory for specific channel
    */
    for (channel=0; channel<TP862_NUM_CHANS; channel++)
    {
        /*
        ** RX/TX interrupt queues
        */
        pDCB->pCCB[channel]->rx_irq = (u32*)pci_alloc_consistent( pDCB->dev, IRQ_RING_SIZE*sizeof(u32), &pDCB->pCCB[channel]->rx_irq_dma );
        if (!pDCB->pCCB[channel]->rx_irq)
        {
            printk(KERN_WARNING "%s Error getting memory for RX-IRQ (board #%d, channel #%d)\n", DEBUG_NAME, BoardNumber, channel);
            return -1;
        }
        pDCB->pCCB[channel]->tx_irq = (u32*)pci_alloc_consistent( pDCB->dev, IRQ_RING_SIZE*sizeof(u32), &pDCB->pCCB[channel]->tx_irq_dma );
        if (!pDCB->pCCB[channel]->tx_irq)
        {
            printk(KERN_WARNING "%s Error getting memory for TX-IRQ (board #%d, channel #%d)\n", DEBUG_NAME, BoardNumber, channel);
            return -1;
        }

        /*
        ** RX/TX descriptor lists
        */
        pDCB->pCCB[channel]->rx_fd = (TP862_RxFD*)pci_alloc_consistent( pDCB->dev, RX_DATA_QUEUE_SIZE*sizeof(TP862_RxFD), &pDCB->pCCB[channel]->rx_fd_dma );
        if (!pDCB->pCCB[channel]->rx_fd)
        {
            printk(KERN_WARNING "%s Error getting memory for RX-FD (board #%d, channel #%d)\n", DEBUG_NAME, BoardNumber, channel);
            return -1;
        }
        pDCB->pCCB[channel]->tx_fd = (TP862_TxFD*)pci_alloc_consistent( pDCB->dev, TX_DATA_QUEUE_SIZE*sizeof(TP862_TxFD), &pDCB->pCCB[channel]->tx_fd_dma );
        if (!pDCB->pCCB[channel]->rx_fd)
        {
            printk(KERN_WARNING "%s Error getting memory for TX-FD (board #%d, channel #%d)\n", DEBUG_NAME, BoardNumber, channel);
            return -1;
        }

        /*
        ** RX/TX data buffer
        */
        pDCB->pCCB[channel]->pInternalRingBuffer = (TP862_INTERNAL_RINGBUFFER*)pci_alloc_consistent( pDCB->dev, sizeof(TP862_INTERNAL_RINGBUFFER), &pDCB->pCCB[channel]->InternalRingBuffer_dma );
        if (!pDCB->pCCB[channel]->pInternalRingBuffer)
        {
            printk(KERN_WARNING "%s Error getting memory for internal ringbuffer (board #%d, channel #%d)\n", DEBUG_NAME, BoardNumber, channel);
            return -1;
        }

        pDCB->pCCB[channel]->tx_buf = (unsigned char*)pci_alloc_consistent( pDCB->dev, TX_DATA_QUEUE_SIZE*TP862_TX_BUFFER_SIZE*sizeof(unsigned char), &pDCB->pCCB[channel]->tx_buf_dma );
        if (!pDCB->pCCB[channel]->rx_fd)
        {
            printk(KERN_WARNING "%s Error getting memory for TX-BUF (board #%d, channel #%d)\n", DEBUG_NAME, BoardNumber, channel);
            return -1;
        }

        pDCB->pCCB[channel]->tx_fd_first_dma = pDCB->pCCB[channel]->tx_fd_dma;
        pDCB->pCCB[channel]->tx_fd_last_dma  = pDCB->pCCB[channel]->tx_fd_dma;
        pDCB->pCCB[channel]->tx_fd_last_index = 1;


    }

    /*
    ** register interrupt service routine
    */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
  if (request_irq(dev->irq, &tp862_isr, SA_SHIRQ, DEBUG_NAME, pDCB)){
#else
  if (request_irq(dev->irq, &tp862_isr, IRQF_SHARED, DEBUG_NAME, pDCB)){
#endif
    printk(KERN_WARNING "%s: IRQ %d busy\n", DEBUG_NAME, dev->irq);
  }

    /* configure module-hardware parameters */
    result = init_hardware( pDCB );
    if (result < 0)
    {
        printk(KERN_WARNING "%s Error during hardware init. Aborting.\n", DEBUG_NAME);
        cleanup_device( pDCB );
        return -1;
    }

    BoardNumber++;

    return 0;
}



static void tp862_remove_one(struct pci_dev *dev)
{
    printk(KERN_INFO "\n%s Remove device (vendor=0x%04X, device=0x%04X)\n",
        DEBUG_NAME, dev->vendor, dev->device);
}



/*****************************************************************************
*
*  cleanup_device - release resources allocated by the specified carrier board
*
*****************************************************************************/
static void cleanup_device(  TP862_DCB* pDCB )

{
    int  i;
    int channel;


    /* set channels into reset-state */
    for (channel=0; channel<TP862_NUM_CHANS; channel++)
    {
        pDCB->pCCB[channel]->tx_fd_first_dma = pDCB->pCCB[channel]->tx_fd_dma;
        pDCB->pCCB[channel]->tx_fd_last_dma = pDCB->pCCB[channel]->tx_fd_dma;
        pDCB->pCCB[channel]->tx_fd_last_index = 1;
        channel_reset_rxtx( pDCB->pCCB[channel] );
    }

    for (i=0; i<TP862_NUM_BAR; i++) {
        if (pDCB->bar[i]) release_mem_region(pci_resource_start(pDCB->dev, i), pci_resource_len(pDCB->dev, i));
    }

    free_irq( pDCB->dev->irq, pDCB );
    free_board_memory( pDCB );

    if (pDCB->cpld_space) iounmap((unsigned char*)pDCB->cpld_space);
    if (pDCB->dscc4_space) iounmap((unsigned long*)pDCB->dscc4_space);

    pci_disable_device( pDCB->dev );

    /*
    ** free channel resources
    */
    for (channel=0; channel<TP862_NUM_CHANS; channel++)
    {
        /*
        ** kill tasklet for kernel 2.6.x
        */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
        /*tasklet_kill( &pDCB->pCCB[channel]->tlet );*/
#endif


#ifdef CONFIG_DEVFS_FS
      devfs_unregister(pDCB->pCCB[channel]->devfs_handle);
#endif
        kfree( pDCB->pCCB[channel] );
    }

    kfree(pDCB);
}



/*
**  The following definitions and the array tpmc862_pci_table[] are describing the PCI
**  devices we are interested in.
*/
#define TPMC862_VENDOR_ID       0x110A
#define TPMC862_DEVICE_ID       0x2102
#define TPMC862_SUBVENDOR_ID    0x0000
#define TPMC862_SUBDEVICE_ID    0x0000



static struct pci_device_id tpmc862_pci_table[] = {
    {TPMC862_VENDOR_ID, TPMC862_DEVICE_ID, TPMC862_SUBVENDOR_ID, TPMC862_SUBDEVICE_ID, 0, 0, 0},
    {0, }
};


MODULE_DEVICE_TABLE(pci, tpmc862_pci_table);


/*
**  The 'pci_driver' structure describes our driver and provide the PCI subsystem
**  with callback function pointer. The id_table contains PCI device ID's of devices we
**  are interested in.
*/
static struct pci_driver tp862_driver = {
    name:       DRIVER_NAME,
    probe:      tp862_init_one,
    remove:     tp862_remove_one,
    id_table:   tpmc862_pci_table,
};



/*****************************************************************************
*
*  driver_init - module initialization function
*
*  This module will be called during module initialization. The init function
*  allocates necessary resources and initializes internal data structures.
*
*****************************************************************************/
static int __init tcp862_init(void)
{
    int result, tcp862_minor;
    struct pci_dev *dev;

    printk(KERN_INFO "%s version %s (%s)\n", DRIVER_NAME, DRIVER_VERSION, DRIVER_REVDATE);


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
    dev = pci_find_device(TPMC862_VENDOR_ID,TPMC862_DEVICE_ID,NULL);
#else
    dev = pci_get_device(TPMC862_VENDOR_ID,TPMC862_DEVICE_ID,NULL);
#endif

    if (!dev) {
      printk(KERN_ALERT "HACK: device not found\n");
      return -ENODEV;
    }


    /*
    * Register the major device
    */
    result = register_chrdev(tcp862_major, "tcp862", &tp862_fops);

    if (result < 0) {
        printk(KERN_WARNING "%s can't get major %d\n", DEBUG_NAME,
          tcp862_major);

        return result;
    }
    if (tcp862_major == 0) tcp862_major = result; /* dynamic */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    tcp862_class = class_create(THIS_MODULE, TCP862_DEVCLASS);
    for(tcp862_minor = 0; tcp862_minor < TP862_NUM_CHANS; tcp862_minor++) {
      device_create(tcp862_class, NULL, MKDEV(tcp862_major, tcp862_minor),
        "%s%d", TCP862_DEVNAME, tcp862_minor);
    }
#endif

    INIT_LIST_HEAD(&tp862_board_root);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
    return pci_module_init(&tp862_driver);
#else
    return pci_register_driver(&tp862_driver);
#endif
}

/*****************************************************************************
*
*  driver_cleanup - module cleanup
*
*  This module will be called before module removal. The cleanup function
*  free all allocated resources.
*
*****************************************************************************/
static void __exit tcp862_exit(void)
{
    struct list_head  *ptr, *next;
    TP862_DCB* pDCB = NULL;
    int tcp862_minor;

    for (ptr=tp862_board_root.next; ptr != &tp862_board_root; ) {

        next = ptr->next;
        pDCB = list_entry(ptr, TP862_DCB, node);

        /*  remove the board info structure from the list */
        list_del(ptr);

        /*  release all allocated resource and free the memory (kfree) allocated by the info structure */
        cleanup_device( pDCB );

        ptr = next;
    }

    //printk(KERN_INFO "%s TCP862 driver removed\n", DEBUG_NAME);
    pci_unregister_driver(&tp862_driver);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    for(tcp862_minor = 0; tcp862_minor < TP862_NUM_CHANS; tcp862_minor++)
      device_destroy(tcp862_class, MKDEV(tcp862_major, tcp862_minor));
    class_unregister(tcp862_class);
    class_destroy(tcp862_class);
#endif

    unregister_chrdev(tcp862_major, "tcp862");
    return;
}

int evaluate_buildoption_register( TP862_DCB* pDCB )
{
    unsigned char bopt;
    BUILDOPTION_STRUCT BuildOption[16] = {
        {TP862_BUILDOPTION_00_XTAL1, TP862_BUILDOPTION_00_XTAL2},
        {TP862_BUILDOPTION_01_XTAL1, TP862_BUILDOPTION_01_XTAL2},
        {TP862_BUILDOPTION_02_XTAL1, TP862_BUILDOPTION_02_XTAL2},
        {TP862_BUILDOPTION_03_XTAL1, TP862_BUILDOPTION_03_XTAL2},
        {TP862_BUILDOPTION_04_XTAL1, TP862_BUILDOPTION_04_XTAL2},
        {TP862_BUILDOPTION_05_XTAL1, TP862_BUILDOPTION_05_XTAL2},
        {TP862_BUILDOPTION_06_XTAL1, TP862_BUILDOPTION_06_XTAL2},
        {TP862_BUILDOPTION_07_XTAL1, TP862_BUILDOPTION_07_XTAL2},
        {TP862_BUILDOPTION_08_XTAL1, TP862_BUILDOPTION_08_XTAL2},
        {TP862_BUILDOPTION_09_XTAL1, TP862_BUILDOPTION_09_XTAL2},
        {TP862_BUILDOPTION_10_XTAL1, TP862_BUILDOPTION_10_XTAL2},
        {TP862_BUILDOPTION_11_XTAL1, TP862_BUILDOPTION_11_XTAL2},
        {TP862_BUILDOPTION_12_XTAL1, TP862_BUILDOPTION_12_XTAL2},
        {TP862_BUILDOPTION_13_XTAL1, TP862_BUILDOPTION_13_XTAL2},
        {TP862_BUILDOPTION_14_XTAL1, TP862_BUILDOPTION_14_XTAL2},
        {TP862_BUILDOPTION_15_XTAL1, TP862_BUILDOPTION_15_XTAL2},
    };

    bopt = READ_REGISTER_UCHAR( (unsigned char*)(pDCB->cpld_space + TP862_CPLD_BUILDOPTION_REG) );

    if (bopt > 0x0F)
    {
        printk(KERN_WARNING "%s Invalid build option! Using Xtal values for BuildOption 00.\n", DEBUG_NAME);
        bopt = 0;
    }

    pDCB->xtal1_hz = BuildOption[bopt].xtal1_hz;
    pDCB->xtal2_hz = BuildOption[bopt].xtal2_hz;
#ifdef TDRV001_DEBUG_VIEW
    printk(KERN_INFO "%s --------------------------------------------------\n", DEBUG_NAME);
    printk(KERN_INFO "%s BuildOption %d: Xtal1 = %ld; Xtal2 = %ld\n", DEBUG_NAME, bopt, pDCB->xtal1_hz, pDCB->xtal2_hz );
    printk(KERN_INFO "%s --------------------------------------------------\n", DEBUG_NAME);
#endif
    return 0;
}

int setup_channel_rx_descriptor_list( TP862_CCB* pCCB )
{
    unsigned long descriptor;
    TP862_RxFD* pRxFD;
    TP862_RxFD* pRxFD_base_virt;
    TP862_RxFD* pRxFD_base_dma;
    TP862_INTERNAL_RINGBUFFER* pRingBuf;
    TP862_INTERNAL_RINGBUFFER* pRingBufDma;

    /*
    **  init single descriptors and build up a linked list
    */
    pRxFD_base_virt = (TP862_RxFD*)pCCB->rx_fd;
    pRxFD_base_dma  = (TP862_RxFD*)pCCB->rx_fd_dma;

    pRingBufDma     = (TP862_INTERNAL_RINGBUFFER*)pCCB->InternalRingBuffer_dma;
    pRingBuf        = (TP862_INTERNAL_RINGBUFFER*)pCCB->pInternalRingBuffer;


    for (descriptor=0; descriptor<pCCB->rx_data_queue_size; descriptor++)
    {
        pRxFD = &pRxFD_base_virt[descriptor];
        pRxFD->data     = __le32_to_cpu( (unsigned long)pRingBufDma->entry[descriptor].pData );
        pRxFD->end      = 0;
        pRxFD->state1   = __le32_to_cpu( (unsigned long)(RECEIVE_LENGTH_LIMIT << 16) | HiDesc );
        pRxFD->state2   = 0;
        pRxFD->next     = __le32_to_cpu( (unsigned long)&pRxFD_base_dma[(descriptor+1) % pCCB->rx_data_queue_size] );
        /* init rx buffer content */
        memset( pRingBuf->entry[descriptor].pData, RECEIVE_LENGTH_LIMIT, ('a'+descriptor) );
    }
    return TRUE;
}


int setup_channel_tx_descriptor_list( TP862_CCB* pCCB )
{
    unsigned long descriptor;
    TP862_TxFD* pTxFD;
    TP862_TxFD* pTxFD_base_virt;
    TP862_TxFD* pTxFD_base_dma;
    unsigned long data_addr_dma;
    unsigned long data_addr_virt;

    /*
    **  init single descriptors and build up a linked list
    */
    data_addr_dma = (unsigned long)pCCB->tx_buf_dma;
    data_addr_virt = (unsigned long)pCCB->tx_buf;
    pTxFD_base_virt = (TP862_TxFD*)pCCB->tx_fd;
    pTxFD_base_dma  = (TP862_TxFD*)pCCB->tx_fd_dma;
    for (descriptor=0; descriptor<TX_DATA_QUEUE_SIZE; descriptor++)
    {
        pTxFD = &pTxFD_base_virt[descriptor];
        pTxFD->data     = __le32_to_cpu( data_addr_dma );
        pTxFD->complete = 0;
        pTxFD->state    = __le32_to_cpu( 0xc0040000 );   /* 1 DWORD standard length */
        pTxFD->size = 0;
        pTxFD->txbuf_virt = 0;
        pTxFD->txbuf_dma = 0;
        pTxFD->next     = __le32_to_cpu( (unsigned long)&pTxFD_base_dma[(descriptor+1) % TX_DATA_QUEUE_SIZE] );
        /* init rx buffer content */
        memset( (unsigned char*)data_addr_virt, ('a'+descriptor), TP862_TX_BUFFER_SIZE );
        data_addr_dma += TP862_TX_BUFFER_SIZE;
        data_addr_virt += TP862_TX_BUFFER_SIZE;
    }
    pCCB->tx_insert = 1;

    return TRUE;
}


int remove_descriptor( TP862_CCB* pCCB, unsigned long rempos )
{
    unsigned long pos, src_pos, dest_pos;
    int down_result;

    TP862_TxFD* pTxFD;
    TP862_TxFD* pTxFD_dma;
    TP862_TxFD* pTxFD_src;
    TP862_TxFD* pTxFD_dest;


    /*
    ** acquire mutex
    */
    down_result = down_interruptible( &pCCB->sema );

    pTxFD = (TP862_TxFD*)&pCCB->tx_fd[rempos];


#ifdef TDRV001_DEBUG_VIEW
    printk(KERN_INFO "%s --- Cancelling descriptor #%ld ---\n", DEBUG_NAME, rempos);
#endif

    /*
    ** channel reset
    */
    channel_reset_rxtx( pCCB );

    /*
    ** free allocated memory
    */
    if (pTxFD->txbuf_virt && pTxFD->size && pTxFD->txbuf_dma)
    {
        pci_free_consistent(pCCB->dev, pTxFD->size, (unsigned char*)pTxFD->txbuf_virt, (dma_addr_t)pTxFD->txbuf_dma);
    }


    /*
    ** copy descriptor contents
    */
    pos = rempos;
    while (pos != pCCB->tx_insert)
    {
        pTxFD_dest = (TP862_TxFD*)&pCCB->tx_fd[pos];
        pTxFD_src = (TP862_TxFD*)&pCCB->tx_fd[(pos+1)%TX_DATA_QUEUE_SIZE];

        src_pos = (pos+1)%TX_DATA_QUEUE_SIZE;
        dest_pos = pos;
#ifdef TDRV001_DEBUG_VIEW
        printk("%s Moving #%ld -> #%ld\n", DEBUG_NAME, src_pos, dest_pos);
#endif
        pTxFD_dest->complete    = pTxFD_src->complete;
        pTxFD_dest->data        = pTxFD_src->data;
        pTxFD_dest->finished    = pTxFD_src->finished;
        pTxFD_dest->size        = pTxFD_src->size;
        pTxFD_dest->state       = pTxFD_src->state;
        pTxFD_dest->txbuf_dma   = pTxFD_src->txbuf_dma;
        pTxFD_dest->txbuf_virt  = pTxFD_src->txbuf_virt;

        pos = (pos + 1) % TX_DATA_QUEUE_SIZE;

    }

    /*
    ** set new first/last descriptor
    */
    pTxFD_dma = (TP862_TxFD*)pCCB->tx_fd_dma;
    pCCB->tx_insert = (pCCB->tx_insert + TX_DATA_QUEUE_SIZE - 1)%TX_DATA_QUEUE_SIZE;
    pCCB->tx_fd_last_dma = (unsigned long)&pTxFD_dma[pCCB->tx_insert];
#ifdef TDRV001_DEBUG_VIEW
    printk(KERN_INFO "%s New last descriptor: #%ld\n", DEBUG_NAME, pCCB->tx_insert);
#endif
    pCCB->tx_free++;

    pCCB->tx_fd_first_dma  = (unsigned long)&pTxFD_dma[pCCB->tx_fd_index];
    pCCB->tx_fd_last_dma   = (unsigned long)&pTxFD_dma[pCCB->tx_insert];

    /* clear new insert-descriptor */
    pTxFD = (TP862_TxFD*)&pCCB->tx_fd[pCCB->tx_insert];
    pTxFD->data = __le32_to_cpu( pCCB->tx_buf_dma );
    pTxFD->state    = __le32_to_cpu( 0xc0040000 );   /* 1 DWORD standard length */
    pTxFD->size = 0;
    pTxFD->txbuf_virt = 0;
    pTxFD->txbuf_dma = 0;


    if (pCCB->tx_free > TX_DATA_QUEUE_SIZE)
    {
        /*printk("Correcting descriptor count\n");*/
        pCCB->tx_free = TX_DATA_QUEUE_SIZE;
    }

    /*
    ** restart transfer
    */
    channel_init_rxtx( pCCB );

    /*
    ** release mutex
    */
    up ( &pCCB->sema );

    return 0;
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
void timeout_remove( void* data )
{
    TP862_CCB* pCCB = (TP862_CCB*)data;
#else
void timeout_remove( struct work_struct* ws )
{
    TP862_CCB* pCCB = container_of(ws, TP862_CCB, Work);
#endif

#ifdef TDRV001_DEBUG_VIEW
    printk(KERN_INFO "%s ---------- timeout_remove started -------------\n", DEBUG_NAME);
#endif
    remove_descriptor( pCCB, pCCB->tx_fd_index );

    pCCB->TxTimeRemaining = pCCB->DeviceWriteTimeout;

    /*
    ** update error-count
    */
    pCCB->tx_count_error++;

    return;
}


void timeout_function( unsigned long data )
{
    TP862_CCB* pCCB = (TP862_CCB*)data;

    if (pCCB->TxTimeRemaining > -1)
    {

        /*printk("Timeout(%d,%d): %d seconds remaining\n", pCCB->BoardNumber, pCCB->ChannelNumber, pCCB->TxTimeRemaining);*/
        /*
        ** let's see if we have a timeout for a transmit descriptor
        */
        if (--pCCB->TxTimeRemaining < 0)
        {
#ifdef TDRV001_DEBUG_VIEW
            printk(KERN_INFO "%s Timeout(%d,%d): Timeout happened.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
#endif

            if (pCCB->tx_free < TX_DATA_QUEUE_SIZE)
            {

                /*
                ** cancel the corresponding write operation (first entry)
                ** done within a separate task (bottom half) because we are running on interrupt level.
                */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
                /* Kernel 2.4.x */
                schedule_task( &pCCB->taskQ );
#else
                /* Kernel 2.6.x */
                schedule_work( &pCCB->Work );
#endif
                pCCB->TxTimeRemaining = pCCB->DeviceWriteTimeout;
            } else {
#ifdef TDRV001_DEBUG_VIEW
                printk(KERN_INFO "%s Timeout(%d,%d): No entry available for cancel.\n", DEBUG_NAME, pCCB->BoardNumber, pCCB->ChannelNumber);
#endif
                pCCB->TxTimeRemaining = -1;
            }

        }
    } else {
        /* timeout function disabled */
    }

    pCCB->timer.expires = jiffies + HZ; /* restart every second */
    add_timer( &pCCB->timer );

}


void stop_receiver( TP862_CCB* pCCB )
{
    /* stop receiver */
    pCCB->scc_regs_local[CCR2>>2] &= ~(RxActivate);
    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->scc_regstart + CCR2), pCCB->scc_regs_local[CCR2>>2] );
}


void start_receiver( TP862_CCB* pCCB )
{
    /* start receiver */
    pCCB->scc_regs_local[CCR2>>2] |= (RxActivate);
    WRITE_REGISTER_ULONG( (unsigned long*)(pCCB->scc_regstart + CCR2), pCCB->scc_regs_local[CCR2>>2] );
}



module_init(tcp862_init);
module_exit(tcp862_exit);
