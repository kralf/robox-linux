/*---------------------------------------------------------------------------
** Inova Computers GmbH. Sudetenstr 5, 87600 Kaufbeuren
**
** (c) Copyright 1998-2000. All Rights Reserved
**
** Date: Sep.26.2000
**
** Operating System: Linux
**
** Compiler : Gnu C/C++ Compiler 
**
**---------------------------------------------------------------------------
** Project: ICP-MULTI
** Title:   ICP-MULTI Linux Driver
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** DESCRIPTION
**-------------------------------------------------------------------------*/
/* This module contains the driver code of the ICP-MULTI CompactPCI
 * Digital Input Adapter
 */


 
/*---------------------------------------------------------------------------
** MODIFICATION HISTORY
**-------------------------------------------------------------------------*/
/*
modification history
-------------------- 
000,26sep00,bt   created
00a,27oct00,bt   changed mio_analog_out_init()'s param

*/
 
 
/*---------------------------------------------------------------------------
** PRAGMA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** INCLUDE
**-------------------------------------------------------------------------*/
#ifndef __KERNEL__
  #define __KERNEL__
#endif

#define __NO_VERSION__ /* don't define kernel_verion in module.h */
#include <linux/module.h> /* MOD_DEC_USE_COUNT */

#include <linux/kernel.h> /* printk()... */
#include <linux/malloc.h> /* kmalloc() */

#include <linux/types.h>  /* size_t */

#include <asm/string.h>

#include <icp_drv_gen.h>

/*
** local module headers
*/
#include "../h/icp_mio.h"
#include "../h/icp_mio.msg"

/*---------------------------------------------------------------------------
** DEFINE
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** DEFINE
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** TYPEDEF
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** DATA
**-------------------------------------------------------------------------*/
static ICP_MIO_STRUCT*  pMioDevices    = NULL;
static unsigned int     nrOfMios       = 0;
static BOOL             mioListInitialized = FALSE;

/*---------------------------------------------------------------------------
** FORWARD DECLARATIONS
**-------------------------------------------------------------------------*/
static STATUS mioInitCallback(void* param);
static int    mio_linux_fs_close(LINUX_FS_MIO* pFs);
static int    mio_linux_fs_open(LINUX_FS_MIO* pFs);
static void   mio_linux_fs_cleanup(LINUX_FS_MIO* pFs);
static STATUS mio_linux_fs_init(LINUX_FS_MIO* pFs, int index);
static int    mio_open(struct inode* p_inode, struct file* p_file);
static int    mio_close(struct inode* p_inode, struct file* p_file);
static int    mio_ioctl(struct inode* p_inode, struct file* p_file, 
                        unsigned int cmd, unsigned long arg);

//static STATUS icpMioNumberTest(int number);
/*---------------------------------------------------------------------------
** STATIC FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* mio_linux_fs_init - Initialize the Linux file system object
*
* Description
*
* Parameters:
*         
*
* RETURNS: error code, in case of an error (Out of memory, No device found)
*          OK   , if everything was OK.
*   
* SEE ALSO: 
******************************************************************************/
static STATUS mio_linux_fs_init(LINUX_FS_MIO* pFs, int index)
{
  int  result;

  pFs->state = unregistered;

  /*
  ** fill out the file_operations structure
  */
  pFs->mio_fops.llseek              = NULL; /* NOT USED for the board */
  pFs->mio_fops.read                = NULL; /* NOT USED for the board */
  pFs->mio_fops.write               = NULL; /* NOT USED for the board */
  pFs->mio_fops.readdir             = NULL; /* NOT USED for the board */
  pFs->mio_fops.poll                = NULL; /* NOT USED for the board */
  pFs->mio_fops.ioctl               = mio_ioctl; /* ioct() for setting general 
                                                 ** params 
                                                 */
  pFs->mio_fops.mmap                = NULL; /* NOT USED for the board */
  pFs->mio_fops.open                = mio_open; /* general open routine redirect 
                                                ** the calls to the different
                                                ** channels
                                                */
  pFs->mio_fops.flush               = NULL; /* NOT USED for the board */
  pFs->mio_fops.release             = mio_close; /* general close routine, must 
                                                 ** be called by every channel's 
                                                 ** close routine
                                                 */
  pFs->mio_fops.fsync               = NULL; /* NOT USED for the board */
  pFs->mio_fops.fasync              = NULL; /* NOT USED for the board */
  pFs->mio_fops.check_media_change  = NULL; /* NOT USED for the board */
  pFs->mio_fops.revalidate          = NULL; /* NOT USED for the board */
  pFs->mio_fops.lock                = NULL; /* NOT USED for the board */

  /*
  ** register a char device for the founded MIO card
  ** device name is mioNr (mio0, mio1...), major number dynamically assigned
  */
  sprintf(pFs->devName, "%s%i",ICP_MIO_DEV_NAME,index);
  result = register_chrdev(0, pFs->devName, &(pFs->mio_fops));
  if (result < 0)
  {
    icpErrorMsg(ICP_MIO_NAME, ICP_MIO_CANT_REG_CHRDEV);
    return result;
  }
  else
  {
    /*
    ** save the major number and device name
    */
    pFs->major_number = result;
  }
  pFs->state = registered;

  return OK;
}


/******************************************************************************
*
* mio_linux_fs_cleanup - Cleanup the Linux file system object
*
* Description
*
* Parameters:
*         
*
* RETURNS: -
*   
* SEE ALSO: 
******************************************************************************/
static void mio_linux_fs_cleanup(LINUX_FS_MIO* pFs)
{
  if (pFs->state != registered)
  {
    icpErrorMsg(ICP_MIO_NAME, ICP_MIO_FS_CLEANUP_ERR);
  }
  /*
  ** unregister the char device for the MIO card
  */
  unregister_chrdev(pFs->major_number, pFs->devName);
  pFs->major_number = -1;
  strcpy(pFs->devName, "");
  pFs->state = unregistered;
}


/******************************************************************************
*
* mio_linux_fs_open - Opens the Linux file system object
*
* Description
*
* Parameters:
*         
*
* RETURNS: -
*   
* SEE ALSO: 
******************************************************************************/
static int mio_linux_fs_open(LINUX_FS_MIO* pFs)
{
  switch (pFs->state)
  {
    case unregistered:
    {
      icpErrorMsg(ICP_MIO_NAME, ICP_MIO_FS_OPEN_ERR1);
      return -ENODEV; /* No such device */
    }
    case opened:
    {
      icpErrorMsg(ICP_MIO_NAME, ICP_MIO_FS_OPEN_ERR2);
      return -EMFILE; /* Too many open files */
    }
    case registered:
    {
      /*
      ** open the file
      */
      pFs->state = opened;
      MOD_INC_USE_COUNT;
      return 0; /* success */
    }
    default:
    {
      /* FATAL Error */
      return -EPERM; /* operation not permitted */
    }
  }
}


/******************************************************************************
*
* mio_linux_fs_close - Closes the Linux file system object
*
* Description
*
* Parameters:
*         
*
* RETURNS: -
*   
* SEE ALSO: 
******************************************************************************/
static int mio_linux_fs_close(LINUX_FS_MIO* pFs)
{
  switch (pFs->state)
  {
    case unregistered:
    {
      icpErrorMsg(ICP_MIO_NAME, ICP_MIO_FS_CLOSE_ERR1);
      return -ENODEV; /* No such device */
    }
    case opened:
    {
      /*
      ** Close the file
      */
      pFs->state = registered;
      break; /* success */
    }
    case registered:
    {
      icpErrorMsg(ICP_MIO_NAME, ICP_MIO_FS_CLOSE_ERR2);
      return -ENOENT; /* Mo such file or directory */
    }
    default:
    {
      /* FATAL Error */
      return -EPERM; /* operation not permitted */
    }
  }
  MOD_DEC_USE_COUNT;
  return 0;
}
/******************************************************************************
*
* mioInitCallback - short descr
*
* Description
*
* Parameters:
*         
*
* RETURNS: ERROR, in case of an error (Out of memory, No device found)
*          OK   , if everything was OK.
*   
* SEE ALSO: 
******************************************************************************/
static STATUS mioInitCallback(void* param)
{
  static int      index = 0; /* Keep track of which card is initilized (1st, 2nd...) */

  ICP_MIO_STRUCT* pMio = (ICP_MIO_STRUCT*)param;
  struct pci_dev* pPci = pMio->pci.pci_min.pciDev;
  int             status;
  u8*             mem_base;


  pMio->state = uninitialized;

  /*
  ** Init sequence (important this order)
  ** 1. Pci init
  ** 2. EEPROM init
  ** 3. Linux file system init
  ** 4. Inetrrupt init
  ** 5. analog input init
  ** 6. analog output init
  ** 7. digital input init
  ** 8. digital output init
  ** 9. counter init
  */
  /*
  ** 1. PCI parameter initialization
  */
  I_CHK_STATUSGMS(mio_pci_init(&(pMio->pci), (void**)(&mem_base)), 
                  Error1, ICP_MIO_NAME, ICP_MIO_PCI_INIT_ERR);
  /*
  ** 2. Initialization of the EEPROM
  */
  I_CHK_STATUSGMS(mio_eeprom_init(&(pMio->eeprom), pPci), Error2,
                  ICP_MIO_NAME, ICP_MIO_EEPROM_INIT_ERR);
  /*
  ** 3. Initialization of the Linux file system struct
  */
  I_CHK_STATUSGMS(mio_linux_fs_init(&(pMio->fs), index), Error3,
                  ICP_MIO_NAME, ICP_MIO_FS_INIT_ERR);
  /*
  ** 4. Interrupt Initialization
  */
  I_CHK_STATUSGMS(mio_interrupt_init(&(pMio->irq), pPci, pMio->fs.devName), 
                  Error4, ICP_MIO_NAME, ICP_MIO_IRQ_INIT_ERR);
  /*
  ** 5. Initialization of the analog inputs
  */
  I_CHK_STATUSGMS(mio_analog_in_init(&(pMio->a_i), mem_base, 
                                     (void*)&(pMio->irq), 
                                     pMio->eeprom.fact_calib),
                  Error5, ICP_MIO_NAME, ICP_MIO_AI_INIT_ERR);
  /*
  ** 6. Initialization of the analog outputs
  */
  I_CHK_STATUSGMS(mio_analog_out_init(&(pMio->a_o), mem_base, 
                                      (void*)&(pMio->irq), 
                                      (void*)&(pMio->a_i.adc)),
                  Error6, ICP_MIO_NAME, ICP_MIO_AO_INIT_ERR);
  /*
  ** 7. Initialization of the digital inputs
  */
  I_CHK_STATUSGMS(mio_digital_in_init(&(pMio->d_i), mem_base, 
                                      (void*)&(pMio->irq)),
                  Error7, ICP_MIO_NAME, ICP_MIO_DI_INIT_ERR);
  /*
  ** 8. Initialization of the digital outputs
  */
  I_CHK_STATUSGMS(mio_digital_out_init(&(pMio->d_o), mem_base, 
                                       (void*)&(pMio->irq)),
                  Error8, ICP_MIO_NAME, ICP_MIO_DO_INIT_ERR);
  /*
  ** 9. Initialization of the counters
  */
  I_CHK_STATUSGMS(mio_counter_init(&(pMio->ctr), mem_base, 
                                   (void*)&(pMio->irq)),
                  Error9, ICP_MIO_NAME, ICP_MIO_CTR_INIT_ERR);
  /*
  ** be prepared for the next card
  */
  index++;

  pMio->state = initialized;

#ifdef DEBUG_MIO
  printk("<1> mio init OK\n");
#endif
  return OK;

Error9:
  mio_digital_out_cleanup(&(pMio->d_o));

Error8:
  mio_digital_in_cleanup(&(pMio->d_i));

Error7:
  mio_analog_out_cleanup(&(pMio->a_o));

Error6:
  mio_analog_in_cleanup(&(pMio->a_i));

Error5:
  mio_interrupt_cleanup(&(pMio->irq));

Error4:
  mio_linux_fs_cleanup(&(pMio->fs));

Error3:
  mio_eeprom_cleanup(&(pMio->eeprom));

Error2:
  mio_pci_cleanup(&(pMio->pci));

Error1:
  return status;
}

/******************************************************************************
*
* mioCleanupCallback - cleanup routine for resource deallocation
*
* Description
*
* Parameters:
*         
*
* RETURNS: -
*   
* SEE ALSO: 
******************************************************************************/
static void mioCleanupCallback(void* param)
{
  ICP_MIO_STRUCT* pMio = (ICP_MIO_STRUCT*)param;

  /*
  ** cleanup sequence (important this order)
  ** 1. counter 
  ** 2. digital output 
  ** 3. digital input 
  ** 4. analog output 
  ** 5. analog input 
  ** 6. Inetrrupt 
  ** 7. Linux file system 
  ** 8. Pci 
  */
  mio_counter_cleanup(&(pMio->ctr));
  mio_digital_out_cleanup(&(pMio->d_o));
  mio_digital_in_cleanup(&(pMio->d_i));
  mio_analog_out_cleanup(&(pMio->a_o));
  mio_analog_in_cleanup(&(pMio->a_i));
  mio_interrupt_cleanup(&(pMio->irq));
  mio_linux_fs_cleanup(&(pMio->fs));
  mio_pci_cleanup(&(pMio->pci));

  pMio->state = uninitialized;

#ifdef DEBUG_MIO
  printk("<1> mio deinit OK\n");
#endif
}
/******************************************************************************
*
* mio_open - open routine for the mio board, this routine is called by the FS
*            to open one of the MIO board's channels
* Description

*
* Parameters:
*         
*
* RETURNS: -
*   
* SEE ALSO: 
******************************************************************************/
static int mio_open(struct inode* p_inode, struct file* p_file)
{
  ICP_MIO_STRUCT* p_mio = NULL;
  int major_nr          = MAJOR(p_inode->i_rdev);
  int minor_nr          = MINOR(p_inode->i_rdev) - ICP_MIO_MINOR_OFFS;
  int ret_val;

  /*
  ** check if it is a major_nr for us
  ** iterate the list
  */
#ifdef DEBUG_MIO
  printk("<1> mio open called OK\n");
#endif

  for (p_mio = pMioDevices; p_mio != NULL; p_mio = p_mio->pci.pci_min.next)
  {
    if( p_mio->fs.major_number == major_nr )
    {
      /*
      ** we've found it
      */
      break;
    }
  }
  if (p_mio == NULL)
  {
    /*
    ** no such major number
    */
    return -ENODEV; /* No scuh device */
  }

  if (minor_nr < 0)
  {
    /*
    ** bad param
    */
    icpErrorMsg(ICP_MIO_NAME, ICP_MIO_OPEN_ERR);
    return -ENODEV;
  }
  else if ( minor_nr == 0 )
  {
    /*
    ** open the MIO board
    */
#ifdef DEBUG_MIO
  printk("<1> open the mio board\n");
#endif
    if ( (ret_val = mio_linux_fs_open(&(p_mio->fs))) != 0 )
    {
      return ret_val; /* error opening */
    }
    else
    {
      /*
      ** set the private data field
      */
      p_file->private_data = (void*)p_mio;
      return ret_val;
    }
  }
  else
  {
    /*
    ** try the different open routines
    */
    if ( (ret_val = mio_analog_in_open(p_inode, p_file, &(p_mio->a_i))) != 
                                                             ICP_MIO_BAD_MINOR)
    {
#ifdef DEBUG_MIO
  printk("<1> mio _analog_in returned %i\n", ret_val);
#endif
      return ret_val;
    }
    else if ( (ret_val = mio_analog_out_open(p_inode, p_file, &(p_mio->a_o))) != 
                                                             ICP_MIO_BAD_MINOR)
    {
#ifdef DEBUG_MIO
  printk("<1> mio _analog_out returned %i\n", ret_val);
#endif
      return ret_val;
    }
    else if ( (ret_val = mio_digital_in_open(p_inode, p_file, &(p_mio->d_i))) != 
                                                             ICP_MIO_BAD_MINOR)
    {
#ifdef DEBUG_MIO
  printk("<1> mio_digital_in returned %i\n", ret_val);
#endif
      return ret_val;
    }
    else if ( (ret_val = mio_digital_out_open(p_inode, p_file, &(p_mio->d_o))) != 
                                                             ICP_MIO_BAD_MINOR)
    {
#ifdef DEBUG_MIO
  printk("<1> mio_digital_out returned %i\n", ret_val);
#endif
      return ret_val;
    }
    else if ( (ret_val = mio_counter_open(p_inode, p_file, &(p_mio->ctr))) != 
                                                             ICP_MIO_BAD_MINOR)
    {
#ifdef DEBUG_MIO
  printk("<1> mio_counter returned %i\n", ret_val);
#endif
      return ret_val;
    }
    else
    {
      return -ENODEV;
    }
  }
  return ret_val; /* success */
}
/******************************************************************************
*
* mio_close - close routine for the mio board, this routine is called by the FS
*             if the mio board is opened
* Description
*
* Parameters:
*         
*
* RETURNS: -
*   
* SEE ALSO: 
******************************************************************************/
static int mio_close(struct inode* p_inode, struct file* p_file)
{
  ICP_MIO_STRUCT* p_mio = (ICP_MIO_STRUCT*)p_file->private_data;

  return (mio_linux_fs_close(&(p_mio->fs)));
}

/******************************************************************************
*
* mio_ioctl - ioctl routine for the mio board, this routine is called by the FS
*             if the mio board is opened
* Description
*
* Parameters:
*         
*
* RETURNS: -
*   
* SEE ALSO: 
******************************************************************************/
static int mio_ioctl(struct inode* p_inode, struct file* p_file, 
                     unsigned int cmd, unsigned long arg)
{
  ICP_MIO_STRUCT* p_mio = (ICP_MIO_STRUCT*)p_file->private_data;
  int ret_val;
  
  if (cmd == ICP_MIO_IOCTL_GET_NR)
  {
    return nrOfMios;
  }
  else if ((ret_val = mio_analog_in_ioctl(p_file, cmd, arg, &(p_mio->a_i))) !=
                                                            ICP_MIO_BAD_IOCTL)
  {
    return ret_val;
  }
  else if ((ret_val = mio_analog_out_ioctl(p_file, cmd, arg, &(p_mio->a_o))) !=
                                                            ICP_MIO_BAD_IOCTL)
  {
    return ret_val;
  }
  else if ((ret_val = mio_digital_in_ioctl(p_file, cmd, arg, &(p_mio->d_i))) !=
                                                            ICP_MIO_BAD_IOCTL)
  {
    return ret_val;
  }
  else if ((ret_val = mio_digital_out_ioctl(p_file, cmd, arg, &(p_mio->d_o))) !=
                                                            ICP_MIO_BAD_IOCTL)
  {
    return ret_val;
  }
  else if ((ret_val = mio_counter_ioctl(p_file, cmd, arg, &(p_mio->ctr))) !=
                                                            ICP_MIO_BAD_IOCTL)
  {
    return ret_val;
  }
  else if ((ret_val = mio_interrupt_ioctl(p_file, cmd, arg, &(p_mio->irq))) !=
                                                            ICP_MIO_BAD_IOCTL)
  {
    return ret_val;
  }
  else
  {
    return -EINVAL; /* Invalid argument */
  }
  return ret_val;
}


/*---------------------------------------------------------------------------
** FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* icp_mio_init - Initialize the MIO cards
*
* This routine initializes the MIO adapters
*
* Parameters:
*         -
*
* RETURNS: ERROR, in case of an error (Out of memory, No device found)
*          OK   , if everything was OK.
*   
* SEE ALSO: 
******************************************************************************/
STATUS icp_mio_init(void)
{
  if (mioListInitialized)
  {
    icpErrorMsg(ICP_MIO_NAME, ICP_MIO_LIST_ALREADY_INIT);
    
    /* Already initialized */
    return ERROR;
  }

  if ( icpFindAllPciDev(ICP_MIO_VENDORID, ICP_MIO_DEVICEID, 
                        ICP_MIO_SUBSYSID, ICP_MIO_SUBDEVID,
                        (void**)&pMioDevices, sizeof(ICP_MIO_STRUCT), 
                        &nrOfMios, mioInitCallback) != OK)
  {
    /* Can't initialize MIO-Card */
    icpErrorMsg(ICP_MIO_NAME, ICP_MIO_CANT_INIT);
    /*
    ** cleanup
    */
    icp_mio_cleanup();
    return ERROR;
  }
  else if (nrOfMios == 0)
  {
    /* No MIO found */
    icpErrorMsg(ICP_MIO_NAME, ICP_MIO_NOT_FOUND);
    
    return -ENODEV; /* No such device */
  }
  
  mioListInitialized = TRUE;
  return nrOfMios;
}


/******************************************************************************
*
* icp_mio_cleanup - Cleanup the MIO device list release all reserved resources
*
*
* Parameters:
*         -
*
* RETURNS: -
*   
* SEE ALSO: 
******************************************************************************/
void icp_mio_cleanup(void)
{
  /*
  ** Free MIO device list
  */
  icpFreeListWithCallback((void**)&pMioDevices, mioCleanupCallback);
  mioListInitialized = FALSE;
}

/******************************************************************************
*
* icp_mio_get_nr - 
*
* This routine returns the number of initialized Mio boards
*
* Parameters:
*         -
*
* RETURNS: see description
*   
* SEE ALSO: 
******************************************************************************/
unsigned int icp_mio_get_nr(void)
{
  return nrOfMios;
}


/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/
