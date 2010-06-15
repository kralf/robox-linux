/*---------------------------------------------------------------------------
** Inova Computers GmbH. Sudetenstr 5, 87600 Kaufbeuren
**
** (c) Copyright 1998-2000. All Rights Reserved
**
** Date: Oct.3.2000
**
** Operating System: Linux
**
** Compiler : Gnu C/C++ Compiler 
**
**---------------------------------------------------------------------------
** Project: ICP-MULTI Linux Driver
** Title:   EEPROM Object
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** DESCRIPTION
**-------------------------------------------------------------------------*/
/* This module contains the driver code of the ICP-MULTI CompactPCI card
 */


 
/*---------------------------------------------------------------------------
** MODIFICATION HISTORY
**-------------------------------------------------------------------------*/
/*
modification history
-------------------- 
000,03Oct00,bt   created

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

#include <linux/kernel.h> /* printk()... */
#include <linux/fs.h>
#include <linux/types.h>  /* size_t */
#include <linux/pci.h>
#include <linux/delay.h>

#include <icp_carr_def.h>
#include <icp_drv_gen.h>

/*
** local module headers
*/
#include "../h/eeprom.h"
#include "../h/icp_mio.msg"

/*---------------------------------------------------------------------------
** DEFINE
**-------------------------------------------------------------------------*/
/* ICP-CARR users manual page 19 */
#define SERIAL_EEPROM_CONTROL_REGISTER  0x44
#define SERIAL_EEPROM_EEPCLK            0x0040
#define SERIAL_EEPROM_EEPENA            0x0020
#define SERIAL_EEPROM_EEPDAT            0x0010
#define SCL                             SERIAL_EEPROM_EEPCLK
#define SDA                             SERIAL_EEPROM_EEPDAT


#define EEP_WAIT_TIME                   100 /* time in us units */

/*
** factory calibration data
*/
#define EEP_FACTORY_CALIB_DATA_ADDR     0x10    /* Address 0x10 */
#define EEP_FACTORY_CALIB_DATA_LENGTH   0x02    /* 2 Bytes */
#define REF_FACTORY_CALIB               2400000 /* uV */

/*---------------------------------------------------------------------------
** TYPEDEF
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** DATA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** FORWARD DECLARATIONS
**-------------------------------------------------------------------------*/
static int    eepI2CByteRead(struct pci_dev* p_pci, unsigned char cDevSel, 
                             unsigned char cAddr, unsigned char* cData);
static BOOL   eepI2CSetAddressCommon(struct pci_dev* p_pci, 
                                     unsigned char cDevSel, 
                                     unsigned char cAddr);
static BOOL   eepI2CSetAddressRead(struct pci_dev* p_pci, 
                                   unsigned char cDevSel);
static void   eepI2CStopOperation(struct pci_dev* p_pci);

static BOOL   eepI2CAck(struct pci_dev* p_pci);
static void   eepI2CPutByte(struct pci_dev* p_pci, unsigned char cData);
static void   eepI2CGetByte(struct pci_dev* p_pci, unsigned char* cData);
static int    mio_eeprom_read(struct pci_dev* p_pci, u8 address, u8* p_data);

#ifdef IOCTL_FOR_EEPROM_WRITE
  static int    eepI2CByteWrite(struct pci_dev* p_pci, unsigned char cDevSel, 
                                unsigned char cAddr, unsigned char cData);
  static int    mio_eeprom_write(struct pci_dev* p_pci, u8 address, u8 data);
#endif

/*---------------------------------------------------------------------------
** STATIC FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* eepI2CByteRead - reads a byte
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
static int eepI2CByteRead(struct pci_dev* p_pci,
                          unsigned char cDevSel, unsigned char cAddr,
                          unsigned char* cData)
{
  if ( !eepI2CSetAddressCommon(p_pci, cDevSel, cAddr) )
  {
    /*
    ** there is no eeprom
    */
    *cData = 0xFF;
    return OK;
  }

  if ( !eepI2CSetAddressRead(p_pci, cDevSel) )
  {
    /*
    ** there is no eeprom
    */
    *cData = 0xFF;
    return OK;
  }

  /* 
  ** Get Byte Data 
  */
  eepI2CGetByte( p_pci, cData);

  /* 
  ** Acknowledge 
  */
  (void)eepI2CAck(p_pci);
  udelay(EEP_WAIT_TIME);

  eepI2CStopOperation(p_pci);

  return(OK);
}

#ifdef IOCTL_FOR_EEPROM_WRITE
/******************************************************************************
*
* eepI2CByteWrite - write a byte to I2C bus
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
static int eepI2CByteWrite(struct pci_dev* p_pci,
                           unsigned char cDevSel, unsigned char cAddr,
                           unsigned char cData)
{
  if ( !eepI2CSetAddressCommon(p_pci, cDevSel, cAddr) )
  {
    /*
    ** there is no device
    */
    return ERROR;
  }

  /* 
  ** Put Byte Data 
  **
  ** SCL=0,SDA=0
  */
  (void)pci_write_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, 
                              SERIAL_EEPROM_EEPENA);

  /*
  ** Put Byte Data
  */
  eepI2CPutByte ( p_pci, cData);

  /*
  ** Acknowledge
  */
  if ( !eepI2CAck(p_pci) )
  {
    /*
    ** there is no device
    */
    return ERROR;
  }
  
  /*
  ** SCL=0,SDA=0
  */
  (void)pci_write_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, 
                              SERIAL_EEPROM_EEPENA);

  eepI2CStopOperation(p_pci);
  return OK;
}
#endif


/******************************************************************************
*
* eepI2CSetAddressCommon - set the I2C address for the operation
*
* Description
*
* Parameters:
*         
*
* RETURNS: TRUE, if address could be set
*          FALSE, if there is no piggy back
*   
* SEE ALSO: 
******************************************************************************/
static BOOL eepI2CSetAddressCommon(struct pci_dev* p_pci,
                                   unsigned char cDevSel, 
                                   unsigned char cAddr)
{
  /* 
  ** Random Byte Lesen 
  **
  ** Start Condition 
  **
  ** set SCL=1  + SDA=1 
  */
  (void)pci_write_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, 
                              SERIAL_EEPROM_EEPENA | SDA | SCL);
  udelay(EEP_WAIT_TIME);
  
  /* 
  ** set SCL=1  + SDA=0 
  */
  (void)pci_write_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, 
                              SERIAL_EEPROM_EEPENA | SCL);
  udelay(EEP_WAIT_TIME);

  /* 
  ** set SCL=0  + SDA=0 
  */
  (void)pci_write_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, 
                              SERIAL_EEPROM_EEPENA);
  udelay(EEP_WAIT_TIME);


  /* 
  ** Put Byte Device Select,/Write=0 
  */
  eepI2CPutByte(p_pci, cDevSel & 0xfe);

  /* 
  ** Acknowledge 
  */
  if ( !eepI2CAck(p_pci) )
  {
    /*
    ** there is no device
    */
    return FALSE;
  }

  /* 
  ** set SCL=0  + SDA=0 
  */
  (void)pci_write_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, 
                              SERIAL_EEPROM_EEPENA);

  /* 
  ** Put Write Address 
  */
  eepI2CPutByte (p_pci, cAddr);

  /* 
  ** Acknowledge 
  */
  if ( !eepI2CAck(p_pci) )
  {
    /*
    ** there is no device
    */
    return FALSE;
  }

  return TRUE;
}

/******************************************************************************
*
* eepI2CSetAddressRead - short descr
*
* Description
*
* Parameters:
*         
*
* RETURNS: TRUE, if address could be set
*          FALSE, if there is no piggy back
*   
* SEE ALSO: 
******************************************************************************/
static BOOL eepI2CSetAddressRead(struct pci_dev* p_pci, unsigned char cDevSel)
{
  /* 
  ** set SCL=0  + SDA=1 
  */
  (void)pci_write_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, 
                              SERIAL_EEPROM_EEPENA | SDA);
  udelay(EEP_WAIT_TIME);

  /* 
  ** set SCL=1  + SDA=1 
  */
  (void)pci_write_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, 
                              SERIAL_EEPROM_EEPENA | SDA | SCL);
  udelay(EEP_WAIT_TIME);

  /* 
  ** Start Condition 
  **
  ** set SCL=1  + SDA=1 
  */
  (void)pci_write_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, 
                              SERIAL_EEPROM_EEPENA | SDA | SCL);
  udelay(EEP_WAIT_TIME);

  /* 
  ** set SCL=1  + SDA=0 
  */
  (void)pci_write_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, 
                              SERIAL_EEPROM_EEPENA | SCL);
  udelay(EEP_WAIT_TIME);

  /* 
  ** set SCL=0  + SDA=0 
  */
  (void)pci_write_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, 
                              SERIAL_EEPROM_EEPENA);
  udelay(EEP_WAIT_TIME);

  /* 
  ** Put Byte Device Select,/Write=0 
  */
  eepI2CPutByte ( p_pci, cDevSel | 0x01);

  /* 
  ** Acknowledge 
  */
  if ( !eepI2CAck(p_pci) )
  {
    /*
    ** there is no device
    */
    return FALSE;
  }

  /* 
  ** set SCL=0  + SDA=1 
  */
  (void)pci_write_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, 
                              SERIAL_EEPROM_EEPENA | SDA);

  return TRUE;
}


/******************************************************************************
*
* eepI2CStopOperation - short descr
*
* Description
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static void eepI2CStopOperation(struct pci_dev* p_pci)
{
  /* 
  ** Stop 
  **
  ** set SCL=0  + SDA=0 
  */
  (void)pci_write_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, 
                              SERIAL_EEPROM_EEPENA);
  udelay(EEP_WAIT_TIME);

  /* 
  ** set SCL=1  + SDA=0 
  */
  (void)pci_write_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, 
                              SERIAL_EEPROM_EEPENA | SCL);
  udelay(EEP_WAIT_TIME);

  /* 
  ** set SCL=1  + SDA=1 
  */
  (void)pci_write_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, 
                              SERIAL_EEPROM_EEPENA | SDA | SCL);
  udelay(EEP_WAIT_TIME);
}

/******************************************************************************
*
* eepI2CAck - short descr
*
* Description
*
* Parameters:
*         
*
* RETURNS: TRUE   if everything is OK, there is a piggy back
*          FALSE, if there is an error, no piggy back
*   
* SEE ALSO: 
******************************************************************************/
static BOOL eepI2CAck(struct pci_dev* p_pci) 
{
  short sValue;

  /* 
  ** set SCL=0  + SDA=1 
  */
  (void)pci_write_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, 
                              SERIAL_EEPROM_EEPENA | SDA);
  udelay(EEP_WAIT_TIME);

  /* 
  ** set SCL=1  + SDA=1 
  */
  (void)pci_write_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, 
                              SERIAL_EEPROM_EEPENA | SDA | SCL);

  (void)pci_read_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, &sValue);

  if( (sValue & 0x0010) != 0 )
  {
    return FALSE;
  }
  udelay(EEP_WAIT_TIME);

  return TRUE;
}


/******************************************************************************
*
* eepI2CPutByte - short descr
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
static void eepI2CPutByte(struct pci_dev* p_pci, unsigned char cData)
{
  /* Put Byte */
  char           cBit;
  unsigned short sValue;

  for(cBit=7;cBit>=0;cBit--)
  {
    /* 
    ** select 1 Bit of Data-Byte (MSB first); 
    ** put this Bit to EEPDAT (SDA) Bit 
    ** of serial EEPROM control register 
    */
    sValue = ((unsigned short)cData & 0x0080) >>3;

    /* 
    ** EEPROM output == EEPROM contol register 
    */
    sValue |= SERIAL_EEPROM_EEPENA; 

    /* 
    ** set SCL= 0  + SDA=Data 
    */
    (void)pci_write_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, 
                                sValue);
    udelay(EEP_WAIT_TIME);

    /* 
    ** set CLK= 1  + SDA=Data
    */
    (void)pci_write_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, 
                                sValue | SCL);
    udelay(EEP_WAIT_TIME);

    /* 
    ** set CLK= 0  + SDA=Data  (old value of sValue) 
    */
    (void)pci_write_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, 
                                sValue);

    /* 
    ** next data-Bit  (new MSB = old MSB - 1)
    */
    cData<<=1;
  }
}

/******************************************************************************
*
* eepI2CGetByte - short descr
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
static void eepI2CGetByte(struct pci_dev* p_pci, unsigned char* cData)
{
  char           cBit;
  unsigned short sValue;
  u16            iOutputValue;

  *cData = 0x00;
  for(cBit=7;cBit>=0;cBit--)
  {
    *cData = *cData << 1;
    iOutputValue = SERIAL_EEPROM_EEPENA | SDA;

    /* 
    ** set SCL=0 + SDA=1 
    */
    (void)pci_write_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, 
                                iOutputValue);
    udelay(EEP_WAIT_TIME);

    /* 
    ** set SCL=1 + SDA=1 
    */
    (void)pci_write_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, 
                                iOutputValue | SCL);
    udelay(EEP_WAIT_TIME);

    /*
    ** read 1 bit 
    */
    (void)pci_read_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, &sValue);
    
    if ( sValue & SDA )
    {
      *cData |= 0x01; /* this bit is set */
    }

    /* 
    ** set SCL=0 + SDA=1 
    */
    (void)pci_write_config_word(p_pci, SERIAL_EEPROM_CONTROL_REGISTER, 
                                iOutputValue);
  }
}

/******************************************************************************
*
* mio_eeprom_read - read a byte from the EEPROM
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
static int mio_eeprom_read(struct pci_dev* p_pci, u8 address, u8* p_data)
{
  return eepI2CByteRead(p_pci,ICP_MIO_EEP_DEV_SEL, address, p_data);
}

#ifdef IOCTL_FOR_EEPROM_WRITE
/******************************************************************************
*
* mio_eeprom_write - write a byte to the EEPROM
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
static int mio_eeprom_write(struct pci_dev* p_pci, u8 address, u8 data)
{
  return eepI2CByteWrite(p_pci,ICP_MIO_EEP_DEV_SEL, address, data);
}
#endif

/*---------------------------------------------------------------------------
** FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* mio_eeprom_init - Initialization of the EEPROM object
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
STATUS mio_eeprom_init(MIO_EEPROM* p_eeprom, struct pci_dev* p_pci)
{
  int  i;
  u8   address;
  u8   data[EEP_FACTORY_CALIB_DATA_LENGTH];
  /*
  ** Init the structure
  */
  p_eeprom->pci = p_pci;

  /* 
  ** read out the factory calibration data
  */
  address = EEP_FACTORY_CALIB_DATA_ADDR; 
  for (i = 0; i < EEP_FACTORY_CALIB_DATA_LENGTH; i++)
  {
    if (mio_eeprom_read(p_pci, address, &(data[i])) != OK)
    {
      icpErrorMsg(ICP_MIO_NAME, ICP_MIO_EEP_READ_ERROR);
      return -EIO; /* I/O error*/
    }
    address++;
  }

  p_eeprom->fact_calib  = (u32)data[0] + ((u32)data[1] << 8);
  p_eeprom->fact_calib += REF_FACTORY_CALIB;
  p_eeprom->state       = eeprom_initialized;

#ifdef DEBUG_EEPROM
  printk("<1> eeprom init factory calib: %i OK\n", p_eeprom->fact_calib);
#endif
  return OK;
}

/******************************************************************************
*
* mio_eeprom_cleanup - Cleanup of the Eeprom object
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
void mio_eeprom_cleanup(MIO_EEPROM* p_eeprom)
{
#ifdef DEBUG_EEPROM
  printk("<1> eeprom deinit OK\n");
#endif

}


/******************************************************************************
*
* mio_eeprom_ioctl - ioctl routine for the eeprom object
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
int mio_eeprom_ioctl(struct file* p_file, unsigned int cmd, 
                     unsigned long arg, MIO_EEPROM* p_eeprom)
{
#if 0
  struct pci_dev* p_pci   = p_eeprom->pci;
  int             ret_val = OK;

  switch(cmd)
  {
    case MIO_IOCTL_IRQ_GEN_ENABLE:
    {
      if (arg)
      {
        enable_gen_interrupt(p_pci);
      }
      else
      {
        disable_gen_interrupt(p_pci);
      }
      ret_val = OK;
      break;
    }
    default:
    {
      ret_val = ICP_MIO_BAD_IOCTL;
    }
  }
  return ret_val;
#else
  return ICP_MIO_BAD_IOCTL;
#endif
}


/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/
