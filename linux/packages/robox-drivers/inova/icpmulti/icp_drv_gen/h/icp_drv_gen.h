/*---------------------------------------------------------------------------
** Inova Computers GmbH. Sudetenstr 5, 87600 Kaufbeuren
**
** (c) Copyright 1998-2000. All Rights Reserved
**
** Date: Sep.25.2000
**
** Operating System: Linix
**
** Compiler : Gnu C/C++ Compiler 
**
**---------------------------------------------------------------------------
** Project: K6-CPU Linux 
** Title:   Generic CPCI Library
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** DESCRIPTION
**-------------------------------------------------------------------------*/

 
/*---------------------------------------------------------------------------
** MODIFICATION HISTORY
**-------------------------------------------------------------------------*/
/*
modification history
------------------ 
000,25sep00,bt   created from the VxWorks header

*/


#ifndef INCicpDrvGenh
#define INCicpDrvGenh

#ifdef __cplusplus
  extern "C" {
#endif


/*---------------------------------------------------------------------------
** INCLUDE
**-------------------------------------------------------------------------*/
#include <linux/pci.h>

/*---------------------------------------------------------------------------
** DEFINE
**-------------------------------------------------------------------------*/
#define ICP_ERRMSG_MAX_LENGTH     1024
#define ICP_MODNAME_MAX_LENGTH    16

#define OK    0
#define ERROR (-1)

#define FALSE 0
#define TRUE  1

/*-------------------------------------------------------------------------
**                        Error logging command
**-----------------------------------------------------------------------*/
#define ICP_ERRORLOG(errStr) printk("<1> %s", (errStr))

/*-------------------------------------------------------------------------
**                        Error Macros
**-----------------------------------------------------------------------*/
#define I_CHK_STATUSG(Function, ErrLabel)     \
        if ((Function) != OK) goto ErrLabel;

#define I_CHK_STATUSR(Function)               \
        if ((Function) != OK) return ERROR;

#define I_CHK_STATUSGM(Function, ErrLabel, ModName, ErrMsg) \
        if ((Function) != OK)                               \
	{                                                   \
          icpErrorMsg((ModName), (ErrMsg) );                \
          goto ErrLabel;                                    \
        }

#define I_CHK_STATUSRM(Function, ModName, ErrMsg) \
        if ((Function) != OK)                     \
        {                                         \
          icpErrorMsg( ModName, ErrMsg );         \
          return ERROR;                           \
        }


#define I_CHK_STATUSGS(Function, ErrLabel)  \
        if ( (status = (Function)) != OK )  \
        {                                   \
          goto ErrLabel;                    \
        }

#define I_CHK_STATUSRS(Function)            \
        if ( (status = (Function)) != OK )  \
        {                                   \
          return status;                    \
        }


#define I_CHK_STATUSGMS(Function, ErrLabel, ModName, ErrMsg)  \
        if ( (status = (Function)) != OK )                    \
        {                                                     \
          icpErrorMsg((ModName), (ErrMsg) );                  \
          goto ErrLabel;                                      \
        }

#define I_CHK_STATUSRMS(Function, ModName, ErrMsg)  \
        if ( (status = (Function)) != OK )          \
        {                                           \
          icpErrorMsg( (ModName), (ErrMsg) );       \
          return status;                            \
        }


#define I_GM(ErrLabel, ModName, ErrMsg)     \
        icpErrorMsg( (ModName), (ErrMsg) ); \
        goto ErrLabel;                      \

#define I_RM(ModName, ErrMsg)               \
        icpErrorMsg( (ModName), (ErrMsg) ); \
        return ERROR;                       \


#define ICP_IODELAYPORT                   0xED
/******************************************************************************
**
**  IODELAY - Perform a delay cycle between back-to-back IO accesses
**
**  IODELAY
**
**  Parameters:
**            -
**
**  IODELAYPORT - the unused IO port (00h-FFh) to recieve the data
**
**  Exit:
**
**  Processing:
**  Performs a delay between back-to-back IO cycles by causing a
**  write to the specified unused IO port.  This delay is for four
**  BCLKs.  On a standard AT system with an 8MHz bus clock, this
**  is approximately 1/2us.
**
**  Return:
**        -
**
******************************************************************************/
//#define ICP_IODELAY                       sysInByte(ICP_IODELAYPORT)




/*---------------------------------------------------------------------------
** TYPEDEF
**-------------------------------------------------------------------------*/
typedef struct _PCI_MIN_STRUCT
{
  void*  next;
  struct pci_dev* pciDev;
} PCI_MIN_STRUCT;

typedef int          STATUS;
typedef int          BOOL;
/*---------------------------------------------------------------------------
** EXTERN DATA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** EXTERN FUNCTION
**-------------------------------------------------------------------------*/

#if defined(__STDC__) || defined(__cplusplus)

  extern STATUS icpFindAllPciDev(unsigned int vendorId, 
                                 unsigned int deviceId,
                                 u16 subVendorId, u16 subSystemId,
                                 void** ppListBegin, size_t listElementSize,
                                 unsigned int* NrOfDevs, 
                                 STATUS (*CallBackFunc)(void*) );

  extern void icpFreeListWithCallback(void** ppListBegin, 
                                      void (*CleanupCallBackFunc)(void*));

  extern void   icpFreeList(void** ppListBegin);

  extern void*  icpGetNthDev(void* pListBegin, int nthElement);

  extern void   icpErrorMsg(char* modName, const char* pErrStr, ... )
                            __attribute__ ((format (printf, 2, 3)));
  extern void   icpDelay(uint32_t cyclye);

#else /* __STDC__ */

  STATUS icpFindAllPciDev();
  void icpFreeListWithCallback();
  void   icpFreeList();
  void*  icpGetNthDev();

  void   icpErrorMsg();
  void   icpDelay();

#endif /* __STDC__ */


#ifdef __cplusplus
  }
#endif


#endif /* INCicpDrvGenh */
/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/
