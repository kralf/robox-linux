/*---------------------------------------------------------------------------
** Inova Computers GmbH. Sudetenstr 5, 87600 Kaufbeuren
**
** (c) Copyright 1998-1999. All Rights Reserved
**
** Date: Mai.11.1999
**
** Operating System: VxWorks Ver. 5.3.1. / 5.4.
**
** Compiler : Gnu C/C++ Compiler (Tornado)
**
**---------------------------------------------------------------------------
** Project: K6-CPU BSP
** Title:   ICP-CARRIER board vxWorks driver
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
000,11mai99,bt   created
*/


#ifndef  INCicpCarrierh
#define  INCicpCarrierh

#ifdef __cplusplus
  extern "C" {
#endif

/*---------------------------------------------------------------------------
** INCLUDE
**-------------------------------------------------------------------------*/
#include "vxWorks.h"

/*---------------------------------------------------------------------------
** DEFINE
**-------------------------------------------------------------------------*/

#ifndef ICP_CARR_VENDORID         
  #define ICP_CARR_VENDORID            0x104c
#endif /* ICP_CARR_VENDORID */

#ifndef ICP_CARR_DEVICEID
  #define ICP_CARR_DEVICEID            0x8000
#endif /* ICP_CARR_DEVICEID */


#ifndef ICP_CARR_SUB_VENDORID         
  #define ICP_CARR_SUB_VENDORID        0x14A6
#endif /* ICP_CARR_SUB_VENDORID */

#ifndef ICP_CARR_SUB_SYSID
  #define ICP_CARR_SUB_SYSID           0x0001
#endif /* ICP_CARR_SUB_SYSID */

#ifndef ICP_CARR_ONBOARD_SUB_SYSID    
  #define ICP_CARR_ONBOARD_SUB_SYSID   0x0011
#endif /* ICP_CARR_ONBOARD_SUB_SYSID */


#ifndef ICP_CARR_MAX_SOCKETS
  #define ICP_CARR_MAX_SOCKETS         2
#endif

#ifndef ICP_CARR_INVALID_VENDORID
  #define ICP_CARR_INVALID_VENDORID    0xFFFF
#endif

#ifndef ICP_CARR_INVALID_DEVICEID
  #define ICP_CARR_INVALID_DEVICEID    0xFFFF
#endif


/*
** Reset time in miliseconds
*/
#define ICP_CARR_RESET_TIME            0x5

/*
** 0 - 14 Number of wait states
** 15 - Ready controlled
*/
#define ICP_CARR_MAX_WAITSTATES       15

/*
** ACCESSWIDTH = 0 => 8  Bit Access width
** ACCESSWIDTH = 1 => 16 Bit Access width
*/
#define ICP_CARR_MAX_ACCESSWIDTH      1


/*---------------------------------------------------------------------------
** TYPEDEF
**-------------------------------------------------------------------------*/

/*
** ISR routine type for piggy backs
*/    
typedef BOOL (*ICPBOOLFUNCPTR)(void *);

/*
** min struct to create a list of the founded piggy backs
*/
typedef struct _ICP_CARR_MIN_STRUCT
{
  void*           next;
  unsigned long   socketNr;
} ICP_CARR_MIN_STRUCT;


/*---------------------------------------------------------------------------
** EXTERN DATA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** EXTERN FUNCTION
**-------------------------------------------------------------------------*/

#if defined(__STDC__) || defined(__cplusplus)

  extern STATUS icpCarrInit(void);
  
  extern void* icpCarrGetNthPiggyStruct(void* pListBegin, 
                                        int index);
  extern STATUS icpCarrGetSocketIds (unsigned long   socketNr, 
                                     unsigned short* pVendorId,
                                     unsigned short* pDeviceId);
  extern STATUS icpCarrGetSocketBaseAddress(unsigned long  socketNr, 
                                            UINT32* pBaseAddress);
  extern STATUS icpCarrGetSocketIrqNr(unsigned long  socketNr, 
                                      UINT8* pIrqNr);
  extern STATUS icpCarrGetSocketsPciPar(unsigned long  socketNr, 
                                        int* pBus, int* pDev, int* pFunc);
  extern STATUS icpCarrSocketConnectIrq(unsigned long  socketNr, 
                                        ICPBOOLFUNCPTR irqRoutine, 
                                        void* irqParam);
  extern STATUS icpCarrSocketDisconnectIrq(unsigned long  socketNr);
  extern STATUS icpCarrSocketEnableIrq(unsigned long  socketNr);
  extern STATUS icpCarrSocketDisableIrq(unsigned long  socketNr);
  extern STATUS icpCarrEnableSocketLED(unsigned long  socketNr, 
                                       BOOL led1, BOOL led2);
  extern STATUS icpCarrSetSocketLED(unsigned long  socketNr, 
                                    BOOL led1, BOOL led2);
  extern STATUS icpCarrInitSocketDataAccess(unsigned long  socketNr, 
                                            unsigned short waitStates,
                                            unsigned short accessWidth);
  extern STATUS icpCarrFindSocket(unsigned short vendorId, 
                                  unsigned short deviceId,
                                  unsigned long  index,
                                  unsigned long* socketNr);
  extern STATUS icpCarrFindAllPiggy(unsigned short vendorId, 
                                    unsigned short deviceId,
                                    void** ppListBegin, 
                                    UINT32 listElementSize, 
                                    UINT32* NrOfDevs, 
                                    STATUS (*CallBackFunc)(void*) );
  extern void icpCarrFreeList(void** ppListBegin);
  extern UINT32 icpCarrGetNr(void);
  extern STATUS icpCarrWriteRom(unsigned long  socketNr, 
                                unsigned char  address,
                                unsigned char  data);


#else /* __STDC__ */

  STATUS icpCarrInit();
  void*  icpCarrGetNthPiggyStruct();
  STATUS icpCarrGetSocketIds();
  STATUS icpCarrGetSocketBaseAddress();
  STATUS icpCarrGetSocketIrqNr();
  STATUS icpCarrGetSocketsPciPar();
  STATUS icpCarrSocketConnectIrq();
  STATUS icpCarrSocketDisconnectIrq();
  STATUS icpCarrSocketEnableIrq();
  STATUS icpCarrSocketDisableIrq();
  STATUS icpCarrEnableSocketLED();
  STATUS icpCarrSetSocketLED();
  STATUS icpCarrInitSocketDataAccess();
  STATUS icpCarrFindSocket();
  STATUS icpCarrFindAllPiggy();
  void   icpCarrFreeList();
  UINT32 icpCarrGetNr();
  STATUS icpCarrWriteRom();

#endif /* __STDC__ */


#ifdef __cplusplus
  }
#endif


#endif /* INCicpDinh  */
/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/
