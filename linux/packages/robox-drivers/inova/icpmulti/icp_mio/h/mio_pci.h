/*---------------------------------------------------------------------------
** Inova Computers GmbH. Sudetenstr 5, 87600 Kaufbeuren
**
** (c) Copyright 1998-2000. All Rights Reserved
**
** Date: Oct.2.2000
**
** Operating System: Linux
**
** Compiler : Gnu C/C++ Compiler
**
**---------------------------------------------------------------------------
** Project: ICP-MULTI Linux driver
** Title:   Pci Object header
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
000,02Oct00,bt   created
*/


#ifndef  INCmioPcih
#define  INCmioPcih

#ifdef __cplusplus
  extern "C" {
#endif

/*---------------------------------------------------------------------------
** INCLUDE
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** DEFINE
**-------------------------------------------------------------------------*/
#ifndef ICP_MIO_VENDORID         
  #define ICP_MIO_VENDORID            0x104c
#endif /* ICP_MIO_VENDORID */

#ifndef ICP_MIO_DEVICEID
  #define ICP_MIO_DEVICEID            0x8000
#endif /* ICP_MIO_DEVICEID */

#ifndef ICP_MIO_SUBSYSID         
  #define ICP_MIO_SUBSYSID            0x14A6
#endif /* ICP_MIO_SUNSYSID */

#ifndef ICP_MIO_SUBDEVID
  #define ICP_MIO_SUBDEVID            0x00A1
#endif /* ICP_MIO_SUBDEVID */


#define ICP_MIO_ACCESS_WIDTH  0x1L  /* 0 -> 8 Bit, 1-> 16 Bit */
#define ICP_MIO_WAIT_STATES   0x0L  /* wait states of the fw controller */

/*---------------------------------------------------------------------------
** TYPEDEF
**-------------------------------------------------------------------------*/
typedef struct _MIO_PCI
{
  PCI_MIN_STRUCT  pci_min;
  void*           mapped_addr;
}MIO_PCI;

/*---------------------------------------------------------------------------
** EXTERN DATA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** EXTERN FUNCTION
**-------------------------------------------------------------------------*/

#if defined(__STDC__) || defined(__cplusplus)

  extern STATUS mio_pci_init(MIO_PCI* p_pci, void** mapped_addr);
  extern void   mio_pci_cleanup(MIO_PCI* p_pci); 

#else /* __STDC__ */

  extern STATUS mio_pci_init();
  extern void   mio_pci_cleanup(); 

#endif /* __STDC__ */


#ifdef __cplusplus
  }
#endif

#endif /* INCmioPcih  */
/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/