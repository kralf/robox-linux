/*---------------------------------------------------------------------------
** Inova Computers GmbH. Sudetenstr 5, 87600 Kaufbeuren
**
** (c) Copyright 1998-1999. All Rights Reserved
**
** Date: Mai.09.1999
**
** Operating System: VxWorks Ver. 5.3.1.
**
** Compiler : Gnu C/C++ Compiler (Tornado)
**
**---------------------------------------------------------------------------
** Project: K6-CPU BSP
** Title:   ICP-Carrier defines
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
001,19mai99,bt   created
*/


#ifndef  INCicpCarrDefh
#define  INCicpCarrDefh

#ifdef __cplusplus
  extern "C" {
#endif

/*---------------------------------------------------------------------------
** INCLUDE
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** DEFINE
**-------------------------------------------------------------------------*/
#define ICP_CARR_GPIO01               0xCC
#define ICP_CARR_GPIO0_ON             0x1     /* Bit0 is for GPIO0 */
#define ICP_CARR_GPIO1_ON             0x2     /* Bit1 is for GPIO1 */

#define ICP_CARR_GPIO23               0xF0
#define ICP_CARR_GPIO2_ON             0x4     /* Bit2 is for GPIO2 */
#define ICP_CARR_GPIO3_ON             0x8     /* Bit3 is for GPIO3 */

#define ICP_CARR_GPIO01_CTRL          0xB8
#define ICP_CARR_GPIO01_INITVALUE     0x01040004
#define ICP_CARR_GPIO0_ENABLE         0x00000001
#define ICP_CARR_GPIO1_ENABLE         0x00010000

#define ICP_CARR_GPIO23_CTRL          0xBC
#define ICP_CARR_GPIO23_INITVALUE     0x03040204
#define ICP_CARR_GPIO2_ENABLE         0x00000001
#define ICP_CARR_GPIO3_ENABLE         0x00010000


#define ICP_CARR_LOCAL_BUS_CTRL_REG   0xB0

#define ICP_CARR_SOCK1WAITSATES(x)    ((x & 0x0F) << 20)       /* Bit20 - 23 */
#define ICP_CARR_SOCK1DATAWIDTH(x)    (x ? 0x00010000L : 0x0L) /* Bit16 */
#define ICP_CARR_SOCK2WAITSATES(x)    ((x & 0x0F) << 12)       /* Bit12 - 15 */
#define ICP_CARR_SOCK2DATAWIDTH(x)    (x ? 0x00000100L : 0x0L) /* Bit8 */
#define ICP_CARR_RESET                0x00020000               /* Bit17 */
#define ICP_CARR_AUX_INT_POL          0x00040000               /* Bit18 */


#define ICP_CARR_IRQSTATUS            0x48
#define ICP_CARR_IRQENABLE            0x4C
#define ICP_CARR_AUX_INT              0x00020000               /* Bit17 */
#define ICP_CARR_IRQ_ON               0x00020000               /* Bit17 */
#define ICP_CARR_INT_PEND             0x80000000               /* Bit31 */

/*---------------------------------------------------------------------------
** TYPEDEF
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** EXTERN DATA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** EXTERN FUNCTION
**-------------------------------------------------------------------------*/


#ifdef __cplusplus
  }
#endif

#endif /* INCicpCarrDefh  */
/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/