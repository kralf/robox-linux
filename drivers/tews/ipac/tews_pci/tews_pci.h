/* $Id: tews_pci.h 17 2006-02-13 13:38:24Z Welzel $ */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @   T E W S _ P C I   @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          Linux - IPAC Carrier Driver                           **
**                                                                           **
**                                                                           **
**    File             tpci200.h                                             **
**                                                                           **
**                                                                           **
**    Function         TEWS PCI carrier programming defintions               **
**                                                                           **
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
**                     Copyright (c) 2006                                    **
**                     TEWS TECHNOLOGIES GmbH                                **
**                                                                           **
**                                                                           **
**                                                                           **
**    System           Linux                                                 **
**                                                                           **
**    $Date: 2006-02-13 14:38:24 +0100 (Mo, 13 Feb 2006) $   $Rev: 17 $      **
**                                                                           **
*******************************************************************************
*******************************************************************************/


#ifndef __TEWS_PCI_H__
#define __TEWS_PCI_H__


#define TxPCI_MAX_SLOT      4
#define TxPCI_NUM_BAR       6


/* IP Interface Register Definitions */
#define TxPCI_REVISION_REG          0x00
#define TxPCI_CONTROL_A_REG         0x02
#define TxPCI_CONTROL_B_REG         0x04
#define TxPCI_CONTROL_C_REG         0x06
#define TxPCI_CONTROL_D_REG         0x08
#define TxPCI_RESET_REG             0x0A
#define TxPCI_STATUS_REG            0x0C

#define TxPCI_IFACE_SIZE            0x100


/* IP IO/ID/INT Space Definitions */
#define TxPCI_IO_SPACE_OFF          0x0000
#define TxPCI_IO_SPACE_GAP          0x0100
#define TxPCI_IO_SPACE_SIZE         0x0080

#define TxPCI_ID_SPACE_OFF          0x0080
#define TxPCI_ID_SPACE_GAP          0x0100
#define TxPCI_ID_SPACE_SIZE         0x0040

#define TxPCI_INT_SPACE_OFF         0x00C0
#define TxPCI_INT_SPACE_GAP         0x0100
#define TxPCI_INT_SPACE_SIZE        0x0040

#define TxPCI_IOIDINT_SIZE          0x0400


/* IP MEM Space Definitions */
#define TxPCI_MEM8_GAP              0x00400000
#define TxPCI_MEM8_SIZE             0x00400000
#define TxPCI_MEM16_GAP             0x00800000
#define TxPCI_MEM16_SIZE            0x00800000


/* IP Interface Control Register */
#define TxPCI_INT1_EN               0x0080      /* INT1 enabled */
#define TxPCI_INT0_EN               0x0040      /* INT0 enabled */
#define TxPCI_INT1_EDGE             0x0020      /* INT1 edge sensitive */
#define TxPCI_INT0_EDGE             0x0010      /* INT0 edge sensitive */
#define TxPCI_ERR_INT_EN            0x0008      /* ERR INT enabled */
#define TxPCI_TIME_INT_EN           0x0004      /* TIMEOUT INT enabled */
#define TxPCI_RECOVER_EN            0x0002      /* IP recover tme enabled */
#define TxPCI_CLK32                 0x0001      /* 32 MHz clock */

/* IP Interface Reset Register */
#define TxPCI_D_RESET               0x0008      /* Assert IP RESET# signal at slot D*/
#define TxPCI_C_RESET               0x0004      /* Assert IP RESET# signal at slot C*/
#define TxPCI_B_RESET               0x0002      /* Assert IP RESET# signal at slot B*/
#define TxPCI_A_RESET               0x0001      /* Assert IP RESET# signal at slot A*/

/* IP Interface Status Register */
#define TxPCI_D_TIMEOUT             0x8000      /* read: timeout has occurred, write: clear timeout status  */
#define TxPCI_C_TIMEOUT             0x4000
#define TxPCI_B_TIMEOUT             0x2000
#define TxPCI_A_TIMEOUT             0x1000
#define TxPCI_D_ERROR               0x0800      /* ERROR# signal asserted */
#define TxPCI_C_ERROR               0x0400
#define TxPCI_B_ERROR               0x0200
#define TxPCI_A_ERROR               0x0100
#define TxPCI_D_INT1                0x0080      /* read: INT1 interrupt request, write: clear INT1 status */
#define TxPCI_D_INT0                0x0040      /* read: INT0 interrupt request, write: clear INT0 status */  
#define TxPCI_C_INT1                0x0020
#define TxPCI_C_INT0                0x0010
#define TxPCI_B_INT1                0x0008
#define TxPCI_B_INT0                0x0004
#define TxPCI_A_INT1                0x0002
#define TxPCI_A_INT0                0x0001


#endif      /* __TEWS_PCI_H__ */
