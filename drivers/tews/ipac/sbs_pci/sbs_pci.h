/* $Id: sbs_pci.h 17 2006-02-13 13:38:24Z Welzel $ */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @    S B S _ P C I    @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          Linux - IPAC Carrier Driver                           **
**                                                                           **
**                                                                           **
**    File             sbs_pci.h                                             **
**                                                                           **
**                                                                           **
**    Function         SBS PCI carrier programming defintions                **
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


#ifndef __SBS_PCI_H__
#define __SBS_PCI_H__


#define SxPCI_MAX_SLOT      4
#define SxPCI_NUM_BAR       3


/* IP Status and Control Register Definitions */
#define SxPCI_CNTL0_REG             0x500
#define SxPCI_CNTL1_REG             0x600
#define SxPCI_CNTL2_REG             0x700


/* IP IO/ID/INT Space Definitions */
#define SxPCI_IO_SPACE_OFF          0x1000
#define SxPCI_IO_SPACE_GAP          0x1000
#define SxPCI_IO_SPACE_SIZE         0x0100

#define SxPCI_ID_SPACE_OFF          0x1100
#define SxPCI_ID_SPACE_GAP          0x1000
#define SxPCI_ID_SPACE_SIZE         0x0100

#define SxPCI_INT_SPACE_OFF         0x1200
#define SxPCI_INT_SPACE_GAP         0x1000
#define SxPCI_INT_SPACE_SIZE        0x0100

#define SxPCI_IOIDINT_SIZE          0x4300


/* IP MEM Space Definitions */
#define SxPCI_MEM16_OFF             0x01000000
#define SxPCI_MEM16_GAP             0x00800000
#define SxPCI_MEM16_SIZE            0x00800000


/* Control Register 0 */
#define SxPCI_CLK32A                0x01        /* set = 32 MHz clock, clear = 8 MHz clock */
#define SxPCI_CLK32B                0x02        /* set = 32 MHz clock, clear = 8 MHz clock */
#define SxPCI_CLK32C                0x04        /* set = 32 MHz clock, clear = 8 MHz clock */
#define SxPCI_CLK32D                0x08        /* set = 32 MHz clock, clear = 8 MHz clock */
#define SxPCI_CLR_AUTO              0x10        /* enable and clear the AUTO_INT_SET bit   */
#define SxPCI_AUTO_ACK              0x20        /* enable auto acknowledge feature         */
#define SxPCI_INTEN                 0x40        /* enable interrupts from the SxPCI        */
#define SxPCI_INTSET                0x80        /* generate an PCI interrupt (debugging)   */

/* Control Register 1 */
#define SxPCI_IRQA0                 0x01        /* set means interrupt is pending (INT0)   */ 
#define SxPCI_IRQA1                 0x02        /* set means interrupt is pending (INT1)   */ 
#define SxPCI_IRQB0                 0x04        /* set means interrupt is pending (INT0)   */ 
#define SxPCI_IRQB1                 0x08        /* set means interrupt is pending (INT1)   */ 
#define SxPCI_IRQC0                 0x10        /* set means interrupt is pending (INT0)   */ 
#define SxPCI_IRQC1                 0x20        /* set means interrupt is pending (INT1)   */ 
#define SxPCI_IRQD0                 0x40        /* set means interrupt is pending (INT0)   */ 
#define SxPCI_IRQD1                 0x80        /* set means interrupt is pending (INT1)   */ 

/* Control Register 2 */
#define SxPCI_AUTO_INT_SET          0x40        /* set means a bus error has occurred      */
#define SxPCI_LINT                  0x80        /* set means interrupt pending to PLX      */


#endif      /* __SBS_PCI_H__ */
