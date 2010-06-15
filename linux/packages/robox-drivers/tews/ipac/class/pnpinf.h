/* $Id: pnpinf.h 110 2007-09-18 13:52:57Z Hesse $ */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @     P N P I N F     @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          Linux - IPAC Carrier Driver                           **
**                                                                           **
**                                                                           **
**    File             pnpinf.h                                              **
**                                                                           **
**                                                                           **
**    Function         PnP information for carrier and IPAC port driver      **
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
**    $Date: 2007-09-18 15:52:57 +0200 (Di, 18 Sep 2007) $   $Rev: 110 $      **
**                                                                           **
*******************************************************************************
*******************************************************************************/


#ifndef __PNPINF_H__
#define __PNPINF_H__

struct ipac_PnP_entry {
    unsigned int  manufacturer;
    unsigned int  model_number;
    char  *driver_name;
};

struct carrier_PnP_entry {
    unsigned int  vendor;
    unsigned int  device;
    unsigned int  ss_vendor;
    unsigned int  ss_device;
    char  *driver_name;
};



static struct ipac_PnP_entry  ipac_PnP_list[] = {
/*   manufacturer | model number | driver name       */
    { 0xB3,  0x01,  "tip810drv"      },             /* TIP810-10 - Basic CAN Interface              */
    { 0xB3,  0x02,  "tip670drv"      },             /* TIP670-10 - 8 digital I/O                    */
    { 0xB3,  0x03,  "tip670drv"      },             /* TIP670-20 - 4 digital I/O                    */
    { 0xB3,  0x04,  "tip600drv"      },             /* TIP600-10 - 16 digital Inputs                */
    { 0xB3,  0x05,  "tip700drv"      },             /* TIP700-10 - 16 digital Outputs               */
    { 0xB3,  0x06,  "tip700drv"      },             /* TIP700-20 - 8 digital Outputs                */
    { 0xB3,  0x07,  "tip830drv"      },             /* TIP830-20 - 8x16-bit ADC                     */
    { 0xB3,  0x08,  "tip830drv"      },             /* TIP830-10 - 8x16-bit ADC                     */
    { 0xB3,  0x09,  "tip850drv"      },             /* TIP850-10 - 12-bit ADC                       */
    { 0xB3,  0x0A,  "tip102drv"      },             /* TIP102-1x - 1 channel motion controller      */
    { 0xB3,  0x0B,  "tip102drv"      },             /* TIP102-2x - 2 channel motion controller      */
    { 0xB3,  0x0C,  "tip111drv"      },             /* TIP111-1x - 1 channel motion controller      */
    { 0xB3,  0x0D,  "tip840drv"      },             /* TIP840-xx - 12-bit ADC                       */
    { 0xB3,  0x0E,  "tip813drv"      },             /* TIP813-xx - LON controller                   */
    { 0xB3,  0x0F,  "tip111drv"      },             /* TIP111-2x - 2 channel motion controller      */
    { 0xB3,  0x10,  "tip865drv"      },             /* TIP865-xx - 4 channel serial RS232/TTL/RS422 */
    { 0xB3,  0x11,  "tip850drv"      },             /* TIP850-1x - 12-bit ADC/DAC                   */
    { 0xB3,  0x12,  "tip815drv"      },             /* TIP815-xx - ARCNET Controller                */
    { 0xB3,  0x13,  "tip837drv"      },             /* TIP837-xx - 1 channel 16-bit ADC             */
    { 0xB3,  0x14,  "tip150drv"      },             /* TIP150-xx - 1/2 channel resolver             */
    { 0xB3,  0x15,  "tip812drv"      },             /* TIP812-xx - SERCOS controller                */
    { 0xB3,  0x16,  "tip811drv"      },             /* TIP811-xx - INTERBUS-S controller            */
    { 0xB3,  0x17,  "tip870drv"      },             /* TIP870-xx - PCMCIA                           */
    { 0xB3,  0x18,  "tip500drv"      },             /* TIP500-xx - 12-bit ADC                       */
    { 0xB3,  0x19,  "tip550drv"      },             /* TIP550-xx - 12-bit DAC                       */
    { 0xB3,  0x1A,  "tip605drv"      },             /* TIP605-10 - 16 digital Inputs                */
    { 0xB3,  0x1B,  "tip816drv"      },             /* TIP816-10 - Extended CAN (one channel)       */
    { 0xB3,  0x1C,  "tip903drv"      },             /* TIP903-10 - Extended CAN (three channel)     */
    { 0xB3,  0x1D,  "tip866drv"      },             /* TIP866-10 - 8 serial ports, RS232            */
    { 0xB3,  0x1E,  "tip120drv"      },             /* TIP120-xx - 1/2 axis motion controller       */
    { 0xB3,  0x1F,  "tip817drv"      },             /* TIP817-10 - Interbus Slave                   */
    { 0xB3,  0x20,  "tip906drv"      },             /* TIP906-10 - analog module                    */
    { 0xB3,  0x22,  "tip501drv"      },             /* TIP501-xx - Isolated 16-Bit ADC              */
    { 0xB3,  0x23,  "tip551drv"      },             /* TIP551-xx - Isolated 16-Bit DAC              */
    { 0xB3,  0x24,  "tip867drv"      },             /* TIP867-10 - 8 serial ports RS485             */
    { 0xB3,  0x27,  "tip915drv"      },             /* TIP915-11 - Special function + 2 serial ports*/
    { 0xB3,  0x29,  "tip818drv"      },             /* TIP818-xx - Fault Tolerant CAN Bus IP        */
    { 0xB3,  0x2A,  "tip114drv"      },             /* TIP114-xx - 10 channel SSI Interface         */
    { 0xB3,  0x2B,  "tip610drv"      },             /* TIP610-10 - 20 Digital I/O                   */
    { 0xB3,  0x2C,  "tip570drv"      },             /* TIP570-xx - 16x12-Bit ADC, 8x12-Bit DAC      */
    { 0xB3,  0x2E,  "tip606drv"      },             /* TIP606-10 - Digital Input (Avionic Applications)*/
    { 0xB3,  0x30,  "tip250drv"      },             /* TIP250-10 - 8MB Flash Memory                 */
    { 0xB3,  0x31,  "tip255drv"      },             /* TIP255-10 - 2MB battery backup SRAM          */
    { 0xB3,  0x32,  "tip302drv"      },             /* TIP302-12 - COM302-IP                        */
    { 0xB3,  0x33,  "tip710drv"      },             /* TIP710-10 - 16 Digital Output 1A high side   */
    { 0xB3,  0x34,  "tip672drv"      },             /* TIP672-10 - 24 Digital I/O differential      */
    { 0xB3,  0x36,  "tip675drv"      },             /* TIP675-10 - 48 Digital I/O tri-state         */
    { 0xB3,  0x37,  "tip116drv"      },             /* TIP116-10 - 4 Channel Quadrature / General Purpose Counter  */
    { 0xB3,  0x38,  "tip630drv"      },             /* TIP630-10 - IP - FPGA                        */
    { 0xB3,  0x39,  "tip845drv"      },             /* TIP845-10 - 48 Channel 14bit A/D             */
    { 0xB3,  0x3A,  "tip115drv"      },             /* TIP115-10 - 5 Channel Encoder Interface (SSI) Monitor  */
    { 0xB3,  0x3B,  "tip672_50drv"   },             /* TIP672-50 - 24 Digital I/O with differential I/O lines and Handshake Mode  */
    { 0xB3,  0x3C,  "tip991drv"      },             /* TIP991-10 - 10 Channel SSI Encoder Stimulator*/
    { 0xB3,  0x3D,  "tip119drv"      },             /* TIP119-10 - Six channel quadrature counter   */
    { 0xB3,  0x3E,  "tip360drv"      },             /* TIP360-10 - Quad Integrated Communication IP based on MC68360   */
    { 0xB3,  0x40,  "tip620drv"      },             /* TIP620-11 - Dual Programmable Interface/Timer*/
    { 0xB3,  0x41,  "tip620drv"      },             /* TIP620-10 - 48 Line Digital Interface        */
    {    0,     0,  NULL             },             /* end of list entry                            */
};


/* Dynamic module loading from driver context is only valid on Kernel 2.4 systems */

static struct carrier_PnP_entry  carrier_PnP_list[] = {
/*    vendor | device |  subvendor | subsystem  | driver name      */                    
    {  0x1498,  0x30C8,      0x1498,  PCI_ANY_ID, "carrier_tews_pci" },     /* TEWS TECHNOLOGIES - TPCI200 (4 slots)  */
    {  0x1498,  0x3064,      0x1498,  PCI_ANY_ID, "carrier_tews_pci" },     /* TEWS TECHNOLOGIES - TPCI100 (2 slots)  */
    {  0x1498,  0x20C9,      0x1498,  PCI_ANY_ID, "carrier_tews_pci" },     /* TEWS TECHNOLOGIES - TCP201  (4 slots)  */
    {  0x1498,  0x20D3,      0x1498,  PCI_ANY_ID, "carrier_tews_pci" },     /* TEWS TECHNOLOGIES - TCP211  (2 slots)  */
    {  0x1498,  0x20D4,      0x1498,  PCI_ANY_ID, "carrier_tews_pci" },     /* TEWS TECHNOLOGIES - TCP212  (2 slots)  */
    {  0x1498,  0x20D5,      0x1498,  PCI_ANY_ID, "carrier_tews_pci" },     /* TEWS TECHNOLOGIES - TCP213  (2 slots)  */
    {  0x1498,  0x20DC,      0x1498,  PCI_ANY_ID, "carrier_tews_pci" },     /* TEWS TECHNOLOGIES - TCP220  (4 slots)  */
    {  0x124B,  0x0040,  PCI_ANY_ID,  PCI_ANY_ID, "carrier_sbs_pci"  },     /* SBS - PCI40 (4 slots)                  */
    {       0,       0,           0,           0, NULL               },     /* end of list entry                      */
};

#endif      /* __PNPINF_H__ */
