/* $Header: /Tews/Device Driver/Linux/TDRV001/Code/pef20534.h 3     18.02.05 10:03 Hesse $ */
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
**    Project          TDRV001 Driver                                        **
**                                                                           **
**                                                                           **
**    File             pef20534.h                                            **
**                                                                           **
**                                                                           **
**    Function         Specific definitions for Infineon's PEF20534 / DSCC4  **
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
**                     Copyright (c) 2004-2005                               **
**                     TEWS TECHNOLOGIES GmbH                                **
**                                                                           **
**                                                                           **
**                                                                           **
*******************************************************************************
*******************************************************************************/




#ifndef _PEF20534_H
#define _PEF20534_H


#define SccInt              (1 << 25)

/* GLOBAL registers definitions */
#define GCMDR               0x00
#define GSTAR               0x04
#define GMODE               0x08
#define IQLENR0             0x0C
#define IQLENR1             0x10
#define IQRX0               0x14
#define IQTX0               0x24
#define IQCFGBAR            0x3c
#define IQPBAR              0x40
#define FIFOCR1             0x44
#define FIFOCR2             0x48
#define FIFOCR3             0x4c
#define FIFOCR4             0x34
#define CH0CFG              0x50
#define CH0BRDA             0x54
#define CH0BTDA             0x58
#define CH0FRDA             0x98
#define CH0FTDA             0xb0
#define CH0LRDA             0xc8
#define CH0LTDA             0xe0

/* SCC registers definitions */
#define SCC_START	        0x0100
#define SCC_OFFSET          0x80
#define CMDR                0x00
#define STAR                0x04
#define CCR0                0x08
#define CCR1                0x0c
#define CCR2                0x10
#define BRR                 0x2C
#define RLCR                0x40
#define IMR                 0x54
#define ISR                 0x58

/* PERIPHERAL registers definitions */
#define LCONF               0x300
#define SSCCON              0x380
#define SSCBR               0x384
#define SSCTB               0x388
#define SSCRB               0x38C
#define SSCCSE              0x390
#define SSCIM               0x394
#define GPDIR               0x400
#define GPDATA              0x404
#define GPIM                0x408



/* Bit masks */
#define RxBNO               0x1fff0000

#define EncodingMask	    0x00700000
#define CrcMask		        0x00000003

#define IntRxScc0	        0x10000000
#define IntTxScc0	        0x01000000

#define TxPollCmd	        0x00000400
#define RxActivate	        0x08000000
#define MTFi		        0x04000000
#define Rdr		            0x00400000
#define Rdt		            0x00200000
#define Idr		            0x00100000
#define Idt		            0x00080000
#define TxSccRes	        0x01000000
#define RxSccRes	        0x00010000
#define TxSizeMax	        0x1fff

#define Ccr0ClockMask	    0x0000003f
#define Ccr1LoopMask	    0x00000200
#define IsrMask		        0x000fffff
#define BrrExpMask	        0x00000f00
#define BrrMultMask	        0x0000003f
#define EncodingMask	    0x00700000
#define Hold		        0x40000000
#define SccBusy		        0x10000000
#define PowerUp		        0x80000000
#define FrameOk		        (FrameVfr | FrameCrc)
#define FrameVfr	        0x80
#define FrameRdo	        0x40
#define FrameCrc	        0x20
#define FrameRab	        0x10
#define FrameAborted	    0x00000200
#define FrameEnd	        0x80000000
#define DataComplete	    0x40000000
#define LengthCheck	        0x00008000
#define SccEvt		        0x02000000
#define NoAck		        0x00000200
#define Action		        0x00000001
#define HiDesc		        0x20000000

/* SCC events */
#define RxEvt		        0xf0000000
#define TxEvt		        0x0f000000
#define Alls		        0x00040000
#define Xdu		            0x00010000
#define Cts		            0x00004000
#define Xmr		            0x00002000
#define Xpr		            0x00001000
#define Rdo		            0x00000080
#define Rfs		            0x00000040
#define Cd		            0x00000004
#define Rfo		            0x00000002
#define Flex		        0x00000001

/* DMA core events */
#define Cfg		            0x00200000
#define Hi		            0x00040000
#define Fi		            0x00020000
#define Err		            0x00010000
#define Arf		            0x00000002
#define ArAck	        	0x00000001

/* Misc */
#define NeedIDR		        0x00000001
#define NeedIDT		        0x00000002
#define RdoSet		        0x00000004






/********************************************************************************/
/* General Registers */
/*#define GCMDR		0x0000	*/	/* Global Command Register */
#define  AR		(1<<0)		/* Action Request */
#define  IMAR		(1<<9)		/* Interrupt Mask Action Request */
#define  TXPR0		(1<<10)		/* Transmit Poll Request Channel 0 */
#define  TXPR1		(1<<11)		/* Transmit Poll Request Channel 1 */
#define  TXPR2		(1<<12)		/* Transmit Poll Request Channel 2 */
#define  TXPR3		(1<<13)		/* Transmit Poll Request Channel 3 */
#define  CFGIQP		(1<<20)		/* Configure IQ Peripheral */
#define  CFGIQCFG	(1<<21)		/* Configure IQ Peripheral */
#define  CFGIQSCC0TX	(1<<24)		/* Configure IQ SCC0 Transmit */
#define  CFGIQSCC1TX	(1<<25)		/* Configure IQ SCC1 Transmit */
#define  CFGIQSCC2TX	(1<<26)		/* Configure IQ SCC2 Transmit */
#define  CFGIQSCC3TX	(1<<27)		/* Configure IQ SCC3 Transmit */
#define  CFGIQSCC0RX	(1<<28)		/* Configure IQ SCC0 Receive */
#define  CFGIQSCC1RX	(1<<29)		/* Configure IQ SCC1 Receive */
#define  CFGIQSCC2RX	(1<<30)		/* Configure IQ SCC2 Receive */
#define  CFGIQSCC3RX	(1<<31)		/* Configure IQ SCC3 Receive */
#define  ARACK		(1<<0)		/* Action Request Acknowledge Status */
#define  ARF		(1<<1)		/* Action Request Failed Status */
#define  IIPGPP		(1<<16)		/* Int. Indication Peripheral Queue GPP */
#define  IIPLBI		(1<<18)		/* Int. Indication Peripheral Queue LBI */
#define  IIPSSC		(1<<19)		/* Int. Indication Peripheral Queue SSC */
#define  IICFG		(1<<21)		/* Int. Indication Configuration Queue */
#define  IISCC0TX	(1<<24)		/* Int. Indication Queue SCC0 TX */
#define  IISCC1TX	(1<<25)		/* Int. Indication Queue SCC1 TX */
#define  IISCC2TX	(1<<26)		/* Int. Indication Queue SCC2 TX */
#define  IISCC3TX	(1<<27)		/* Int. Indication Queue SCC3 TX */
#define  IISCC0RX	(1<<28)		/* Int. Indication Queue SCC0 RX */
#define  IISCC1RX	(1<<29)		/* Int. Indication Queue SCC1 RX */
#define  IISCC2RX	(1<<30)		/* Int. Indication Queue SCC2 RX */
#define  IISCC3RX	(1<<31)		/* Int. Indication Queue SCC3 RX */
#define  CMODE		(1<<0)		/* DMA Control Mode */
#define  DBE		(1<<1)		/* DEMUX Burst Enable */
#define  ENDIAN		(1<<2)		/* Endian Selection */
#define  CHN		(1<<13)		/* Channel Number Highest Priority */
#define  SPRI		(1<<15)		/* Select Priority */
#define  PERCFG		(1<<16)		/* Peripheral Block Configuration */
#define  LCD		(1<<19)		/* LBI Clock Division */
#define  OSCPD		(1<<21)		/* Oscillator Power Down */

/* IRQ Queue Control Registers */
#define  IQSCC0TXLEN	(1<<12)		/* Interrupt Queue SCC0 TX Length */
#define  IQSCC1TXLEN	(1<<8)		/* Interrupt Queue SCC1 TX Length */
#define  IQSCC2TXLEN	(1<<4)		/* Interrupt Queue SCC2 TX Length */
#define  IQSCC3TXLEN	(1<<0)		/* Interrupt Queue SCC3 TX Length */
#define  IQSCC0RXLEN	(1<<28)		/* Interrupt Queue SCC0 RX Length */
#define  IQSCC1RXLEN	(1<<24)		/* Interrupt Queue SCC1 RX Length */
#define  IQSCC2RXLEN	(1<<20)		/* Interrupt Queue SCC2 RX Length */
#define  IQSCC3RXLEN	(1<<16)		/* Interrupt Queue SCC3 RX Length */
#define IQLENR2		0x0010		/* Interrupt Queue Length Register 2 */
#define  IQPLEN		(1<<16)		/* Interrupt Queue Peripheral Length */
#define  IQCFGLEN	(1<<20)		/* Interrupt Queue Configuration Length */
#define IQSCC0RXBAR	0x0014		/* Interrupt Queue SCC0 RX Base Address */
#define IQSCC1RXBAR	0x0018		/* Interrupt Queue SCC1 RX Base Address */
#define IQSCC2RXBAR	0x001c		/* Interrupt Queue SCC2 RX Base Address */
#define IQSCC3RXBAR	0x0020		/* Interrupt Queue SCC3 RX Base Address */
#define IQSCC0TXBAR	0x0024		/* Interrupt Queue SCC0 TX Base Address */
#define IQSCC1TXBAR	0x0028		/* Interrupt Queue SCC1 TX Base Address */
#define IQSCC2TXBAR	0x002c		/* Interrupt Queue SCC2 TX Base Address */
#define IQSCC3TXBAR	0x0030		/* Interrupt Queue SCC3 TX Base Address */
#define  TFFTHRES0	(1<<0)		/* Transmit FIFO Forward Threshold Chan. 0 */
#define  TFFTHRES1	(1<<8)		/* Transmit FIFO Forward Threshold Chan. 1 */
#define  TFFTHRES2	(1<<16)		/* Transmit FIFO Forward Threshold Chan. 2 */
#define  TFFTHRES3	(1<<24)		/* Transmit FIFO Forward Threshold Chan. 3 */


/* SCC registers */
#define  RNR		(1<<0)		/* Receiver Not Ready Command */
#define  STI		(1<<8)		/* Start Timer Command */
#define  RRES		(1<<16)		/* Receiver Reset Command */
#define  RFRD		(1<<17)		/* Receive FIFO Read Enable Command */
#define  HUNT		(1<<18)		/* Enter Hunt State Command */
#define  XRES		(1<<24)		/* Transmitter Reset Command */
#define  RRNR		(1<<16)		/* Received RNR Status */
#define  XRNR		(1<<17)		/* Transmit RNR Status */
#define  WFA		(1<<18)		/* Wait For Acknowledgement */
#define  DPLA		(1<<19)		/* DPLL Asynchronous */
#define  RLI		(1<<20)		/* Receive Line Inactive */
#define  CD		(1<<21)		/* Carrier Detect Input Signal State */
#define  RFNE		(1<<22)		/* Receive FIFO Not Empty */
#define  SYNC		(1<<23)		/* Synchronisation Status */
#define  CTS		(1<<24)		/* Clear To Send Input Signal State */
#define  FCS		(1<<27)		/* Flow Control Status */
#define  CEC		(1<<28)		/* Command Executing */
#define  TEC		(1<<29)		/* TIC executing */
#define  CM		(1<<0)		/* Clock Mode */
#define  CM0		(1<<0)
#define  CM1		(1<<1)
#define  CM2		(1<<2)
#define  HS		(1<<3)		/* High Speed (PEB-20534H-52) */
#define  SSEL		(1<<4)		/* Clock Source Select (a/b Select) */
#define  TOE		(1<<5)		/* Transmit Clock Out Enable */
#define  BCR		(1<<7)		/* Bit Clock Rate */
#define	 PSD		(1<<8)		/* DPLL Phase Shift Disable */
#define  VIS		(1<<12)		/* Masked Interrupts Visible */
#define  SM		(1<<16)		/* Serial Port Mode */
#define  SM0		(1<<16)
#define	 SM1		(1<<17)
#define  SC		(1<<20)		/* Serial Port Configuration */
#define  SC0		(1<<20)
#define  SC1		(1<<21)
#define  SC2		(1<<22)
#define  PU		(1<<31)		/* Power Up */
#define  C32		(1<<0)		/* CRC-32 Select */
#define  TOLEN		(1<<0)		/* Time Out Length */
#define  CRL		(1<<1)		/* CRC Reset Value */
#define  SFLAG		(1<<7)		/* Shared Flags Transmission */
#define  TOIE		(1<<7)		/* Time Out Indication Enable */
#define  TLP		(1<<8)		/* Test Loop */
#define  MCS		(1<<9)		/* Modulo Count Select */
#define  PPM0		(1<<10)		/* PPP Mode Select 0 */
#define  BISNC		(1<<10)		/* Enable BISYNC Mode */
#define  PPM1		(1<<11)		/* PPP Mode Select 1 */
#define  SLEN		(1<<11)		/* SYNC Character Length */
#define  NRM		(1<<12)		/* Normal Response Mode */
#define  ADM		(1<<13)		/* Address Mode Select */
#define  MDS0		(1<<14)		/* Mode Select (HDLC Protocol Sub-Mode) */
#define  MDS1		(1<<15)
#define  CAS		(1<<17)		/* Carrier Detect Auto Start */
#define  FCTS		(1<<18)		/* Flow Control (Using Signal /CTS) */
#define  FRTS		(1<<19)		/* Flow Control (Using Signal /RTS) */
#define  RTS		(1<<20)		/* Request To Send Pin Control */
#define  TCLKO		(1<<21)		/* Transmit Clock Output */
#define  ICD		(1<<22)		/* Invert Carrier Detect Pin Polarity */
#define  ODS		(1<<25)		/* Output Driver Select */
#define  DIV		(1<<26)		/* Data Inversion */
#define  SOC0		(1<<28)		/* Serial Output Control */
#define  SOC1		(1<<29)
#define  XCRC		(1<<0)		/* Transmit CRC Checking Mode */
#define  FLON		(1<<0)		/* Flow Control Enable */
#define  CRCM		(1<<0)		/* CRC Mode Select */
#define  OIN		(1<<1)		/* One Insertion */
#define  CAPP		(1<<1)		/* CRC Append */
#define  SXIF		(1<<2)		/* Selects Transmission Of I-Frames */
#define  CRLBS		(1<<2)		/* CRC Reset Value In BISYNC Mode */
#define  ITF		(1<<3)		/* Interframe Time Fill */
#define  PRE0		(1<<4)		/* Number Of Preamble Repetitions */
#define  PRE1		(1<<5)
#define  EPT		(1<<7)		/* Enable Preamble Transmission */
#define  PRE		(1<<8)		/* Preamble */
#define  RFTH		(1<<16)		/* Receive FIFO Threshold */
#define  RFDF		(1<<19)		/* Receive FIFO Data Format */
#define  RADD		(1<<20)		/* Receive Address Pushed To FIFO */
#define  DPS		(1<<20)		/* Data Parity Storage */
#define  RCRC		(1<<21)		/* Receive CRC Checking Mode */
#define  PARE		(1<<21)		/* Parity Enable */
#define  DRCRC		(1<<22)		/* Disable Receive CRC Checking */
#define  PAR0		(1<<22)		/* Parity Format */
#define  PAR1		(1<<23)
#define  STOP		(1<<24)		/* Stop Bit Number */
#define  SLOAD		(1<<24)		/* Enable SYNC Character Load */
#define  XBRK		(1<<25)		/* Transmit Break */
#define  DXS		(1<<26)		/* Disable Storage of XON/XOFF-Characters */
#define  RAC		(1<<27)		/* Receiver Active */
#define  CHL0		(1<<28)		/* Character Length */
#define  CHL1		(1<<29)
#define ACCM		0x0014		/* ASYNC Control Character Map */
#define UDAC		0x0018		/* User Defined ASYNC Character */
#define  AC0		(1<<0)		/* User Defined ASYNC Character Control Map */
#define  AC1		(1<<8)		/* User Defined ASYNC Character Control Map */
#define  AC2		(1<<16)		/* User Defined ASYNC Character Control Map */
#define  AC3		(1<<24)		/* User Defined ASYNC Character Control Map */
#define TTSA		0x001c		/* TX Time Slot Assignment Register */
#define  TCC		(1<<0)		/* Transmit Channel Capacity */
#define  TEPCM		(1<<15)		/* Enable PCM Mask Transmit */
#define  TCS		(1<<16)		/* Transmit Clock Shift */
#define  TTSN		(1<<24)		/* Transmit Time Slot Number */
#define RTSA		0x0020		/* RX Time Slot Assignment Register */
#define  RCC		(1<<0)		/* Receive Channel Capacity */
#define  REPCM		(1<<15)		/* Enable PCM Mask Receive */
#define  RCS		(1<<16)		/* Receive Clock Shift */
#define  RTSN		(1<<24)		/* Receive Time Slot Number */
#define PCMMTX		0x0024		/* PCM Mask for Transmit */
#define PCMMRX		0x0028		/* PCM Mask for Receive */
#define  BRN		(1<<0)		/* Baud Rate Factor N */
#define  BRM		(1<<8)		/* Baud Rate Factor M   k=(N+1)*2^M */
#define TIMR		0x0030		/* Timer Register */
#define  TVALUE		(1<<0)		/* Timer Expiration Value */
#define  CNT		(1<<24)		/* Counter */
#define  TMD		(1<<28)		/* Timer Mode */
#define  SRC		(1<<31)		/* Clock Source */
#define XADR		0x0034		/* TX Address Register */
#define  XAD1		(1<<0)		/* Transmit Address 1 */
#define  XAD2		(1<<8)		/* Transmit Address 2 */
#define RADR		0x0038		/* RX Address Register */
#define  RAL1		(1<<16)		/* RX Address 1 Low-Byte */
#define  RAH1		(1<<24)		/* RX Address 1 High-Byte */
#define  RAL2		(1<<0)		/* RX Address 2 Low-Byte */
#define  RAH2		(1<<8)		/* RX Address 2 High-Byte */
#define RAMR		0x003c		/* Receive Address Mask Register */
#define  AMRAL1		(1<<0)		/* Receive Mask Address 1 Low-Byte */
#define  AMRAH1		(1<<8)		/* Receive Mask Address 1 High-Byte */
#define  AMRAL2		(1<<16)		/* Receive Mask Address 2 Low-Byte */
#define  AMRAH2		(1<<24)		/* Receive Mask Address 2 High-Byte */
#define  RL		(1<<0)		/* Receive Length Check Limit */
#define  RCE		(1<<15)		/* Receive Length Check Enable */
#define XNXFR		0x0044		/* XON/XOFF Register */
#define  MXOFF		(1<<0)		/* XOFF Character Mask */
#define  MXON		(1<<8)		/* XON Character Mask */
#define  CXOFF		(1<<16)		/* XOFF Character */
#define  CXON		(1<<24)		/* XON Character */
#define TCR		0x0048		/* Termination Character Register */
#define  TC		(1<<0)		/* Termination Character */
#define  TCDE		(1<<15)		/* Termination Character Detection Enable */
#define TICR		0x004c		/* Transmit Immediate Character Register */
#define SYNCR		0x0050		/* Synchronization Character Register */
#define  SYNCL		(1<<0)		/* Synchronization Character Low */
#define  SYNCH		(1<<8)		/* Synchronization Character High */
#define  FLEX		(1<<0)		/* Frame Length Exceeded Interrupt */
#define  RFO		(1<<1)		/* RX FIFO Overflow Interrupt */
#define  CDSC		(1<<2)		/* Carrier Detect Status Change Interrupt */
#define  PLLA		(1<<3)		/* DPLL Asynchronous Interrupt */
#define  PCE		(1<<4)		/* Protocol Error Interrupt */
#define  FERR		(1<<4)		/* Framing Error Interrupt */
#define  SCD		(1<<4)		/* SYN Character Detected Interrupt */
#define  RSC		(1<<5)		/* Receive Status Change Interrupt */
#define  PERR		(1<<5)		/* Parity Error Interrupt */
#define  RFS		(1<<6)		/* Receive Frame Start Interrupt */
#define  TOUT		(1<<6)		/* Time Out Interrupt */
#define  RDO		(1<<7)		/* Receive Data Overflow Interrupt */
#define  TCD		(1<<7)		/* Termination Character Detected Interrupt */
#define  BRKT		(1<<8)		/* Break Terminated Interrupt */
#define  BRK		(1<<9)		/* Break Interrupt */
#define  XPR		(1<<12)		/* Transmit Pool Ready Interrupt */
#define  XMR		(1<<13)		/* Transmit Message Repeat */
#define  XON		(1<<13)		/* XOFF Character Detected Interrupt */
#define  CSC		(1<<14)		/* /CTS Status Change */
#define  TIN		(1<<15)		/* Timer Interrupt */
#define  XDU		(1<<16)		/* Transmit Data Underrun Interrupt */
#define  ALLS		(1<<18)		/* All Sent Interrupt */


/* DMAC control registers */
#define  TFSIZE0	(1<<27)		/* Transmit FIFO Size Channel 0 */
#define  TFSIZE1	(1<<22)		/* Transmit FIFO Size Channel 1 */
#define  TFSIZE2	(1<<17)		/* Transmit FIFO Size Channel 2 */
#define  TFSIZE3	(1<<11)		/* Transmit FIFO Size Channel 3 */
#define  M4_0		(1<<7)		/* Multiplier 4 FIFO Channel 0 */
#define  M2_0		(1<<6)		/* Multiplier 2 FIFO Channel 0 */
#define  M4_1		(1<<5)		/* Multiplier 4 FIFO Channel 1 */
#define  M2_1		(1<<4)		/* Multiplier 2 FIFO Channel 1 */
#define  M4_2		(1<<3)		/* Multiplier 4 FIFO Channel 2 */
#define  M2_2		(1<<2)		/* Multiplier 2 FIFO Channel 2 */
#define  M4_3		(1<<1)		/* Multiplier 4 FIFO Channel 3 */
#define  M2_3		(1<<0)		/* Multiplier 2 FIFO Channel 3 */
#define  TFRTHRES0	(1<<27)		/* Transmit FIFO Refill Threshold Chan. 0 */
#define  TFRTHRES1	(1<<22)		/* Transmit FIFO Refill Threshold Chan. 1 */
#define  TFRTHRES2	(1<<17)		/* Transmit FIFO Refill Threshold Chan. 2 */
#define  TFRTHRES3	(1<<11)		/* Transmit FIFO Refill Threshold Chan. 3 */
#define  RFTHRES	(1<<0)		/* RX FIFO Threshold */
#define  M2		(1<<7)		/* RX FIFO Threshold Multiplier 2 */
#define  M4		(1<<8)		/* RX FIFO Threshold Multiplier 4 */
#define CH1CFG		0x005c		/* Channel 1 Configuration */
#define CH1BRDA		0x0060		/* Channel 1 Base Address RX Descriptor */
#define CH1BTDA		0x0064		/* Channel 1 Base Address TX Descriptor */
#define CH2CFG		0x0068		/* Channel 2 Configuration */
#define CH2BRDA		0x006c		/* Channel 2 Base Address RX Descriptor */
#define CH2BTDA		0x0070		/* Channel 2 Base Address TX Descriptor */
#define CH3CFG		0x0074		/* Channel 3 Configuration */
#define CH3BRDA		0x0078		/* Channel 3 Base Address RX Descriptor */
#define CH3BTDA		0x007c		/* Channel 3 Base Address TX Descriptor */
#define  IDT		(1<<19)
#define  IDR		(1<<20)
#define  RDT		(1<<21)
#define  RDR		(1<<22)
#define  MTERR		(1<<24)		/* Mask TX ERR-Interrupt */
#define  MRERR		(1<<25)		/* Mask RX ERR-Interrupt */
#define  MTFI		(1<<26)		/* Mask RX FI-Interrupt */
#define  MRFI		(1<<27)		/* Mask TX FI-Interrupt */
#define CH1FRDA		0x009c		/* Channel 1 First RX Descriptor Address */
#define CH2FRDA		0x00a0		/* Channel 2 First RX Descriptor Address */
#define CH3FRDA		0x00a4		/* Channel 3 First RX Descriptor Address */
#define CH1FTDA		0x00b4		/* Channel 1 First TX Descriptor Address */
#define CH2FTDA		0x00b8		/* Channel 2 First TX Descriptor Address */
#define CH3FTDA		0x00bc		/* Channel 3 First TX Descriptor Address */
#define CH1LRDA		0x00cc		/* Channel 1 Last RX Descriptor Address */
#define CH2LRDA		0x00d0		/* Channel 2 Last RX Descriptor Address */
#define CH3LRDA		0x00d4		/* Channel 3 Last RX Descriptor Address */
#define CH1LTDA		0x00e4		/* Channel 1 Last TX Descriptor Address */
#define CH2LTDA		0x00e8		/* Channel 2 Last TX Descriptor Address */
#define CH3LTDA		0x00ec		/* Channel 3 Last TX Descriptor Address */
/**********************************************************************************/



#endif
