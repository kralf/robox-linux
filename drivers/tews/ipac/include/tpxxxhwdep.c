/* $Id: tpxxxhwdep.c 9 2005-10-17 15:11:07Z Welzel $ */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @ t p x x x h w d e p @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          TPMC Linux Device Driver                              **
**                                                                           **
**    File             tpxxxhwdep.c                                          **
**                                                                           **
**    Function         Collection of hardware dependent functions            **
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
**                     Copyright (c) 2005                                    **
**                     TEWS TECHNOLOGIES GmbH                                **
**                                                                           **
**                                                                           **
**    $Date: 2005-10-17 17:11:07 +0200 (Mo, 17 Okt 2005) $   $Rev: 9 $       **
**                                                                           **
*******************************************************************************
*******************************************************************************/

#include "tpxxxhwdep.h"


/*
//  I/O space read and write functions.
//
//  The READ/WRITE_REGISTER_* calls manipulate I/O registers in MEMORY space.
//
//  The READ/WRITE_PORT_* calls manipulate I/O registers in PORT space.
*/


/*****************************************************************************
**  READ_REGISTER_UCHAR    - Read 8 bit from memory space 
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS unsigned char READ_REGISTER_UCHAR(void *pReg) 

{
    return readb((unsigned char*)pReg);
}


/*****************************************************************************
**  READ_REGISTER_BUFFER_UCHAR     - Read a buffer of 8 bit data from memory 
**                                   space 
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS void READ_REGISTER_BUFFER_UCHAR(void *pReg, unsigned char *pBuf, unsigned long count) 
{
	unsigned char *temp = (unsigned char *)pReg;

	while (count--)
	{
		*(pBuf++) = readb(temp);
		temp++;
	}
}


/*****************************************************************************
**  WRITE_REGISTER_UCHAR    - Write 8 bit to memory space 
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS void WRITE_REGISTER_UCHAR(void *pReg, unsigned char value) 

{
    writeb(value, (unsigned char*)pReg);
}


/*****************************************************************************
**  WRITE_REGISTER_BUFFER_UCHAR   - Write a buffer of 8 bit data to memory 
**                                  space
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS void WRITE_REGISTER_BUFFER_UCHAR(void *pReg, unsigned char *pBuf, unsigned long count) 
{
	unsigned char *temp = (unsigned char *)pReg;

	while (count--)
	{
		writeb(*(pBuf++), temp);
		temp++;
	}
}


/*****************************************************************************
**  READ_REGISTER_USHORT    - Read 16 bit from memory space and swap bytes
**                            for Intel x86 based plattforms
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS unsigned short READ_REGISTER_USHORT(void *pReg) 

{
	unsigned short temp_u16;
	temp_u16 = readw((unsigned short*)pReg);
    return SWAPS(temp_u16);
}


/*****************************************************************************
**  READ_REGISTER_BUFFER_USHORT    - Read a buffer of 16 bit words from memory 
**                                   space and swap bytes for Intel x86 based 
**                                   plattforms
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS void READ_REGISTER_BUFFER_USHORT(void *pReg, unsigned short *pBuf, unsigned long count) 
{
	unsigned short *temp = (unsigned short *)pReg;
	unsigned short temp_u16;

	while (count--)
	{
		temp_u16 = readw(temp);
		*(pBuf++) = SWAPS(temp_u16);
		temp++;
	}
}


/*****************************************************************************
**  WRITE_REGISTER_USHORT   - Write 16 bit to memory space and swap bytes
**                            for Intel x86 based plattforms
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS void WRITE_REGISTER_USHORT(void *pReg, unsigned short value) 

{
    writew(SWAPS(value), (unsigned short*)pReg);
}


/*****************************************************************************
**  WRITE_REGISTER_BUFFER_USHORT   - Write a buffer of 16 bit words to memory 
**                            space and swap bytes for Intel x86 based plattforms
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS void WRITE_REGISTER_BUFFER_USHORT(void *pReg, unsigned short *pBuf, unsigned long count) 
{
	unsigned short *temp = (unsigned short *)pReg;

	while (count--)
	{
		writew(SWAPS(*pBuf), temp);
		temp++;
		pBuf++;
	}
}


/*****************************************************************************
**  READ_REGISTER_ULONG     - Read 32 bit from memory space and swap bytes
**                            for Intel x86 based plattforms
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS unsigned long READ_REGISTER_ULONG(void *pReg) 
{
	unsigned long temp_u32;
	temp_u32 = readl((unsigned long*)pReg);
    return SWAPL(temp_u32);
}


/*****************************************************************************
**  READ_REGISTER_BUFFER_ULONG     - Read a buffer of 32 bit longwords from memory 
**                                   space and swap bytes for Intel x86 based 
**                                   plattforms
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS void READ_REGISTER_BUFFER_ULONG(void *pReg, unsigned long *pBuf, unsigned long count) 
{
	unsigned long *temp = (unsigned long *)pReg;
	unsigned long temp_u32;

	while (count--)
	{
		temp_u32 = readl(temp);
		*(pBuf++) = SWAPL(temp_u32);
		temp++;
	}
}


/*****************************************************************************
**  WRITE_REGISTER_ULONG    - Write 32 bit to memory space and swap bytes
**                            for Intel x86 based plattforms
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS void WRITE_REGISTER_ULONG(void *pReg, unsigned long value) 
{
    writel(SWAPL(value), (unsigned long*)pReg);
}


/*****************************************************************************
**  WRITE_REGISTER_BUFFER_ULONG   - Write a buffer of 32 bit longwords to memory 
**                            space and swap bytes for Intel x86 based plattforms
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS void WRITE_REGISTER_BUFFER_ULONG(void *pReg, unsigned long *pBuf, unsigned long count) 
{
	unsigned long *temp = (unsigned long *)pReg;

	while (count--)
	{
		writel(SWAPL(*pBuf), temp);
		temp++;
		pBuf++;
	}
}



/*****************************************************************************
**  READ_PORT_UCHAR    - Read 8 bit from I/O port space 
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS unsigned char READ_PORT_UCHAR(void *pReg) 

{
    return inb((unsigned)pReg);
}


/*****************************************************************************
**  READ_PORT_BUFFER_UCHAR     - Read a buffer of 8 bit data from I/O port space 
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS void READ_PORT_BUFFER_UCHAR(void *pReg, unsigned char *pBuf, unsigned long count) 
{
	unsigned char *temp = (unsigned char *)pReg;

	while (count--)
	{
		*(pBuf++) = inb((unsigned)temp);
		temp++;
	}
}


/*****************************************************************************
**  WRITE_PORT_UCHAR    - Write 8 bit to I/O port space 
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS void WRITE_PORT_UCHAR(void *pReg, unsigned char value) 

{
	outb(value, (unsigned)pReg);
}


/*****************************************************************************
**  WRITE_PORT_BUFFER_UCHAR   - Write a buffer of 8 bit data to I/O port space
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS void WRITE_PORT_BUFFER_UCHAR(void *pReg, unsigned char *pBuf, unsigned long count) 
{
	unsigned char *temp = (unsigned char *)pReg;

	while (count--)
	{
		outb(*(pBuf++), (unsigned)temp);
		temp++;
	}
}


/*****************************************************************************
**  READ_PORT_USHORT    - Read 16 bit from I/O port space and swap bytes
**                        for Intel x86 based plattforms
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS unsigned short READ_PORT_USHORT(void *pReg) 
{
	unsigned short temp_u16;
	temp_u16 = inw((unsigned)pReg);
    return SWAPS(temp_u16);
}


/*****************************************************************************
**  READ_PORT_BUFFER_USHORT    - Read a buffer of 16 bit words from I/O port 
**                               space and swap bytes for Intel x86 based 
**                               plattforms
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS void READ_PORT_BUFFER_USHORT(void *pReg, unsigned short *pBuf, unsigned long count) 
{
	unsigned short *temp = (unsigned short *)pReg;
	unsigned short temp_u16;

	while (count--)
	{
		temp_u16 = inw((unsigned)temp);
		*(pBuf++) = SWAPS(temp_u16);
		temp++;
	}
}


/*****************************************************************************
**  WRITE_PORT_USHORT   - Write 16 bit to I/O port space and swap bytes
**                        for Intel x86 based plattforms
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS void WRITE_PORT_USHORT(void *pReg, unsigned short value) 
{
    outw(SWAPS(value), (unsigned)pReg);
}


/*****************************************************************************
**  WRITE_PORT_BUFFER_USHORT   - Write a buffer of 16 bit words to I/O port 
**                            space and swap bytes for Intel x86 based plattforms
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS void WRITE_PORT_BUFFER_USHORT(void *pReg, unsigned short *pBuf, unsigned long count) 
{
	unsigned short *temp = (unsigned short *)pReg;

	while (count--)
	{
		outw(SWAPS(*pBuf), (unsigned)temp);
		temp++;
		pBuf++;
	}
}


/*****************************************************************************
**  READ_PORT_ULONG     - Read 32 bit from I/O port space and swap bytes
**                        for Intel x86 based plattforms
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS unsigned long READ_PORT_ULONG(void *pReg) 
{
	unsigned long temp_u32;
	temp_u32 = inl((unsigned)pReg);
    return SWAPL(temp_u32);
}


/*****************************************************************************
**  READ_PORT_BUFFER_ULONG     - Read a buffer of 32 bit longwords from I/O port 
**                               space and swap bytes for Intel x86 based 
**                               plattforms
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS void READ_PORT_BUFFER_ULONG(void *pReg, unsigned long *pBuf, unsigned long count) 
{
	unsigned long *temp = (unsigned long *)pReg;
	unsigned long temp_u32;

	while (count--)
	{
		temp_u32 = inl((unsigned)temp);
		*(pBuf++) = SWAPL(temp_u32);
		temp++;
	}
}


/*****************************************************************************
**  WRITE_PORT_ULONG    - Write 32 bit to I/O port space and swap bytes
**                        for Intel x86 based plattforms
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS void WRITE_PORT_ULONG(void *pReg, unsigned long value) 

{
    outl(SWAPL(value), (unsigned)pReg);
}


/*****************************************************************************
**  WRITE_PORT_BUFFER_ULONG   - Write a buffer of 32 bit longwords to I/O port 
**                            space and swap bytes for Intel x86 based plattforms
******************************************************************************/
TPXXXHWDEP_STORAGE_CLASS void WRITE_PORT_BUFFER_ULONG(void *pReg, unsigned long *pBuf, unsigned long count) 
{
	unsigned long *temp = (unsigned long *)pReg;

	while (count--)
	{
		outl(SWAPL(*pBuf), (unsigned)temp);
		temp++;
		pBuf++;
	}
}
