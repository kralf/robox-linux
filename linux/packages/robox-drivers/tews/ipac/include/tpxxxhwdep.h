/* $Id: tpxxxhwdep.h 9 2005-10-17 15:11:07Z Welzel $ */
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
**    File             tpxxxhwdep.h                                          **
**                                                                           **
**    Function         Hardware dependent functions Include                  **
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
**                     Copyright (c) 2004                                    **
**                     TEWS TECHNOLOGIES GmbH                                **
**                                                                           **
**                                                                           **
**    $Date: 2005-10-17 17:11:07 +0200 (Mo, 17 Okt 2005) $   $Rev: 9 $       **
**                                                                           **
*******************************************************************************
*******************************************************************************/
#ifndef __TPXXXHWDEP_H__
#define __TPXXXHWDEP_H__

#define TPXXXHWDEP_STORAGE_CLASS static
#define TPXXXHWDEP_FUNC_ATTR __attribute__ ((unused))

#define	SWAPL(x)	((((x) >> 24) & 0x000000FF) | (((x) >> 8) & 0x0000FF00) | (((x) << 8) & 0x00FF0000) | (((x) << 24) & 0xFF000000))
#define	SWAPS(x)	((((x) >> 8) & 0x00FF) | (((x) << 8) & 0xFF00))

/*
** MEM (Register) Access Functions
*/
TPXXXHWDEP_STORAGE_CLASS unsigned char READ_REGISTER_UCHAR(void *pReg) TPXXXHWDEP_FUNC_ATTR; 
TPXXXHWDEP_STORAGE_CLASS void READ_REGISTER_BUFFER_UCHAR(void *pReg, unsigned char *pBuf, unsigned long count) TPXXXHWDEP_FUNC_ATTR;
TPXXXHWDEP_STORAGE_CLASS void WRITE_REGISTER_UCHAR(void *pReg, unsigned char value) TPXXXHWDEP_FUNC_ATTR;
TPXXXHWDEP_STORAGE_CLASS void WRITE_REGISTER_BUFFER_UCHAR(void *pReg, unsigned char *pBuf, unsigned long count) TPXXXHWDEP_FUNC_ATTR;

TPXXXHWDEP_STORAGE_CLASS unsigned short READ_REGISTER_USHORT(void *pReg) TPXXXHWDEP_FUNC_ATTR;
TPXXXHWDEP_STORAGE_CLASS void READ_REGISTER_BUFFER_USHORT(void *pReg, unsigned short *pBuf, unsigned long count) TPXXXHWDEP_FUNC_ATTR;
TPXXXHWDEP_STORAGE_CLASS void WRITE_REGISTER_USHORT(void *pReg, unsigned short value) TPXXXHWDEP_FUNC_ATTR;
TPXXXHWDEP_STORAGE_CLASS void WRITE_REGISTER_BUFFER_USHORT(void *pReg, unsigned short *pBuf, unsigned long count) TPXXXHWDEP_FUNC_ATTR;

TPXXXHWDEP_STORAGE_CLASS unsigned long READ_REGISTER_ULONG(void *pReg) TPXXXHWDEP_FUNC_ATTR;
TPXXXHWDEP_STORAGE_CLASS void READ_REGISTER_BUFFER_ULONG(void *pReg, unsigned long *pBuf, unsigned long count) TPXXXHWDEP_FUNC_ATTR;
TPXXXHWDEP_STORAGE_CLASS void WRITE_REGISTER_ULONG(void *pReg, unsigned long value) TPXXXHWDEP_FUNC_ATTR;
TPXXXHWDEP_STORAGE_CLASS void WRITE_REGISTER_BUFFER_ULONG(void *pReg, unsigned long *pBuf, unsigned long count) TPXXXHWDEP_FUNC_ATTR;


/*
** I/O (Port) Access Functions
*/
TPXXXHWDEP_STORAGE_CLASS unsigned char READ_PORT_UCHAR(void *pReg) TPXXXHWDEP_FUNC_ATTR; 
TPXXXHWDEP_STORAGE_CLASS void READ_PORT_BUFFER_UCHAR(void *pReg, unsigned char *pBuf, unsigned long count) TPXXXHWDEP_FUNC_ATTR;
TPXXXHWDEP_STORAGE_CLASS void WRITE_PORT_UCHAR(void *pReg, unsigned char value) TPXXXHWDEP_FUNC_ATTR;
TPXXXHWDEP_STORAGE_CLASS void WRITE_PORT_BUFFER_UCHAR(void *pReg, unsigned char *pBuf, unsigned long count) TPXXXHWDEP_FUNC_ATTR;

TPXXXHWDEP_STORAGE_CLASS unsigned short READ_PORT_USHORT(void *pReg) TPXXXHWDEP_FUNC_ATTR;
TPXXXHWDEP_STORAGE_CLASS void READ_PORT_BUFFER_USHORT(void *pReg, unsigned short *pBuf, unsigned long count) TPXXXHWDEP_FUNC_ATTR;
TPXXXHWDEP_STORAGE_CLASS void WRITE_PORT_USHORT(void *pReg, unsigned short value) TPXXXHWDEP_FUNC_ATTR;
TPXXXHWDEP_STORAGE_CLASS void WRITE_PORT_BUFFER_USHORT(void *pReg, unsigned short *pBuf, unsigned long count) TPXXXHWDEP_FUNC_ATTR;

TPXXXHWDEP_STORAGE_CLASS unsigned long READ_PORT_ULONG(void *pReg) TPXXXHWDEP_FUNC_ATTR;
TPXXXHWDEP_STORAGE_CLASS void READ_PORT_BUFFER_ULONG(void *pReg, unsigned long *pBuf, unsigned long count) TPXXXHWDEP_FUNC_ATTR;
TPXXXHWDEP_STORAGE_CLASS void WRITE_PORT_ULONG(void *pReg, unsigned long value) TPXXXHWDEP_FUNC_ATTR;
TPXXXHWDEP_STORAGE_CLASS void WRITE_PORT_BUFFER_ULONG(void *pReg, unsigned long *pBuf, unsigned long count) TPXXXHWDEP_FUNC_ATTR;



#endif  /* __TPXXXHWDEP_H__ */

