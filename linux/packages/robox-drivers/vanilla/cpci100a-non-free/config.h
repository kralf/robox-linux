/* $Id: config.h 36 2007-05-07 15:59:24Z Welzel $ */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @     c o n f i g     @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          Linux Device Driver                                   **
**                                                                           **
**    File             config.h                                              **
**                                                                           **
**    Function         Kernel independent config header                      **
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
**                     Copyright (c) 2007                                    **
**                     TEWS TECHNOLOGIES GmbH                                **
**                                                                           **
**                                                                           **
**    $Date: 2007-05-07 17:59:24 +0200 (Mo, 07 Mai 2007) $   $Rev: 36 $      **
**                                                                           **
*******************************************************************************
*******************************************************************************/
#ifndef __TEWS_CONFIG_H__
#define __TEWS_CONFIG_H__

#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
#include <linux/config.h>
#endif

#endif /* __TEWS_CONFIG_H__ */
