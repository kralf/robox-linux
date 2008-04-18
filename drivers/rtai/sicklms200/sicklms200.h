/*-------------------------------------------------------------
 * Linux on RoboX project - ASL EPFL
 *-------------------------------------------------------------
 *
 * sicklms200 interface
 *
 *-------------------------------------------------------------
 * $Id: sicklms200.h,v 1.1 2004/02/12 09:38:36 cvs Exp $
 *-------------------------------------------------------------*/

#ifndef _SICKLMS200_H
#define _SICKLMS200_H

/* exported functions */

int sicklms200_read_channel(int channel, unsigned char *buf);
int sicklms200_get_scanlen(int channel);

#endif
