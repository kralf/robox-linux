/*-------------------------------------------------------------
 * Linux on RoboX project - ASL EPFL
 *-------------------------------------------------------------
 *
 * tip866 linux driver for kernel 2.6
 *
 *-------------------------------------------------------------
 * $Id: tip866.h,v 1.2 2004/02/03 08:25:15 fred Exp $
 *-------------------------------------------------------------*/

#ifndef _TIP866_H
#define _TIP866_H

/* exported functions */

void tip866_init_channel(int channel);
int tip866_read_channel(int channel, unsigned char *buf);
int tip866_write_channel(int channel, const unsigned char *buf, int count);

#endif
