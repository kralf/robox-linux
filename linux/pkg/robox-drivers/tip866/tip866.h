/*-------------------------------------------------------------
 * Linux on RoboX project - ASL ETHZ
 *-------------------------------------------------------------
 *
 * TEWS TIP866-20 serial interface linux driver for kernel 2.6
 *
 *-------------------------------------------------------------
 * $Id: tip866.h,v 1.2 2004/02/03 08:25:15 fred Exp $
 *-------------------------------------------------------------*/

#ifndef TIP866_H
#define TIP866_H

void tip866_init_channel(int channel);
int tip866_read_channel(int channel, unsigned char *buf);
int tip866_write_channel(int channel, const unsigned char *buf, int count);

#endif
