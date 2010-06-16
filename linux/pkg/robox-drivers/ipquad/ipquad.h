/*-------------------------------------------------------------
 * Linux on RoboX project - ASL ETHZ
 *-------------------------------------------------------------
 *
 * SBS Technologies IP Quadrature linux driver for kernel 2.6
 *
 *-------------------------------------------------------------
 * $Id: ipquad.h,v 1.2 2008/04/27 08:25:15 kralf Exp $
 *-------------------------------------------------------------*/

#ifndef IPQUAD_H
#define IPQUAD_H

int ipquad_read_channel(int channel, int* value);
int ipquad_write_channel(int channel, int value);

int ipquad_invert_channel(int channel, int invert);

#endif
