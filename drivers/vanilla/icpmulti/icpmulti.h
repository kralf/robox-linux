#ifndef ICPMULTI_H

/*-------------------------------------------------------------
 * Linux on RoboX project - ASL EPFL
 *-------------------------------------------------------------
 *
 * functions exported by icpmulti linux driver.
 *
 *-------------------------------------------------------------
 * $Id: icpmulti.h,v 1.3 2004/01/12 12:54:08 fred Exp $
 *-------------------------------------------------------------*/

int icpmulti_read_ai(int channel, int *value);
int icpmulti_write_ao(int channel, short value);
int icpmulti_read_ao(int channel, int *value);
int icpmulti_write_do(int channel, unsigned char value);
int icpmulti_read_di(int channel, int *value);

#endif
