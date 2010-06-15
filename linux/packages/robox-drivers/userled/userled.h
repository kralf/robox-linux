#ifndef USERLED_H

/*-------------------------------------------------------------
 * Linux on RoboX project - ASL EPFL
 *-------------------------------------------------------------
 *
 * functions exported by icpmulti linux driver.
 *
 *-------------------------------------------------------------
 * $Id: icpmulti.h,v 1.3 2004/01/12 12:54:08 fred Exp $
 *-------------------------------------------------------------*/

// User LED states
#define USERLED_OFF            0x00
#define USERLED_GREEN          0x01
#define USERLED_RED            0x02

unsigned char userled_read(void);
int userled_write(unsigned char value);

#endif
