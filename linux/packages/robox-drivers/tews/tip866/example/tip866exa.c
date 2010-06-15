/* $Id: tip866exa.c 59 2006-07-11 14:24:47Z Welzel $ */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @    T E S T 8 6 6    @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          TIP866 - Device Driver                                **
**                                                                           **
**    File             tip866exa.c                                           **
**                                                                           **
**    Description      Simple test program for TIP866 device driver          **
**                                                                           **
**    Owner            TEWS TECHNOLOGIES GmbH                                **
**                     Am Bahnhof 7                                          **
**                     D-25469 Halstenbek                                    **
**                     Germany                                               **
**                                                                           **
**                     Tel.: +49 / (0)4101 / 4058-0                          **
**                     Fax.: +49 / (0)4101 / 4058-19                         **
**                     EMail: Support@tews.com                               **
**                     Web: http://www.tews.com                              **
**                                                                           **
**                                                                           **
**                     Copyright (c) 2006                                    **
**                     TEWS TECHNOLOGIES GmbH                                **
**                                                                           **
**                                                                           **
**                                                                           **
**    System           Linux                                                 **
**                                                                           **
**    $Date: 2006-07-11 16:24:47 +0200 (Di, 11 Jul 2006) $   $Rev: 59 $      **
**                                                                           **
*******************************************************************************
*******************************************************************************/

#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/* our test uses the current baudrate */
#define BLOCK_LEN   200

static const char *ttyname_prefix = "ttySTIP866_";

int main(int argc, char *argv[])

{
    int   tty1, tty2;
    long  i, ii, num;
    long  loops;
    unsigned char  buffer[BLOCK_LEN];
    char  DevName[30];
    int   minor1, minor2;
    unsigned long NumberOfBytes;
    struct termios oldtty1, oldtty2, newtermios;



    if (argc != 4) {
        printf("format : tip866 <minor1> <minor2> <#bytes>\n");
        printf("example: tip866 0 7 10000\n");
        printf("         transfers 10000 byte between /dev/%s0 and /dev/%s7\n\n", ttyname_prefix, ttyname_prefix); 
        return -1;
    }
    else {
        minor1 = atoi(argv[1]);
        minor2 = atoi(argv[2]);
        NumberOfBytes = atol(argv[3]);
    }


    sprintf(DevName, "/dev/%s%0d", ttyname_prefix, minor1);
    tty1 = open(DevName, O_RDWR | O_NOCTTY);
    if (tty1 < 0) {
        printf("Error open minor device %d\n", minor1);
        return -1;
    }

    sprintf(DevName, "/dev/%s%0d", ttyname_prefix, minor2);
    tty2 = open(DevName, O_RDWR | O_NOCTTY);
    if (tty2 < 0) {
        printf("Error open minor device %d\n", minor2);
        return -1;
    }



    /*
     *  save current modem settings 
     */
    tcgetattr(tty1, &oldtty1);
    tcgetattr(tty2, &oldtty2);


    newtermios = oldtty1;
    /* 
     *  Set bps rate and hardware flow control and 8n1 (8bit,no parity,1 stopbit).
     *  Also don't hangup automatically and ignore modem status.
     *  Finally enable receiving characters.
     */
    newtermios.c_cflag = (oldtty1.c_cflag & CBAUD) | CS8 | CLOCAL | CREAD;
 
    /*
     *  Ignore bytes with parity errors and make terminal raw and dumb.
     */
    newtermios.c_iflag = IGNPAR;
 
    /*
     *  Raw output.
     */
    newtermios.c_oflag = 0;
 
    /*
     *  Don't echo characters and don't generate signals.
     */
    newtermios.c_lflag = 0;


    newtermios.c_cc[VMIN] = BLOCK_LEN;
    newtermios.c_cc[VTIME] = 0;


    /* 
     *  now clean the modem line and activate the settings for modem 
     */
    tcflush(tty1, TCIFLUSH);
    tcsetattr(tty1, TCSANOW, &newtermios);

    tcflush(tty2, TCIFLUSH);
    tcsetattr(tty2, TCSANOW, &newtermios);


    /*
     *  Transfer bytes between devices
     */

    for (i=0; i<BLOCK_LEN; i++) buffer[i] = 32+i;
    
    loops = NumberOfBytes / BLOCK_LEN;

    printf("Transfer %ld bytes in portion of %d bytes from minor %d to minor %d\n\n",
        loops*BLOCK_LEN, BLOCK_LEN, minor1, minor2);


    for (i=0; i < loops; i++) {

        if ((num = write(tty1, buffer, BLOCK_LEN)) != BLOCK_LEN) {
            printf("wrong number of bytes written (%ld instead of %d)\n", num, BLOCK_LEN);
            break;
        }

        for (ii=0; ii<BLOCK_LEN; ii++) buffer[ii] = 0;

        if ((num = read(tty2, buffer, BLOCK_LEN)) != BLOCK_LEN) {
            printf("wrong number of bytes read  (%ld instead of %d)\n", num, BLOCK_LEN);
            break;
        }
    
        for (ii=0; ii<BLOCK_LEN; ii++) {
            if (buffer[ii] != (ii+32)) {
                printf("[%ld] bad data received %d instead of %ld\n", ii, buffer[ii], ii+32);
                break;
            }
        }

        if (((i*BLOCK_LEN) % 1000) == 0) {
            printf("%ld bytes sent\r", (long)(i * BLOCK_LEN));
            fflush(stdout);
        }
    }


    printf("%ld bytes sent\n", (long)(i * BLOCK_LEN));

    /*
     *  restore old terminal settings and close the devices
     */
    tcsetattr(tty1, TCSANOW, &oldtty1);
    tcsetattr(tty2, TCSANOW, &oldtty2);

    close(tty1);
    close(tty2);

    return 0;
}

