/* $Id: tip866setspeed.c 59 2006-07-11 14:24:47Z Welzel $ */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @   S E T S P E E D   @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          TIP866 - Device Driver                                **
**                                                                           **
**    File             tip866setspeed.c                                      **
**                                                                           **
**    Description      Changes the baudrate of a specified channel           **
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

static const char *ttyname_prefix = "ttySTIP866_";

int main(int argc, char *argv[])

{
    int   tty1;
    char  DevName[30];
    int   minor, result;
    unsigned long baud;
    unsigned long code;
    struct termios termios;



    if (argc != 3) {
        printf("format : setspeed <minor> <baud>\n");
        printf("example: setspeed 7 9600\n");
        printf("         setup /dev/%s7 with 9600 baud \n\n", ttyname_prefix); 
        return -1;
    }
    else {
        minor = atoi(argv[1]);
        baud = atoi(argv[2]);
    }

    sprintf(DevName, "/dev/%s%0d", ttyname_prefix, minor);
    tty1 = open(DevName, O_RDWR | O_NOCTTY);
    if (tty1 < 0) {
        printf("Error open minor device %d\n", minor);
        return -1;
    }

    switch (baud) {
    case 50:
        code = B50;
        break;
    case 75:
        code = B75;
        break;
    case 110:
        code = B110;
        break;
    case 134:
        code = B134;
        break;
    case 150:
        code = B150;
        break;
    case 200:
        code = B200;
        break;
    case 300:
        code = B300;
        break;
    case 600:
        code = B600;
        break;
    case 1200:
        code = B1200;
        break;
    case 1800:
        code = B1800;
        break;
    case 2400:
        code = B2400;
        break;
    case 4800:
        code = B4800;
        break;
    case 9600:
        code = B9600;
        break;
    case 19200:
        code = B19200;
        break;
    case 38400:
        code = B38400;
        break;
    case 57600:
        code = B57600;
        break;
    case 115200:
        code = B115200;
        break;
    case 230400:
        code = B230400;
        break;
    case 460800:
        code = B460800;
        break;
    default:
        code = 0;
        break;

    }

    tcgetattr(tty1, &termios);

    result = cfsetospeed(&termios, code);

    if (result < 0) {
        printf("Unsupported baudrate  baud=%ld\n", baud);
        return -1;
    }
    
    result = cfsetispeed(&termios, code);

    if (result < 0) {
        printf("Unsupported baudrate  baud=%ld\n", baud);
        return -1;
    }
    
    tcsetattr(tty1, TCSANOW, &termios);

    printf("Set baudrate to %ld baud\n", baud);

    close(tty1);

    return 0;
}

