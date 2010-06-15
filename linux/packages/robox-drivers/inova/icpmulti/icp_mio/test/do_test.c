/*---------------------------------------------------------------------------
** Inova Computers GmbH. Sudetenstr 5, 87600 Kaufbeuren
**
** (c) Copyright 1998-2000. All Rights Reserved
**
** Date: Oct.04.2000
**
** Operating System: Linux
**
** Compiler : Gnu C/C++ Compiler 
**
**---------------------------------------------------------------------------
** Project: ICP-MULTI Linux Driver
** Title:   Digital output Test
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** DESCRIPTION
**-------------------------------------------------------------------------*/
/* This module contains the test code of the driver of the ICP-MULTI 
** CompactPCI card */

 
/*---------------------------------------------------------------------------
** MODIFICATION HISTORY
**-------------------------------------------------------------------------*/
/*
modification history
-------------------- 
000,25oct00,bt   created
*/
 
/*---------------------------------------------------------------------------
** PRAGMA
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** INCLUDE
**-------------------------------------------------------------------------*/
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <fcntl.h>
#include "../h/icp_mio_hw.h"
/*---------------------------------------------------------------------------
** DEFINE
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** TYPEDEF
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** DATA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** FORWARD DECLARATIONS
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** STATIC FUNCTION
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* do_test - do test
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
void do_test(int card_nr)
{
  static int values[15] = {0xAA, 0x55, 0xA5, 0x5A, 0x01, 0x02,
                           0x0000, 0x1111, 0x2222, 0x3333, 
                           0x4444, 0x5555, 0x6666, 0x7777, 0x8888 };
  int i,j;
  int fd_do;
  int ctr_val = 0;
  char devName[80];
  char ch;
  int  mystdin;

  
  /*
  ** open the do channel
  */
  printf("\n Open the do channel on %i card: ", card_nr);

  sprintf(devName, "/dev/mio%i.do", card_nr);
  if ((fd_do = open(devName,1, 0)) < 0)
  {
    printf("Error openening do channel on card%i\n", card_nr);
    return;
  }
  else
  {
    printf(" OK\n");
  }


  /*
  **  Write test with different params
  */
  printf("\nWrite test ");
  ctr_val = 0xAA55;
  for(i=0; i < 15; i++)
  {
  
    for (j = 1; j < 5; j++)
    {
      ctr_val = 0xAA55;
      if ( write(fd_do,&ctr_val, j) != 1 )
      {
          printf ("write error on do on card%i\n", card_nr); 
          goto Error;
       }
    }
  }

  printf(" OK\n");



  /*
  ** permanent ctr val display
  */
  /* set stdin to non-blocking */
  mystdin = open("/dev/stdin", O_RDONLY , 0);
  if(mystdin < 0)
  {
    printf("stdin open error\n");
    goto Error;
  }

  i = 0;
  do
  {
    i++;
    i = i % 15;
    ctr_val = values[i];
    printf("write data: %x >> break down with SPACE then ENTER\n", ctr_val);
    if ( write(fd_do,&ctr_val, 1) != 1 )
    {
      printf ("write error on do on card%i\n", card_nr);
      close(mystdin);
      goto Error;
    }
    read(mystdin /*stdin */, &ch, 1);
  } while (ch != ' ');

  
  /*
  ** dout error mode test
  */
  if (ioctl(fd_do,MIO_IOCTL_DO_ERROR_OUT_VAL, 0x55) < 0)
  {
    printf("ioctl set error on do\n");
    close(mystdin);
    goto Error;
  }

  /*
  **  write wait no update 
  */
  if (ioctl(fd_do,MIO_IOCTL_DO_WRITE_MODE, 1) < 0)
  {
    printf("ioctl set error on do\n");
    close(mystdin);
    goto Error;
  }

  printf("write data 0x01 to do\n please gen an error irq\n");
  ctr_val = 0x01;
  if ( write(fd_do,&ctr_val, 1) !=  1 )
  {
    printf ("write error on do \n"); 
    close(mystdin);
    goto Error;
  }
  printf("wakeup test for do OK\n");
  printf("waiting press g + Enter\n");
  do
  {
    read(mystdin, &ch,1);
  } while (ch != 'g');

  close(mystdin);


  /*
  **  write wait qith update 
  */
  if (ioctl(fd_do,MIO_IOCTL_DO_WRITE_MODE, 0) < 0)
  {
    printf("ioctl set error on do\n");
    goto Error;
  }
  printf("write data 0x01 to do\n please gen an error irq\n");
  ctr_val = 0x02;
  if ( write(fd_do,&ctr_val, 1) !=  1 )
  {
    printf ("write error on do \n"); 
    goto Error;
  }
  printf("wakeup test for do OK\n");


  
Error:
  close(fd_do);

}

/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/