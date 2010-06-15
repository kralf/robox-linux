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
** Title:   Digital input Test
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
* di_test - di test
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
void di_test(int card_nr)
{
  int i,j;
  int fd_di;
  int ctr_val = 0;
  char devName[80];
  char ch;
  int  mystdin;

  
  /*
  ** open the ctr channels
  */
  printf("\n Open the di on %i card: ", card_nr);
  sprintf(devName, "/dev/mio%i.di", card_nr);
  if ((fd_di = open(devName, 0, 0)) < 0)
  {
    printf("Error openening di channel on card%i\n", card_nr);
    return;
  }
  else
  {
    printf(" OK\n");
  }

  /*
  ** read test
  */
  printf ("read test: ");
 
  for(i=0; i < 15; i++)
  {
    for (j = 1; j < 5; j++)
    {
      ctr_val = 0;
      if ( read(fd_di,&ctr_val, j) != ((j > 2)? 2: j) )
      {
        printf ("read error on di on card%i\n", card_nr);
        goto Error;
      }
    }
  }
  printf(" OK\n");


  /*
  ** permanent di val display
  */
  /* set stdin to non-blocking */
  mystdin = open("/dev/stdin", O_RDONLY | O_NONBLOCK, 0);
  if(mystdin < 0)
  {
    printf("stdin open error\n");
    goto Error;
  }

  do
  {
    ctr_val = 0;
    if ( read(fd_di,&ctr_val, 2) != 2 )
    {
      printf ("read error on di on card%i\n", card_nr);
      close(mystdin);
      goto Error;
    }
    else 
    {
      printf ("di on card%i: %i >>break with SPACE then ENTER\n", card_nr, ctr_val);
    }

    for(j=0; j < 1000000; j++)
    {
      ctr_val = 1;
    };
    read(mystdin /*stdin */, &ch, 1);
  } while (ch != ' ');
  close(mystdin);

  
  /*
  ** wakeup mode test
  */
  if (ioctl(fd_di,MIO_IOCTL_DI_READ_MODE, 1) < 0)
  {
    printf("ioctl set error on di\n");
    goto Error;
  }

  printf("please gen a status change irq\n");
  ctr_val = 0;
  if ( read(fd_di,&ctr_val, 2) !=  2 )
  {
    printf ("read error on di on card%i\n", card_nr); 
    goto Error;
  }
  printf("wakeup test for di OK, value:%i\n", ctr_val);

Error:
  close(fd_di);
}
/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/