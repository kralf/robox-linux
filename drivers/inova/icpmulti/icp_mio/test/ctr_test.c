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
** Title:   Counter Test
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** DESCRIPTION
**-------------------------------------------------------------------------*/
/* This module contains the test code of the ICP-MULTI CompactPCI card */

 
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
* ctr_tes - counter test
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
void ctr_test(int card_nr)
{
  static int values[15] = {0x55AA, 0xAA55, 0xA5A5, 0x5A5A, 0x5555, 0xAAAA, 
                           0x0000, 0x1111, 0x2222, 0x3333, 
                           0x4444, 0x5555, 0x6666, 0x7777, 0x8888 };
  int i,j,k;
  int fd_ctr[4];
  int ctr_val = 0;
  char devName[80];
  char ch;
  int  mystdin;

  
  /*
  ** open the ctr channels
  */
  printf("\n Open the counters on %i card\n", card_nr);

  for(i=0; i < 4; i++)
  {
    printf("Openening ctr channel%i on card%i \n",i, card_nr);
    sprintf(devName, "/dev/mio%i.ctr%i", card_nr, i);
    if ((fd_ctr[i] = open(devName, 2, 0)) < 0)
    {
      printf("Error openening ctr channel%i on card%i\n",i, card_nr);
      break;
    }
    else
    {
      printf(" OK\n");
    }

  }
  if (i != 4)
  {
    /*
    ** There was an error
    */
    printf("CTR open error\n");
    for(j=0; j < i; j++)
    {
      close(fd_ctr[j]);
    }
  }

  /*
  **  Write test with different params
  */
  printf("\nWrite test ");
  ctr_val = 0xAA55;
  for(k=0; k < 4; k++)
  {
    for(i=0; i < 15; i++)
    {
      for (j = 1; j < 5; j++)
      {
        ctr_val = 0xAA55;
        if ( write(fd_ctr[k],&ctr_val, j) != ((j > 2)? 2: j) )
        {
          printf ("write error on ctr%i on card%i\n",k, card_nr); 
          goto Error;
        }
      }
    }
  }

  printf(" OK\n");

  /*
  ** read test
  */
  printf ("read test: ");
 
  for(k=0; k < 4; k++)
  {
    for(i=0; i < 15; i++)
    {
      for (j = 1; j < 5; j++)
      {
        ctr_val = 0;
        if ( read(fd_ctr[k],&ctr_val, j) != ((j > 2)? 2: j) )
        {
          printf ("read error on ctr%i on card%i\n",k, card_nr);
          goto Error;
        }
        else if (ctr_val != ((j >= 2)? 0xAA55: 0x55))
        {
          printf ("bad value read on ctr%i on card%i, expected:%x, got:%x\n",k, card_nr, ((j > 2)? 0xAA55: 0x55), ctr_val);
          goto Error;
        }
      }
    }
  }
  printf(" OK\n");


  /*
  ** read-write test
  */
  printf ("read-write test ");
 
  for(k=0; k < 4; k++)
  {
    for(i=0; i < 15; i++)
    {
      for (j = 1; j < 5; j++)
      {
        ctr_val = values[i];
        /*
        ** first write it
        */
        if ( write(fd_ctr[k],&ctr_val, j) != ((j > 2)? 2: j) )
        {
          printf ("write error on ctr%i on card%i\n",k, card_nr); 
          goto Error;
        }
        ctr_val = 0;
        if ( read(fd_ctr[k],&ctr_val, j) != ((j > 2)? 2: j) )
        {
          printf ("read error on ctr%i on card%i\n",k, card_nr);
          goto Error;
        }
        else if (ctr_val != ((j >= 2)? (values[i] & 0xFFFF): (values[i] & 0xFF)) )
        {
          printf ("bad value read on ctr%i on card%i\n",k, card_nr);
          goto Error;
        }
      }
    }
  }
  printf(" OK\n");

  /*
  ** permanent ctr val display
  */
  /* set stdin to non-blocking */
  mystdin = open("/dev/stdin", O_RDONLY | O_NONBLOCK, 0);
  if(mystdin < 0)
  {
    printf("stdin open error\n");
    goto Error;
  }

  /* clear display */
  do
  {
    for(k=0; k < 4; k++)
    {
      ctr_val = 0;
      if ( read(fd_ctr[k],&ctr_val, 2) != 2 )
      {
        printf ("read error on ctr%i on card%i\n",k, card_nr);
        close(mystdin);
        goto Error;
      }
      else 
      {
        printf ("ctr%i on card%i: %i >>break down SPACE then ENTER\n",k, card_nr, ctr_val);
      }
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
  for(k=0; k<4; k++)
  {
    if (ioctl(fd_ctr[k],MIO_IOCTL_CTR_CH_MODE, 1) < 0)
    {
      printf("ioctl set error on ctr%i\n", k);
      goto Error;
    }
    else
    {
      printf("write data 0xFFFD to ctr%i\n please gen an overflow irq\n",k);
      ctr_val = 0xFFFD;
      if ( write(fd_ctr[k],&ctr_val, 2) !=  2 )
      {
        printf ("write error on ctr%i on card%i\n",k, card_nr); 
        goto Error;
      }
      printf("wakeup test for ctr%i OK\n",k);
    }
  }
  
Error:
  close(fd_ctr[0]);
  close(fd_ctr[1]);
  close(fd_ctr[2]);
  close(fd_ctr[3]);

}

/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/