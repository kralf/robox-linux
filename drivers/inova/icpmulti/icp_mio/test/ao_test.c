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
** Title:   Analog output test
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** DESCRIPTION
**-------------------------------------------------------------------------*/
/* This module contains the test code for the driver of the ICP-MULTI 
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
#define FALSE 0
#define TRUE  1
/*---------------------------------------------------------------------------
** TYPEDEF
**-------------------------------------------------------------------------*/
#define BOOL int
/*---------------------------------------------------------------------------
** DATA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** FORWARD DECLARATIONS
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** STATIC FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* ao_ch_test - analog out channel test
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static int ao_ch_test(int fd_ao_ch, BOOL bip, BOOL range, int units)
{
  static int values[25] = {0,      5000,   4800,  800,   1500, 
                           2500,   3500,   4500,  500,   7000, 
                           8000,   9000,   10000, -3000, 11000, 
                           -10000, -11000, -5000, -1500, -2500, 
                           -3500,  -4500,  -500,  -100,  100 };
  int  mystdin;
  int  i = 0;
  int  mult;
  int  div;
  int  ctr_val = 0;
  char ch;

  switch (units)
  {
    case DAC_100uV_UNIT:
      mult = 10;
      div  = 1;
      break;
    case DAC_mV_UNIT:
      mult = 1;
      div  = 1;
      break;
    case DAC_10mV_UNIT:
      mult = 1;
      div  = 10;
      break;
    case DAC_100mV_UNIT:
      mult = 1;
      div  = 100;
      break;
    case DAC_V_UNIT:
      mult = 1;
      div  = 1000;
      break;
    default:
      mult = 1;
      div  = 1;
      break;
  }

  if (ioctl(fd_ao_ch,MIO_IOCTL_AO_CH_DAC_UNITS, units) < 0)
  {
     printf ("ioctl error on ao_ch can't set units\n");
     return -1;
  }

  if (ioctl(fd_ao_ch,MIO_IOCTL_AO_CH_DAC_BIP_INP_RANGE, bip) < 0)
  {
     printf ("ioctl error on ao_ch can't set bipolar output range\n");
     return -1;
  }
  if (ioctl(fd_ao_ch,MIO_IOCTL_AO_CH_DAC_INP_RANGE, range) < 0)
  {
     printf ("ioctl error on ao_ch can't set range\n");
     return -1;
  }

  /*
  ** permanent value write
  */
  /* set stdin to non-blocking */
  mystdin = open("/dev/stdin", O_RDONLY, 0);
  if(mystdin < 0)
  {
    printf("stdin open error\n");
    return -1;
  }

  printf("Usage: to switch to the next output value just press ENTER\n");
  printf("       to switch to the next test step just press SPACE and then ENTER\n");
  printf("       to break down the test just press Z and then ENTER\n");

  do
  {
    ctr_val = values[(i++) % 25];
    ctr_val = (ctr_val*mult)/div;
    if ( write(fd_ao_ch,&ctr_val, 4) !=  4 )
    {
      printf ("write error on ao_ch\n");
      close(mystdin);
      return -1;
    }
    else 
    {
      printf ("ao_ch output should be %i\n", ctr_val);
    }
    /* block */
    read(mystdin /*stdin */, &ch, 1);
  } while ( (ch != ' ') && (ch != 'Z') );
  close(mystdin);

  if ( ch == 'Z' )
  {
    printf ("User break\n");
    return -1;
  }
  return 0;
}

/******************************************************************************
*
* ao_ch_glob_test
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
int ao_ch_glob_test(int fd_ao_ch, int units)
{
  char str[6][50] = { "100 uV Units",
                      "mV Units",
                      "10 mV Units",
                      "100 mV Units",
                      "1 V Units",
                      "Unknown units"
                    };
  int i = 0;

  switch (units)
  {
    case DAC_100uV_UNIT:
      i = 0;
      break;
    case DAC_mV_UNIT:
      i = 1;
      break;
    case DAC_10mV_UNIT:
      i = 2;
      break;
    case DAC_100mV_UNIT:
      i = 3;
      break;
    case DAC_V_UNIT:
      i = 4;
      break;
    default:
      i = 5;
      break;
  }

  /*
  ** 5V unipolar
  */
  printf("5V range unipolar test %s\n",str[i]);
  if (ao_ch_test(fd_ao_ch, FALSE, FALSE, units) != 0)
  {
    printf("Test FAILED\n");
    goto Error;
  }
  else
  {
    printf("Test PASSED\n");
  }

  /*
  ** 5V bipolar
  */
  printf("5V range bipolar test %s\n", str[i]);
  if (ao_ch_test(fd_ao_ch, TRUE, FALSE, units) != 0)
  {
    printf("Test FAILED\n");
    goto Error;
  }
  else
  {
    printf("Test PASSED\n");
  }

  /*
  ** 10V unipolar
  */
  printf("10V range unipolar test %s\n", str[i]);
  if (ao_ch_test(fd_ao_ch, FALSE, TRUE, units) != 0)
  {
    printf("Test FAILED\n");
    goto Error;
  }
  else
  {
    printf("Test PASSED\n");
  }

  /*
  ** 10V bipolar
  */
  printf("10V range bipolar test %s\n", str[i]);
  if (ao_ch_test(fd_ao_ch, TRUE, TRUE, units) != 0)
  {
    printf("Test FAILED\n");
    goto Error;
  }
  else
  {
    printf("Test PASSED\n");
  }
  return 0;

Error:
  return -1;
}
  
    

/*---------------------------------------------------------------------------
** FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* ao_test - analog out test
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
void ao_test(int card_nr)
{
  int  i,j,k;
  int  fd_ao[4];
  char devName[80];
  int  ctr_val;
  
  /*
  ** open the ai channels
  */
  printf("\n Open the ao channels on %i card\n", card_nr);

  for(i=0; i < 4; i++)
  {
    printf("Openening ao channel%i on card%i \n",i, card_nr);
    sprintf(devName, "/dev/mio%i.ao%i", card_nr, i);
    if ((fd_ao[i] = open(devName, 1, 0)) < 0)
    {
      printf("Error openening ao channel%i on card%i\n",i, card_nr);
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
    printf("AO open error\n");
    for(j=0; j < i; j++)
    {
      close(fd_ao[j]);
    }
  }

  /*
  ** write test
  */
  printf ("write test: ");
 
  for(k=0; k < 4; k++)
  {
    for(i=0; i < 15; i++)
    {
      for (j = 1; j < 5; j++)
      {
        ctr_val = 0xAA55AA55;
        if ( write(fd_ao[k],&ctr_val, j) != ((j > 4)? 4: j) )
        {
          printf ("write error on ao_ch%i on card%i\n",k, card_nr);
          goto Error;
        }
      }
    }
  }
  printf(" OK\n");

  /*
  ** general test
  */
  /*
  ** Testing the channels
  */
  for(k=0;k<4;k++)
  {
    printf("Testing channel %i on card %i\n",k, card_nr);
    
    /* Test with 100uV units */
    printf("Test with 100uV units\n");
    if (ao_ch_glob_test(fd_ao[k],DAC_100uV_UNIT) != 0)
    {
      printf("Test FAILED\n");
      goto Error;
    }
    else
    {
      printf("Test PASSED\n");
    }
    /* Test with mV units */
    printf("Test with mV units\n");
    if (ao_ch_glob_test(fd_ao[k],DAC_mV_UNIT) != 0)
    {
      printf("Test FAILED\n");
      goto Error;
    }
    else
    {
      printf("Test PASSED\n");
    }
    /* Test with 10mV units */
    printf("Test with 10mV units\n");
    if (ao_ch_glob_test(fd_ao[k],DAC_10mV_UNIT) != 0)
    {
      printf("Test FAILED\n");
      goto Error;
    }
    else
    {
      printf("Test PASSED\n");
    }
    /* Test with 100mV units */
    printf("Test with 100mV units\n");
    if (ao_ch_glob_test(fd_ao[k],DAC_100mV_UNIT) != 0)
    {
      printf("Test FAILED\n");
      goto Error;
    }
    else
    {
      printf("Test PASSED\n");
    }
    /* Test with V units */
    printf("Test with V units\n");
    if (ao_ch_glob_test(fd_ao[k],DAC_V_UNIT) != 0)
    {
      printf("Test FAILED\n");
      goto Error;
    }
    else
    {
      printf("Test PASSED\n");
    }
  }
    
Error:
  for(k=0; k < 4; k++)
  {
    close(fd_ao[k]);
  }
}


/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/