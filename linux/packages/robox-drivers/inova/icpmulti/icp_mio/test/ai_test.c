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
** Title:   analog input Test
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
#define BOOL int
#define FALSE 0
#define TRUE  1
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
/******************************************************************************
*
* ai_ch_test - analog in channel test
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static int ai_ch_test(int fd_ai_ch, BOOL bip, BOOL range, int units)
{
  int  mystdin;
  int  ctr_val = 0;
  char ch;

  if (ioctl(fd_ai_ch,MIO_IOCTL_AI_CH_ADC_UNITS, units) < 0)
  {
     printf ("ioctl error on ai_ch can't set units\n");
     return -1;
  }

  if (ioctl(fd_ai_ch,MIO_IOCTL_AI_CH_ADC_BIP_INP_RANGE, bip) < 0)
  {
     printf ("ioctl error on ai_ch can't set bipolar output range\n");
     return -1;
  }
  if (ioctl(fd_ai_ch,MIO_IOCTL_AI_CH_ADC_INP_RANGE, range) < 0)
  {
     printf ("ioctl error on ai_ch can't set range\n");
     return -1;
  }

  /*
  ** permanent read 
  */
  /* set stdin to non-blocking */
  mystdin = open("/dev/stdin", O_RDONLY, 0);
  if(mystdin < 0)
  {
    printf("stdin open error\n");
    return -1;
  }

  printf("Usage: to switch to the next input value just press ENTER\n");
  printf("       to switch to the next test step just press SPACE and then ENTER\n");
  printf("       to break down the test just press Z and then ENTER\n");

  do
  {
    if ( read(fd_ai_ch,&ctr_val, 4) !=  4 )
    {
      printf ("read error on ai_ch\n");
      close(mystdin);
      return -1;
    }
    else 
    {
      printf ("ai_ch input value %i\n", ctr_val);
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
* ai_ch_glob_test
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
int ai_ch_glob_test(int fd_ai_ch, int units)
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
    case ADC_100uV_UNIT:
      i = 0;
      break;
    case ADC_mV_UNIT:
      i = 1;
      break;
    case ADC_10mV_UNIT:
      i = 2;
      break;
    case ADC_100mV_UNIT:
      i = 3;
      break;
    case ADC_V_UNIT:
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
  if (ai_ch_test(fd_ai_ch, FALSE, FALSE, units) != 0)
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
  if (ai_ch_test(fd_ai_ch, TRUE, FALSE, units) != 0)
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
  if (ai_ch_test(fd_ai_ch, FALSE, TRUE, units) != 0)
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
  if (ai_ch_test(fd_ai_ch, TRUE, TRUE, units) != 0)
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
* ai_test - ai test
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
void ai_test(int card_nr)
{
  int i,j,k;
  int fd_ai[16];
  int ctr_val = 0;
  char devName[80];

  
  /*
  ** open the ai channels
  */
  printf("\n Open the counters on %i card\n", card_nr);

  for(i=0; i < 16; i++)
  {
    printf("Openening ai channel%i on card%i \n",i, card_nr);
    sprintf(devName, "/dev/mio%i.ai%i", card_nr, i);
    if ((fd_ai[i] = open(devName, 0, 0)) < 0)
    {
      printf("Error openening ai channel%i on card%i\n",i, card_nr);
      break;
    }
    else
    {
      printf(" OK\n");
    }

  }
  if (i != 16)
  {
    /*
    ** There was an error
    */
    printf("AI open error\n");
    for(j=0; j < i; j++)
    {
      close(fd_ai[j]);
    }
  }

  /*
  ** read test
  */
  printf ("read test: ");
 
  for(k=0; k < 16; k++)
  {
    for(i=0; i < 15; i++)
    {
      for (j = 1; j < 5; j++)
      {
        ctr_val = 0;
        if ( read(fd_ai[k],&ctr_val, j) != ((j > 4)? 4: j) )
        {
          printf ("read error on ai_ch%i on card%i\n",k, card_nr);
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
  ** Non-Differential input mode test
  */
  for(k=0; k < 16; k++)
  {
    if (ioctl(fd_ai[k],MIO_IOCTL_AI_CH_ADC_DIFF_INP_MODE, 0) < 0)
    {
       printf ("ioctl error on ai_ch%i on card%i\n",k, card_nr);
       goto Error;
    }
  }
  /*
  ** Testing the channels
  */
  for(k=0;k<16;k+=15)
  {
    printf("Testing channel %i on card %i\n",k, card_nr);
    
    /* Test with 100uV units */
    printf("Test with 100uV units\n");
    if (ai_ch_glob_test(fd_ai[k],ADC_100uV_UNIT) != 0)
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
    if (ai_ch_glob_test(fd_ai[k],ADC_mV_UNIT) != 0)
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
    if (ai_ch_glob_test(fd_ai[k],ADC_10mV_UNIT) != 0)
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
    if (ai_ch_glob_test(fd_ai[k],ADC_100mV_UNIT) != 0)
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
    if (ai_ch_glob_test(fd_ai[k],ADC_V_UNIT) != 0)
    {
      printf("Test FAILED\n");
      goto Error;
    }
    else
    {
      printf("Test PASSED\n");
    }
  }
  
  /* 
  ** Differential input mode test
  */
  for(k=0; k < 8; k++)
  {
    if (ioctl(fd_ai[k],MIO_IOCTL_AI_CH_ADC_DIFF_INP_MODE, 1) < 0)
    {
       printf ("ioctl error on ai_ch%i on card%i\n",k, card_nr);
       goto Error;
    }
  }
  /*
  ** Testing the channels
  */
  for(k=0;k<8;k++)
  {
    printf("Testing channel %i on card %i\n",k, card_nr);
    
    /* Test with 100uV units */
    printf("Test with 100uV units\n");
    if (ai_ch_glob_test(fd_ai[k],ADC_100uV_UNIT) != 0)
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
    if (ai_ch_glob_test(fd_ai[k],ADC_mV_UNIT) != 0)
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
    if (ai_ch_glob_test(fd_ai[k],ADC_10mV_UNIT) != 0)
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
    if (ai_ch_glob_test(fd_ai[k],ADC_100mV_UNIT) != 0)
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
    if (ai_ch_glob_test(fd_ai[k],ADC_V_UNIT) != 0)
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
  for(k=0; k < 16; k++)
  {
    close(fd_ai[k]);
  }
}
/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/