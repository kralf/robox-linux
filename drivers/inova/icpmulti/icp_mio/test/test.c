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
** Title:   main Test file
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
** imported functions
**-------------------------------------------------------------------------*/
extern void ctr_test(int card_nr);
extern void di_test(int card_nr);
extern void do_test(int card_nr);
extern void ai_test(int card_nr);
extern void ao_test(int card_nr);


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
* main - 
*
*
* Parameters:
*         
*
* RETURNS: ERROR, in case of an error (Out of memory, No device found)
*          OK   , if everything was OK.
*   
* SEE ALSO: 
******************************************************************************/
int main()
{
  int  i;
  int  fd_mio;
  int  nr_of_mio_cards;
  char devName[80];


  /*
  ** Get the number of ICP-MULTI cards
  */
  /*
  ** open the first mio card
  */
  printf("\n Open the first mio card:");

  sprintf(devName, "/dev/mio0");
  if ((fd_mio = open(devName,0, 0)) < 0)
  {
    printf("Error opening mio0 card\n");
    return -1;
  }
  else
  {
    printf(" OK\n");
  }

  /*
  ** Get the number of ICP-MULTI cards
  */
  if ( (nr_of_mio_cards = ioctl(fd_mio, ICP_MIO_IOCTL_GET_NR, 0)) < 0)
  {
    printf("ioctl error on mio1\n");
    close(fd_mio);
  }
  else
  {
    printf("Number of MIO-CARDS in the system is: %i\n", nr_of_mio_cards);
  }
  

  /*
  ** Test the mio cards
  */
  for(i = 0; i < nr_of_mio_cards; i++)
  {
  
    printf("Testing card %i press Enter to start\n",i+1);
    scanf("%c",&devName[0]);
    
    /*
    ** 1. Counter test
    */
    ctr_test(i);

    /*
    ** 2. Digital Input test
    */
    di_test(i);
  
    /*
    ** 3. digital output test
    */
    do_test(i);

    /*
    ** 4. analog input test
    */
    ai_test(i);

    /*
    ** 5. analog output test
    */
    ao_test(i);
  }
  
  close(fd_mio);

  return 0;
}

/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/