/* $Header: /Tews/Device Driver/Linux/TDRV001/Code/Example/example.c 5     18.02.05 10:02 Hesse $ */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @   T D R V 0 0 1     @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          Linux - TDRV001 Driver                                **
**                                                                           **
**    File             Example.c                                             **
**                                                                           **
**    Function         test program for TDRV001 driver                       **
**                                                                           **
**                                                                           **
**    Owner            TEWS TECHNOLOGIES                                     **
**                     Am Bahnhof 7                                          **
**                     D-25469 Halstenbek                                    **
**                     Germany                                               **
**                                                                           **
**                                                                           **
**                     Tel.: +49(0)4101/4058-0                               **
**                     Fax.: +49(0)4101/4058-19                              **
**                     e-mail: support@tews.com                              **
**                                                                           **
**                                                                           **
**                     Copyright (c) 2004                                    **
**                     TEWS TECHNOLOGIES GmbH                                **
**                                                                           **
**                                                                           **
**                                                                           **
**                                                                           **
**                                                                           **
*******************************************************************************
*******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <string.h>

#include "../tdrv001.h"
#include "../pef20534.h"

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE !FALSE
#endif



#define NUM_TX_BUFFERS          50
#define RX_BUFFER_SIZE          250
#define TX_BUFFER_SIZE          200

#define MAX_BOARDS              4
#define NUM_CHANNELS            4
#define DEV_BASE_NAME           "/dev/tdrv001"

char* commtypeToString( COMM_TYPE mode );
char* clockmodeToString( CLOCK_MODE mode );
char* dcedteToString( DCEDTE mode );
char* transceiverToString( TRANSCEIVER_MODE mode );
static void PrintErrorMessage(void);


int act_channel = 0;
int act_board   = 0;



int convertStringToInt( char* text )
{
    int intval = 0;
    if (sscanf(text, "0x%x", &intval) > 0)
    {
        /*printf("hex\n");*/
        return strtoul(text, NULL, 16);
    } else {
        return atoi( text );
    }
    return intval;
}



int main(int argc, char *argv[])

{
    char devname[20];
    int devHandle[MAX_BOARDS][NUM_CHANNELS];
    int hCurrent;
    char select[50];
    char buffer[200];
    int selnum, result, error, count, i, channel, nbytes, idx, remaining;
    int found_modules, found_channels;
    unsigned long ulValue;
    int rw_blocking = FALSE;
    TP862_OPERATION_MODE_STRUCT OperationMode;
    TP862_REGISTER_STRUCT RegStruct;
    OSC_SOURCE currentOsc = OSCSOURCE_XTAL1;
    OSC_SOURCE tmpOsc;
    char databuffer[TX_BUFFER_SIZE];
    char* pBuffer;



    found_modules  = 0;
    found_channels = 0;

    for (i=0; i<MAX_BOARDS; i++)
    {
        for (channel=0; channel<NUM_CHANNELS; channel++)
        {
            sprintf(devname, "%s_%d_%d", DEV_BASE_NAME, i, channel);
            devHandle[i][channel] = open(devname, O_RDWR);
            if (devHandle[i][channel] < 0) {
             //   printf("Error open device '%s'\n", devname);
                devHandle[i][channel] = 0;
            } else {
                printf("Opened device '%s'\n", devname);
                found_channels++;
                found_modules=i+1;
            }
        }
    }

    printf("TDRV001-Modules found: %d\n", found_modules);
    printf("   channels available: %d\n", found_channels);

    if (found_channels == 0)
    {
        exit(1);
    }

    hCurrent = devHandle[0][0];


    do {

        do {
            printf("\nPlease select function for device %s_%d_%d\n\n", DEV_BASE_NAME, act_board, act_channel);
            printf("    1  --  Set Baudrate\n");
            printf("    2  --  Perform DSCC4 reset\n");
            printf("    5  --  Get Transmit-Error-Count\n");
            printf("    6  --  Get Transmit-OK-Count\n");
            printf("    7  --  Set Read Timeout\n");
            printf("   10  --  Debug - Show Registers\n");
            printf("   13  --  Set externally supplied baudrate\n");
            printf("   14  --  Change Global OscillatorIn (Osc=%s)\n", ((currentOsc == OSCSOURCE_XTAL1) ? "XTAL1" : "XTAL2"));
            printf("   15  --  Set Operation Mode\n");
            printf("   16  --  Get Operation Mode\n");
            printf("   20  --  Disable Receiver\n");
            printf("   21  --  Enable Receiver\n");
            printf("   30  --  Send some text\n");
            printf("   31  --  Send some text multiple times\n");
            printf("   35  --  Send a large packet (5000 bytes)\n");
            printf("   51  --  Read Frames\n");
            printf("   52  --  clear RX buffer\n");
            printf("   60  --  read CPLD register\n");
            printf("   61  --  write CPLD register\n");
            printf("   62  --  read SCC register\n");
            printf("   63  --  write SCC register\n");
            printf("   80  --  Toggle Blocking-Read/Write (%s)\n", (rw_blocking) ? "enabled" : "disabled");
            printf("   88  --  Change Channel\n");
            printf("   89  --  Change Board\n");
            printf("    0  --  Quit\n");

            printf("select ");
            fgets( select, 50, stdin );
            selnum = atoi(select);

        } while (selnum < 1 && selnum > 100);


        switch (selnum)
        {

        case 1:
            //
            // set baudrate
            //
            printf("baudrate : ");
            fgets( buffer, 50, stdin );
            ulValue = atoi( buffer );
            printf("calling IOCTL_TP862_SET_BAUDRATE with baudrate=%ld...\n", ulValue);
            result = ioctl(hCurrent, TP862_IOCT_SET_BAUDRATE, ulValue);
            if (result < 0)
            {
                // error setting baudrate
                printf("Error setting baudrate! %d\n", errno);
                PrintErrorMessage();
            }
            break;

        case 2:
            result = ioctl(hCurrent, TP862_IOC_DSCC4_RESET, 1);
            break;
        case 5:
            ulValue = 42;
            result = ioctl(hCurrent, TP862_IOCQ_GET_TX_COUNT_ERROR, &ulValue);
            printf("Transmit-Error count = %ld\n", ulValue);
            break;
        case 6:
            ulValue = 42;
            result = ioctl(hCurrent, TP862_IOCQ_GET_TX_COUNT_OK, &ulValue);
            printf("Transmit-OK count = %ld\n", ulValue);
            break;

        case 7:
            printf("New Read Timeout: ");
            fgets( select, 50, stdin );
            ulValue = atoi( select );
            result = ioctl(hCurrent, TP862_IOCT_SET_READ_TIMEOUT, ulValue);
            break;

        case 10:
            printf("Debug Register contents:\n");
            RegStruct.offset = CCR0;
            result = ioctl(hCurrent, TP862_IOCG_SCC_REG_READ, &RegStruct);
            if (result >= 0)
            {
                printf( "SCC CCR0 = 0x%.8lX\n", RegStruct.value );
            } else {
                printf( "SCC CCR0 = ERROR! %d\n", errno );
                PrintErrorMessage();
            }


            RegStruct.offset = CCR1;
            result = ioctl(hCurrent, TP862_IOCG_SCC_REG_READ, &RegStruct);
            if (result >= 0)
            {
                printf( "SCC CCR1 = 0x%.8lX\n", RegStruct.value );
            } else {
                printf( "SCC CCR1 = ERROR! %d\n", errno );
                PrintErrorMessage();
            }

            RegStruct.offset = STAR;
            result = ioctl(hCurrent, TP862_IOCG_SCC_REG_READ, &RegStruct);
            if (result >= 0)
            {
                printf( "SCC STAR = 0x%.8lX\n", RegStruct.value );
            } else {
                printf( "SCC STAR = ERROR! %d\n", errno );
                PrintErrorMessage();
            }


            RegStruct.offset = IQPBAR;
            result = ioctl(hCurrent, TP862_IOCG_SCC_REG_READ, &RegStruct);
            if (result >= 0)
            {
                printf( "GLOB IQPBAR = 0x%.8lX\n", RegStruct.value );
            } else {
                printf( "GLOB IQPBAR = ERROR! %d\n", errno );
                PrintErrorMessage();
            }

            RegStruct.offset = TP862_CPLD_ID_REG;
            result = ioctl(hCurrent, TP862_IOCG_CPLD_REG_READ, &RegStruct);
            if (result >= 0)
            {
                printf( "CPLD ID = 0x%.2lX\n", RegStruct.value );
            } else {
                printf( "CPLD ID = ERROR! %d\n", errno );
                PrintErrorMessage();
            }

            RegStruct.offset = TP862_CPLD_MODE_REG0;
            result = ioctl(hCurrent, TP862_IOCG_CPLD_REG_READ, &RegStruct);
            if (result >= 0)
            {
                printf( "CPLD MODE0 = 0x%.2lX\n", RegStruct.value );
            } else {
                printf( "CPLD MODE0 = ERROR! %d\n", errno );
                PrintErrorMessage();
            }

            RegStruct.offset = TP862_CPLD_MODE_REG1;
            result = ioctl(hCurrent, TP862_IOCG_CPLD_REG_READ, &RegStruct);
            if (result >= 0)
            {
                printf( "CPLD MODE1 = 0x%.2lX\n", RegStruct.value );
            } else {
                printf( "CPLD MODE1 = ERROR! %d\n", errno );
                PrintErrorMessage();
            }

            RegStruct.offset = TP862_CPLD_TX_CLK_SEL_REG;
            result = ioctl(hCurrent, TP862_IOCG_CPLD_REG_READ, &RegStruct);
            if (result >= 0)
            {
                printf( "CPLD CLKSEL = 0x%.2lX\n", RegStruct.value );
            } else {
                printf( "CPLD CLKSEL = ERROR! %d\n", errno );
                PrintErrorMessage();
            }

            RegStruct.offset = TP862_CPLD_BUILDOPTION_REG;
            result = ioctl(hCurrent, TP862_IOCG_CPLD_REG_READ, &RegStruct);
            if (result >= 0)
            {
                printf( "CPLD BOPT = 0x%.2lX\n", RegStruct.value );
            } else {
                printf( "CPLD BOPT = ERROR! %d\n", errno );
                PrintErrorMessage();
            }


            break;

        case 13:
            /*
            ** set external baudrate
            */
            printf("Externally supplied baudrate = ");
            fgets( select, 50, stdin );
            ulValue = atoi( select );
            result = ioctl(hCurrent, TP862_IOCT_SET_EXT_XTAL, ulValue);
            if (result >= 0)
            {
                printf( "CPLD BOPT = 0x%.2lX\n", RegStruct.value );
            } else {
                printf( "CPLD BOPT = ERROR! %d\n", errno );
                PrintErrorMessage();
            }
            break;

        case 14:
            /*
            ** change global oscillator input
            */
            switch (currentOsc)
            {
            case OSCSOURCE_XTAL1:
                tmpOsc = OSCSOURCE_XTAL2;
                break;
            case OSCSOURCE_XTAL2:
                tmpOsc = OSCSOURCE_XTAL1;
                break;
            default:
                /* this should never happen */
                tmpOsc = OSCSOURCE_XTAL1;
                break;
            }
            result = ioctl(hCurrent, TP862_IOCT_SET_OSC_SRC, tmpOsc);
            /*
            **  Check the result of the last device I/O control operation
            */
            if( result >= 0 ) {
                currentOsc = tmpOsc;
            }
            else {
                printf("Operation failed (0x%X)\n", result);
            }
            break;

        case 15:
            //
            // set operation mode 
            //
            error = FALSE;

            do {
                printf("Communication type:\n");
                printf("  1 - HDLC\n");
                printf("  2 - ASYNC\n");
                printf("  0 - return\n\n");
                printf("select: ");
                fgets( buffer, 50, stdin );
                selnum = atoi(buffer);
            } while ( (selnum != 0) && (selnum!=1) && (selnum!=2) );
            
            switch (selnum)
            {
            case 1:
                OperationMode.commtype  = COMMTYPE_HDLC;
                break;
            case 2:
                OperationMode.commtype  = COMMTYPE_ASYNC;
                break;
            default:
                error = TRUE;
                selnum = 99;
                break;
            }

            if (error) break;

            do {
                printf("Clockmode:\n");
                printf("  1 - CLOCKMODE_CM0A\n");
                printf("  2 - CLOCKMODE_CM0B (Sync HDLC)\n");
                printf("  3 - CLOCKMODE_CM3B\n");
                printf("  4 - CLOCKMODE_CM4\n");
                printf("  5 - CLOCKMODE_CM7B\n");
                printf("  6 - CLOCKMODE_CM8\n");
                printf("  7 - CLOCKMODE_CM8A\n");
                printf("  8 - CLOCKMODE_CM8B\n");
                printf("  9 - CLOCKMODE_CM9\n");
                printf("  0 - return\n\n");
                printf("select: ");
                fgets( buffer, 50, stdin );
                selnum = atoi(buffer);
            } while ( (selnum != 0) && (selnum < 1) && (selnum > 9) );
            
            switch (selnum)
            {
            case 1:
                OperationMode.clockmode  = CLOCKMODE_CM0A;
                break;
            case 2:
                OperationMode.clockmode  = CLOCKMODE_CM0B;
                break;
            case 3:
                OperationMode.clockmode  = CLOCKMODE_CM3B;
                break;
            case 4:
                OperationMode.clockmode  = CLOCKMODE_CM4;
                break;
            case 5:
                OperationMode.clockmode  = CLOCKMODE_CM7B;
                break;
            case 6:
                OperationMode.clockmode  = CLOCKMODE_CM8;
                break;
            case 7:
                OperationMode.clockmode  = CLOCKMODE_CM8A;
                break;
            case 8:
                OperationMode.clockmode  = CLOCKMODE_CM8B;
                break;
            case 9:
                OperationMode.clockmode  = CLOCKMODE_CM9;
                break;
            default:
                error = TRUE;
                selnum = 99;
                break;
            }
	    
	    if (error) break;
			
	    
	    do {
	      printf("Tranceiver:\n");
	      printf("  1  -     TRANSCEIVER_NO_CHANGES\n");
	      printf("  2  -     TRANSCEIVER_NOT_USED\n");
	      printf("  3  -     TRANSCEIVER_RS530A\n");
	      printf("  4  -     TRANSCEIVER_RS530\n");
	      printf("  5  -     TRANSCEIVER_X21\n");
	      printf("  6  -     TRANSCEIVER_V35\n");
	      printf("  7  -     TRANSCEIVER_RS449\n");
	      printf("  8  -     TRANSCEIVER_V36\n");
	      printf("  9  -     TRANSCEIVER_RS232\n");
	      printf("  10 -     TRANSCEIVER_V28\n");
	      printf("  11 -     TRANSCEIVER_NO_CABLE\n");
	      printf("  0  -     return\n\n");
	      printf("select: ");
	      fgets( buffer, 50, stdin );
	      selnum = atoi(buffer);
	    } while ( (selnum != 0) && (selnum < 1) && (selnum > 11) );
	    
	    switch (selnum)
            {
            case 1:
	      OperationMode.transceivermode  = TRANSCEIVER_NO_CHANGES;
                break;
            case 2:
	      OperationMode.transceivermode  = TRANSCEIVER_NOT_USED;
                break;
            case 3:
	      OperationMode.transceivermode  = TRANSCEIVER_RS530A;
                break;
            case 4:
	      OperationMode.transceivermode  = TRANSCEIVER_RS530;
                break;
            case 5:
	      OperationMode.transceivermode  = TRANSCEIVER_X21;
                break;
            case 6:
	      OperationMode.transceivermode  = TRANSCEIVER_V35;
                break;
            case 7:
	      OperationMode.transceivermode = TRANSCEIVER_RS449;
                break;
            case 8:
	      OperationMode.transceivermode  = TRANSCEIVER_V36;
                break;
            case 9:
	      OperationMode.transceivermode  = TRANSCEIVER_RS232;
                break;
            case 10:
	      OperationMode.transceivermode  = TRANSCEIVER_V28;
                break;
            case 11:
	      OperationMode.transceivermode  = TRANSCEIVER_NO_CABLE;
	      break;
            default:
	      error = TRUE;
	      selnum = 99;
	      break;
            }
	   
            if (error) break;

            do {
                printf("DTE - DCE:\n");
                printf("  1 - DTE\n");
                printf("  2 - DCE\n");
                printf("  0 - return\n\n");
                printf("select: ");
                fgets( buffer, 50, stdin );
                selnum = atoi(buffer);
            } while ( (selnum != 0) && (selnum!=1) && (selnum!=2) );
            
            switch (selnum)
            {
            case 1:
	      OperationMode.dce_dte           = DCEDTE_DTE;
	      break;
            case 2:
	      OperationMode.dce_dte           = DCEDTE_DCE;
	      break;
            default:
	      error = TRUE;
	      selnum = 99;
	      break;
            }

            if (error) break;

            do {
                printf("TxClock output:\n");
                printf("  1 - enabled\n");
                printf("  2 - disabled\n");
                printf("  0 - return\n\n");
                printf("select: ");
                fgets( buffer, 50, stdin );
                selnum = atoi(buffer);
            } while ( (selnum != 0) && (selnum!=1) && (selnum!=2) );
            
            switch (selnum)
            {
            case 1:
                OperationMode.txclk_out  = ENABLED;
                break;
            case 2:
                OperationMode.txclk_out  = DISABLED;
                break;
            default:
                error = TRUE;
                selnum = 99;
                break;
            }

            if (error) break;

            do {
                printf("Use a TerminationCharacter (ASYNC):\n");
                printf("  1 - enabled\n");
                printf("  2 - disabled\n");
                printf("  0 - return\n\n");
                printf("select: ");
                fgets( buffer, 50, stdin );
                selnum = atoi(buffer);
            } while ( (selnum != 0) && (selnum!=1) && (selnum!=2) );
            
            switch (selnum)
            {
            case 1:
                OperationMode.usetermchar  = ENABLED;

                printf("  Enter TerminationCharacter: ");
                fgets(buffer, 50, stdin);
                OperationMode.termchar = buffer[0];
                break;
            case 2:
                OperationMode.usetermchar  = DISABLED;
                break;
            default:
                error = TRUE;
                selnum = 99;
                break;
            }

            if (error) break;


            printf("baudrate: ");
            fgets(buffer, 50, stdin);
            
            OperationMode.baudrate  = atoi(buffer);
	    
            //OperationMode.transceivermode   = TRANSCEIVER_RS530;
            //OperationMode.transceivermode   = TRANSCEIVER_RS232;
            //OperationMode.dce_dte           = DCEDTE_DCE;
            OperationMode.oversampling      = ENABLED;
            //OperationMode.oversampling      = DISABLED;
            OperationMode.clockinversion    = CLOCKINV_NONE;

            result = ioctl(hCurrent, TP862_IOCS_SET_OPERATION_MODE, &OperationMode);
            //
            //  Check the result of the last device I/O control operation
            //
            if( result >= 0 ) {
                printf( "\nIOCTL function called successfully.\n" );
            }
            else {
                printf("Operation failed (0x%X)\n", result);
            }
            break;

        case 16:
            //
            // get operation mode
            //
            result = ioctl(hCurrent, TP862_IOCG_GET_OPERATION_MODE, &OperationMode);
            if (result>=0)
            {
                printf("Operation Mode:\n");
                printf("  Commtype:           %s\n", commtypeToString(OperationMode.commtype));
                printf("  Clockmode:          %s\n", clockmodeToString(OperationMode.clockmode));
                printf("  TxClk output:       %s\n", ((OperationMode.txclk_out==ENABLED) ? "ENABLED" : "DISABLED"));
                printf("  Transceiver:        %s\n", transceiverToString(OperationMode.transceivermode));
                printf("  DCE/DTE:            %s\n", dcedteToString(OperationMode.dce_dte));
                printf("  Async Oversampling: %s\n", ((OperationMode.oversampling==ENABLED) ? "ENABLED" : "DISABLED"));
                printf("  Use TermChar:       %s\n", ((OperationMode.usetermchar==ENABLED) ? "ENABLED" : "DISABLED"));
                printf("      termchar:       '%c'\n", OperationMode.termchar);
                printf("  Baudrate:           %ld\n", OperationMode.baudrate);
                printf("  TxClk inversion:    %s\n", ((OperationMode.clockinversion & CLOCKINV_TXC) ? "inverted" : "not inverted"));
                printf("  RxClk inversion:    %s\n", ((OperationMode.clockinversion & CLOCKINV_RXC) ? "inverted" : "not inverted"));
            } else {
                printf("Operation failed (0x%X)\n", result);
            }
            break;

        case 20:
            result = ioctl(hCurrent, TP862_IOCT_SET_RECEIVER_STATE, TP_RECEIVERSTATE_OFF);
            break;
        case 21:
            result = ioctl(hCurrent, TP862_IOCT_SET_RECEIVER_STATE, TP_RECEIVERSTATE_ON);
            break;

        case 30:
            //
            // send some text
            //
			printf("  Enter send-data (max. 100)  : ");
			fgets( buffer, 100, stdin );
            nbytes = strlen(buffer);
            printf("length=%d\n", nbytes);
            // kill line-feed
            // terminate string
            buffer[nbytes-1] = '\0';
            memcpy( databuffer, buffer, nbytes );
            nbytes = strlen(databuffer)+1;
            result = write( hCurrent, databuffer, nbytes );
            //
            //  Check the result of the last device I/O control operation
            //
            if( result < nbytes-1 ) {
                printf( "Not all data written\n"); 
            }            
            break;


        case 31:
            //
            // send some text
            //
			printf("  Enter send-data (max. 100)  : ");
			fgets( buffer, 100, stdin );
            nbytes = strlen(buffer);

            printf("  Send cycles : ");
			fgets( select, 50, stdin);
            ulValue = atoi(select);

            // kill line-feed
            //nbytes = (nbytes > 0) ? (nbytes-1) : nbytes;
            // terminate string
            buffer[nbytes-1] = '\0';
            for (i=0; i<ulValue; i++)
            {
                sprintf(databuffer, "%s(%d)", buffer, i);
                nbytes = strlen(databuffer)+1;
                result = write( hCurrent, databuffer, nbytes );
                //
                //  Check the result of the last device I/O control operation
                //
                if( result < nbytes ) {
                    printf( "Not all data written (%d instead of %d)\n", result, nbytes);
                }            
            }
            break;

        case 35:
            //
            // send a large packet
            //
            pBuffer = (char*)malloc(5000*sizeof(char));

            memset(pBuffer, 0, 5000);
            remaining = 5000;
            count = 0;
            while (remaining)
            {
                memset(buffer, 0, 100);
                sprintf(buffer, "(%d)", count);
                if (remaining >= (strlen(buffer)+1))
                {
                    memcpy(&pBuffer[strlen(pBuffer)], buffer, (strlen(buffer)+1));
                    remaining -= strlen(buffer);
                } else {
                    break;
                }
                count++;
            }
            printf("Ok, we counted from 0 to %d and used %d bytes.\n", count-1, strlen(pBuffer));
            //printf("%s\n", pTxBuf);

            nbytes = strlen(pBuffer)+1;
            result = write( hCurrent, pBuffer, nbytes );
            //
            //  Check the result of the last device I/O control operation
            //
            if( result < nbytes-1 ) {
                printf( "Not all data written\n"); 
            }            

            free( pBuffer );
            break;

        case 51:
            //
            // read some frames with read-method
            //
            printf("Number of frames: ");
            fgets( select, 50, stdin );
            count = atoi( select );
            printf("How many Bytes: ");
            fgets( select, 50, stdin );
            nbytes = atoi( select );

            pBuffer = (char*)malloc(nbytes*sizeof(char));

            for (i=0; i<count; i++)
            {

                result = read(hCurrent, pBuffer, nbytes);
                if (result>=0)
                {
                    if (result > 0)
                    {
                        // print buffer
                        printf("Buffer[%d]='", i);
                        for (idx=0; idx<result; idx++)
                        {
                            printf("%c", pBuffer[idx]);
                        }
                        printf("'\n");
                    } else {
                        printf("no data.\n");
                        break;
                    }
                } else {
                    printf("Error during READ operation (0x%X)\n", result);
                }

            }
            free( pBuffer );
            break;

        case 52:
            result = ioctl(hCurrent, TP862_IOC_CLEAR_RX_BUFFER, &OperationMode);
            break;

        case 60:
             printf("  Enter CPLD register offset : ");
             fgets( select, 50, stdin );
             RegStruct.offset = convertStringToInt( select );
             RegStruct.value = 0xFFFFFFFF;

            result = ioctl( hCurrent,
					TP862_IOCG_CPLD_REG_READ,
					&RegStruct);
            printf("CPLD[0x%lX] = 0x%lX\n", RegStruct.offset, RegStruct.value);
            break;

        case 61:
             printf("  Enter CPLD register offset : ");
             fgets( select, 50, stdin );
             RegStruct.offset = convertStringToInt( select );

             printf("  Enter CPLD register value : ");
             fgets( select, 50, stdin );
             RegStruct.value = convertStringToInt( select );

            result = ioctl( hCurrent,
					TP862_IOCS_CPLD_REG_WRITE,
					&RegStruct);
            break;

        case 62:
             printf("  Enter SCC register offset : ");
             fgets( select, 50, stdin );
             RegStruct.offset = convertStringToInt( select );
             RegStruct.value = 0xFFFFFFFF;

            result = ioctl( hCurrent,
					TP862_IOCG_SCC_REG_READ,
					&RegStruct);
            printf("SCC[0x%lX] = 0x%lX\n", RegStruct.offset, RegStruct.value);
            break;

        case 63:
             printf("  Enter SCC register offset : ");
             fgets( select, 50, stdin );
             RegStruct.offset = convertStringToInt( select );

             printf("  Enter SCC register value : ");
             fgets( select, 50, stdin );
             RegStruct.value = convertStringToInt( select );

            result = ioctl( hCurrent,
					TP862_IOCS_SCC_REG_WRITE,
					&RegStruct);
            break;

        case 80:
            rw_blocking = !rw_blocking;
            break;
        case 88:
            // change channel
            act_channel = (act_channel + 1) % NUM_CHANNELS;
            hCurrent = devHandle[act_board][act_channel];
            break;
        case 89:
            act_board = (act_board + 1) % found_modules;
            hCurrent = devHandle[act_board][act_channel];
            break;

        default:
            break;

        }

    } while (selnum != 0);


    // close devices
    for (i=0; i<MAX_BOARDS; i++)
    {
        for (channel=0; channel<NUM_CHANNELS; channel++)
        {
            if (devHandle[i][channel])
            {
                if (close( devHandle[i][channel] ) < 0) printf("Error closing device '%s_%d_%d'\n", DEV_BASE_NAME, i, channel);
            }
        }
    }


    return 0;
}




char* transceiverToString( TRANSCEIVER_MODE mode )
{
    switch (mode)
    {
    case TRANSCEIVER_NO_CHANGES:
        return "TRANSCEIVER_NO_CHANGES";
        break;
    case TRANSCEIVER_NOT_USED:
        return "TRANSCEIVER_NOT_USED";
        break;
    case TRANSCEIVER_RS530A:
        return "TRANSCEIVER_RS530A";
        break;
    case TRANSCEIVER_RS530:
        return "TRANSCEIVER_RS530";
        break;
    case TRANSCEIVER_X21:
        return "TRANSCEIVER_X21";
        break;
    case TRANSCEIVER_V35:
        return "TRANSCEIVER_V35";
        break;
    case TRANSCEIVER_RS449:
        return "TRANSCEIVER_RS449";
        break;
    case TRANSCEIVER_V36:
        return "TRANSCEIVER_V36";
        break;
    case TRANSCEIVER_RS232:
        return "TRANSCEIVER_RS232";
        break;
    case TRANSCEIVER_V28:
        return "TRANSCEIVER_V28";
        break;
    case TRANSCEIVER_NO_CABLE:
        return "TRANSCEIVER_NO_CABLE";
        break;
    default:
        return "unknown";
        break;
    }
}


char* dcedteToString( DCEDTE mode )
{
    switch (mode)
    {
    case DCEDTE_DCE:
        return "DCE";
        break;
    case DCEDTE_DTE:
        return "DTE";
        break;
    case DCEDTE_NO_CHANGES:
        return "no_changes";
        break;
    default:
        return "unknown";
        break;
    }
}


char* clockmodeToString( CLOCK_MODE mode )
{
    switch (mode)
    {
    case CLOCKMODE_CM0A:
        return "CLOCKMODE_CM0A";
        break;
    case CLOCKMODE_CM0B:
        return "CLOCKMODE_CM0B";
        break;
    case CLOCKMODE_CM3B:
        return "CLOCKMODE_CM3B";
        break;
    case CLOCKMODE_CM4:
        return "CLOCKMODE_CM4";
        break;
    case CLOCKMODE_CM7B:
        return "CLOCKMODE_CM7B";
        break;
    case CLOCKMODE_CM8A:
        return "CLOCKMODE_CM8A";
        break;
    case CLOCKMODE_CM8:
    case CLOCKMODE_CM8B:
        return "CLOCKMODE_CM8B";
        break;
    case CLOCKMODE_CM9:
        return "CLOCKMODE_CM9";
        break;
    case CLOCKMODE_NO_CHANGES:
        return "no_changes";
        break;
    default:
        return "unknown";
        break;
    }
}


char* commtypeToString( COMM_TYPE mode )
{
    switch (mode)
    {
    case COMMTYPE_NO_CHANGES:
        return "COMMTYPE_NO_CHANGES";
        break;
    case COMMTYPE_HDLC:
        return "COMMTYPE_HDLC";
        break;
    case COMMTYPE_ASYNC:
        return "COMMTYPE_ASYNC";
        break;
    default:
        return "unknown";
        break;
    }
}



/******************************************************************************
**  Function:
**      PrintErrorMessage
**
**  Description:
**      Formats a message string for the last error code
**
**  Arguments:
**      none
**
**  Return Value:
**      none
*******************************************************************************/

static void PrintErrorMessage(void)

{
    printf("%s\n", strerror (errno));
}

