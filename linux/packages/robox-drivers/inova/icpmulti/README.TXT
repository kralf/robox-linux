//-----------------------------------------------------------------------------
// Inova Computers GmbH. Sudetenstr 5, 87600 Kaufbeuren
//
// (c) Copyright 1998-2000. Alle Rechte vorbehalten.
//
// Author: BT
// 
// Date : Nov.01.2000
//
//-----------------------------------------------------------------------------
// Project:   ICP-MULTI Linux driver
// Titel:     Readme info for the Beat version of the driver
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// General information
//-----------------------------------------------------------------------------
The ICP-MULTI is a CompactPCI adapter card with 
- 16 digital inputs 
- 8 digital outputs
- 16 analog inputs
- 4 analog outputs
- 4 counters (DIN0-3)

This file contains the readme information for the first beta version of the 
Linux driver for the above card.

The Linux driver is written under RedHat Linux release 6.1, 
with the Kernel version: 2.2.12-20.

It was only tested with this configuration.

//-----------------------------------------------------------------------------
// Actual state of the development
//-----------------------------------------------------------------------------
The driver supports at the moment the digital input and output channels, 
including the 4 16-bit counter and the analog inputs and outputs.

The driver supports multiple ICP-MULTI I/O boards, it detects and initailizes 
all ICP-MULTI I/O boards in the system.

//-----------------------------------------------------------------------------
// Driver installation
//-----------------------------------------------------------------------------
At the moment there is no autoamtic installation support for the ICP-MULTI 
driver. The driver files should be copied onto your hard-disk. The driver files
are organized in the following folder structure:

/h - general Inova header files used by the driver module 

/icp_drv_gen - general Inova CPCI driver module
  |
  +-/bin - binary files of the general Inova CPCI driver module
  |        icp_drv_gen.o (if you recompile the driver, this file should be
  |        recreated here automatically)
  |
  +-/src - source files of the of the general Inova CPCI driver module 
  |         (with the makefile together)
  |
  +-/h   - local headers of the of the general Inova CPCI driver module


/icp_mio - ICP-MULTI multi I/O board's driver module
  |
  +-/bin - binary files of the ICP-MULTI multi I/O board's driver module
  |        icp_multi.o (if you recompile the driver, this file should be
  |        recreated here automatically)
  |
  +-/src - source files of the of ICP-MULTI multi I/O board's driver module
  |         (with the makefile together)
  |
  +-/h   - local headers of the of the ICP-MULTI multi I/O board's driver module
  |
  +-/obj - temporary object files
  |
  +-/test - test program source for the ICP-MULTI multi I/O board's driver module


The ICP-MULTI driver is driver module, which can be loaded dynamically into the
system. The driver module is not "standalone", it builds upon a general CPCI
driver used by the Inova CPCI adapter cards. To be able to use and load the 
ICP-MULTI driver this general CPCI driver should be loaded first. 

The Inova  general CPCI driver module is called icp_drv_gen.o, and can be found
in the icp_drv_gen/bin directory (see above). The driver module is loaded using 
the following command: "insmod icp_drv_gen.o"

After loading the Inova general CPCI driver the ICP-MULTI driver module can be 
loaded. The driver module's file can be found in the /icp_mio/bin folder (see above)
and called icp_multi.o. It can be loaded using the following command:
"insmod icp_multi.o"

After loading the driver, we need to create devices in the system's /dev folder to
be able to communicate with the driver. This task can be done with the help of
the /icp_mio/bin/dev_create shell script file. This shell script generates 
entries in the system's /dev folder which can be later used for communication
with the driver and so with the ICP-MULTI board. It can be executed after the
ICP-MULTI driver is loaded by using the command: "./dev_create0"

The shell script generates the devices only for one ICP-MULTI I/O board, it has
to be edited and executed for every ICP-MULTI I/O boards, if more than one 
ICP-MULTI I/O boards are present in the system. The shell script variables 
"device" "module" has to changed according to ICP-MULTI I/O board's number for
which the entries in the system's /dev folder are created. (mio0 for - board 0,
mio1 for board 1,...)

NOTE: There are 6 files for creating devices for 6 ICP-MULTI I/O boards 
in the system's /dev folder. (/bin/dev_create0 ... /bin/dev_create5)

After loading the driver module and creating the entries in the system's /dev
folder, the driver can be used using the folloing interface:


//-----------------------------------------------------------------------------
// Driver interface
//-----------------------------------------------------------------------------
The ICP-MUlti driver module is integrated into the Linux driver and file system:
An entry in the system's /dev folder is created by the "dev_create" shell script 
for every possible "channel" of the ICP-MULTI I/O board. This channel name follows 
always the identifier of the ICP-MULTI I/O board mioX, where X is the number of the
ICP-MULTI I/O board to use.
The channels are listed below:
mioX.di    - digital input channel

mioX.do    - digital output channel

mioX.ctr0  - counter0
mioX.ctr1  - counter1
mioX.ctr2  - counter2
mioX.ctr3  - counter3

mioX.ai0   - analog input channel0
mioX.ai1   - analog input channel1
mioX.ai2   - analog input channel2
mioX.ai3   - analog input channel3
mioX.ai4   - analog input channel4
mioX.ai5   - analog input channel5
mioX.ai6   - analog input channel6
mioX.ai7   - analog input channel7
mioX.ai8   - analog input channel8
mioX.ai9   - analog input channel9
mioX.ai10  - analog input channel10
mioX.ai11  - analog input channel11
mioX.ai12  - analog input channel12
mioX.ai13  - analog input channel13
mioX.ai14  - analog input channel14
mioX.ai15  - analog input channel15

mioX.ao0   - analog output channel0
mioX.ao1   - analog output channel1
mioX.ao2   - analog output channel2
mioX.ao3   - analog output channel3

The above listed entries in the system's /dev folder can be used to open a desired
channel of the ICP-MULTI I/O board. 
The following channels can be opened only for reading:
- digital input channel
- analog  input channels

The following channels can be opened only for writing:
- digital output channel
- analog  output channels

The following channels can be opened for read/write:
- counters

After opening a channel the standard read, write routines can be used to exchange data
with the driver.



//-----------------------------------------------------------------------------
// Digital inputs
//-----------------------------------------------------------------------------
The digital inputs can be read by reading from the opened digital input channel.
The digital inputs can be read in 2 modes:
a.) normal read mode
The data is retrieved immediatly from the card. 
b.) read with waiting for the Status change interrupt of the digital inputs.
The read operation is blocked untill a status change interrupt is generated
by the ICP-MULTI I/O board.

To switch between the 2 read modes use the following ioctl function call:

ioctl(fd_di,MIO_IOCTL_DI_READ_MODE, 0)  - normal read mode
ioctl(fd_di,MIO_IOCTL_DI_READ_MODE, 1)  - read mode with waiting for status change

The ioctl codes of the ICP-MULTI I/O board can be found in 
"/icp_mio/h/icp_mio_hw.h" file

//-----------------------------------------------------------------------------
// Digital outputs
//-----------------------------------------------------------------------------
The digital outputs can be written by writting to the opened digital output channel.
The digital outputs can be written in 3 modes:

a.) normal write mode
The data is written immediatly to the card. 

b.) write with waiting for the Digital output error interrupt, no update on the outputs.
The write operation is blocked untill the digital output error interrupt is generated
by the ICP-MULTI I/O board.

c.) write with waiting for the Digital output error interrupt,  update on the outputs.
The write operation is blocked untill the digital output error interrupt is generated
by the ICP-MULTI I/O board, after this interrupt a preselected data is written to the
outputs.


To switch between the write modes use the following ioctl function call:

ioctl(fd_do,MIO_IOCTL_DO_WRITE_MODE, 2)  - normal write mode
ioctl(fd_do,MIO_IOCTL_DO_WRITE_MODE, 1)  - write with waiting for irq no update
ioctl(fd_do,MIO_IOCTL_DO_WRITE_MODE, 0)  - write with waiting for irq + update

The value written after a dout error interrupt can be set by the following ioctl call:

ioctl(fd_do,MIO_IOCTL_DO_ERROR_OUT_VAL, 0x55)

The ioctl codes of the ICP-MULTI I/O board can be found in 
"/icp_mio/h/icp_mio_hw.h" file

//-----------------------------------------------------------------------------
// Counters
//-----------------------------------------------------------------------------
The counters can be opened for both read and write opertaions. 
The read operation retrieves the data immediatelly from the selected counter. 
The write operation can work in 2 modes:

a.) normal write mode
The data is written immediatly to the counter. 

b.) write with wakeup
The data is written immediatly to the card and then it is blocked so long a
counter overflow irq is occured.

To switch between the write modes use the following ioctl function call:

ioctl(fd_ctr[k],MIO_IOCTL_CTR_CH_MODE, 0) - normal write mode
ioctl(fd_ctr[k],MIO_IOCTL_CTR_CH_MODE, 1) - write with blocking

The ioctl codes of the ICP-MULTI I/O board can be found in 
"/icp_mio/h/icp_mio_hw.h" file

//-----------------------------------------------------------------------------
// Analog inputs
//-----------------------------------------------------------------------------
The analog inputs can be read by reading from an opened analog input channel.
The analog inputs can be read in different modes:

 - 5V  unipolar non-doifferential mode
 - 10V unipolar non-differential mode
 - 5V  bipolar non-differential mode
 - 10V bipolar non-differential mode

 - 5V  unipolar differential mode
 - 10V unipolar differential mode
 - 5V  bipolar differential mode
 - 10V bipolar differential mode

In addition the measurement unit of the retrieved data can be also set to the 
following values:
 - 100 uV units
 - 1   mV units
 - 10  mV units
 - 100 mV units
 - 1   V  units

The above options can be set for each analog input channel individually, but please note, 
that in differential input mode only 8 analog input channels can be used.

To set the above options the following ioctl() function calls can be used:

 a.) Input range (5V/10V)
     ioctl(fd_ai_ch, MIO_IOCTL_AI_CH_ADC_INP_RANGE, range); 
     If range is TRUE the channel is set to 10V input range, if false to 5V
     input range.

 b.) Unipolar/Bipolar input range
     ioctl(fd_ai_ch, MIO_IOCTL_AI_CH_ADC_BIP_INP_RANGE, range); 
     If range is TRUE the channel is set to bipolar input range, if false to unipolar
     input range.

 c.) Differential/Non-differential input mode
     ioctl(fd_ai_ch,MIO_IOCTL_AI_CH_ADC_DIFF_INP_MODE, mode)
     If mode is TRUE the channel is set to differential input mode, if false to 
     non-differential input mode. (Please note, that only the channels 0-7 can be
     set to differential input mode.) 

 d.) Measurment units
     ioctl(fd_ai_ch,MIO_IOCTL_AI_CH_ADC_UNITS, units); 
     units can have the following values:
     ADC_100uV_UNIT  - 100uV units
     ADC_mV_UNIT     - 1mV   units
     ADC_10mV_UNIT   - 10mV units
     ADC_100mV_UNIT  - 100mV units 
     ADC_V_UNIT      - 1V units

The ioctl codes of the ICP-MULTI I/O board can be found in 
"/icp_mio/h/icp_mio_hw.h" file

Known limitations:
 - The ICP-MULTI I/O board can be set up for currency measurment, this option is
   at the moment not supported by the driver.

//-----------------------------------------------------------------------------
// Analog outputs
//-----------------------------------------------------------------------------
The analog outputs can be written by writting to an opened analog output channel.
The analog outputs can be written in different modes:

 - 5V  unipolar mode
 - 10V unipolar mode
 - 5V  bipolar mode
 - 10V bipolar mode

In addition the measurement unit of the data to be written can be also set to the 
following values:
 - 100 uV units
 - 1   mV units
 - 10  mV units
 - 100 mV units
 - 1   V  units

The above options can be set for each analog output channel individually.

To set the above options the following ioctl() function calls can be used:

 a.) Input range (5V/10V)
     ioctl(fd_ao_ch, MIO_IOCTL_AO_CH_DAC_INP_RANGE, range); 
     If range is TRUE the channel is set to 10V input range, if false to 5V
     input range.

 b.) Unipolar/Bipolar input range
     ioctl(fd_ao_ch, MIO_IOCTL_AO_CH_DAC_BIP_INP_RANGE, range); 
     If range is TRUE the channel is set to bipolar input range, if false to unipolar
     input range.

 c.) Measurment units
     ioctl(fd_ao_ch,MIO_IOCTL_AO_CH_DAC_UNITS, units); 
     units can have the following values:
     DAC_100uV_UNIT  - 100uV units
     DAC_mV_UNIT     - 1mV   units
     DAC_10mV_UNIT   - 10mV units
     DAC_100mV_UNIT  - 100mV units 
     DAC_V_UNIT      - 1V units

The ioctl codes of the ICP-MULTI I/O board can be found in 
"/icp_mio/h/icp_mio_hw.h" file

//-----------------------------------------------------------------------------
// Files in the /test folder
//-----------------------------------------------------------------------------
There are test files for testing the card, it is a very raw state, but can be 
used as reference for bringing up the driver. Unfortunatelly I haven't got enough 
time to precisely "comment" this file.

//-----------------------------------------------------------------------------
// ICP-MULTI driver compilation
//-----------------------------------------------------------------------------
The Makefile contains Variables for setting the Debug outputs for each 
sw module of the driver, they can be used for switching on/off the debug messages.
This variables can be found at the beginning of the Makefile.


//-----------------------------------------------------------------------------
// End of file
//-----------------------------------------------------------------------------
