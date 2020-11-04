//---------------------------------------------------------------------------
//
//  ue9_externaltriggerstream.c
//  
//  Does a 6-channel externally triggered stream of AIN0, AIN1, EIO_FIO,
//	MIO_CIO, Timer0_Low, and Timer0_High.
//
//	Timer0 (FIO0) is enabled as SystemTimerLow (mode 10).  With an externally
//	triggered stream, the timing is determined by the external trigger signal,
//	so this timer value can be used to timestamp each scan.
//
//	Timer1 (FIO1) is enabled as frequency output at 1 kHz, and can be
//	used as the external trigger signal for testing.
//
//	External triggering is enabled for the stream, so the trigger signal should
//	be connected to FIO2.
//
//	For testing, connect FIO1 to FIO2.
//
//  support@labjack.com
//  April 30, 2007
//----------------------------------------------------------------------
//

#include <stdio.h>
#include <conio.h>
#include <windows.h>
#include "c:\program files\labjack\drivers\LabJackUD.h"
//The project must also know where to find labjackud.lib.  Here we do
//that by putting the lib file in the file view to the left.  The relative
//path stored by Visual Studio might not be the same on your machine, so
//you will probably have to delete the lib in the file view and right-click
//to re-add it from the install location ...\labjack\drivers\labjackud.lib.
//Another option, besides using the .lib, is dynamic linking.  Some
//compilers might not be able to link the .lib, and dynamic linking
//is usually the solution in those cases.




//This is our simple error handling function that is called after every UD
//function call.  This function displays the errorcode and string description
//of the error.  It also has a line number input that can be used with the
//macro __LINE__ to display the line number in source code that called the
//error handler.  It also has an iteration input is useful when processing
//results in a loop (getfirst/getnext).
void ErrorHandler (LJ_ERROR lngErrorcode, long lngLineNumber, long lngIteration)
{
	char err[255];

	if (lngErrorcode != LJE_NOERROR)
	{
		ErrorToString(lngErrorcode,err);
		printf("Error number = %d\n",lngErrorcode);
		printf("Error string = %s\n",err);
		printf("Source line number = %d\n",lngLineNumber);
		printf("Iteration = %d\n\n",lngIteration);
		if(lngErrorcode > LJE_MIN_GROUP_ERROR)
		{
			//Quit if this is a group error.
			getchar();
			exit(0);
		}
   }
}





main()
{
	LJ_ERROR lngErrorcode;
	LJ_HANDLE lngHandle=0;
	long lngGetNextIteration;
	long i=0,k=0;
	long lngIOType=0, lngChannel=0;
	double dblValue=0, dblCommBacklog=0;

	//The actual scan rate is determined by the external clock, but we need
	//an idea of how fast the scan rate will be so that we can make
	//the buffers big enough.  Also, the driver needs to have an idea of the
	//expected scan rate to help it decide how big of packets to transfer.
	double scanRate = 1000;
	long delayms = 1000;
	double numScans = 2000;  //2x the expected # of scans (2*scanRate*delayms/1000)
	double numScansRequested;
	double adblData[12000] = {0};  //Max buffer size (#channels*numScansRequested)


	//Make a long parameter which holds the address of the data array.  We do this
	//so the compiler does not generate a warning in the eGet call that retrieves
	//stream data.  Note that the x1 parameter  in eGet (and AddRequest) is fairly
	//generic, in that sometimes it could just be a write parameter, and sometimes
	//it has the address of an array.  Since x1 is not declared as a pointer, the
	//compiler will complain if you just pass the array pointer without casting
	//it to a long as follows.
	long padblData = (long)&adblData[0];


	//Open the first found LabJack UE9 over USB.
	lngErrorcode = OpenLabJack (LJ_dtUE9, LJ_ctUSB, "1", 1, &lngHandle);
	//lngErrorcode = OpenLabJack (LJ_dtUE9, LJ_ctETHERNET, "192.168.1.184", 0, &lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);


	//Make sure the UE9 is not streaming.
    lngErrorcode = eGet(lngHandle, LJ_ioSTOP_STREAM, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);


	//Disable all timers and counters to put everything in a known initial state.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chNUMBER_TIMERS_ENABLED, 0, 0, 0);
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_COUNTER_ENABLE, 0, 0, 0, 0);
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_COUNTER_ENABLE, 1, 0, 0, 0);
	lngErrorcode = GoOne(lngHandle);


	//First we will configure Timer0 as system timer low and configure Timer1 to
	//output a 1000 Hz square wave.

    //Use the fixed 750kHz timer clock source.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_BASE, LJ_tc750KHZ, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Set the divisor to 3 so the actual timer clock is 250 kHz.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_DIVISOR, 3, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Enable 2 timers.  They will use FIO0-FIO1.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chNUMBER_TIMERS_ENABLED, 2, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Configure Timer0 as system timer low.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_MODE, 0, LJ_tmSYSTIMERLOW, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Configure Timer1 as frequency output.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_MODE, 1, LJ_tmFREQOUT, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Set the frequency output on Timer1 to 1000 Hz (250000/(2*125) = 1000).
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_VALUE, 1, 125, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Execute the requests on a single LabJack.  The driver will use a
    //single low-level TimerCounter command to handle all the requests above.
	lngErrorcode = GoOne(lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Get all the results just to check for errors.
	lngErrorcode = GetFirstResult(lngHandle, &lngIOType, &lngChannel, &dblValue, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	lngGetNextIteration=0;	//Used by the error handling function.
	while(lngErrorcode < LJE_MIN_GROUP_ERROR)
	{
		lngErrorcode = GetNextResult(lngHandle, &lngIOType, &lngChannel, &dblValue, 0, 0);
		if(lngErrorcode != LJE_NO_MORE_DATA_AVAILABLE)
		{
			ErrorHandler(lngErrorcode, __LINE__, lngGetNextIteration);
		}

		lngGetNextIteration++;
	}




	//Configure the stream:
    //Configure resolution for all analog inputs.  Since the test external clock
	//is at 1000 Hz, and we are scanning 6 channels, we will have a
	//sample rate of 6000 samples/second.  That means the maximum resolution
	//we could use is 13-bit.  We will use 12-bit in this example.
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chAIN_RESOLUTION, 12, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    //Configure the analog input range on channel 0 for bipolar +-5 volts.
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_AIN_RANGE, 0, LJ_rgBIP5V, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
	//Configure the analog input range on channel 1 for bipolar +-5 volts.
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_AIN_RANGE, 1, LJ_rgBIP5V, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    //Give the driver a 5 second buffer (scanRate * 6 channels * 5 seconds).
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chSTREAM_BUFFER_SIZE, scanRate*6*5, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    //Configure reads to retrieve whatever data is available without waiting (wait mode LJ_swNONE).
	//See comments below to change this program to use LJ_swSLEEP mode.
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chSTREAM_WAIT_MODE, LJ_swNONE, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
	//Configure for external triggering.
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chSTREAM_EXTERNAL_TRIGGER, 1, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    //Define the scan list.
    lngErrorcode = AddRequest(lngHandle, LJ_ioCLEAR_STREAM_CHANNELS, 0, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    lngErrorcode = AddRequest(lngHandle, LJ_ioADD_STREAM_CHANNEL, 0, 0, 0, 0);	//AIN0
    ErrorHandler(lngErrorcode, __LINE__, 0);
    lngErrorcode = AddRequest(lngHandle, LJ_ioADD_STREAM_CHANNEL, 1, 0, 0, 0);	//AIN1
    ErrorHandler(lngErrorcode, __LINE__, 0);
	lngErrorcode = AddRequest(lngHandle, LJ_ioADD_STREAM_CHANNEL, 193, 0, 0, 0);	//EIO_FIO
    ErrorHandler(lngErrorcode, __LINE__, 0);
	lngErrorcode = AddRequest(lngHandle, LJ_ioADD_STREAM_CHANNEL, 194, 0, 0, 0);	//MIO_CIO
    ErrorHandler(lngErrorcode, __LINE__, 0);
	lngErrorcode = AddRequest(lngHandle, LJ_ioADD_STREAM_CHANNEL, 200, 0, 0, 0);	//Timer0 LSW
    ErrorHandler(lngErrorcode, __LINE__, 0);
	lngErrorcode = AddRequest(lngHandle, LJ_ioADD_STREAM_CHANNEL, 224, 0, 0, 0);	//Timer0 MSW
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
    //Execute the list of requests.
    lngErrorcode = GoOne(lngHandle);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
	//Get all the results just to check for errors.
	lngErrorcode = GetFirstResult(lngHandle, &lngIOType, &lngChannel, &dblValue, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	lngGetNextIteration=0;	//Used by the error handling function.
	while(lngErrorcode < LJE_MIN_GROUP_ERROR)
	{
		lngErrorcode = GetNextResult(lngHandle, &lngIOType, &lngChannel, &dblValue, 0, 0);
		if(lngErrorcode != LJE_NO_MORE_DATA_AVAILABLE)
		{
			ErrorHandler(lngErrorcode, __LINE__, lngGetNextIteration);
		}

		lngGetNextIteration++;
	}
    
	//Start the stream.
    lngErrorcode = eGet(lngHandle, LJ_ioSTART_STREAM, 0, &dblValue, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);

  

	//Read data
	while(!kbhit())	//Loop will run until any key is hit
	{
		//Since we are using wait mode LJ_swNONE, we will wait a little, then
		//read however much data is available.  Thus this delay will control how
		//fast the program loops and how much data is read each loop.  An
		//alternative common method is to use wait mode LJ_swSLEEP where the
		//stream read waits for a certain number of scans.  In such a case
		//you would not have a delay here, since the stream read will actually
		//control how fast the program loops.
		//
		//To change this program to use sleep mode,
		//	-change numScans to the actual number of scans desired per read,
		//	-change wait mode addrequest value to LJ_swSLEEP,
		//	-comment out the following Sleep command.

		Sleep(delayms);	//Remove if using LJ_swSLEEP.

		//init array so we can easily tell if it has changed
		for(k=0;k<numScans*2;k++)
		{
			adblData[k] = 9999.0;
		}

		//Read the data.  We will request twice the number we expect, to
		//make sure we get everything that is available.
		//Note that the array we pass must be sized to hold enough SAMPLES, and
		//the Value we pass specifies the number of SCANS to read.
		numScansRequested=numScans;
		lngErrorcode = eGet(lngHandle, LJ_ioGET_STREAM_DATA, LJ_chALL_CHANNELS, &numScansRequested, padblData);
		//This displays the number of scans that were actually read.
		printf("\nIteration # %d\n",i);
		printf("Number read = %.0f\n",numScansRequested);
		//This displays just the first scan.
		printf("First scan = %.6f,%.6f,%.6f,%.6f,%.6f, %.6f\n",adblData[0],adblData[1],adblData[2],adblData[3],adblData[4],adblData[5]);
		ErrorHandler(lngErrorcode, __LINE__, 0);
		//Retrieve the current Comm backlog.  The UD driver retrieves stream data from
		//the UE9 in the background, but if the computer is too slow for some reason
		//the driver might not be able to read the data as fast as the UE9 is
		//acquiring it, and thus there will be data left over in the UE9 buffer.
		lngErrorcode = eGet(lngHandle, LJ_ioGET_CONFIG, LJ_chSTREAM_BACKLOG_COMM, &dblCommBacklog, 0);
		printf("Comm Backlog = %.0f\n",dblCommBacklog);
		i++;
	}

   
    //Stop the stream
    lngErrorcode = eGet(lngHandle, LJ_ioSTOP_STREAM, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);

	//Disable the timers.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chNUMBER_TIMERS_ENABLED, 0, 0, 0);
	lngErrorcode = GoOne(lngHandle);

	printf("\nDone");
	getchar();
	return 0;

}
