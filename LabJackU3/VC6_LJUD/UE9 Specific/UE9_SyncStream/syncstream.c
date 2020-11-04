//---------------------------------------------------------------------------
//
//  syncstream.c
//  
//  Demonstrates synchronized streaming with 2 UE9s.  Connect FIO0 on unit A
//  to FIO0 on unit B.  If the units are isolated from each other, generally
//  meaning no USB cables, you should also connect GND to GND.  Note that
//	the external clock input or output uses FIO0 in this example since
//  no other timers or counters are enabled.  The external clock uses
//	Counter1, so if any other timers or counters are enabled, the external
//  stream clock will be on the next available FIO line, as Counter1 is
//  always last.  If using other timer/counters, it is recommended that
//	they be configured before the stream.
//
//  We have done one setup in particular with 8 synchronized UE9s scanning 8
//  channels each at 2 kscans/second (16 ksamples/second per UE9).  One thing
//  required for synchronizing multiple UE9s is a good clean sync signal.  If
//  you combine many UE9s, long sync cables, and a very noisy environment,
//  special care might be needed for the sync cables.  For instance,
//  differential line drivers can be used to send around the sync signal.
//  The above application with 8 UE9s was in an industrial setting with a
//  very noisy overhead crane, so we used the LJTick-Proto to build line
//  drivers using the MAX3082E, and even threw in an IL710 to provide
//	isolation between each UE9 (schematic available, although the LJTick-Proto
//  is not yet available as of this writing).
//
//  support@labjack.com
//  Feb 3, 2006
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
	long lngGetNextIteration;
	LJ_HANDLE lngHandleA=0, lngHandleB=0;
	long i=0,k=0;
	long lngIOType=0, lngChannel=0;
	double dblValue=0, dblCommBacklog=0;
	double scanRate = 1000.0;
	long delayms = 1000;
	double numScans = 2000;  //2x the expected # of scans (2*scanRate*delayms/1000)
	double numScansRequested;
	double adblDataA[4000] = {0};  //Max buffer size (#channels*numScansRequested)
	double adblDataB[4000] = {0};

	//Make a long parameter which holds the address of the data array.  We do this
	//so the compiler does not generate a warning in the eGet call that retrieves
	//stream data.  Note that the x1 parameter  in eGet (and AddRequest) is fairly
	//generic, in that sometimes it could just be a write parameter, and sometimes
	//it has the address of an array.  Since x1 is not declared as a pointer, the
	//compiler will complain if you just pass the array pointer without casting
	//it to a long as follows.
	long padblDataA = (long)&adblDataA[0];
	long padblDataB = (long)&adblDataB[0];


	//Open the LabJack UE9s.
	lngErrorcode = OpenLabJack (LJ_dtUE9, LJ_ctUSB, "2", 0, &lngHandleA);
	//lngErrorcode = OpenLabJack (LJ_dtUE9, LJ_ctETHERNET, "192.168.1.187", 0, &lngHandleA);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	lngErrorcode = OpenLabJack (LJ_dtUE9, LJ_ctUSB, "3", 0, &lngHandleB);
	//lngErrorcode = OpenLabJack (LJ_dtUE9, LJ_ctETHERNET, "192.168.1.188", 0, &lngHandleB);
	ErrorHandler(lngErrorcode, __LINE__, 0);


	//Make sure devices are not streaming.
    lngErrorcode = eGet(lngHandleA, LJ_ioSTOP_STREAM, 0, 0, 0);
	lngErrorcode = eGet(lngHandleB, LJ_ioSTOP_STREAM, 0, 0, 0);


	//Configure the stream on unit A:
    //Configure analog inputs for 12-bit resolution
    lngErrorcode = AddRequest(lngHandleA, LJ_ioPUT_CONFIG, LJ_chAIN_RESOLUTION, 12, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    //Configure the analog input range on channels 0 and 1 for bipolar +-5 volts.
    lngErrorcode = AddRequest(lngHandleA, LJ_ioPUT_AIN_RANGE, 0, LJ_rgBIP5V, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    lngErrorcode = AddRequest(lngHandleA, LJ_ioPUT_AIN_RANGE, 1, LJ_rgBIP5V, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    //Set the scan rate.
    lngErrorcode = AddRequest(lngHandleA, LJ_ioPUT_CONFIG, LJ_chSTREAM_SCAN_FREQUENCY, scanRate, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    //Give the driver a 5 second buffer (scanRate * 2 channels * 5 seconds).
    lngErrorcode = AddRequest(lngHandleA, LJ_ioPUT_CONFIG, LJ_chSTREAM_BUFFER_SIZE, scanRate*2*5, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    //Configure reads to retrieve whatever data is available without waiting.
    lngErrorcode = AddRequest(lngHandleA, LJ_ioPUT_CONFIG, LJ_chSTREAM_WAIT_MODE, LJ_swNONE, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
	//Configure for internal or external triggering.
    lngErrorcode = AddRequest(lngHandleA, LJ_ioPUT_CONFIG, LJ_chSTREAM_EXTERNAL_TRIGGER, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    //Enable or disable stream clock output.
    lngErrorcode = AddRequest(lngHandleA, LJ_ioPUT_CONFIG, LJ_chSTREAM_CLOCK_OUTPUT, 1, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    //Define the scan list as AIN0 then AIN1.
    lngErrorcode = AddRequest(lngHandleA, LJ_ioCLEAR_STREAM_CHANNELS, 0, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    lngErrorcode = AddRequest(lngHandleA, LJ_ioADD_STREAM_CHANNEL, 0, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    lngErrorcode = AddRequest(lngHandleA, LJ_ioADD_STREAM_CHANNEL, 1, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
    //Execute the list of requests.
    lngErrorcode = GoOne(lngHandleA);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
	//Get all the results just to check for errors.
	lngErrorcode = GetFirstResult(lngHandleA, &lngIOType, &lngChannel, &dblValue, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	lngGetNextIteration=0;	//Used by the error handling function.
	while(lngErrorcode < LJE_MIN_GROUP_ERROR)
	{
		lngErrorcode = GetNextResult(lngHandleA, &lngIOType, &lngChannel, &dblValue, 0, 0);
		if(lngErrorcode != LJE_NO_MORE_DATA_AVAILABLE)
		{
			ErrorHandler(lngErrorcode, __LINE__, lngGetNextIteration);
		}

		lngGetNextIteration++;
	}
    


	//Configure the stream on unit B:
    //Configure analog inputs for 12-bit resolution
    lngErrorcode = AddRequest(lngHandleB, LJ_ioPUT_CONFIG, LJ_chAIN_RESOLUTION, 12, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    //Configure the analog input range on channels 0 and 1 for bipolar +-5 volts.
    lngErrorcode = AddRequest(lngHandleB, LJ_ioPUT_AIN_RANGE, 0, LJ_rgBIP5V, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    lngErrorcode = AddRequest(lngHandleB, LJ_ioPUT_AIN_RANGE, 1, LJ_rgBIP5V, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    //Give the driver a 5 second buffer (scanRate * 2 channels * 5 seconds).
    lngErrorcode = AddRequest(lngHandleB, LJ_ioPUT_CONFIG, LJ_chSTREAM_BUFFER_SIZE, scanRate*2*5, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    //Configure reads to retrieve whatever data is available without waiting.
    lngErrorcode = AddRequest(lngHandleB, LJ_ioPUT_CONFIG, LJ_chSTREAM_WAIT_MODE, LJ_swNONE, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
	//Configure for internal or external triggering.
    lngErrorcode = AddRequest(lngHandleB, LJ_ioPUT_CONFIG, LJ_chSTREAM_EXTERNAL_TRIGGER, 1, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    //Enable or disable stream clock output.
    lngErrorcode = AddRequest(lngHandleB, LJ_ioPUT_CONFIG, LJ_chSTREAM_CLOCK_OUTPUT, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    //Define the scan list as AIN0 then AIN1.
    lngErrorcode = AddRequest(lngHandleB, LJ_ioCLEAR_STREAM_CHANNELS, 0, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    lngErrorcode = AddRequest(lngHandleB, LJ_ioADD_STREAM_CHANNEL, 0, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    lngErrorcode = AddRequest(lngHandleB, LJ_ioADD_STREAM_CHANNEL, 1, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
    //Execute the list of requests.
    lngErrorcode = GoOne(lngHandleB);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
	//Get all the results just to check for errors.
	lngErrorcode = GetFirstResult(lngHandleB, &lngIOType, &lngChannel, &dblValue, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	lngGetNextIteration=0;	//Used by the error handling function.
	while(lngErrorcode < LJE_MIN_GROUP_ERROR)
	{
		lngErrorcode = GetNextResult(lngHandleB, &lngIOType, &lngChannel, &dblValue, 0, 0);
		if(lngErrorcode != LJE_NO_MORE_DATA_AVAILABLE)
		{
			ErrorHandler(lngErrorcode, __LINE__, lngGetNextIteration);
		}

		lngGetNextIteration++;
	}



	//Start the stream on unit B first, so it is ready to receive clocks from unit A.
    lngErrorcode = eGet(lngHandleB, LJ_ioSTART_STREAM, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
	lngErrorcode = eGet(lngHandleA, LJ_ioSTART_STREAM, 0, &dblValue, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
	//The actual scan rate is dependent on how the desired scan rate divides into
	//the LabJack clock.  The actual scan rate is returned in the value parameter
	//from the start stream command.
	printf("Actual Scan Rate = %.3f\n",dblValue);
	printf("Actual Sample Rate = %.3f\n",2*dblValue);


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
		Sleep(delayms);

		//init arrays so we can easily tell if they have changed
		for(k=0;k<numScans*2;k++)
		{
			adblDataA[k] = 9999.0;
			adblDataB[k] = 9999.0;
		}

		//Read the data.  We will request twice the number we expect, to
		//make sure we get everything that is available.
		//Note that the array we pass must be sized to hold enough SAMPLES, and
		//the Value we pass specifies the number of SCANS to read.
		numScansRequested=numScans;
		lngErrorcode = eGet(lngHandleA, LJ_ioGET_STREAM_DATA, LJ_chALL_CHANNELS, &numScansRequested, padblDataA);
		//This displays the number of scans that were actually read.
		printf("\nIteration # %d\n",i);
		printf("Number read from unit A = %.0f\n",numScansRequested);
		//This displays just the first scan.
		printf("First scan A = %.3f, %.3f\n",adblDataA[0],adblDataA[1]);
		ErrorHandler(lngErrorcode, __LINE__, 0);
		//Retrieve and display the Comm Backlog.
		lngErrorcode = eGet(lngHandleA, LJ_ioGET_CONFIG, LJ_chSTREAM_BACKLOG_COMM, &dblCommBacklog, 0);
		printf("Comm Backlog A = %.0f\n",dblCommBacklog);

		numScansRequested=numScans;
		lngErrorcode = eGet(lngHandleB, LJ_ioGET_STREAM_DATA, LJ_chALL_CHANNELS, &numScansRequested, padblDataB);
		//This displays the number of scans that were actually read.
		printf("\nIteration # %d\n",i);
		printf("Number read from unit B = %.0f\n",numScansRequested);
		//This displays just the first scan.
		printf("First scan B = %.3f, %.3f\n",adblDataB[0],adblDataB[1]);
		ErrorHandler(lngErrorcode, __LINE__, 0);
		//Retrieve and display the Comm Backlog.
		lngErrorcode = eGet(lngHandleB, LJ_ioGET_CONFIG, LJ_chSTREAM_BACKLOG_COMM, &dblCommBacklog, 0);
		printf("Comm Backlog B = %.0f\n",dblCommBacklog);
	}

   
    //Stop the stream
    lngErrorcode = eGet(lngHandleA, LJ_ioSTOP_STREAM, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
	lngErrorcode = eGet(lngHandleB, LJ_ioSTOP_STREAM, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);


	printf("\nDone");
	getchar();
	return 0;

}
