//---------------------------------------------------------------------------
//
//  simplestream.c
//  
//  2-channel stream on the U3.
//
//  support@labjack.com
//  Jul 21, 2008
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
	double scanRate = 1000;
	long delayms = 1000;
	double numScans = 2000;  //2x the expected # of scans (2*scanRate*delayms/1000)
	double numScansRequested;
	double adblData[4000] = {0};  //Max buffer size (#channels*numScansRequested)


	//Make a long parameter which holds the address of the data array.  We do this
	//so the compiler does not generate a warning in the eGet call that retrieves
	//stream data.  Note that the x1 parameter  in eGet (and AddRequest) is fairly
	//generic, in that sometimes it could just be a write parameter, and sometimes
	//it has the address of an array.  Since x1 is not declared as a pointer, the
	//compiler will complain if you just pass the array pointer without casting
	//it to a long as follows.
	long padblData = (long)&adblData[0];

	//Read and display the UD version.
	dblValue = GetDriverVersion();
	printf("UD Driver Version = %.3f\n\n",dblValue);

	//Open the first found LabJack U3.
	lngErrorcode = OpenLabJack (LJ_dtU3, LJ_ctUSB, "1", 1, &lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Read and display the hardware version of this U3.
	lngErrorcode = eGet (lngHandle, LJ_ioGET_CONFIG, LJ_chHARDWARE_VERSION, &dblValue, 0);
	printf("U3 Hardware Version = %.3f\n\n",dblValue);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Read and display the firmware version of this U3.
	lngErrorcode = eGet (lngHandle, LJ_ioGET_CONFIG, LJ_chFIRMWARE_VERSION, &dblValue, 0);
	printf("U3 Firmware Version = %.3f\n\n",dblValue);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Start by using the pin_configuration_reset IOType so that all
	//pin assignments are in the factory default condition.
	lngErrorcode = ePut (lngHandle, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Configure FIO0 and FIO1 as analog, all else as digital.  That means we
	//will start from channel 0 and update all 16 flexible bits.  We will
	//pass a value of b0000000000000011 or d3.
	lngErrorcode = ePut (lngHandle, LJ_ioPUT_ANALOG_ENABLE_PORT, 0, 3, 16);
	ErrorHandler(lngErrorcode, __LINE__, 0);


	//Configure the stream:
    //Set the scan rate.
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chSTREAM_SCAN_FREQUENCY, scanRate, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    //Give the driver a 5 second buffer (scanRate * 2 channels * 5 seconds).
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chSTREAM_BUFFER_SIZE, scanRate*2*5, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    //Configure reads to retrieve whatever data is available without waiting (wait mode LJ_swNONE).
	//See comments below to change this program to use LJ_swSLEEP mode.
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chSTREAM_WAIT_MODE, LJ_swNONE, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    //Define the scan list as AIN0 then AIN1.
    lngErrorcode = AddRequest(lngHandle, LJ_ioCLEAR_STREAM_CHANNELS, 0, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    lngErrorcode = AddRequest(lngHandle, LJ_ioADD_STREAM_CHANNEL, 0, 0, 0, 0); // first method for single ended reading - AIN0
    ErrorHandler(lngErrorcode, __LINE__, 0);
    lngErrorcode = AddRequest(lngHandle, LJ_ioADD_STREAM_CHANNEL_DIFF, 1, 0, 32, 0); // second method for single ended reading - AIN1
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
		//
		//To change this program to use sleep mode,
		//	-change numScans to the actual number of scans desired per read,
		//	-change wait mode addrequest value to LJ_swSLEEP,
		//	-comment out the following Sleep command.

		Sleep(delayms);	//Remove if using LJ_swSLEEP.

		//init array so we can easily tell if it has changed
		for(k=0;k<numScans*2;k++)
		{
			adblData[k] = 99999.0;
		}

		//Read the data.  We will request twice the number we expect, to
		//make sure we get everything that is available.
		//Note that the array we pass must be sized to hold enough SAMPLES, and
		//the Value we pass specifies the number of SCANS to read.
		numScansRequested=numScans;
		lngErrorcode = eGet(lngHandle, LJ_ioGET_STREAM_DATA, LJ_chALL_CHANNELS, &numScansRequested, padblData);
		//The displays the number of scans that were actually read.
		printf("\nIteration # %d\n",i);
		printf("Number read = %.0f\n",numScansRequested);
		//This displays just the first scan.
		printf("First scan = %.3f, %.3f\n",adblData[0],adblData[1]);
		ErrorHandler(lngErrorcode, __LINE__, 0);
		//Retrieve the current backlog.  The UD driver retrieves stream data from
		//the U3 in the background, but if the computer is too slow for some reason
		//the driver might not be able to read the data as fast as the U3 is
		//acquiring it, and thus there will be data left over in the U3 buffer.
		lngErrorcode = eGet(lngHandle, LJ_ioGET_CONFIG, LJ_chSTREAM_BACKLOG_COMM, &dblCommBacklog, 0);
		printf("Comm Backlog = %.0f\n",dblCommBacklog);
		i++;
	}

   
    //Stop the stream
    lngErrorcode = eGet(lngHandle, LJ_ioSTOP_STREAM, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);


	printf("\nDone");
	getchar();
	return 0;

}
