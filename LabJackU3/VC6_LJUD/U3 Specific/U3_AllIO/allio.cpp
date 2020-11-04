//---------------------------------------------------------------------------
//
//  allio.cpp
// 
//	Demonstrates using the add/go/get method to efficiently write and read
//	some or all analog and digital I/O on the LabJack U3.
//	Records the time for 1000 iterations and divides by 1000, to allow
//	verification of the basic command/response communication times of the
//	LabJack U3 as documented in Section 3.1 of the U3 User's Guide.
//
//  support@labjack.com
//  Jul 21, 2008
//----------------------------------------------------------------------
//

#include <stdio.h>
#include <windows.h>
#include <math.h>
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
	char ch;

	long lngIOType=0, lngChannel=0;
	double dblValue=0;
	double ValueDIPort=0;
	double ValueAIN[16]={9999};
	

	long time=0, i=0, j=0;
	long numIterations = 100;
	long numChannels = 16;  //Number of AIN channels, 0-16.
	long quickSample = 0;  //Set to TRUE for quick AIN sampling.
	long longSettling = 1;  //Set to TRUE for extra AIN settling time.


	//Open the first found LabJack.
	lngErrorcode = OpenLabJack (LJ_dtU3, LJ_ctUSB, "1", 1, &lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Start by using the pin_configuration_reset IOType so that all
	//pin assignments are in the factory default condition.
	lngErrorcode = ePut (lngHandle, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Configure quickSample.
	lngErrorcode = ePut (lngHandle, LJ_ioPUT_CONFIG, LJ_chAIN_RESOLUTION, quickSample, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Configure longSettling.
	lngErrorcode = ePut (lngHandle, LJ_ioPUT_CONFIG, LJ_chAIN_SETTLING_TIME, longSettling, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	
	//Configure the necessary lines as analog.
	lngErrorcode = ePut (lngHandle, LJ_ioPUT_ANALOG_ENABLE_PORT, 0, pow(2,numChannels)-1, numChannels);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	
	//Now an Add/Go/Get block to configure the timers and counters.  These
	//are configured on EIO0-EIO3, so if more than 8 analog inputs are
	//enabled then the analog inputs use these lines.
	if(numChannels <= 8)
	{
		//Set the timer/counter pin offset to 8, which will put the first
		//timer/counter on EIO0.
		lngErrorcode = AddRequest (lngHandle,  LJ_ioPUT_CONFIG, LJ_chTIMER_COUNTER_PIN_OFFSET, 8, 0, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);

		//Use the default clock source.
		lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_BASE, LJ_tc48MHZ, 0, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);

		//Enable 2 timers.
		lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chNUMBER_TIMERS_ENABLED, 2, 0, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);

		//Configure Timer0 as 8-bit PWM.
		lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_MODE, 0, LJ_tmPWM8, 0, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);

		//Set the PWM duty cycle to 50%.
		lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_VALUE, 0, 32768, 0, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);

		//Configure Timer1 as 8-bit PWM.
		lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_MODE, 1, LJ_tmPWM8, 0, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);

		//Set the PWM duty cycle to 50%.
		lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_VALUE, 1, 32768, 0, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);

		//Enable Counter0.
		lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_COUNTER_ENABLE, 0, 1, 0, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);

		lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_COUNTER_ENABLE, 1, 1, 0, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);

		//Execute the requests.
		lngErrorcode = GoOne (lngHandle);
		ErrorHandler(lngErrorcode, __LINE__, 0);
	}



	//Now add requests that will be processed every iteration of the loop.

	//Add analog input requests.
	for(j=0;j<numChannels;j++)
	{
		lngErrorcode = AddRequest (lngHandle, LJ_ioGET_AIN, j, 0, 0, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);
	}

	//Set DAC0 to 2.5 volts.
	lngErrorcode = AddRequest (lngHandle, LJ_ioPUT_DAC, 0, 2.5, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Read CIO digital lines.
	lngErrorcode = AddRequest (lngHandle, LJ_ioGET_DIGITAL_PORT, 16, 0, 4, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Only do the timer/counter stuff if there are less than 8 analog inputs.
	if(numChannels <= 8)
	{
		lngErrorcode = AddRequest (lngHandle, LJ_ioGET_COUNTER, 0, 0, 0, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);

		lngErrorcode = AddRequest (lngHandle, LJ_ioGET_COUNTER, 1, 0, 0, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);

		lngErrorcode = AddRequest (lngHandle, LJ_ioGET_TIMER, 0, 0, 0, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);

		lngErrorcode = AddRequest (lngHandle, LJ_ioGET_TIMER, 1, 0, 0, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);

		//Set the PWM duty cycle to 50%.
		lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_VALUE, 0, 32768, 0, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);

		//Set the PWM duty cycle to 50%.
		lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_VALUE, 1, 32768, 0, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);
	}


	time = GetTickCount();	

	for(i=0;i<numIterations;i++)
	{

		//Execute the requests.
		lngErrorcode = GoOne (lngHandle);
		ErrorHandler(lngErrorcode, __LINE__, 0);

	   	//Get all the results.  The input measurement results are stored.  All other
		//results are for configuration or output requests so we are just checking
		//whether there was an error.
		lngErrorcode = GetFirstResult(lngHandle, &lngIOType, &lngChannel, &dblValue, 0, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);
		lngGetNextIteration=0;	//Used by the error handling function.
		while(lngErrorcode < LJE_MIN_GROUP_ERROR)
		{
			switch(lngIOType)
			{

			case LJ_ioGET_AIN :
				ValueAIN[lngChannel]=dblValue;
				break;

			case LJ_ioGET_DIGITAL_PORT :
				ValueDIPort=dblValue;
				break;
			}

			lngErrorcode = GetNextResult(lngHandle, &lngIOType, &lngChannel, &dblValue, 0, 0);
			if(lngErrorcode != LJE_NO_MORE_DATA_AVAILABLE)
			{
				ErrorHandler(lngErrorcode, __LINE__, lngGetNextIteration);
			}

			lngGetNextIteration++;

		}



	}


	time = GetTickCount() - time;

	printf("Milleseconds per iteration = %.3f\n", (double)time / (double)numIterations);

	printf("\nDigital Input = %f\n",ValueDIPort);

	printf("\nAIN readings from last iteration:\n");
	for(j=0;j<numChannels;j++)
	{
			printf("%.3f\n", ValueAIN[j]);
	}

	ch = getchar();
	return 0;

}
