//---------------------------------------------------------------------------
//
//  allio.cpp
// 
//	Demonstrates using the add/go/get method to efficiently write and read
//	virtually all analog and digital I/O on the LabJack UE9.
//	Records the time for 1000 iterations and divides by 1000, to allow
//	verification of the basic command/response communication times of the
//	LabJack UE9 as documented in Section 3.1 of the UE9 User's Guide.
//
//  support@labjack.com
//  Feb 2, 2006
//----------------------------------------------------------------------
//

#include <stdio.h>
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
	char ch;
	long lngGetNextIteration;

	long lngIOType=0, lngChannel=0;
	double dblValue=0;
	double ValueDIPort=0;
	double ValueAIN[16]={9999};
	

	long time=0, i=0, j=0;
	long numIterations = 1000;
	long numChannels = 5;  //Number of AIN channels, 0-16.
	long ainResolution = 12;


	//Open the first found LabJack UE9.
	lngErrorcode = OpenLabJack (LJ_dtUE9, LJ_ctUSB, "1", 1, &lngHandle);
	//lngErrorcode = OpenLabJack (LJ_dtUE9, LJ_ctETHERNET, "192.168.1.38", 0, &lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);


		//Set DAC0 to 2.5 volts.
	lngErrorcode = AddRequest (lngHandle, LJ_ioPUT_DAC, 0, 2.5, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Set DAC1 to 3.5 volts.
	lngErrorcode = AddRequest (lngHandle, LJ_ioPUT_DAC, 1, 3.5, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Write all digital I/O.  Doing a bunch of bit instructions, rather than
	//the following port instruction, should not make a noticable difference
	//in the overall execution time.
	lngErrorcode = AddRequest (lngHandle, LJ_ioPUT_DIGITAL_PORT, 0, 0, 23, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Configure the desired resolution.  Note that depending on resolution and
	//number of analog inputs, numIterations might need to be reduced from the
	//default above so the program does not take too long to execute.
	lngErrorcode = AddRequest (lngHandle, LJ_ioPUT_CONFIG, LJ_chAIN_RESOLUTION, ainResolution, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Add analog input requests.
	for(j=0;j<numChannels;j++)
	{
		lngErrorcode = AddRequest (lngHandle, LJ_ioGET_AIN, j, 0, 0, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);
	}

	//Request a read of all digital I/O.  Doing a bunch of bit instructions,
	//rather than the following port instruction, should not make a noticable
	//difference in the overall execution time.
	lngErrorcode = AddRequest (lngHandle, LJ_ioGET_DIGITAL_PORT, 0, 0, 23, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);


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
                                                                                                                                                            