//---------------------------------------------------------------------------
//
//  simple.cpp
//  
//  Basic command/response UD U6 example.
//
//	For testing timers, please attach a jumper to FIO0 and tap GND
//	
//  support@labjack.com
//  Feb 2, 2006
//	Revised December 27, 2010
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
	long lngIOType=0, lngChannel=0;
	double dblValue=0;
	double Value2=0,Value3=0;
	double ValueDIBit=0,ValueDIPort=0,ValueCounter=0;
	LJ_HANDLE lngHandle=0;
	long lngGetNextIteration;
	char ch;


	//Open the first found LabJack U6.
	lngErrorcode = OpenLabJack (LJ_dtU6, LJ_ctUSB, "1", 1, &lngHandle);
	

	//First some configuration commands.  These will be done with the ePut
	//function which combines the add/go/get into a single call.

	//Configure resolution of the analog inputs (pass a non-zero value for quick sampling). 
	//See section 2.6 / 3.1 for more information.
	lngErrorcode = ePut (lngHandle, LJ_ioPUT_CONFIG, LJ_chAIN_RESOLUTION, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Configure the analog input range on channels 2 and 3 for bipolar gain=1.
	lngErrorcode = ePut (lngHandle,  LJ_ioPUT_AIN_RANGE, 2, LJ_rgBIP10V, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	lngErrorcode = ePut (lngHandle,  LJ_ioPUT_AIN_RANGE, 3, LJ_rgBIP10V, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Enable Counter0 which will appear on FIO0 (assuming no other
	//program has enabled any timers or Counter1).
	lngErrorcode = ePut (lngHandle,  LJ_ioPUT_COUNTER_ENABLE, 0, 1, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);


	//Now we add requests to write and read I/O.  These requests
	//will be processed repeatedly by go/get statements in every
	//iteration of the while loop below.

	//Request AIN2 and AIN3.
	lngErrorcode = AddRequest (lngHandle, LJ_ioGET_AIN, 2, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	lngErrorcode = AddRequest (lngHandle, LJ_ioGET_AIN, 3, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Set DAC0 to 2.5 volts.
	lngErrorcode = AddRequest (lngHandle, LJ_ioPUT_DAC, 0, 2.5, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Read digital input FIO1.
	lngErrorcode = AddRequest (lngHandle, LJ_ioGET_DIGITAL_BIT, 1, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Set digital output FIO2 to output-high.
	lngErrorcode = AddRequest (lngHandle, LJ_ioPUT_DIGITAL_BIT, 2, 1, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Read digital inputs FIO3 through FIO7.
	lngErrorcode = AddRequest (lngHandle, LJ_ioGET_DIGITAL_PORT, 3, 0, 5, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Request the value of Counter0.
	lngErrorcode = AddRequest (lngHandle, LJ_ioGET_COUNTER, 0, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);


	while (1)
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
				switch(lngChannel)
				{
					case 2:
						Value2=dblValue;
						break;
					case 3:
						Value3=dblValue;
						break;
				}
				break;

			case LJ_ioGET_DIGITAL_BIT :
				ValueDIBit=dblValue;
				break;

			case LJ_ioGET_DIGITAL_PORT :
				ValueDIPort=dblValue;
				break;

			case LJ_ioGET_COUNTER :
				ValueCounter=dblValue;
				break;

			}

			lngErrorcode = GetNextResult(lngHandle, &lngIOType, &lngChannel, &dblValue, 0, 0);
			if(lngErrorcode != LJE_NO_MORE_DATA_AVAILABLE)
			{
				ErrorHandler(lngErrorcode, __LINE__, lngGetNextIteration);
			}

			lngGetNextIteration++;

		}


		printf("AIN2 = %f\n",Value2);
  		printf("AIN3 = %f\n",Value3);
		printf("FIO1 = %f\n",ValueDIBit);
		printf("FIO3-FIO7 = %f\n",ValueDIPort);  //Will read 31 if all 5 lines are pulled-high as normal.
		printf("Counter0 (FIO0) = %f\n",ValueCounter);

		printf("\nPress Enter to go again or (q) to quit\n");
		ch = getchar();
		if (ch == 'q') return 0;
	}


}

