//---------------------------------------------------------------------------
//
//  U3simple.cpp
//  
//  Basic command/response U3 example using the UD driver.
//
//  support@labjack.com
//  Jul 21, 2008
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
	long lngGetNextIteration;
	double dblDriverVersion;
	long lngIOType=0, lngChannel=0;
	double dblValue=0;
	double Value0=9999,Value1=9999,Value2=9999;
	double ValueDIBit=9999,ValueDIPort=9999,ValueCounter=9999;
	LJ_HANDLE lngHandle=0;
	char ch;


	//Read and display the UD version.
	dblDriverVersion = GetDriverVersion();
	printf("UD Driver Version = %.3f\n\n",dblDriverVersion);


	//Open the first found LabJack U3.
	lngErrorcode = OpenLabJack (LJ_dtU3, LJ_ctUSB, "1", 1, &lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Start by using the pin_configuration_reset IOType so that all
	//pin assignments are in the factory default condition.
	lngErrorcode = ePut (lngHandle, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);


	//First some configuration commands.  These will be done with the ePut
	//function which combines the add/go/get into a single call.

	//Configure FIO0-FIO3 as analog, all else as digital.  That means we
	//will start from channel 0 and update all 16 flexible bits.  We will
	//pass a value of b0000000000001111 or d15.
	lngErrorcode = ePut (lngHandle, LJ_ioPUT_ANALOG_ENABLE_PORT, 0, 15, 16);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Set the timer/counter pin offset to 7, which will put the first
	//timer/counter on FIO7.
	lngErrorcode = ePut (lngHandle,  LJ_ioPUT_CONFIG, LJ_chTIMER_COUNTER_PIN_OFFSET, 7, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Enable Counter1 (FIO7).
	lngErrorcode = ePut (lngHandle,  LJ_ioPUT_COUNTER_ENABLE, 1, 1, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);


	//The following commands will use the add-go-get method to group
	//multiple requests into a single low-level function.

	//Request a single-ended reading from AIN0.
	lngErrorcode = AddRequest (lngHandle, LJ_ioGET_AIN, 0, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Request a single-ended reading from AIN1.
	lngErrorcode = AddRequest (lngHandle, LJ_ioGET_AIN, 1, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Request a reading from AIN2 using the Special range.
	lngErrorcode = AddRequest (lngHandle, LJ_ioGET_AIN_DIFF, 2, 0, 32, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Set DAC0 to 3.5 volts.
	lngErrorcode = AddRequest (lngHandle, LJ_ioPUT_DAC, 0, 3.5, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Set digital output FIO4 to output-high.
	lngErrorcode = AddRequest (lngHandle, LJ_ioPUT_DIGITAL_BIT, 4, 1, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Read digital input FIO5.
	lngErrorcode = AddRequest (lngHandle, LJ_ioGET_DIGITAL_BIT, 5, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Read digital inputs FIO5 through FIO6.
	lngErrorcode = AddRequest (lngHandle, LJ_ioGET_DIGITAL_PORT, 5, 0, 2, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Request the value of Counter1 (FIO7).
	lngErrorcode = AddRequest (lngHandle, LJ_ioGET_COUNTER, 1, 0, 0, 0);
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
					case 0:
						Value0=dblValue;
						break;
					case 1:
						Value1=dblValue;
						break;
				}
				break;

			case LJ_ioGET_AIN_DIFF :
				Value2=dblValue;
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

		printf("AIN0 = %f\n",Value0);
  		printf("AIN1 = %f\n",Value1);
		printf("AIN2 = %f\n",Value2);
		printf("FIO5 = %f\n",ValueDIBit);
		printf("FIO5-FIO6 = %f\n",ValueDIPort);  //Will read 3 (binary 11) if both lines are pulled-high as normal.
		printf("Counter1 (FIO7) = %f\n",ValueCounter);

		printf("\nPress Enter to go again or (q) to quit\n");
		ch = getchar();
		if (ch == 'q') return 0;
	}


}
