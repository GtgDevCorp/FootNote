//---------------------------------------------------------------------------
//
//  efunctions.cpp
//  
//  Demonstrates the UD E-functions with the LabJack U6.
//
//  support@labjack.com
//  June 5, 2009
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




void main()
{
	LJ_ERROR lngErrorcode;
	//long lngIOType=0, lngChannel=0;
	double dblValue=0;
	long lngValue=0;
	LJ_HANDLE lngHandle=0;
	//long lngGetNextIteration;
	//char ch;

	long lngBinary;


	//Open the first found LabJack U6.
	lngErrorcode = OpenLabJack (LJ_dtU6, LJ_ctUSB, "1", 1, &lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Take a single-ended measurement from AIN3.
	lngBinary = 0;
	lngErrorcode = eAIN (lngHandle, 3, 15, &dblValue, LJ_rgAUTO, 0, 0, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	printf("AIN3 = %f\n",dblValue);

	//Set DAC0 to 3.0 volts.
	dblValue = 3.0;
	lngBinary = 0;
	lngErrorcode = eDAC (lngHandle, 0, dblValue, lngBinary, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	printf("DAC0 set to %f volts\n",dblValue);

	//Read state of FIO0.
	lngErrorcode = eDI (lngHandle, 0, &lngValue);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	printf("FIO0 = %d\n",lngValue);

	//Set the state of FIO1.
	lngValue = 1;
	lngErrorcode = eDO (lngHandle, 1, lngValue);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	printf("FIO1 set to = %d\n\n",lngValue);

	// Pause for the user
	getchar();

}

