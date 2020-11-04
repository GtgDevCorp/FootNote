//---------------------------------------------------------------------------
//
//  Quadrature.cpp
//  
//	Enables quadrature input on FIO0/FIO1 and reads the current count
//	every half second, until a key is pressed.  To make something happen
//	without an actual quadrature signal, connect a couple wires to
//	ground and tap them on FIO0 and FIO1.
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
	long lngIOType=0, lngChannel=0;
	double dblValue=0;
	LJ_HANDLE lngHandle=0;


	//Open the LabJack UE9.
	lngErrorcode = OpenLabJack (LJ_dtUE9, LJ_ctUSB, "1", 1, &lngHandle);
	//lngErrorcode = OpenLabJack (LJ_dtUE9, LJ_ctETHERNET, "192.168.1.209", 0, &lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);


	//Disable all timers and counters to put everything in a known initial state.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chNUMBER_TIMERS_ENABLED, 0, 0, 0);
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_COUNTER_ENABLE, 0, 0, 0, 0);
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_COUNTER_ENABLE, 1, 0, 0, 0);
	lngErrorcode = GoOne(lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);


	//First enable the quadrature input.

	//Enable 2 timers for phases A and B.  They will use FIO0 and FIO1.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chNUMBER_TIMERS_ENABLED, 2, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Configure Timer0 as quadrature.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_MODE, 0, LJ_tmQUAD, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Configure Timer1 as quadrature.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_MODE, 1, LJ_tmQUAD, 0, 0);
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



	while(!kbhit())	//Program will run until any key is hit
	{

		//Wait 500 milliseconds
		Sleep(500);

		//Request a read from Timer0.  Timer0 and Timer1 both return the same
		//quadrature value.
		lngErrorcode = eGet(lngHandle, LJ_ioGET_TIMER, 0, &dblValue, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);
		printf("Quad Counter = %.1f\n",dblValue);

	}


	//Disable the timers and the FIO lines will return to digital I/O.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chNUMBER_TIMERS_ENABLED, 0, 0, 0);
	lngErrorcode = GoOne(lngHandle);

	getchar();
	return 0;


}
