//---------------------------------------------------------------------------
//
//  U3timercounter.cpp
//  
//  Basic U3 example does a PWM output and a counter input, using AddGoGet method.
//
//  support@labjack.com
//  Jul 21, 2008
//----------------------------------------------------------------------
//

#include <stdio.h>
#include <windows.h>
#include "E:\PROJECTS\UPW\Slepian\FootNote\test1\LabJackUD.h"
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
	long lngGetNextIteration;
	long lngIOType=0, lngChannel=0;
	double dblValue=0;
	LJ_HANDLE lngHandle=0;


	//Open the first found LabJack U3.
	lngErrorcode = OpenLabJack (LJ_dtU3, LJ_ctUSB, "1", 1, &lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Start by using the pin_configuration_reset IOType so that all
	//pin assignments are in the factory default condition.
	lngErrorcode = ePut (lngHandle, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);


	//First requests to configure the timer and counter.  These will be
	//done with and add/go/get block.

	//Set the timer/counter pin offset to 4, which will put the first
	//timer/counter on FIO4.
	lngErrorcode = AddRequest (lngHandle,  LJ_ioPUT_CONFIG, LJ_chTIMER_COUNTER_PIN_OFFSET, 4, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

    //Use the 48 MHz timer clock base with divider.  Since we are using clock with divisor
	//support, Counter0 is not available.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_BASE, LJ_tc48MHZ_DIV, 0, 0);
	//lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_BASE, LJ_tc24MHZ_DIV, 0, 0);  //Use this line instead for hardware rev 1.20.
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Set the divisor to 48 so the actual timer clock is 1 MHz.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_DIVISOR, 48, 0, 0);
	//lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_DIVISOR, 24, 0, 0);  //Use this line instead for hardware rev 1.20.
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Enable 1 timer.  It will use FIO4.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chNUMBER_TIMERS_ENABLED, 1, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Configure Timer0 as 8-bit PWM.  Frequency will be 1M/256 = 3906 Hz.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_MODE, 0, LJ_tmPWM8, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Set the PWM duty cycle to 50%.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_VALUE, 0, 32768, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Enable Counter1.  It will use FIO5 since 1 timer is enabled.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_COUNTER_ENABLE, 1, 1, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Execute the requests.
	lngErrorcode = GoOne (lngHandle);
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


	//Wait 1 second.
	Sleep(1000);

	//Request a read from the counter.
	lngErrorcode = eGet(lngHandle, LJ_ioGET_COUNTER, 1, &dblValue, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//This should read roughly 4k counts if FIO4 is shorted to FIO5.
	printf("Counter = %.1f\n",dblValue);

	//Wait 1 second.
	Sleep(1000);

	//Request a read from the counter.
	lngErrorcode = eGet(lngHandle, LJ_ioGET_COUNTER, 1, &dblValue, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//This should read about 3906 counts more than the previous read.
	printf("Counter = %.1f\n",dblValue);

	//Reset all pin assignments to factory default condition.
	lngErrorcode = ePut (lngHandle, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//The PWM output sets FIO4 to output, so we do a read here to set
	//it to input.
	lngErrorcode = eGet (lngHandle, LJ_ioGET_DIGITAL_BIT, 4, &dblValue, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	
}
