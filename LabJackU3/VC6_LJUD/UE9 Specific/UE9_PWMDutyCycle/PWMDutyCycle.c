//---------------------------------------------------------------------------
//
//  PWMDutyCycle.c
//  
//	Demonstrates 1 PWM output and 2 duty cycle inputs.
//	Connect FIO0 to FIO1 to FIO2.
//
//  support@labjack.com
//  Feb 22, 2006
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



//Function used to read and display the duty cycle from a timer.
void ReadDutyCycle (LJ_HANDLE lngHandle, long timerNumber)
{
	LJ_ERROR lngErrorcode;
	double dblValue=0;
	double highTime,lowTime,dutyCycle;

	//Read from Timer.  We will go ahead and reset the timer (by writing
	//a value of 0) at the same time as the read.  This way if no new
	//edges occur (e.g. 0% or 100% duty cycle) the next read will return
	//the preset high/low times (0/65535 or 65535/0) rather than returning
	//the old values.
	lngErrorcode = AddRequest(lngHandle, LJ_ioGET_TIMER, timerNumber, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_VALUE, timerNumber, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	lngErrorcode = GoOne(lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	lngErrorcode = GetResult(lngHandle, LJ_ioPUT_TIMER_VALUE, timerNumber, 0);  //just to check for error
	ErrorHandler(lngErrorcode, __LINE__, 0);
	lngErrorcode = GetResult(lngHandle, LJ_ioGET_TIMER, timerNumber, &dblValue);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	//High time is LSW
	highTime = (double)(((unsigned long)dblValue) % (65536));
	//Low time is MSW
	lowTime = (double)(((unsigned long)dblValue) / (65536));
	//Calculate the duty cycle percentage.
	dutyCycle = 100*highTime/(highTime+lowTime);

	printf("\nHigh clicks Timer%d = %.1f\n",timerNumber,highTime);
	printf("Low clicks Timer%d = %.1f\n",timerNumber,lowTime);
	printf("Duty cycle Timer%d = %.1f\n",timerNumber,dutyCycle);
}




main()
{
	LJ_ERROR lngErrorcode;
	long lngGetNextIteration;
	long lngIOType=0, lngChannel=0;
	double dblValue=0;
	LJ_HANDLE lngHandle=0;
	long resetCounter=1;
	double count=-1;


	//Open the LabJack UE9.
	lngErrorcode = OpenLabJack (LJ_dtUE9, LJ_ctUSB, "1", 1, &lngHandle);
	//errorcode = OpenLabJack (LJ_dtUE9, LJ_ctETHERNET, "192.168.1.209", 0, &Handle);
	ErrorHandler(lngErrorcode, __LINE__, 0);


	//Disable all timers and counters to put everything in a known initial state.
	//Disable the timer and counter, and the FIO lines will return to digital I/O.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chNUMBER_TIMERS_ENABLED, 0, 0, 0);
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_COUNTER_ENABLE, 0, 0, 0, 0);
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_COUNTER_ENABLE, 1, 0, 0, 0);
	lngErrorcode = GoOne(lngHandle);


	//Output a PWM output on Timer0 (FIO0) and measure
	//the duty cycle on Timer1 FIO1 and Timer2 FIO2.

    //Use the fixed 750kHz timer clock source.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_BASE, LJ_tc750KHZ, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Set the divisor to 3 so the actual timer clock is 250kHz.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_DIVISOR, 3, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Enable 2 timers.  They will use FIO0 and FIO1.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chNUMBER_TIMERS_ENABLED, 3, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Configure Timer0 as 8-bit PWM.  Frequency will be 250k/256 = 977 Hz.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_MODE, 0, LJ_tmPWM8, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Set the PWM duty cycle to 50%.  The passed value is the low time.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_VALUE, 0, 32768, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Configure Timer1 and Timer2 as duty cycle measurement.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_MODE, 1, LJ_tmDUTYCYCLE, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_MODE, 2, LJ_tmDUTYCYCLE, 0, 0);
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



	//Set the PWM duty cycle to 25%.  The passed value is the low time.
	lngErrorcode = ePut(lngHandle, LJ_ioPUT_TIMER_VALUE, 0, 49152, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	//Now we will reset the duty cycle input timers, so we are sure the
	//reads we do are not old values from before the PWM output was updated.
	Sleep(10);
	lngErrorcode = ePut(lngHandle, LJ_ioPUT_TIMER_VALUE, 1, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	lngErrorcode = ePut(lngHandle, LJ_ioPUT_TIMER_VALUE, 2, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Wait a little so we are sure a duty cycle measurement has occured.
	Sleep(10);

	//Read from Timer1.
	ReadDutyCycle (lngHandle, 1);

	//Read from Timer2.
	ReadDutyCycle (lngHandle, 2);




	//Set the PWM duty cycle to 0%.  The passed value is the low time.
	//We are specifying 65535 out of 65536 clicks to be low.  Since
	//this is 8-bit PWM, we actually get 255 low clicks out of 256 total
	//clicks, so the minimum duty cycle is 0.4%.
	lngErrorcode = ePut(lngHandle, LJ_ioPUT_TIMER_VALUE, 0, 65535, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	//Now we will reset the duty cycle input timers, so we are sure the
	//reads we do are not old values from before the PWM output was updated.
	Sleep(10);
	lngErrorcode = ePut(lngHandle, LJ_ioPUT_TIMER_VALUE, 1, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	lngErrorcode = ePut(lngHandle, LJ_ioPUT_TIMER_VALUE, 2, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Wait a little so we are sure a duty cycle measurement has occured.
	Sleep(10);

	//Read from Timer1.
	ReadDutyCycle (lngHandle, 1);

	//Read from Timer2.
	ReadDutyCycle (lngHandle, 2);




	//Set the PWM duty cycle to 100%.  The passed value is the low time.
	//We are specifying 0 out of 65536 clicks to be low, so the signal
	//will be high the entire time, meaning there are no edges
	//for the input timers to detect, and no measurement should be made.
	lngErrorcode = ePut(lngHandle, LJ_ioPUT_TIMER_VALUE, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	//Now we will reset the duty cycle input timers, so we are sure the
	//reads we do are not old values from before the PWM output was updated.
	Sleep(10);
	lngErrorcode = ePut(lngHandle, LJ_ioPUT_TIMER_VALUE, 1, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	lngErrorcode = ePut(lngHandle, LJ_ioPUT_TIMER_VALUE, 2, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Wait a little so we are sure a duty cycle measurement has been attempted.
	Sleep(10);

	//Read from Timer1.
	ReadDutyCycle (lngHandle, 1);

	//Read from Timer2.
	ReadDutyCycle (lngHandle, 2);




	//Disable all timers and counters, and the FIO lines will return to digital I/O.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chNUMBER_TIMERS_ENABLED, 0, 0, 0);
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_COUNTER_ENABLE, 0, 0, 0, 0);
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_COUNTER_ENABLE, 1, 0, 0, 0);
	lngErrorcode = GoOne(lngHandle);


	getchar();
	return 0;

}
