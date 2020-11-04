//---------------------------------------------------------------------------
//
//  TimerCounter.cpp
//  
//	Demonstrates a few of different timer/counter features.
//
//  support@labjack.com
//  Oct 30, 2006
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





void main()
{
	LJ_ERROR lngErrorcode;
	long lngGetNextIteration;
	long lngIOType=0, lngChannel=0;
	double dblValue=0;
	LJ_HANDLE lngHandle=0;
	long resetCounter=1;
	double count=-1;
	double highTime,lowTime;
	double period16,period32;


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


	//First we will output a square wave and count the number of pulses for about 1 second.
	//Connect a jumper on the UE9 from FIO0 (PWM output) to
	//FIO1 (Counter0 input).

    //Use the fixed 750kHz timer clock source.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_BASE, LJ_tc750KHZ, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Set the divisor to 3 so the actual timer clock is 250kHz.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_DIVISOR, 3, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Enable 1 timer.  It will use FIO0.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chNUMBER_TIMERS_ENABLED, 1, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Configure Timer0 as 8-bit PWM.  Frequency will be 250k/256 = 977 Hz.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_MODE, 0, LJ_tmPWM8, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Set the PWM duty cycle to 50%.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_VALUE, 0, 32768, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Enable Counter0.  It will use FIO1 since 1 timer is enabled.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_COUNTER_ENABLE, 0, 1, 0, 0);
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


	//Wait 1 second.
	Sleep(1000);

	//Request a read from the counter.
	lngErrorcode = eGet(lngHandle, LJ_ioGET_COUNTER, 0, &dblValue, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//This should read roughly 977 counts.
	printf("Counter = %.1f\n\n",dblValue);

	//Disable the timer and counter, and the FIO lines will return to digital I/O.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chNUMBER_TIMERS_ENABLED, 0, 0, 0);
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_COUNTER_ENABLE, 0, 0, 0, 0);
	lngErrorcode = GoOne(lngHandle);




	//Output a square wave and measure the period.
	//Connect a jumper on the UE9 from FIO0 (PWM8 output) to
	//FIO1 (RISINGEDGES32 input) and FIO2 (RISINGEDGES16).

    //Use the fixed 750kHz timer clock source.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_BASE, LJ_tc750KHZ, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Set the divisor to 3 so the actual timer clock is 250kHz.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_DIVISOR, 3, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Enable 3 timers.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chNUMBER_TIMERS_ENABLED, 3, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Configure Timer0 as 8-bit PWM.  Frequency will be 250k/256 = 977 Hz.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_MODE, 0, LJ_tmPWM8, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Set the PWM duty cycle to 50%.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_VALUE, 0, 32768, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Configure Timer1 as 32-bit period measurement.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_MODE, 1, LJ_tmRISINGEDGES32, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Configure Timer2 as 16-bit period measurement.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_MODE, 2, LJ_tmRISINGEDGES16, 0, 0);
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


	//Wait 1 second.
	Sleep(1000);

	//Now read the period measurements from the 2 timers.  We
	//will use the Add/Go/Get method so that both
	//reads are done in a single low-level call.

	//Request a read from Timer1
	lngErrorcode = AddRequest(lngHandle, LJ_ioGET_TIMER, 1, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Request a read from Timer2
	lngErrorcode = AddRequest(lngHandle, LJ_ioGET_TIMER, 2, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Execute the requests on a single LabJack.  The driver will use a
    //single low-level TimerCounter command to handle all the requests above.
	lngErrorcode = GoOne(lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Get the results of the two read requests.
	lngErrorcode = GetFirstResult(lngHandle, &lngIOType, &lngChannel, &dblValue, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	lngGetNextIteration=0;	//Used by the error handling function.
	while(lngErrorcode < LJE_MIN_GROUP_ERROR)
	{
		switch(lngIOType)
		{
		case LJ_ioGET_TIMER :
			switch(lngChannel)
			{
				case 1:
					period32=dblValue;
					break;
				case 2:
					period16=dblValue;
					break;
			}
			break;
		}

		lngErrorcode = GetNextResult(lngHandle, &lngIOType, &lngChannel, &dblValue, 0, 0);
		if(lngErrorcode != LJE_NO_MORE_DATA_AVAILABLE)
		{
			ErrorHandler(lngErrorcode, __LINE__, lngGetNextIteration);
		}

		lngGetNextIteration++;

	}


	//Both period measurements should read about 256.  The timer
	//clock was set to 250 kHz, so each tick equals 4 microseconds, so
	//256 ticks means a period of 1024 microseconds which is a frequency
	//of 977 Hz.
	printf("Period32 = %.1f\n",period32);
	printf("Period16 = %.1f\n\n",period16);

	//Disable the timer and counter, and the FIO lines will return to digital I/O.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chNUMBER_TIMERS_ENABLED, 0, 0, 0);
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_COUNTER_ENABLE, 0, 0, 0, 0);
	lngErrorcode = GoOne(lngHandle);




	//Now we will output a 25% duty-cycle PWM output on Timer0 (FIO0) and measure
	//the duty cycle on Timer1 FIO1.  Requires Control firmware V1.21 or higher.

    //Use the fixed 750kHz timer clock source.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_BASE, LJ_tc750KHZ, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Set the divisor to 3 so the actual timer clock is 250kHz.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_DIVISOR, 3, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Enable 2 timers.  They will use FIO0 and FIO1.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chNUMBER_TIMERS_ENABLED, 2, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Configure Timer0 as 8-bit PWM.  Frequency will be 250k/256 = 977 Hz.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_MODE, 0, LJ_tmPWM8, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Set the PWM duty cycle to 25%.  The passed value is the low time.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_VALUE, 0, 49152, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Configure Timer1 as duty cycle measurement.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_MODE, 1, LJ_tmDUTYCYCLE, 0, 0);
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


	//Wait a little so we are sure a duty cycle measurement has occured.
	Sleep(100);

	//Request a read from Timer1.
	lngErrorcode = eGet(lngHandle, LJ_ioGET_TIMER, 1, &dblValue, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//High time is LSW
	highTime = (double)(((unsigned long)dblValue) % (65536));
	//Low time is MSW
	lowTime = (double)(((unsigned long)dblValue) / (65536));

	printf("High clicks = %.1f\n",highTime);
	printf("Low clicks = %.1f\n",lowTime);
	printf("Duty cycle = %.1f\n",100*highTime/(highTime+lowTime));


	//Disable the timers, and the FIO lines will return to digital I/O.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chNUMBER_TIMERS_ENABLED, 0, 0, 0);
	lngErrorcode = GoOne(lngHandle);


	//The PWM output sets FIO0 to output, so we do a read here to set
	//FIO0 to input.
	lngErrorcode = eGet (lngHandle, LJ_ioGET_DIGITAL_BIT, 0, &dblValue, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);




	getchar();
	return;

}
