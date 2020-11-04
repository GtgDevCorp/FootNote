//---------------------------------------------------------------------------
//
//  efunctions.cpp
//  
//  Demonstrates the UD E-functions with the LabJack U3.  For timer/counter
//  testing, connect FIO4 to FIO5 and FIO6.
//
//  support@labjack.com
//  Jul 22, 2008
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
	long alngEnableTimers[2], alngEnableCounters[2], lngTCPinOffset;
	long lngTimerClockBaseIndex, lngTimerClockDivisor, alngTimerModes[2];
	double adblTimerValues[2];
	long alngReadTimers[2], alngUpdateResetTimers[2], alngReadCounters[2], alngResetCounters[2];
	double adblCounterValues[2] = {0,0};
	double highTime, lowTime, dutyCycle;


	//Open the first found LabJack U3.
	lngErrorcode = OpenLabJack (LJ_dtU3, LJ_ctUSB, "1", 1, &lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Start by using the pin_configuration_reset IOType so that all
	//pin assignments are in the factory default condition.
	lngErrorcode = ePut (lngHandle, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Take a single-ended measurement from AIN3.
	lngBinary = 0;
	lngErrorcode = eAIN (lngHandle, 3, 31, &dblValue, -1, -1, -1, lngBinary, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	printf("AIN3 = %f\n",dblValue);

	//Set DAC0 to 3.0 volts.
	dblValue = 3.0;
	lngBinary = 0;
	lngErrorcode = eDAC (lngHandle, 0, dblValue, lngBinary, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	printf("DAC0 set to %f volts\n",dblValue);

	//Read state of FIO4.
	lngErrorcode = eDI (lngHandle, 4, &lngValue);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	printf("FIO4 = %d\n",lngValue);

	//Set the state of FIO7.
	lngValue = 1;
	lngErrorcode = eDO (lngHandle, 7, lngValue);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	printf("FIO7 set to = %d\n\n",lngValue);

	//Timers and Counters example.
	//First, a call to eTCConfig.  Fill the arrays with the desired values, then make the call.
	alngEnableTimers[0] = 1; //Enable Timer0 (uses FIO4).
	alngEnableTimers[1] = 1; //Enable Timer1 (uses FIO5).
	alngEnableCounters[0] = 0; //Disable Counter0.
	alngEnableCounters[1] = 1; //Enable Counter1 (uses FIO6).
	lngTCPinOffset = 4;  //Offset is 4, so timers/counters start at FIO4.
	lngTimerClockBaseIndex = LJ_tc48MHZ_DIV;  //Base clock is 48 MHz with divisor support, so Counter0 is not available.
	//lngTimerClockBaseIndex = LJ_tc24MHZ_DIV;  //Use this line instead for hardware rev 1.20.
	lngTimerClockDivisor = 48; //Thus timer clock is 1 MHz.
	//lngTimerClockDivisor = 24;  //Use this line instead for hardware rev 1.20.
	alngTimerModes[0] = LJ_tmPWM8; //Timer0 is 8-bit PWM output.  Frequency is 1M/256 = 3906 Hz.
	alngTimerModes[1] = LJ_tmDUTYCYCLE; //Timer1 is duty cyle input.
	adblTimerValues[0] = 16384; //Set PWM8 duty-cycle to 75%.
	adblTimerValues[1] = 0;
	lngErrorcode = eTCConfig(lngHandle, alngEnableTimers, alngEnableCounters, lngTCPinOffset, lngTimerClockBaseIndex, lngTimerClockDivisor, alngTimerModes, adblTimerValues, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	printf("Timers and Counters enabled.\n\n");

	Sleep(1000); //Wait 1 second.

	//Now, a call to eTCValues.
	alngReadTimers[0] = 0; //Don't read Timer0 (output timer).
	alngReadTimers[1] = 1; //Read Timer1;
	alngUpdateResetTimers[0] = 1; //Update Timer0;
	alngUpdateResetTimers[1] = 1; //Reset Timer1;
	alngReadCounters[0] = 0;
	alngReadCounters[1] = 1; //Read Counter1;
	alngResetCounters[0] = 0;
	alngResetCounters[1] = 1; //Reset Counter1.
	adblTimerValues[0] = 32768; //Change Timer0 duty-cycle to 50%.
	adblTimerValues[1] = 0;
	lngErrorcode = eTCValues(lngHandle, alngReadTimers, alngUpdateResetTimers, alngReadCounters, alngResetCounters, adblTimerValues, adblCounterValues, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	printf("Timer1 value = %.3f\n",adblTimerValues[1]);
	printf("Counter1 value = %.3f\n",adblCounterValues[1]);

	//Convert Timer1 value to duty-cycle percentage.
	//High time is LSW
	highTime = (double)(((unsigned long)adblTimerValues[1]) % (65536));
	//Low time is MSW
	lowTime = (double)(((unsigned long)adblTimerValues[1]) / (65536));
	//Calculate the duty cycle percentage.
	dutyCycle = 100*highTime/(highTime+lowTime);
	printf("\nHigh clicks Timer1 = %.1f\n",highTime);
	printf("Low clicks Timer1 = %.1f\n",lowTime);
	printf("Duty cycle Timer1 = %.1f\n",dutyCycle);


	//Disable all timers and counters.
	alngEnableTimers[0] = 0;
	alngEnableTimers[1] = 0;
	alngEnableCounters[0] = 0;
	alngEnableCounters[1] = 0;
	lngErrorcode = eTCConfig(lngHandle, alngEnableTimers, alngEnableCounters, 4, lngTimerClockBaseIndex, lngTimerClockDivisor, alngTimerModes, adblTimerValues, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);



}

