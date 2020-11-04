//---------------------------------------------------------------------------
//
//  efunctions.cpp
//  
//  Demonstrates the UD E-functions with the LabJack UE9.
//
//  support@labjack.com
//  Jun 12, 2006
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

	long lngRange, lngResolution, lngBinary;
	long alngEnableTimers[6], alngEnableCounters[2], lngTimerClockBaseIndex, lngTimerClockDivisor, alngTimerModes[6];
	double adblTimerValues[6];
	long alngReadTimers[6], alngUpdateResetTimers[6], alngReadCounters[2], alngResetCounters[2];
	double adblCounterValues[2] = {0,0};
	double highTime, lowTime, dutyCycle;


	//Open the first found LabJack UE9.
	lngErrorcode = OpenLabJack (LJ_dtUE9, LJ_ctUSB, "1", 1, &lngHandle);
	//lngErrorcode = OpenLabJack (LJ_dtUE9, LJ_ctETHERNET, "192.168.1.209", 0, &lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);


	//Take a measurement from AIN3.
	lngRange = LJ_rgBIP5V;
	lngResolution = 17;
	lngBinary = 0;
	lngErrorcode = eAIN (lngHandle, 3, 0, &dblValue, lngRange, lngResolution, -1, lngBinary, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	printf("AIN3 = %f\n",dblValue);

	//Set DAC0 to 3.0 volts.
	dblValue = 3.0;
	lngBinary = 0;
	lngErrorcode = eDAC (lngHandle, 0, dblValue, lngBinary, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	printf("DAC0 set to %f volts\n",dblValue);

	//Read state of FIO2.
	lngErrorcode = eDI (lngHandle, 2, &lngValue);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	printf("FIO2 = %d\n",lngValue);

	//Set the state of FIO3.
	lngValue = 0;
	lngErrorcode = eDO (lngHandle, 3, lngValue);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	printf("FIO3 set to = %d\n\n",lngValue);


	//Timers and Counters example.
	//First, a call to eTCConfig.  Fill the arrays with the desired values, then make the call.
	lngTimerClockBaseIndex = LJ_tc750KHZ;  //Choose 750 kHz base clock.
	lngTimerClockDivisor = 3; //Divide by 3, thus timer clock is 250 kHz.
	alngEnableTimers[0] = 1; //Enable Timer0 (uses FIO0).
	alngEnableTimers[1] = 1; //Enable Timer1 (uses FIO1).
	alngEnableTimers[2] = 1; //Enable Timer2 (uses FIO2).
	alngEnableTimers[3] = 1; //Enable Timer3 (uses FIO3).
	alngEnableTimers[4] = 0; //Disable Timer4.
	alngEnableTimers[5] = 0; //Disable Timer5.
	alngTimerModes[0] = LJ_tmPWM8; //Timer0 is 8-bit PWM output.  Frequency is 250k/256 = 977 Hz.
	alngTimerModes[1] = LJ_tmDUTYCYCLE; //Timer1 is duty cyle input.
	alngTimerModes[2] = LJ_tmFIRMCOUNTER; //Timer2 is firmware counter input.
	alngTimerModes[3] = LJ_tmRISINGEDGES16; //Timer3 is 16-bit period measurement.
	alngTimerModes[4] = 0; //Timer4 not enabled.
	alngTimerModes[5] = 0; //Timer5 not enabled.
	adblTimerValues[0] = 16384; //Set PWM8 duty-cycle to 75%.
	adblTimerValues[1] = 0;
	adblTimerValues[2] = 0;
	adblTimerValues[3] = 0;
	adblTimerValues[4] = 0;
	adblTimerValues[5] = 0;
	alngEnableCounters[0] = 1; //Enable Counter0 (uses FIO4).
	alngEnableCounters[1] = 1; //Enable Counter1 (uses FIO5).
	lngErrorcode = eTCConfig(lngHandle, alngEnableTimers, alngEnableCounters, 0, lngTimerClockBaseIndex, lngTimerClockDivisor, alngTimerModes, adblTimerValues, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	printf("Timers and Counters enabled.\n\n");

	Sleep(1000); //Wait 1 second.

	//Now, a call to eTCValues.
	alngReadTimers[0] = 0; //Don't read Timer0 (output timer).
	alngReadTimers[1] = 1; //Read Timer1;
	alngReadTimers[2] = 1; //Read Timer2;
	alngReadTimers[3] = 1; //Read Timer3;
	alngReadTimers[4] = 0; //Timer4 not enabled.
	alngReadTimers[5] = 0; //Timer5 not enabled.
	alngUpdateResetTimers[0] = 1; //Update Timer0;
	alngUpdateResetTimers[1] = 1; //Reset Timer1;
	alngUpdateResetTimers[2] = 1; //Reset Timer2;
	alngUpdateResetTimers[3] = 1; //Reset Timer3;
	alngUpdateResetTimers[4] = 0; //Timer4 not enabled.
	alngUpdateResetTimers[5] = 0; //Timer5 not enabled.
	alngReadCounters[0] = 1; //Read Counter0;
	alngReadCounters[1] = 1; //Read Counter1;
	alngResetCounters[0] = 1; //Reset Counter0.
	alngResetCounters[1] = 1; //Reset Counter1.
	adblTimerValues[0] = 32768; //Change Timer0 duty-cycle to 50%.
	adblTimerValues[1] = 0;
	adblTimerValues[2] = 0;
	adblTimerValues[3] = 0;
	adblTimerValues[4] = 0;
	adblTimerValues[5] = 0;
	lngErrorcode = eTCValues(lngHandle, alngReadTimers, alngUpdateResetTimers, alngReadCounters, alngResetCounters, adblTimerValues, adblCounterValues, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	printf("Timer1 value = %.3f\n",adblTimerValues[1]);
	printf("Timer2 value = %.3f\n",adblTimerValues[2]);
	printf("Timer3 value = %.3f\n",adblTimerValues[3]);
	printf("Counter0 value = %.3f\n",adblCounterValues[0]);
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
	alngEnableTimers[2] = 0;
	alngEnableTimers[3] = 0;
	alngEnableTimers[4] = 0;
	alngEnableTimers[5] = 0;
	alngEnableCounters[0] = 0;
	alngEnableCounters[1] = 0;
	lngErrorcode = eTCConfig(lngHandle, alngEnableTimers, alngEnableCounters, 0, lngTimerClockBaseIndex, lngTimerClockDivisor, alngTimerModes, adblTimerValues, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);



}

