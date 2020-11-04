//---------------------------------------------------------------------------
//
//  This project demonstrates the line to line timer mode on the U3.
//  For this example, FIO4 and FIO5 should be jumpered to FIO6 and FIO7
//  respectively. If things are working properly, the first iteration
//  and second iterations should read 1325 and 4309 counts respectively.
//
//  Tested on firmware 1.30 and hardware version 1.30
//
//  support@labjack.com
//  Jul 15, 2010
//----------------------------------------------------------------------
//

#include <stdio.h>
#include <windows.h>
#include "LabJackUD.h"
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


const long DRIVER_CLOCK_BASE = LJ_tc48MHZ; // The value to be passed to the driver
const long DRIVER_CLOCK_DIV = 1; // The divisor to be passed to the driver
                                 // This constant is only provided in case you expand upon this
                                 // example but it is not in use.
const long CLOCK = 48000000; // Base / divisor in Hz


void main()
{
	long lngErrorcode, lngHandle, lngIOType, lngChannel;
	double dblValue;
	double firstIterationCount, secondIterationCount;
	double firstIterationTime, secondIterationTime;
	int i;

	// Open the first found LabJack U3.
	lngErrorcode = OpenLabJack (LJ_dtU3, LJ_ctUSB, "1", 1, &lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	// Start by using the pin_configuration_reset IOType so that all
	// pin assignments are in the factory default condition.
	lngErrorcode = ePut (lngHandle, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);


	// First requests to configure the timer and counter.  These will be
	// done with and add/go/get block.

	// Set the timer/counter pin offset to 6, which will put the first
	// timer/counter on FIO6.
	lngErrorcode = AddRequest (lngHandle,  LJ_ioPUT_CONFIG, LJ_chTIMER_COUNTER_PIN_OFFSET, 6, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

    //Use the 48 MHz timer clock base
	//Max pulse of 1365 microseconds as given by the formula: max pulse (ms) = (2^16-1) / clock
	//NOTE: If you change this value, please update the constants above
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_BASE, DRIVER_CLOCK_BASE, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	// Use this next section for a divisor (only for users that expand upon this example)
    // Set the divisor to 48 so the actual timer clock is 1 MHz.
    // NOTE: If you use this section, please update the constants above
	//lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_DIVISOR, DRIVER_CLOCK_DIV, 0, 0);
	//ErrorHandler(lngErrorcode, __LINE__, 0);

	// Enable 2 timers.  It will use FIO6 and FIO7.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chNUMBER_TIMERS_ENABLED, 2, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	// Configure Timer 0 and 1 as line to line (mode 14) (req firmware 1.30 or later)
	// This timer mode is newer than the latest release of the UD driver and a constant has
	// not yet been included in the header file
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_MODE, 0, 14, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_MODE, 1, 14, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
    // Set timers to read rising edges
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_VALUE, 0, 1, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_VALUE, 1, 1, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);

    // Set FIO lines low to begin the demonstration
    
    // Set FIO4 low
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_DIGITAL_BIT, 4, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
    // Set FIO5 low
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_DIGITAL_BIT, 5, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
    // Execute the requests.
    lngErrorcode = GoOne(lngHandle);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
    // This first example deomonstrates a quick pulse by first setting FIO4 high
    // and then immediately setting FIO5 high
    
    // Set FIO4 high to start timer (assumes that FIO4 is jumpered to FIO6)
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_DIGITAL_BIT, 4, 1, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
    // Set FIO5 high to stop timer (assumes that FIO5 is jumpered to FIO7)
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_DIGITAL_BIT, 5, 1, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
    // Execute the requests.
    lngErrorcode = GoOne(lngHandle);
    ErrorHandler(lngErrorcode, __LINE__, 0);

    // Get all the results just to check for errors
    lngErrorcode = GetFirstResult(lngHandle, &lngIOType, &lngChannel, &dblValue, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    while (lngErrorcode < LJE_MIN_GROUP_ERROR)
	{
        lngErrorcode = GetNextResult(lngHandle, &lngIOType, &lngChannel, &dblValue, 0, 0);
        if (lngErrorcode != LJE_NO_MORE_DATA_AVAILABLE) 
			ErrorHandler(lngErrorcode, __LINE__, 0);
	}
    
    // Get the reading from Timer0
    lngErrorcode = eGet(lngHandle, LJ_ioGET_TIMER, 0, &firstIterationCount, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
    // Reset the timer and FIO lines. Please note that, when using line to line
    // mode, the user must reset the timer before the LabJack will measure again.
    
    // Set FIO4 low
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_DIGITAL_BIT, 4, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
    // Set FIO5 low
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_DIGITAL_BIT, 5, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
    // Reset the timers
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_VALUE, 0, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_VALUE, 1, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
    // This next example demonstrates a longer pulse by setting FIO4 high
    // repeatedly and then setting FIO5 high.
    
    // Set FIO4 high
    for (i = 0; i < 4; i++)
	{
        lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_DIGITAL_BIT, 4, 1, 0, 0);
        ErrorHandler(lngErrorcode, __LINE__, 0);
    }
    
    // Set FIO5 high to stop the timer
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_DIGITAL_BIT, 5, 1, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
    // Read from Timer0
    lngErrorcode = AddRequest(lngHandle, LJ_ioGET_TIMER, 0, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
    // Execute the requests.
    lngErrorcode = GoOne(lngHandle);
    ErrorHandler(lngErrorcode, __LINE__, 0);

    // Get all the results just to check for errors
    lngErrorcode = GetFirstResult(lngHandle, &lngIOType, &lngChannel, &dblValue, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    while (lngErrorcode < LJE_MIN_GROUP_ERROR)
	{
        lngErrorcode = GetNextResult(lngHandle, &lngIOType, &lngChannel, &dblValue, 0, 0);
        if (lngErrorcode != LJE_NO_MORE_DATA_AVAILABLE) 
			ErrorHandler(lngErrorcode, __LINE__, 0);
	}

	 // Get the reading from Timer0
    lngErrorcode = eGet(lngHandle, LJ_ioGET_TIMER, 0, &secondIterationCount, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
    // Calculate the time of the pulse for each iteration
    // NOTE: if you change the clock, this formula's cosntants will need to be
    //       updated above
    firstIterationTime = firstIterationCount / CLOCK * 1000000;
    secondIterationTime = secondIterationCount / CLOCK * 1000000;
    
    // Display results
    printf("First experiment: %4.3f microseconds\n", firstIterationTime);
    printf("Second experiment: %4.3f microseconds\n", secondIterationTime);
    
    // Reset the timer and FIO lines in case you run it again
    
    // Set FIO4 low
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_DIGITAL_BIT, 4, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
    // Set FIO5 low
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_DIGITAL_BIT, 5, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
    // Reset the timers
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_VALUE, 0, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_TIMER_VALUE, 1, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);

	// Execute the requests.
    lngErrorcode = GoOne(lngHandle);
    ErrorHandler(lngErrorcode, __LINE__, 0);

    // Get all the results just to check for errors
    lngErrorcode = GetFirstResult(lngHandle, &lngIOType, &lngChannel, &dblValue, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    while (lngErrorcode < LJE_MIN_GROUP_ERROR)
	{
        lngErrorcode = GetNextResult(lngHandle, &lngIOType, &lngChannel, &dblValue, 0, 0);
        if (lngErrorcode != LJE_NO_MORE_DATA_AVAILABLE)
			ErrorHandler(lngErrorcode, __LINE__, 0);
	}

	
}
