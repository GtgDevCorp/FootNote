//---------------------------------------------------------------------------
//
//  Asynch.cpp
//  
//	Demonstrates asynchronous communication using a loopback from
//	FIO4 to FIO5 on a U3 rev 1.30.  On earlier hardware revisions
//  use SDA and SCL.
//
//	Tested with UD driver V3.03.
//  Tested with U3C firmware V1.08?.
//
//  support@labjack.com
//  Aug 4, 2008
//----------------------------------------------------------------------
//

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
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
	//long lngGetNextIteration;
	//long lngIOType=0, lngChannel=0;
	//double dblValue=0;
	LJ_HANDLE lngHandle=0;

	//double numI2CBytesToWrite;
	double numBytes;
	unsigned char array[256] = {0};
	long parray = (long)&array[0];


	//Open the LabJack.
	lngErrorcode = OpenLabJack (LJ_dtU3, LJ_ctUSB, "1", 1, &lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);


	//Start by using the pin_configuration_reset IOType so that all
	//pin assignments are in the factory default condition.
	lngErrorcode = ePut (lngHandle, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);


	// 1 MHz timer clock base.
	lngErrorcode = ePut(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_BASE, LJ_tc1MHZ_DIV, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	// Set clock divisor to 1, so timer clock is 1 MHz.
	lngErrorcode = ePut(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_DIVISOR, LJ_tc1MHZ_DIV, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	// Set timer/counter pin offset to 4. TX and RX appear after any timers and counters on U3
	// hardware rev 1.30.  We have no timers or counters enabled, so TX=FIO4 and RX=FIO5.
	lngErrorcode = ePut(lngHandle, LJ_ioPUT_CONFIG, LJ_chTIMER_COUNTER_PIN_OFFSET, 4, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	// Set data rate for 9600 bps communication.
	lngErrorcode = ePut(lngHandle, LJ_ioPUT_CONFIG, LJ_chASYNCH_BAUDFACTOR, 204, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	// Enable UART.
	lngErrorcode = ePut(lngHandle, LJ_ioASYNCH_COMMUNICATION, LJ_chASYNCH_ENABLE, 1, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	// Transmit 2 bytes.
	numBytes = 2;
	array[0] = 12;
	array[1] = 34;
	lngErrorcode = eGet(lngHandle, LJ_ioASYNCH_COMMUNICATION, LJ_chASYNCH_TX, &numBytes, parray);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	// Read 2 bytes.
	numBytes = 9999;  //Dummy values so we can see them change.
	array[0] = 222;
	array[1] = 222;
	lngErrorcode = eGet(lngHandle, LJ_ioASYNCH_COMMUNICATION, LJ_chASYNCH_RX, &numBytes, parray);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Display the read data.
	printf("Pre-read buffer size = %.0f\n\n",numBytes);
	printf("Read data = %d, %d\n\n",array[0],array[1]);


	getchar();
	return;

}
