//---------------------------------------------------------------------------
//
//  rawoutin.c
// 
//	Simple demonstration of using RAW_OUT and RAW_IN.  Sends a 0x70,0x70
//	which is a do nothing Comm command that simply responds with an echo.
//	Records the time for 1000 iterations and divides by 1000, to determine
//  the basic communication time of the LabJack.
//
//	For more in depth examples of low-level function calls, see the
//	Linux downloads.
//
//  support@labjack.com
//  Feb 3, 2006
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
	LJ_HANDLE lngHandle=0;
	char ch;
	long time=0, i=0;
	long numIterations = 1000;

	double numBytesToWrite;
	double numBytesToRead;
	char writeArray[2] = {0x70,0x70};
	long pwriteArray = (long)&writeArray[0];
	char readArray[2] = {0};
	long preadArray = (long)&readArray[0];


	//Open the first found LabJack UE9.
	lngErrorcode = OpenLabJack (LJ_dtUE9, LJ_ctUSB, "1", 1, &lngHandle);
	//lngErrorcode = OpenLabJack (LJ_dtUE9, LJ_ctETHERNET, "192.168.1.209", 0, &lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	time = GetTickCount();

	for(i=0;i<numIterations;i++)
	{
		numBytesToWrite = 2;
		numBytesToRead = 2;

		//Raw Out
		lngErrorcode = eGet(lngHandle, LJ_ioRAW_OUT, 0, &numBytesToWrite, pwriteArray);
		ErrorHandler(lngErrorcode, __LINE__, 0);

		//Raw In
		lngErrorcode = eGet(lngHandle, LJ_ioRAW_IN, 0, &numBytesToRead, preadArray);
		ErrorHandler(lngErrorcode, __LINE__, 0);

	}

	time = GetTickCount() - time;

	printf("Milleseconds per iteration = %.3f\n", (double)time / (double)numIterations);

	ch = getchar();
	return 0;

}
