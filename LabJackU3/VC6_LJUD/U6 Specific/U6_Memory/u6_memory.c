//---------------------------------------------------------------------------
//
//  u6_memory.c
//  
//  Demonstrates writing and reading to the user and cal memory on the U6.
//
//	Tested with UD driver V3.17, and firmware 1.13.
//
//  support@labjack.com
//  May 26, 2010
//---------------------------------------------------------------------------
//

#include <stdio.h>
#include <stdlib.h>
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
	LJ_HANDLE lngHandle=0;
	long i=0;
	char achrUserMem[256] = {0};
	double adblCalMem[64] = {0};


	//Make a long parameter which holds the address of the data arrays.  We do this
	//so the compiler does not generate a warning in the eGet call that passes
	//the data.  Note that the x1 parameter  in eGet (and AddRequest) is fairly
	//generic, in that sometimes it could just be a write parameter, and sometimes
	//it has the address of an array.  Since x1 is not declared as a pointer, the
	//compiler will complain if you just pass the array pointer without casting
	//it to a long as follows.
	long pachrUserMem = (long)&achrUserMem[0];
	long padblCalMem = (long)&adblCalMem[0];

	//Seed the random number function.
	srand(GetTickCount());


	//Open the first found LabJack U6.
	lngErrorcode = OpenLabJack (LJ_dtU6, LJ_ctUSB, "1", 1, &lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);


	//First a user memory example.  We will read the memory, update a few elements,
	//and write the memory.  The entire memory area is read and written each time.
	//The user memory is just stored as bytes, so almost any information can be
	//put in there such as integers, doubles, or strings.

	//Read the user memory.
    lngErrorcode = eGet(lngHandle, LJ_ioGET_CONFIG, LJ_chUSER_MEM, 0, pachrUserMem);
    ErrorHandler(lngErrorcode, __LINE__, 0);
	//Display the first 4 elements.
	printf("Read User Mem [0-3] = %d, %d, %d, %d\n",achrUserMem[0],achrUserMem[1],achrUserMem[2],achrUserMem[3]);
	//Create 4 new pseudo-random numbers to write.  We will update the first
	//4 elements of user memory, but the rest will be unchanged.
	for(i=0;i<4;i++)
	{
		achrUserMem[i] = (char)(255*((float)rand()/RAND_MAX));
	}
	printf("Write User Mem [0-3] = %d, %d, %d, %d\n",achrUserMem[0],achrUserMem[1],achrUserMem[2],achrUserMem[3]);
	lngErrorcode = ePut(lngHandle, LJ_ioPUT_CONFIG, LJ_chUSER_MEM, 0, pachrUserMem);
    ErrorHandler(lngErrorcode, __LINE__, 0);
	//Re-read the user memory.
    lngErrorcode = eGet(lngHandle, LJ_ioGET_CONFIG, LJ_chUSER_MEM, 0, pachrUserMem);
    ErrorHandler(lngErrorcode, __LINE__, 0);
	//Display the first 4 elements.
	printf("Read User Mem [0-3] = %d, %d, %d, %d\n",achrUserMem[0],achrUserMem[1],achrUserMem[2],achrUserMem[3]);

	

	//Now a cal constants example.  The calibration memory is passed as doubles.
	//The memory area consists of 16 blocks (0-15) of 4 doubles each, for a total
	//of 64 elements. As of writing this, only blocks 0-9 are used.

	//This cal example is commented out by default, as writing and reading
    //the cal area is an advanced operation.
	/**
	//Read the cal constants.
    lngErrorcode = eGet(lngHandle, LJ_ioGET_CONFIG, LJ_chCAL_CONSTANTS, 0, padblCalMem);
    ErrorHandler(lngErrorcode, __LINE__, 0);
	//Display block 1 -> elements 0-3
	printf("Read Cal Constants [0-3] = %f, %f, %f, %f\n",adblCalMem[0],adblCalMem[1],adblCalMem[2],adblCalMem[3]);
	**/
	getchar();

}
