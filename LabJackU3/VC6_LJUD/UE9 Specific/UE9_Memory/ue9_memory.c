//---------------------------------------------------------------------------
//
//  ue9_memory.c
//  
//  Demonstrates writing and reading to the user and cal memory on the UE9.
//
//	Tested with UD driver V2.66, and Comm/Control firmware 1.39/1.66.
//
//  support@labjack.com
//  Jan 11, 2007
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
	char achrUserMem[1024] = {0};
	double adblCalMem[128] = {0};


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


	//Open the first found LabJack UE9.
	lngErrorcode = OpenLabJack (LJ_dtUE9, LJ_ctUSB, "1", 1, &lngHandle);
	//lngErrorcode = OpenLabJack (LJ_dtUE9, LJ_ctETHERNET, "192.168.1.209", 0, &lngHandle);
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
	//The memory area consists of 8 blocks (0-7) of 16 doubles each, for a total
	//of 128 elements.  As of this writing, block 7 is not used, so we will
	//use the last 4 elements of block 7 for testing, which is elements 124-127.
	//We will read the constants, update a few elements, and write the constants.  
	//The entire memory area is read and written each time.

	//This cal example is commented out by default, as writing and reading
    //the cal area is an advanced operation.
	
	//Read the cal constants.
    lngErrorcode = eGet(lngHandle, LJ_ioGET_CONFIG, LJ_chCAL_CONSTANTS, 0, padblCalMem);
    ErrorHandler(lngErrorcode, __LINE__, 0);
	//Display the last 4 elements.
	printf("Read Cal Constants [124-127] = %f, %f, %f, %f\n",adblCalMem[124],adblCalMem[125],adblCalMem[126],adblCalMem[127]);
	//Create 4 new pseudo-random numbers to write.  We will update the last
	//4 cal constants, but the rest will be unchanged.
	for(i=124;i<128;i++)
	{
		adblCalMem[i] = (100*((double)rand()/RAND_MAX))-50;
	}
	printf("Write  Cal Constants [124-127] = %f, %f, %f, %f\n",adblCalMem[124],adblCalMem[125],adblCalMem[126],adblCalMem[127]);
	//The special value (0x4C6C) must be put in to write the cal constants.
	lngErrorcode = ePut(lngHandle, LJ_ioPUT_CONFIG, LJ_chCAL_CONSTANTS, 19564, padblCalMem);
    ErrorHandler(lngErrorcode, __LINE__, 0);
	//Re-read the cal constants.
    lngErrorcode = eGet(lngHandle, LJ_ioGET_CONFIG, LJ_chCAL_CONSTANTS, 0, padblCalMem);
    ErrorHandler(lngErrorcode, __LINE__, 0);
	//Display the first 4 elements.
	printf("Read  Cal Constants [124-127] = %f, %f, %f, %f\n",adblCalMem[124],adblCalMem[125],adblCalMem[126],adblCalMem[127]);
	


	getchar();

}
