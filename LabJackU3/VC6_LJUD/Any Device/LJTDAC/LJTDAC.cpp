//---------------------------------------------------------------------------
//
//  LJTDAC.cpp
//  
//	Communicates with an LJTick-DAC using the UD driver.
//
//	Tested with UD driver V3.03.
//  Tested with UE9 Comm firmware V1.40 and Control firmware V1.84.
//  Tested with U3-HV firmware V1.07.
//
//  support@labjack.com
//  Aug 4, 2008
//----------------------------------------------------------------------
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
//	long lngGetNextIteration;
//	long lngIOType=0, lngChannel=0;
//	double dblValue=0;
	LJ_HANDLE lngHandle=0;

	long i=0;
	double pinNum = 4;  //4 means the LJTick-DAC is connected to FIO4/FIO5.
	char achrUserMem[64] = {0};
	double adblCalMem[4] = {0};
	double serialNumber=0;

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



	//Open the LabJack.
	//lngErrorcode = OpenLabJack (LJ_dtUE9, LJ_ctUSB, "1", 1, &lngHandle);
	//lngErrorcode = OpenLabJack (LJ_dtUE9, LJ_ctETHERNET, "192.168.1.209", 0, &lngHandle);
	//lngErrorcode = OpenLabJack (LJ_dtU3, LJ_ctUSB, "1", 1, &lngHandle);
	lngErrorcode = OpenLabJack (LJ_dtU6, LJ_ctUSB, "1", 1, &lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);



	//Specify where the LJTick-DAC is plugged in.
	//This is just setting a parameter in the driver, and not actually talking
	//to the hardware, and thus executes very fast.
	lngErrorcode = ePut(lngHandle, LJ_ioPUT_CONFIG, LJ_chTDAC_SCL_PIN_NUM,pinNum,0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Set DACA to 1.2 volts.  If the driver has not previously talked to an LJTDAC
	//on the specified pins, it will first retrieve and store the cal constants.  The
	//low-level I2C command can only update 1 DAC channel at a time, so there
	//is no advantage to doing two updates within a single add-go-get block.
	lngErrorcode = ePut(lngHandle, LJ_ioTDAC_COMMUNICATION, LJ_chTDAC_UPDATE_DACA, 1.2, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	printf("DACA set to 1.2 volts\n\n");

	//Set DACB to 2.3 volts.
	lngErrorcode = ePut(lngHandle, LJ_ioTDAC_COMMUNICATION, LJ_chTDAC_UPDATE_DACB, 2.3, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	printf("DACB set to 2.3 volts\n\n");



	//Now for more advanced operations.


	//If at this point you removed that LJTDAC and plugged a different one
	//into the same pins, the driver would not know and would use the wrong
	//cal constants on future updates.  If we do a cal constant read,
	//the driver will store the constants from the new read.
	lngErrorcode = eGet(lngHandle, LJ_ioTDAC_COMMUNICATION, LJ_chTDAC_READ_CAL_CONSTANTS, 0, padblCalMem);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	printf("DACA Slope = %.1f bits/volt\n",adblCalMem[0]);
	printf("DACA Offset = %.1f bits\n",adblCalMem[1]);
	printf("DACB Slope = %.1f bits/volt\n",adblCalMem[2]);
	printf("DACB Offset = %.1f bits\n\n",adblCalMem[3]);



	//Read the serial number.
	lngErrorcode = eGet(lngHandle, LJ_ioTDAC_COMMUNICATION, LJ_chTDAC_SERIAL_NUMBER, &serialNumber, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	printf("LJTDAC Serial Number = %.0f\n\n",serialNumber);



	//Now a user memory example.  We will read the memory, update a few elements,
	//and write the memory.  The entire memory area is read and written each time.
	//The user memory is just stored as bytes, so almost any information can be
	//put in there such as integers, doubles, or strings.

	//Read the user memory.
    lngErrorcode = eGet(lngHandle, LJ_ioTDAC_COMMUNICATION, LJ_chTDAC_READ_USER_MEM, 0, pachrUserMem);
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
	lngErrorcode = ePut(lngHandle, LJ_ioTDAC_COMMUNICATION, LJ_chTDAC_WRITE_USER_MEM, 0, pachrUserMem);
    ErrorHandler(lngErrorcode, __LINE__, 0);
	//The driver delays 2 ms to allow the EEPROM to finish writing.
	//Re-read the user memory.
    lngErrorcode = eGet(lngHandle, LJ_ioTDAC_COMMUNICATION, LJ_chTDAC_READ_USER_MEM, 0, pachrUserMem);
    ErrorHandler(lngErrorcode, __LINE__, 0);
	//Display the first 4 elements.
	printf("Read User Mem [0-3] = %d, %d, %d, %d\n",achrUserMem[0],achrUserMem[1],achrUserMem[2],achrUserMem[3]);




	getchar();
	return;

}
