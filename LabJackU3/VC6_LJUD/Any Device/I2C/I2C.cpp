//---------------------------------------------------------------------------
//
//  I2C.cpp
//  
//	Demonstrates I2C communication using the LJTick-DAC, which is plugged
//  into FIO0/FIO1 for this example.  Writes/reads 4 bytes to the
//  user area (bytes 0-63) on the 24C01C EEPROM.  Reads the cal constants
//  and serial number from the EEPROM.  Updates both DACs on the LTC2617.
//
//  Note that this example is meant to demonstrate I2C communication with
//  the UD driver, not LJTick-DAC operation with the UD driver.  The UD
//  driver has high-level support for the LJTick-DAC, so the I2C
//  communication demonstrated here would not normally be used with the
//  LJTick-DAC when using the UD driver.
//
//	Tested with UD driver V2.74.
//  Tested with UE9 Comm firmware V1.43 and Control firmware V1.83.
//  Tested with U3 firmware V1.43.
//
//  support@labjack.com
//  Aug 20, 2007
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
	long lngGetNextIteration;
	long lngIOType=0, lngChannel=0;
	double dblValue=0;
	LJ_HANDLE lngHandle=0;

	double numI2CBytesToWrite;
	double numI2CBytesToRead;
	unsigned char writeArray[128] = {0};
	unsigned char readArray[128] = {0};
	long i=0;
	long pwriteArray = (long)&writeArray[0];
	long preadArray = (long)&readArray[0];
	long serialNumber=0;
	double slopeDACA=0, offsetDACA=0, slopeDACB=0, offsetDACB=0;
	double writeACKS=0, expectedACKS=0;

	//Seed the random number function.
	srand(GetTickCount());


	//Open the LabJack.
	lngErrorcode = OpenLabJack (LJ_dtU6, LJ_ctUSB, "1", 1, &lngHandle);
	//lngErrorcode = OpenLabJack (LJ_dtUE9, LJ_ctETHERNET, "192.168.1.209", 0, &lngHandle);
	//lngErrorcode = OpenLabJack (LJ_dtU3, LJ_ctUSB, "1", 1, &lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);



	//Configure the I2C communication.
	//The address of the EEPROM on the LJTick-DAC is 0xA0.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chI2C_ADDRESS,160,0,0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//SCL is FIO0
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chI2C_SCL_PIN_NUM,0,0,0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//SDA is FIO1
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chI2C_SDA_PIN_NUM,1,0,0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//See description of low-level I2C function.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chI2C_OPTIONS,0,0,0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//See description of low-level I2C function.  0 is max speed of about 130 kHz.
	lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chI2C_SPEED_ADJUST,0,0,0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	
	//Execute the requests on a single LabJack.
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




	//Initial read of EEPROM bytes 0-3 in the user memory area.
	//We need a single I2C transmission that writes the address and then reads
	//the data.  That is, there needs to be an ack after writing the address,
	//not a stop condition.  To accomplish this, we use Add/Go/Get to combine
	//the write and read into a single low-level call.
	numI2CBytesToWrite = 1;
	writeArray[0] = 0;  //Memory address.  User area is 0-63.
	lngErrorcode = AddRequest(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_WRITE, numI2CBytesToWrite, pwriteArray, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	lngErrorcode = AddRequest(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_GET_ACKS, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	numI2CBytesToRead = 4;
	lngErrorcode = AddRequest(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_READ, numI2CBytesToRead, preadArray, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Execute the requests.
	lngErrorcode = GoOne(lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Get the result of the write just to check for an error.
	lngErrorcode = GetResult(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_WRITE, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Get the write ACKs and compare to the expected value.  We expect bit 0 to be
	//the ACK of the last data byte progressing up to the ACK of the address
	//byte (data bytes only for Control firmware 1.43 and less).  So if n is the
	//number of data bytes, the ACKs value should be (2^(n+1))-1.
	lngErrorcode = GetResult(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_GET_ACKS, &writeACKS);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	expectedACKS = pow(2,numI2CBytesToWrite+1) - 1;
	if(writeACKS != expectedACKS) printf("Expected ACKs = %.0f, Received ACKs = %0.f\n",expectedACKS,writeACKS);


	//When the GoOne processed the read request, the read data was put into the readArray buffer that
	//we passed, so this GetResult is also just to check for an error.
	lngErrorcode = GetResult(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_READ, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Display the first 4 elements.
	printf("Read User Mem [0-3] = %d, %d, %d, %d\n",readArray[0],readArray[1],readArray[2],readArray[3]);




	//Write EEPROM bytes 0-3 in the user memory area, using the page write technique.  Note
	//that page writes are limited to 16 bytes max, and must be aligned with the 16-byte
	//page intervals.  For instance, if you start writing at address 14, you can only write
	//two bytes because byte 16 is the start of a new page.
	numI2CBytesToWrite = 5;
	writeArray[0] = 0;  //Memory address.  User area is 0-63.

	//Create 4 new pseudo-random numbers to write.
	for(i=1;i<5;i++)
	{
		writeArray[i] = (char)(255*((float)rand()/RAND_MAX));
	}
	
	lngErrorcode = AddRequest(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_WRITE, numI2CBytesToWrite, pwriteArray, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	lngErrorcode = AddRequest(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_GET_ACKS, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Execute the requests.
	lngErrorcode = GoOne(lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Get the result of the write just to check for an error.
	lngErrorcode = GetResult(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_WRITE, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Get the write ACKs and compare to the expected value.  We expect bit 0 to be
	//the ACK of the last data byte progressing up to the ACK of the address
	//byte (data bytes only for Control firmware 1.43 and less).  So if n is the
	//number of data bytes, the ACKs value should be (2^(n+1))-1.
	lngErrorcode = GetResult(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_GET_ACKS, &writeACKS);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	expectedACKS = pow(2,numI2CBytesToWrite+1) - 1;
	if(writeACKS != expectedACKS) printf("Expected ACKs = %.0f, Received ACKs = %0.f\n",expectedACKS,writeACKS);

	//Delay to allow the EEPROM to complete the write cycle.  Datasheet says 1.5 ms max.
	Sleep(2);

	printf("Write User Mem [0-3] = %d, %d, %d, %d\n",writeArray[1],writeArray[2],writeArray[3],writeArray[4]);




	//Final read of EEPROM bytes 0-3 in the user memory area.
	//We need a single I2C transmission that writes the address and then reads
	//the data.  That is, there needs to be an ack after writing the address,
	//not a stop condition.  To accomplish this, we use Add/Go/Get to combine
	//the write and read into a single low-level call.
	numI2CBytesToWrite = 1;
	writeArray[0] = 0;  //Memory address.  User area is 0-63.
	lngErrorcode = AddRequest(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_WRITE, numI2CBytesToWrite, pwriteArray, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	lngErrorcode = AddRequest(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_GET_ACKS, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	numI2CBytesToRead = 4;
	lngErrorcode = AddRequest(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_READ, numI2CBytesToRead, preadArray, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Execute the requests.
	lngErrorcode = GoOne(lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Get the result of the write just to check for an error.
	lngErrorcode = GetResult(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_WRITE, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Get the write ACKs and compare to the expected value.  We expect bit 0 to be
	//the ACK of the last data byte progressing up to the ACK of the address
	//byte (data bytes only for Control firmware 1.43 and less).  So if n is the
	//number of data bytes, the ACKs value should be (2^(n+1))-1.
	lngErrorcode = GetResult(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_GET_ACKS, &writeACKS);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	expectedACKS = pow(2,numI2CBytesToWrite+1) - 1;
	if(writeACKS != expectedACKS) printf("Expected ACKs = %.0f, Received ACKs = %0.f\n",expectedACKS,writeACKS);

	//When the GoOne processed the read request, the read data was put into the readArray buffer that
	//we passed, so this GetResult is also just to check for an error.
	lngErrorcode = GetResult(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_READ, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Display the first 4 elements.
	printf("Read User Mem [0-3] = %d, %d, %d, %d\n\n",readArray[0],readArray[1],readArray[2],readArray[3]);




	//Read cal constants and serial number.
	//We need a single I2C transmission that writes the address and then reads
	//the data.  That is, there needs to be an ack after writing the address,
	//not a stop condition.  To accomplish this, we use Add/Go/Get to combine
	//the write and read into a single low-level call.
	//
	//64-71   DACA Slope
	//72-79   DACA Offset
	//80-87   DACB Slope
	//88-95   DACB Offset
	//96-99   Serial Number
	//
	numI2CBytesToWrite = 1;
	writeArray[0] = 64;  //Memory address.  Cal constants start at 64.
	lngErrorcode = AddRequest(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_WRITE, numI2CBytesToWrite, pwriteArray, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	lngErrorcode = AddRequest(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_GET_ACKS, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	numI2CBytesToRead = 36;
	lngErrorcode = AddRequest(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_READ, numI2CBytesToRead, preadArray, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Execute the requests.
	lngErrorcode = GoOne(lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Get the result of the write just to check for an error.
	lngErrorcode = GetResult(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_WRITE, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Get the write ACKs and compare to the expected value.  We expect bit 0 to be
	//the ACK of the last data byte progressing up to the ACK of the address
	//byte (data bytes only for Control firmware 1.43 and less).  So if n is the
	//number of data bytes, the ACKs value should be (2^(n+1))-1.
	lngErrorcode = GetResult(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_GET_ACKS, &writeACKS);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	expectedACKS = pow(2,numI2CBytesToWrite+1) - 1;
	if(writeACKS != expectedACKS) printf("Expected ACKs = %.0f, Received ACKs = %0.f\n",expectedACKS,writeACKS);

	//When the GoOne processed the read request, the read data was put into the readArray buffer that
	//we passed, so this GetResult is also just to check for an error.
	lngErrorcode = GetResult(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_READ, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Convert fixed point values to floating point doubles.
	__int64 *pBegin = (__int64 *)(readArray);
	slopeDACA = (double)(*(pBegin))/(double)4294967296;
	offsetDACA = (double)(*(pBegin + 1))/(double)4294967296;
	slopeDACB = (double)(*(pBegin + 2))/(double)4294967296;
	offsetDACB = (double)(*(pBegin + 3))/(double)4294967296;
	printf("DACA Slope = %.1f bits/volt\n",slopeDACA);
	printf("DACA Offset = %.1f bits\n",offsetDACA);
	printf("DACB Slope = %.1f bits/volt\n",slopeDACB);
	printf("DACB Offset = %.1f bits\n",offsetDACB);

	//Convert serial number bytes to long.
	serialNumber = (int)readArray[32] + ((int) readArray[33] << 8) + ((int) readArray[34] << 16) + ((int) readArray[35] << 24);
	printf("Serial Number = %d\n\n",serialNumber);




	//Update both DAC outputs.

	//Set the I2C address in the UD driver so that we not talk to the DAC chip.
	//The address of the DAC chip on the LJTick-DAC is 0x24.
	lngErrorcode = ePut(lngHandle, LJ_ioPUT_CONFIG, LJ_chI2C_ADDRESS,36,0);
	ErrorHandler(lngErrorcode, __LINE__, 0);


	//Set DACA to 1.2 volts.
	numI2CBytesToWrite = 3;
	writeArray[0] = 48;  //Write and update DACA.
	writeArray[1] = (long)((1.2*slopeDACA)+offsetDACA)/256;  //Upper byte of binary DAC value.
	writeArray[2] = (long)((1.2*slopeDACA)+offsetDACA)%256;  //Lower byte of binary DAC value.

	lngErrorcode = AddRequest(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_WRITE, numI2CBytesToWrite, pwriteArray, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	lngErrorcode = AddRequest(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_GET_ACKS, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Execute the requests.
	lngErrorcode = GoOne(lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Get the result of the write just to check for an error.
	lngErrorcode = GetResult(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_WRITE, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Get the write ACKs and compare to the expected value.  We expect bit 0 to be
	//the ACK of the last data byte progressing up to the ACK of the address
	//byte (data bytes only for Control firmware 1.43 and less).  So if n is the
	//number of data bytes, the ACKs value should be (2^(n+1))-1.
	lngErrorcode = GetResult(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_GET_ACKS, &writeACKS);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	expectedACKS = pow(2,numI2CBytesToWrite+1) - 1;
	if(writeACKS != expectedACKS) printf("Expected ACKs = %.0f, Received ACKs = %0.f\n",expectedACKS,writeACKS);

	printf("DACA set to 1.2 volts\n\n");


	//Set DACB to 2.3 volts.
	numI2CBytesToWrite = 3;
	writeArray[0] = 49;  //Write and update DACB.
	writeArray[1] = (long)((2.3*slopeDACB)+offsetDACB)/256;  //Upper byte of binary DAC value.
	writeArray[2] = (long)((2.3*slopeDACB)+offsetDACB)%256;  //Lower byte of binary DAC value.
	
	lngErrorcode = AddRequest(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_WRITE, numI2CBytesToWrite, pwriteArray, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	lngErrorcode = AddRequest(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_GET_ACKS, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Execute the requests.
	lngErrorcode = GoOne(lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Get the result of the write just to check for an error.
	lngErrorcode = GetResult(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_WRITE, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Get the write ACKs and compare to the expected value.  We expect bit 0 to be
	//the ACK of the last data byte progressing up to the ACK of the address
	//byte (data bytes only for Control firmware 1.43 and less).  So if n is the
	//number of data bytes, the ACKs value should be (2^(n+1))-1.
	lngErrorcode = GetResult(lngHandle, LJ_ioI2C_COMMUNICATION, LJ_chI2C_GET_ACKS, &writeACKS);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	expectedACKS = pow(2,numI2CBytesToWrite+1) - 1;
	if(writeACKS != expectedACKS) printf("Expected ACKs = %.0f, Received ACKs = %0.f\n",expectedACKS,writeACKS);
	
	printf("DACB set to 2.3 volts\n");




	getchar();
	return;

}
