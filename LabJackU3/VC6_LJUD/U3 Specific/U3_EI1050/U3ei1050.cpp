//---------------------------------------------------------------------------
//
//  U3ei1050.cpp
//  
//  Demonstrates talking to 1 or 2 EI-1050 probes.
//
//  support@labjack.com
//  Jul 21, 2008
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
	double dblValue=0;
	LJ_HANDLE lngHandle=0;
	char ch;



	//Open the first found LabJack U3.
	lngErrorcode = OpenLabJack (LJ_dtU3, LJ_ctUSB, "1", 1, &lngHandle);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Start by using the pin_configuration_reset IOType so that all
	//pin assignments are in the factory default condition.
	lngErrorcode = ePut (lngHandle, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);



	//Set the Data line to FIO4, which is the default anyway. 
	lngErrorcode = ePut (lngHandle, LJ_ioPUT_CONFIG, LJ_chSHT_DATA_CHANNEL, 4, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Set the Clock line to FIO5, which is the default anyway. 
	lngErrorcode = ePut (lngHandle, LJ_ioPUT_CONFIG, LJ_chSHT_CLOCK_CHANNEL, 5, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Set FIO6 to output-high to provide power to the EI-1050s. 
	lngErrorcode = ePut (lngHandle, LJ_ioPUT_DIGITAL_BIT, 6, 1, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);


	
	///*
	//Use this code if only a single EI-1050 is connected.
	//	Connections for one probe:
	//	Red (Power)         FIO6
	//	Black (Ground)      GND
	//	Green (Data)        FIO4
	//	White (Clock)       FIO5
	//	Brown (Enable)      FIO6

			//Now, an add/go/get block to get the temp & humidity at the same time.
			//Request a temperature reading from the EI-1050.
			lngErrorcode = AddRequest (lngHandle, LJ_ioSHT_GET_READING, LJ_chSHT_TEMP, 0, 0, 0);
			ErrorHandler(lngErrorcode, __LINE__, 0);

			//Request a humidity reading from the EI-1050.
			lngErrorcode = AddRequest (lngHandle, LJ_ioSHT_GET_READING, LJ_chSHT_RH, 0, 0, 0);
			ErrorHandler(lngErrorcode, __LINE__, 0);

			//Execute the requests.  Will take about 0.5 seconds with a USB high-high
			//or Ethernet connection, and about 1.5 seconds with a normal USB connection.
			lngErrorcode = GoOne (lngHandle);
			ErrorHandler(lngErrorcode, __LINE__, 0);

			//Get the temperature reading.
			lngErrorcode = GetResult (lngHandle, LJ_ioSHT_GET_READING, LJ_chSHT_TEMP, &dblValue);
			ErrorHandler(lngErrorcode, __LINE__, 0);
			printf("Temp Probe A = %f deg K\n",dblValue);
			printf("Temp Probe A = %f deg C\n",(dblValue-273.15));
			printf("Temp Probe A = %f deg F\n",(((dblValue-273.15)*1.8)+32));

			//Get the humidity reading.
			lngErrorcode = GetResult (lngHandle, LJ_ioSHT_GET_READING, LJ_chSHT_RH, &dblValue);
			ErrorHandler(lngErrorcode, __LINE__, 0);
			printf("RH Probe A = %f percent\n\n",dblValue);

	//End of single probe code.
	//*/


	/*
	//Use this code if two EI-1050 probes are connected.
	//	Connections for both probes:
	//	Red (Power)         FIO6
	//	Black (Ground)      GND
	//	Green (Data)        FIO4
	//	White (Clock)       FIO5
	//
	//	Probe A:
	//	Brown (Enable)    FIO7
	//
	//	Probe B:
	//	Brown (Enable)    DAC0

			//Set FIO7 to output-low to disable probe A. 
			lngErrorcode = ePut (lngHandle, LJ_ioPUT_DIGITAL_BIT, 7, 0, 0);
			ErrorHandler(lngErrorcode, __LINE__, 0);

			//Set DAC0 to 0 volts to disable probe B.
			lngErrorcode = ePut (lngHandle, LJ_ioPUT_DAC, 0, 0.0, 0);
			ErrorHandler(lngErrorcode, __LINE__, 0);




			//Set FIO7 to output-high to enable probe A. 
			lngErrorcode = ePut (lngHandle, LJ_ioPUT_DIGITAL_BIT, 7, 1, 0);
			ErrorHandler(lngErrorcode, __LINE__, 0);

			//Now, an add/go/get block to get the temp & humidity at the same time.
			//Request a temperature reading from the EI-1050.
			lngErrorcode = AddRequest (lngHandle, LJ_ioSHT_GET_READING, LJ_chSHT_TEMP, 0, 0, 0);
			ErrorHandler(lngErrorcode, __LINE__, 0);

			//Request a humidity reading from the EI-1050.
			lngErrorcode = AddRequest (lngHandle, LJ_ioSHT_GET_READING, LJ_chSHT_RH, 0, 0, 0);
			ErrorHandler(lngErrorcode, __LINE__, 0);

			//Execute the requests.  Will take about 0.5 seconds with a USB high-high
			//or Ethernet connection, and about 1.5 seconds with a normal USB connection.
			lngErrorcode = GoOne (lngHandle);
			ErrorHandler(lngErrorcode, __LINE__, 0);

			//Get the temperature reading.
			lngErrorcode = GetResult (lngHandle, LJ_ioSHT_GET_READING, LJ_chSHT_TEMP, &dblValue);
			ErrorHandler(lngErrorcode, __LINE__, 0);
			printf("Temp Probe A = %f deg K\n",dblValue);
			printf("Temp Probe A = %f deg C\n",(dblValue-273.15));
			printf("Temp Probe A = %f deg F\n",(((dblValue-273.15)*1.8)+32));

			//Get the humidity reading.
			lngErrorcode = GetResult (lngHandle, LJ_ioSHT_GET_READING, LJ_chSHT_RH, &dblValue);
			ErrorHandler(lngErrorcode, __LINE__, 0);
			printf("RH Probe A = %f percent\n\n",dblValue);

			//Set FIO7 to output-low to disable probe A. 
			lngErrorcode = ePut (lngHandle, LJ_ioPUT_DIGITAL_BIT, 7, 0, 0);
			ErrorHandler(lngErrorcode, __LINE__, 0);




			//Set DAC0 to 3.3 volts to enable probe B.
			lngErrorcode = ePut (lngHandle, LJ_ioPUT_DAC, 0, 3.3, 0);
			ErrorHandler(lngErrorcode, __LINE__, 0);

			//Since the DACs on the U3 are slower than the communication speed,
			//we put a delay here to make sure the DAC has time to rise to 3.3 volts
			//before communicating with the EI-1050.
			Sleep(30);  //Wait 30 ms.

			//Now, an add/go/get block to get the temp & humidity at the same time.
			//Request a temperature reading from the EI-1050.
			lngErrorcode = AddRequest (lngHandle, LJ_ioSHT_GET_READING, LJ_chSHT_TEMP, 0, 0, 0);
			ErrorHandler(lngErrorcode, __LINE__, 0);

			//Request a humidity reading from the EI-1050.
			lngErrorcode = AddRequest (lngHandle, LJ_ioSHT_GET_READING, LJ_chSHT_RH, 0, 0, 0);
			ErrorHandler(lngErrorcode, __LINE__, 0);

			//Execute the requests.  Will take about 0.5 seconds with a USB high-high
			//or Ethernet connection, and about 1.5 seconds with a normal USB connection.
			lngErrorcode = GoOne (lngHandle);
			ErrorHandler(lngErrorcode, __LINE__, 0);

			//Get the temperature reading.
			lngErrorcode = GetResult (lngHandle, LJ_ioSHT_GET_READING, LJ_chSHT_TEMP, &dblValue);
			ErrorHandler(lngErrorcode, __LINE__, 0);
			printf("Temp Probe B = %f deg K\n",dblValue);
			printf("Temp Probe B = %f deg C\n",(dblValue-273.15));
			printf("Temp Probe B = %f deg F\n",(((dblValue-273.15)*1.8)+32));

			//Get the humidity reading.
			lngErrorcode = GetResult (lngHandle, LJ_ioSHT_GET_READING, LJ_chSHT_RH, &dblValue);
			ErrorHandler(lngErrorcode, __LINE__, 0);
			printf("RH Probe B = %f percent\n\n",dblValue);

			//Set DAC0 to 0 volts to disable probe B.
			lngErrorcode = ePut (lngHandle, LJ_ioPUT_DAC, 0, 0.0, 0);
			ErrorHandler(lngErrorcode, __LINE__, 0);
			
			//If we were going to loop and talk to probe A next, we would
			//want a delay here to make sure the DAC falls to 0 volts
			//before enabling probe A.
			Sleep(30);  //Wait 30 ms.

	//End of dual probe code.
	*/




	ch = getchar();
	return;

}
