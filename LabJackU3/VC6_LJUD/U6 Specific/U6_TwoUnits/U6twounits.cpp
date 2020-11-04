//---------------------------------------------------------------------------
//
//  U6twounits.cpp
//  
//  Simple example demonstrates communication with 2 U6s.
//
//  support@labjack.com
//  June 5, 2009
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
	long lngGetNextIteration;
	double dblDriverVersion;
	long lngIOType=0, lngChannel=0;
	double dblValue=0;
	double Value12=9999,Value22=9999,Value32=9999;
	double Value13=9999,Value23=9999,Value33=9999;
	LJ_HANDLE lngHandle2=0,lngHandle3=0;
	char ch;


	//Read and display the UD version.
	dblDriverVersion = GetDriverVersion();
	printf("UD Driver Version = %.3f\n\n",dblDriverVersion);


	//Open the U6 with local ID 2.
	lngErrorcode = OpenLabJack (LJ_dtU6, LJ_ctUSB, "2", 0, &lngHandle2);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Open the U6 with local ID 3.
	lngErrorcode = OpenLabJack (LJ_dtU6, LJ_ctUSB, "3", 0, &lngHandle3);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//The following commands will use the add-go-get method to group
	//multiple requests into a single low-level function.

	//Request a single-ended reading from AIN1.
	lngErrorcode = AddRequest (lngHandle2, LJ_ioGET_AIN, 1, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	lngErrorcode = AddRequest (lngHandle3, LJ_ioGET_AIN, 1, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Request a single-ended reading from AIN2.
	lngErrorcode = AddRequest (lngHandle2, LJ_ioGET_AIN, 2, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	lngErrorcode = AddRequest (lngHandle3, LJ_ioGET_AIN, 2, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Request a reading from AIN3 using the Special 0-3.6 range.
	lngErrorcode = AddRequest (lngHandle2, LJ_ioGET_AIN_DIFF, 3, 0, 199, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	lngErrorcode = AddRequest (lngHandle3, LJ_ioGET_AIN_DIFF, 3, 0, 199, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);


	while (1)
	{

		//Execute all requests on all open LabJacks.
		lngErrorcode = Go ();
		ErrorHandler(lngErrorcode, __LINE__, 0);


	   	//Get all the results for unit 2.  The input measurement results are stored.
		lngErrorcode = GetFirstResult(lngHandle2, &lngIOType, &lngChannel, &dblValue, 0, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);

		lngGetNextIteration=0;	//Used by the error handling function.

		while(lngErrorcode < LJE_MIN_GROUP_ERROR)
		{
			switch(lngIOType)
			{

			case LJ_ioGET_AIN :
				switch(lngChannel)
				{
					case 1:
						Value12=dblValue;
						break;
					case 2:
						Value22=dblValue;
						break;
				}
				break;

			case LJ_ioGET_AIN_DIFF :
				Value32=dblValue;
				break;

			}

			lngErrorcode = GetNextResult(lngHandle2, &lngIOType, &lngChannel, &dblValue, 0, 0);
			if(lngErrorcode != LJE_NO_MORE_DATA_AVAILABLE)
			{
				ErrorHandler(lngErrorcode, __LINE__, lngGetNextIteration);
			}

			lngGetNextIteration++;

		}


		//Get all the results for unit 3.  The input measurement results are stored.
		lngErrorcode = GetFirstResult(lngHandle3, &lngIOType, &lngChannel, &dblValue, 0, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);

		lngGetNextIteration=0;	//Used by the error handling function.

		while(lngErrorcode < LJE_MIN_GROUP_ERROR)
		{
			switch(lngIOType)
			{

			case LJ_ioGET_AIN :
				switch(lngChannel)
				{
					case 1:
						Value13=dblValue;
						break;
					case 2:
						Value23=dblValue;
						break;
				}
				break;

			case LJ_ioGET_AIN_DIFF :
				Value33=dblValue;
				break;

			}

			lngErrorcode = GetNextResult(lngHandle3, &lngIOType, &lngChannel, &dblValue, 0, 0);
			if(lngErrorcode != LJE_NO_MORE_DATA_AVAILABLE)
			{
				ErrorHandler(lngErrorcode, __LINE__, lngGetNextIteration);
			}

			lngGetNextIteration++;

		}



		printf("AIN1 (Unit 2) = %f\n",Value12);
		printf("AIN1 (Unit 3) = %f\n",Value13);
  		printf("AIN2 (Unit 2) = %f\n",Value22);
		printf("AIN2 (Unit 3) = %f\n",Value23);
		printf("AIN3 (Unit 2) = %f\n",Value32);
		printf("AIN3 (Unit 3) = %f\n",Value33);
	
		printf("\nPress Enter to go again or (q) to quit\n");
		ch = getchar();
		if (ch == 'q') return 0;
	}


}
