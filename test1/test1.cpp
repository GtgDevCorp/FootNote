//
//      FILE:           test1.cpp
//      AUTHOR:         Neil Meisenbacher (nmeis@gtgd.com)
//						Copyright (c), 2020, GTG Development Corporation
//						All Rights Reserved
//
//      PURPOSE:        Main code module for FootNote/LabJack-U3 "FootNote" interface test program #1
//
//
//		MAJOR
//		REVISIONS:		08/04/2020 - nmeis - wrote it
//
//
//      REFERENCES:     Footnotes-072820.docx   (Don Slepian <don.slepian@gmail.com>)
//                      https://labjack.com/support/software/api/ud/overview
//                      https://forums.labjack.com/index.php?showtopic=4279
//                      https://www.music.mcgill.ca/~gary/rtmidi/
//                      https://forums.labjack.com/index.php?showtopic=6741
//                      https://labjack.com/support/datasheets/u3/operation/stream-mode/digital-inputs-timers-counters
//                      https://www.cs.cmu.edu/~music/cmsip/readings/MIDI%20tutorial%20for%20programmers.html
//                      https://docs.microsoft.com/en-us/windows/win32/sysinfo/acquiring-high-resolution-time-stamps


//
#include <iostream>
#include <bitset>
#include <Windows.h>
#include <time.h>
#include <conio.h>
#include <sys\timeb.h>
#include "LabJackUD.h"
#include "RtMidi.h"

// pull in Windows MM library (required for MIDI interface)
#pragma comment(lib, "winmm.lib")

// useful macros
#define TRUE                        1
#define FALSE                       0
#define ZERO(x)                     memset(&x,0,sizeof(x))
#define NO_SCAN_VALUE               ((double)-9999.98765)
#define TIMER_TYPE			        __int64
#define CSV_FILE                    "test1.csv"

// LabJack device initialization constants
#define LJ_DEVICE_TYPE              LJ_dtU3
#define LJ_DEVICE_INTERFACE         LJ_ctUSB
#define LJ_DEVICE_NUM               "1" 
#define SCAN_FREQUENCY              5000
#define NUM_CHANNELS                9
#define NUM_SCANS_PER_ITERATION     10
#define DAC0_VOLTAGE_VALUE          2.6

// min & max allowable velocities for KG1 & KG2 (in seconds)
#define MIN_KG1_VELOCITY_SECS       (0.005000)
#define MAX_KG1_VELOCITY_SECS       (1.579818)
#define MIN_KG2_VELOCITY_SECS       MIN_KG1_VELOCITY_SECS
#define MAX_KG2_VELOCITY_SECS       MAX_KG1_VELOCITY_SECS

// the LabJack device handle
static long         lngHandle = 0;

// LabJack device scan result buffer
static double       adblData[NUM_CHANNELS * NUM_SCANS_PER_ITERATION];

// MIDI device interface
static RtMidiOut        *midiout = NULL;
static const int        iMIDI_NOTE_OFF = 0x80;
static const int        iMIDI_NOTE_ON = 0x90;
static const int        iMIDI_CHANNEL_PRESSURE = 0xD0;
static const int        iMIDI_CHANNEL_SLOPE = 0x40;
static const int        iMidiPortNum = 1;
static const int        iMIDI_CC_PRESSURE_SLOPE1 = 102;
static const int        iMIDI_CC_PRESSURE_PAD1A = 103;
static const int        iMIDI_CC_PRESSURE_PAD1B = 104;
static const int        iMIDI_CC_PRESSURE_TOESWITCH1 = 105;
static const int        iMIDI_CC_PRESSURE_SLOPE2 = 106;
static const int        iMIDI_CC_PRESSURE_PAD2A = 107;
static const int        iMIDI_CC_PRESSURE_PAD2B = 108;
static const int        iMIDI_CC_PRESSURE_TOESWITCH2 = 109;

// the values that we'll read from the U3
    // Footnote #1
        static double T1 = 0;
        static double V1 = 0;
        static double V2 = 0;
        static double V3 = 0;
        static double V4 = 0;
        static volatile int TS1 = 0;
        static volatile int KG1A = 0;
        static volatile int KG1B = 0;
    // Footnote #2
        static double T2 = 0;
        static double V5 = 0;
        static double V6 = 0;
        static double V7 = 0;
        static double V8 = 0;
        static volatile int TS2 = 0;
        static volatile int KG2A = 0;
        static volatile int KG2B = 0;

// the derived MIDI note commands
static unsigned char chVelocity1 = 82;
static unsigned char chVelocity2 = 82;
static unsigned char chNote1ON[3] = { iMIDI_NOTE_ON, (unsigned char)0, chVelocity1 };
static unsigned char chNote1OFF[3] = { iMIDI_NOTE_OFF, (unsigned char)0, chVelocity1 };
static unsigned char chNote2ON[3] = { iMIDI_NOTE_ON, (unsigned char)0, chVelocity2 };
static unsigned char chNote2OFF[3] = { iMIDI_NOTE_OFF, (unsigned char)0, chVelocity2 };

// the derived MIDI pressure commands
static unsigned char chPressure1A[3] = { iMIDI_CHANNEL_PRESSURE, iMIDI_CC_PRESSURE_PAD1A, (unsigned char)0 };
static unsigned char chPressure1B[3] = { iMIDI_CHANNEL_PRESSURE, iMIDI_CC_PRESSURE_PAD1B, (unsigned char)0 };
static unsigned char chPressure2A[3] = { iMIDI_CHANNEL_PRESSURE, iMIDI_CC_PRESSURE_PAD2A, (unsigned char)0 };
static unsigned char chPressure2B[3] = { iMIDI_CHANNEL_PRESSURE, iMIDI_CC_PRESSURE_PAD2B, (unsigned char)0 };
static unsigned char chPressure1Aold[3] = { iMIDI_CHANNEL_PRESSURE, iMIDI_CC_PRESSURE_PAD1A, (unsigned char)0 };
static unsigned char chPressure1Bold[3] = { iMIDI_CHANNEL_PRESSURE, iMIDI_CC_PRESSURE_PAD1B, (unsigned char)0 };
static unsigned char chPressure2Aold[3] = { iMIDI_CHANNEL_PRESSURE, iMIDI_CC_PRESSURE_PAD2A, (unsigned char)0 };
static unsigned char chPressure2Bold[3] = { iMIDI_CHANNEL_PRESSURE, iMIDI_CC_PRESSURE_PAD2B, (unsigned char)0 };

// the derived MIDI slope commands
static unsigned char chSlope1[3] = { iMIDI_CHANNEL_SLOPE, iMIDI_CC_PRESSURE_SLOPE1, (unsigned char)0 };
static unsigned char chSlope2[3] = { iMIDI_CHANNEL_SLOPE, iMIDI_CC_PRESSURE_SLOPE2, (unsigned char)0 };
static unsigned char chSlope1old[3] = { iMIDI_CHANNEL_SLOPE, iMIDI_CC_PRESSURE_SLOPE1, (unsigned char)0 };
static unsigned char chSlope2old[3] = { iMIDI_CHANNEL_SLOPE, iMIDI_CC_PRESSURE_SLOPE2, (unsigned char)0 };

// the note memory values
static volatile int         i_PIN_EIO2_WAS_HIGH = (-1);
static volatile int         i_PIN_EIO6_WAS_HIGH = (-1);

// the KG high time values
volatile TIMER_TYPE         t_KG2B_HIGH;
volatile TIMER_TYPE         t_KG2A_HIGH;
volatile TIMER_TYPE         t_KG1B_HIGH;
volatile TIMER_TYPE         t_KG1A_HIGH;

// local prototypes
static BOOL         WINAPI consoleHandler(DWORD signal);
static char *       TimeStr();
static void         ErrorHandler(LJ_ERROR lngErrorcode, long lngLineNumber, long lngIteration);
static void         InitLabJackDevice(void);
static void         InitSignalValues(void);
static void         InitSignalBuffer(void);
static void         GetTimerValues(void);
static void         GetSignalValues(int iNumScansReceived);
static void         InitMidiDevice(void);
static void         CloseMidiDevice(void);
static void         SendMidiCommand(unsigned char *chCommand, unsigned char *chCommandOld, int iNumBytes);
static TIMER_TYPE	GetClock();
static TIMER_TYPE	GetClockTicksPerSec();
static int          Prop(double x, double y, double s, double *dRatio);

// local data
static volatile double      dNumSecs2 = 0.0;
static volatile double      dNumSecs1 = 0.0;
static double               dRatio2 = 0.0;
static double               dRatio1 = 0.0;

int main()
{
    try
    {
        int         iResult = 0;
        int         iDone = 0;
        long        lngErrorcode = 0;
        double      dblValue = 0;        
        double      numScansRequested;
        TIMER_TYPE  tClockTicksPerSec = GetClockTicksPerSec();
        FILE        *fCSV=NULL;        
        long        lNumCsvLines = 1;

        // init
        _unlink(CSV_FILE);
        if (fopen_s(&fCSV, CSV_FILE, "a") != 0) fCSV = NULL;
        else if (fCSV != NULL) 
        {
            fprintf(fCSV, "Timer(**) at Which K2B Goes HIGH,Timer(**) at Which K2A Goes HIGH,Ticks per Sec,# Seconds Between K2B-High & K2A-High,MIN_KG2_VELOCITY_SECS,MAX_KG2_VELOCITY_SECS,dNumSecs2,dRatio2 (Excel Calc),Volume (Excel Calc),dRatio2 (Test1.exe Calc),Volume (Test1.exe Calc)\n");
        }

        // trap & ignore Ctrl-C
        SetConsoleCtrlHandler((PHANDLER_ROUTINE)consoleHandler, TRUE);

        // init LabJack device
        InitLabJackDevice();

        // init Midi device
        InitMidiDevice();

        // start the stream
        lngErrorcode = eGetPtr(lngHandle, LJ_ioSTART_STREAM, 0, &dblValue, 0);
        ErrorHandler(lngErrorcode, __LINE__, 0);

        // report scan rate
        printf("%s: Actual Scan Rate = %.3f\n", TimeStr(), dblValue);
        printf("%s: Actual Sample Rate = %.3f\n", TimeStr(), 2 * dblValue);

        // read stream data until keypress
        TIMER_TYPE      t1 = GetClock(), t2 = GetClock();
        long            i = 0;        
        while (!_kbhit())
        {
            // init scan buffer
            InitSignalBuffer();

            // read-in all scans
            numScansRequested = NUM_SCANS_PER_ITERATION;
            lngErrorcode = eGetPtr(lngHandle, LJ_ioGET_STREAM_DATA, LJ_chALL_CHANNELS, &numScansRequested, adblData);

            // report iteration #
            //printf("\n%s: Iteration # %ld\n", TimeStr(), (i + 1));
            //printf("%s: Number read = %.0f\n", TimeStr(), numScansRequested);            

            // parse-out returned scan values & derive resultant midi notes
            GetSignalValues((int)numScansRequested);                        

            // report to screen
            /*
            printf( "%s: V1 = %.3f, V2 = %.3f, V3 = %.3f, V4 = %.3f, V5 = %.3f, V6 = %.3f, V7 = %.3f, V8 = %.3f\n                          "\
                    "    TS1 = %d, KG1A = %d, KG1B = %d, T1 = %.3f, TS2 = %d, KG2A = %d, KG2B = %d, T2 = %.3f\n",
                TimeStr(),
                V1, V2, V3, V4, V5, V6, V7, V8,
                TS1, KG1A, KG1B, T1, TS2, KG2A, KG2B, T2);
            printf("%s: MIDI_NOTE1 = %d, MIDI_NOTE2 = %d\n", TimeStr(), (int)chNote1ON[1], (int)chNote2ON[1]);
            */

            // dump to file
            if (KG2B == 1)
            {
                if (fCSV != NULL)
                {
                    lNumCsvLines++;
                    fprintf(fCSV,   "%I64d,%I64d,%I64d,=((a%ld-b%ld)/c%ld),%1.6f,%1.6f,%1.6f,\"=IF(G%ld<=E%ld,1,IF(G%ld>=(F%ld-E%ld),0,G%ld/(F%ld-E%ld)))\",=127-(127*H%ld),%1.6f,%d\n", 
                                    t_KG2B_HIGH, t_KG2A_HIGH, tClockTicksPerSec, lNumCsvLines, lNumCsvLines, lNumCsvLines, 
                                    MIN_KG2_VELOCITY_SECS, MAX_KG2_VELOCITY_SECS,
                                    dNumSecs2,
                                    lNumCsvLines, lNumCsvLines, lNumCsvLines, lNumCsvLines, lNumCsvLines, lNumCsvLines, lNumCsvLines, lNumCsvLines,
                                    lNumCsvLines,
                                    dRatio2,
                                    (int)chVelocity2);
                }
            }

            // send pressure commands
            SendMidiCommand(chPressure1A, chPressure1Aold, _countof(chPressure1A));
            SendMidiCommand(chPressure1B, chPressure1Bold, _countof(chPressure1B));
            SendMidiCommand(chPressure2A, chPressure2Aold, _countof(chPressure2A));
            SendMidiCommand(chPressure2B, chPressure2Bold, _countof(chPressure2B));

            // send slope commands
            SendMidiCommand(chSlope1, chSlope1old, _countof(chSlope1));
            SendMidiCommand(chSlope2, chSlope2old, _countof(chSlope2));

            // play KG1B
            if (KG1B > 0)
            {               
                switch (i_PIN_EIO2_WAS_HIGH)
                {
                case (-1):
                    midiout->sendMessage(chNote1ON, _countof(chNote1ON));
                    chNote1OFF[1] = chNote1ON[1];
                    i_PIN_EIO2_WAS_HIGH = 1;
                    break;
                case 1:
                    i_PIN_EIO2_WAS_HIGH = 1;
                    break;
                }
            }
            else
            {
                if (i_PIN_EIO2_WAS_HIGH > 0) midiout->sendMessage(chNote1OFF, _countof(chNote1OFF));
                i_PIN_EIO2_WAS_HIGH = (-1);
            }

            // play KG2B
            if (KG2B > 0)
            {                
                switch (i_PIN_EIO6_WAS_HIGH)
                {
                case (-1):                    
                    midiout->sendMessage(chNote2ON, _countof(chNote2ON));
                    chNote2OFF[1] = chNote2ON[1];
                    i_PIN_EIO6_WAS_HIGH = 1;
                    break;
                case 1:
                    i_PIN_EIO6_WAS_HIGH = 1;
                    break;
                }
            }
            else
            {
                if (i_PIN_EIO6_WAS_HIGH > 0) midiout->sendMessage(chNote2OFF, _countof(chNote2OFF));
                i_PIN_EIO6_WAS_HIGH = (-1);
            }

            // retrieve (and report) the current backlog
            //double      dblCommBacklog;
            //lngErrorcode = eGet(lngHandle, LJ_ioGET_CONFIG, LJ_chSTREAM_BACKLOG_COMM, &dblCommBacklog, 0);
            //double dPctBacklog = (100 * dblCommBacklog / (2.0 * 5.0 * SCAN_FREQUENCY));
            //if (dPctBacklog >= 10.0) printf("%s: Comm Backlog = %.1f%%\n", TimeStr(), dPctBacklog);

            // next iteration
            i++;
        }

        // report performance
        t2 = GetClock();
        TIMER_TYPE  llNumSecs = ((TIMER_TYPE)(t2 - t1)) / tClockTicksPerSec;
        printf("\n%s: # iterations       = %ld\n", TimeStr(), i);
        printf("%s: # seconds          = %lld\n", TimeStr(), llNumSecs);
        printf("%s: Secs per iteration = %.6f\n", TimeStr(), ((double)llNumSecs / (double)i));

        // stop the stream
        lngErrorcode = eGetPtr(lngHandle, LJ_ioSTOP_STREAM, 0, 0, 0);
        ErrorHandler(lngErrorcode, __LINE__, 0);

        // clean up & return
        CloseMidiDevice();
        if (fCSV != NULL)
        {
            fprintf(fCSV, "min=,=min(d1:d%ld)\n", lNumCsvLines);
            fprintf(fCSV, "max=,=max(d1:d%ld)\n", lNumCsvLines);
            fprintf(fCSV, "avg=,=average(d1:d%ld)\n", lNumCsvLines);
            fclose(fCSV);
        }
        return(iResult);
    }
    catch (...)
    {
    }
}/*main*/

static void     InitMidiDevice(void)
//
// SUBROUTINE:	InitMidiDevice
// PURPOSE:		Initialize our MIDI device via RtMidi() object interface
//
// INPUTS:		None
// OUTPUTS:		None
//
// RETURN VALUE:	None
//					program terminates if initialize fails
//
{
    // create the MIDI interface
    try 
    {
        midiout = new RtMidiOut();
    }
    catch (RtMidiError& error) 
    {
        printf("%s: Error in InitMidiDevice() init: %s\n", TimeStr(), error.getMessage().c_str());
        exit(EXIT_FAILURE);
    }

    // open MIDI port
    try
    {
        midiout->openPort(iMidiPortNum);
    }
    catch (RtMidiError& error)
    {
        printf("%s: Error in InitMidiDevice() open: %s\n", TimeStr(), error.getMessage().c_str());
        exit(EXIT_FAILURE);
    }
}/*InitMidiDevice*/

static void     CloseMidiDevice(void)
//
// SUBROUTINE:	CloseMidiDevice
// PURPOSE:		Close our MIDI device via RtMidi() object interface
//
// INPUTS:		None
// OUTPUTS:		None
//
// RETURN VALUE:	None
//
{
    // check
    if (midiout == NULL) return;

    // close the MIDI interface
    try
    {
        midiout->closePort();
        delete midiout;
    }
    catch (...)
    {
        
    }

    // clean up & return
    midiout = NULL;
    return;
}/*CloseMidiDevice*/

static void     InitLabJackDevice(void)
//
// SUBROUTINE:	InitLabJackDevice
// PURPOSE:		Initialize our LabJack device and stream interface
//
// INPUTS:		None
// OUTPUTS:		None
//
// RETURN VALUE:	None
//					program terminates if initialize fails
//
{
    double      dblValue;
    long        lngErrorcode;

    //Read and display the UD version.
    dblValue = GetDriverVersion();
    printf("%s: UD Driver Version = %.3f\n\n", TimeStr(), dblValue);

    //Open the first found LabJack U3.
    lngErrorcode = OpenLabJack(LJ_DEVICE_TYPE, LJ_DEVICE_INTERFACE, LJ_DEVICE_NUM, 1, &lngHandle);
    ErrorHandler(lngErrorcode, __LINE__, 0);

    //Read and display the hardware version of this U3.
    lngErrorcode = eGet(lngHandle, LJ_ioGET_CONFIG, LJ_chHARDWARE_VERSION, &dblValue, 0);
    printf("%s: U3 Hardware Version = %.3f\n\n", TimeStr(), dblValue);
    ErrorHandler(lngErrorcode, __LINE__, 0);

    //Read and display the firmware version of this U3.
    lngErrorcode = eGet(lngHandle, LJ_ioGET_CONFIG, LJ_chFIRMWARE_VERSION, &dblValue, 0);
    printf("%s: U3 Firmware Version = %.3f\n\n", TimeStr(), dblValue);
    ErrorHandler(lngErrorcode, __LINE__, 0);

    // First add requests that will initialize the device
        //Start by using the pin_configuration_reset IOType so that all
        //pin assignments are in the factory default condition.
        lngErrorcode = ePut(lngHandle, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0);
        ErrorHandler(lngErrorcode, __LINE__, 0);

        //Configure [FIO0 ... FIO7] as analog, all else as digital.  That means we
        //will start from channel 0 and update all 16 flexible bits.  We will
        //pass a value of: 0000 0000 1111 1111 = 0x00ff = 255.
        lngErrorcode = ePut(lngHandle, LJ_ioPUT_ANALOG_ENABLE_PORT, 0, 0x00ff, 16);
        ErrorHandler(lngErrorcode, __LINE__, 0);        

        //Set DAC0 to correct # volts.
        lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_DAC, 0, DAC0_VOLTAGE_VALUE, 0, 0);
        ErrorHandler(lngErrorcode, __LINE__, 0);

    // Now add requests that will be processed every iteration of the loop.        
        //Configure the stream:
            // Set the scan rate.
            lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chSTREAM_SCAN_FREQUENCY, SCAN_FREQUENCY, 0, 0);
            ErrorHandler(lngErrorcode, __LINE__, 0);

            // Give the driver a 5 second buffer (scanRate * 2 channels * 5 seconds).
            lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chSTREAM_BUFFER_SIZE, (double)(1.0 * SCAN_FREQUENCY * 2 * 5.0), 0, 0);
            ErrorHandler(lngErrorcode, __LINE__, 0);

            // Configure read mode
            lngErrorcode = AddRequest(lngHandle, LJ_ioPUT_CONFIG, LJ_chSTREAM_WAIT_MODE, LJ_swSLEEP, 0, 0);
            ErrorHandler(lngErrorcode, __LINE__, 0);

            // define the scan list as [FIO0 ... FIO7] ...
            lngErrorcode = AddRequest(lngHandle, LJ_ioCLEAR_STREAM_CHANNELS, 0, 0, 0, 0);
            ErrorHandler(lngErrorcode, __LINE__, 0);
            for (int iChannel = 0; (iChannel < 8); iChannel++)
            {
                lngErrorcode = AddRequest(lngHandle, LJ_ioADD_STREAM_CHANNEL, iChannel, 0, 0, 0);
                ErrorHandler(lngErrorcode, __LINE__, iChannel);
            
            }
            // ... then the input states of 16 bits of digital I/O
            lngErrorcode = AddRequest(lngHandle, LJ_ioADD_STREAM_CHANNEL, 193, 0, 0, 0);
            ErrorHandler(lngErrorcode, __LINE__, 193);

        //Execute the requests.
            lngErrorcode = GoOne(lngHandle);
            ErrorHandler(lngErrorcode, __LINE__, 0);

}/*InitLabJackDevice*/

static char *   TimeStr()
//
// SUBROUTINE:	TimeStr
// PURPOSE:		Return a C-style "time string" which displays milliseconds
//
// INPUTS:		None
// OUTPUTS:		None
//
// RETURN VALUE:	pointer to (static storage) of C-style time string
//
{
    char                nowStr[1024];
    struct _timeb		timebuffer;
    static char         szTime[1024];

    // get time
    _ftime_s(&timebuffer);
    ctime_s(nowStr, sizeof(nowStr) - 1, &(timebuffer.time));
    nowStr[strlen(nowStr) - 1] = 0;

    // compose entire message
    sprintf_s(szTime, "%.19s.%03hu %s", nowStr, timebuffer.millitm, &nowStr[20]);

    // return time
    return(szTime);
}/*TimeStr*/

static void ErrorHandler(LJ_ERROR lngErrorcode, long lngLineNumber, long lngIteration)
// SUBROUTINE:	ErrorHandler
// PURPOSE:		LabJack device error hadling and reporting
//
// INPUTS:		lngErrorcode  - the LabJack device error code
//              lngLineNumber - the source line # at which error occurred
//              lngIteration  - the iteration (if any) at which the error occurred
// OUTPUTS:		None
//
// RETURN VALUE:	None
//
{
    char        err[2028];

    if (lngErrorcode != LJE_NOERROR)
    {
        ErrorToString(lngErrorcode, err);
        printf("Error number = %d\n", lngErrorcode);
        printf("Error string = %s\n", err);
        printf("Source line number = %d\n", lngLineNumber);
        if (lngIteration > 0) printf("Iteration = %d\n\n", lngIteration);
        //if (lngErrorcode > LJE_MIN_GROUP_ERROR)
        {
            exit(lngErrorcode);
        }
    }
}/*ErrorHandler*/

static BOOL WINAPI consoleHandler(DWORD signal) 
//
// SUBROUTINE:	consoleHandler
// PURPOSE:		C runtime signal trap; used to trap & ignore Ctrl-C and Ctrl-Break (as these leave the LJ in a unrecoverable state)
//
// INPUTS:		signal  - console signal code
// OUTPUTS:		None
//
// RETURN VALUE:	None
//
{
    switch (signal)
    {
        // Handle the CTRL-C signal. 
    case CTRL_C_EVENT:
        return(TRUE);

        // CTRL-CLOSE: confirm that the user wants to exit. 
    case CTRL_CLOSE_EVENT:
        return(TRUE);

        // Pass other signals to the next handler. 
    case CTRL_BREAK_EVENT:
        return TRUE;

    case CTRL_LOGOFF_EVENT:
        return FALSE;

    case CTRL_SHUTDOWN_EVENT:
        return FALSE;

    default:
        return FALSE;
    }
}/*consoleHandler*/

static TIMER_TYPE		GetClock()
// SUBROUTINE:	GetClock
// PURPOSE:		Return the number of microseconds that have elapsed since the system was started, up to 49.7 days.
//
// INPUTS:		None
// OUTPUTS:		None
//
// RETURN VALUE:	# of microseconds
//
//
{
    LARGE_INTEGER		ii;
    TIMER_TYPE			i;

    ZERO(ii);
    QueryPerformanceCounter(&ii);
    memcpy(&i, &ii, sizeof(i));
    return(i);
}/*GetClock*/

static TIMER_TYPE		GetClockTicksPerSec()
// SUBROUTINE:	GetClockTicksPerSec
// PURPOSE:		Return the number of clock ticks per second
//
// INPUTS:		None
// OUTPUTS:		None
//
// RETURN VALUE:	ticks/sec
//
//
{
    LARGE_INTEGER		ii;
    TIMER_TYPE			i;

    ZERO(ii);
    QueryPerformanceFrequency(&ii);
    memcpy(&i, &ii, sizeof(i));
    return(i);
}/*GetClockTicksPerSec*/


static void     InitSignalBuffer(void)
//
// SUBROUTINE:	InitSignalBuffer
// PURPOSE:		Initialize the (global) scan buffer array
//
// INPUTS:		None
// OUTPUTS:		None
//
// RETURN VALUE:	None
//
{
    for (int k = 0; (k < _countof(adblData)); k++)
    {
        adblData[k] = NO_SCAN_VALUE;
    }
}/*InitSignalBuffer*/

static void     InitSignalValues(void)
//
// SUBROUTINE:	InitSignalValues
// PURPOSE:		Initialize the (global) LJ voltage values and MIDI notes
//
// INPUTS:		None
// OUTPUTS:		None
//
// RETURN VALUE:	None
//
{
    // Footnote #1
    V1 = 0;
    V2 = 0;
    V3 = 0;
    V4 = 0;
    TS1 = 0;
    KG1A = 0;
    KG1B = 0;
    T1 = 0;
    // Footnote #2
    V5 = 0;
    V6 = 0;
    V7 = 0;
    V8 = 0;
    TS2 = 0;
    KG2A = 0;
    KG2B = 0;
    T2 = 0;
}/*InitSignalValues*/

static double   GetVoltageFromBuffer(int iIndex, int *iCounter)
//
// SUBROUTINE:	GetVoltageFromBuffer
// PURPOSE:		Extract a voltage value from the (global) adblData[] buffer
//
// INPUTS:		iIndex      - the index (within adblData[]) of the value to extract
// OUTPUTS:		*iCounter   - if the iIndex'th value in adblData[] is a valid value (i.e., <> NO_SCAN_VALUE) then this is incremented by 1
//
// RETURN VALUE:	if the iIndex'th value in adblData[] is a valid value, then this value is returned otherwise 0.0 is returned
//
{
    double dRes = 0.0;

    if ((dRes = adblData[iIndex]) == NO_SCAN_VALUE) 
    {
        dRes = 0.0;
    }
    else
    {
        *iCounter += 1;
    }
    return(dRes);
}/*GetVoltageFromBuffer*/

static void         SendMidiCommand(unsigned char *chCommand, unsigned char *chCommandOld, int iNumBytes)
//
// SUBROUTINE:	SendMidiCommand
// PURPOSE:		Send a MIDI command if it is different from the command previously sent
//
// INPUTS:		chCommand      - the MIDI command bytes
//              iNumBytes      - the # of bytes in the command
// OUTPUTS:		chCommandOld   - if the chCommand is sent (as a result of being different from chCommandOld) then chCommand is copied into chCommandOld
//
// RETURN VALUE:	None
//
{
    if (memcmp(chCommand, chCommandOld, iNumBytes) != 0)
    {
        midiout->sendMessage(chCommand, iNumBytes);
        memcpy(chCommandOld, chCommand, iNumBytes);
    }
}/*SendMidiCommand*/

static void     GetSignalValues(int iNumScansReceived)
//
// SUBROUTINE:	GetSignalValues
// PURPOSE:		Parse-out the  LJ voltage values from the "adblData[]" stream and derive the corresponding MIDI notes
//
// INPUTS:		iNumScansReceived - # of scans in the "adblData[]" stream
// OUTPUTS:		None
//
// RETURN VALUE:	None
//
{
    int             iCounter[NUM_CHANNELS];
    std::string     strEIO_FIO;

    // init
    ZERO(iCounter);
    InitSignalValues();

    // get voltage values from analog ports [FIO-0 ... FIO-7]
    // see: https://forums.labjack.com/index.php?showtopic=4279
    if (iNumScansReceived > 0)
    {        
        for (int i = 0; (i < iNumScansReceived); i++)
        {
            // get the [FIO-0 ... FIO-7] analog values
            V1 += GetVoltageFromBuffer(NUM_CHANNELS * i + 0, &iCounter[0]);
            V2 += GetVoltageFromBuffer(NUM_CHANNELS * i + 1, &iCounter[1]);
            V3 += GetVoltageFromBuffer(NUM_CHANNELS * i + 2, &iCounter[2]);
            V4 += GetVoltageFromBuffer(NUM_CHANNELS * i + 3, &iCounter[3]);
            V5 += GetVoltageFromBuffer(NUM_CHANNELS * i + 4, &iCounter[4]);
            V6 += GetVoltageFromBuffer(NUM_CHANNELS * i + 5, &iCounter[5]);
            V7 += GetVoltageFromBuffer(NUM_CHANNELS * i + 6, &iCounter[6]);
            V8 += GetVoltageFromBuffer(NUM_CHANNELS * i + 7, &iCounter[7]);

            // get the [EIO-0 ... EIO-7] digital values
            if (i == 0)
            {
                unsigned short int      c = (unsigned short int)GetVoltageFromBuffer(NUM_CHANNELS * i + 8, &iCounter[8]);
                strEIO_FIO = std::bitset<16>(c).to_string();                
            }
        }
        
        V1 = (V1 / (1.0 * iCounter[0]));
        V2 = (V2 / (1.0 * iCounter[1]));
        V3 = (V3 / (1.0 * iCounter[2]));
        V4 = (V4 / (1.0 * iCounter[3]));

        V5 = (V5 / (1.0 * iCounter[4]));
        V6 = (V6 / (1.0 * iCounter[5]));
        V7 = (V7 / (1.0 * iCounter[6]));
        V8 = (V8 / (1.0 * iCounter[7]));
    }

    // get digital port values from [EIO-0 ... EIO-7]
    // see:  https://labjack.com/support/datasheets/u3/operation/stream-mode/digital-inputs-timers-counters
    std::string     strEIO  = strEIO_FIO.substr(0, 8);
    std::string     strEIO7 = strEIO.substr(0, 1);
    std::string     strEIO6 = strEIO.substr(1, 1);
    std::string     strEIO5 = strEIO.substr(2, 1);
    std::string     strEIO4 = strEIO.substr(3, 1);
    std::string     strEIO3 = strEIO.substr(4, 1);
    std::string     strEIO2 = strEIO.substr(5, 1);
    std::string     strEIO1 = strEIO.substr(6, 1);
    std::string     strEIO0 = strEIO.substr(7, 1);

    //printf("%s: strEIO0 = %s\n", TimeStr(), strEIO0.c_str());
    //printf("%s: strEIO1 = %s\n", TimeStr(), strEIO1.c_str());
    //printf("%s: strEIO2 = '%s'\n", TimeStr(), strEIO2.c_str());
    //printf("%s: strEIO3 = '%s'\n", TimeStr(), strEIO3.c_str());
    //printf("%s: strEIO4 = %s\n", TimeStr(), strEIO4.c_str());
    //printf("%s: strEIO5 = %s\n", TimeStr(), strEIO5.c_str());
    //printf("%s: strEIO6 = '%s'\n", TimeStr(), strEIO6.c_str());
    //printf("%s: strEIO7 = %s\n", TimeStr(), strEIO7.c_str());

    KG1A = (strEIO3.compare("1") == 0 ? 1 : 0);     // note: document reads KG1A is from EIO1; 10/08/2020 - we changed to EIO3
    KG1B = (strEIO2.compare("1") == 0 ? 1 : 0);

    KG2A = (strEIO5.compare("1") == 0 ? 1 : 0);
    KG2B = (strEIO6.compare("1") == 0 ? 1 : 0);

    // derive resulting midi notes
    chNote1ON[1] = (unsigned char)int((21 * V1) + 36);
    chNote2ON[1] = (unsigned char)int((21 * V5) + 36);

    // store KG2 high times
    if (KG2A == 1)
    {
        t_KG2A_HIGH = GetClock();
        t_KG2B_HIGH = 0;
    }
    if ((KG2B == 1) && (t_KG2B_HIGH == 0))
    {
        if (t_KG2A_HIGH == 0)
        {
            KG2B = 0;
        }
        else
        {
            t_KG2B_HIGH = GetClock();
        }        
    }

    // compute KG2 velocity
    if ((t_KG2A_HIGH != 0) && (t_KG2B_HIGH != 0))
    {        
        dRatio2 = 0.0;
        dNumSecs2 = (((double)1.0 * t_KG2B_HIGH) - ((double)1.0 * t_KG2A_HIGH)) / ((double)1.0 * GetClockTicksPerSec());

        int k = Prop(MIN_KG2_VELOCITY_SECS, MAX_KG2_VELOCITY_SECS, dNumSecs2, &dRatio2);

        chVelocity2 = (unsigned char)(127 - k);

        printf("V5 = %1.4f  NOTE2 = %d:\t\tdNumSecs2 = %1.6f\tk = %d\tchVelocity2 = %d\n", V5, (int)chNote2ON[1], dNumSecs2, k, (int)chVelocity2);

        t_KG2A_HIGH = 0;
        t_KG2B_HIGH = 0;        
    }

    // store KG1 high times
    if (KG1A == 1)
    {
        t_KG1A_HIGH = GetClock();
        t_KG1B_HIGH = 0;
    }
    if ((KG1B == 1) && (t_KG1B_HIGH == 0))
    {
        if (t_KG1A_HIGH == 0)
        {
            KG1B = 0;
        }
        else
        {
            t_KG1B_HIGH = GetClock();
        }
    }

    // compute KG1 velocity
    if ((t_KG1A_HIGH != 0) && (t_KG1B_HIGH != 0))
    {
        dRatio1 = 0.0;
        dNumSecs1 = (((double)1.0 * t_KG1B_HIGH) - ((double)1.0 * t_KG1A_HIGH)) / ((double)1.0 * GetClockTicksPerSec());

        int k = Prop(MIN_KG1_VELOCITY_SECS, MAX_KG1_VELOCITY_SECS, dNumSecs1, &dRatio1);

        chVelocity1 = (unsigned char)(127 - k);

        printf("V1 = %1.4f  NOTE1 = %d:\t\tdNumSecs1 = %1.6f\tk = %d\tchVelocity1 = %d\n", V1, (int)chNote1ON[1], dNumSecs1, k, (int)chVelocity1);

        t_KG1A_HIGH = 0;
        t_KG1B_HIGH = 0;
    }

    // set resulting midi velocities
    chNote1ON[2] = chVelocity1;
    chNote2ON[2] = chVelocity2;

    // derive resulting Pressure Pad 1A, Pressure Pad 1B, Pressure Pad 2A, and Pressure Pad 2B
    chPressure1A[2] = (unsigned char)Prop(0, DAC0_VOLTAGE_VALUE, V3, NULL);
    chPressure1B[2] = (unsigned char)Prop(0, DAC0_VOLTAGE_VALUE, V4, NULL);
    chPressure2A[2] = (unsigned char)Prop(0, DAC0_VOLTAGE_VALUE, V7, NULL);
    chPressure2B[2] = (unsigned char)Prop(0, DAC0_VOLTAGE_VALUE, V8, NULL);

    // derive resulting Slope1 and Slope2
    chSlope1[2] = (unsigned char)Prop(0, DAC0_VOLTAGE_VALUE, V2, NULL);
    chSlope2[2] = (unsigned char)Prop(0, DAC0_VOLTAGE_VALUE, V6, NULL);
}/*GetSignalValues*/

static int          Prop(double x, double y, double s, double *dRatio)
//
// SUBROUTINE:	Prop
// PURPOSE:		Derive an integer values that sits in [0..127] in the same proportion (ratio) as a specified real in a specified range of reals
//
// INPUTS:		x - lower bound of range of reals
//              y - upper bound of range of reals
//              s - a real value; x <= s <= y
// OUTPUTS:		dRatio - if non-NULL then this will hold the derived ratio
//
// RETURN VALUE:	an integer in [0..127]
//
{
    int         k = 0;
    double      displace = 0.0;
    double      prop = 0.0;

    if (x != y) 
    {
        displace = abs(x);
        if (s < x) s = x;
        else if (s > y) s = y;
        if (x < 0)
        {
            if (abs(x) < abs(y))
            {
                displace = abs(x);
                prop = (s + displace) / (y + displace);
            }
            else if (abs(x) > abs(y))
            {
                displace = abs(y);
                prop = (s + displace) / (x + displace);
            }
            else
            {
                displace = abs(y);
                prop = (s + displace) / (2 * y);
            }
        }
        else
        {
            if (abs(x) < abs(y))
            {
                displace = abs(x);
                prop = (s - displace) / (y - displace);
            }
            else if (abs(x) > abs(y))
            {
                displace = abs(y);
                prop = (s - displace) / (x - displace);
            }
            else
            {
                displace = abs(y);
                prop = (s + displace) / (2 * y);
            }
        }
    }    
    if (dRatio != NULL) *dRatio = prop;
    k = (int)(prop * 127);
    return(k);
}/*Prop*/
