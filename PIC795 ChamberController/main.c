/********************************************************************
* FILENAME: main.c
* PROJECT:  PIC795_ChamberController
*      
* Processor:    PIC32MX795F512L on UBW32 Board
* Compiler:     Microchip XC32 V1.30 MPLABX IDE: V6.00 
* 
* First test version fires two 110V AC triacs to power wet and dry air pumps 
* for setting the humidity in a chamber monitored by a hygrometer.
* The hygrometer transmits humidity RH readings via an RS232 input to the PIC microcontroller.
* This firmware compares the measured RH to the humidity setpoint 
* and sets the triac phase for the air pumps.
* Closed loop PID control is implemented to provide stable humidity in the chamber.
* A PWM output is also provided for a fan to evenly circulate water vapor in the chamber.
* The PWM duty cycle is controlled by pot input to an AD converter.
*
* PIC LCD pin assignments:
* LCD DATA: PIC RA0-RA3 connected to LCD display pins D4, D5, D6, D7
* 
* LCD_RS: RE0
* LCD_RW: RE1
* LCD_EN: RE2 
* 
* 8-21-20  JBS Written, debugged, tested, stored on Git
* 7-19-22  Modified to work with PORT E for control and PORT A for data
*          using New Haven NHD-0420H1Z-FSW-GBW 4x20 display.
* 7-22-22: Added GetDS3231Date() and AdjustDS3231Date()
* 7-23-22: 
 * 8-4-22: 
 * 8-6-22: Added fan speed control
 * 8-7-22: Tuning PID: Achievable range with four fish tank stone bubblers: 30% - 62%
 *         KP = 20, KI = 1, KD = 8, OFFSET = 100, FILTERSIZE = 8

 * 
********************************************************************/
#include "LCD_4BIT_PIC32.h"
#include "I2C_4BUS_EEPROM_PIC32.h"
#include "Defs.h"
#include "DS3231.h"
#include "string.h"
#include "plib.h"
#include "Delay.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <proc/p32mx795f512l.h>


/** CONFIGURATION **************************************************/
#pragma config FSOSCEN   = ON           // Secondary osclllator enabled
#pragma config UPLLEN   = ON            // USB PLL Enabled
#pragma config FPLLMUL  = MUL_15        // PLL Multiplier
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select



// #define TRIAC_OUT1 LATDbits.LATD8 
#define TRIAC_OUT2 LATDbits.LATD9
#define TRIAC_OUT3 LATDbits.LATD10
#define TRIAC_OUT4 LATDbits.LATD11
#define FAN_PWM OC3RS

#define PWM_MAX 1023
#define PWM_MIN 0

#define MAXPOTS 4
#define TEST_OUT LATBbits.LATB0

#define CR 13
#define LF 10
#define BACKSPACE 8

// UART FOR PC SERIAL PORT
#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR

// UART FOR RH REFERENCE SERIAL PORT (EXTECH HYGRO THERMOMETER)
#define REFuart UART4
#define REFbits U4STAbits
#define REF_VECTOR _UART_4_VECTOR

enum {
    NO_COMMAND = 0,
    RUNMODE,
    SETTIME,
    SETDATE,
    SETKP,
    SETKI,
    SETKD,
    SETOFF,
    SETPOINT,
    SHOW
};

/** V A R I A B L E S ********************************************************/

float SumError = 0;

#define MAXSTRING 32
#define MAXCOMMANDS 10
const unsigned char *CommandList[MAXCOMMANDS] = {"NOCOMMAND", "RUN", "TIM", "DAT", "KP", "KI", "KD", "OFF", "STP", "SHW"};

void InitializeSystem(void);
int ADC10_ManualInit(void);
unsigned char ParseCommandString (unsigned char *ptrReceivedString, int *Command, float *ptrFloats, int *NumFloats, unsigned char *ptrString);


#define MAX_RX_BUFFERSIZE 256
unsigned char HOSTRxBuffer[MAX_RX_BUFFERSIZE];
unsigned char HOSTRxBufferFull = false;
unsigned char ControlCommand = 0;

unsigned char REFRxBuffer[MAX_RX_BUFFERSIZE];
unsigned char REFRxBufferFull = false;
unsigned short REFRxIndex = 0;

// #define MAX_DATA_INTEGERS 16
#define MAX_DATA_FLOATS 16

BYTE intFlag = false;
unsigned short ADresult[MAXPOTS];

unsigned short Timer1Reload, Timer2Reload, Timer3Reload, Timer4Reload, Timer5Reload;
unsigned char  EnableWetPump = false, EnableDryPump = false;

int RHControl(float RHSetpoint, float RHActual);

#define KP 20
#define KI 1
#define KD 8
#define OFFSET 100
float kP = KP, kI = KI, kD = KD, Offset = OFFSET;
float arrAccumulatedError[FILTERSIZE]; 

unsigned char RunMode = false;

void main(void)
{       
    int i, j, TestCounter = 0, intCounter = 0;
    unsigned char arrString[MAXSTRING];
    unsigned char strAMPM[2];
    unsigned char strLineOne[32];
    unsigned char strLineTwo[32];
    unsigned char TestChar = 'A';
    unsigned char ControlRegister;     

    int Command, NumData;    
    float arrNumericArgs[MAX_DATA_FLOATS];
    int Seconds = 0;
    float area;
    float theta;    
    int FanSpeed = 0;
    #define EXTECH_HEADER_LENGTH 6
    #define NUM_EXTECH_ZEROS 4
    #define EXTECH_DATA_LENGTH 4
    #define EXTECH_TOTAL_PACEKT_LENGTH (EXTECH_HEADER_LENGTH + NUM_EXTECH_ZEROS + EXTECH_DATA_LENGTH)
    #define START_EXTECH_DATA (EXTECH_HEADER_LENGTH + EXTECH_DATA_LENGTH)
    unsigned char strExtechHeader[EXTECH_HEADER_LENGTH+1];
    unsigned char strExtechData[EXTECH_DATA_LENGTH+1];
    int ExtechRxLength = 0;
    unsigned char ch, ExtechError = false;
    unsigned long ExtechRxCounter = 0;
    int PWMvalue = 0;
    
    #define MAXNUMBYTES 20
    unsigned char Line1String[MAXNUMBYTES+1] = "HEY Bopparee bop bop";
    unsigned char Line2String[MAXNUMBYTES+1] = "Diddle hay diddle do";
    unsigned char Line3String[MAXNUMBYTES+1] = "Dee dum dot wah wah!";
    unsigned char Line4String[MAXNUMBYTES+1] = "ONE MORE TIME 123456";
    float ExtechRefRH, ExtechRefTempC, ExtechRefTempF, RHSetpoint = 50, floADC;

    InitializeSystem();   
    for (i = 0; i < FILTERSIZE; i++) arrAccumulatedError[i] = 0;
    ADC10_ManualInit();
    
    DelayMs(200);    
    
    REF_DTR_OUT = REF_ENABLED;
    printf("\rTesting FULL PID... #1");
    DelayMs(200);
    
    while(1)
    {
        if (ControlCommand)
        {
            RunMode = false;
            printf("\rCTL: %d\r", ControlCommand);
            ControlCommand = ControlCommand + '@';
            if (ControlCommand == 'R')
            {
                printf("\rRUN MODE");
                RunMode = true;
            }
            else printf("\rKP: %.3f, KI: %.3f, KD: %.3f, OFF: %.0f\r", kP,kI,kD,Offset);
            ControlCommand = 0;
        }
        else if (HOSTRxBufferFull)
        {
            HOSTRxBufferFull = false;            
            if (RunMode)
            {
                RunMode = false;
                printf("\r\rENTER COMMANDS:\r");
            }
           else
            {
                printf("\rRECEIVED: %s", HOSTRxBuffer);       
                if (ParseCommandString (HOSTRxBuffer, &Command, arrNumericArgs, &NumData, arrString))
                {                    
                    printf("\rCommand: %d", Command);         
                    
                    if (Command == SETKP) kP = arrNumericArgs[0];
                    else if (Command == SETKI) kI = arrNumericArgs[0];
                    else if (Command == SETKD) kD = arrNumericArgs[0];
                    else if (Command == SETOFF) Offset = arrNumericArgs[0];
                    else if (Command == SETPOINT)
                    {
                        RHSetpoint = arrNumericArgs[0];
                        if (RHSetpoint > 100) RHSetpoint = 100;
                        else if (RHSetpoint < 0) RHSetpoint = 0;
                        printf("\rRH Setpoint: %.1f", RHSetpoint);
                    }
                    
                    
                    if (Command == SETKP || Command == SETKI || Command == SETKD || Command == SETOFF)
                        printf("\rKP: %.3f, KI: %.3f, KD: %.3f, OFF: %.1f\r", kP,kI,kD,Offset);
                    
                    /*
                    if (Command == SETTIME)
                    {
                        hours = arrNumericArgs[0];
                        minutes = arrNumericArgs[1];
                        seconds = arrNumericArgs[2];
                        printf("\rSET TIME: HOURS: %d, MINUTES: %d, SECONDS: %d, AMPM: %s", hours, minutes, seconds, arrString);
                        if (strstr ("PM", arrString)) PMflag = 1;
                        else PMflag = 0;
                        AdjustDS3231Time(hours, minutes, seconds, PMflag);
                    }
                    else if (Command == SETDATE)
                    {
                        month = arrNumericArgs[0];
                        day = arrNumericArgs[1];
                        year = arrNumericArgs[2];
                        printf("\rSET DATE: MONTH: %d, DAY: %d, YEAR: %d", month, day, year);                    
                        AdjustDS3231Date(month, day, year);
                    }
                    */
                }
                else printf("\rNo command");
                Command = 0;                                
                // previousSeconds = -1;                
            }
        }
        
        
        if (REFRxBufferFull)
        {
            REFRxBufferFull = false;
            ExtechError = false;
            ExtechRxLength = strlen(REFRxBuffer);            
            
            if (ExtechRxLength < EXTECH_TOTAL_PACEKT_LENGTH)
            {
                ExtechError = true;
                printf("\rEXTECH ERROR: RX string length: %d", ExtechRxLength);
            }
            else 
            {
                for (i = 0; i < EXTECH_TOTAL_PACEKT_LENGTH; i++)
                {
                    ch = REFRxBuffer[i];
                    if (!isdigit(ch))
                    {
                        ExtechError = true;
                        printf("\rEXTECH ERROR: non-numeric character #%d = %c %d", i, ch);
                        break;
                    }
                    else strExtechHeader[i] = ch;
                }
            }
            
            if (!ExtechError)
            {
                strExtechHeader[EXTECH_HEADER_LENGTH] = '\0';                
                for (i = EXTECH_HEADER_LENGTH; i < EXTECH_HEADER_LENGTH + NUM_EXTECH_ZEROS; i++)
                {
                    if (REFRxBuffer[i] != '0')
                    {
                        ExtechError = true;
                        printf("\rEXTECH ERROR: ZERO CHARACTER: %c", REFRxBuffer[i]);
                        break;
                    }
                }
                if (!ExtechError)
                {                
                    j = 0;
                    for (i = START_EXTECH_DATA; i < START_EXTECH_DATA + EXTECH_DATA_LENGTH; i++)
                    {
                        ch = REFRxBuffer[i];
                        strExtechData[j++] = ch;
                        strExtechData[EXTECH_DATA_LENGTH] = '\0';
                    }
                    if (0 == strcmp("410401", strExtechHeader))
                    {                        
                        ExtechRefRH = atof(strExtechData) / 10.0;
                        //printf("\r#%d REF RH: %.1f%%, ", ExtechRxCounter++, ExtechRefRH);
                    }
                    else if (0 == strcmp("420201", strExtechHeader))
                    {                        
                        ExtechRefTempF = atof(strExtechData) / 10.0;
                        //printf("TEMP: %.1f %cF", ExtechRefTempF, 248);
                    }
                    else if (0 == strcmp("420101", strExtechHeader))
                    {                        
                        ExtechRefTempC = atof(strExtechData) / 10.0;
                        ExtechRefTempF = (ExtechRefTempC * 9 / 5) + 32;
                        //printf("TEMP: %.1f %cF", ExtechRefTempF, 248);
                    }                    
                    else printf("\rUNRECOGNIZED STRING: %s", strExtechHeader);                    
                }
            }          
            
            IFS1bits.AD1IF = 0;
            while(!IFS1bits.AD1IF);        
            AD1CON1bits.ASAM = 0;        // Pause sampling. 
            for (i = 0; i < MAXPOTS; i++)            
                ADresult[i] = (unsigned short) ReadADC10(i); // read the result of channel 0 conversion from the idle buffer            
            AD1CON1bits.ASAM = 1;        // Restart sampling.            
            
            FAN_PWM = ADresult[0];            
                      
            if (RunMode)
            {
                PWMvalue = RHControl(RHSetpoint, ExtechRefRH);            
                if (PWMvalue == 0)
                {
                    EnableWetPump = false;
                    EnableDryPump = false;                
                }
                else if (PWMvalue > 0)
                {
                    EnableWetPump = true;
                    EnableDryPump = false;
                
                    area = (float) PWMvalue / 1024;
                    theta = acos ((2 * area)-1);                   
                    Timer3Reload = (unsigned short) (theta * 15625 / M_PI);                 
                    
                    Timer4Reload = 0;
                }
                else
                {
                    EnableWetPump = false;
                    EnableDryPump = true;
                
                    area = (float) (0-PWMvalue) / 1024;
                    theta = acos ((2 * area)-1);                   
                    Timer4Reload = (unsigned short) (theta * 15625 / M_PI);                 
            
                    Timer3Reload = 0;
                }
            }            
            else
            {
                EnableWetPump = false;
                EnableDryPump = false;
                TRIAC_OUT2 = TRIAC_OUT3 = TRIAC_OUT4 = 0;
            }
        }
    }
   
    /*
    TestChar = 'A';
    
    printf("\rTESTING TIME CLOCK DS3231 #1");
    
    LCDInit();
    LCDGoto(0, 0);
    LCDWriteArray(Line1String);
    LCDGoto(0, 1);
    LCDWriteArray(Line2String);
    LCDGoto(0, 2);
    LCDWriteArray(Line3String);
    LCDGoto(0, 3);
    LCDWriteArray(Line4String);
    
    
    printf("\rInitializing I2C3...");
    initI2C(I2C3);
    
    printf("\rEnabling battery backup (Clearing /EOSC bit) on DS3231:");    
    #define CONTROL_REGISTER_ADDRESS 0x0E
    DS3231ReadByte(I2C3, CONTROL_REGISTER_ADDRESS, &ControlRegister);
    ControlRegister = ControlRegister & 0b01111111;    // Clear /EOSC bit so oscillator will run if power is off.
    DS3231WriteByte(I2C3, CONTROL_REGISTER_ADDRESS, ControlRegister);       
*/
    
    while(1)
    {    
        
        if (intFlag)
        {
            intFlag = false;
            
            
        }
    }
}
        /*
        GetDS3231Time(&hours, &minutes, &seconds, &PMflag);
        GetDS3231Date(&month, &day, &year);
        
        if (RunMode)
        {
            if (previousSeconds != seconds)
            {
                previousSeconds = seconds;
                if (PMflag) printf("\r\rTime: %d:%02d:%02d PM", hours, minutes, seconds);
                else printf("\r\rTime: %d:%02d:%02d AM", hours, minutes, seconds);
                printf("\rDate: %d/%d/%d", month, day, year);
            }
        }
        */
/*        
        
        DelayMs(100);    
        sprintf(strLineOne, "Test %d:        ", TestCounter++);
        sprintf(strLineTwo, "Character: %c    ", TestChar);
        
        LCDGoto(0, 0);
        LCDWriteArray(strLineOne);
        LCDGoto(0, 1);
        LCDWriteArray(strLineTwo);
        
        LCDGoto(19, 2);
        LCDPutChar(TestChar++);
        LCDGoto(19, 3);
        LCDPutChar(TestChar++);        
        
        if (TestChar > 'Z') TestChar = 'A';        
    }
}    
*/


void InitializeSystem(void) 
{         
    SYSTEMConfigPerformance(60000000);
    OSCCONbits.SOSCEN = 1; // Enable secondary oscillator
    
    // Turn off JTAG so we get the pins back
    mJTAGPortEnable(false);
   
    // Set up Timer 1 for 1 millisecond interrupts
    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_64, 1250);
    // Set interrupt with a priority of 5 and zero sub-priority
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_5);        
 
    
    // Set up HOST UART for 115200 baud
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);

    UARTSetDataRate(HOSTuart, SYS_FREQ, 115200);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure HOST UART Interrupts
    INTEnable(INT_SOURCE_UART_TX(HOSTuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);
    
    // Set up REF UART for 9600 baud (EXTECH HYGROMETER)
    UARTConfigure(REFuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(REFuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(REFuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);

    UARTSetDataRate(REFuart, SYS_FREQ, 9600);
    UARTEnable(REFuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure REF UART Interrupts
    INTEnable(INT_SOURCE_UART_TX(REFuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(REFuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(REFuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(REFuart), INT_SUB_PRIORITY_LEVEL_0);    

    mPORTBClearBits(BIT_0);    
    mPORTBSetPinsDigitalOut(BIT_0);
    

    REF_DTR_OUT = REF_DISABLED;        // REF_DTR pin goes low to enable EXTECH RH meter for sending RH and Temp reference measurements.
    mPORTCSetPinsDigitalOut(BIT_3);
    
    mPORTESetPinsDigitalOut(BIT_0 | BIT_1 | BIT_2); 
    mPORTEClearBits(BIT_0 | BIT_1 | BIT_2);
    
    mPORTASetPinsDigitalOut(BIT_0 | BIT_1 | BIT_2 | BIT_3); 
    mPORTAClearBits(BIT_0 | BIT_1 | BIT_2 | BIT_3);
    
    mPORTDSetPinsDigitalIn(BIT_13);
    LATDbits.LATD8 = LATDbits.LATD9 = LATDbits.LATD10 = LATDbits.LATD11 = 0;    
    mPORTDClearBits(BIT_8 | BIT_9 | BIT_10 | BIT_11);
    mPORTDSetPinsDigitalOut(BIT_8 | BIT_9 | BIT_10 | BIT_11);    
    
    mCNOpen(CN_ON, CN19_ENABLE, 0);
    ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);              
    
    
    
    // Set up PWM OC3
    OC3CON = 0x00;
    OC3CONbits.OC32 = 0; // 16 bit PWM
    OC3CONbits.ON = 1; // Turn on PWM
    OC3CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC3CONbits.OCM2 = 1; // PWM enabled, no fault pin
    OC3CONbits.OCM1 = 1;
    OC3CONbits.OCM0 = 0;
    OC3RS = 0;

    // Set up Timer 2 for PWM time base    
    T2CON = 0x00;
    T2CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T2CONbits.TCKPS1 = 0;
    T2CONbits.TCKPS0 = 0;    
    PR2 = 3000; // Use 50 microsecond rollover for 20 khz
    T2CONbits.TON = 1; // Let her rip   
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);       
    
    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();

}//end UserInit

#define ESC 27
#define CR 13
#define BACKSPACE 8
void __ISR(HOST_VECTOR, IPL2AUTO) IntHostUartHandler(void) 
{
BYTE ch, inByte;
static int HostRxIndex = 0;

    if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) 
    {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));                 
        if (HOSTbits.OERR || HOSTbits.FERR) {
            if (UARTReceivedDataIsAvailable(HOSTuart))
                ch = UARTGetDataByte(HOSTuart);
            HOSTbits.OERR = 0;            
        }
        if (UARTReceivedDataIsAvailable(HOSTuart)) 
        {
            inByte = UARTGetDataByte(HOSTuart);
            ch = toupper(inByte);           


            if (ch != 0 && ch != '\n')            
            {            
                if (' ' == ch && HostRxIndex == 0)
                {
                    if (RunMode) 
                    {
                        RunMode = false;
                        printf("\rSYSTEM STANDBY\r");
                    }
                    else
                    {                        
                        RunMode = true;
                        printf("\rRUN MODE\r");
                    }
                    SumError = 0;
                }
                else if ('\r' == ch) 
                {
                    HOSTRxBufferFull = true;
                    HOSTRxBuffer[HostRxIndex++] = ch;
                    HOSTRxBuffer[HostRxIndex] = '\0';
                    HostRxIndex = 0;
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, '\r');
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, '\n');
                }     
                else if (ch < ' ')
                {
                    ControlCommand = ch;                    
                    HostRxIndex = 0;
                }
                else if (ch == BACKSPACE) 
                {
                    if (HostRxIndex != 0) HostRxIndex--;
                    HOSTRxBuffer[HostRxIndex] = '\0';
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, ' ');
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, BACKSPACE);                
                } 
                else if (HostRxIndex < MAX_RX_BUFFERSIZE) 
                {
                    HOSTRxBuffer[HostRxIndex] = ch;
                    HostRxIndex++;
                }            
            }
        }
    }         
    
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) 
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));            
}








// Timer 1 generates an interrupt every 50 microseconds approximately
void __ISR(_TIMER_1_VECTOR, ipl5) Timer1Handler(void) 
{        
    //if (TEST_OUT) TEST_OUT = 0;
    //else TEST_OUT = 1;
    
    intFlag = true;
    
    mT1ClearIntFlag(); // clear the interrupt flag     
}



// When using REV 2 or REV 3 MD13S board:
    // Analog input Analog inputs 1-4 are diagnostic pot inputs and inputs 12-15 are rotary pot inputs
    int ADC10_ManualInit(void)
    {
        int i, dummy;
    
        AD1CON1bits.ON = 0;
        mAD1IntEnable(INT_DISABLED);   
        mAD1ClearIntFlag();
    
        AD1CON1 = 0;
        AD1CON2 = 0;
        AD1CON3 = 0;
        AD1CHS  = 0;
        AD1CSSL = 0;
    
        // Set each Port B pin for digital or analog
        // Analog = 0, digital = 1
        AD1PCFGbits.PCFG0 = 1; 
        AD1PCFGbits.PCFG1 = 1; 
        AD1PCFGbits.PCFG2 = 1; 
        AD1PCFGbits.PCFG3 = 1; 
        AD1PCFGbits.PCFG4 = 1; 
        AD1PCFGbits.PCFG5 = 1; 
        AD1PCFGbits.PCFG6 = 1; 
        AD1PCFGbits.PCFG7 = 1; 
        AD1PCFGbits.PCFG8 = 1; 
        AD1PCFGbits.PCFG9 = 1; 
        AD1PCFGbits.PCFG10 = 1; 
        AD1PCFGbits.PCFG11 = 1;  
        
        AD1PCFGbits.PCFG12 = 0; 
        AD1PCFGbits.PCFG13 = 0; 
        AD1PCFGbits.PCFG14 = 0; 
        AD1PCFGbits.PCFG15 = 0;     
    
        AD1CON1bits.FORM = 000;        // 16 bit integer format.
        AD1CON1bits.SSRC = 7;        // Auto Convert
        AD1CON1bits.CLRASAM = 0;    // Normal operation - buffer overwritten by next conversion sequence
        AD1CON1bits.ASAM = 0;        // Not enable Automatic sampling yet.
        
        AD1CON2bits.VCFG = 0;        // Reference AVdd, AVss
        AD1CON2bits.OFFCAL = 0;        // Offset calibration disable.
        AD1CON2bits.CSCNA = 1;        // Scan inputs for CH0+ SHA Input for Mux A input 
        AD1CON2bits.SMPI = 0b1000;        // Interrupt after 9+1 conversion
        AD1CON2bits.BUFM = 0;        // One 16 word buffer
        AD1CON2bits.ALTS = 0;        // Use only Mux A
        AD1CON2bits.SMPI =  MAXPOTS-1;    // Number of channels to sample
        AD1CON2bits.BUFM = 0;                // Single 16-word buffer with CLRASAM.    
        AD1CHSbits.CH0NA = 0; // Mux A Negative input from VR-
        AD1CHSbits.CH0SA = 3; // Mux A Positive input from pin AN3
        
        // Set conversion clock and set sampling time.
        AD1CON3bits.ADRC = 0;        // Clock derived from peripheral bus clock
        AD1CON3bits.SAMC = 0b11111;        // Sample time max
        AD1CON3bits.ADCS = 0b11111111;   // Conversion time max

        // Select channels to scan. Scan channels = 1, Skip channels = 0
        AD1CSSLbits.CSSL0 = 0;
        AD1CSSLbits.CSSL1 = 0;
        AD1CSSLbits.CSSL2 = 0;
        AD1CSSLbits.CSSL3 = 0;
        AD1CSSLbits.CSSL4 = 0;
        AD1CSSLbits.CSSL5 = 0;
        AD1CSSLbits.CSSL6 = 0;
        AD1CSSLbits.CSSL7 = 0;
        AD1CSSLbits.CSSL8 = 0;
        AD1CSSLbits.CSSL9 = 0;
        AD1CSSLbits.CSSL10 = 0;
        AD1CSSLbits.CSSL11 = 0;
        AD1CSSLbits.CSSL12 = 1;
        AD1CSSLbits.CSSL13 = 1;
        AD1CSSLbits.CSSL14 = 1;
        AD1CSSLbits.CSSL15 = 1;
        
        // Make sure all buffers have been Emptied. 
        for (i = 0; i < 16; i++) dummy = (ADC1BUF0+i*4);    
        
        AD1CON1bits.ASAM = 1;        // Start Automatic Sampling. 
        AD1CON1bits.ON = 1;            // Turn on ADC.
        return (1);
    }

/*    
void __ISR(_TIMER_2_VECTOR, IPL2AUTO) Timer2Handler(void) 
{   
    mT2ClearIntFlag(); // clear the interrupt flag              
    TRIAC_OUT1 = 1;
    CloseTimer2();
}
*/

void __ISR(_TIMER_3_VECTOR, IPL2AUTO) Timer3Handler(void) 
{   
    mT3ClearIntFlag(); // clear the interrupt flag          
    TRIAC_OUT2 = 1;
    CloseTimer3();
}

void __ISR(_TIMER_4_VECTOR, IPL2AUTO) Timer4Handler(void) 
{   
    mT4ClearIntFlag(); // clear the interrupt flag      
    TRIAC_OUT3 = 1;
    CloseTimer4();    
}

void __ISR(_TIMER_5_VECTOR, IPL2AUTO) Timer5Handler(void) 
{
    mT5ClearIntFlag(); // Clear interrupt flag     
    TRIAC_OUT4 = 1;
    CloseTimer5();    
}



// AC ZERO CROSSING INTERRUPTS HANDLED HERE
// This executes 120 times per second for 60 Hz AC.
// A zero cross detection circuit must be connected to RB0 on PORT B.
// All for triacs are shut off here each time AC goes to zero.
void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNotice_Handler(void) 
{
    static unsigned char SinePositive = false;
    unsigned char ZCross = false;
    unsigned short PORTD_Read = 0;
    
    // Step #1 - clear mismatch first    
    PORTD_Read = PORTD;      
    
    if (PORTD_Read & 0x2000)
    {
        if (!SinePositive)
        {
            SinePositive = true;
            ZCross = true;
        }
    }
    else
    {
        if (SinePositive)
        {
            SinePositive = false;
            ZCross = true;
        }        
    }    
    
    if (ZCross)    
    {
        // TRIAC_OUT1 = 0;
        TRIAC_OUT2 = 0;
        TRIAC_OUT3 = 0;            
        TRIAC_OUT4 = 0;    
        
        if (TEST_OUT) TEST_OUT = 0;
        else TEST_OUT = 1;
            
        /*
        if (dimmerValues[0]!=0)
        {
            TMR2 = 0x0000;
            OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_32, Timer2Reload);
            ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
        }
        else CloseTimer2();
        */
    
        if (EnableWetPump)
        {
            TMR3 = 0x0000;        
            OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_32, Timer3Reload);
            ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2);           
        }
        else 
        {
            TRIAC_OUT2 = 0;
            CloseTimer3();
        }
    
        if (EnableDryPump)
        {
            TMR4 = 0x0000;        
            OpenTimer4(T4_ON | T4_SOURCE_INT | T4_PS_1_32, Timer4Reload);
            ConfigIntTimer4(T4_INT_ON | T4_INT_PRIOR_2);           
        }
        else
        {
            TRIAC_OUT3 = 0;
            CloseTimer4();
        }
    
/*
        if (dimmerValues[3]!=0)
        {
            TMR5 = 0x0000;        
            OpenTimer5(T5_ON | T5_SOURCE_INT | T5_PS_1_32, Timer5Reload);
            ConfigIntTimer5(T5_INT_ON | T5_INT_PRIOR_2);                                            
        }
        else 
*/            
        TRIAC_OUT4 = 0;            
        CloseTimer5();
    }
    // Step #2 - clear interrupt flag
    mCNClearIntFlag();
}

// Timer 2 generates an interrupt every 50 microseconds approximately
void __ISR(_TIMER_2_VECTOR, ipl5) Timer2Handler(void) 
{            
    mT2ClearIntFlag(); // Clear interrupt flag            
}

void __ISR(REF_VECTOR, IPL2AUTO) IntRefUartHandler(void) 
{
BYTE ch;
static int RefRxIndex = 0;

    if (INTGetFlag(INT_SOURCE_UART_RX(REFuart))) 
    {
        INTClearFlag(INT_SOURCE_UART_RX(REFuart));                 
        if (REFbits.OERR || REFbits.FERR) {
            if (UARTReceivedDataIsAvailable(REFuart))
                ch = UARTGetDataByte(REFuart);
            REFbits.OERR = 0;            
        }
        if (UARTReceivedDataIsAvailable(REFuart)) 
        {
            ch = UARTGetDataByte(REFuart);
            if (RefRxIndex < MAX_RX_BUFFERSIZE) 
                REFRxBuffer[RefRxIndex++] = ch;
            else RefRxIndex = 0;

            if (ch == 2) RefRxIndex = 0;
            else if ('\r' == ch) 
            {
                REFRxBufferFull = true;                
                REFRxBuffer[RefRxIndex] = '\0';                    
                RefRxIndex = 0;
            }     
        }
    }         
    
    if (INTGetFlag(INT_SOURCE_UART_TX(REFuart))) 
        INTClearFlag(INT_SOURCE_UART_TX(REFuart));            
}



int RHControl(float RHSetpoint, float RHActual)
{
    float Error;
    float totalDerError = 0;
    float derError;        
    float PCorr = 0, ICorr = 0, DCorr = 0, correction = 0;
    short i;
    static int errIndex = 0;
    static int counter = 0;
    static unsigned char SaturationFlag = false;
    int PWMvalue = 0; 
    
    Error = RHActual - RHSetpoint;     
    if (!SaturationFlag) SumError = SumError + Error;    
        
    totalDerError = 0;
    for (i = 0; i < FILTERSIZE; i++)
        totalDerError = totalDerError + arrAccumulatedError[i];
    
    derError = totalDerError / FILTERSIZE;    
        
    arrAccumulatedError[errIndex] = Error;
    errIndex++;
    if (errIndex >= FILTERSIZE) errIndex = 0;
    
    
    PCorr = Error * -kP;    
    ICorr = SumError  * -kI;
    DCorr = derError * -kD;

    correction = PCorr + ICorr + DCorr;
    if (correction > 0) PWMvalue = (int) correction + (int) Offset;
    else if (correction < 0) PWMvalue = (int) correction - (int) Offset;
    else PWMvalue = 0;             
     
    
    if (PWMvalue > PWM_MAX) 
    {
        PWMvalue = PWM_MAX;
        SaturationFlag = true;
    }
    else if (PWMvalue < -PWM_MAX) 
    {
        PWMvalue = -PWM_MAX;
        SaturationFlag = true;
    }
    else SaturationFlag = false;        
            
    if (PWMvalue > 0)  printf("\rWET: %.1f, ACT: %.1f, ERR: %0.1f, P: %0.1f, I: %.1f, D: %.1f, PWM: %d", RHSetpoint, RHActual, Error, PCorr, ICorr, DCorr, PWMvalue);
    else printf("\rDRY %.1f, ACT: %.1f, ERR: %0.1f, P: %0.1f, I: %.1f, D: %.1f, PWM: %d", RHSetpoint, RHActual, Error, PCorr, ICorr, DCorr, PWMvalue);
        
    return PWMvalue;
}



            //FanSpeed = ADresult[0] * 4;
            //if (FanSpeed < 0) FanSpeed = 0;
            //if (FanSpeed > 4000) FanSpeed = 4000;            
            //FAN_PWM = FanSpeed;
                      
            
            /*
            area = (float) ADresult[1] / 1024;
            theta = acos ((2 * area)-1);                   
            Timer3Reload = (unsigned short) (theta * 15625 / M_PI);                 
            
            area = (float) ADresult[2] / 1024;
            theta = acos ((2 * area)-1);                   
            Timer4Reload = (unsigned short) (theta * 15625 / M_PI);                 
            
            area = (float) ADresult[3] / 1024;
            theta = acos ((2 * area)-1);                   
            Timer5Reload = (unsigned short) (theta * 15625 / M_PI);              
            */
            
/*
unsigned char ParseCommandString (unsigned char *ptrReceivedString, int *Command, int *ptrIntegers, int *NumIntegers, unsigned char *ptrString)
{
    int TokenIndex = 0, ArgIndex = 0, IntegerIndex = 0, listindex = 0, length, integerValue = 0, stringIndex = 0;
    unsigned char *token, *ptrArgs;
    unsigned char delimiters[] = ":,\r /";
    unsigned char ch;
#define MAXNUMLENGTH 16
    unsigned char strNum[MAXNUMLENGTH+1];
    
    TokenIndex = IntegerIndex = listindex = length = 0;
    *Command = NO_COMMAND;

    if (ptrString) ptrString[0] = '\0';
    
    // Step through received UART characters & check for valid command
    token = strtok(ptrReceivedString, delimiters);
    while(token)
    {
        listindex = 0;                
        if (*Command == NO_COMMAND)
        {
            // Is substring a valid command?
            do
            {
                // Check list of valid commands for a match
                ptrArgs = strstr(token, CommandList[listindex]);
                // If command is valid, go to next step
                if (ptrArgs) 
                {                
                    *Command = listindex;
                    break;
                }
                listindex++;
            } while (listindex < MAXCOMMANDS);
        }          
        // If command is valid, check for numeric arguments:
        else
        {
            length = strlen(token);
            TokenIndex = 0;       
            ArgIndex = 0;
            stringIndex = 0;
            do {
                if (TokenIndex >= length) break;
                ch = token[TokenIndex++];
                if (isdigit(ch))
                {
                    if (ArgIndex < MAXNUMLENGTH) strNum[ArgIndex++] = ch;
                }
                else if (isalpha(ch))
                {
                    if (stringIndex < MAXSTRING)
                        ptrString[stringIndex++] = ch;
                    else return false;
                }
            } while(ch);
            if (ArgIndex)
            {
                strNum[ArgIndex] = '\0';
                if (IntegerIndex >= MAX_DATA_INTEGERS) return false;
                integerValue = atoi(strNum);
                ptrIntegers[IntegerIndex++] = integerValue;
                ArgIndex = 0;
            }
        }
        
        token = strtok(NULL, delimiters);
    }
    *NumIntegers = IntegerIndex;
    if (ptrString) ptrString[stringIndex] = '\0';
    
    if (*Command) return true;
    else return false;
}
 */
 
unsigned char ParseCommandString (unsigned char *ptrReceivedString, int *Command, float *ptrFloats, int *NumFloats, unsigned char *ptrString)
{
    int TokenIndex = 0, ArgIndex = 0, FloatIndex = 0, listindex = 0, length, stringIndex = 0;
    float FloatValue = 0;
    unsigned char *token, *ptrArgs;
    unsigned char delimiters[] = ":,\r /";
    unsigned char ch;
#define MAXNUMLENGTH 16
    unsigned char strNum[MAXNUMLENGTH+1];
    
    if (ptrReceivedString == NULL || Command == NULL) return false;
    
    if (ptrReceivedString[0] == '\r') 
    {
        *Command  = RUNMODE;
        printf("\rRESUME RUN MODE");
        return true;
    }
    
    TokenIndex = FloatIndex = listindex = length = 0;
    *Command = NO_COMMAND;    

    if (ptrString) ptrString[0] = '\0';
    
    // Step through received UART characters & check for valid command
    token = strtok(ptrReceivedString, delimiters);
    while (token)
    {
        listindex = 0;                
        if (*Command == NO_COMMAND)
        {
            // Is substring a valid command?
            do
            {
                // Check list of valid commands for a match
                ptrArgs = strstr(token, CommandList[listindex]);
                // If command is valid, go to next step
                if (ptrArgs) 
                {                
                    *Command = listindex;
                    break;
                }
                listindex++;
            } while (listindex < MAXCOMMANDS);
        }          
        // If command is valid, check for numeric arguments:
        else if (ptrFloats != NULL)
        {
            length = strlen(token);
            TokenIndex = 0;       
            ArgIndex = 0;
            stringIndex = 0;
            do {
                if (TokenIndex >= length) break;
                ch = token[TokenIndex++];
                if (isdigit(ch) || ch == '.')
                {
                    if (ArgIndex < MAXNUMLENGTH) strNum[ArgIndex++] = ch;
                }
                else if (isalpha(ch) && ptrString != NULL)
                {
                    if (stringIndex < MAXSTRING)
                        ptrString[stringIndex++] = ch;
                    else return false;
                }
            } while(ch);
            if (ArgIndex && ptrFloats != NULL)
            {
                strNum[ArgIndex] = '\0';
                if (FloatIndex >= MAX_DATA_FLOATS) return false;
                FloatValue = atof(strNum);
                ptrFloats[FloatIndex++] = FloatValue;
                ArgIndex = 0;
            }
        }
        else return true;
        
        token = strtok(NULL, delimiters);
    }
    if (NumFloats != NULL) *NumFloats = FloatIndex;
    if (ptrString != NULL) ptrString[stringIndex] = '\0';
    
    if (*Command) return true;
    else return false;
}