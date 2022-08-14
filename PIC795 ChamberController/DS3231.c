#include "I2C_4BUS_EEPROM_PIC32.h"
//#include "string.h"
#include "plib.h"
#include "Delay.h"
#include <stdio.h>
#include <stdlib.h>
#include "Defs.h"
#include "DS3231.h"
//#include <ctype.h>




unsigned char GetDS3231Time(short *hours, short *minutes, short *seconds, short *PMflag)
{
    unsigned char arrClockRegisters[3];
    unsigned char secondsTens, secondsOnes, minutesTens, minutesOnes, hoursTens, hoursOnes;
    
    if (!DS3231ReadBlock(I2C3, 0x00, arrClockRegisters, 3)) return false;        
        
    secondsOnes = arrClockRegisters[0] & 0x0F;
    secondsTens =  ((arrClockRegisters[0] & 0xF0) >> 4);
    *seconds = secondsOnes + (secondsTens * 10);            
    if (*seconds < 0 ) *seconds = 0;
    else if (*seconds > 59 ) *seconds = 59;
    
    minutesOnes = arrClockRegisters[1] & 0x0F;
    minutesTens =  ((arrClockRegisters[1] & 0xF0) >> 4);
    *minutes = minutesOnes + (minutesTens * 10);            
    if (*minutes < 0 ) *minutes = 0;
    else if (*minutes > 59 ) *minutes = 59;
    
    hoursOnes = arrClockRegisters[2] & 0x0F;
    hoursTens =  ((arrClockRegisters[2] & 0x10) >> 4);
    *hours = hoursOnes + (hoursTens * 10);            
    if (*hours < 1 ) *hours = 1;
    else if (*hours > 12 ) *hours = 12;
    
    if ((arrClockRegisters[2] & 0x20) != 0) *PMflag = 1;
    else *PMflag = 0;
    
    return true;
}


unsigned char AdjustDS3231Time(short hours, short minutes, short seconds, short PMflag)
{
unsigned char arrCommand[3];
unsigned char secondsTens, secondsOnes, minutesTens, minutesOnes, hoursTens, hoursOnes;

    if (seconds < 0) arrCommand[0] = 0x00;
    else if (seconds > 59) arrCommand[0] = 0x00;
    else 
    {
        secondsTens = seconds / 10;
        secondsOnes = seconds - (secondsTens * 10);
        arrCommand[0] = (secondsTens << 4) | secondsOnes;
    }

    if (minutes < 0) arrCommand[1] = 0x00;
    else if (minutes > 59) arrCommand[1] = 0x00;
    else 
    {
        minutesTens = minutes / 10;
        minutesOnes = minutes - (minutesTens * 10);
        arrCommand[1] = (minutesTens << 4) | minutesOnes;
    }

    if (hours < 1) hoursOnes = 1;
    else if (hours > 12)
    {
        hoursOnes = 2;
        hoursTens = 1;
    }

    hoursTens = hours / 10;
    hoursOnes = hours - (hoursTens * 10);    
    
    if (PMflag) arrCommand[2] = 0x60 | (hoursTens<<4) | hoursOnes;
    else arrCommand[2] = 0x40 | (hoursTens<<4) | hoursOnes;
    
    if (!DS3231WriteBlock(I2C3, 0x00, arrCommand, 3)) return false;
    else return true;
}


unsigned char GetDS3231Date(short *month, short *day, short *year)
{
    unsigned char arrClockRegisters[3];
    unsigned char dateTens, dateOnes, monthOnes, yearTens, yearOnes;
    
    if (!DS3231ReadBlock(I2C3, 0x04, arrClockRegisters, 3)) return false;       
    
    dateOnes = arrClockRegisters[0] & 0x0F;
    dateTens =  ((arrClockRegisters[0] & 0x30) >> 4);
    *day = dateOnes + (dateTens * 10); 
    
    if (*day < 1 ) *day = 1;
    else if (*day > 31 ) *day = 31;
    
    monthOnes = arrClockRegisters[1] & 0x0F;
    if ((arrClockRegisters[1] & 0x10) != 0) *month = monthOnes + 10;
    else *month = monthOnes;
    
    if (*month < 1 ) *month = 1;
    else if (*month > 12 ) *month = 12;
    
    yearOnes = arrClockRegisters[2] & 0x0F;
    yearTens =  ((arrClockRegisters[2] & 0xF0) >> 4);
    *year = yearOnes + (yearTens * 10);  
    
    if (*year < 0) *year = 1;
    else if (*year > 99) *year = 1;
    *year = *year + 2000;
            
    return true;
}

unsigned char AdjustDS3231Date(short month, short day, short year)
{
unsigned char arrCommand[3];
unsigned char dayTens, dayOnes, monthTens, monthOnes, yearTens, yearOnes, yearHundreds;

   if (day < 1) arrCommand[0] = 1;
    else if (day > 31) arrCommand[0] = 1;
    else 
    {
        dayTens = day / 10;
        dayOnes = day - (dayTens * 10);
        arrCommand[0] = (dayTens << 4) | dayOnes;
    }

    if (month < 1) arrCommand[1] = 1;
    else if (month > 12) arrCommand[1] = 1;
    else 
    {
        monthTens = month / 10;
        monthOnes = month - (monthTens * 10);
        arrCommand[1] = (monthTens << 4) | monthOnes;
    }

    if (year < 1)
    {
        yearOnes = 1;
        yearTens = 0;        
    }
    else if (year > 99)
    {
        yearHundreds = year / 100;
        year = year - (yearHundreds * 100);
        yearTens = year / 10;
        yearOnes = year - (yearTens * 10);            
    }
    else 
    {
        yearTens = year / 10;
        yearOnes = year - (yearTens * 10);    
    }
    arrCommand[2] = (yearTens << 4) | yearOnes;
    
    if (!DS3231WriteBlock(I2C3, 0x04, arrCommand, 3)) return false;
    else return true;
}