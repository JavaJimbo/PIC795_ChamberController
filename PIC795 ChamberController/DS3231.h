/* 
 * File:   DS3231.h
 * Author: JimSe
 *
 * Created on July 21, 2022, 6:21 PM
 */

#ifndef DS3231_H
#define	DS3231_H

#ifdef	__cplusplus
extern "C" {
#endif

unsigned char AdjustDS3231Time(short hours, short minutes, short seconds, short AMPM);
unsigned char GetDS3231Time(short *hours, short *minutes, short *seconds, short *PMflag);
unsigned char GetDS3231Date(short *month, short *day, short *year);
unsigned char AdjustDS3231Date(short month, short day, short year);


#ifdef	__cplusplus
}
#endif

#endif	/* DS3231_H */

