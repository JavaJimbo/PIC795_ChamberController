/* 
 * File:   Defs.h
 * Author: JimSe
 *
 * Created on July 21, 2022, 6:03 PM
 */

#ifndef DEFS_H
#define	DEFS_H

#ifdef	__cplusplus
extern "C" {
#endif

#define SYS_FREQ 60000000
#define REF_DTR_OUT LATCbits.LATC3    
#define REF_DISABLED 1    
#define REF_ENABLED 0
#define FILTERSIZE 8
    
#define false FALSE
#define true TRUE    


#ifdef	__cplusplus
}
#endif

#endif	/* DEFS_H */

