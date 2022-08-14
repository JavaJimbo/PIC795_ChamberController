/********************************************************************
* FileName:     LCD_4BIT_PIC32.c
* 
* Tested with:  20x4 New Haven NHD-0420H1Z-FSW-GBW
* 
* Processor:    PIC 32MX795F512L on UBW32 Board
* Compiler:     Microchip XC32 V1.30 MPLABX IDE: V6.00
*
* 
* PIC LCD pin assignments:
* LCD DATA: PIC RA0-RA3 connected to LCD display pins D4, D5, D6, D7
* 
* LCD_RS: RE0
* LCD_RW: RE1
* LCD_EN: RE2  
* 
* Created 8-21-2020 JBS
* Updated 7-20-22
********************************************************************/
#include "plib.h"
#include "LCD_4BIT_PIC32.h"
#include "Delay.h"

//*****************************************************************************
//                            CONSTANT DEFINITION
//*****************************************************************************




void WaitLCD(void){
	DelayMs(4);
}

/*
* Function  		WriteNibble
*
* @brief         	This function writes the specified high nibble to the LCD and
*                  	notifies the (LCD) controller whether it has to interpret
*                  	the nibble as data or command.
*
* @param	        CommandFlag=TRUE for command, FALSE to write character
*                   	
*
* @param	        byte command or character byte
*
*/
/*******************************************************************/

void WriteNibble(unsigned char CommandFlag, unsigned char byte)
{
unsigned int PORTA_Data;    
    
    // Set LCD RS input for DATA or CONTROL:
    if (CommandFlag) mPORTEClearBits(BIT_0);
    else mPORTESetBits(BIT_0);          

    PORTA_Data = mPORTARead() & 0xFFF0;        // Read PORTA, mask off lowest nibble
    PORTA_Data = PORTA_Data | (byte & 0x0F);   // OR in data nibble to write    
    mPORTAWrite(PORTA_Data);                   // Write nibble to PORT A    
     
    LCD_STROBE                                  // Latch LCD data
}


/********************************************************************
* Function:         WriteByte
* 
*
* @brief          	This function writes the specified Byte to the LCD and
*                   notifies the (LCD) controller whether it has to interpret
*                   the Byte as data or command. 
*
* @param	        CommandFlag		This flag specifies whether the data to be written to
*                   		the LCD is a command or data to be displayed.
*
* @param	        byte		Actual data or command to be written to the LCD controller.
*
* @note    			This routine is meant to be used from within this module only.
*/
/*******************************************************************/ 

void WriteByte(unsigned char CommandFlag, unsigned char byte)
{
unsigned char ch;

    ch = (byte & 0xF0) >> 4;
    WriteNibble(CommandFlag, ch);     // Send the high nibble
    
    ch = byte & 0x0F;
    WriteNibble(CommandFlag, ch);     // Send low nibble
}

/********************************************************************
* Function:         LCDInit
*
* @brief          	This routine is called once at start up to initialize the
*                   MCU hardware for proper LCD operation.
*
* @note    			Should be called at system start up only.
*/                                                                          
/*******************************************************************/



// initialise the LCD - put into 4 bit mode
#define BIT_DELAY 10
void LCDInit(void)
{	
    //WriteByte(LCD_COMMAND, lcd_FunctionReset);
    
    DelayMs(200);	// power on delay
    mPORTAClearBits(BIT_3);
    mPORTAClearBits(BIT_2);
    mPORTASetBits(BIT_1);
    mPORTASetBits(BIT_0);
    LCD_STROBE
    DelayMs(2);
    LCD_STROBE
    DelayMs(2);
    LCD_STROBE
    DelayMs(20);
    
    WriteByte(LCD_COMMAND, lcd_FunctionSet4bit);    
    DelayMs(BIT_DELAY);	

    WriteByte(LCD_COMMAND, lcd_Clear);    
    DelayMs(BIT_DELAY);	
    
    WriteByte(LCD_COMMAND, lcd_EntryMode);    
    DelayMs(BIT_DELAY);	

    WriteByte(LCD_COMMAND, lcd_DisplayOn);    
    DelayMs(BIT_DELAY);	

	
//	WriteByte(TRUE, 0x28);	// 4 bit mode, 1/16 duty, 5x8 font
//	WriteByte(TRUE, 0x10);	// set cursor
//	WriteByte(TRUE, 0x0F);	// Display ON; Blinking cursor
//	WriteByte(TRUE, 0x06);	// entry mode
}



/********************************************************************
* Function:         LCDClear
* 
* PreCondition: 	None
*
* @brief         	This function is called to wipe the LCD display out.
*
* @note    			None.
*/  
/*******************************************************************/

void LCDClear(void){
  WriteByte(TRUE,0x01);                       // Send clear display command
  WaitLCD();                                  // Wait until command is finished
}

/********************************************************************
* Function:         LCDGoto
*
* This function positions the cursor at the specified Line and column.
*
* 	        Pos		Column (0 to 19) the cursor should be positioned at*
* 	        Ln		Line (0,1,2,3) the cursor should be positioned at.
*
*/                                                                    
/*******************************************************************/

void LCDGoto(unsigned char Pos,  unsigned char Ln)
{
	if (Ln==0) WriteByte(TRUE, 0x80|Pos);
	else if (Ln==1) WriteByte(TRUE, 0xC0|Pos);
    else if (Ln==2) WriteByte(TRUE, 0x80|(Pos + 0x14));
    else WriteByte(TRUE, 0xC0|(Pos+0x54));
	WaitLCD();                                      			 // Wait for the LCD to finish
}


/********************************************************************
* Function:         LCDPutChar
*
* @brief            This function displays the specified ASCII character at
*                   current position on the LCD
*
* @param	    ch - ASCII data character to be displayed.
*
*/ 
/*******************************************************************/

void LCDPutChar(unsigned char ch){
  WriteByte(FALSE, ch);              // Go output the character to the display
  WaitLCD();                          // Wait until it's finished
}

/********************************************************************
* Function:         LCDPutByte
*
* @brief            This function displays the specified binary value at
*                   current position on the LCD. It converts the binary
*                   value into displayable ASCII characters.
*
* @param	    Val		Binary data byte to be displayed
*
* @note    			In the present configuration, this routine displays a
*                   2-digit value, by prefilling with '0' any value lower
*                   than 10.
*/
/*******************************************************************/

void LCDPutByte(unsigned char Val){
  LCDPutChar(Val/10+'0');                   // Output the high digit
  LCDPutChar(Val % 10+'0');                 // Output low
}

/********************************************************************
* Function:         LCDWriteStr
*
* @brief          	This function displays the specified string starting from
*                   current position on the LCD.
*
* @param	        Str		IF 0; Terminated string to be displayed.
*
* @note    			None
*/
/*******************************************************************/

void LCDWriteStr(char  *Str){
  unsigned char i = 0;                                     // Char index buffer

  while (Str[i])                                   // While string not finished
    LCDPutChar(Str[i++]);                          // Go display current char
}

#define MAX_LCD_STRING 20
void LCDWriteArray (unsigned char  *arrayPtr)
{
unsigned char ch, i=0;                                     // Char index buffer

	i=0;
	for(i=0; i<MAX_LCD_STRING; i++){
		ch=arrayPtr[i];
		if(ch)LCDPutChar(ch);                          // Go display current char
		else break;
	}
}

