/********************************************************************
* FileName:        LCD_4BIT_PIC32.h
* 
* Updated 7-20-22 to work with 20x4 New Haven NHD-0420H1Z-FSW-GBW
* Use PORT E2 to latch data on LCD EN pin
********************************************************************/

#define LCD_COMMAND 1
#define LCD_DATA 0

#define lcd_Clear           0b00000001          // replace all characters with ASCII 'space'
#define lcd_Home            0b00000010          // return cursor to first position on first line
#define lcd_EntryMode       0b00000110          // shift cursor from left to right on read/write
#define lcd_DisplayOff      0b00001000          // turn display off
#define lcd_DisplayOn       0b00001100          // display on, cursor off, don't blink character
#define lcd_FunctionReset   0b00110000          // reset the LCD
#define lcd_FunctionSet4bit 0b00101000          // 4-bit data, TWO OR FOUR LINE DISPLAY!!!y, 5 x 7 font
#define lcd_SetCursor       0b10000000          // set cursor position

#define	LCD_STROBE {mPORTESetBits(BIT_2); DelayUs(2); mPORTEClearBits(BIT_2);}  // Latches data on LCD EN pin

void WaitLCD(void);
void WriteNibble(unsigned char CommandFlag, unsigned char byte);
void WriteByte(unsigned char CommandFlag, unsigned char byte);
void LCDInit(void);
void LCDClear(void);
void LCDGoto(unsigned char Pos,  unsigned char Ln);
void LCDPutChar(unsigned char Data);
void LCDPutByte(unsigned char Val);
void LCDWriteStr(char  *Str);
void LCDWriteArray (unsigned char  *arrayPtr);

