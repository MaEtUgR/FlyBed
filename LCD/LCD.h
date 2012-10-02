/* mbed LCD Library, for a 4-bit LCD based on HD44780
 * Copyright (c) 2007-2010, hb9gaa
 */

#ifndef LCD_H
#define LCD_H

#include "mbed.h"

class TextLCD : public Stream {
public:

    /** LCD panel format */
    enum LCDType {
        LCD16x2     /**< 16x2 LCD panel (default) */
        , LCD16x2B  /**< 16x2 LCD panel alternate addressing */
        , LCD20x2   /**< 20x2 LCD panel */
        , LCD20x4   /**< 20x4 LCD panel */
    };

    /** Create a TextLCD interface
     * @param rs    Instruction/data control line
     * @param e     Enable line (clock)
     * @param d0-d3 Data lines
     * @param type  Sets the panel size/addressing mode (default = LCD16x2)
     */
    TextLCD(PinName rs, PinName rw, PinName e, PinName d0, PinName d1, PinName d2, PinName d3, LCDType type = LCD16x2);

#if DOXYGEN_ONLY
    /** Write a character to the LCD
     * @param c The character to write to the display
     */
    int putc(int c);

    /** Write a formated string to the LCD
     * @param format A printf-style format string, followed by the
     *               variables to use in formating the string.
     */
    int printf(const char* format, ...);
#endif

    /** Locate to a screen column and row
     * @param column  The horizontal position from the left, indexed from 0
     * @param row     The vertical position from the top, indexed from 0
     */
    void locate(int column, int row);

    /** Clear the screen and locate to 0,0 */
    void cls();

    int rows();
    int columns();

protected:

    // Stream implementation functions
    virtual int _putc(int value);
    virtual int _getc();

    int address(int column, int row);
    void character(int column, int row, int c);
    void writeByte(int value);
    void writeCommand(int command);
    void writeData(int data);

    DigitalOut _rs, _rw, _e;
    BusOut _d;
    LCDType _type;

    int _column;
    int _row;
};

#endif
