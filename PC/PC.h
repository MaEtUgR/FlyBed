#include "mbed.h"

#ifndef PC_H
#define PC_H

#define COMMAND_MAX_LENGHT 300

class PC : public Serial 
{
    public:
        PC(PinName tx, PinName rx, int baud);
        void cls();                                                                        // to clear the display
        void locate(int column, int row);                                                  // to relocate the cursor
        void readcommand(void (*executer)(char*));                  // to read a char from the pc to the command string
        
        char command[COMMAND_MAX_LENGHT];
    private:
        int command_char_count;
};
#endif
