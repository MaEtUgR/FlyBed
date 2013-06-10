#include "PC.h"
#include "mbed.h"

PC::PC(PinName tx, PinName rx, int baudrate) : Serial(tx, rx) 
{
    baud(baudrate);
    cls();
    
    command[0] = '\0';
    command_char_count = 0;
}


void PC::cls() 
{
    printf("\x1B[2J");
}


void PC::locate(int Spalte, int Zeile) 
{
    printf("\x1B[%d;%dH", Zeile + 1, Spalte + 1);
}

void PC::readcommand(void (*executer)(char*))
{
    char input = getc();             // get the character from serial bus
    if(input == '\r') {                 // if return was pressed, the command must be executed
        command[command_char_count] = '\0';
        executer(&command[0]);
        
        command_char_count = 0;                 // reset command
        command[command_char_count] = '\0';
    } else if (command_char_count < COMMAND_MAX_LENGHT) {
        command[command_char_count] = input;
        command_char_count++;
    }
}