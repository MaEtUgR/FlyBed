
#include "terminal.h"
#include "mbed.h"

terminal::terminal(PinName tx, PinName rx) : Serial(tx, rx) 
    {
    }


void terminal::cls() 
    {
    printf("\x1B[2J");
    }


void terminal::locate(int Spalte, int Zeile) 
    {
    printf("\x1B[%d;%dH", Zeile + 1, Spalte + 1);
    }
