#include "PC.h"
#include "mbed.h"

PC::PC(PinName tx, PinName rx) : Serial(tx, rx) 
{
}


void PC::cls() 
{
    printf("\x1B[2J");
}


void PC::locate(int Spalte, int Zeile) 
{
    printf("\x1B[%d;%dH", Zeile + 1, Spalte + 1);
}
