// Funktionen zur Interruptkontrolle bei IO-Ports
//---------------------------------------------------------------------------------------------------------------------------------------
#include "mbed.h"                       //muss immer geladen werden
#include "IntCtrl.h"



// Enable GPIO Interrupt (nur fuer  P0.0..P0.30 und P2.0..P2.13)
// PortNumber = 0 oder 2, BitPosition = 0..31, RiseFall: 0 = rise, 1 = fall, 2 = rise and fall
//-------------------------------------------------------------------------------
void GPIO_IntEnable(char PortNumber, char BitPosition, char RiseFall)
    {
    switch ( PortNumber )
        case 0:
            {
            switch ( RiseFall )
                {
                case 0:
                    LPC_GPIOINT->IO0IntEnR |= (0x1<<BitPosition);
                break;
                
                case 1:
                    LPC_GPIOINT->IO0IntEnF |= (0x1<<BitPosition);
                break;
                
                case 2:
                    LPC_GPIOINT->IO0IntEnR |= (0x1<<BitPosition);
                    LPC_GPIOINT->IO0IntEnF |= (0x1<<BitPosition);
                break;
 
                default:
                break;
               }
            
        case 2:
            {
            switch ( RiseFall )
                {
                case 0:
                    LPC_GPIOINT->IO2IntEnR |= (0x1<<BitPosition);
                break;
                
                case 1:
                    LPC_GPIOINT->IO2IntEnF |= (0x1<<BitPosition);
                break;
                
                case 2:
                    LPC_GPIOINT->IO2IntEnR |= (0x1<<BitPosition);
                    LPC_GPIOINT->IO2IntEnF |= (0x1<<BitPosition);
                break;

                default:
                break;
                }
            }
        }
    }
            

// Disable GPIO Interrupt (nur fuer  P0.0..P0.30 und P2.0..P2.13)
// PortNumber = 0 oder 2, BitPosition = 0..31, RiseFall: 0 = rise, 1 = fall, 2 = rise and fall
//-------------------------------------------------------------------------------
void GPIO_IntDisable(char PortNumber, char BitPosition, char RiseFall)
    {
    switch ( PortNumber )
        case 0:
            {
            switch ( RiseFall )
                {
                case 0:
                    LPC_GPIOINT->IO0IntEnR &= ~(0x1<<BitPosition);
                break;
                
                case 1:
                    LPC_GPIOINT->IO0IntEnF &= ~(0x1<<BitPosition);
                break;
                
                case 2:
                    LPC_GPIOINT->IO0IntEnR &= ~(0x1<<BitPosition);
                    LPC_GPIOINT->IO0IntEnF &= ~(0x1<<BitPosition);
                break;
 
                default:
                break;
               }
            
        case 2:
            {
            switch ( RiseFall )
                {
                case 0:
                    LPC_GPIOINT->IO2IntEnR &= ~(0x1<<BitPosition);
                break;
                
                case 1:
                    LPC_GPIOINT->IO2IntEnF &= ~(0x1<<BitPosition);
                break;
                
                case 2:
                    LPC_GPIOINT->IO2IntEnR &= ~(0x1<<BitPosition);
                    LPC_GPIOINT->IO2IntEnF &= ~(0x1<<BitPosition);
                break;

                default:
                break;
                }
            }
        }
    }
            