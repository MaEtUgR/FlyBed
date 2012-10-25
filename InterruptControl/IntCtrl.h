#ifndef __INTCTRL_H
#define __INTCTRL_H

extern void GPIO_IntEnable(char PortNumber, char BitPosition, char RiseFall);
extern void GPIO_IntDisable(char PortNumber, char BitPosition, char RiseFall);


/* mbed DIP Pin     LPC1768 Pin
        p5      =   P0_9
        p6      =   P0_8
        p7      =   P0_7
        p8      =   P0_6
        p9      =   P0_0
        p10     =   P0_1
        p11     =   P0_18
        p12     =   P0_17
        p13     =   P0_15
        p14     =   P0_16
        p15     =   P0_23
        p16     =   P0_24
        p17     =   P0_25
        p18     =   P0_26
        p21     =   P2_5
        p22     =   P2_4
        p23     =   P2_3
        p24     =   P2_2
        p25     =   P2_1
        p26     =   P2_0
        p27     =   P0_11
        p28     =   P0_10
        p29     =   P0_5
        p30     =   P0_4
*/


#endif