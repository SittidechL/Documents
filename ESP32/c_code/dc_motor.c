#include <stdio.h>
#include <stdlib.h>
#include <includes.h>

void main()
{
    // Setting up PIC modules such as Timers, IOs OCs,Interrupts, ...
    InitializeIO();
    InitializeLEDs();
    InitializeTimers();

    while(1) {
        WaitOnBtn1();
        Forward(4.0,70);
        Stop(1.0);
        Backward(3.0,50);
        Stop(2);
        Forward(3.0,40);
        Stop(1.0);
        Backward(2.0,20);
        LEDsOFF();
    }
    return;
}

void InitializeIO(){
    TRISAbits.TRISA6 = 1;
    TRISAbits.TRISA7 = 1;
    TRISGbits.TRISG12 = 0;
    TRISGbits.TRISB13 = 0;
    LATGbits.LATB12 = 0;
    LATGbits.LATB13 = 0;
    return;
}
