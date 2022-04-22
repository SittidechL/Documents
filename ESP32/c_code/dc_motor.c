//https://stackoverflow.com/questions/35468533/embedded-c-code-to-control-a-dc-motor-with-a-pic-microcontroller
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

void InitializeLEDs(){
    //code to initialize LEDS
}

void InitializeTimers(){
    // Initialize Timer1
    T1CON = 0x0000; // Set Timer1 Control to zeros
    T1CONbits.TCKPS=3; // prescale by 256
    T1CONbits.ON = 1; // Turn on Timer
    PR1= 0xFFFF; // Period of Timer1 to be full
    TMR1 = 0; // Initialize Timer1 to zero
    // Initialize Timer2
    T2CON = 0;
    T2CONbits.TCKPS = 7; // prescale by 256
    T2CONbits.T32 = 1; // use 32 bits timer
    T2CONbits.ON = 1;
    PR2 = 0xFFFFFFFF; // Period is set for 32 bits
    TMR2 = 0;
}

void WaitOnBtn1(){
    // wait on Btn1 indefinitely
    while(PORTAbits.RA6 == 0);

    // Turn On LED1 indicating it is Btn1 is Pushed
    LATBbits.LATB10 = 1;
    return;
}

void Forward(float Sec, int D){
    int RunTime = (int)(Sec*39000); // convert the total
    time to number of Tics
    TMR2 = 0;
    //LEDs
    LATGbits.LATG12 = 1; // forward Direction
    LATBbits.LATB12 = 0;
    LATBbits.LATB13 = 0;
    LATBbits.LATB11 = 1;
    // Keep on firing the PWM as long as Run time is not
    elapsed
    while (TMR2 < RunTime){
        PWM(D);
    }
    return;
}

void PWM(int D){
    TMR1 = 0;
    int Period = 400;
    while (TMR1< Period) {
        if (TMR1 < Period*D/100){
            LATGbits.LATG13 = 1;
        }
        else{
        LATGbits.LATG13 = 0;
    }
}
