// Created: 10/03/18
// Last Edited: 10/05/18
// Author: Cameron Bendzynski

#include <msp430.h> 

void LEDSetup();                                    // prototyping functions
void TimerA0Setup();

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;                       // stop watchdog timer
//    UCSCTL4 = SELA_1;                               // ACLK (10kHz)
    LEDSetup();                                     // Function for LED setup
    TimerA0Setup();                                 // Function for Timer0 setup

    __bis_SR_register(GIE);                         // Global Interrupt Enable
    while(1);                                       // Continuously runs program
}

void LEDSetup()                                     // All initial settings for LED use
{
    P1DIR |= BIT0;                                  // Set P1.0 to output - R LED
    P4DIR |= BIT7;                                  // Set P2.3 to output - G LED
}

void TimerA0Setup()                                 // All initial settings for TimerA0
{
    TA0CTL = TASSEL_2 + MC_1 + TAIE;                       // Set Timer A0 in Up Mode with divider, counter clear, and interrupt enabled
    TA0CCTL0 |= CCIE;                               // Capture/Compare enable on Timer1 CCR1
    TA0CCTL1 |= CCIE;                               // Capture/Compare enable on Timer1 CCR0
    TA0CCTL2 |= CCIE;
//    TA0CCTL3 |= CCIE;
    TA0CCR0 = 257;                                  // Set Capture/Compare register to 2000
    TA0CCR1 = 128;
    TA0CCR2 = 64;
//    TA0CCR3 = 8;
}

#pragma vector = TIMER0_A0_VECTOR                   // Detects interrupt for CCR0 on Timer1
__interrupt void Timer_A00(void)
{
    P1OUT |= BIT0;                              // Turns the LED on
    P4OUT |= BIT7;
}

#pragma vector = TIMER0_A1_VECTOR                   // Detects interrupt for CCR1 on Timer1
__interrupt void Timer_A01(void)
{
    switch (TA0IV)
    {
    case TA0IV_TACCR1:                    // Checks the interrupt vector to determine if CCR1 was triggered
        P1OUT &= ~BIT0;                             // Turns the LED off
        break;
    case TA0IV_TACCR2:                      // Checks the interrupt vector to determine if CCR2 was triggered
        P4OUT &= ~BIT7;                             // Turns the LED off
        break;
//    if (TA0IV == TA0IV_TACCR1)                      // Checks the interrupt vector to determine if CCR1 was triggered
//        P1OUT &= ~BIT0;                             // Turns the LED off
    }
}
