/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//   MSP430F552x Demo - USCI_A1, 115200 UART Echo ISR, DCO SMCLK
//
//   Description: Echo a received character, RX ISR used. Normal mode is LPM0.
//   USCI_A1 RX interrupt triggers TX Echo.
//   Baud rate divider with 1048576hz = 1048576/115200 = ~9.1 (009h|01h)
//   ACLK = REFO = ~32768Hz, MCLK = SMCLK = default DCO = 32 x ACLK = 1048576Hz
//   See User Guide for baud rate divider table
//
//                 MSP430F552x
//             -----------------
//         /|\|                 |
//          | |                 |
//          --|RST              |
//            |                 |
//            |     P3.3/UCA1TXD|------------>
//            |                 | 9600 - 8N1
//            |     P3.4/UCA1RXD|<------------
//
//   Bhargavi Nisarga
//   Texas Instruments Inc.
//   April 2009
//   Built with CCSv4 and IAR Embedded Workbench Version: 4.21
//******************************************************************************
#include <msp430f5529.h>
//Program should be flashed to board used for testing final main.c 
//Recieves packets over USB and sets over the UART pin
unsigned char state = 1;                    //initialize state machine in state 1 
unsigned char i;                            //create variable to count packet length 

void LEDSetup()                              // All initial settings for LED use
{
    P1DIR |= BIT2;                                 // Set P1.2 to output - R LED
    P1DIR |= BIT3;                                 // Set P1.3 to output - G LED
    P1DIR |= BIT4;                                 // Set P1.4 to output - B LED
}

void TimerA0Setup()                          // All initial settings for TimerA0
{
    TA0CTL = TASSEL_2 + MC_1 + TAIE; // Set Timer A0 in Up Mode , counter clear, and interrupt enabled
    TA0CCTL0 |= CCIE;                   // Capture/Compare enable on Timer0 CCR0
    TA0CCTL1 |= CCIE;                   // Capture/Compare enable on Timer0 CCR1
    TA0CCTL2 |= CCIE;                   // Capture/Compare enable on Timer0 CCR2
    TA0CCTL3 |= CCIE;                    // Capture/Compare enable on Timer0 CCR3
    TA0CCR0 = 256;                      // Set Capture/Compare register 0 to 256
    TA0CCR1 = 0;                        //Initialize CCR1 to 0 
    TA0CCR2 = 0;                       //Initialize CCR2 to 0
    TA0CCR3 = 0;                        //Initialize CCR3 to 0
}
int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
    LEDSetup();                                     // Function for LED setup
    TimerA0Setup();                                 // Function for Timer0 setup
    P4SEL |= BIT4 + BIT5;                           //UART over USB enable
    P3SEL |= BIT3 + BIT4;                       // P3.3,4 = USCI_A1 TXD/RXD, UART via pins to/from boards 
    //UART over USB setup 
    UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA1CTL1 |= UCSSEL_2;                     // SMCLK
    UCA1BR0 = 6;                            // 1MHz 9600 
    UCA1BR1 = 0;                              // 1MHz 9600
    UCA1MCTL |= UCBRS_0 + UCBRF_13 + UCOS16;   // Modulation UCBRSx=1, UCBRFx=0
    UCA1CTL1 &= ~UCSWRST;                   // **Initialize USCI state machine**
    UCA1IE |= UCRXIE;                         // Enable USCI_A1 RX interrupt
    //UART over pins setup 
    UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA0CTL1 |= UCSSEL_2;                     // SMCLK
    UCA0BR0 = 6;                            // 1MHz 9600 
    UCA0BR1 = 0;                              // 1MHz 9600
    UCA0MCTL |= UCBRS_0 + UCBRF_13 + UCOS16;   // Modulation UCBRSx=1, UCBRFx=0
    UCA0CTL1 &= ~UCSWRST;                   // **Initialize USCI state machine**
    UCA0IE |= UCRXIE;                         // Enable USCI_A1 RX interrupt

    __bis_SR_register(GIE);             // Global interrupts enabled
    while (1);                             //infinite while loop 
}

#pragma vector = TIMER0_A0_VECTOR                   // Detects interrupt for CCR0 on Timer1
__interrupt void Timer_A00(void)
{
    if (TA0CCR1)                                    //check for non-zero value 
        P1OUT &= ~BIT2;                              // Turns the red LED on
    if (TA0CCR2)                                    //check for non-zero value 
        P1OUT &= ~BIT3;                                //Turns green LED on 
    if (TA0CCR3)                                     //check for non-zero value 
        P1OUT &= ~BIT4;                             //Turns blue LED on 

}

#pragma vector = TIMER0_A1_VECTOR                   // Detects interrupt for CCR1/CCR2/CCCR3 on Timer1
__interrupt void Timer_A01(void)
{
    switch (TA0IV)                  //Check interrupt vector register 
    {
    case TA0IV_TACCR1: // Checks the interrupt vector to determine if CCR1 was triggered
        P1OUT |= BIT2; // Turns the red LED off
        break;
    case TA0IV_TACCR2: // Checks the interrupt vector to determine if CCR2 was triggered
        P1OUT |= BIT3; // Turns the green LED off
        break;
    case TA0IV_TACCR3: // Checks the interrupt vector to determine if CCR3 was triggered
        P1OUT |= BIT4; //Turns the blue LED off
        break;
    default:
        break;
    }
}

#pragma vector=USCI_A1_VECTOR //UART Interrupt vector 
__interrupt void USCI_A1_ISR(void)
{

    switch (__even_in_range(UCA1IV, 4)) //checks UART interrupt vector register 
    {
    case 0:
        break;                             // Vector 0 - no interrupt
    case 2:                                   // Vector 2 - RXIFG - receive detected 
        while (!(UCA1IFG & UCTXIFG);  // USCI_A1 TX buffer ready?
        switch (state) //checks what state the code is in 
        {
        case 1: //length byte
            i = UCA1RXBUF; //store given length
            i--; //decrement length
            UCA0TXBUF = UCA1RXBUF - 3; //send length minus 3
            state = 2; //change state to 2
            break;
        case 2: //red byte
            TA0CCR1 = UCA1RXBUF; //store red byte in CCR1
            i--; //decrement length 
            state = 3; //change to state 3 
            break;
        case 3: //green byte
            TA0CCR2 = UCA1RXBUF; //store green byte in CCR2
            i--; //decrement length 
            state = 4; //change to state 4 
            break;
        case 4: //blue byte
            TA0CCR3 = UCA1RXBUF; //store blue byte in CCR3
            i--; //decrement length 
            state = 5; //change to state 5 
            break;
        case 5: //blue byte
            if (i == 1) //check if length is 1, signifying stop character 
            { 
                UCA0TXBUF = 0x0D; //send stop character 
                state = 1; //reset state to prepare for any future input 
            }
            else
            {
                UCA0TXBUF = UCA1RXBUF; //if length is greater than 1, send all received bytes 
                i--; //decrement length 
            }
            break;
        default:
            break;
        }

        break;
    default:
        break;
    }
}
