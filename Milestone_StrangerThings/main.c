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
//            |     P3.3/UCA0TXD|------------>
//            |                 | 9600 - 8N1
//            |     P3.4/UCA0RXD|<------------
//
//   Bhargavi Nisarga
//   Texas Instruments Inc.
//   April 2009
//   Built with CCSv4 and IAR Embedded Workbench Version: 4.21
//******************************************************************************
#include <msp430f5529.h>
unsigned char state = 1;
unsigned char i;

void LEDSetup()                                 // All initial settings for LED use
{
    P1DIR |= BIT2;                              // Set P6.0 to output - R LED - 1.8V
    P1DIR |= BIT3;                              // Set P6.1 to output - G LED - 2.3V
    P1DIR |= BIT4;                              // Set P6.2 to output - B LED - 2.7V
}

void TimerA0Setup()                             // All initial settings for TimerA0
{
    TA0CTL = TASSEL_2 + MC_1 + TAIE;            // Set Timer A0 in Up Mode , counter clear, and interrupt enabled
    TA0CCTL0 |= CCIE;                           // Capture/Compare enable on Timer0 CCR0
    TA0CCTL1 |= CCIE;                           // Capture/Compare enable on Timer0 CCR1
    TA0CCTL2 |= CCIE;                           // Capture/Compare enable on Timer0 CCR2
    TA0CCTL3 |= CCIE;                           // Capture/Compare enable on Timer0 CCR3
    TA0CCR0 = 270;                              // Set Capture/Compare register CCR0 to 270
    TA0CCR1 = 0;                                // Set Capture/Compare register CCR1 to 0
    TA0CCR2 = 0;                                // Set Capture/Compare register CCR2 to 0
    TA0CCR3 = 0;                                // Set Capture/Compare register CCR3 to 0
}
int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;                   // Stop WDT
    LEDSetup();                                 // Function for LED setup
    TimerA0Setup();                             // Function for Timer0 setup
    P3SEL |= BIT3 + BIT4;                       // P3.3,4 = USCI_A0 TXD/RXD

    UCA0CTL1 |= UCSWRST;                        // **Put state machine in reset**
    UCA0CTL1 |= UCSSEL_2;                       // SMCLK
    UCA0BR0 = 6;                                // 1MHz 9600 (see User's Guide)
    UCA0BR1 = 0;                                // 1MHz 9600
    UCA0MCTL |= UCBRS_0 + UCBRF_13 + UCOS16;    // Modulation UCBRSx=1, UCBRFx=0
    UCA0CTL1 &= ~UCSWRST;                       // **Initialize USCI state machine**
    UCA0IE |= UCRXIE;                           // Enable USCI_A0 RX interrupt

    __bis_SR_register(GIE);                     // Interrupts enabled
    while (1);
}

#pragma vector = TIMER0_A0_VECTOR               // Detects interrupt for CCR0 on Timer0
__interrupt void Timer_A00(void)
{
    if (TA0CCR1)                                // When the CCR1 is not 0
        P1OUT &= ~BIT2;                         // Turns the Red LED on
    if (TA0CCR2)                                // When the CCR2 is not 0
        P1OUT &= ~BIT3;                         // Turns the Green LED on
    if (TA0CCR3)                                // When the CCR3 is not 0
        P1OUT &= ~BIT4;                         // Turns the Blue LED on
}

#pragma vector = TIMER0_A1_VECTOR               // Detects interrupt for CCR1-3 on Timer0
__interrupt void Timer_A01(void)
{
    switch (TA0IV)                              // Checks interrupt vector against different cases
    {
    case TA0IV_TACCR1:                          // Checks the interrupt vector to determine if CCR1 was triggered
        P1OUT |= BIT2;                          // Turns the Red LED off
        break;
    case TA0IV_TACCR2:                          // Checks the interrupt vector to determine if CCR2 was triggered
        P1OUT |= BIT3;                          // Turns the Green LED off
        break;
    case TA0IV_TACCR3:                          // Checks the interrupt vector to determine if CCR3 was triggered
        P1OUT |= BIT4;                          // Turns the Blue LED off
        break;
    default:
        break;
    }
}

#pragma vector=USCI_A0_VECTOR                   // Detects interrupt for UART
__interrupt void USCI_A0_ISR(void)
{

    switch (__even_in_range(UCA0IV, 4))
    {
    case 0:                                     // Vector 0 - no interrupt
        break;
    case 2:                                     // Vector 2 - RXIFG
        while (!(UCA0IFG & UCTXIFG));           // USCI_A1 TX buffer ready?
            switch (state)
            {
            case 1:                             // Length state
                i = UCA0RXBUF;                  // Store given length
                if(i >= 5 & i != 0x0D)          // Check if packet is long enough and not the end character
                    state = 2;                  // Change to Red LED state
                i--;                            // Decrement length
                UCA0TXBUF = UCA0RXBUF - 3;      // Send length minus 3
                break;
            case 2:                             // Red LED
                TA0CCR1 = UCA0RXBUF;            // Store red byte in CCR1
                i--;                            // Decrement length
                state = 3;                      // Change to Green LED state
                break;
            case 3:                             // Green LED
                TA0CCR2 = UCA0RXBUF;            // Store green byte in CCR2
                i--;                            // Decrement length
                state = 4;                      // Change to Blue LED state
                break;
            case 4:                             // Blue LED
                TA0CCR3 = UCA0RXBUF;            // Store blue byte in CCR3
                i--;                            // Decrement length
                state = 5;                      // Change to Transmit state
                break;
            case 5:                             // Transmit
                if (i == 1)                     // Check if end of packet
                {
                    UCA0TXBUF = 0x0D;           // Send end character
                    state = 1;                  // Return to Length state
                }
                else                            // Not end of packet
                {
                    UCA0TXBUF = UCA0RXBUF;      // Transmit remaining RGB bytes
                    i--;                        // Decrement length
                }
                break;
            default:
                break;
            }
        break;
    case 4:
        break;                                  // Vector 4 - TXIFG
    default:
        break;
    }
}
