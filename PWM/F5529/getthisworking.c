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
unsigned char state = 1;
unsigned char i;

void LEDSetup()                              // All initial settings for LED use
{
    P1DIR |= BIT2;                                 // Set P6.0 to output - R LED
    P1DIR |= BIT3;                                 // Set P6.1 to output - G LED
    P1DIR |= BIT4;                                 // Set P6.2 to output - B LED
}

void TimerA0Setup()                          // All initial settings for TimerA0
{
    TA0CTL = TASSEL_2 + MC_1 + TAIE + ID_2; // Set Timer A0 in Up Mode , counter clear, and interrupt enabled
    TA0CCTL0 |= CCIE;                   // Capture/Compare enable on Timer1 CCR1
    TA0CCTL1 |= CCIE;                   // Capture/Compare enable on Timer1 CCR0
    TA0CCTL2 |= CCIE;
    TA0CCTL3 |= CCIE;
    TA0CCR0 = 256;                        // Set Capture/Compare register to 255
    TA0CCR1 = 255;
    TA0CCR2 = 0;
    TA0CCR3 = 0;
}
int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
    //    UCSCTL4 = SELA_1;                               // ACLK (10kHz)
    LEDSetup();                                     // Function for LED setup
    TimerA0Setup();                                 // Function for Timer0 setup
    P4SEL |= BIT4 + BIT5;
    //P4DIR |= BIT7;
    P3SEL |= BIT3 + BIT4;                       // P3.3,4 = USCI_A1 TXD/RXD
    UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA1CTL1 |= UCSSEL_2;                     // SMCLK
    UCA1BR0 = 6;                            // 1MHz 9600 (see User's Guide)
    UCA1BR1 = 0;                              // 1MHz 9600
    UCA1MCTL |= UCBRS_0 + UCBRF_13 + UCOS16;   // Modulation UCBRSx=1, UCBRFx=0
    UCA1CTL1 &= ~UCSWRST;                   // **Initialize USCI state machine**
    UCA1IE |= UCRXIE;                         // Enable USCI_A1 RX interrupt
    __bis_SR_register(LPM4_bits + GIE);       // Enter LPM0, interrupts enabled
    while (1);
}

#pragma vector = TIMER0_A0_VECTOR                   // Detects interrupt for CCR0 on Timer1
__interrupt void Timer_A00(void)
{
    if (TA0CCR1)
        P1OUT &= ~BIT2;                              // Turns the LED on
    if (TA0CCR2)
        P1OUT &= ~BIT3;
    if (TA0CCR3)
    P1OUT &= ~BIT4;

}


#pragma vector = TIMER0_A1_VECTOR                   // Detects interrupt for CCR1 on Timer1
__interrupt void Timer_A01(void)
{
    switch (TA0IV)
    {
        case TA0IV_TACCR1: // Checks the interrupt vector to determine if CCR1 was triggered
        P1OUT |= BIT2;// Turns the LED off
        break;
        case TA0IV_TACCR2:// Checks the interrupt vector to determine if CCR2 was triggered
        P1OUT |= BIT3;// Turns the LED off
        break;
        case TA0IV_TACCR3:
        P1OUT |= BIT4;
        break;
        default:
        break;
    }
}

#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    switch (__even_in_range(UCA1IV, 4))
    {
    case 0:
        break;                             // Vector 0 - no interrupt
    case 2:                                   // Vector 2 - RXIFG
        while (!(UCA1IFG & UCTXIFG))
            ;             // USCI_A1 TX buffer ready?
        switch (state)
        {
        case 1: //length byte
            i = UCA0RXBUF; //store given length
            i--; //decrement length
            UCA0TXBUF = UCA0RXBUF - 3; //send length minus 3
            state = 2; //change state to 2
            break;
        case 2: //red byte
            TA0CCR0 = UCA0RXBUF; //store red byte in CCR0
            i--;
            state = 3;
            break;
        case 3: //green byte
            TA0CCR1 = UCA0RXBUF; //store green byte in CCR1
            i--;
            state = 4;
            break;
        case 4: //blue byte
            TA0CCR2 = UCA0RXBUF; //store blue byte in CCR2
            i--;
            state = 5;
            break;
        case 5: //blue byte
            if (i == 1)
            { //if
                UCA0TXBUF = 0x0D;
                while (1)
                    ;
            }
            else
            {
                UCA0TXBUF = UCA0RXBUF;
                i--;
            }
            break;
        default:
            break;
        }

        P4OUT ^= BIT7;
        break;
    case 4:
        break;                             // Vector 4 - TXIFG
    default:
        break;
    }
}
