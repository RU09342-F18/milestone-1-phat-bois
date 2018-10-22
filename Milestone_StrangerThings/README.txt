Milestone 1: Communicating with Will Byers
Alex Marino and Cameron Bendzynski
Embedded Systems Section 3
Due by October 18, 2018
Created October 14, 2018
Version 1.0 - Lasted updated October 17, 2018
Supported Boards: MSP430F5529
###############################################
Included files:
main.c
README.txt
Also required: msp430.h
###############################################
The Stranger Things program is designed to create a string of different-colored 
lights, similar to the Christmas lights used to communicate with Will Byers in the 
popular Netflix series Stranger Things. For the program to work as designed, the 
development boards would each need to be connected to an RGB LED, and to each other. 
The head node of the chain would receive a large packet containing color information, 
and would have to set its LED to the specified combination of colors, and then send the 
remaining color information as a packet to the next board. This was achieved by 
communicating between boards over UART, and using pulse width modulation to control 
the red, green, and blue pins of the LED. For more information, refer to the Application Note. 
